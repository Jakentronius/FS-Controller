"""Hardware and machine-control layer for the friction surfacing dashboard.

Owns the NAU7802 acquisition thread (~80 Hz), Moonraker telemetry polling,
the active Z-force control loop, and the job execution thread.
"""

import collections
import csv
import json
import os
import re
import threading
import time

import requests

try:
    import qwiic_nau7802
    QWIIC_AVAILABLE = True
except ImportError:
    QWIIC_AVAILABLE = False

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(BASE_DIR, "config.json")
LOG_DIR = os.path.join(BASE_DIR, "logs")
JOB_DIR = os.path.join(BASE_DIR, "jobs")
PROFILE_DIR = os.path.join(BASE_DIR, "profiles")
MESH_PATH = os.path.join(BASE_DIR, "mesh.json")      # the active mesh
MESH_DIR = os.path.join(BASE_DIR, "meshes")          # named saved meshes
TOUCHOFF_PATH = os.path.join(BASE_DIR, "touchoff.json")

# Controller settings captured in / restored from a mechtrode profile.
# force_target is intentionally excluded: targets belong to jobs.
PROFILE_KEYS = ("pid_kp", "pid_ki", "pid_kd", "force_deadband",
                "z_step_max", "control_cooldown", "force_avg_samples",
                "force_ramp_rate", "stiffness_curve", "schedule_ref_force",
                "use_gain_schedule", "backlash_comp")

MOONRAKER_URL = "http://127.0.0.1:7125"

# Columns written to every log (manual and per-job run logs). Raw ADC is
# always included so calibration can be redone offline.
LOG_COLUMNS = ["Time_epoch_s", "Raw_ADC", "Force_units", "Force_target",
               "X", "Y", "Z", "Z_offset"]

DEFAULT_CONFIG = {
    # Scale response factor: ADC counts per unit of force (signed).
    "counts_per_unit": 1.0,
    "units_label": "g",
    # Active Z-force control parameters (all force values in calibrated units)
    "force_target": 500.0,
    "force_deadband": 50.0,
    # PID controller: every control_cooldown seconds, Z is nudged by
    # clamp(Kp*err + I + Kd*derr, +-z_step_max) while outside the deadband.
    "pid_kp": 0.0002,
    "pid_ki": 0.0,
    "pid_kd": 0.0,
    "z_step_max": 0.05,
    "force_avg_samples": 4,
    "control_cooldown": 0.1,
    # Drivetrain windup compensation (mm): the first slice of a Z
    # direction reversal is absorbed by the leadscrew/nut/rotor before
    # the carriage moves, so reversing corrections arrive late and
    # undersized. When a correction flips direction, this much extra is
    # pre-added in the new direction. The auto-tune step test measures
    # it from the forward-vs-reversal response asymmetry. 0 disables.
    "backlash_comp": 0.0,
    # Setpoint ramp: on an upward force-target change the controller slews
    # an internal effective target at this rate (units/s) instead of
    # presenting the full error at once — touch-downs load up gradually
    # instead of overshooting by stiffness * approach speed * loop latency.
    # Downward changes apply instantly. 0 disables.
    "force_ramp_rate": 200.0,
    # Abort behavior
    "retract_mm": 5.0,
    "retract_feedrate": 600,
    # Mid-job rod change: pause the job once the force controller has
    # plunged this many mm (rod consumption). 0 disables the check.
    "max_rod_consumption": 0.0,
    "rod_change_retract": 10.0,
    # After a mid-job rod change re-engages, hold this long at force
    # before the job continues (re-induction of the fresh cold rod)
    "rod_change_induction_s": 0.0,
    # Safety limits. max_force: any reading beyond this (either sign)
    # trips a fault -> control off, everything aborted, Z retracted.
    # 0 disables. z_floor: machine Z that no routine or correction may
    # descend below (set just above the build plate).
    "max_force": 0.0,
    "z_floor": -1000.0,
    # Last tare offset (raw ADC counts), persisted so restarts keep the
    # zero. Written by tare()/clear_tare(), not meant for hand-editing.
    "tare_offset": 0.0,
    # Park position for the one-click Park button (Z raises first)
    "park_x": 0.0,
    "park_y": 0.0,
    "park_z": 45.0,
    # Travel feeds for manual positioning (Jog / Go To / Park), mm/min.
    # Klipper still clamps to printer.cfg max_velocity / max_z_velocity,
    # so these can be set high and the firmware limit wins.
    "travel_feedrate": 3000.0,
    "z_travel_feedrate": 900.0,
    # Surface-mesh feedforward: 1 = follow the probed mesh during force
    # control (the force PID stays primary and corrects the residual)
    "mesh_enabled": 1.0,
    # Gain schedule from a sweep auto-tune: stiffness_curve is a sorted
    # [[force, units_per_mm], ...] table; the PID gains scale by
    # S(ref)/S(target) each cycle when enabled.
    "stiffness_curve": [],
    "schedule_ref_force": 0.0,
    "use_gain_schedule": 0.0,
}

# Auto-tune probe routine: per-run parameters (not persisted) and the hard
# clamps applied to anything the UI sends.
AUTOTUNE_DEFAULTS = {
    "sweep": 0.0,           # 1 = fit S(F) bands + gain schedule + 3-point hold
    "probe_force": 300.0,   # characterization force target (calibrated units)
    "max_travel": 15.0,     # max approach descent before giving up (mm)
    "approach_step": 0.05,  # coarse approach step (mm)
    "fine_step": 0.01,      # characterization step (mm)
    "feedrate": 120.0,      # Z feed for all probing moves (mm/min)
    "settle_s": 0.25,       # settle time after each step before reading (s)
    "hold_s": 8.0,          # closed-loop verification duration (s)
}
AUTOTUNE_LIMITS = {
    "sweep": (0.0, 1.0),
    "probe_force": (10.0, 100000.0),
    "max_travel": (1.0, 40.0),
    "approach_step": (0.01, 0.2),
    "fine_step": (0.002, 0.05),
    "feedrate": (30.0, 600.0),
    "settle_s": (0.05, 2.0),
    "hold_s": (2.0, 60.0),
}


class _AutotuneAbort(Exception):
    """Raised inside the auto-tune worker when the user aborts."""


# Surface-mesh probing: per-run parameters and clamps.
MESH_DEFAULTS = {
    "size_mm": 100.0,        # side length of the probed square (mm)
    "points_per_side": 5,    # N x N grid (5x5 catches local height bumps)
    # Rectangular grids: 0 = inherit the square values above. A dense
    # X-heavy grid (e.g. 21 x 2 over 100 x 20 mm) captures short-
    # wavelength height waviness along the pass direction that a coarse
    # square grid aliases right past.
    "points_x": 0,
    "points_y": 0,
    "size_x_mm": 0.0,
    "size_y_mm": 0.0,
    "max_travel": 15.0,      # max descent below the start height (mm)
    "clearance": 2.0,        # travel height above the highest contact (mm)
    "threshold": 10.0,       # contact force floor (units; 6*noise if bigger)
    "feedrate": 120.0,       # Z probing feed (mm/min)
    "xy_feedrate": 1200.0,   # XY travel feed (mm/min)
    "approach_step": 0.05,   # coarse probe step (mm)
    "fine_step": 0.01,       # refine step (mm)
    "settle_s": 0.25,        # settle before each reading (s)
}
MESH_LIMITS = {
    "size_mm": (10.0, 300.0),
    "points_per_side": (2, 7),
    "points_x": (0, 41),
    "points_y": (0, 41),
    "size_x_mm": (0.0, 300.0),
    "size_y_mm": (0.0, 300.0),
    "max_travel": (1.0, 40.0),
    "clearance": (0.5, 10.0),
    "threshold": (1.0, 100000.0),
    "feedrate": (30.0, 600.0),
    "xy_feedrate": (100.0, 3000.0),
    "approach_step": (0.01, 0.2),
    "fine_step": (0.002, 0.05),
    "settle_s": (0.05, 2.0),
}


# Single-point touch-off: rough tool length / starting height after a
# mechtrode swap or wear.
TOUCHOFF_DEFAULTS = {
    "hover_mm": 2.0,        # park height above the surface afterwards (mm)
    "max_travel": 20.0,     # max descent below the start height (mm)
    "threshold": 10.0,      # contact force floor (units; 6*noise if bigger)
    "feedrate": 120.0,
    "approach_step": 0.05,
    "fine_step": 0.01,
    "settle_s": 0.25,
}
TOUCHOFF_LIMITS = {
    "hover_mm": (0.5, 20.0),
    "max_travel": (1.0, 45.0),
    "threshold": (1.0, 100000.0),
    "feedrate": (30.0, 600.0),
    "approach_step": (0.01, 0.2),
    "fine_step": (0.002, 0.05),
    "settle_s": (0.05, 2.0),
}


def load_touchoff():
    try:
        with open(TOUCHOFF_PATH) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def load_mesh():
    try:
        with open(MESH_PATH) as f:
            m = json.load(f)
        if m.get("xs") and m.get("ys") and m.get("z"):
            return m
    except (OSError, ValueError):
        pass
    return None


def save_mesh(mesh):
    with open(MESH_PATH, "w") as f:
        json.dump(mesh, f, indent=2)


def _mesh_interp(mesh, x, y):
    """Bilinear interpolation of the mesh surface Z at (x, y); positions
    outside the grid clamp to the nearest edge."""
    xs, ys, zg = mesh["xs"], mesh["ys"], mesh["z"]
    x = min(max(x, xs[0]), xs[-1])
    y = min(max(y, ys[0]), ys[-1])
    ix = next(i for i in range(len(xs) - 1) if x <= xs[i + 1])
    iy = next(i for i in range(len(ys) - 1) if y <= ys[i + 1])
    tx = (x - xs[ix]) / (xs[ix + 1] - xs[ix])
    ty = (y - ys[iy]) / (ys[iy + 1] - ys[iy])
    return (zg[iy][ix] * (1 - tx) * (1 - ty)
            + zg[iy][ix + 1] * tx * (1 - ty)
            + zg[iy + 1][ix] * (1 - tx) * ty
            + zg[iy + 1][ix + 1] * tx * ty)


def _median(vals):
    s = sorted(vals)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def _fit_slope(points):
    """Least-squares slope of force vs depth for [[depth, force], ...]."""
    n = len(points)
    sx = sum(p[0] for p in points)
    sy = sum(p[1] for p in points)
    sxx = sum(p[0] * p[0] for p in points)
    sxy = sum(p[0] * p[1] for p in points)
    denom = n * sxx - sx * sx
    if denom == 0:
        return None
    return (n * sxy - sx * sy) / denom


def _interp_curve(curve, f):
    """Linear interpolation of a sorted [[force, value], ...] table,
    clamped at both ends. Returns None on an empty table."""
    if not curve:
        return None
    if f <= curve[0][0]:
        return curve[0][1]
    if f >= curve[-1][0]:
        return curve[-1][1]
    for (f0, v0), (f1, v1) in zip(curve, curve[1:]):
        if f0 <= f <= f1 and f1 > f0:
            t = (f - f0) / (f1 - f0)
            return v0 + t * (v1 - v0)
    return curve[-1][1]


def _fit_stiffness_bands(load_pts, fmin, fmax, bands=4):
    """Fit a local stiffness in equal-force bands of the loading curve.
    Returns a sorted [[band_mid_force, slope], ...] table (bands with too
    few points or non-positive slope are skipped)."""
    curve = []
    span = fmax - fmin
    if span <= 0:
        return curve
    for b in range(bands):
        lo = fmin + span * b / bands
        hi = fmin + span * (b + 1) / bands
        pts = [q for q in load_pts if lo <= q[1] <= hi]
        if len(pts) < 3:
            continue
        slope = _fit_slope(pts)
        if slope and slope > 0:
            curve.append([round((lo + hi) / 2, 1), round(slope, 1)])
    return curve


def _cross_depth(points, level, rising):
    """Depth at which force crosses `level` (linear interpolation between
    consecutive samples). Points are in acquisition order."""
    for (d0, f0), (d1, f1) in zip(points, points[1:]):
        lo, hi = (f0, f1) if rising else (f1, f0)
        if lo <= level <= hi and f1 != f0:
            return d0 + (level - f0) * (d1 - d0) / (f1 - f0)
    return None


def load_config():
    cfg = dict(DEFAULT_CONFIG)
    try:
        with open(CONFIG_PATH) as f:
            saved = json.load(f)
        cfg.update({k: v for k, v in saved.items() if k in DEFAULT_CONFIG})
    except (OSError, ValueError):
        pass
    return cfg


def save_config(cfg):
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)


# Accepted header aliases for job CSV columns
JOB_COLUMNS = {
    "x": ["x"],
    "y": ["y"],
    "force": ["target_force_units", "target_force", "force"],
    "feedrate": ["feedrate", "f", "feed"],
}

JOB_EXTENSIONS = (".csv", ".gcode", ".nc", ".g")

# Job directives are G-code comments interpreted by the job runner (never
# sent to Klipper):
#   ;FORCE=<units>              set the Z-force control target
#   ;WAIT_FORCE [TIMEOUT=<s>]   block until force is within deadband of target
#   ;DWELL=<s>                  hold for N seconds while force control runs
_RE_FORCE = re.compile(r"^force\s*=\s*(-?\d+(?:\.\d+)?)$", re.I)
_RE_DWELL = re.compile(r"^dwell\s*=\s*(\d+(?:\.\d+)?)$", re.I)
_RE_WAIT = re.compile(r"^wait_force(?:\s+timeout\s*=\s*(\d+(?:\.\d+)?))?$", re.I)
_RE_ZREF = re.compile(r"^zref\s*=\s*surface$", re.I)

WAIT_FORCE_DEFAULT_TIMEOUT = 60.0

# Guarded engagement (WAIT_FORCE from free air): commanded creep toward
# the surface until first contact, so the approach gap is closed by
# commanded motion instead of the force controller's persistent offset.
ENGAGE_STEP_MM = 0.03      # per-step force jump = stiffness * step
ENGAGE_FEED = 120.0        # mm/min, same creep rate as auto-tune probing
ENGAGE_MAX_MM = 3.0        # give up: park height must be nearer than this

# XY draw moves are sliced into sub-moves of about this duration.
# Klipper throttles gcode input once buffered motion runs ~2 s ahead of
# real time, so a single long G1 blocks SET_GCODE_OFFSET force
# corrections for its entire duration; short paced slices keep the
# buffer shallow so corrections land within ~2 slice times (~0.2 s —
# about the control loop's own cadence, which is the useful floor: a
# correction always queues behind >= 1 buffered slice, and the sensor
# average + eval period add ~0.15 s regardless). The distance floor must
# stay tiny: a 0.5 mm floor at F10 made 3 s slices, the buffer ran ~6 s
# ahead, and Klipper's throttled input starved every force correction
# for entire traverses (force sagged out of band with a frozen Z offset).
SLICE_TIME_S = 0.15
SLICE_MIN_MM = 0.01


def _parse_directive(line):
    """Return a directive step dict for a comment line, or None if it is an
    ordinary comment."""
    body = line.lstrip(";").strip()
    m = _RE_FORCE.match(body)
    if m:
        return {"type": "force", "value": float(m.group(1)),
                "desc": f"set force target {m.group(1)}"}
    m = _RE_DWELL.match(body)
    if m:
        return {"type": "dwell", "seconds": float(m.group(1)),
                "desc": f"dwell {m.group(1)}s at force"}
    m = _RE_WAIT.match(body)
    if m:
        timeout = float(m.group(1)) if m.group(1) else WAIT_FORCE_DEFAULT_TIMEOUT
        return {"type": "wait_force", "timeout": timeout,
                "desc": f"wait for force (timeout {timeout:.0f}s)"}
    if _RE_ZREF.match(body):
        # All job Z coordinates are relative to the last touched-off
        # surface (Z0 = surface). Requires a touch-off before running.
        return {"type": "zref", "mode": "surface",
                "desc": "surface-referenced Z frame"}
    return None


def _move_step(lines):
    return {"type": "move", "lines": lines,
            "desc": f"move ({len(lines)} line{'s' if len(lines) != 1 else ''})"}


def _parse_csv_job(path):
    """Parse a CSV toolpath (X, Y, Target_Force_Units, Feedrate) into
    steps, a force step + move step per row."""
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("File is empty")
        headers = {h.strip().lower(): h for h in reader.fieldnames}

        colmap = {}
        for key, aliases in JOB_COLUMNS.items():
            for alias in aliases:
                if alias in headers:
                    colmap[key] = headers[alias]
                    break
            else:
                raise ValueError(
                    f"Missing column '{aliases[0]}' (found: {reader.fieldnames})")

        steps = []
        for i, row in enumerate(reader, start=2):
            if not any((v or "").strip() for v in row.values()):
                continue
            try:
                x = float(row[colmap["x"]])
                y = float(row[colmap["y"]])
                force = float(row[colmap["force"]])
                feed = float(row[colmap["feedrate"]])
            except (TypeError, ValueError):
                raise ValueError(f"Bad numeric value on line {i}: {row}")
            steps.append({"type": "force", "value": force,
                          "desc": f"set force target {force:g}"})
            steps.append(_move_step([f"G1 X{x:.3f} Y{y:.3f} F{feed:.0f}"]))

    if not steps:
        raise ValueError("No data rows found")
    return steps


def _parse_gcode_job(path):
    """Parse an annotated G-code job into a list of steps.

    Standard G-code plus job directives (;FORCE=, ;WAIT_FORCE, ;DWELL=).
    Consecutive G-code lines are grouped into a single move step so Klipper
    can motion-plan across them smoothly; each directive flushes the pending
    move block and becomes its own runner-side step.
    """
    steps = []
    pending = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith(";"):
                directive = _parse_directive(line)
                if directive:
                    if pending:
                        steps.append(_move_step(pending))
                        pending = []
                    steps.append(directive)
                continue
            # Strip trailing comments
            line = line.split(";", 1)[0].strip()
            if line:
                pending.append(line)
    if pending:
        steps.append(_move_step(pending))
    if not any(s["type"] == "move" for s in steps):
        raise ValueError("No G-code commands found")
    return steps


def parse_job_file(path):
    """Parse a job file (.csv or .gcode/.nc/.g) into a list of step dicts
    (type: move | force | wait_force | dwell).

    Raises ValueError with a human-readable message on bad input.
    """
    if path.lower().endswith(".csv"):
        return _parse_csv_job(path)
    return _parse_gcode_job(path)


_RE_LINEAR = re.compile(r"^G[01](?:\s|$)", re.I)
_RE_ARC = re.compile(r"^G[23](?:\s|$)", re.I)
_RE_G4 = re.compile(r"^G4(?:\s|$|P)", re.I)
_RE_WORD = re.compile(r"([XYZF])(-?\d+(?:\.\d+)?)", re.I)


def validate_job(steps, limits=None, config=None, mesh=None,
                 xy_offset=(0.0, 0.0), z_offset=0.0):
    """Static preflight of a parsed job: machine-volume bounds, Z floor,
    feedrates vs printer limits, force targets vs max force, mesh
    coverage, and constructs that fight the force controller. Returns
    {errors, warnings, bbox, max_feed}; errors should block the run.

    Coordinates are commanded (offset-frame) values — the force
    controller's Z offset shifts the machine frame during a run, which
    this cannot predict; bounds are checked as authored.
    """
    config = config or {}
    errors, warnings = [], []
    absolute = True
    pos = {"x": None, "y": None, "z": None}
    lo, hi = {}, {}
    unknown_rel = False
    max_feed = 0.0
    force_active = 0.0

    def track(axis, value):
        lo[axis] = min(lo.get(axis, value), value)
        hi[axis] = max(hi.get(axis, value), value)

    for si, step in enumerate(steps, start=1):
        if step["type"] == "force":
            force_active = step["value"]
            mf = float(config.get("max_force", 0) or 0)
            if mf > 0 and abs(step["value"]) > mf:
                errors.append(f"step {si}: force target {step['value']:g} "
                              f"exceeds the max force limit {mf:g}")
            continue
        if step["type"] != "move":
            continue

        block_len = 0.0
        for line in step["lines"]:
            u = line.upper()
            if u.startswith("G90"):
                absolute = True
                continue
            if u.startswith("G91"):
                absolute = False
                continue
            if u.startswith("G28"):
                warnings.append(
                    f"step {si}: G28 (homing) inside a job — position "
                    "tracking resets and the force-control offset frame "
                    "is disturbed")
                pos = {"x": None, "y": None, "z": None}
                continue
            if _RE_G4.match(u):
                warnings.append(f"step {si}: G4 blocks force corrections "
                                "— use ;DWELL= instead")
                continue
            if u.startswith("M204"):
                m = re.search(r"[SPT](\d+(?:\.\d+)?)", u[4:])
                if (m and limits and limits.get("max_accel")
                        and float(m.group(1)) > limits["max_accel"]):
                    warnings.append(
                        f"step {si}: M204 {m.group(1)} mm/s² is above the "
                        f"printer's max accel {limits['max_accel']:g} — "
                        "Klipper will cap it")
                continue
            if u.startswith("SET_GCODE_OFFSET"):
                warnings.append(
                    f"step {si}: SET_GCODE_OFFSET conflicts with the "
                    "force controller's Z offset")
                continue
            if _RE_ARC.match(u):
                warnings.append(f"step {si}: arc move (G2/G3) — only the "
                                "arc's endpoint is bounds-checked")
            elif not _RE_LINEAR.match(u):
                continue

            prev = dict(pos)
            for axis, val in _RE_WORD.findall(u):
                axis, val = axis.lower(), float(val)
                if axis == "f":
                    if val <= 0:
                        errors.append(
                            f"step {si}: feedrate F{val:g} is invalid")
                    elif val < 10.0:
                        warnings.append(
                            f"step {si}: feedrate F{val:g} mm/min looks "
                            "like a typo — this segment will take minutes "
                            "and starve force control")
                    max_feed = max(max_feed, val)
                    continue
                if absolute:
                    pos[axis] = val
                elif pos[axis] is not None:
                    pos[axis] += val
                else:
                    unknown_rel = True
                if pos[axis] is not None:
                    track(axis, pos[axis])
            if all(pos[a] is not None and prev[a] is not None
                   for a in ("x", "y")):
                block_len += ((pos["x"] - prev["x"]) ** 2
                              + (pos["y"] - prev["y"]) ** 2) ** 0.5

        # (Long passes at force are no longer flagged: the runner slices
        # XY moves into ~SLICE_TIME_S sub-moves, so force corrections,
        # abort, pause and rod changes all act mid-pass.)

    # Machine-frame checks: authored X/Y shift by the active work origin,
    # authored Z by the surface reference (when the job declares one)
    off = {"x": xy_offset[0], "y": xy_offset[1], "z": z_offset}
    origin_note = (" (with the work origin applied)"
                   if xy_offset[0] or xy_offset[1] else "")
    if limits:
        amin, amax = limits["min"], limits["max"]
        for i, axis in enumerate("xyz"):
            if axis in lo and lo[axis] + off[axis] < amin[i] - 1e-6:
                errors.append(f"{axis.upper()} goes to "
                              f"{lo[axis] + off[axis]:g} mm{origin_note}, "
                              f"below the machine minimum {amin[i]:g}")
            if axis in hi and hi[axis] + off[axis] > amax[i] + 1e-6:
                errors.append(f"{axis.upper()} goes to "
                              f"{hi[axis] + off[axis]:g} mm{origin_note}, "
                              f"above the machine maximum {amax[i]:g}")
        vmax = limits.get("max_velocity")
        if vmax and max_feed / 60.0 > vmax + 1e-6:
            warnings.append(
                f"max feedrate {max_feed:g} mm/min is above the printer "
                f"limit {vmax * 60:g} mm/min — Klipper will cap it")
    else:
        warnings.append("Klipper not ready — machine travel and velocity "
                        "limits were NOT verified")

    z_floor = float(config.get("z_floor", -1000.0) or -1000.0)
    if "z" in lo and lo["z"] + off["z"] < z_floor:
        errors.append(f"Z goes to {lo['z'] + off['z']:g} mm{origin_note}, "
                      f"below the Z floor {z_floor:g} (Calibration tab)")

    if mesh and ("x" in lo or "y" in lo):
        xs, ys = mesh["xs"], mesh["ys"]
        outside = (("x" in lo and (lo["x"] + off["x"] < xs[0]
                                   or hi["x"] + off["x"] > xs[-1]))
                   or ("y" in lo and (lo["y"] + off["y"] < ys[0]
                                      or hi["y"] + off["y"] > ys[-1])))
        if outside:
            warnings.append(
                "toolpath leaves the probed mesh region — the mesh "
                "feedforward clamps to the grid edge out there")

    if unknown_rel:
        warnings.append("relative (G91) moves before any absolute "
                        "position — bounds only partially verified")

    return {
        "errors": errors,
        "warnings": warnings,
        "bbox": {a: [round(lo[a], 3), round(hi[a], 3)] for a in lo},
        "max_feed": max_feed,
    }


class Machine:
    """Global machine state plus all background threads."""

    def __init__(self):
        os.makedirs(LOG_DIR, exist_ok=True)
        os.makedirs(JOB_DIR, exist_ok=True)
        os.makedirs(PROFILE_DIR, exist_ok=True)
        os.makedirs(MESH_DIR, exist_ok=True)

        self.config = load_config()
        self.start_time = time.time()
        self._running = True
        self._session = requests.Session()

        # Sensor state
        self.sensor_ok = False
        self.sensor_error = "not started"
        self.sample_hz = 0.0
        self._last_sample_t = None

        # Safety fault latch (max force exceeded, sensor stall, ...)
        self.fault = None
        self.raw_adc = 0
        # Tare survives app restarts — an in-memory-only tare once made
        # a parked job read the platform's static weight as ~950 g of
        # phantom force and back away from touch-down until timeout.
        self.tare_offset = float(self.config.get("tare_offset", 0.0))
        self.is_tared = self.tare_offset != 0.0
        self.force_history = collections.deque(maxlen=10)

        # CSV logging (manual via the dashboard, or auto per job run)
        self.logging = False
        self.log_path = None
        self._log_lock = threading.Lock()
        self._log_file = None
        self._log_writer = None
        self._log_auto = False   # True when the job runner started the log

        # Klipper telemetry
        self.klipper_state = "unknown"
        self.homed_axes = ""
        self.position = [0.0, 0.0, 0.0]
        self.z_offset = 0.0
        # Work-origin XY offset (SET_GCODE_OFFSET X/Y): job coordinates
        # are commanded in this frame; machine = commanded + offset.
        self.xy_offset = [0.0, 0.0]

        # Active Z-force control
        self.control_enabled = False
        self.force_target = float(self.config["force_target"])
        self._last_adjust = 0.0
        self.pid_state = {}
        # True while a job motion segment holds Klipper's gcode queue:
        # the controller must not stack adjustments behind it (they would
        # execute late as a destructive burst)
        self._move_active = False
        # True while the job streams paced XY slices: corrections are
        # then sent without MOVE=1 and ride the upcoming slices — an
        # interleaved Z-only move zeroes the XY junction speed, which
        # reads as chop at up to the correction rate.
        self._streaming_xy = False

        # Job execution
        self.job = None
        self._job_thread = None
        self._job_abort = threading.Event()
        self._job_confirm = threading.Event()
        # (timestamp, message) of the last failed Moonraker gcode call;
        # consumed by _fail_why() to explain user-facing failures.
        self._gcode_error = (0.0, None)
        self._job_pause_req = False
        # Rod-consumption baseline: captured at the first stable
        # engagement of a job (and re-captured after each rod change) so
        # the approach plunge from park height never counts as wear.
        self._rod_baseline = None

        # Auto-tune probe routine
        self.autotune = {"state": "idle"}
        self._autotune_thread = None
        self._autotune_abort = threading.Event()

        # Surface mesh (persisted across restarts) and its probe routine
        self.mesh = load_mesh()
        self.mesh_probe = {"state": "idle"}
        self._mesh_thread = None
        self._mesh_abort = threading.Event()
        self._mesh_ref = None       # (x, y) where force control engaged
        self._mesh_applied = 0.0    # mesh feedforward already in the offset

        # Single-point touch-off (last result persisted across restarts)
        self.touchoff = {"state": "idle", "last": load_touchoff()}
        self._touchoff_thread = None
        self._touchoff_abort = threading.Event()

    # ------------------------------------------------------------------ #
    # Lifecycle

    def start(self):
        for fn in (self._sensor_loop, self._telemetry_loop,
                   self._control_loop, self._watchdog_loop):
            threading.Thread(target=fn, daemon=True).start()

    def shutdown(self):
        self._running = False
        self.stop_log()

    # ------------------------------------------------------------------ #
    # Calibration / conversion

    def force_units(self, raw=None):
        """Convert a raw ADC reading to calibrated units using the tare offset."""
        raw = self.raw_adc if raw is None else raw
        factor = float(self.config.get("counts_per_unit") or 1.0)
        if factor == 0:
            factor = 1.0
        return (raw - self.tare_offset) / factor

    def force_avg(self):
        """Rolling average over the last force_avg_samples readings, or None
        if not enough samples have arrived yet."""
        n = int(self.config.get("force_avg_samples", 4))
        n = max(1, min(n, self.force_history.maxlen))
        hist = list(self.force_history)
        if len(hist) < n:
            return None
        window = hist[-n:]
        return sum(window) / n

    def update_config(self, updates, persist=True):
        for key in DEFAULT_CONFIG:
            if key in updates:
                if key == "units_label":
                    self.config[key] = str(updates[key])[:10]
                elif key == "stiffness_curve":
                    clean = []
                    for pair in (updates[key] or []):
                        try:
                            f, s = float(pair[0]), float(pair[1])
                        except (TypeError, ValueError, IndexError):
                            continue
                        if s > 0:
                            clean.append([f, s])
                    clean.sort()
                    self.config[key] = clean
                else:
                    self.config[key] = float(updates[key])
        # The configured target is also the active one: tuning the target
        # field takes effect immediately.
        if "force_target" in updates:
            self.force_target = float(updates["force_target"])
        if persist:
            save_config(self.config)

    def tare(self, duration=0.5):
        """Optional display/control zero. Never required for logging."""
        if not self.sensor_ok:
            return False
        samples = []
        deadline = time.time() + duration
        while time.time() < deadline:
            samples.append(self.raw_adc)
            time.sleep(0.0125)
        if samples:
            self.tare_offset = sum(samples) / len(samples)
            self.is_tared = True
            self.config["tare_offset"] = self.tare_offset
            save_config(self.config)
        return True

    def clear_tare(self):
        self.tare_offset = 0.0
        self.config["tare_offset"] = 0.0
        save_config(self.config)
        self.is_tared = False

    # ------------------------------------------------------------------ #
    # Sensor acquisition (~80 Hz)

    def _sensor_loop(self):
        if not QWIIC_AVAILABLE:
            self.sensor_error = "qwiic_nau7802 library not installed"
            return
        scale = qwiic_nau7802.QwiicNAU7802()
        if not scale.is_connected():
            self.sensor_error = "NAU7802 not detected on I2C bus"
            return
        scale.begin()
        scale.set_sample_rate(qwiic_nau7802.QwiicNAU7802.NAU7802_SPS_80)
        self.sensor_ok = True
        self.sensor_error = None

        count = 0
        window_start = time.time()

        while self._running:
            if scale.available():
                raw = scale.get_reading()
                self.raw_adc = raw
                self.force_history.append(self.force_units(raw))
                self._last_sample_t = time.time()
                # Auto-recover the health flag when samples resume after a
                # stall (control stays off — the user re-enables).
                if not self.sensor_ok:
                    self.sensor_ok = True
                    self.sensor_error = None

                count += 1
                now = time.time()
                if now - window_start >= 1.0:
                    self.sample_hz = count / (now - window_start)
                    count = 0
                    window_start = now

                if self.logging:
                    with self._log_lock:
                        if self._log_writer:
                            self._log_writer.writerow([
                                f"{time.time():.4f}",
                                raw,
                                f"{self.force_units(raw):.3f}",
                                f"{self.force_target:g}",
                                f"{self.position[0]:.3f}",
                                f"{self.position[1]:.3f}",
                                f"{self.position[2]:.4f}",
                                f"{self.z_offset:.4f}",
                            ])
            else:
                time.sleep(0.001)

    # ------------------------------------------------------------------ #
    # Raw logging

    def start_log(self, prefix="log", header=None, auto=False):
        """Open a new CSV log. `header` is a list of strings written as
        '# ' comment lines before the column row (human-readable run
        metadata)."""
        if self.logging:
            return self.log_path
        path = os.path.join(
            LOG_DIR, time.strftime(f"{prefix}_%Y%m%d_%H%M%S.csv"))
        with self._log_lock:
            self._log_file = open(path, "w", newline="")
            for line in header or []:
                self._log_file.write(f"# {line}\n")
            self._log_writer = csv.writer(self._log_file)
            self._log_writer.writerow(LOG_COLUMNS)
            self.log_path = path
            self.logging = True
            self._log_auto = auto
        return path

    def stop_log(self):
        with self._log_lock:
            self.logging = False
            self._log_auto = False
            if self._log_file:
                self._log_file.close()
            self._log_file = None
            self._log_writer = None

    def list_logs(self):
        """All CSV logs, newest first, with their parsed '# key: value'
        header metadata."""
        names = [n for n in os.listdir(LOG_DIR) if n.endswith(".csv")]
        names.sort(key=lambda n: os.path.getmtime(os.path.join(LOG_DIR, n)),
                   reverse=True)
        logs = []
        for name in names:
            path = os.path.join(LOG_DIR, name)
            meta = {}
            notes = []
            try:
                with open(path) as f:
                    for _ in range(60):
                        line = f.readline()
                        if not line.startswith("#"):
                            break
                        body = line[1:].strip()
                        if ":" not in body:
                            continue
                        k, v = body.split(":", 1)
                        k, v = k.strip(), v.strip()
                        if k == "note":
                            notes.append(v)
                        else:
                            meta[k] = v
            except OSError:
                continue
            if notes:
                meta["notes"] = "\n".join(notes)
            logs.append({
                "name": name,
                "size": os.path.getsize(path),
                "mtime": os.path.getmtime(path),
                "is_run": name.startswith("run_"),
                "active": self.logging and self.log_path == path,
                "meta": meta,
            })
        return logs

    def delete_log(self, name):
        name = os.path.basename(name)
        path = os.path.join(LOG_DIR, name)
        if self.logging and self.log_path == path:
            return False, "That log is currently recording"
        try:
            os.remove(path)
        except OSError:
            return False, f"Log '{name}' not found"
        return True, name

    # ------------------------------------------------------------------ #
    # Safety watchdog

    def _trip_fault(self, reason):
        """Latch a safety fault: kill force control, abort every routine
        and the job, retract clear of the work. Stays latched until the
        user clears it."""
        if self.fault:
            return
        # Snapshot what owns the machine BEFORE latching the fault
        # (_exclusive_busy reports the fault itself once set)
        job_running = self.job and self.job.get("state") == "running"
        routine_running = (self.autotune_running() or self.mesh_probing()
                           or self.touchoff_running())
        self.fault = {"reason": reason, "time": time.time()}
        print(f"[FAULT] {reason}", flush=True)   # journalctl trail
        self.control_enabled = False
        self._autotune_abort.set()
        self._mesh_abort.set()
        self._touchoff_abort.set()
        if job_running:
            self._job_abort.set()   # the job thread retracts on abort
        elif not routine_running:
            # Nothing else owns the machine: retract here, off-thread so
            # the watchdog keeps watching.
            retract = float(self.config["retract_mm"])
            feed = float(self.config["retract_feedrate"])
            threading.Thread(
                target=self._gcode,
                args=(f"G91\nG1 Z{retract:.2f} F{feed:.0f}\nG90",),
                kwargs={"timeout": 30.0}, daemon=True).start()

    def clear_fault(self):
        self.fault = None

    def _watchdog_loop(self):
        """Independent safety monitor: trips a fault on force-sensor
        stall (stale samples while force is being trusted) or on any
        reading beyond max_force."""
        over = 0
        while self._running:
            time.sleep(0.02)
            if self.fault:
                continue

            # While a paused job waits for the user (rod swap, resume),
            # they are handling the head: force spikes from chucking a rod
            # are expected and nothing is moving under force control, so
            # both watchdog checks stand down until the pause ends.
            job = self.job
            job_running = job and job.get("state") == "running"
            pause = job.get("pause") if job_running else None
            user_waiting = bool(
                pause and str(pause.get("stage", "")).startswith("waiting"))
            if user_waiting:
                over = 0
                continue

            # Sensor stall: samples stopped arriving. Dangerous whenever
            # anything is acting on force (control loop or a probe).
            trusting = (self.control_enabled or self.autotune_running()
                        or self.mesh_probing() or self.touchoff_running()
                        or job_running)
            if (self.sensor_ok and self._last_sample_t is not None
                    and time.time() - self._last_sample_t > 0.5):
                self.sensor_ok = False
                self.sensor_error = "sensor stalled — no samples for 0.5 s"
                if trusting:
                    self._trip_fault(
                        "Force sensor stopped updating while force was "
                        "being controlled — everything halted")
                continue

            # Force ceiling (either sign: an inverted calibration reads
            # negative when pressing). Needs 3 consecutive over-limit
            # polls (~60 ms) so a single noise spike can't trip it.
            max_force = float(self.config.get("max_force", 0))
            if max_force > 0 and self.sensor_ok:
                if abs(self.force_units()) > max_force:
                    over += 1
                else:
                    over = 0
                if over >= 3:
                    over = 0
                    self._trip_fault(
                        f"Force exceeded the {max_force:g} limit "
                        f"({self.force_units():.0f} measured)")

    def _gain_scale(self, target):
        """Gain-schedule multiplier from the sweep auto-tune's stiffness
        curve: S(ref)/S(target), clamped 0.25-4x. 1.0 when disabled or
        no usable curve."""
        if float(self.config.get("use_gain_schedule", 0)) == 0.0:
            return 1.0
        curve = self.config.get("stiffness_curve") or []
        if len(curve) < 2:
            return 1.0
        s_t = _interp_curve(curve, abs(target))
        ref = float(self.config.get("schedule_ref_force", 0))
        s_r = _interp_curve(curve, ref) if ref > 0 else curve[-1][1]
        if not s_t or not s_r or s_t <= 0 or s_r <= 0:
            return 1.0
        return max(0.25, min(4.0, s_r / s_t))

    def _clamp_z_floor(self, adjust):
        """Clamp a downward Z adjustment so the head cannot descend past
        the configured Z floor."""
        if adjust >= 0:
            return adjust
        floor = float(self.config.get("z_floor", -1000.0))
        allowed_down = max(0.0, self.position[2] - floor)
        return max(adjust, -allowed_down)

    # ------------------------------------------------------------------ #
    # Moonraker helpers

    def _gcode(self, script, timeout=5.0):
        """Send a G-code script. Moonraker blocks until the script completes.
        On failure the underlying reason (Klipper's own error text when
        available) is kept in _gcode_error for _fail_why()."""
        try:
            resp = self._session.post(
                f"{MOONRAKER_URL}/printer/gcode/script",
                json={"script": script}, timeout=timeout)
            if resp.status_code == 200:
                return True
            try:
                msg = resp.json()["error"]["message"]
            except (ValueError, KeyError, TypeError):
                msg = f"Moonraker HTTP {resp.status_code}"
            # Moonraker sometimes wraps Klipper's text in a dict repr;
            # pull out the inner message when it does.
            m = re.search(r"'message':\s*'([^']+)'", msg)
            if m:
                msg = m.group(1)
            self._gcode_error = (time.time(), msg)
            return False
        except requests.Timeout:
            self._gcode_error = (time.time(),
                                 f"no reply from Klipper within {timeout:g} s")
            return False
        except requests.RequestException as e:
            self._gcode_error = (time.time(),
                                 f"Moonraker unreachable ({type(e).__name__})")
            return False

    def _fail_why(self, context):
        """context, plus the actual Klipper/Moonraker error from the most
        recent failed _gcode call when it is fresh — so errors surfaced to
        the user say WHY, not just what was being attempted."""
        ts, msg = self._gcode_error
        if msg and time.time() - ts < 10.0:
            return f"{context}: {msg}"
        return context

    def _exclusive_busy(self):
        """Reason string if a machine-exclusive routine is active, else
        None. Jobs, auto-tune, mesh probing and touch-off never overlap
        with each other or with manual motion."""
        if self.job and self.job.get("state") == "running":
            return "a job is running"
        if self.autotune_running():
            return "auto-tune is running"
        if self.mesh_probing():
            return "mesh probing is running"
        if self.touchoff_running():
            return "touch-off is running"
        if self.fault:
            return f"a safety fault is active ({self.fault['reason']})"
        return None

    def jog(self, axis, dist, feed=None):
        """Relative jog. Refused while any exclusive routine is running."""
        busy = self._exclusive_busy()
        if busy:
            return False, f"Cannot jog while {busy}"
        axis = axis.lower()
        if axis not in ("x", "y", "z"):
            return False, f"Bad axis: {axis}"
        dist = float(dist)
        if abs(dist) > 50:
            return False, "Jog distance limited to 50 mm"
        if (axis == "z" and dist < 0
                and self.position[2] + dist
                < float(self.config.get("z_floor", -1000.0))):
            return False, "Move would cross the Z floor (Calibration tab)"
        if feed is None:
            feed = float(self.config["z_travel_feedrate"] if axis == "z"
                         else self.config["travel_feedrate"])
        script = f"G91\nG1 {axis.upper()}{dist:.3f} F{float(feed):.0f}\nG90"
        if not self._gcode(script, timeout=30.0):
            return False, self._fail_why("Jog failed")
        return True, "ok"

    def goto(self, x=None, y=None, z=None, feed=None):
        """Absolute move to the given coordinates (any subset of X/Y/Z).
        Ordered safely: Z raises before the XY traverse, descends after.
        Refused while any exclusive routine is running."""
        busy = self._exclusive_busy()
        if busy:
            return False, f"Cannot move while {busy}"
        homed = self.homed_axes or ""
        coords = {}
        for axis, val in (("x", x), ("y", y), ("z", z)):
            if val is None or val == "":
                continue
            coords[axis] = float(val)
            if axis not in homed:
                return False, f"{axis.upper()} is not homed"
        if not coords:
            return False, "No coordinates given"
        if ("z" in coords
                and coords["z"] < float(self.config.get("z_floor", -1000.0))):
            return False, "Target is below the Z floor (Calibration tab)"

        xy_feed = (float(feed) if feed
                   else float(self.config["travel_feedrate"]))
        z_feed = (float(feed) if feed
                  else float(self.config["z_travel_feedrate"]))
        # Go-to targets are MACHINE coordinates; convert to the commanded
        # frame so an active work origin doesn't shift them.
        cmd = dict(coords)
        if "x" in cmd:
            cmd["x"] -= self.xy_offset[0]
        if "y" in cmd:
            cmd["y"] -= self.xy_offset[1]
        lines = ["G90"]
        z_target = cmd.get("z")
        z_up = z_target is not None and coords["z"] >= self.position[2]
        if z_target is not None and z_up:
            lines.append(f"G1 Z{z_target:.3f} F{z_feed:.0f}")
        if "x" in cmd or "y" in cmd:
            xy = " ".join(f"{a.upper()}{cmd[a]:.3f}"
                          for a in ("x", "y") if a in cmd)
            lines.append(f"G1 {xy} F{xy_feed:.0f}")
        if z_target is not None and not z_up:
            lines.append(f"G1 Z{z_target:.3f} F{z_feed:.0f}")
        lines.append("M400")
        if not self._gcode("\n".join(lines), timeout=120.0):
            return False, self._fail_why("Move failed")
        return True, "ok"

    def set_xy_zero(self):
        """Make the current head position the work origin: job X0 Y0 maps
        to right here. Implemented as Klipper's gcode XY offset."""
        busy = self._exclusive_busy()
        if busy:
            return False, f"Cannot set XY zero while {busy}"
        homed = self.homed_axes or ""
        if "x" not in homed or "y" not in homed:
            return False, "X and Y must be homed first"
        x, y = self.position[0], self.position[1]
        if not self._gcode(f"SET_GCODE_OFFSET X={x:.3f} Y={y:.3f}",
                           timeout=5.0):
            return False, self._fail_why("Could not reach Klipper")
        self.xy_offset = [x, y]  # telemetry will confirm shortly
        return True, f"Work origin set at machine ({x:.1f}, {y:.1f})"

    def clear_xy_zero(self):
        busy = self._exclusive_busy()
        if busy:
            return False, f"Cannot clear XY zero while {busy}"
        if not self._gcode("SET_GCODE_OFFSET X=0 Y=0", timeout=5.0):
            return False, self._fail_why("Could not reach Klipper")
        self.xy_offset = [0.0, 0.0]
        return True, "Work origin cleared (machine coordinates)"

    def home(self, axes):
        """Home 'all', 'xy', or 'z'. Refused while any exclusive routine
        is running."""
        busy = self._exclusive_busy()
        if busy:
            return False, f"Cannot home while {busy}"
        cmds = {"all": "G28", "xy": "G28 X Y", "z": "G28 Z"}
        cmd = cmds.get(axes.lower())
        if cmd is None:
            return False, f"Bad axes: {axes}"
        if not self._gcode(cmd, timeout=120.0):
            return False, self._fail_why("Homing failed")
        return True, "ok"

    def estop(self):
        self.control_enabled = False
        self._autotune_abort.set()
        self._mesh_abort.set()
        self._touchoff_abort.set()
        # Without this, a job waiting at a rod-change/pause confirmation
        # would keep waiting forever after the firmware halt.
        self._job_abort.set()
        try:
            self._session.post(
                f"{MOONRAKER_URL}/printer/emergency_stop", timeout=2.0)
            return True
        except requests.RequestException:
            return False

    def firmware_restart(self):
        """Klipper FIRMWARE_RESTART (needed after E-STOP / MCU errors)."""
        try:
            resp = self._session.post(
                f"{MOONRAKER_URL}/printer/firmware_restart", timeout=5.0)
            return resp.status_code == 200
        except requests.RequestException:
            return False

    def _machine_limits(self):
        """Travel/velocity/accel limits from Klipper, or None when it is
        not ready to report them."""
        try:
            resp = self._session.get(
                f"{MOONRAKER_URL}/printer/objects/query"
                "?toolhead=axis_minimum,axis_maximum,max_velocity,max_accel",
                timeout=2.0)
            if resp.status_code == 200:
                th = resp.json()["result"]["status"]["toolhead"]
                return {"min": th["axis_minimum"][:3],
                        "max": th["axis_maximum"][:3],
                        "max_velocity": th.get("max_velocity"),
                        "max_accel": th.get("max_accel")}
        except (requests.RequestException, KeyError, ValueError):
            pass
        return None

    def _surface_ref(self, steps):
        """(is_surface_referenced, surface_z or None) for a parsed job."""
        if not any(s["type"] == "zref" and s.get("mode") == "surface"
                   for s in steps):
            return False, None
        last = self.touchoff.get("last") or {}
        return True, last.get("surface_z")

    def check_job(self, steps, limits=None):
        mesh = (self.mesh
                if float(self.config.get("mesh_enabled", 1.0)) != 0.0
                else None)
        surface_ref, surface_z = self._surface_ref(steps)
        report = validate_job(steps,
                              limits=limits or self._machine_limits(),
                              config=self.config, mesh=mesh,
                              xy_offset=tuple(self.xy_offset),
                              z_offset=surface_z or 0.0)
        if surface_ref and surface_z is None:
            report["errors"].append(
                "job is surface-referenced (;ZREF=SURFACE) but no "
                "touch-off has been done — run Touch Off first")
        elif surface_ref:
            report["warnings"].insert(0,
                f"surface-referenced: Z0 = touched-off surface at machine "
                f"Z{surface_z:g}")
        return report

    def _telemetry_loop(self):
        while self._running:
            try:
                resp = self._session.get(
                    f"{MOONRAKER_URL}/printer/objects/query"
                    "?toolhead=position,homed_axes&print_stats=state"
                    "&gcode_move=homing_origin&webhooks=state"
                    "&motion_report=live_position",
                    timeout=1.0)
                if resp.status_code == 200:
                    status = resp.json()["result"]["status"]
                    # webhooks.state is the host's real health (error /
                    # startup / shutdown); print_stats.state can read
                    # "standby" even when the MCU is unreachable.
                    host = status.get("webhooks", {}).get("state", "ready")
                    self.klipper_state = (status["print_stats"]["state"]
                                          if host == "ready" else host)
                    self.homed_axes = status["toolhead"]["homed_axes"]
                    # motion_report.live_position is where the head
                    # physically is right now; toolhead.position is the
                    # planner's committed endpoint, which runs a whole
                    # move ahead of the machine — using it for arrival
                    # polling / mesh lookup acts on the future, not the
                    # present.
                    live = status.get("motion_report",
                                      {}).get("live_position")
                    self.position = (list(live[:3]) if live
                                     else status["toolhead"]["position"][:3])
                    origin = status["gcode_move"]["homing_origin"]
                    self.z_offset = origin[2]
                    self.xy_offset = [origin[0], origin[1]]
                else:
                    self.klipper_state = "offline"
            except (requests.RequestException, KeyError, ValueError):
                self.klipper_state = "offline"
            time.sleep(0.05)

    # ------------------------------------------------------------------ #
    # Active Z-force control

    def _control_loop(self):
        """PID Z-force controller.

        Evaluates every control_cooldown seconds while enabled:
        error = target - averaged force. Outside the deadband the Z gcode
        offset is nudged by clamp(Kp*err + I + Kd*derr, +-z_step_max);
        positive error (too little force) plunges (negative Z offset).

        Derivative acts on the measurement (no setpoint kick on target
        steps). The integrator only accumulates near the target (within
        4x deadband, or while unwinding), is clamped to a quarter of
        z_step_max, bled while inside the deadband, and reset whenever
        control is (re-)enabled.
        """
        integ = 0.0
        prev_avg = None
        active = False
        target_eff = 0.0
        prev_dir = 0.0      # sign of the last sent Z correction

        while self._running:
            if not (self.control_enabled and self.sensor_ok):
                if active:
                    active = False
                    self.pid_state = {}
                time.sleep(0.1)
                continue

            if self._move_active:
                # A motion segment is in flight holding the gcode queue:
                # sending now would only queue a stale burst behind it.
                time.sleep(0.005)
                continue

            now = time.time()
            cooldown = max(0.02, float(self.config["control_cooldown"]))
            if now - self._last_adjust < cooldown:
                time.sleep(0.005)
                continue

            avg = self.force_avg()
            if avg is None:
                time.sleep(0.005)
                continue

            if not active:
                integ = 0.0
                prev_avg = avg
                active = True
                prev_dir = 0.0      # lash state unknown on (re-)engage
                # Ramp from the force actually on the head right now, so
                # (re-)engaging never starts with a full-target error step.
                target_eff = min(avg, self.force_target)
                # Mesh feedforward is referenced to wherever control
                # engages: zero delta here, tilt-following from here on.
                self._mesh_ref = (self.position[0], self.position[1])
                self._mesh_applied = 0.0

            raw_dt = now - self._last_adjust
            dt = raw_dt if raw_dt < 5 * cooldown else cooldown
            self._last_adjust = now

            # Setpoint ramping: slew the effective target toward upward
            # target changes; decreases (and rate 0) snap immediately so
            # ;FORCE=0 disengages stay instant. Never hold the setpoint
            # below the measured force on the way up — that would command
            # a retract mid-load-up.
            ramp = float(self.config.get("force_ramp_rate", 0.0))
            tgt = self.force_target
            if ramp <= 0 or tgt <= target_eff:
                target_eff = tgt
            else:
                target_eff = max(target_eff, min(avg, tgt))
                target_eff = min(tgt, target_eff + ramp * dt)

            error = target_eff - avg
            deadband = float(self.config["force_deadband"])
            sched = self._gain_scale(self.force_target)
            kp = float(self.config["pid_kp"]) * sched
            ki = float(self.config["pid_ki"]) * sched
            kd = float(self.config["pid_kd"]) * sched
            max_step = float(self.config["z_step_max"])

            # Mesh feedforward (secondary to the force feedback): follow
            # the probed surface tilt as XY moves; the PID corrects the
            # residual. Applied even inside the deadband, capped like any
            # other correction.
            mesh_adj = 0.0
            if (self.mesh
                    and float(self.config.get("mesh_enabled", 1.0)) != 0.0
                    and self._mesh_ref is not None):
                want = (_mesh_interp(self.mesh, self.position[0],
                                     self.position[1])
                        - _mesh_interp(self.mesh, *self._mesh_ref))
                mesh_adj = max(-max_step,
                               min(max_step, want - self._mesh_applied))

            # d(error)/dt with constant target == -d(measurement)/dt
            deriv = -(avg - prev_avg) / dt if dt > 0 else 0.0
            prev_avg = avg

            # While XY slices are streaming, send offset changes without
            # MOVE=1: the offset applies to every slice planned after it,
            # so corrections ride the motion as shallow XYZ ramps instead
            # of Z-only moves that zero the XY junction speed (chop).
            # Stationary (dwell/hold/engage), MOVE=1 realizes the offset
            # immediately; the first stationary correction also realizes
            # any leftover streamed offset, so the frame self-heals.
            move_sfx = "" if self._streaming_xy else " MOVE=1"

            if abs(error) <= deadband:
                integ *= 0.98
                self.pid_state = {"error": error, "p": 0.0, "i": integ,
                                  "d": 0.0, "out": 0.0, "mesh": mesh_adj,
                                  "target_eff": round(target_eff, 1),
                                  "in_deadband": True}
                mesh_adj = self._clamp_z_floor(mesh_adj)
                # timeout rides out brief gcode-input throttling at slice
                # junctions: a dropped correction is worse than a late one
                if abs(mesh_adj) >= 0.0005 and self._gcode(
                        f"SET_GCODE_OFFSET Z_ADJUST={mesh_adj:.4f}"
                        f"{move_sfx}", timeout=2.5):
                    self._mesh_applied += mesh_adj
                    prev_dir = mesh_adj     # mesh moves shift lash too
                continue

            # Conditional integration: the I term exists to erase the
            # deadband-edge sag, so it only accumulates near the target
            # (or when unwinding) and carries a quarter of the step
            # authority. A full-authority integrator winds up during
            # ramps and touch-downs, carries the plunge straight through
            # the setpoint, and hunts (observed as a ~2.5 s force
            # oscillation at contact).
            unwinding = integ != 0.0 and (integ > 0) != (error > 0)
            if abs(error) <= 4.0 * deadband or unwinding:
                integ += ki * error * dt
            i_clamp = 0.25 * max_step
            integ = max(-i_clamp, min(i_clamp, integ))
            p_term = kp * error
            d_term = kd * deriv
            out = max(-max_step, min(max_step, p_term + integ + d_term))

            # Backlash / windup compensation: the drivetrain absorbs the
            # first slice of a direction reversal before the carriage
            # moves, so pre-add the measured windup when the correction
            # flips sign — it arrives full-size instead of late and
            # half-eaten. The extra travel is eaten by the lash, not the
            # workpiece, so it sits outside the z_step_max clamp.
            comp = float(self.config.get("backlash_comp", 0.0))
            step_cmd = -out + mesh_adj
            comp_add = 0.0
            if (comp > 0.0 and abs(step_cmd) >= 0.0005 and prev_dir
                    and (step_cmd > 0) != (prev_dir > 0)):
                comp_add = comp if step_cmd > 0 else -comp
            self.pid_state = {"error": error, "p": p_term, "i": integ,
                              "d": d_term, "out": out, "mesh": mesh_adj,
                              "comp": round(comp_add, 4),
                              "sched": round(sched, 3),
                              "target_eff": round(target_eff, 1),
                              "in_deadband": False}

            adjust = self._clamp_z_floor(step_cmd + comp_add)
            if abs(adjust) >= 0.0005:
                if self._gcode(
                        f"SET_GCODE_OFFSET Z_ADJUST={adjust:.4f}"
                        f"{move_sfx}", timeout=2.5):
                    self._mesh_applied += mesh_adj
                    prev_dir = adjust

    # ------------------------------------------------------------------ #
    # Auto-tune: surface probe + stiffness characterization
    #
    # With a rigid, non-spinning rod in the collet, parked over the
    # substrate: tare in free air, measure sensor noise, descend in small
    # steps until contact, then step in/out in fine_step increments while
    # recording force. The loading-curve slope is the system stiffness
    # (units/mm), the load/unload offset is the mechanical hysteresis
    # (backlash + compliance). While loaded at the probe force it also
    # pulses the Z offset through the controller's own command path and
    # times the force response — the measured dead time + T63 bound the
    # usable feedback bandwidth, so Kp is derived from the actual loop
    # lag instead of a fixed fraction. Recommendations are verified with
    # a short closed-loop hold.

    def autotune_running(self):
        return self.autotune.get("state") == "running"

    def start_autotune(self, params=None):
        busy = self._exclusive_busy()
        if busy:
            return False, f"Cannot auto-tune: {busy}"
        if not self.sensor_ok:
            return False, "Sensor not available"
        if self.klipper_state in ("offline", "unknown", "error",
                                  "startup", "shutdown"):
            return False, "Klipper is not connected"
        if "z" not in (self.homed_axes or ""):
            return False, "Z is not homed"

        p = dict(AUTOTUNE_DEFAULTS)
        for key, (lo, hi) in AUTOTUNE_LIMITS.items():
            if params and key in params:
                try:
                    p[key] = float(params[key])
                except (TypeError, ValueError):
                    return False, f"Bad value for {key}"
            p[key] = max(lo, min(hi, p[key]))

        self.control_enabled = False
        self._autotune_abort.clear()
        self.autotune = {
            "state": "running", "phase": "starting", "message": "",
            "params": p, "started": time.time(), "error": None,
            "points": {"load": [], "unload": []},
            "results": None, "recommended": None,
        }
        self._autotune_thread = threading.Thread(
            target=self._autotune_worker, daemon=True)
        self._autotune_thread.start()
        return True, "Auto-tune started"

    def abort_autotune(self):
        if not self.autotune_running():
            return False
        self.control_enabled = False
        self._autotune_abort.set()
        return True

    def _at_phase(self, phase, message=""):
        self.autotune["phase"] = phase
        self.autotune["message"] = message

    def _at_sample(self, duration):
        """Mean and standard deviation of the calibrated force over a
        sampling window."""
        vals = []
        end = time.time() + duration
        while time.time() < end:
            vals.append(self.force_units())
            time.sleep(0.0125)
        mean = sum(vals) / len(vals)
        std = (sum((v - mean) ** 2 for v in vals) / len(vals)) ** 0.5
        return mean, std

    def _at_move_z(self, dz, feed):
        return self._gcode(
            f"G91\nG1 Z{dz:.4f} F{feed:.0f}\nG90\nM400", timeout=60.0)

    def _probe_contact(self, max_travel, feed, approach_step, fine_step,
                       settle_s, threshold_floor, abort_event):
        """Descend from the current position until contact and return the
        contact Z. Requires: gcode Z offset zeroed (commanded == machine
        coords), head in free air, freshly tared. Coarse approach, back
        off, fine re-touch — same scheme as every probe routine here.
        Raises RuntimeError on failure, _AutotuneAbort on abort."""
        def check_abort():
            if abort_event.is_set():
                raise _AutotuneAbort()

        def zgo(z):
            check_abort()
            if not self._gcode(f"G90\nG1 Z{z:.4f} F{feed:.0f}\nM400",
                               timeout=60.0):
                raise RuntimeError(
                    "Z move rejected (Klipper offline or outside limits)")
            return z

        time.sleep(0.6)  # let telemetry refresh
        start_z = cur_z = self.position[2]
        baseline, noise = self._at_sample(1.5)
        check_abort()
        threshold = max(6.0 * noise, threshold_floor)
        floor_z = max(start_z - max_travel,
                      float(self.config.get("z_floor", -1000.0)))

        while True:
            if cur_z - approach_step < floor_z:
                raise RuntimeError(f"no contact within {max_travel:g} mm")
            cur_z = zgo(cur_z - approach_step)
            time.sleep(settle_s)
            f, _ = self._at_sample(0.15)
            if f - baseline >= threshold:
                break

        backoff = max(2.0 * approach_step, 4.0 * fine_step)
        cur_z = zgo(cur_z + backoff)
        time.sleep(max(0.3, settle_s))
        limit = int(backoff / fine_step * 3) + 10
        misses = 0
        while True:
            cur_z = zgo(cur_z - fine_step)
            time.sleep(settle_s)
            f, _ = self._at_sample(0.15)
            if f - baseline >= threshold:
                return cur_z
            misses += 1
            if misses > limit:
                raise RuntimeError("lost contact during fine re-touch")

    def _at_step_response(self, noise, s_est, fine_step, p, check_abort):
        """Measure the loop's actuation lag while the rod is loaded:
        apply small Z offset steps through the exact command path the
        force controller uses (SET_GCODE_OFFSET ... MOVE=1) and time the
        force response at full sensor rate. The blocking gcode send is
        part of the measured dead time on purpose — the control loop
        pays it too.

        Detection keys off the PEAK response, not the settled value —
        viscoelastic setups (substrate seating, load-cell mounts) can
        relax away most of a step's force within a second, and that
        relaxation is reported separately as relax_frac rather than
        being mistaken for 'no response'.

        Always returns a dict with ok, dz_mm and per-rep diagnostics in
        steps; when ok is true it also carries dead_time_s,
        dead_time_std_s, time_constant_s and relax_frac. The net applied
        offset is restored to zero before returning."""
        # Step size: enough for a ~8-sigma / 5%-of-probe-force response,
        # small enough to stay far inside the 1.5x safety ceiling. The
        # floor must clear the drivetrain lash — a reversal step smaller
        # than the windup is absorbed entirely and reads as 'no
        # response' (3x the configured comp ~= 2x the raw windup, since
        # comp is saved as 0.7x windup).
        comp = float(self.config.get("backlash_comp", 0.0))
        floor = max(fine_step, 0.004, 3.0 * comp)
        df_want = max(8.0 * noise, 0.05 * p["probe_force"])
        dz_cap = max(floor,
                     0.5 * p["probe_force"] / max(s_est, 1e-6))
        dz = min(max(df_want / max(s_est, 1e-6), floor), dz_cap)
        window = 2.0            # response capture per step (s)
        results = []
        steps = []
        net = 0.0
        try:
            for rep in range(4):
                check_abort()
                direction = -1.0 if rep % 2 == 0 else 1.0   # load up first
                time.sleep(max(0.4, p["settle_s"]))
                f0, local_noise = self._at_sample(0.5)
                sigma = max(noise, local_noise)
                check_abort()
                diag = {"dir": "down" if direction < 0 else "up",
                        "f0": round(f0, 1), "sigma": round(sigma, 2)}
                steps.append(diag)
                t0 = time.time()
                if not self._gcode(
                        f"SET_GCODE_OFFSET Z_ADJUST={direction * dz:.4f}"
                        " MOVE=1", timeout=5.0):
                    diag["fail"] = "gcode send failed"
                    continue
                net += direction * dz
                df_exp = -direction * s_est * dz    # down = more force
                sgn = 1.0 if df_exp >= 0 else -1.0
                diag["df_exp"] = round(abs(df_exp), 1)
                trace = []
                while time.time() - t0 < window:
                    trace.append((time.time() - t0, self.force_units()))
                    time.sleep(0.005)
                if len(trace) < 20:
                    diag["fail"] = "trace too short"
                    continue
                resp = [sgn * (f - f0) for _, f in trace]
                # Peak response: mean of the top samples (noise-robust)
                delta_peak = sum(sorted(resp)[-10:]) / 10.0
                tail = [r for (t, _), r in zip(trace, resp)
                        if t > window - 0.5]
                tail_mean = sum(tail) / len(tail) if tail else 0.0
                diag["peak"] = round(delta_peak, 1)
                diag["tail"] = round(tail_mean, 1)
                if delta_peak < max(6.0 * sigma, 0.4 * abs(df_exp)):
                    diag["fail"] = "no clear response"
                    continue
                # Departure: two consecutive samples past the threshold
                # (a single crossing can be a noise spike)
                thresh = max(4.0 * sigma, 0.25 * delta_peak)
                dead = next((trace[i][0] for i in range(len(trace) - 1)
                             if resp[i] >= thresh
                             and resp[i + 1] >= thresh), None)
                t63 = next((trace[i][0] for i in range(len(trace))
                            if resp[i] >= 0.63 * delta_peak), None)
                if dead is None or t63 is None:
                    diag["fail"] = "could not time the rise"
                    continue
                relax = (1.0 - tail_mean / delta_peak
                         if delta_peak > 0 else 0.0)
                diag["dead_s"] = round(dead, 4)
                diag["t63_s"] = round(t63, 4)
                results.append((dead, max(0.0, t63 - dead),
                                max(0.0, min(1.0, relax))))
                self.autotune["message"] = (
                    f"step {rep + 1}/4: dead time {dead * 1e3:.0f} ms, "
                    f"T63 {(t63 - dead) * 1e3:.0f} ms")
        finally:
            # Symmetric steps normally cancel; restore any leftover so
            # the unload curve keeps a clean coordinate frame. (The
            # worker's retract clears the offset regardless.)
            if abs(net) >= 5e-5 and self._gcode(
                    f"SET_GCODE_OFFSET Z_ADJUST={-net:.4f} MOVE=1",
                    timeout=5.0):
                net = 0.0
        if len(results) < 2:
            return {"ok": False, "dz_mm": round(dz, 4), "steps": steps}
        deads = [d for d, _, _ in results]
        mean_d = sum(deads) / len(deads)
        std_d = (sum((d - mean_d) ** 2 for d in deads) / len(deads)) ** 0.5
        # Reversal windup: rep 0 continues the loading direction (no
        # reversal); every later rep reverses. A reversal peak markedly
        # below the forward peak means that slice of the step was eaten
        # by drivetrain lash/windup before the carriage moved.
        windup = ratio = None
        rev_peaks = [s["peak"] for s in steps[1:]
                     if "fail" not in s and s.get("peak")]
        if "fail" not in steps[0] and steps[0].get("peak") and rev_peaks:
            ratio = max(0.0, min(1.0,
                                 _median(rev_peaks) / steps[0]["peak"]))
            if ratio < 0.85:
                windup = round(dz * (1.0 - ratio), 4)
        return {"ok": True, "dz_mm": round(dz, 4), "reps": len(results),
                "steps": steps,
                "dead_time_s": round(_median(deads), 4),
                "dead_time_std_s": round(std_d, 4),
                "time_constant_s": round(
                    _median([x for _, x, _ in results]), 4),
                "relax_frac": round(
                    _median([r for _, _, r in results]), 3),
                "reversal_ratio": (round(ratio, 2)
                                   if ratio is not None else None),
                "windup_mm": windup}

    def _autotune_worker(self):
        at = self.autotune
        p = at["params"]
        feed = p["feedrate"]
        descended = 0.0    # mm below the starting height (positive = down)
        start_z = None
        final_state = "error"

        def check_abort():
            if self._autotune_abort.is_set():
                raise _AutotuneAbort()

        def move_z(dz):
            nonlocal descended
            check_abort()
            if not self._at_move_z(dz, feed):
                raise RuntimeError(
                    "Z move rejected (Klipper offline or outside limits)")
            descended -= dz

        def settle_and_read():
            time.sleep(p["settle_s"])
            mean, _ = self._at_sample(0.15)
            return mean

        try:
            # A leftover force-control offset would corrupt the absolute
            # return move, so start from a clean coordinate frame.
            self._at_phase("setup", "Clearing Z offset, taring in free air")
            self.control_enabled = False
            if not self._gcode("SET_GCODE_OFFSET Z=0", timeout=5.0):
                raise RuntimeError("Could not reach Klipper")
            time.sleep(0.6)  # let telemetry refresh the position
            start_z = self.position[2]
            at["start_z"] = round(start_z, 3)
            p["max_travel"] = min(
                p["max_travel"],
                max(0.0, start_z
                    - float(self.config.get("z_floor", -1000.0))))
            if not self.tare():
                raise RuntimeError("Tare failed (sensor unavailable)")

            self._at_phase("noise", "Measuring baseline sensor noise")
            baseline, noise = self._at_sample(1.5)
            check_abort()
            at["noise_std"] = round(noise, 3)
            if noise > 0.1 * p["probe_force"]:
                raise RuntimeError(
                    f"Sensor noise ({noise:.1f}) is too large relative to "
                    "the probe force — raise the probe force or fix the "
                    "signal before tuning")
            threshold = max(6.0 * noise, 0.02 * p["probe_force"])
            safety = 1.5 * p["probe_force"]

            # ---------------- coarse approach ---------------- #
            self._at_phase("approach", "Descending until contact")
            contact = False
            while descended < p["max_travel"]:
                move_z(-p["approach_step"])
                rel = settle_and_read() - baseline
                at["message"] = (f"{descended:.2f} mm down, "
                                 f"force {rel:+.1f}")
                if rel <= -3.0 * threshold:
                    raise RuntimeError(
                        "Force went negative on contact — the calibration "
                        "sign looks inverted (negate counts-per-unit)")
                if rel >= threshold:
                    contact = True
                    break
            if not contact:
                raise RuntimeError(
                    f"No contact within {p['max_travel']:g} mm — park the "
                    "head closer to the surface or raise max travel")

            # ------------- back off, fine re-approach ------------- #
            self._at_phase("refine", "Backing off, re-approaching finely")
            backoff = max(4.0 * p["fine_step"], 2.0 * p["approach_step"])
            move_z(backoff)
            time.sleep(max(0.4, p["settle_s"]))
            fine_steps = 0
            max_fine = int(backoff / p["fine_step"] * 3) + 10
            while True:
                move_z(-p["fine_step"])
                rel = settle_and_read() - baseline
                fine_steps += 1
                if rel >= threshold:
                    break
                if fine_steps > max_fine:
                    raise RuntimeError(
                        "Could not re-find the surface on fine approach")
            at["contact_z"] = round(start_z - descended, 3)

            # ---------------- loading curve ---------------- #
            self._at_phase("load", "Stepping into the surface")
            contact_depth = descended
            fine = p["fine_step"]
            for load_attempt in (1, 2):
                load_pts = [[round(descended, 4), round(rel, 3)]]
                while True:
                    move_z(-fine)
                    rel = settle_and_read() - baseline
                    load_pts.append([round(descended, 4), round(rel, 3)])
                    at["points"] = {"load": load_pts, "unload": []}
                    at["message"] = (f"depth "
                                     f"{descended - contact_depth:.3f} mm,"
                                     f" force {rel:.1f}")
                    if rel >= p["probe_force"] or rel >= safety:
                        break
                    if descended - contact_depth > 2.0:
                        raise RuntimeError(
                            "Force is not building with depth (compliant "
                            "setup or slipping rod) — stopped for safety")
                if load_attempt == 2 or len(load_pts) >= 10:
                    break
                # Very stiff setup: the whole curve fit in a handful of
                # steps, too coarse for band fits, the deadband and the
                # step test. Back off to contact and redo finer.
                depth = descended - contact_depth
                fine2 = max(0.0025, round(depth / 24.0, 4))
                if fine2 >= 0.8 * fine:
                    break
                self._at_phase(
                    "load", f"Stiff setup — re-probing with "
                    f"{fine2:g} mm steps")
                move_z(descended - contact_depth)
                time.sleep(max(0.4, p["settle_s"]))
                rel = settle_and_read() - baseline
                fine = fine2
            peak = load_pts[-1][1]

            # ------------ loop dead time (step response) ------------ #
            # Measured here, at the probe force, where the response has
            # the most signal over the noise floor.
            self._at_phase("step", "Measuring loop dead time")
            s_quick = _fit_slope(
                [q for q in load_pts if q[1] >= 0.3 * peak])
            if not s_quick or s_quick <= 0:
                s_quick = peak / max(descended - contact_depth, 1e-6)
            step_resp = self._at_step_response(
                noise, s_quick, fine, p, check_abort)

            # ---------------- unloading curve ---------------- #
            self._at_phase("unload", "Stepping back out")
            unload_pts = [load_pts[-1][:]]
            while True:
                move_z(fine)
                rel = settle_and_read() - baseline
                unload_pts.append([round(descended, 4), round(rel, 3)])
                at["points"] = {"load": load_pts, "unload": unload_pts}
                if rel <= threshold or len(unload_pts) > len(load_pts) + 25:
                    break

            # ---------------- analysis ---------------- #
            self._at_phase("analysis", "Fitting stiffness")
            fit_pts = [q for q in load_pts
                       if 0.2 * peak <= q[1] <= 0.95 * peak]
            if len(fit_pts) < 4:
                fit_pts = load_pts
            stiffness = _fit_slope(fit_pts)
            if not stiffness or stiffness <= 0:
                raise RuntimeError(
                    "Could not fit a positive stiffness — data too noisy")

            half = 0.5 * peak
            d_load = _cross_depth(load_pts, half, rising=True)
            d_unload = _cross_depth(unload_pts, half, rising=False)
            hysteresis = (max(0.0, d_load - d_unload)
                          if d_load is not None and d_unload is not None
                          else None)

            # Local stiffness per force band (always computed; drives the
            # gain schedule when sweeping)
            curve = _fit_stiffness_bands(load_pts, threshold, peak)
            sweeping = p["sweep"] >= 0.5 and len(curve) >= 2

            # Recommendations:
            #  Kp: delay-aware. The loop applies Kp*err as a position
            #    nudge every cooldown, which is integral action with
            #    Ki_eff = Kp*S/cooldown (1/s) — the loop-gain crossover.
            #    Holding crossover * total lag <= 0.5 rad keeps ~60 deg
            #    of phase margin, so the fraction of the error corrected
            #    per cycle is alpha = 0.5*cooldown/lag, where lag =
            #    measured dead time + T63 + half a cycle (sampling) +
            #    the force-average window lag. Falls back to the legacy
            #    conservative 30%/cycle if the step test was
            #    inconclusive. Referenced to the STIFFEST band when
            #    sweeping (stable everywhere; the schedule speeds up the
            #    soft end).
            #  deadband: above sensor noise AND the force quantum of the
            #    smallest mechanically-meaningful Z move (hysteresis)
            #  max step: full probe-force error takes >= 8 cycles to slew
            if sweeping:
                ref_force, s_ref = max(curve, key=lambda q: q[1])
            else:
                ref_force, s_ref = p["probe_force"], stiffness
            cooldown = max(0.02, float(self.config["control_cooldown"]))
            if step_resp and step_resp.get("ok"):
                avg_lag = 0.5 * (max(
                    1.0, float(self.config["force_avg_samples"])) - 1.0) \
                    / 80.0      # rolling-average lag at the 80 Hz rate
                lag0 = (step_resp["dead_time_s"]
                        + step_resp["time_constant_s"] + avg_lag)
                # Cadence: evaluating faster than the plant responds
                # just stacks in-flight corrections; ~3 cycles per plant
                # lag also makes each step larger, which punches through
                # stiction better than a stream of micro-nudges.
                cooldown = min(0.5, max(cooldown, round(lag0 / 3.0, 2)))
                lag = lag0 + 0.5 * cooldown
                alpha = min(0.4, max(0.05, 0.5 * cooldown / lag))
                step_resp["corr_frac_per_cycle"] = round(alpha, 3)
            else:
                alpha = 0.3
            kp = alpha / s_ref
            # Integral time ~3 s (30 eval cycles): a P-only controller
            # with a deadband rides at deadband-edge error under steady
            # drift (deposition, surface falling away); a gentle I term
            # erases that sag without fighting the P response. (The loop
            # additionally gates integration to near-target errors and
            # quarter-step authority.)
            ki = kp / 3.0
            deadband = max(5.0 * noise,
                           0.75 * stiffness * (hysteresis or fine),
                           0.01 * p["probe_force"])
            z_step_max = min(0.1, max(0.01,
                                      p["probe_force"] / stiffness / 8.0))
            rec = {"pid_kp": round(kp, 6), "pid_ki": round(ki, 6),
                   "pid_kd": 0.0,
                   "force_deadband": round(deadband, 1),
                   "z_step_max": round(z_step_max, 3),
                   "stiffness_curve": curve if sweeping else [],
                   "schedule_ref_force": round(ref_force, 1) if sweeping
                   else 0.0,
                   "use_gain_schedule": 1.0 if sweeping else 0.0}
            if step_resp and step_resp.get("ok"):
                rec["control_cooldown"] = cooldown
                rec["backlash_comp"] = (
                    round(0.7 * step_resp["windup_mm"], 4)
                    if step_resp.get("windup_mm") else 0.0)

            # ------------- closed-loop hold verification ------------- #
            # Sweep: hold at 25/50/100% of the probe force so the gain
            # schedule is proven across the range, not at one point.
            fractions = (0.25, 0.5, 1.0) if sweeping else (1.0,)
            saved_cfg = {k: self.config.get(k) for k in rec}
            saved_target = self.force_target
            holds = []
            try:
                for attempt in (1, 2):
                    self.update_config(rec, persist=False)
                    holds = []
                    oscillating = False
                    self.control_enabled = True
                    for frac in fractions:
                        check_abort()
                        target = frac * p["probe_force"]
                        self._at_phase(
                            "hold", f"Closed-loop hold at {target:g} "
                            f"(pass {attempt})")
                        self.force_target = target
                        samples = []
                        end = time.time() + p["hold_s"]
                        while time.time() < end:
                            check_abort()
                            samples.append(self.force_units())
                            time.sleep(0.02)
                        tail = samples[len(samples) // 3:]
                        mean = sum(tail) / len(tail)
                        rms = (sum((s - target) ** 2
                                   for s in tail) / len(tail)) ** 0.5
                        std = (sum((s - mean) ** 2
                                   for s in tail) / len(tail)) ** 0.5
                        holds.append({"target": round(target, 1),
                                      "mean": round(mean, 2),
                                      "rms_error": round(rms, 2),
                                      "std": round(std, 2),
                                      "kp_used": rec["pid_kp"]})
                        if std > max(3.0 * noise, 0.75 * deadband):
                            oscillating = True
                    self.control_enabled = False
                    if not oscillating or attempt == 2:
                        break
                    # Spread well beyond noise = oscillating: back off Kp
                    # (Ki keeps its ratio to Kp)
                    rec["pid_kp"] = round(rec["pid_kp"] * 0.5, 6)
                    rec["pid_ki"] = round(rec["pid_kp"] / 3.0, 6)
                    holds[-1]["note"] = ("oscillation detected — Kp "
                                         "halved, retried")
            finally:
                self.control_enabled = False
                self.update_config(saved_cfg, persist=False)
                self.force_target = saved_target

            at["results"] = {
                "noise_std": round(noise, 3),
                "contact_z": at["contact_z"],
                "stiffness": round(stiffness, 1),
                "stiffness_curve": curve,
                "hysteresis_mm": (round(hysteresis, 4)
                                  if hysteresis is not None else None),
                "peak_force": round(peak, 1),
                "step_response": step_resp,
                "fine_step_used": round(fine, 4),
                "note": ("sweep requested but the loading curve was too "
                         "sparse to fit stiffness bands — no gain schedule"
                         if p["sweep"] >= 0.5 and not sweeping else None),
                "hold": holds[-1] if holds else None,
                "holds": holds,
            }
            at["recommended"] = rec
            final_state = "done"

        except _AutotuneAbort:
            final_state = "aborted"
            at["message"] = "aborted by user"
        except RuntimeError as e:
            at["error"] = str(e)
        except Exception as e:  # never leave the head buried on a bug
            at["error"] = f"unexpected: {e}"
        finally:
            self.control_enabled = False
            self._at_phase("retract", "Retracting to start height")
            self._gcode("SET_GCODE_OFFSET Z=0", timeout=5.0)
            retracted = False
            retract_feed = float(self.config["retract_feedrate"])
            if start_z is not None:
                retracted = self._gcode(
                    f"G90\nG1 Z{start_z:.3f} F{retract_feed:.0f}\nM400",
                    timeout=60.0)
            if not retracted and descended > 0:
                self._at_move_z(min(descended,
                                    float(self.config["retract_mm"])),
                                retract_feed)
            at["state"] = final_state
            at["phase"] = "finished"
            if final_state == "done":
                at["message"] = "auto-tune complete"

    # ------------------------------------------------------------------ #
    # Surface mesh probing
    #
    # Touch off an N x N grid centered on the current XY (same rigid
    # non-spinning rod as auto-tune) and record the contact Z at each
    # point. The result maps the tilt/warp of the load-cell platform and
    # substrate. During force control it is used as a feedforward only —
    # the live force reading always remains the primary signal.

    def mesh_probing(self):
        return self.mesh_probe.get("state") == "running"

    def start_mesh(self, params=None):
        busy = self._exclusive_busy()
        if busy:
            return False, f"Cannot probe mesh: {busy}"
        if not self.sensor_ok:
            return False, "Sensor not available"
        if self.klipper_state in ("offline", "unknown", "error",
                                  "startup", "shutdown"):
            return False, "Klipper is not connected"
        homed = self.homed_axes or ""
        if not all(a in homed for a in "xyz"):
            return False, "All axes must be homed for mesh probing"

        p = dict(MESH_DEFAULTS)
        for key, (lo, hi) in MESH_LIMITS.items():
            if params and key in params:
                try:
                    p[key] = float(params[key])
                except (TypeError, ValueError):
                    return False, f"Bad value for {key}"
            p[key] = max(lo, min(hi, p[key]))
        p["points_per_side"] = int(round(p["points_per_side"]))
        # Resolve the rectangular grid (0 = inherit the square values)
        nx = int(round(p["points_x"])) or p["points_per_side"]
        ny = int(round(p["points_y"])) or p["points_per_side"]
        if nx < 2 or ny < 2:
            return False, "At least 2 points per axis"
        if nx * ny > 120:
            return False, (f"Grid too dense ({nx}x{ny} = {nx * ny} "
                           "points, max 120)")
        p["points_x"], p["points_y"] = nx, ny
        p["size_x_mm"] = p["size_x_mm"] or p["size_mm"]
        p["size_y_mm"] = p["size_y_mm"] or p["size_mm"]

        self.control_enabled = False
        self._mesh_abort.clear()
        self.mesh_probe = {
            "state": "running", "phase": "starting", "message": "",
            "params": p, "started": time.time(), "error": None,
            "done": 0, "total": nx * ny,
        }
        self._mesh_thread = threading.Thread(
            target=self._mesh_worker, daemon=True)
        self._mesh_thread.start()
        return True, "Mesh probing started"

    def abort_mesh(self):
        if not self.mesh_probing():
            return False
        self.control_enabled = False
        self._mesh_abort.set()
        return True

    def clear_mesh(self):
        self.mesh = None
        try:
            os.remove(MESH_PATH)
        except OSError:
            pass

    # -------- named mesh library (one mesh per substrate mount) -------- #

    def list_meshes(self):
        meshes = []
        for fname in sorted(os.listdir(MESH_DIR)):
            if not fname.endswith(".json"):
                continue
            try:
                with open(os.path.join(MESH_DIR, fname)) as f:
                    m = json.load(f)
            except (OSError, ValueError):
                continue
            meshes.append({
                "name": m.get("name", fname[:-5]),
                "created": m.get("created"),
                "saved": m.get("saved"),
                "range_mm": m.get("range_mm"),
                "size_mm": m.get("size_mm"),
                "points_per_side": m.get("points_per_side"),
                "points_x": m.get("points_x"),
                "points_y": m.get("points_y"),
                "size_x_mm": m.get("size_x_mm"),
                "size_y_mm": m.get("size_y_mm"),
                "center": m.get("center"),
            })
        return meshes

    def save_mesh_as(self, name):
        if not self.mesh:
            return False, "No active mesh to save"
        clean, path = self._named_path(MESH_DIR, name)
        if not clean:
            return False, "Mesh needs a name"
        m = dict(self.mesh)
        m["name"] = clean
        m["saved"] = time.time()
        try:
            with open(path, "w") as f:
                json.dump(m, f, indent=2)
        except OSError as e:
            return False, f"Could not write mesh: {e}"
        return True, clean

    def load_mesh_named(self, name):
        clean, path = self._named_path(MESH_DIR, name)
        if not clean:
            return False, "Mesh needs a name"
        try:
            with open(path) as f:
                m = json.load(f)
        except (OSError, ValueError):
            return False, f"Mesh '{clean}' not found or unreadable"
        if not (m.get("xs") and m.get("ys") and m.get("z")):
            return False, f"Mesh '{clean}' is malformed"
        self.mesh = m
        save_mesh(m)  # becomes the active mesh, survives restart
        return True, clean

    def delete_mesh_named(self, name):
        clean, path = self._named_path(MESH_DIR, name)
        if not clean:
            return False, "Mesh needs a name"
        try:
            os.remove(path)
        except OSError:
            return False, f"Mesh '{clean}' not found"
        return True, clean

    def _mesh_worker(self):
        mp = self.mesh_probe
        p = mp["params"]
        probe_feed = p["feedrate"]
        travel_feed = float(self.config["retract_feedrate"])
        start_z = None
        cur_z = None
        final_state = "error"

        def check_abort():
            if self._mesh_abort.is_set():
                raise _AutotuneAbort()

        def zgo(z, feed):
            check_abort()
            if not self._gcode(f"G90\nG1 Z{z:.4f} F{feed:.0f}\nM400",
                               timeout=60.0):
                raise RuntimeError(
                    "Z move rejected (Klipper offline or outside limits)")
            return z

        # Mesh coordinates are MACHINE coords: convert to the commanded
        # frame so an active XY work origin doesn't shift the grid.
        offx, offy = self.xy_offset

        def xygo(x, y):
            check_abort()
            if not self._gcode(
                    f"G90\nG1 X{x - offx:.3f} Y{y - offy:.3f} "
                    f"F{p['xy_feedrate']:.0f}\nM400",
                    timeout=120.0):
                raise RuntimeError(
                    f"XY move to ({x:.1f}, {y:.1f}) rejected — grid point "
                    "outside machine limits?")

        try:
            mp["phase"] = "setup"
            mp["message"] = "Clearing Z offset, taring in free air"
            self.control_enabled = False
            if not self._gcode("SET_GCODE_OFFSET Z=0", timeout=5.0):
                raise RuntimeError("Could not reach Klipper")
            time.sleep(0.6)
            start_x, start_y = self.position[0], self.position[1]
            start_z = cur_z = self.position[2]
            if not self.tare():
                raise RuntimeError("Tare failed (sensor unavailable)")

            mp["phase"] = "noise"
            mp["message"] = "Measuring baseline sensor noise"
            baseline, noise = self._at_sample(1.5)
            check_abort()
            threshold = max(6.0 * noise, p["threshold"])
            floor_z = max(start_z - p["max_travel"],
                          float(self.config.get("z_floor", -1000.0)))

            # Serpentine grid centered on the start position
            nx, ny = p["points_x"], p["points_y"]
            sx, sy = p["size_x_mm"], p["size_y_mm"]
            xs = [start_x - sx / 2.0 + i * sx / (nx - 1)
                  for i in range(nx)]
            ys = [start_y - sy / 2.0 + i * sy / (ny - 1)
                  for i in range(ny)]
            order = []
            for iy in range(ny):
                row = range(nx) if iy % 2 == 0 else range(nx - 1, -1, -1)
                order.extend((ix, iy) for ix in row)

            zgrid = [[None] * nx for _ in range(ny)]
            highest = None  # highest contact Z seen so far (machine coords)

            for count, (ix, iy) in enumerate(order, start=1):
                gx, gy = xs[ix], ys[iy]
                mp["phase"] = "probe"
                mp["message"] = (f"point {count}/{nx * ny} "
                                 f"({gx:.1f}, {gy:.1f})")

                travel_z = (start_z if highest is None
                            else min(start_z, highest + p["clearance"]))
                cur_z = zgo(travel_z, travel_feed)
                xygo(gx, gy)
                time.sleep(p["settle_s"])

                # Coarse descent until contact
                while True:
                    if cur_z - p["approach_step"] < floor_z:
                        raise RuntimeError(
                            f"No contact at ({gx:.1f}, {gy:.1f}) within "
                            f"{p['max_travel']:g} mm of the start height")
                    cur_z = zgo(cur_z - p["approach_step"], probe_feed)
                    time.sleep(p["settle_s"])
                    f, _ = self._at_sample(0.15)
                    if f - baseline >= threshold:
                        break

                # Back off and refine
                backoff = max(2.0 * p["approach_step"], 4.0 * p["fine_step"])
                cur_z = zgo(cur_z + backoff, probe_feed)
                time.sleep(max(0.3, p["settle_s"]))
                fine_limit = int(backoff / p["fine_step"] * 3) + 10
                fine_n = 0
                while True:
                    cur_z = zgo(cur_z - p["fine_step"], probe_feed)
                    time.sleep(p["settle_s"])
                    f, _ = self._at_sample(0.15)
                    if f - baseline >= threshold:
                        break
                    fine_n += 1
                    if fine_n > fine_limit:
                        raise RuntimeError(
                            f"Lost contact refining ({gx:.1f}, {gy:.1f})")

                zgrid[iy][ix] = round(cur_z, 4)
                highest = cur_z if highest is None else max(highest, cur_z)
                mp["done"] = count

            mp["phase"] = "finish"
            mp["message"] = "Returning to start"
            zvals = [z for row in zgrid for z in row]
            self.mesh = {
                "created": time.time(),
                "center": [round(start_x, 3), round(start_y, 3)],
                "size_mm": max(sx, sy),
                "size_x_mm": sx,
                "size_y_mm": sy,
                "points_per_side": max(nx, ny),
                "points_x": nx,
                "points_y": ny,
                "xs": [round(v, 3) for v in xs],
                "ys": [round(v, 3) for v in ys],
                "z": zgrid,
                "noise_std": round(noise, 3),
                "threshold": round(threshold, 2),
                "range_mm": round(max(zvals) - min(zvals), 4),
            }
            save_mesh(self.mesh)
            final_state = "done"

        except _AutotuneAbort:
            final_state = "aborted"
            mp["message"] = "aborted by user"
        except RuntimeError as e:
            mp["error"] = str(e)
        except Exception as e:  # never leave the head buried on a bug
            mp["error"] = f"unexpected: {e}"
        finally:
            self.control_enabled = False
            mp["phase"] = "retract"
            if start_z is not None:
                retract_ok = self._gcode(
                    f"G90\nG1 Z{start_z:.3f} F{travel_feed:.0f}\nM400",
                    timeout=60.0)
                if retract_ok and final_state == "done":
                    self._gcode(
                        f"G90\nG1 X{start_x - offx:.3f} "
                        f"Y{start_y - offy:.3f} "
                        f"F{p['xy_feedrate']:.0f}\nM400", timeout=120.0)
            mp["state"] = final_state
            mp["phase"] = "finished"
            if final_state == "done":
                mp["message"] = (f"mesh complete — range "
                                 f"{self.mesh['range_mm']:.3f} mm")

    # ------------------------------------------------------------------ #
    # Single-point touch-off
    #
    # Quick tool-length / starting-height reference after swapping to a
    # different-length mechtrode or after wear: descend at the current XY
    # until contact, record the surface Z, then park hover_mm above it.
    # The delta against the previous touch-off is reported (wear, or the
    # length difference of the new rod).

    def touchoff_running(self):
        return self.touchoff.get("state") == "running"

    def start_touchoff(self, params=None):
        busy = self._exclusive_busy()
        if busy:
            return False, f"Cannot touch off: {busy}"
        if not self.sensor_ok:
            return False, "Sensor not available"
        if self.klipper_state in ("offline", "unknown", "error",
                                  "startup", "shutdown"):
            return False, "Klipper is not connected"
        if "z" not in (self.homed_axes or ""):
            return False, "Z is not homed"

        p = dict(TOUCHOFF_DEFAULTS)
        for key, (lo, hi) in TOUCHOFF_LIMITS.items():
            if params and key in params:
                try:
                    p[key] = float(params[key])
                except (TypeError, ValueError):
                    return False, f"Bad value for {key}"
            p[key] = max(lo, min(hi, p[key]))

        self.control_enabled = False
        self._touchoff_abort.clear()
        self.touchoff = {
            "state": "running", "phase": "starting", "message": "",
            "params": p, "error": None,
            "last": self.touchoff.get("last"),
        }
        self._touchoff_thread = threading.Thread(
            target=self._touchoff_worker, daemon=True)
        self._touchoff_thread.start()
        return True, "Touch-off started"

    def abort_touchoff(self):
        if not self.touchoff_running():
            return False
        self.control_enabled = False
        self._touchoff_abort.set()
        return True

    def _touchoff_worker(self):
        to = self.touchoff
        p = to["params"]
        start_z = None
        cur_z = None
        final_state = "error"

        def check_abort():
            if self._touchoff_abort.is_set():
                raise _AutotuneAbort()

        def zgo(z, feed):
            check_abort()
            if not self._gcode(f"G90\nG1 Z{z:.4f} F{feed:.0f}\nM400",
                               timeout=60.0):
                raise RuntimeError(
                    "Z move rejected (Klipper offline or outside limits)")
            return z

        try:
            to["phase"] = "setup"
            to["message"] = "Clearing Z offset, taring in free air"
            self.control_enabled = False
            if not self._gcode("SET_GCODE_OFFSET Z=0", timeout=5.0):
                raise RuntimeError("Could not reach Klipper")
            time.sleep(0.6)
            x, y = self.position[0], self.position[1]
            start_z = cur_z = self.position[2]
            if not self.tare():
                raise RuntimeError("Tare failed (sensor unavailable)")

            to["phase"] = "noise"
            to["message"] = "Measuring baseline sensor noise"
            baseline, noise = self._at_sample(1.5)
            check_abort()
            threshold = max(6.0 * noise, p["threshold"])
            floor_z = max(start_z - p["max_travel"],
                          float(self.config.get("z_floor", -1000.0)))

            to["phase"] = "approach"
            while True:
                if cur_z - p["approach_step"] < floor_z:
                    raise RuntimeError(
                        f"No contact within {p['max_travel']:g} mm — park "
                        "the head closer or raise max travel")
                cur_z = zgo(cur_z - p["approach_step"], p["feedrate"])
                time.sleep(p["settle_s"])
                f, _ = self._at_sample(0.15)
                to["message"] = (f"Z {cur_z:.2f}, "
                                 f"force {f - baseline:+.1f}")
                if f - baseline >= threshold:
                    break

            to["phase"] = "refine"
            to["message"] = "Backing off, re-touching finely"
            backoff = max(2.0 * p["approach_step"], 4.0 * p["fine_step"])
            cur_z = zgo(cur_z + backoff, p["feedrate"])
            time.sleep(max(0.3, p["settle_s"]))
            fine_limit = int(backoff / p["fine_step"] * 3) + 10
            fine_n = 0
            while True:
                cur_z = zgo(cur_z - p["fine_step"], p["feedrate"])
                time.sleep(p["settle_s"])
                f, _ = self._at_sample(0.15)
                if f - baseline >= threshold:
                    break
                fine_n += 1
                if fine_n > fine_limit:
                    raise RuntimeError("Lost contact during fine re-touch")

            prev = to.get("last") or {}
            prev_z = prev.get("surface_z")
            last = {
                "surface_z": round(cur_z, 4),
                "x": round(x, 3), "y": round(y, 3),
                "created": time.time(),
                "prev_surface_z": prev_z,
                "delta": (round(cur_z - prev_z, 4)
                          if prev_z is not None else None),
                "noise_std": round(noise, 3),
            }
            to["last"] = last
            try:
                with open(TOUCHOFF_PATH, "w") as f:
                    json.dump(last, f, indent=2)
            except OSError:
                pass
            final_state = "done"

        except _AutotuneAbort:
            final_state = "aborted"
            to["message"] = "aborted by user"
        except RuntimeError as e:
            to["error"] = str(e)
        except Exception as e:  # never leave the head buried on a bug
            to["error"] = f"unexpected: {e}"
        finally:
            self.control_enabled = False
            to["phase"] = "retract"
            feed = float(self.config["retract_feedrate"])
            if final_state == "done" and cur_z is not None:
                # Park hover_mm above the freshly measured surface,
                # ready to engage
                park = min(start_z, cur_z + p["hover_mm"])
                self._gcode(f"G90\nG1 Z{park:.3f} F{feed:.0f}\nM400",
                            timeout=60.0)
            elif start_z is not None:
                self._gcode(f"G90\nG1 Z{start_z:.3f} F{feed:.0f}\nM400",
                            timeout=60.0)
            to["state"] = final_state
            to["phase"] = "finished"
            if final_state == "done":
                d = to["last"]["delta"]
                to["message"] = (
                    f"surface Z {to['last']['surface_z']:.3f} mm"
                    + (f", {d:+.3f} vs last touch-off" if d is not None
                       else ""))

    # ------------------------------------------------------------------ #
    # Mechtrode calibration profiles
    #
    # A profile freezes one mechtrode's tuning (controller settings plus,
    # when it came from auto-tune, the measured stiffness/hysteresis and
    # probe curves) into a named JSON file so switching rods is a one-click
    # reload instead of a re-tune-from-scratch.

    @staticmethod
    def _named_path(dirpath, name):
        """Sanitized filesystem path for a user-supplied name, or
        (None, None) if the name reduces to nothing."""
        clean = re.sub(r"[^A-Za-z0-9._ -]+", "_", str(name)).strip("._ ")
        clean = clean[:60].strip()
        if not clean:
            return None, None
        return clean, os.path.join(dirpath, clean + ".json")

    @classmethod
    def _profile_path(cls, name):
        return cls._named_path(PROFILE_DIR, name)

    def list_profiles(self):
        profiles = []
        for fname in sorted(os.listdir(PROFILE_DIR)):
            if not fname.endswith(".json"):
                continue
            try:
                with open(os.path.join(PROFILE_DIR, fname)) as f:
                    p = json.load(f)
            except (OSError, ValueError):
                continue
            m = p.get("measurement") or {}
            s = p.get("settings") or {}
            profiles.append({
                "name": p.get("name", fname[:-5]),
                "created": p.get("created"),
                "source": p.get("source"),
                "units_label": p.get("units_label"),
                "stiffness": m.get("stiffness"),
                "pid_kp": s.get("pid_kp"),
                "force_deadband": s.get("force_deadband"),
            })
        return profiles

    def save_profile(self, name, source="current"):
        clean, path = self._profile_path(name)
        if not clean:
            return False, "Profile needs a name"

        settings = {k: self.config[k] for k in PROFILE_KEYS}
        measurement = None
        points = None
        if source == "autotune":
            rec = self.autotune.get("recommended")
            if not rec:
                return False, "No auto-tune results to save yet"
            settings.update(rec)
            measurement = self.autotune.get("results")
            points = self.autotune.get("points")

        profile = {
            "name": clean,
            "created": time.time(),
            "source": source,
            # Scale calibration at save time, for reference and a
            # load-time sanity check (profiles store calibrated units).
            "units_label": self.config["units_label"],
            "counts_per_unit": self.config["counts_per_unit"],
            "settings": settings,
            "measurement": measurement,
            "points": points,
        }
        try:
            with open(path, "w") as f:
                json.dump(profile, f, indent=2)
        except OSError as e:
            return False, f"Could not write profile: {e}"
        return True, clean

    def load_profile(self, name):
        clean, path = self._profile_path(name)
        if not clean:
            return False, "Profile needs a name", None
        try:
            with open(path) as f:
                p = json.load(f)
        except (OSError, ValueError):
            return False, f"Profile '{clean}' not found or unreadable", None

        settings = {k: v for k, v in (p.get("settings") or {}).items()
                    if k in PROFILE_KEYS}
        if not settings:
            return False, f"Profile '{clean}' has no settings", None
        self.update_config(settings, persist=True)

        warnings = []
        if p.get("units_label") not in (None, self.config["units_label"]):
            warnings.append(
                f"profile was saved in '{p['units_label']}' but the scale "
                f"is now in '{self.config['units_label']}'")
        cpu = p.get("counts_per_unit")
        if cpu and abs(cpu - float(self.config["counts_per_unit"])) \
                > 0.01 * abs(cpu):
            warnings.append(
                "scale calibration factor has changed since this profile "
                "was saved — consider re-running auto-tune")
        return True, clean, warnings

    def delete_profile(self, name):
        clean, path = self._profile_path(name)
        if not clean:
            return False, "Profile needs a name"
        try:
            os.remove(path)
        except OSError:
            return False, f"Profile '{clean}' not found"
        return True, clean

    # ------------------------------------------------------------------ #
    # Mid-job rod change
    #
    # Rod consumption = how far the force controller has plunged beyond
    # the mesh feedforward since the job started (or since the last rod
    # change). When it hits max_rod_consumption while engaged, the job
    # pauses: retract -> user swaps rod -> tare + touch-off establishes
    # the new rod length -> the job coordinate frame is shifted by the
    # length difference -> user confirms spindle -> slow re-engage to the
    # active force target -> the job continues where it left off.

    def rod_consumption(self):
        if self._rod_baseline is None:
            return 0.0
        return max(0.0, -(self.z_offset - self._mesh_applied
                          - self._rod_baseline))

    def _rod_change_due(self):
        limit = float(self.config.get("max_rod_consumption", 0))
        if limit <= 0:
            return False
        # Only evaluate while actually engaged (holding a nonzero target):
        # the pause position doubles as the old rod's surface reference,
        # so the tip must be at the surface when we stop.
        if not self.control_enabled or self.force_target <= 0:
            return False
        avg = self.force_avg()
        if avg is None or (abs(avg - self.force_target)
                           > 2.0 * float(self.config["force_deadband"])):
            return False
        # First stable engagement defines the baseline: the plunge from
        # park height to the surface is approach, not wear.
        if self._rod_baseline is None:
            self._rod_baseline = self.z_offset - self._mesh_applied
            return False
        return self.rod_consumption() >= limit

    def confirm_job(self):
        """User confirmation for the current rod-change wait stage."""
        job = self.job
        if not (job and job.get("state") == "running" and job.get("pause")):
            return False, "No paused job awaiting confirmation"
        if not str(job["pause"].get("stage", "")).startswith("waiting"):
            return False, "Job is not waiting for confirmation"
        self._job_confirm.set()
        return True, "confirmed"

    def _wait_confirm(self):
        """Block until the user confirms or the job is aborted. Returns
        False on abort."""
        self._job_confirm.clear()
        while not self._job_confirm.wait(timeout=0.2):
            if self._job_abort.is_set():
                return False
        return True

    def _rod_change_pause(self, job):
        """Run the full rod-change sequence. Returns an error string, or
        None on success or abort (caller re-checks the abort flag)."""
        retract = float(self.config["rod_change_retract"])
        feed = float(self.config["retract_feedrate"])
        tp = TOUCHOFF_DEFAULTS
        pause = {"stage": "retracting",
                 "message": "Rod consumption limit reached — retracting",
                 "consumed": round(self.rod_consumption(), 3)}
        job["pause"] = pause

        try:
            self.control_enabled = False
            time.sleep(0.6)  # let the last correction land, telemetry catch up
            o_pause = self.z_offset
            zm_pause = self.position[2]   # tip was at the surface here

            if not self._gcode(
                    f"G91\nG1 Z{retract:.3f} F{feed:.0f}\nG90\nM400",
                    timeout=60.0):
                return self._fail_why("Rod-change retract failed")

            pause["stage"] = "waiting_rod_change"
            pause["message"] = (
                f"Rod consumption limit reached ({pause['consumed']:g} mm). "
                "Swap/reload the mechtrode with the spindle OFF, then "
                "confirm.")
            if not self._wait_confirm():
                return None

            pause["stage"] = "touchoff"
            pause["message"] = "Taring and touching off with the new rod"
            if not self._gcode("SET_GCODE_OFFSET Z=0", timeout=5.0):
                return self._fail_why("Could not clear the Z offset")
            time.sleep(0.6)
            if not self.tare():
                return "Tare failed (sensor unavailable)"
            try:
                zm_new = self._probe_contact(
                    max_travel=retract + 20.0, feed=tp["feedrate"],
                    approach_step=tp["approach_step"],
                    fine_step=tp["fine_step"], settle_s=tp["settle_s"],
                    threshold_floor=tp["threshold"],
                    abort_event=self._job_abort)
            except _AutotuneAbort:
                return None
            except RuntimeError as e:
                return f"Rod-change touch-off failed: {e}"

            # Shift the job's coordinate frame by the rod length change so
            # commanded coordinates put the new tip where the old tip was.
            length_delta = zm_new - zm_pause
            o_new = o_pause + length_delta
            pause["length_delta"] = round(length_delta, 3)

            if not self._gcode(
                    f"G91\nG1 Z{retract:.3f} F{feed:.0f}\nG90\nM400",
                    timeout=60.0):
                return self._fail_why("Post-touch-off retract failed")
            if not self._gcode(f"SET_GCODE_OFFSET Z={o_new:.4f}",
                               timeout=5.0):
                return self._fail_why("Could not restore the Z offset")
            # Re-capture the consumption baseline at the next stable
            # engagement of the new rod
            self._rod_baseline = None

            pause["stage"] = "waiting_spindle"
            pause["message"] = (
                f"New rod measured ({length_delta:+.2f} mm vs old). Start "
                "the spindle, then confirm to re-engage.")
            if not self._wait_confirm():
                return None

            pause["stage"] = "reengage"
            pause["message"] = (
                f"Re-engaging to force target {self.force_target:g}")
            # Commanded coordinate of the new contact is zm_new - o_new;
            # park 0.5 mm above it, then let the force controller plunge.
            approach_cmd = (zm_new - o_new) + 0.5
            if not self._gcode(
                    f"G90\nG1 Z{approach_cmd:.3f} F300\nM400",
                    timeout=120.0):
                return self._fail_why("Re-approach move failed")
            self.control_enabled = True
            deadline = time.time() + 120.0
            deadband = float(self.config["force_deadband"])
            while not self._job_abort.is_set():
                avg = self.force_avg()
                if (avg is not None
                        and abs(avg - self.force_target) <= deadband):
                    break
                if time.time() > deadline:
                    return ("Re-engage timed out (force target not "
                            "reached in 120 s)")
                time.sleep(0.05)
            # Re-induction: let the fresh rod heat before traversing
            induct = float(self.config.get("rod_change_induction_s", 0))
            if induct > 0 and not self._job_abort.is_set():
                pause["message"] = (f"Re-inducting new rod: {induct:g} s "
                                    "at force")
                end = time.time() + induct
                while (time.time() < end
                       and not self._job_abort.is_set()):
                    time.sleep(0.05)
            return None
        finally:
            job["pause"] = None

    # ------------------------------------------------------------------ #
    # Manual pause / resume

    def request_pause(self):
        """Ask the job runner to pause after the current segment."""
        job = self.job
        if not (job and job.get("state") == "running"):
            return False, "No job running"
        if job.get("pause"):
            return False, "Job is already paused"
        self._job_pause_req = True
        return True, "Pausing after the current segment"

    def _manual_pause(self, job):
        """User-requested pause: retract, wait for Resume, return to the
        pause position and re-engage if the tool was at force. Returns an
        error string, or None on success or abort."""
        self._job_pause_req = False
        retract = float(self.config["rod_change_retract"])
        feed = float(self.config["retract_feedrate"])
        pause = {"stage": "pausing", "message": "Pausing — retracting"}
        job["pause"] = pause
        try:
            avg = self.force_avg()
            engaged = (self.control_enabled and self.force_target > 0
                       and avg is not None
                       and abs(avg - self.force_target)
                       <= 2.0 * float(self.config["force_deadband"]))
            self.control_enabled = False
            time.sleep(0.5)
            zm = self.position[2]
            off = self.z_offset

            if not self._gcode(
                    f"G91\nG1 Z{retract:.3f} F{feed:.0f}\nG90\nM400",
                    timeout=60.0):
                return self._fail_why("Pause retract failed")

            pause["stage"] = "waiting_resume"
            pause["message"] = ("Job paused. Press Resume to return to "
                                "position and continue."
                                + (" The tool will re-engage to "
                                   f"{self.force_target:g}." if engaged
                                   else ""))
            if not self._wait_confirm():
                return None

            pause["stage"] = "resume"
            pause["message"] = "Returning to position"
            # Commanded coordinate of the pause position (offset is
            # unchanged during the pause); hover 0.3 mm short if the tool
            # was engaged so force control does the final approach.
            target_cmd = (zm - off) + (0.3 if engaged else 0.0)
            if not self._gcode(
                    f"G90\nG1 Z{target_cmd:.3f} F300\nM400", timeout=120.0):
                return self._fail_why("Resume move failed")
            self.control_enabled = True
            if engaged:
                pause["message"] = (f"Re-engaging to force target "
                                    f"{self.force_target:g}")
                deadline = time.time() + 120.0
                deadband = float(self.config["force_deadband"])
                while not self._job_abort.is_set():
                    avg = self.force_avg()
                    if (avg is not None
                            and abs(avg - self.force_target) <= deadband):
                        break
                    if time.time() > deadline:
                        return ("Re-engage timed out (force target not "
                                "reached in 120 s)")
                    time.sleep(0.05)
            return None
        finally:
            job["pause"] = None

    def _pause_or_rodchange_due(self):
        return self._job_pause_req or self._rod_change_due()

    def _handle_pause_point(self, job):
        """Run whichever pause is due. Returns an error string or None."""
        if self._job_pause_req:
            return self._manual_pause(job)
        return self._rod_change_pause(job)

    # ------------------------------------------------------------------ #
    # Job execution

    def list_jobs(self):
        jobs = []
        limits = self._machine_limits()   # fetch once for the whole list
        for name in sorted(os.listdir(JOB_DIR)):
            if not name.lower().endswith(JOB_EXTENSIONS):
                continue
            path = os.path.join(JOB_DIR, name)
            entry = {"name": name, "size": os.path.getsize(path),
                     "mtime": os.path.getmtime(path)}
            try:
                steps = parse_job_file(path)
                entry["steps"] = len(steps)
                entry["valid"] = True
                entry["preflight"] = self.check_job(steps, limits=limits)
            except ValueError as e:
                entry["steps"] = 0
                entry["valid"] = False
                entry["error"] = str(e)
            jobs.append(entry)
        return jobs

    def start_job(self, filename, notes="", start_step=1):
        busy = self._exclusive_busy()
        if busy:
            return False, f"Cannot start job: {busy}"
        if self.klipper_state in ("offline", "unknown", "error", "startup",
                                  "shutdown"):
            return False, f"Klipper is not ready ({self.klipper_state})"

        path = os.path.join(JOB_DIR, os.path.basename(filename))
        try:
            steps = parse_job_file(path)
        except (OSError, ValueError) as e:
            return False, str(e)

        # Preflight: a force reading far from zero with the head parked
        # in free air means a stale tare or something new on the
        # platform — the controller would chase a phantom load and never
        # reach its target.
        max_target = max((s["value"] for s in steps
                          if s["type"] == "force"), default=0.0)
        if self.sensor_ok and max_target > 0:
            reading = self.force_units()
            if abs(reading) > max(100.0, 0.3 * max_target):
                return False, (
                    f"Force reads {reading:.0f} "
                    f"{self.config['units_label']} before starting — "
                    "stale tare or load on the platform. Tare in free "
                    "air (rod not touching) and retry")

        # Preflight: refuse jobs with hard errors (out of machine volume,
        # below the Z floor, force above the safety limit, bad feedrates)
        report = self.check_job(steps)
        if report["errors"]:
            shown = "; ".join(report["errors"][:3])
            more = len(report["errors"]) - 3
            if more > 0:
                shown += f" (+{more} more)"
            return False, f"Preflight failed: {shown}"

        try:
            start_step = max(1, min(int(start_step), len(steps)))
        except (TypeError, ValueError):
            return False, "Bad start step"
        # When resuming mid-file, carry over the force target the skipped
        # steps would have left active.
        resume_force = None
        for s in steps[:start_step - 1]:
            if s["type"] == "force":
                resume_force = s["value"]

        self._job_abort.clear()
        self._job_pause_req = False
        self.job = {
            "file": os.path.basename(filename),
            "state": "running",
            "row": start_step - 1,
            "total": len(steps),
            "start_step": start_step,
            "step_desc": "",
            "error": None,
            "started": time.time(),
            "log": None,
            "pause": None,
        }
        self._job_thread = threading.Thread(
            target=self._job_worker,
            args=(steps, str(notes or ""), start_step, resume_force),
            daemon=True)
        self._job_thread.start()
        return True, "Job started"

    def _run_log_header(self, jobname, notes):
        """Human-readable '# key: value' metadata for a run log."""
        cfg = self.config
        lines = [
            f"job: {jobname}",
            f"started: {time.strftime('%Y-%m-%d %H:%M:%S')}",
            f"units: {cfg['units_label']}",
            f"counts_per_unit: {cfg['counts_per_unit']}",
            (f"pid: kp={cfg['pid_kp']:g} ki={cfg['pid_ki']:g} "
             f"kd={cfg['pid_kd']:g}"),
            (f"controller: deadband={cfg['force_deadband']:g} "
             f"z_step_max={cfg['z_step_max']:g} "
             f"cooldown={cfg['control_cooldown']:g} "
             f"avg_samples={cfg['force_avg_samples']:g}"),
            f"tared: {self.is_tared} (offset {self.tare_offset:g})",
        ]
        if self.mesh:
            lines.append(
                f"mesh: {self.mesh.get('name', 'unnamed')} "
                f"range={self.mesh.get('range_mm')} mm "
                f"enabled={float(cfg.get('mesh_enabled', 1.0)) != 0.0}")
        else:
            lines.append("mesh: none")
        last_to = self.touchoff.get("last") or {}
        if last_to.get("surface_z") is not None:
            lines.append(
                f"touchoff: surface_z={last_to['surface_z']} at "
                f"({last_to.get('x')}, {last_to.get('y')}) "
                + time.strftime('%Y-%m-%d %H:%M:%S',
                                time.localtime(last_to.get('created', 0))))
        else:
            lines.append("touchoff: none")
        for ln in notes.splitlines():
            lines.append(f"note: {ln}")
        return lines

    def _run_step(self, step):
        """Execute one job step. Returns an error string, or None on success."""
        if step["type"] == "force":
            self.force_target = step["value"]
            return None

        if step["type"] == "zref":
            return None   # frame applied at job start

        if step["type"] == "move":
            return self._run_move_streamed(step)

        if step["type"] == "wait_force":
            # Klipper's queue is empty here, so the force controller's Z
            # nudges execute in real time while we watch the rolling average.
            deadline = time.time() + step["timeout"]
            err = self._guarded_engage(deadline)
            if err:
                return err
            deadband = float(self.config["force_deadband"])
            while not self._job_abort.is_set():
                avg = self.force_avg()
                if (avg is not None
                        and abs(avg - self.force_target) <= deadband):
                    if self._pause_or_rodchange_due():
                        t0 = time.time()
                        err = self._handle_pause_point(self.job)
                        if err:
                            return err
                        if self._job_abort.is_set():
                            return None
                        deadline += time.time() - t0
                        continue
                    return None
                if time.time() > deadline:
                    return (f"WAIT_FORCE timed out after "
                            f"{step['timeout']:.0f}s (target "
                            f"{self.force_target:g} not reached)")
                time.sleep(0.05)
            return None  # aborted; caller handles it

        if step["type"] == "dwell":
            end = time.time() + step["seconds"]
            while not self._job_abort.is_set() and time.time() < end:
                if self._pause_or_rodchange_due():
                    t0 = time.time()
                    err = self._handle_pause_point(self.job)
                    if err:
                        return err
                    if self._job_abort.is_set():
                        return None
                    end += time.time() - t0  # pause doesn't eat dwell time
                time.sleep(0.05)
            return None

        return f"Unknown step type: {step['type']}"

    def _guarded_engage(self, deadline):
        """Close the authored approach gap (park height -> surface) with
        commanded creep moves before letting the PID pull to target.

        If WAIT_FORCE starts in free air and the controller closes the gap
        through SET_GCODE_OFFSET, that air distance stays in the offset for
        the rest of the job: it reads as rod consumption that never
        happened, and every later park (authored relative to the surface)
        lands offset-deep — at or below the material — and rams at plunge
        feed. Creeping down with commanded moves keeps the offset holding
        only real engagement depth and rod wear.

        No-op unless the head is clearly in free air (force below the
        contact threshold with a positive target). Returns an error string
        or None.
        """
        target = self.force_target
        deadband = float(self.config["force_deadband"])
        threshold = max(2.0 * deadband, 0.1 * target)
        avg = self.force_avg()
        if target <= 0 or avg is None or avg >= threshold:
            return None

        floor = float(self.config["z_floor"])
        descended = 0.0
        # Zero the target during the creep: the controller sits in its
        # deadband instead of plunging the offset in parallel.
        self.force_target = 0.0
        try:
            while not self._job_abort.is_set():
                avg = self.force_avg()
                if avg is None:
                    return "sensor stopped reporting during engagement"
                if avg >= threshold:
                    return None   # contact — PID takes it from here
                if time.time() > deadline:
                    return (f"no surface contact before the WAIT_FORCE "
                            f"timeout ({descended:.2f} mm descended)")
                if descended + ENGAGE_STEP_MM > ENGAGE_MAX_MM:
                    return (f"no surface contact within {ENGAGE_MAX_MM:g} "
                            f"mm of guarded descent — check touch-off "
                            f"and the job's park heights")
                if self.position[2] - ENGAGE_STEP_MM < floor:
                    return (f"guarded descent stopped at the Z floor "
                            f"({floor:g})")
                self._move_active = True
                try:
                    ok = self._gcode(
                        f"G91\nG1 Z-{ENGAGE_STEP_MM:g} F{ENGAGE_FEED:g}\n"
                        f"G90\nM400", timeout=10.0)
                finally:
                    self._move_active = False
                if not ok:
                    return self._fail_why("Move failed during guarded descent")
                descended += ENGAGE_STEP_MM
                time.sleep(0.06)   # let the rolling average catch up
            return None   # aborted; caller checks the flag
        finally:
            self.force_target = target

    def _run_move_streamed(self, step):
        """Send a move block one line at a time, paced to execution.

        The old approach (whole block + M400 in one script) held Klipper's
        gcode queue for the entire block: force-control adjustments issued
        meanwhile timed out client-side but still executed later as a
        destructive burst, and the controller was blind during every
        traverse. Streaming keeps the queue free between segments —
        corrections land with roughly one-segment latency — and makes
        abort/pause/rod-change responsive at segment granularity.
        """
        # Track commanded XY to estimate each segment's duration, and the
        # commanded job-frame Z so slices can re-command it: Klipper only
        # applies a gcode offset to axes explicitly present in a command,
        # so an XY-only slice would never realize streamed Z corrections
        # (the offset state silently diverges from the physical head).
        lx = self.position[0] - self.xy_offset[0]
        ly = self.position[1] - self.xy_offset[1]
        lz = self.position[2] - self.z_offset
        feed = None
        absolute = True
        # Dead-reckoned pacing clock: estimated wall-clock time at which
        # the last submitted motion finishes executing. Estimates assume
        # the commanded feed (the fastest a move can run), so this clock
        # can only lag reality — pacing on it keeps the planner fed
        # without ever racing ahead of the machine.
        ahead = time.time()

        def barrier():
            nonlocal ahead
            self._move_active = True
            try:
                return self._gcode("M400", timeout=3600.0)
            finally:
                self._move_active = False
                ahead = time.time()   # queue drained: clock re-synced

        def pause_gate():
            """Handle a pending pause / rod change. Returns (paused, err);
            the caller re-checks the abort flag."""
            if not self._pause_or_rodchange_due():
                return False, None
            self._streaming_xy = False   # stationary from here on
            if not barrier():   # let buffered motion finish first
                return True, self._fail_why("Motion sync before pause failed")
            return True, self._handle_pause_point(self.job)

        def xy_continues(next_line):
            """True if next_line is another absolute XY-only linear move,
            so the slice stream can flow through the junction without
            stopping for exact arrival."""
            if next_line is None or not absolute:
                return False
            nu = next_line.upper()
            if not _RE_LINEAR.match(nu):
                return False
            has_xy = False
            for axis, _ in _RE_WORD.findall(nu):
                a = axis.lower()
                if a in ("x", "y"):
                    has_xy = True
                elif a != "f":
                    return False
            return has_xy

        def submit(gline):
            self._move_active = True
            try:
                return self._gcode(gline, timeout=3600.0)
            finally:
                self._move_active = False

        lines = step["lines"]
        for li, line in enumerate(lines):
            if self._job_abort.is_set():
                return None
            _, err = pause_gate()
            if err:
                return err
            if self._job_abort.is_set():
                return None

            # Parse the segment: endpoint for arrival tracking + duration
            est = 0.1
            xy_target = None
            slices = None
            u = line.upper()
            if u.startswith("G90"):
                absolute = True
            elif u.startswith("G91"):
                absolute = False
            elif _RE_LINEAR.match(u):
                ox, oy = lx, ly
                nx, ny = lx, ly
                has_xy = False
                xy_only = True
                for axis, val in _RE_WORD.findall(u):
                    axis, val = axis.lower(), float(val)
                    if axis == "f":
                        feed = val
                    elif axis == "x":
                        nx = val if absolute else lx + val
                        has_xy = True
                    elif axis == "y":
                        ny = val if absolute else ly + val
                        has_xy = True
                    elif axis == "z":
                        lz = val if absolute else lz + val
                        xy_only = False   # mixed XYZ move: send verbatim
                dist = ((nx - lx) ** 2 + (ny - ly) ** 2) ** 0.5
                if has_xy and absolute:
                    xy_target = (nx + self.xy_offset[0],
                                 ny + self.xy_offset[1])
                lx, ly = nx, ny
                if feed and dist:
                    est = dist / feed * 60.0
                # Slice long XY moves into ~SLICE_TIME_S sub-moves (see
                # the constant): a single long G1 jams Klipper's gcode
                # input via lookahead pacing, blocking force corrections
                # for the whole pass.
                if xy_target is not None and xy_only and feed and dist:
                    slice_mm = max(SLICE_MIN_MM, feed / 60.0 * SLICE_TIME_S)
                    n = int(dist / slice_mm) + 1
                    if n > 1:
                        slices = [(ox + (nx - ox) * i / n,
                                   oy + (ny - oy) * i / n)
                                  for i in range(1, n + 1)]

            if slices:
                # Paced streaming, dead-reckoned on time. Telemetry
                # position (motion_report via Moonraker) only refreshes
                # every ~0.25 s, so gating each ~0.15 s slice on measured
                # arrival starved the planner: the head decelerated at
                # nearly every slice boundary and averaged ~55% of the
                # commanded feed — chop, plus a speed-coupled force
                # disturbance the Z controller then chased. Pacing on the
                # `ahead` clock instead keeps ~2 slices always buffered so
                # slices blend at full speed; the (stale) telemetry
                # distance stays as a coarse backstop so junction
                # slowdowns can't grow the buffer — and with it the force
                # correction latency — without bound. While streaming,
                # force corrections ride the upcoming slices (no MOVE=1);
                # abort and pause still act between slices. If the next
                # gcode line continues the XY path, the stream flows
                # through the junction without stopping for exact arrival.
                slice_dur = est / len(slices)
                lead_s = 2.0 * slice_dur
                cap2 = (5.0 * dist / len(slices)) ** 2
                deadline = time.time() + max(10.0, est * 3.0)
                flow_on = xy_continues(lines[li + 1]
                                       if li + 1 < len(lines) else None)
                self._streaming_xy = True
                for i, (px, py) in enumerate(slices):
                    if self._job_abort.is_set():
                        return None
                    paused, err = pause_gate()
                    if err:
                        return err
                    if self._job_abort.is_set():
                        return None
                    if paused:
                        # The pause held the machine stationary for
                        # minutes: without these resets the rest of this
                        # line runs with MOVE=1 corrections chopping the
                        # traverse and an expired deadline flooding the
                        # queue (~2 s correction latency).
                        self._streaming_xy = True
                        deadline = time.time() + max(10.0, est * 3.0)
                    # The explicit Z word (same base Z every slice) is
                    # what lets streamed corrections ride the motion:
                    # each slice re-commands Z, so it plans with the
                    # offset state as of its submission.
                    sub = f"G1 X{px:.3f} Y{py:.3f} Z{lz:.4f}"
                    if i == 0:
                        sub += f" F{feed:g}"
                    if not submit(sub):
                        return self._fail_why(f"Move failed ({sub.strip()})")
                    ahead = max(time.time(), ahead) + slice_dur
                    tx = px + self.xy_offset[0]
                    ty = py + self.xy_offset[1]
                    if i == len(slices) - 1 and not flow_on:
                        # Path ends here: wait for true physical arrival
                        # (next line changes mode, e.g. a Z lift).
                        while not self._job_abort.is_set():
                            dx = self.position[0] - tx
                            dy = self.position[1] - ty
                            if dx * dx + dy * dy < 0.0025:
                                break
                            if time.time() > deadline:
                                break   # telemetry hiccup: barrier syncs
                            time.sleep(0.03)
                        ahead = time.time()
                    else:
                        while not self._job_abort.is_set():
                            now = time.time()
                            dx = self.position[0] - tx
                            dy = self.position[1] - ty
                            if (ahead - now <= lead_s
                                    and dx * dx + dy * dy < cap2):
                                break
                            if now > deadline:
                                break   # estimate drifted: barrier syncs
                            time.sleep(0.02)
                continue

            if not submit(line):
                return self._fail_why(f"Move failed ({line.strip()})")

            if xy_target is not None:
                self._streaming_xy = True
                ahead = max(time.time(), ahead) + est
                if xy_continues(lines[li + 1]
                                if li + 1 < len(lines) else None):
                    # Next line continues the XY path: pace by time so
                    # short CAM segments blend through the junction
                    # instead of stopping for a stale arrival reading.
                    # The 2.5 mm telemetry backstop keeps corner
                    # slowdowns from growing the buffer unboundedly.
                    while not self._job_abort.is_set():
                        dx = self.position[0] - xy_target[0]
                        dy = self.position[1] - xy_target[1]
                        if (ahead - time.time() <= 0.3
                                and dx * dx + dy * dy < 6.25):
                            break
                        time.sleep(0.02)
                else:
                    # Path ends: wait for physical arrival via telemetry
                    # instead of an M400 barrier, so the gcode queue
                    # stays free and corrections keep flowing.
                    deadline = time.time() + max(10.0, est * 3.0)
                    while not self._job_abort.is_set():
                        dx = self.position[0] - xy_target[0]
                        dy = self.position[1] - xy_target[1]
                        if dx * dx + dy * dy < 0.0025:   # within 0.05 mm
                            break
                        if time.time() > deadline:
                            break   # telemetry hiccup: end barrier syncs
                        time.sleep(0.05)
                    ahead = time.time()
            else:
                # Z/lift/modal lines are short: cheap sync barrier
                self._streaming_xy = False
                if not barrier():
                    return self._fail_why("Motion sync failed")

        self._streaming_xy = False
        if not barrier():
            return self._fail_why("Motion sync at block end failed")
        return None

    def _job_worker(self, steps, notes="", start_step=1, resume_force=None):
        job = self.job
        try:
            # Establish the job's Z frame. Surface-referenced jobs anchor
            # Z0 to the last touched-off surface; otherwise the frame is
            # cleaned of any leftover force-control offset. The XY work
            # origin is deliberately preserved either way.
            surface_ref, surface_z = self._surface_ref(steps)
            base_z = surface_z if (surface_ref and surface_z is not None) \
                else 0.0
            job["surface_ref"] = surface_ref
            if not self._gcode(f"G90\nSET_GCODE_OFFSET Z={base_z:.4f}",
                               timeout=5.0):
                job["state"] = "error"
                job["error"] = self._fail_why(
                    "Could not set the job's Z frame")
                return

            # Auto-capture force data for the whole run. If the user is
            # already logging manually, leave their log alone.
            if not self.logging:
                base = os.path.splitext(job["file"])[0]
                path = self.start_log(
                    prefix=f"run_{base}",
                    header=self._run_log_header(job["file"], notes),
                    auto=True)
                job["log"] = os.path.basename(path)

            if resume_force is not None:
                self.force_target = resume_force

            self._rod_baseline = None
            self.control_enabled = True

            for i, step in enumerate(steps[start_step - 1:],
                                     start=start_step - 1):
                if self._job_abort.is_set():
                    job["state"] = "aborted"
                    break

                # Manual pause requested, or rod consumption limit reached
                # while engaged? Handle it before starting the next step.
                if self._pause_or_rodchange_due():
                    error = self._handle_pause_point(job)
                    if error:
                        job["state"] = "error"
                        job["error"] = f"Pause: {error}"
                        break
                    if self._job_abort.is_set():
                        job["state"] = "aborted"
                        break

                job["row"] = i + 1
                job["step_desc"] = step["desc"]

                error = self._run_step(step)
                if error:
                    job["state"] = "error"
                    job["error"] = f"Step {i + 1} ({step['desc']}): {error}"
                    break
            else:
                if self._job_abort.is_set():
                    job["state"] = "aborted"
                else:
                    job["state"] = "done"
        finally:
            self.control_enabled = False
            self._streaming_xy = False
            if self._log_auto:
                self.stop_log()
            if job["state"] in ("aborted", "error"):
                retract = float(self.config["retract_mm"])
                feed = float(self.config["retract_feedrate"])
                self._gcode(f"G91\nG1 Z{retract:.2f} F{feed:.0f}\nG90",
                            timeout=30.0)
            # Clear the Z offset accumulated by force control so back-to-back
            # jobs start from a clean coordinate frame. MOVE is omitted on
            # purpose: this must not move the axis.
            self._gcode("SET_GCODE_OFFSET Z=0", timeout=5.0)

    def clear_job(self):
        """Dismiss the record of a finished job (done/aborted/error) so the
        status panel returns to idle. Refused while the job is running."""
        if not self.job:
            return False, "No job to clear"
        if self.job.get("state") == "running":
            return False, "Job is still running — abort it first"
        self.job = None
        return True, "ok"

    def abort_job(self):
        """Disable Z-control immediately; the job thread retracts Z after the
        in-flight move segment completes."""
        self.control_enabled = False
        self._job_abort.set()
        if not (self.job and self.job.get("state") == "running"):
            return False
        return True
