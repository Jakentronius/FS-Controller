#!/usr/bin/env python3
"""Plot a dashboard run log as an XY tool path coloured by contact pressure,
and file it with the job and the log.

Reads a `logs/run_<job>_<timestamp>.csv` written by the FS Controller
dashboard, converts downforce to pressure over the full mechtrode face
(default 1.98 mm diameter) and draws:

  * the tool-centre XY path in machine coordinates, each segment coloured
    by the pressure at the moment the rod passed — a diverging scale
    centred on the force target (blue below, red above) by default, or a
    one-hue scale with --sequential; a thin centre-line by default, or
    --footprint to draw strokes one stepover wide so rows tile into the
    covered area;
  * pressure against run time, with the force target marked.

Rows logged before Klipper reported a position (X = Y = 0) are dropped.

Output goes to  <out-root>/output_<job stem>/  (default out-root: tests/):
    <job stem>_pressure_map.png
    <job file>          copy of the G-code that was run (if found)
    <run log>.csv       copy of the log

Usage:
  tools/plot_run.py logs/run_meander_..._180402.csv
  tools/plot_run.py RUNLOG --gcode jobs/meander_....gcode --diameter 1.98
Needs matplotlib + numpy.
"""

import argparse
import math
import os
import re
import shutil
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                       # noqa: E402
from matplotlib.collections import LineCollection    # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402

G_PER_N = 1.0 / 9.80665e-3          # grams per newton
# One-hue sequential ramp (light = low, dark = high)
RAMP = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95",
        "#0d366b"]
CMAP = LinearSegmentedColormap.from_list("fs_blue", RAMP)
# Diverging ramp centred on the force target: blue = below target,
# neutral grey = on target, red = above target.
DIV = ["#0d366b", "#256abf", "#6da7ec", "#b7d3f6", "#e4e3df", "#f2b39f",
       "#e0654a", "#b93a26", "#7a1d14"]
CMAP_DIV = LinearSegmentedColormap.from_list("fs_div", DIV)
INK, INK2, INK3, GRID, HI = "#1b2430", "#4a5566", "#7b8494", "#e9edf3", "#c8432b"


def read_log(path):
    """(header dict, structured array of the numeric rows)."""
    hdr, rows = {}, []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                k, _, v = line[1:].strip().partition(":")
                hdr[k.strip()] = v.strip()
                continue
            if line.startswith("Time"):
                continue
            p = line.strip().split(",")
            if len(p) >= 8:
                try:
                    rows.append([float(x) for x in p[:8]])
                except ValueError:
                    continue
    if not rows:
        raise SystemExit(f"{path}: no data rows")
    a = np.array(rows)
    return hdr, {"t": a[:, 0] - a[0, 0], "force": a[:, 2],
                 "target": a[:, 3], "x": a[:, 4], "y": a[:, 5],
                 "z": a[:, 6], "zoff": a[:, 7]}


def bin_samples(d, dt):
    """Average force over dt-second bins; position/time from the last
    row of each bin."""
    idx = np.floor(d["t"] / dt).astype(int)
    out = {k: [] for k in ("t", "force", "target", "x", "y", "z")}
    for b in np.unique(idx):
        sel = np.nonzero(idx == b)[0]
        last = sel[-1]
        out["t"].append(d["t"][last])
        out["force"].append(d["force"][sel].mean())
        for k in ("target", "x", "y", "z"):
            out[k].append(d[k][last])
    return {k: np.array(v) for k, v in out.items()}


def find_job_file(job_name, explicit, log_path):
    if explicit:
        return explicit if os.path.exists(explicit) else None
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)
    for cand in (os.path.join(os.path.dirname(log_path), "..", "jobs", job_name),
                 os.path.join(root, "jobs", job_name),
                 os.path.join(root, "tools", job_name),
                 os.path.join(root, "tests", job_name),
                 job_name):
        if os.path.exists(cand):
            return cand
    return None


def job_stepover(job_path):
    """Stepover from a hatch_gcode.py header comment, else None."""
    if not job_path:
        return None
    try:
        with open(job_path) as f:
            for line in f:
                if line.startswith(";") and "stepover" in line:
                    m = re.search(r"stepover\s+([\d.]+)\s*mm", line)
                    if m:
                        return float(m.group(1))
    except OSError:
        pass
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("log", help="run log CSV from the dashboard")
    ap.add_argument("--gcode", help="job file that was run (default: "
                    "look for the '# job:' name next to the log / in jobs/)")
    ap.add_argument("--diameter", type=float, default=1.98,
                    help="mechtrode diameter, mm (default 1.98)")
    ap.add_argument("--stepover", type=float, default=None,
                    help="with --footprint: stroke width, mm (default: from "
                    "the job header, else 0.5)")
    ap.add_argument("--footprint", action="store_true",
                    help="draw the path one stepover wide so the rows tile "
                    "into the covered area (default: a thin centre-line)")
    ap.add_argument("--line-pt", type=float, default=2.0,
                    help="centre-line width in points (default 2)")
    ap.add_argument("--sequential", action="store_true",
                    help="one-hue scale from --vmin instead of the diverging "
                    "scale centred on the force target")
    ap.add_argument("--span", type=float, default=None,
                    help="diverging scale half-range about the target, MPa "
                    "(default: covers the engaged samples, to 0.25)")
    ap.add_argument("--bin", type=float, default=0.25,
                    help="averaging bin, s (default 0.25)")
    ap.add_argument("--out-root", default=None,
                    help="folder that holds output_<job> (default tests/ "
                    "beside tools/)")
    ap.add_argument("--vmin", type=float, default=None,
                    help="bottom of the colour scale, MPa (default: just "
                    "below the 2nd percentile of engaged pressure, so the "
                    "scale spends itself on the variation that exists)")
    ap.add_argument("--dpi", type=int, default=160)
    args = ap.parse_args()

    hdr, raw = read_log(args.log)
    job_name = hdr.get("job", os.path.splitext(os.path.basename(args.log))[0])
    stem = os.path.splitext(job_name)[0]
    job_path = find_job_file(job_name, args.gcode, args.log)
    stepover = args.stepover or job_stepover(job_path) or 0.5
    area = math.pi * (args.diameter / 2.0) ** 2          # mm^2
    to_mpa = 1.0 / (G_PER_N * area)                       # g -> N/mm^2

    # Drop pre-telemetry rows (position not yet reported: X = Y = 0)
    keep = ~((raw["x"] == 0.0) & (raw["y"] == 0.0))
    dropped = int((~keep).sum())
    d = {k: v[keep] for k, v in raw.items()}
    s = bin_samples(d, args.bin)
    p = s["force"] * to_mpa
    target_g = float(s["target"].max()) if s["target"].max() > 0 else None
    target_p = target_g * to_mpa if target_g else None
    eng = s["force"] > (0.1 * target_g if target_g else 50.0)
    vmax = max(p.max(), (target_p or 0) * 1.25)
    vmax = math.ceil(vmax * 2) / 2.0                      # to 0.5 MPa
    if args.vmin is not None:
        vmin = args.vmin
    elif eng.any():
        vmin = math.floor(np.percentile(p[eng], 2) * 2) / 2.0
    else:
        vmin = 0.0
    vmin = max(0.0, min(vmin, vmax - 0.5))
    diverging = bool(target_p) and not args.sequential
    if diverging:
        if args.span:
            half = args.span
        else:
            dev = np.abs(p[eng] - target_p) if eng.any() else np.abs(p - target_p)
            half = math.ceil(np.percentile(dev, 99) * 4) / 4.0
        half = max(half, 0.25)
        norm = plt.Normalize(target_p - half, target_p + half)
        cmap = CMAP_DIV
    else:
        norm = plt.Normalize(vmin, vmax)
        cmap = CMAP

    root = args.out_root or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tests")
    out_dir = os.path.join(root, f"output_{stem}")
    os.makedirs(out_dir, exist_ok=True)

    # ---------------------------------------------------------------- figure
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9,
                         "axes.edgecolor": INK3, "axes.labelcolor": INK2,
                         "xtick.color": INK3, "ytick.color": INK3,
                         "text.color": INK})
    fig = plt.figure(figsize=(11, 6.6), facecolor="white")
    gs = fig.add_gridspec(2, 2, width_ratios=[1.05, 1], height_ratios=[1, 1],
                          left=0.06, right=0.97, top=0.86, bottom=0.09,
                          wspace=0.34, hspace=0.42)
    ax = fig.add_subplot(gs[:, 0])
    ax_t = fig.add_subplot(gs[0, 1])
    ax_z = fig.add_subplot(gs[1, 1])

    # Path coloured by pressure; stroke one stepover wide
    pts = np.column_stack([s["x"], s["y"]])
    segs = np.stack([pts[:-1], pts[1:]], axis=1)
    moving = np.linalg.norm(segs[:, 1] - segs[:, 0], axis=1) > 1e-6
    x0, x1 = s["x"].min() - 1.5, s["x"].max() + 1.5
    y0, y1 = s["y"].min() - 1.5, s["y"].max() + 1.5
    span = max(x1 - x0, y1 - y0)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    ax.set_xlim(cx - span / 2, cx + span / 2)
    ax.set_ylim(cy - span / 2, cy + span / 2)
    ax.set_aspect("equal")
    lc = LineCollection(segs[moving], cmap=cmap, linewidths=args.line_pt,
                        capstyle="round", norm=norm)
    lc.set_array(p[1:][moving])
    ax.add_collection(lc)
    cb = fig.colorbar(lc, ax=ax, fraction=0.046, pad=0.03)
    if args.footprint:
        fig.canvas.draw()      # final axes geometry, colorbar included
        px_per_mm = (ax.transData.transform((1, 0))
                     - ax.transData.transform((0, 0)))[0]
        lc.set_linewidths(stepover * px_per_mm * 72.0 / fig.dpi)
    # engagement point: rod footprint at true diameter
    if eng.any():
        i0 = int(np.argmax(eng))
        ax.add_patch(plt.Circle((s["x"][i0], s["y"][i0]), args.diameter / 2,
                                fill=False, ec=INK, lw=1.0, zorder=5))
        ax.annotate("engage", (s["x"][i0], s["y"][i0]),
                    xytext=(args.diameter / 2 + 0.6, 0.4),
                    textcoords="offset fontsize", fontsize=8, color=INK2)
        i1 = len(s["x"]) - 1 - int(np.argmax(eng[::-1]))
        ax.plot(s["x"][i1], s["y"][i1], marker="s", ms=4, color=INK,
                mfc="none", zorder=5)
        ax.annotate("lift", (s["x"][i1], s["y"][i1]), xytext=(0.5, 0.4),
                    textcoords="offset fontsize", fontsize=8, color=INK2)
    ax.grid(True, color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    ax.set_xlabel("X (mm, machine)")
    ax.set_ylabel("Y (mm, machine)")
    ax.set_title("Tool centre path coloured by contact pressure"
                 + (" (blue below target, red above)" if diverging else ""),
                 loc="left", fontsize=10, color=INK, pad=8)
    if diverging:
        cb.set_label(f"pressure over rod face (MPa)\n"
                     f"centred on the {target_g:g} g target")
    else:
        cb.set_label("pressure over rod face (MPa)"
                     + (f", scale from {vmin:g}" if vmin > 0 else ""))
    cb.outline.set_edgecolor(INK3)
    if target_p:
        cb.ax.axhline(target_p, color=INK, lw=0.8)
        cb.ax.text(-0.35, target_p, f"{target_g:g} g ", color=INK, fontsize=8,
                   va="center", ha="right",
                   transform=cb.ax.get_yaxis_transform())

    # Pressure vs time
    ax_t.fill_between(s["t"], 0, p, color=RAMP[4], alpha=0.12, lw=0)
    ax_t.plot(s["t"], p, color=RAMP[4], lw=1.4)
    if target_p:
        ax_t.axhline(target_p, color=HI, lw=1, ls=(0, (4, 3)))
        ax_t.text(2, target_p, f"{target_g:g} g = {target_p:.2f} MPa",
                  color=HI, fontsize=8, va="bottom", ha="left")
    ax_t.set_ylim(0, vmax)
    ax_t.set_xlim(0, s["t"][-1])
    ax_t.set_ylabel("MPa")
    ax_t.set_title("Contact pressure over the run", loc="left", fontsize=10,
                   pad=6)
    ax_t.grid(True, color=GRID, lw=0.6)
    ax_t.set_axisbelow(True)

    # Machine Z vs time (rod consumption + mesh following show up here)
    ax_z.plot(s["t"], s["z"], color=INK2, lw=1.2)
    ax_z.set_xlim(0, s["t"][-1])
    zz = s["z"][eng] if eng.any() else s["z"]
    pad = max(0.05, 0.15 * (zz.max() - zz.min()))
    ax_z.set_ylim(zz.min() - pad, zz.max() + pad)
    ax_z.set_ylabel("Z (mm, machine)")
    ax_z.set_xlabel("run time (s)")
    ax_z.set_title("Head height while engaged (up positive)", loc="left",
                   fontsize=10, pad=6)
    ax_z.grid(True, color=GRID, lw=0.6)
    ax_z.set_axisbelow(True)
    for a in (ax, ax_t, ax_z):
        for sp in ("top", "right"):
            a.spines[sp].set_visible(False)

    # Header
    t_eng = (s["t"][eng][-1] - s["t"][eng][0]) if eng.any() else 0.0
    fig.text(0.06, 0.955, stem, fontsize=13, fontweight="bold", color=INK)
    sub1 = (f"run {hdr.get('started', '?')}   ·   mechtrode Ø{args.diameter:g} mm "
            f"(face {area:.3f} mm²)"
            + (f"   ·   target {target_g:g} g = {target_p:.2f} MPa" if target_p
               else ""))
    sub2 = (f"mean while engaged {p[eng].mean():.2f} MPa "
            f"({s['force'][eng].mean():.0f} g)   ·   peak {p.max():.2f} MPa   ·   "
            f"engaged {t_eng:.0f} s of {s['t'][-1]:.0f} s")
    fig.text(0.06, 0.922, sub1, fontsize=8.5, color=INK2)
    fig.text(0.06, 0.895, sub2, fontsize=8.5, color=INK2)
    fig.text(0.06, 0.03,
             (f"stroke width = {stepover:g} mm stepover · " if args.footprint
              else f"{args.line_pt:g} pt centre-line · ")
             + f"{dropped} pre-telemetry "
             f"rows at X=Y=0 dropped · {args.bin:g} s bins · units "
             f"{hdr.get('units', 'g')}, {hdr.get('counts_per_unit', '?')} "
             "counts/unit", fontsize=7.5, color=INK3)

    png = os.path.join(out_dir, f"{stem}_pressure_map.png")
    fig.savefig(png, dpi=args.dpi, facecolor="white")

    # File the inputs beside the plot
    shutil.copy2(args.log, os.path.join(out_dir, os.path.basename(args.log)))
    if job_path:
        shutil.copy2(job_path, os.path.join(out_dir, job_name))
    print(f"wrote {png}")
    print(f"  log  -> {os.path.join(out_dir, os.path.basename(args.log))}")
    print(f"  job  -> {os.path.join(out_dir, job_name) if job_path else 'NOT FOUND (pass --gcode)'}")
    print(f"  {len(s['t'])} samples, {dropped} zero-position rows dropped, "
          f"mean engaged {p[eng].mean():.3f} MPa, peak {p.max():.3f} MPa")


if __name__ == "__main__":
    main()
