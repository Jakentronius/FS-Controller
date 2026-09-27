#!/usr/bin/env python3
"""Generate meander / crosshatch friction-surfacing jobs for the FS
Controller dashboard (annotated G-code, surface-referenced Z).

A rectangular patch of LENGTH x WIDTH mm is covered by one or more
hatch layers. Each layer is a single serpentine (meander) of parallel
lines at STEPOVER pitch; the tool stays engaged through the 180-degree
turnarounds. Crosshatching stacks several layers, rotating the hatch
direction between them (--rotate 90 -> 0/90/0/90, --rotate 45 ->
0/45/90/135, or an explicit --angles list). Between layers the tool
lifts, travels to the next start and re-engages (--layer-link lift), or
drags engaged to the next start without lifting (--layer-link engaged).

Two modes, chosen by --force:
  * force > 0  -> force-controlled: the dashboard PID holds the downforce
    (;FORCE=, ;WAIT_FORCE engagement, induction dwell).
  * force == 0 -> position mode: ;CONTROL=OFF suspends the PID, the rod
    plunges to --start-z and every XY move carries a Z word so the head
    descends at a fixed ratio of --xy-per-z mm of XY travel per 1 mm of
    Z (compensating rod consumption open-loop), clamped at --max-plunge.
    The dashboard's max_force fault still guards the run.

Machine conventions (FS Controller, Sept 2026 frame):
  * ;ZREF=SURFACE  - every Z is relative to the last Touch Off, up is
    positive. The dashboard's z_sign flips Z at the Klipper boundary
    (Klipper +Z is DOWN on this rig), so nothing here needs to know.
  * ;FORCE=<units> - the calibrated load-cell unit is GRAMS on this rig
    (see tools/calibration_*.csv), so --force is in grams.
  * X0 Y0 is the dashboard work origin (Zero XY Here).
  * Klipper max_velocity is 10 mm/s: feeds above 600 mm/min are capped
    by the firmware, so the script refuses them unless --allow-fast.

Examples
  # 40 x 20 mm meander, 2 mm stepover, 200 mm/min, 400 g
  hatch_gcode.py -L 40 -W 20 -s 2 -f 200 -F 400

  # 3-layer crosshatch rotating 60 deg per layer (0/60/120)
  hatch_gcode.py -L 30 -W 30 -s 1.5 -f 150 -F 500 --layers 3 --rotate 60

  # 0/90 crosshatch, 4 layers, explicit angles, SVG preview
  hatch_gcode.py -L 30 -W 30 -s 2 -f 200 -F 500 --angles 0,90,0,90 \
      --svg preview.svg -o patch.gcode

  # position mode: no force control, 1 mm of plunge per 150 mm of XY
  hatch_gcode.py -L 30 -W 20 -s 2 -f 150 -F 0 --xy-per-z 150 --max-plunge 4

Standard library only.
"""

import argparse
import math
import sys
import time

MAX_FEED = 600.0        # mm/min, Klipper max_velocity 10 mm/s
MIN_SPAN = 0.05         # drop hatch lines shorter than this (mm)


# --------------------------------------------------------------------------
# Geometry
# --------------------------------------------------------------------------

def rect_corners(length, width, origin, inset=0.0):
    """Axis-aligned rectangle (length along X, width along Y) in job
    coords, eroded by `inset` on every side. origin='center' puts X0 Y0
    at the middle of the full patch, 'corner' at its min-X/min-Y
    corner."""
    if origin == "center":
        x0, y0 = -length / 2.0, -width / 2.0
    else:
        x0, y0 = 0.0, 0.0
    x0, y0 = x0 + inset, y0 + inset
    length, width = length - 2 * inset, width - 2 * inset
    return [(x0, y0), (x0 + length, y0), (x0 + length, y0 + width),
            (x0, y0 + width)]


def _rot(pt, ang):
    c, s = math.cos(ang), math.sin(ang)
    return (pt[0] * c + pt[1] * s, -pt[0] * s + pt[1] * c)


def _unrot(pt, ang):
    return _rot(pt, -ang)


def _convex_span(poly, y):
    """[xmin, xmax] of the intersection of a convex polygon with the
    horizontal line at y, or None if it misses."""
    xs = []
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 <= y < y2) or (y2 <= y < y1):
            xs.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
    if len(xs) < 2:
        return None
    return min(xs), max(xs)


def hatch_rows(corners, stepover, angle_deg, min_span=MIN_SPAN):
    """Parallel hatch lines across a convex polygon at `angle_deg`
    (degrees from +X) and `stepover` pitch. Returns [(p_start, p_end)]
    in job coords, ordered across the band; each row runs in the +hatch
    direction. The comb is centred in the band so the first and last
    lines sit half a leftover in from the edges. Rows shorter than
    `min_span` (the slivers a diagonal hatch leaves in the corners)
    are dropped."""
    ang = math.radians(angle_deg)
    poly = [_rot(p, ang) for p in corners]
    ys = [p[1] for p in poly]
    lo, hi = min(ys), max(ys)
    band = hi - lo
    if band <= 0 or stepover <= 0:
        return []
    count = max(1, int(band / stepover + 1e-9) + 1)
    start = lo + (band - (count - 1) * stepover) / 2.0
    eps = 1e-6 * max(1.0, band)
    rows = []
    for k in range(count):
        y = min(max(start + k * stepover, lo + eps), hi - eps)
        span = _convex_span(poly, y)
        if span is None:
            continue
        a, b = span
        if b - a < min_span:
            continue
        rows.append((_unrot((a, y), ang), _unrot((b, y), ang)))
    return rows


def serpentine(rows, reverse_rows=False, start_at_end=False):
    """Chain hatch rows into one engaged meander: alternate the line
    direction row by row so each end-of-line link is the short hop to
    the next row (inside the rectangle, since it is convex)."""
    rows = list(reversed(rows)) if reverse_rows else list(rows)
    pts = []
    flip = start_at_end
    for a, b in rows:
        pts.extend((b, a) if flip else (a, b))
        flip = not flip
    return pts


CORNERS = ("sw", "se", "ne", "nw")   # index into rect_corners() order


def corner_point(corners, name):
    """The rectangle corner named sw/se/ne/nw (S = min Y, W = min X)."""
    return corners[CORNERS.index(name)]


def best_serpentine(rows, near=None):
    """Of the four ways to run the meander (row order x first-line
    direction), the one starting nearest `near` (or the first, when no
    preference is given)."""
    options = [serpentine(rows, rr, se)
               for rr in (False, True) for se in (False, True)]
    options = [o for o in options if o]
    if not options:
        return []
    if near is None:
        return options[0]
    return min(options, key=lambda o: math.hypot(o[0][0] - near[0],
                                                 o[0][1] - near[1]))


def densify(points, max_mm):
    """Insert intermediate points so no emitted G1 exceeds max_mm."""
    if max_mm <= 0 or len(points) < 2:
        return list(points)
    out = [points[0]]
    for a, b in zip(points, points[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(math.ceil(d / max_mm - 1e-9)))
        for i in range(1, n + 1):
            t = i / n
            out.append((a[0] + (b[0] - a[0]) * t,
                        a[1] + (b[1] - a[1]) * t))
    return out


def path_len(points):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(points, points[1:]))


def layer_angles(args):
    if args.angles:
        return [float(a) for a in args.angles.split(",") if a.strip()]
    return [(args.start_angle + i * args.rotate) % 180.0
            for i in range(args.layers)]


def build_layers(args):
    """[(angle, [points...]), ...] — one engaged meander per layer. The
    first layer starts at the patch corner named by --start-corner;
    later layers start nearest the previous layer's end (so same-angle
    repeats reverse in place, still engaged) or, with --layer-start
    same, all start from that corner again."""
    corners = rect_corners(args.length, args.width, args.origin, args.inset)
    home = corner_point(corners, args.start_corner)
    layers = []
    prev_end = None
    for angle in layer_angles(args):
        rows = hatch_rows(corners, args.stepover, angle,
                          max(MIN_SPAN, args.min_line))
        near = home if (prev_end is None or args.layer_start == "same") \
            else prev_end
        pts = best_serpentine(rows, near)
        if not pts:
            continue
        layers.append((angle, pts))
        prev_end = pts[-1]
    return layers


# --------------------------------------------------------------------------
# G-code
# --------------------------------------------------------------------------

def _xy(pt, feed):
    return f"G1 X{pt[0]:.3f} Y{pt[1]:.3f} F{feed:.0f}"


def _xyz(pt, z, feed):
    return f"G1 X{pt[0]:.3f} Y{pt[1]:.3f} Z{z:.4f} F{feed:.0f}"


def _contiguous(a, b, tol=1e-6):
    return a is not None and math.hypot(a[0] - b[0], a[1] - b[1]) <= tol


def position_mode(args):
    return args.force <= 0


class _Plunger:
    """Open-loop Z ramp for position mode: Z drops by 1/xy_per_z per mm
    of XY travel from start_z, never below start_z - max_plunge."""

    def __init__(self, args):
        self.z = args.start_z
        self.floor = (args.start_z - args.max_plunge if args.max_plunge > 0
                      else -float("inf"))
        self.rate = 1.0 / args.xy_per_z if args.xy_per_z > 0 else 0.0

    def advance(self, dist):
        self.z = max(self.floor, self.z - dist * self.rate)
        return self.z


def _header(args, layers):
    angles = ", ".join(f"{a:g}" for a, _ in layers)
    kind = "crosshatch" if len(layers) > 1 else "meander"
    mode = ("position mode (no force control)" if position_mode(args)
            else "force-controlled")
    lines = [
        f"; FS {kind} patch — generated by tools/hatch_gcode.py "
        f"{time.strftime('%Y-%m-%d %H:%M:%S')}",
        f"; patch {args.length:g} x {args.width:g} mm, origin at "
        f"{'patch centre' if args.origin == 'center' else 'min corner'}",
        f"; stepover {args.stepover:g} mm, inset {args.inset:g} mm, "
        f"{len(layers)} layer(s) at {angles} deg",
        f"; feed {args.feed:g} mm/min, {mode}, induction "
        f"{args.induction:g} s, layer link: {args.layer_link}",
        f"; first meander starts at the {args.start_corner.upper()} corner"
        + (", every layer restarts there" if args.layer_start == "same"
           else ", later layers chain from the previous end"),
    ]
    if position_mode(args):
        lines.append(f"; plunge ratio {args.xy_per_z:g} mm XY per 1 mm Z, "
                     f"start Z {args.start_z:g}, max plunge "
                     f"{args.max_plunge:g} mm (0 = unlimited)")
    else:
        lines.append(f"; force {args.force:g} g")
    lines += [
        "; run with the work origin (Zero XY Here) at the patch's "
        + ("centre" if args.origin == "center" else "min-X/min-Y corner"),
        ";ZREF=SURFACE",
        "; all Z coordinates are relative to the last touched-off",
        "; surface (Z0 = surface, up positive) — Touch Off before running",
    ]
    return lines


def _generate_force(args, layers):
    lines = _header(args, layers)
    prev_end = None
    engagements = 0
    for n, (angle, pts) in enumerate(layers, start=1):
        run = densify(pts, args.segment)
        lines.append("")
        lines.append(f"; --- layer {n}/{len(layers)}: hatch {angle:g} deg, "
                     f"{len(pts) // 2} lines, {path_len(pts):.1f} mm ---")

        if _contiguous(prev_end, run[0]):
            # Same-angle repeat: reverse in place, still engaged.
            lines.append("; (starts where the last layer ended — "
                         "reversing in place, still engaged)")
            for pt in run[1:]:
                lines.append(_xy(pt, args.feed))
            prev_end = run[-1]
            continue

        if prev_end is not None and args.layer_link == "engaged":
            # Drag to the next start under force, no lift.
            lines.append("; (engaged link to the next layer's start)")
            lines.append(_xy(run[0], args.feed))
            for pt in run[1:]:
                lines.append(_xy(pt, args.feed))
            prev_end = run[-1]
            continue

        lines.append(";FORCE=0")
        if prev_end is None:
            lines.append(f"G1 Z{args.travel_z:.3f} F{args.travel_feed:.0f}")
            lines.append(_xy(run[0], args.travel_feed))
            lines.append(f"G1 Z{args.approach_z:.3f} "
                         f"F{args.plunge_feed:.0f}")
        else:
            lines.append("G91")
            lines.append(f"G1 Z{args.layer_lift:.3f} F{args.plunge_feed:.0f}")
            lines.append("G90")
            lines.append(_xy(run[0], args.travel_feed))
            lines.append(f"G1 Z{args.approach_z:.3f} "
                         f"F{args.plunge_feed:.0f}")
        lines.append(f";FORCE={args.force:g}")
        lines.append(f";WAIT_FORCE TIMEOUT={args.wait_timeout:g}")
        if args.induction > 0 and (prev_end is None or args.reinduct):
            lines.append(f";DWELL={args.induction:g}")
        engagements += 1
        for pt in run[1:]:
            lines.append(_xy(pt, args.feed))
        prev_end = run[-1]

    lines.append("")
    if args.end_dwell > 0:
        lines.append(f";DWELL={args.end_dwell:g}")
    lines.append(";FORCE=0")
    lines.append("G91")
    lines.append(f"G1 Z{args.lift:.3f} F{args.travel_feed:.0f}")
    lines.append("G90")
    lines.append(f"G1 Z{args.travel_z:.3f} F{args.travel_feed:.0f}")
    lines.append("")
    return "\n".join(lines), engagements


def _generate_position(args, layers):
    """Position mode: PID off, Z words ramp the plunge with XY travel.
    Mixed XYZ moves are sent to Klipper verbatim (not sliced), so the
    segment length also sets the pause/abort granularity."""
    lines = _header(args, layers)
    lines.append(";FORCE=0")
    lines.append(";CONTROL=OFF")
    plunger = _Plunger(args)
    prev_end = None
    engagements = 0

    def ramp(run, start_idx=1):
        for a, b in zip(run[start_idx - 1:], run[start_idx:]):
            z = plunger.advance(math.hypot(b[0] - a[0], b[1] - a[1]))
            lines.append(_xyz(b, z, args.feed))

    for n, (angle, pts) in enumerate(layers, start=1):
        run = densify(pts, args.segment)
        lines.append("")
        lines.append(f"; --- layer {n}/{len(layers)}: hatch {angle:g} deg, "
                     f"{len(pts) // 2} lines, {path_len(pts):.1f} mm, "
                     f"Z {plunger.z:.3f} at start ---")

        if _contiguous(prev_end, run[0]):
            lines.append("; (starts where the last layer ended — "
                         "reversing in place, still engaged)")
            ramp(run)
            prev_end = run[-1]
            continue

        if prev_end is not None and args.layer_link == "engaged":
            lines.append("; (engaged link to the next layer's start)")
            ramp([prev_end] + run)
            prev_end = run[-1]
            continue

        if prev_end is None:
            lines.append(f"G1 Z{args.travel_z:.3f} F{args.travel_feed:.0f}")
            lines.append(_xy(run[0], args.travel_feed))
            lines.append(f"G1 Z{args.approach_z:.3f} "
                         f"F{args.plunge_feed:.0f}")
        else:
            lines.append("G91")
            lines.append(f"G1 Z{args.layer_lift:.3f} F{args.plunge_feed:.0f}")
            lines.append("G90")
            lines.append(_xy(run[0], args.travel_feed))
            lines.append(f"G1 Z{plunger.z + args.approach_z:.4f} "
                         f"F{args.plunge_feed:.0f}")
        # Commanded plunge to the current ramp depth (no force wait).
        lines.append(f"G1 Z{plunger.z:.4f} F{args.engage_feed:.0f}")
        if args.induction > 0 and (prev_end is None or args.reinduct):
            lines.append(f";DWELL={args.induction:g}")
        engagements += 1
        ramp(run)
        prev_end = run[-1]

    lines.append("")
    lines.append(f"; final Z {plunger.z:.3f} (plunged "
                 f"{args.start_z - plunger.z:.3f} mm below start)")
    if args.end_dwell > 0:
        lines.append(f";DWELL={args.end_dwell:g}")
    lines.append("G91")
    lines.append(f"G1 Z{args.lift:.3f} F{args.travel_feed:.0f}")
    lines.append("G90")
    lines.append(f"G1 Z{args.travel_z:.3f} F{args.travel_feed:.0f}")
    lines.append(";CONTROL=ON")
    lines.append("")
    return "\n".join(lines), engagements, plunger.z


def generate(args, layers):
    """(gcode_text, engagements, final_z) — final_z is None in force mode."""
    if position_mode(args):
        return _generate_position(args, layers)
    text, eng = _generate_force(args, layers)
    return text, eng, None


def estimates(args, layers, engagements):
    deposited = sum(path_len(p) for _, p in layers)
    travel = 0.0
    prev_end = None
    for _, p in layers:
        if prev_end is not None:
            travel += math.hypot(p[0][0] - prev_end[0], p[0][1] - prev_end[1])
        prev_end = p[-1]
    t = deposited / max(args.feed, 1.0) * 60.0
    if args.layer_link == "engaged":
        t += travel / max(args.feed, 1.0) * 60.0
    else:
        t += travel / max(args.travel_feed, 1.0) * 60.0
    t += engagements * (args.induction if args.reinduct else 0.0)
    if engagements and not args.reinduct:
        t += args.induction
    t += engagements * 8.0 + args.end_dwell     # engage/lift overhead guess
    plunge = None
    if position_mode(args):
        total = deposited + (travel if args.layer_link == "engaged" else 0.0)
        plunge = total / args.xy_per_z if args.xy_per_z > 0 else 0.0
        if args.max_plunge > 0:
            plunge = min(plunge, args.max_plunge)
    return {"mode": "position" if position_mode(args) else "force",
            "plunge_mm": plunge,
            "layers": len(layers),
            "lines": sum(len(p) // 2 for _, p in layers),
            "deposited_mm": deposited,
            "travel_mm": travel,
            "engagements": engagements,
            "time_s": t}


# --------------------------------------------------------------------------
# SVG preview
# --------------------------------------------------------------------------

def svg_preview(args, layers):
    corners = rect_corners(args.length, args.width, args.origin)
    eroded = rect_corners(args.length, args.width, args.origin, args.inset)
    xs = [c[0] for c in corners]
    ys = [c[1] for c in corners]
    pad = max(2.0, args.stepover)
    minx, maxx = min(xs) - pad, max(xs) + pad
    miny, maxy = min(ys) - pad, max(ys) + pad
    w, h = maxx - minx, maxy - miny
    scale = 600.0 / max(w, h)
    palette = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd",
               "#8c564b", "#17becf", "#e377c2"]

    def sx(x):
        return (x - minx) * scale

    def sy(y):
        return (maxy - y) * scale          # +Y up on screen

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w * scale:.0f}" '
           f'height="{h * scale:.0f}" viewBox="0 0 {w * scale:.0f} '
           f'{h * scale:.0f}">',
           '<rect width="100%" height="100%" fill="white"/>',
           '<polygon points="'
           + " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in corners)
           + '" fill="none" stroke="#999" stroke-dasharray="4 3"/>']
    if args.inset > 0:
        out.append('<polygon points="'
                   + " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in eroded)
                   + '" fill="none" stroke="#ccc" stroke-dasharray="2 3"/>')
    for i, (angle, pts) in enumerate(layers):
        col = palette[i % len(palette)]
        d = " ".join(f"{'M' if k == 0 else 'L'}{sx(x):.1f},{sy(y):.1f}"
                     for k, (x, y) in enumerate(pts))
        out.append(f'<path d="{d}" fill="none" stroke="{col}" '
                   f'stroke-width="1.2" opacity="0.85"/>')
        out.append(f'<circle cx="{sx(pts[0][0]):.1f}" cy="{sy(pts[0][1]):.1f}" '
                   f'r="4" fill="{col}"/>')
        out.append(f'<text x="8" y="{16 + 16 * i}" font-size="13" '
                   f'font-family="sans-serif" fill="{col}">layer {i + 1}: '
                   f'{angle:g} deg (dot = start)</text>')
    # origin marker
    out.append(f'<line x1="{sx(-1):.1f}" y1="{sy(0):.1f}" x2="{sx(1):.1f}" '
               f'y2="{sy(0):.1f}" stroke="black"/>')
    out.append(f'<line x1="{sx(0):.1f}" y1="{sy(-1):.1f}" x2="{sx(0):.1f}" '
               f'y2="{sy(1):.1f}" stroke="black"/>')
    out.append("</svg>")
    return "\n".join(out)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = p.add_argument_group("patch")
    g.add_argument("-L", "--length", type=float, required=True,
                   help="patch length along X (mm)")
    g.add_argument("-W", "--width", type=float, required=True,
                   help="patch width along Y (mm)")
    g.add_argument("-s", "--stepover", type=float, required=True,
                   help="pitch between hatch lines (mm)")
    g.add_argument("--inset", type=float, default=0.0,
                   help="pull every line end / the outer lines in from the "
                        "rectangle edge (mm) so the bead stays inside")
    g.add_argument("--min-line", type=float, default=None,
                   help="drop hatch lines shorter than this (mm); default "
                        "half the stepover, the corner slivers of a "
                        "diagonal hatch")
    g.add_argument("--origin", choices=("center", "corner"), default="center",
                   help="where X0 Y0 sits on the patch (default center)")

    g = p.add_argument_group("layers")
    g.add_argument("-n", "--layers", type=int, default=1,
                   help="number of hatch layers (1 = plain meander)")
    g.add_argument("--rotate", type=float, default=90.0,
                   help="hatch rotation added per layer, degrees "
                        "(90 -> 0/90/0/90, 45 -> 0/45/90/135; default 90)")
    g.add_argument("--start-angle", type=float, default=0.0,
                   help="first layer's hatch direction, degrees from +X")
    g.add_argument("--angles",
                   help="explicit per-layer angle list, e.g. 0,90,45,135 "
                        "(overrides --layers/--rotate/--start-angle)")
    g.add_argument("--start-corner", choices=CORNERS, default="sw",
                   help="patch corner the first meander starts from "
                        "(S = min Y, W = min X; default sw)")
    g.add_argument("--layer-start", choices=("nearest", "same"),
                   default="nearest",
                   help="later layers start nearest the previous layer's "
                        "end (default) or from --start-corner every time")
    g.add_argument("--layer-link", choices=("lift", "engaged"),
                   default="lift",
                   help="between layers: lift/travel/re-engage (default) "
                        "or drag engaged to the next start")
    g.add_argument("--reinduct", action="store_true",
                   help="repeat the induction dwell at every re-engagement, "
                        "not just the first touch-down")

    g = p.add_argument_group("process")
    g.add_argument("-f", "--feed", type=float, required=True,
                   help="traverse feed while depositing (mm/min)")
    g.add_argument("-F", "--force", type=float, required=True,
                   help="downforce target (grams — the rig's calibrated "
                        "unit); 0 = position mode (see below)")
    g.add_argument("--induction", type=float, default=5.0,
                   help="dwell after touch-down before moving (s, default 5)")
    g.add_argument("--end-dwell", type=float, default=0.0,
                   help="dwell at force at the very end (s)")

    g = p.add_argument_group("position mode (only when --force 0)")
    g.add_argument("--xy-per-z", type=float, default=100.0,
                   help="mm of XY travel per 1 mm of Z plunge "
                        "(default 100); 0 = constant Z")
    g.add_argument("--start-z", type=float, default=0.0,
                   help="Z at which deposition starts, surface-relative "
                        "(default 0 = the touched-off surface; negative "
                        "= below it)")
    g.add_argument("--max-plunge", type=float, default=0.0,
                   help="stop descending after this much plunge below "
                        "start-z (mm); 0 = no limit")
    g.add_argument("--engage-feed", type=float, default=60.0,
                   help="Z feed for the commanded plunge from approach-z "
                        "to the ramp depth (mm/min, default 60)")

    g = p.add_argument_group("heights and travel (surface-relative, up +)")
    g.add_argument("--travel-z", type=float, default=10.0,
                   help="clearance height for travel (mm, default 10)")
    g.add_argument("--approach-z", type=float, default=1.0,
                   help="park height before each engagement (mm, default 1; "
                        "guarded descent gives up after 3 mm)")
    g.add_argument("--layer-lift", type=float, default=1.0,
                   help="relative lift between layers (mm, default 1)")
    g.add_argument("--lift", type=float, default=5.0,
                   help="relative lift-off at the end (mm, default 5)")
    g.add_argument("--travel-feed", type=float, default=600.0,
                   help="XY/Z travel feed (mm/min, default 600)")
    g.add_argument("--plunge-feed", type=float, default=300.0,
                   help="Z approach feed (mm/min, default 300)")
    g.add_argument("--wait-timeout", type=float, default=30.0,
                   help=";WAIT_FORCE timeout per engagement (s, default 30)")
    g.add_argument("--segment", type=float, default=None,
                   help="max emitted G1 length (mm; default 10, or 2 in "
                        "position mode where XYZ moves are not sliced "
                        "by the runner; 0 = none)")
    g.add_argument("--allow-fast", action="store_true",
                   help="permit feeds above Klipper's 600 mm/min ceiling")

    g = p.add_argument_group("output")
    g.add_argument("-o", "--output",
                   help="output .gcode path (default: auto-named in cwd; "
                        "'-' for stdout)")
    g.add_argument("--svg", help="also write an SVG preview of the toolpath")
    a = p.parse_args(argv)

    problems = []
    if a.length <= 0 or a.width <= 0:
        problems.append("length and width must be positive")
    if a.stepover <= 0:
        problems.append("stepover must be positive")
    if a.stepover > max(a.length, a.width):
        problems.append("stepover is larger than the patch")
    if a.inset < 0 or 2 * a.inset >= min(a.length, a.width):
        problems.append("inset must be >= 0 and less than half the "
                        "smaller patch dimension")
    if a.layers < 1:
        problems.append("layers must be >= 1")
    if a.feed <= 0 or a.force < 0:
        problems.append("feed must be positive and force >= 0")
    if a.approach_z <= 0 or a.approach_z > 3.0:
        problems.append("approach-z must be in (0, 3] mm: the runner's "
                        "guarded descent stops after 3 mm without contact")
    if a.travel_z <= a.approach_z:
        problems.append("travel-z must be above approach-z")
    if a.force <= 0:
        if a.xy_per_z < 0:
            problems.append("xy-per-z must be >= 0")
        if a.max_plunge < 0:
            problems.append("max-plunge must be >= 0")
        if a.start_z > 0:
            problems.append("start-z must be at or below the surface (<= 0)")
        if a.engage_feed <= 0:
            problems.append("engage-feed must be positive")
    if not a.allow_fast:
        for name, val in (("feed", a.feed), ("travel-feed", a.travel_feed),
                          ("plunge-feed", a.plunge_feed)):
            if val > MAX_FEED:
                problems.append(f"{name} {val:g} exceeds Klipper's "
                                f"{MAX_FEED:g} mm/min (10 mm/s) ceiling; "
                                "pass --allow-fast to emit it anyway")
    if a.angles:
        try:
            [float(x) for x in a.angles.split(",") if x.strip()]
        except ValueError:
            problems.append("--angles must be a comma-separated number list")
    if problems:
        p.error("; ".join(problems))
    if a.min_line is None:
        a.min_line = 0.5 * a.stepover
    if a.segment is None:
        a.segment = 2.0 if a.force <= 0 else 10.0
    return a


def default_name(args, layers):
    ang = "-".join(f"{a:g}" for a, _ in layers).replace(".", "p")
    kind = "meander" if len(layers) == 1 else "xhatch"
    load = (f"pos{args.xy_per_z:g}" if position_mode(args)
            else f"F{args.force:g}g")
    return (f"{kind}_{args.length:g}x{args.width:g}_s{args.stepover:g}"
            f"_f{args.feed:g}_{load}_a{ang}.gcode")


def main(argv=None):
    args = parse_args(argv)
    layers = build_layers(args)
    if not layers:
        sys.exit("no hatch lines — stepover/inset too large for the patch")
    text, engagements, _ = generate(args, layers)
    est = estimates(args, layers, engagements)

    if args.output == "-":
        sys.stdout.write(text)
        sys.stdout.flush()
        dest = "<stdout>"
    else:
        dest = args.output or default_name(args, layers)
        with open(dest, "w") as fh:
            fh.write(text)
    if args.svg:
        with open(args.svg, "w") as fh:
            fh.write(svg_preview(args, layers))

    mins, secs = divmod(int(round(est["time_s"])), 60)
    print(f"wrote {dest}", file=sys.stderr)
    print(f"  layers {est['layers']} ({', '.join(f'{a:g}' for a, _ in layers)} "
          f"deg), {est['lines']} hatch lines, {est['engagements']} "
          f"engagement(s)", file=sys.stderr)
    print(f"  deposited {est['deposited_mm']:.0f} mm, travel "
          f"{est['travel_mm']:.0f} mm, ~{mins} min {secs:02d} s",
          file=sys.stderr)
    if est["plunge_mm"] is not None:
        print(f"  position mode: total plunge {est['plunge_mm']:.2f} mm "
              f"(final Z {args.start_z - est['plunge_mm']:.2f})",
              file=sys.stderr)
    if args.svg:
        print(f"  preview {args.svg}", file=sys.stderr)


if __name__ == "__main__":
    main()
