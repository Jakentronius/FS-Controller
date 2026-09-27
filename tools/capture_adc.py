#!/usr/bin/env python3
"""Load-cell calibration capture.

For each known load you place on the cell, captures the average of N raw
NAU7802 ADC samples (default 100), prints mean / std / min / max, and
appends a row to a CSV. When you have two or more loads, `f` fits
counts-per-unit (the slope of mean counts vs load) for the Calibration
tab.

By default the samples come from the running dashboard's /api/samples
endpoint, so nothing else touches the I2C bus. With --direct the script
talks to the NAU7802 itself (stop the dashboard first).

    python3 tools/capture_adc.py                      # dashboard on pi:5000
    python3 tools/capture_adc.py --url http://localhost:5000 --n 200
    python3 tools/capture_adc.py --direct             # on the Pi, dashboard stopped

Rows already in the CSV are loaded at start so a fit can span sessions.
"""

import argparse
import csv
import json
import math
import os
import sys
import time
import urllib.request


def fetch_api(url, n):
    with urllib.request.urlopen(f"{url}/api/samples?n={n}",
                                timeout=max(10.0, n / 20.0)) as r:
        d = json.load(r)
    if d.get("status") != "ok":
        raise RuntimeError(d.get("message", "capture failed"))
    return d["samples"]


class DirectReader:
    def __init__(self):
        import qwiic_nau7802
        self.scale = qwiic_nau7802.QwiicNAU7802()
        if not self.scale.is_connected():
            raise RuntimeError("NAU7802 not detected on I2C bus")
        self.scale.begin()
        self.scale.set_sample_rate(
            qwiic_nau7802.QwiicNAU7802.NAU7802_SPS_80)
        time.sleep(0.5)
        while self.scale.available():      # flush stale readings
            self.scale.get_reading()

    def fetch(self, n):
        out = []
        deadline = time.time() + max(10.0, n / 20.0)
        while len(out) < n and time.time() < deadline:
            if self.scale.available():
                out.append(self.scale.get_reading())
            else:
                time.sleep(0.002)
        if len(out) < n:
            raise RuntimeError(f"only {len(out)}/{n} samples arrived")
        return out


def stats(samples):
    n = len(samples)
    mean = sum(samples) / n
    var = sum((v - mean) ** 2 for v in samples) / max(1, n - 1)
    return mean, var ** 0.5, min(samples), max(samples)


def fit(rows):
    """Least-squares counts = slope * load + offset over rows with a
    numeric load. Returns (slope, offset, r2, residuals) or None."""
    pts = [(r["load"], r["mean"]) for r in rows if r["load"] is not None]
    if len(pts) < 2 or len({p[0] for p in pts}) < 2:
        return None
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
    slope = sxy / sxx
    offset = my - slope * mx
    ss_res = sum((y - (slope * x + offset)) ** 2 for x, y in pts)
    ss_tot = sum((y - my) ** 2 for _, y in pts)
    r2 = 1.0 - ss_res / ss_tot if ss_tot else 1.0
    resid = [(x, y, (y - offset) / slope - x) for x, y in pts]
    return slope, offset, r2, resid


def _nice_ticks(lo, hi, n=5):
    span = hi - lo if hi > lo else 1.0
    raw = span / max(1, n)
    mag = 10 ** math.floor(math.log10(raw))
    step = min((s for s in (1, 2, 2.5, 5, 10) if s * mag >= raw),
               key=lambda s: s) * mag
    start = math.floor(lo / step) * step
    ticks = []
    v = start
    while v <= hi + step * 1e-9:
        if v >= lo - step * 1e-9:
            ticks.append(round(v, 10))
        v += step
    return ticks


def _fmt(v):
    return f"{v:g}" if abs(v) < 1e6 else f"{v:.3g}"


def write_fit_report(stem, rows, res, title):
    """Write <stem>_fit.txt (slope / offset / R^2 / residuals) and
    <stem>_fit.svg (data + fit line, residual panel). No dependencies:
    the SVG is written by hand so the tool runs anywhere."""
    slope, offset, r2, resid = res
    pts = sorted((x, y) for x, y, _ in resid)
    txt = stem + "_fit.txt"
    with open(txt, "w") as f:
        f.write(f"{title}\n")
        f.write(f"counts_per_unit: {slope:.6g}\n")
        f.write(f"zero_offset_counts: {offset:.3f}\n")
        f.write(f"r_squared: {r2:.8f}\n")
        f.write(f"points: {len(pts)}\n\n")
        f.write("load, mean_counts, residual_load_units\n")
        for x, y, e in sorted(resid):
            f.write(f"{x:g}, {y:.3f}, {e:+.4f}\n")

    # ---- SVG ---------------------------------------------------------
    W, H = 760, 620
    ml, mr = 90, 24
    top = dict(y0=70, y1=380)          # counts panel
    bot = dict(y0=430, y1=560)         # residual panel
    ink, muted, grid = "#1f2933", "#6b7280", "#e5e7eb"
    hue = "#2563eb"
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    es = [e for _, _, e in resid]
    xlo, xhi = min(xs), max(xs)
    xpad = (xhi - xlo) * 0.05 or 1.0
    xlo, xhi = xlo - xpad, xhi + xpad
    ylo, yhi = min(ys), max(ys)
    ypad = (yhi - ylo) * 0.06 or 1.0
    ylo, yhi = ylo - ypad, yhi + ypad
    emax = max(abs(e) for e in es) or 0.1
    elo, ehi = -emax * 1.25, emax * 1.25

    def sx(v):
        return ml + (v - xlo) / (xhi - xlo) * (W - ml - mr)

    def sy(v, pan, lo, hi):
        return pan["y1"] - (v - lo) / (hi - lo) * (pan["y1"] - pan["y0"])

    o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
         f'viewBox="0 0 {W} {H}" font-family="system-ui, sans-serif" '
         f'font-size="12" fill="{ink}">',
         f'<rect width="{W}" height="{H}" fill="#ffffff"/>',
         f'<text x="{ml}" y="28" font-size="17" font-weight="600">{title}</text>',
         f'<text x="{ml}" y="48" fill="{muted}">counts_per_unit = {slope:.6g}'
         f'   zero offset = {offset:.1f} counts   R\u00b2 = {r2:.6f}'
         f'   ({len(pts)} points)</text>']

    def panel(pan, lo, hi, ylabel, ticks_y):
        for t in ticks_y:
            y = sy(t, pan, lo, hi)
            o.append(f'<line x1="{ml}" x2="{W - mr}" y1="{y:.1f}" y2="{y:.1f}" '
                     f'stroke="{grid}" stroke-width="1"/>')
            o.append(f'<text x="{ml - 8}" y="{y + 4:.1f}" text-anchor="end" '
                     f'fill="{muted}">{_fmt(t)}</text>')
        for t in _nice_ticks(xlo, xhi, 6):
            x = sx(t)
            o.append(f'<line x1="{x:.1f}" x2="{x:.1f}" y1="{pan["y1"]}" '
                     f'y2="{pan["y1"] + 4}" stroke="{muted}"/>')
            o.append(f'<text x="{x:.1f}" y="{pan["y1"] + 17}" '
                     f'text-anchor="middle" fill="{muted}">{_fmt(t)}</text>')
        o.append(f'<line x1="{ml}" x2="{W - mr}" y1="{pan["y1"]}" '
                 f'y2="{pan["y1"]}" stroke="{muted}"/>')
        ym = (pan["y0"] + pan["y1"]) / 2
        o.append(f'<text transform="translate(22 {ym:.0f}) rotate(-90)" '
                 f'text-anchor="middle" fill="{muted}">{ylabel}</text>')

    # top: counts vs load with fit line
    panel(top, ylo, yhi, "mean ADC counts", _nice_ticks(ylo, yhi, 5))
    x0, x1 = xlo, xhi
    o.append(f'<line x1="{sx(x0):.1f}" y1="{sy(slope * x0 + offset, top, ylo, yhi):.1f}" '
             f'x2="{sx(x1):.1f}" y2="{sy(slope * x1 + offset, top, ylo, yhi):.1f}" '
             f'stroke="{hue}" stroke-width="2" stroke-opacity="0.55"/>')
    for x, y in pts:
        o.append(f'<circle cx="{sx(x):.1f}" cy="{sy(y, top, ylo, yhi):.1f}" r="5" '
                 f'fill="{hue}" stroke="#ffffff" stroke-width="2"/>')
    o.append(f'<text x="{W - mr}" y="{top["y0"] - 8}" text-anchor="end" fill="{muted}">'
             f'points = 100-sample means, line = least-squares fit</text>')

    # bottom: residuals in load units
    panel(bot, elo, ehi, "residual (load units)", _nice_ticks(elo, ehi, 4))
    yz = sy(0, bot, elo, ehi)
    o.append(f'<line x1="{ml}" x2="{W - mr}" y1="{yz:.1f}" y2="{yz:.1f}" '
             f'stroke="{muted}" stroke-dasharray="4 3"/>')
    for x, _, e in resid:
        o.append(f'<circle cx="{sx(x):.1f}" cy="{sy(e, bot, elo, ehi):.1f}" r="5" '
                 f'fill="{hue}" stroke="#ffffff" stroke-width="2"/>')
    o.append(f'<text x="{(ml + W - mr) / 2:.0f}" y="{H - 14}" text-anchor="middle" '
             f'fill="{muted}">load</text>')
    o.append("</svg>")
    svg = stem + "_fit.svg"
    with open(svg, "w") as f:
        f.write("\n".join(o))
    return txt, svg


def load_csv(path):
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            try:
                load = float(r["load"]) if r["load"] not in ("", None) \
                    else None
                rows.append({"load": load, "mean": float(r["mean"])})
            except (KeyError, ValueError):
                continue
    return rows


def report_fit(rows, out):
    res = fit(rows)
    if res is None:
        print("  need at least two different loads to fit")
        return False
    slope, offset, r2, resid = res
    print(f"  counts_per_unit = {slope:.6g}   "
          f"(zero offset {offset:.1f} counts, R^2 = {r2:.6f})")
    for x, y, err in sorted(resid):
        print(f"    load {x:g}: mean {y:.1f}  "
              f"residual {err:+.3f} load-units")
    stem = os.path.splitext(out)[0]
    name = os.path.basename(stem)
    if name.startswith("calibration_"):
        name = name[len("calibration_"):]
    txt, svg = write_fit_report(stem, rows, res,
                                f"Load cell calibration {name}")
    print(f"  wrote {txt} and {svg}")
    print("  Paste counts_per_unit into the Calibration tab "
          "(negate it if force reads negative when pressing down).")
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--url", default="http://pi.local:5000",
                    help="dashboard base URL (default http://pi.local:5000)")
    ap.add_argument("--n", type=int, default=100,
                    help="samples per capture (default 100)")
    ap.add_argument("--out", default=None,
                    help="CSV path (default calibration_YYYYMMDD.csv)")
    ap.add_argument("--direct", action="store_true",
                    help="read the NAU7802 directly instead of the API "
                         "(stop the dashboard first)")
    ap.add_argument("--fit", action="store_true",
                    help="fit the CSV, write <csv>_fit.txt/.svg and exit "
                         "(no captures)")
    args = ap.parse_args()
    out = args.out or f"calibration_{time.strftime('%Y%m%d')}.csv"
    if args.fit:
        rows = load_csv(out)
        if not rows:
            sys.exit(f"no rows in {out}")
        sys.exit(0 if report_fit(rows, out) else 1)

    if args.direct:
        reader = DirectReader()
        fetch = reader.fetch
    else:
        fetch = lambda n: fetch_api(args.url.rstrip("/"), n)

    rows = load_csv(out)
    if rows:
        print(f"Loaded {len(rows)} earlier rows from {out}")
    new_file = not os.path.exists(out)
    csv_f = open(out, "a", newline="")
    writer = csv.writer(csv_f)
    if new_file:
        writer.writerow(["timestamp", "load", "n", "mean", "std",
                         "min", "max"])
        csv_f.flush()

    print(f"Capturing {args.n} samples per point -> {out}")
    print("Enter the load on the cell (e.g. 0, 500, 1000), "
          "'f' to fit, 'q' to quit.")
    zero = None
    while True:
        try:
            ans = input("load> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if ans.lower() in ("q", "quit", "exit"):
            break
        if ans.lower() == "f":
            report_fit(rows, out)
            continue
        load = None
        if ans:
            try:
                load = float(ans)
            except ValueError:
                print("  not a number (use 'f' or 'q')")
                continue
        try:
            t0 = time.time()
            samples = fetch(args.n)
        except Exception as e:
            print(f"  capture failed: {e}")
            continue
        mean, std, lo, hi = stats(samples)
        label = f"{load:g}" if load is not None else "(unlabelled)"
        line = (f"  load {label}: mean {mean:.1f}  std {std:.1f}  "
                f"min {lo}  max {hi}  ({len(samples)} samples, "
                f"{time.time() - t0:.1f} s)")
        if load == 0:
            zero = mean
        elif zero is not None:
            line += f"  delta from zero {mean - zero:+.1f}"
        print(line)
        writer.writerow([time.strftime("%Y-%m-%d %H:%M:%S"),
                         "" if load is None else f"{load:g}", len(samples),
                         f"{mean:.3f}", f"{std:.3f}", lo, hi])
        csv_f.flush()
        rows.append({"load": load, "mean": mean})
    csv_f.close()
    print(f"Saved to {out}")


if __name__ == "__main__":
    main()
