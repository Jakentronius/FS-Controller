#!/usr/bin/env python3
"""Tiny tkinter front end for hatch_gcode.py: fill in the patch, layer
and process fields, watch the toolpath preview update, then save the
job. Setting force to 0 switches to position mode (fixed XY:Z plunge
ratio, force controller off) and enables that group of fields.

Run:  python3 tools/hatch_gui.py
"""

import contextlib
import io
import os
import re
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hatch_gcode as hg  # noqa: E402

PALETTE = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd",
           "#8c564b", "#17becf", "#e377c2"]

# (label, argparse flag, default, unit/hint) per group
GROUPS = [
    ("Patch", [
        ("Length (X)", "--length", "30", "mm"),
        ("Width (Y)", "--width", "20", "mm"),
        ("Stepover", "--stepover", "2", "mm"),
        ("Inset", "--inset", "0", "mm in from the edges"),
    ]),
    ("Layers", [
        ("Layers", "--layers", "1", "1 = plain meander"),
        ("Rotate per layer", "--rotate", "90", "deg (90: 0/90/0/90, "
                                                "45: 0/45/90/135)"),
        ("Start angle", "--start-angle", "0", "deg from +X"),
        ("Angle list", "--angles", "", "optional, e.g. 0,90,45,135 "
                                       "(overrides the three above)"),
    ]),
    ("Process", [
        ("Feed", "--feed", "200", "mm/min (max 600)"),
        ("Downforce", "--force", "500", "grams; 0 = position mode"),
        ("Induction dwell", "--induction", "5", "s after touch-down"),
        ("End dwell", "--end-dwell", "0", "s"),
    ]),
    ("Position mode (force = 0)", [
        ("XY per mm of Z", "--xy-per-z", "100", "mm XY travel per 1 mm "
                                                "plunge; 0 = flat"),
        ("Start Z", "--start-z", "0", "mm, surface-relative (<= 0)"),
        ("Max plunge", "--max-plunge", "0", "mm below start Z; 0 = none"),
        ("Engage feed", "--engage-feed", "60", "mm/min, approach -> start Z"),
    ]),
    ("Heights & travel", [
        ("Travel Z", "--travel-z", "10", "mm above surface"),
        ("Approach Z", "--approach-z", "1", "mm (<= 3)"),
        ("Layer lift", "--layer-lift", "1", "mm between layers"),
        ("Final lift", "--lift", "5", "mm"),
        ("Travel feed", "--travel-feed", "600", "mm/min"),
        ("Plunge feed", "--plunge-feed", "300", "mm/min"),
        ("Wait-force timeout", "--wait-timeout", "30", "s"),
        ("Segment", "--segment", "", "mm (blank = 10, or 2 in position "
                                     "mode)"),
    ]),
]
POSITION_FLAGS = {"--xy-per-z", "--start-z", "--max-plunge", "--engage-feed"}
FORCE_ONLY_FLAGS = {"--wait-timeout"}


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("FS hatch job generator")
        self.entries = {}
        self.layers = []
        self.args = None
        self._build()
        self.after(100, self.refresh)

    # ---------------------------------------------------------------- UI
    def _build(self):
        left = ttk.Frame(self, padding=8)
        left.grid(row=0, column=0, sticky="nsw")
        right = ttk.Frame(self, padding=8)
        right.grid(row=0, column=1, sticky="nsew")
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        row = 0
        for title, fields in GROUPS:
            lf = ttk.LabelFrame(left, text=title, padding=(6, 4))
            lf.grid(row=row, column=0, sticky="ew", pady=(0, 6))
            row += 1
            for r, (label, flag, default, hint) in enumerate(fields):
                ttk.Label(lf, text=label).grid(row=r, column=0, sticky="w")
                var = tk.StringVar(value=default)
                ent = ttk.Entry(lf, textvariable=var, width=10)
                ent.grid(row=r, column=1, padx=4)
                ttk.Label(lf, text=hint, foreground="#666").grid(
                    row=r, column=2, sticky="w")
                var.trace_add("write", lambda *_: self._schedule())
                self.entries[flag] = (var, ent)

        opts = ttk.LabelFrame(left, text="Options", padding=(6, 4))
        opts.grid(row=row, column=0, sticky="ew", pady=(0, 6))
        self.origin = tk.StringVar(value="center")
        self.link = tk.StringVar(value="lift")
        self.corner = tk.StringVar(value="sw")
        self.layer_start = tk.StringVar(value="nearest")
        self.reinduct = tk.BooleanVar(value=False)
        self.allow_fast = tk.BooleanVar(value=False)
        ttk.Label(opts, text="X0 Y0 at").grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(opts, text="patch centre", variable=self.origin,
                        value="center").grid(row=0, column=1, sticky="w")
        ttk.Radiobutton(opts, text="min corner", variable=self.origin,
                        value="corner").grid(row=0, column=2, sticky="w")
        ttk.Label(opts, text="Start corner").grid(row=1, column=0,
                                                  sticky="w")
        cf = ttk.Frame(opts)
        cf.grid(row=1, column=1, columnspan=2, sticky="w")
        for name, label in (("sw", "SW (−X −Y)"), ("se", "SE (+X −Y)"),
                            ("nw", "NW (−X +Y)"), ("ne", "NE (+X +Y)")):
            ttk.Radiobutton(cf, text=label, variable=self.corner,
                            value=name).pack(side="left", padx=(0, 4))
        ttk.Label(opts, text="Later layers").grid(row=2, column=0,
                                                  sticky="w")
        ttk.Radiobutton(opts, text="start nearest previous end",
                        variable=self.layer_start, value="nearest").grid(
            row=2, column=1, sticky="w")
        ttk.Radiobutton(opts, text="restart at that corner",
                        variable=self.layer_start, value="same").grid(
            row=2, column=2, sticky="w")
        ttk.Label(opts, text="Between layers").grid(row=3, column=0,
                                                    sticky="w")
        ttk.Radiobutton(opts, text="lift & re-engage", variable=self.link,
                        value="lift").grid(row=3, column=1, sticky="w")
        ttk.Radiobutton(opts, text="stay engaged", variable=self.link,
                        value="engaged").grid(row=3, column=2, sticky="w")
        ttk.Checkbutton(opts, text="repeat induction dwell at every "
                        "re-engagement", variable=self.reinduct).grid(
            row=4, column=0, columnspan=3, sticky="w")
        ttk.Checkbutton(opts, text="allow feeds above 600 mm/min",
                        variable=self.allow_fast).grid(
            row=5, column=0, columnspan=3, sticky="w")
        for v in (self.origin, self.link, self.corner, self.layer_start,
                  self.reinduct, self.allow_fast):
            v.trace_add("write", lambda *_: self._schedule())

        btns = ttk.Frame(left)
        btns.grid(row=row + 1, column=0, sticky="ew")
        ttk.Button(btns, text="Save G-code…", command=self.save_gcode).pack(
            side="left")
        ttk.Button(btns, text="Save SVG…", command=self.save_svg).pack(
            side="left", padx=6)
        ttk.Button(btns, text="Copy command", command=self.copy_cmd).pack(
            side="left")

        self.canvas = tk.Canvas(right, width=560, height=420, bg="white",
                                highlightthickness=1,
                                highlightbackground="#bbb")
        self.canvas.grid(row=0, column=0, sticky="nsew")
        right.rowconfigure(0, weight=1)
        right.columnconfigure(0, weight=1)
        self.canvas.bind("<Configure>", lambda e: self.draw())

        self.status = tk.Text(right, height=7, width=70, wrap="word",
                              relief="flat", bg=self.cget("bg"))
        self.status.grid(row=1, column=0, sticky="ew", pady=(6, 0))
        self.status.configure(state="disabled")
        self.gcode_box = tk.Text(right, height=10, width=70, wrap="none",
                                 font=("TkFixedFont", 9))
        self.gcode_box.grid(row=2, column=0, sticky="nsew", pady=(6, 0))
        right.rowconfigure(2, weight=1)

    # ------------------------------------------------------------ logic
    def _schedule(self):
        if getattr(self, "_pending", None):
            self.after_cancel(self._pending)
        self._pending = self.after(250, self.refresh)

    def argv(self):
        argv = []
        for flag, (var, _) in self.entries.items():
            val = var.get().strip()
            if val:
                argv += [flag, val]
        argv += ["--origin", self.origin.get(),
                 "--layer-link", self.link.get(),
                 "--start-corner", self.corner.get(),
                 "--layer-start", self.layer_start.get()]
        if self.reinduct.get():
            argv.append("--reinduct")
        if self.allow_fast.get():
            argv.append("--allow-fast")
        return argv

    def _set_mode_widgets(self):
        try:
            pos = float(self.entries["--force"][0].get() or "0") <= 0
        except ValueError:
            pos = False
        for flag, (_, ent) in self.entries.items():
            if flag in POSITION_FLAGS:
                ent.configure(state="normal" if pos else "disabled")
            elif flag in FORCE_ONLY_FLAGS:
                ent.configure(state="disabled" if pos else "normal")

    def refresh(self):
        self._pending = None
        self._set_mode_widgets()
        err = io.StringIO()
        try:
            with contextlib.redirect_stderr(err):
                self.args = hg.parse_args(self.argv())
        except SystemExit:
            self.args, self.layers = None, []
            msg = err.getvalue().strip().splitlines()
            self._status(re.sub(r"^.*?: error: ", "", msg[-1])
                         if msg else "invalid input")
            self.draw()
            return
        self.layers = hg.build_layers(self.args)
        if not self.layers:
            self._status("no hatch lines — stepover/inset too large")
            self.draw()
            return
        text, engagements, _ = hg.generate(self.args, self.layers)
        est = hg.estimates(self.args, self.layers, engagements)
        mins, secs = divmod(int(round(est["time_s"])), 60)
        angles = ", ".join(f"{a:g}" for a, _ in self.layers)
        lines = [
            f"{'POSITION' if est['mode'] == 'position' else 'FORCE'} mode  "
            f"|  {est['layers']} layer(s) at {angles} deg  |  "
            f"{est['lines']} hatch lines  |  {est['engagements']} "
            f"engagement(s)",
            f"deposited {est['deposited_mm']:.0f} mm, travel "
            f"{est['travel_mm']:.0f} mm, about {mins} min {secs:02d} s",
        ]
        if est["plunge_mm"] is not None:
            lines.append(f"total plunge {est['plunge_mm']:.2f} mm "
                         f"-> final Z {self.args.start_z - est['plunge_mm']:.2f}"
                         " (surface-relative)")
        lines.append("file: " + hg.default_name(self.args, self.layers))
        self._status("\n".join(lines))
        self.gcode_box.delete("1.0", "end")
        self.gcode_box.insert("1.0", text)
        self.draw()

    def _status(self, text):
        self.status.configure(state="normal")
        self.status.delete("1.0", "end")
        self.status.insert("1.0", text)
        self.status.configure(state="disabled")

    # ---------------------------------------------------------- preview
    def draw(self):
        c = self.canvas
        c.delete("all")
        if not self.args:
            return
        a = self.args
        outer = hg.rect_corners(a.length, a.width, a.origin)
        inner = hg.rect_corners(a.length, a.width, a.origin, a.inset)
        pad = max(2.0, a.stepover)
        xs = [p[0] for p in outer]
        ys = [p[1] for p in outer]
        minx, maxx = min(xs) - pad, max(xs) + pad
        miny, maxy = min(ys) - pad, max(ys) + pad
        W, H = max(c.winfo_width(), 50), max(c.winfo_height(), 50)
        scale = min(W / (maxx - minx), H / (maxy - miny))
        ox = (W - (maxx - minx) * scale) / 2
        oy = (H - (maxy - miny) * scale) / 2

        def sx(x):
            return ox + (x - minx) * scale

        def sy(y):
            return oy + (maxy - y) * scale

        c.create_polygon([(sx(x), sy(y)) for x, y in outer], fill="",
                         outline="#999", dash=(4, 3))
        if a.inset > 0:
            c.create_polygon([(sx(x), sy(y)) for x, y in inner], fill="",
                             outline="#ccc", dash=(2, 3))
        for i, (angle, pts) in enumerate(self.layers):
            col = PALETTE[i % len(PALETTE)]
            c.create_line([(sx(x), sy(y)) for x, y in pts], fill=col,
                          width=1.5)
            x0, y0 = pts[0]
            c.create_oval(sx(x0) - 4, sy(y0) - 4, sx(x0) + 4, sy(y0) + 4,
                          fill=col, outline="")
            c.create_text(8, 10 + 15 * i, anchor="w", fill=col,
                          text=f"layer {i + 1}: {angle:g} deg (dot = start)")
        c.create_line(sx(-1), sy(0), sx(1), sy(0), fill="black")
        c.create_line(sx(0), sy(-1), sx(0), sy(1), fill="black")

    # ----------------------------------------------------------- output
    def _ready(self):
        if not self.args or not self.layers:
            messagebox.showerror("Nothing to save",
                                 "Fix the highlighted problem first.")
            return False
        return True

    def save_gcode(self):
        if not self._ready():
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".gcode",
            initialfile=hg.default_name(self.args, self.layers),
            filetypes=[("G-code", "*.gcode *.nc *.g"), ("All", "*")])
        if not path:
            return
        text, _, _ = hg.generate(self.args, self.layers)
        with open(path, "w") as fh:
            fh.write(text)
        self._status(self.status.get("1.0", "end").strip()
                     + f"\nsaved {path}")

    def save_svg(self):
        if not self._ready():
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".svg",
            initialfile=hg.default_name(self.args, self.layers)
            .replace(".gcode", ".svg"),
            filetypes=[("SVG", "*.svg")])
        if not path:
            return
        with open(path, "w") as fh:
            fh.write(hg.svg_preview(self.args, self.layers))

    def copy_cmd(self):
        cmd = "python3 tools/hatch_gcode.py " + " ".join(
            a if " " not in a else repr(a) for a in self.argv())
        self.clipboard_clear()
        self.clipboard_append(cmd)
        self._status(self.status.get("1.0", "end").strip()
                     + "\ncopied: " + cmd)


if __name__ == "__main__":
    App().mainloop()
