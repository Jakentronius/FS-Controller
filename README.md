# Friction Surfacing Dashboard

Web control panel for a hobby CNC (Genmitsu 3018-class) converted for
micro friction surfacing, with closed-loop plunge-force control. Runs on
the machine's Raspberry Pi alongside Klipper / Moonraker / Mainsail.

Companion project: [FS CAM](https://github.com/Jakentronius/circuit_cam) turns
DXF/SVG drawings into annotated G-code jobs for this dashboard.

> **Safety:** this software drives a machine that presses a spinning rod
> into metal under hundreds of newtons of force. It is a personal research
> tool published as-is, with no warranty. Read the *Safety Limits* section
> before running anything, keep the E-STOP in reach, and treat the PID
> tuning defaults as a starting point for *your* hardware, not a
> guarantee.

## Hardware this was built for

| Part | Role |
|---|---|
| Genmitsu 3018-class CNC frame | X/Y/Z motion |
| BTT SKR 1.4 Turbo running Klipper | Motion controller (via Moonraker's HTTP API) |
| MKS SERVO42C closed-loop stepper | Z axis |
| 4 × load cells + SparkFun Qwiic NAU7802 | Plunge-force measurement (~80 Hz over I2C) |
| Raspberry Pi | Runs Klipper, Moonraker, Mainsail and this dashboard |

## Installation

On the Pi, with Klipper + Moonraker already working (Moonraker is expected
at `http://127.0.0.1:7125`) and I2C enabled (`raspi-config`):

```bash
git clone https://github.com/Jakentronius/FS-Controller.git ~/load_cells/app
python3 -m venv ~/load_cells/venv
~/load_cells/venv/bin/pip install -r ~/load_cells/app/requirements.txt

# run it once by hand to check the sensor and Moonraker are found
cd ~/load_cells/app && ~/load_cells/venv/bin/python app.py

# install as a service (paths in dashboard.service assume the layout above)
sudo cp ~/load_cells/app/dashboard.service /etc/systemd/system/
sudo systemctl enable --now dashboard
```

`config.json`, `logs/`, `jobs/`, `profiles/` and the mesh/touch-off files are
created at runtime and are git-ignored — calibration lives on the machine,
not in the repo.

## Repository layout

```
app.py              Flask JSON API
hardware.py         sensor / telemetry / PID / job-runner threads
templates/, static/ vanilla-JS frontend (Chart.js from CDN)
dashboard.service   systemd unit
example_job.csv     minimal CSV job
example_job.gcode   minimal annotated G-code job
```

---

# User Guide

**Open it at:** `http://<pi-ip>:5000`

## What it does

- Reads Z-plunge force from 4 load cells through a NAU7802 ADC at ~80 Hz
- Regulates that force in real time with a PID controller that nudges
  Klipper's gcode Z offset (`SET_GCODE_OFFSET Z_ADJUST=… MOVE=1`)
- Runs automated toolpaths (CSV or annotated G-code) that combine X/Y motion
  with force targets, force-waits, and timed dwells
- Logs raw sensor data to CSV for offline analysis and calibration

The Z axis is an MKS SERVO42C closed-loop stepper driven by Klipper on a BTT
SKR 1.4 Turbo. Z is homed at the top (`Z=50`); **lower Z = toward the work**.

---

## Starting and stopping

The dashboard runs as a systemd service and starts automatically on boot.
On the pi (or over SSH):

```bash
sudo systemctl restart dashboard    # restart (e.g. after editing code/config)
sudo systemctl stop dashboard       # stop
sudo systemctl start dashboard      # start
sudo journalctl -u dashboard -f     # watch the app log live
```

Mainsail stays available on its usual port; the dashboard talks to the same
Klipper through Moonraker, so both can be open at once.

---

## Header (always visible)

Five status pills:

| Pill | Meaning |
|---|---|
| **Klipper** | Moonraker/Klipper state (`STANDBY`, `PRINTING`, red = offline) |
| **Sensor** | Measured sample rate (~77 Hz is healthy) + tare state; red = ADC fault |
| **AUTO Z** | Force control on/off and the active target |
| **Log** | Whether raw data is being recorded |
| **Job** | Current job progress, or last job's outcome |

**E-STOP** (top right) issues Klipper `M112` — an immediate firmware halt.
Motors stop and Klipper must be restarted afterwards — use the **FW
Restart** button next to it. Use E-STOP when something is actually going
wrong physically.

**Safety faults** — a red banner appears (and everything halts + retracts)
when a watchdog trips: force beyond the **Max force** limit, or the
sensor stalling (no samples for 0.5 s) while force was being relied on.
Nothing that moves under force control will start again until you press
**Clear Fault**. Configure the limits on the Calibration tab → Safety
Limits; set **Max force** below your load cells' rating and **Z floor**
just above the build plate.

---

## Dashboard tab

**Readouts:** live force (calibrated units), raw ADC counts, X/Y/Z machine
position with per-axis homed badges, and the **Z Offset** — how far the force
controller has pushed Z away from the commanded position. Two small charts
show recent force and Z history.

**Controls:**

- **Start/Stop Log** — records `Time_epoch_s, Raw_ADC, Force_units,
  Force_target, X, Y, Z, Z_offset` at full sensor rate to a timestamped
  CSV in `~/load_cells/app/logs/`. Raw ADC counts are always included so
  calibration can be redone offline. No tare needed to log.
- **Tare / Clear Tare** — optional zero for the live display and the force
  controller's error signal. Tare with the head in free air so the tool's own
  weight reads as zero force. Logging is unaffected by tare.
- **Target + Set / AUTO Z** — set the force target and toggle the PID
  controller manually (for experiments outside a job).

**Touch Off** — single-point tool-length / starting-height reference.
With the head parked over the substrate at the current XY (spindle off,
tool able to take light contact — or the rigid rod), it descends slowly
until it feels the surface, records **Surface Z** (shown in the readouts),
then parks *Hover* mm above it, ready to engage. It also reports the delta
against the previous touch-off: after a mechtrode swap that's the length
difference of the new rod; between experiments it's the wear since last
time. The last touch-off persists across restarts. Use it after every rod
change and whenever wear has accumulated enough to matter for park
heights in your job files.

**Jog panel:** X/Y cross pad and Z± buttons; step selector 0.1 / 1 / 10 mm.
**Work zero:** jog the head to where a job should start and press
**Zero XY Here** — that spot becomes the job origin, so author every job
around `X0 Y0` and run it anywhere on any substrate. The status line
shows the origin (machine coords) and the head's current work
coordinates; **Clear** returns to machine coordinates. The origin
survives between jobs (only the force controller's Z offset is cleared)
but is lost on a Klipper restart. Preflight checks job bounds with the
origin applied, so a job that fits at one origin may correctly refuse to
run at another.
**Go to:** type any subset of absolute **machine** X/Y/Z and hit **Go** —
targets are machine coordinates regardless of work origin; Z rises
before the XY traverse and descends after it, so the move can't drag the
tool across the part. **Home All / XY / Z** buttons home via `G28`.
Jogging, Go to, and homing are refused while a job is running. If Klipper rejects a move (not homed, out of
bounds), the reason appears under the pad.

---

## Calibration tab

**Scale response factor** — converts raw ADC counts to physical units
(grams, newtons — your choice of label). This is the number the whole system
uses for display, control targets, and job force values.

Recommended multi-point procedure:

1. Dashboard tab → **Start Log**
2. Apply a series of known masses to the head, a few seconds each
3. **Stop Log**, copy the CSV from `~/load_cells/app/logs/`
4. Fit a line: raw counts vs. known mass → the slope is your factor
5. Enter it under **Counts per unit**, set the units label, **Save**

Quick alternative: the collapsible two-point helper captures an
unloaded and a loaded reading and computes the factor for you (still needs
Save).

The factor is signed — if force reads negative when you press down, negate
it. Persisted in `~/load_cells/app/config.json`, survives reboots.

**Abort behavior** — how far and how fast Z retracts when a job aborts or
errors.

**Travel feeds** — the speeds Jog / Go To / Park use (mm/min, XY and Z
separately). These can be set high; Klipper still caps the real speed at
`printer.cfg` `max_velocity` / `max_z_velocity`, so raise those firmware
limits too if travel feels slow.

**Surface mesh** — the build plate sits on the load-cell platform and is
never perfectly level. The mesh probe touches off a grid (default
5×5 over a 100 mm square; rectangular grids up to 120 points are
supported — e.g. 21×2 over 100×20 mm gives 5 mm resolution along X for
catching short-wavelength height waviness in the weld direction)
centered on the current XY position, using the load
cells as the contact sensor — same setup as Auto-Tune: rigid non-spinning
rod, spindle off, all axes homed, head parked a few mm above the substrate
at the center of the region you want mapped.

Each grid point: rapid to just above the highest contact found so far,
creep down in 0.05 mm steps until the force threshold trips, back off and
re-touch in 0.01 mm steps. The resulting map (shown as a colored grid,
blue = low, red = high) is saved to `mesh.json` and survives restarts.

**Named meshes** — save the active mesh under a name (one per substrate
mount, e.g. `plate-A-mount2`); saved meshes live in
`~/load_cells/app/meshes/` and can be reloaded later, which makes them the
active mesh. Since force control only uses mesh *deltas* (shape, not
absolute height), a saved mesh stays valid across mechtrode swaps — but
re-probe after re-mounting or shimming the plate.

During force control the mesh is a **feedforward only** — the live force
reading always stays primary:

- When AUTO Z engages, the mesh delta is zeroed *at that XY position* —
  the mesh never shifts your absolute touch-off, it only follows relative
  tilt as the tool traverses.
- Each control cycle adds the change in interpolated surface height to
  the Z offset (capped by Max step like any correction), and the PID
  corrects whatever force error remains.
- Uncheck **Use mesh during force control** (persisted) or **Clear Mesh**
  to fall back to pure force feedback.

Re-probe after re-mounting the plate or changing substrate. A mesh range
of a few hundredths of a mm is not worth chasing — the force controller
absorbs it; the mesh earns its keep when tilt across the part approaches
or exceeds the flash thickness.

---

## Tuning tab

Live PID tuning, done **outside** a job: typically with the spinning tool
pressed against scrap material, before production runs or after machine
changes.

- Fast chart (10 Hz): measured force + target line (left axis), controller
  Z offset output (right axis, green)
- **AUTO Z** toggle and **Target** field, plus **± step buttons** to jump the
  target and provoke a step response
- All parameters **apply live as you type** — but are *not* saved until you
  click **Save to Config**. **Revert to Saved** restores and applies the
  last saved values if an experiment goes wrong.
- The readout under the fields shows the live PID terms:
  `error | P + I + D = out mm/step`

### The parameters

| Parameter | Meaning |
|---|---|
| **Kp** | mm of Z correction per unit of force error. The main knob. |
| **Ki** | Integral gain — removes steady-state error. Only integrates near the target (within 4× deadband, or while unwinding) and is clamped to ¼ of Max step, so it can erase the deadband-edge sag but cannot wind up during touch-downs and hunt through the setpoint. Bleeds inside deadband, resets when AUTO Z toggles on. |
| **Kd** | Derivative gain — damps overshoot. Acts on measurement, so no kick on target steps. Amplifies noise. |
| **Deadband** | "Close enough" band around target: no corrections inside it. Also what `;WAIT_FORCE` uses to declare the target reached. |
| **Eval period** | Seconds between corrections (0.1 = 10 corrections/s). Below ~0.05 s you're fighting Moonraker round-trip latency. |
| **Max step** | Hard cap on any single correction — the safety rail. Max Z slew = max step / eval period. |
| **Avg window** | Samples averaged before the controller sees them (4 ≈ 52 ms). Raise for noise, lower for speed. |
| **Ramp** | Setpoint slew rate (units/s) for upward target changes: the controller ramps an internal effective target instead of presenting the full error at once, so touch-downs load up over ~a second instead of overshooting by stiffness × approach speed × loop latency. Downward changes (`;FORCE=0`) apply instantly. 0 disables. The readout shows `ramping to N` while active. |

### Suggested session

1. Calibrate first — gains are meaningless in raw counts
2. Tare in free air, park the tool on scrap, set a modest target, AUTO Z on
3. Pure P (Ki=Kd=0): raise Kp until a target step responds briskly with
   slight overshoot, then back off ~30%
4. If force settles persistently off-target: add small Ki. If the I term
   rails in the readout, Ki is too high or deadband too tight.
5. Add Kd only to tame overshoot; if D looks jittery, raise the avg window
6. **Save to Config**

### Choosing the deadband

The deadband is a noise/hysteresis filter, not a performance knob. Too
tight and the controller chases sensor noise and mechanical backlash in a
permanent limit cycle; too wide and the force is allowed to wander. Rule of
thumb: **at least 4–6× the sensor noise σ**, and at least the force change
produced by the smallest Z move that actually does anything (stiffness ×
backlash). Auto-Tune measures both and recommends a value.

### Auto-Tune (recommended starting point)

The **Auto-Tune** panel on the Tuning tab measures the machine instead of
guessing. Setup: chuck a **rigid, non-spinning** rod in the collet, jog it
a few mm above the substrate near the center of the build area, make sure
the scale is calibrated, spindle off. Then press **Start Auto-Tune**.

The routine, fully automatic and conservative:

1. Clears any leftover Z offset, tares in free air, measures sensor noise σ
2. Creeps down in 0.05 mm steps (120 mm/min) until it feels contact
   (threshold = max(6σ, 2 % of probe force)); gives up after **Max descent**
3. Backs off and re-approaches in 0.01 mm steps to pin down the contact Z
4. Steps **into** the surface in 0.01 mm increments up to **Probe force**,
   then back **out**, recording force at every step (the plotted
   loading/unloading curves)
5. Fits the loading slope → **stiffness** (units/mm); the loading/unloading
   offset → **hysteresis** (backlash + compliance)
6. Derives conservative gains: Kp corrects ~30 % of the error per cycle
   (Kp = 0.3 / stiffness), Ki = Kp / 3 s (a gentle integral that erases
   the deadband-edge sag a P-only controller rides at under steady
   drift), deadband = max(5σ, stiffness × hysteresis), max step sized so
   a full probe-force error takes ≥ 8 cycles
7. Verifies with a short closed-loop hold at the probe force — if the force
   oscillates, Kp is halved and the hold repeats
8. Retracts to the start height and reports results

Nothing is changed permanently until you act on the results: **Load into
fields** applies them live (like typing them in, not saved), **Apply &
Save** writes them to `config.json`. Safety rails: hard force ceiling at
1.5× probe force, descent limit, refuses to run unhomed / mid-job, **Abort**
and E-STOP work at any time, and the head always retracts — including on
errors. Jogging, homing and job starts are locked out while it runs.

### Sweep mode (gain scheduling)

Contact stiffness isn't constant — it usually rises with force, so a Kp
tuned at 300 g is wrong at 900 g. Check **Sweep force range** and set
Probe force to your **maximum** working force: the routine fits a local
stiffness in four force bands of the loading curve, references Kp to the
stiffest band (stable everywhere), verifies with holds at 25/50/100% of
the probe force, and saves the S(F) table as a **gain schedule**. With
the schedule active, the controller scales Kp/Ki/Kd by S(ref)/S(target)
(clamped 0.25–4×) whenever the force target changes — so multi-force
jobs (e.g. signal traces at 400, power at 800) each get the right gain
automatically. The live readout shows the factor as `sched ×N`. The
schedule is part of the config and travels with mechtrode profiles;
"Load into fields" does not carry it — use **Apply & Save**.

Pick a probe force in the range you actually run jobs at (default 300).
Auto-tune gains are measured with a static rod — expect to nudge Kp after
watching the first real (spinning, hot) run; material softening usually
lowers effective stiffness, which makes the static recommendation err on
the conservative side.

### Mechtrode profiles

Different mechtrodes (stiffness, diameter, length) need different gains —
profiles make swapping them a reload instead of a re-tune. The **Mechtrode
Profiles** panel saves the tuning under a name (e.g. `steel-3mm`) as a JSON
file in `~/load_cells/app/profiles/`:

- **Save Auto-Tune Result** — freezes the latest auto-tune recommendations
  *plus* the measured stiffness, hysteresis, noise and the full probe
  curves (available after a run completes)
- **Save Current Tuning** — freezes whatever is currently in the
  controller parameter fields
- **Load** — applies the profile's settings live **and** persists them to
  `config.json`, so they survive a restart. Warns if the scale calibration
  (units or counts-per-unit) has changed since the profile was saved —
  gains are expressed in calibrated units, so a recalibrated scale means
  the profile should be re-tuned.

Typical flow when chucking a different mechtrode: load its profile if one
exists, otherwise run Auto-Tune once and save the result under a new name.
The force target is deliberately **not** part of a profile — targets
belong to jobs.

---

## Jobs tab

Upload a toolpath, hit **Run**, watch progress. Jobs automatically enable
force control at start and disable it at the end. The accumulated
force-control Z offset is cleared when the job finishes, so consecutive jobs
start from a clean coordinate frame.

### Run logs (automatic)

Every job run records force data automatically to
`logs/run_<jobfile>_<timestamp>.csv` — no need to press Start Log. (If a
manual log is already recording when the job starts, it is left alone.)
The file begins with a human-readable `# key: value` header: job name,
start time, calibration, PID settings, active mesh, last touch-off, and
whatever you typed into **Run notes** before hitting Run (material, rpm,
mechtrode, trial number…). Notes are your future metadata — be generous.

The **Run Logs** panel lists all logs with their notes; **View** plots
force vs. target and the controller Z offset right in the browser
(decimated for display — download the CSV for full-rate data),
**Download** grabs the raw file, **Delete** removes it.

**Before running:** machine homed, sensor tared, calibration and PID saved.
Force control needs somewhere to push — park heights and approach moves are
part of the job file.

### Preflight validation

Every job is statically checked at upload, in the job list (⛔/⚠ badges —
hover for details), and again at Run:

**Errors (block the run):** any coordinate outside the machine's travel
volume, commanded Z below the Z floor, force targets above the Max force
limit, zero/negative feedrates.

**Warnings (shown on the Run confirmation):** feedrate or M204
acceleration beyond the printer's limits (Klipper will cap them), `G4`
dwells (block force corrections — use `;DWELL=`), `G28` or
`SET_GCODE_OFFSET` inside a job (fight the controller/coordinate frame),
toolpaths leaving the probed mesh region, and G91 moves
before any absolute position (bounds only partially verifiable).

Bounds are checked on the coordinates as authored; the force controller's
Z offset shifts the frame at runtime, which static checking cannot
predict — the Z floor clamp in the controller is the runtime backstop.
If Klipper is not ready, travel/velocity limits can't be fetched and the
job gets a warning saying so.

### File formats

**CSV** — columns `X, Y, Target_Force_Units, Feedrate` (mm, calibrated
units, mm/min). One straight move per row; each row sets the force target
then executes its move.

**Annotated G-code** (`.gcode`, `.nc`, `.g`) — standard G-code plus comment
directives interpreted by the job runner (never sent to Klipper, invisible
to any other G-code tool):

| Directive | Meaning |
|---|---|
| `;FORCE=<units>` | Set the force target for everything that follows |
| `;WAIT_FORCE TIMEOUT=<s>` | Wait until force is within deadband of target. If the head is still in free air, it first creeps down with commanded moves (0.03 mm steps at 120 mm/min, max 3 mm) until first contact, then lets the PID pull to target — so the park-height air gap never pollutes the force-control Z offset. Default timeout 60 s; on timeout the job errors and retracts. |
| `;DWELL=<s>` | Hold N seconds while force control keeps regulating. Use this, **not** `G4` — a `G4` blocks the controller's corrections. |
| `;ZREF=SURFACE` | All job Z coordinates are relative to the **last touched-off surface** (Z0 = surface). The job refuses to start without a touch-off. FS CAM emits this automatically — it is what makes `Z1.0` mean "1 mm above the real surface" on any rod, any day. |

Consecutive G-code lines are sent to Klipper as one block, so it
motion-plans across them smoothly. Any CAM-generated 2D toolpath works —
annotate it with directives.

### Example: touch-down / dwell / retract cycle

```gcode
;FORCE=0
G1 Z45 F600            ; travel at safe height
G1 X25 Y30 F600        ; over the spot
G1 Z10 F300            ; park just above the surface

;FORCE=800             ; controller starts plunging toward 800
;WAIT_FORCE TIMEOUT=30 ; block until force is in the deadband
;DWELL=10              ; hold 10 s at force

;FORCE=0
G91
G1 Z5 F600             ; lift off
G90
G1 Z45 F600            ; back to safe height
```

`;FORCE=0` during travel matters: 0 is inside the deadband, so the
controller stays quiet while moving in free air.

### Mid-job rod changes (automatic)

Set **Max rod consumption** on the Calibration tab (0 = disabled). Rod
consumption is how far the force controller has plunged beyond the mesh
feedforward since the job (or last rod change) started — i.e. how much
mechtrode has been consumed. When the limit is hit *while engaged* (the
pause position doubles as the old rod's surface reference, so the check
only fires at force), the job runs this sequence:

1. Force control off, retract **Rod-change retract** mm (default 10)
2. **PAUSED** — amber prompt in Job Status: swap/reload the mechtrode
   (spindle OFF) and press **Rod Changed — Touch Off**
3. Tares (a new rod weighs differently), touches off at the current XY,
   measures the new rod's length difference, and shifts the job's Z
   coordinate frame by exactly that amount — commanded coordinates now
   put the new tip where the old tip was
4. Retracts again and prompts: start the spindle, press
   **Spindle On — Re-engage**
5. Descends to 0.5 mm above the measured surface, enables force control,
   and waits until the force target it left off at is re-acquired
   (120 s limit)
6. Continues the job from the same step — a paused `;DWELL` resumes with
   its remaining hold time intact

This repeats every time the limit is reached until the job finishes or
you abort. **Abort Job** works at any point in the sequence, including
while waiting for confirmation. The run log keeps recording throughout,
so the rod change (retract spike, touch-off, re-engage ramp) is visible
in the force trace, and the job status line shows live consumption
(`rod used 3.42/15 mm`).

### Pausing, stopping, resuming

- **Pause Job** — stops after the current segment, retracts, and waits.
  **Resume Job** returns to the pause position; if the tool was at force
  it hovers 0.3 mm short and lets the controller re-engage to the target
  first. Works mid-`WAIT_FORCE` and mid-`DWELL` (remaining dwell time is
  preserved).
- **Abort Job** — disables force control immediately, lets the in-flight
  move segment finish, retracts Z (per Calibration tab settings), clears the
  Z offset. Responsive mid-`WAIT_FORCE` and mid-`DWELL`.
- **E-STOP** — hard `M112` halt for real emergencies. Requires **FW
  Restart** afterwards; no retract happens.
- **Start at step** (Job Files panel) — resume an interrupted job
  mid-file: the last `;FORCE` before that step is applied, then the
  machine moves directly to that step's coordinates — make sure the
  path is clear and consider a Touch Off first.

Long moves need no manual segmenting: the runner slices every XY move
into ~0.15 s sub-moves, paced to execution and flowing smoothly through
polyline junctions. While slices stream, force corrections ride the
upcoming slices as shallow XYZ ramps (landing within ~0.3 s) rather
than interrupting motion; Abort / Pause / rod changes act mid-pass.

---

## Files on the pi

| Path | What |
|---|---|
| `~/load_cells/app/` | The application |
| `~/load_cells/app/config.json` | Calibration + controller settings (UI-managed; hand-editable, restart after) |
| `~/load_cells/app/logs/` | Raw data logs (timestamped CSVs) |
| `~/load_cells/app/jobs/` | Uploaded job files |
| `~/load_cells/app/app.log` | Stdout log (when run manually; systemd uses journalctl) |
| `/etc/systemd/system/dashboard.service` | Service definition |

## Troubleshooting

| Symptom | Check |
|---|---|
| Sensor pill red / "not detected" | I2C wiring; `i2cdetect -y 1` should show the NAU7802 (0x2A) |
| Sample rate well below ~77 Hz | Pi CPU load; another process using the I2C bus |
| Klipper pill red | Is Klipper/Moonraker up? Check Mainsail |
| Jog rejected | Axis not homed, or move outside `printer.cfg` limits |
| Force reads negative when pressing down | Negate the calibration factor |
| Force oscillates under AUTO Z | Lower Kp/Ki, raise avg window or deadband (Tuning tab) |
| `WAIT_FORCE timed out` | Park height too far above surface for the timeout, target unreachable, or controller too slow — tune first |
| Job won't start | Another job running, Klipper offline, or file invalid (see Jobs table) |

## Architecture (for the curious)

`app.py` — Flask JSON API · `hardware.py` — sensor/telemetry/control/job
threads · `templates/index.html` + `static/` — vanilla JS frontend,
Chart.js. Sensor thread reads the NAU7802 at ~80 Hz; telemetry polls
Moonraker at 20 Hz (including `motion_report.live_position`, the head's
physical position); the PID loop evaluates every eval-period; jobs
slice XY moves into ~0.15 s sub-moves streamed one ~2-slice buffer at a
time, paced on estimated execution time (dead reckoning from the
commanded feed, which the machine can never beat; stale telemetry
position is only a backstop cap, since `motion_report` refreshes at
just ~4 Hz) — Klipper throttles gcode
input once buffered motion runs ~2 s ahead, so a long unsliced G1 would
block `SET_GCODE_OFFSET` corrections for its whole duration. While
slices stream, corrections are sent *without* `MOVE=1` and ride the
next planned slices (a Z-only move would zero the XY junction speed);
every slice re-commands Z explicitly because Klipper applies offsets
only to axes present in a command — an XY-only slice would never
realize them. Stationary corrections use `MOVE=1` as before (Z moves
and block ends still sync with `M400`).

---

## License

MIT — see [LICENSE](LICENSE).
