# FS Controller

Web control panel for a hobby CNC (Genmitsu 3018-class) converted for
micro friction surfacing, with closed-loop plunge-force control. Runs on
the machine's Raspberry Pi alongside Klipper / Moonraker / Mainsail.

Companion project: [FS CAM](https://github.com/Jakentronius/circuit_cam)
turns DXF/SVG drawings into annotated G-code jobs for this dashboard.

> **Safety:** this software presses a spinning rod into metal under
> hundreds of newtons. It is a personal research tool, published as-is with
> no warranty. Set the safety limits for *your* hardware before running
> anything and keep the E-STOP in reach.

## What it does

- Reads plunge force from 4 load cells via a NAU7802 ADC at ~80 Hz
- Regulates that force with a PID loop that nudges Klipper's gcode Z offset
- Runs toolpaths (CSV or annotated G-code) combining X/Y motion with force
  targets, force-waits and dwells
- Auto-tunes the PID from a measured contact-stiffness curve, with
  per-mechtrode profiles and optional gain scheduling over force
- Probes a surface mesh with the load cells and uses it as feedforward
- Handles mid-job rod changes (retract → swap → touch off → re-engage)
- Logs every run to CSV with a metadata header

## Hardware

| Part | Role |
|---|---|
| Genmitsu 3018-class frame | X/Y/Z motion |
| BTT SKR 1.4 Turbo + Klipper | Motion controller (driven via Moonraker HTTP) |
| MKS SERVO42C closed-loop stepper | Z axis |
| 4 × load cells + SparkFun Qwiic NAU7802 | Force measurement over I2C |
| Raspberry Pi | Klipper, Moonraker, Mainsail and this dashboard |

## Installation

On the Pi, with Klipper + Moonraker working (`http://127.0.0.1:7125`) and
I2C enabled:

```bash
git clone https://github.com/Jakentronius/FS-Controller.git ~/load_cells/app
python3 -m venv ~/load_cells/venv
~/load_cells/venv/bin/pip install -r ~/load_cells/app/requirements.txt

cd ~/load_cells/app && ~/load_cells/venv/bin/python app.py   # test run

sudo cp dashboard.service /etc/systemd/system/
sudo systemctl enable --now dashboard
```

Open `http://<pi-ip>:5000`. `config.json`, `logs/`, `jobs/`, `profiles/`
and mesh files are created at runtime and git-ignored.

```
app.py              Flask JSON API
hardware.py         sensor / telemetry / PID / job-runner threads
templates/, static/ vanilla-JS frontend (Chart.js)
dashboard.service   systemd unit
example_job.*       minimal CSV and G-code jobs
```

## Using it

**Header** — status pills for Klipper, sensor, force control, logging and
job. **E-STOP** sends `M112` (requires **FW Restart** after). A red fault
banner halts and retracts when force exceeds **Max force** or the sensor
stalls; press **Clear Fault** to continue.

**Dashboard** — live force / position readouts, jog pad, **Go to**,
homing, logging, tare, manual force target. **Touch Off** finds the
surface with the load cells and records it as the Z reference.
**Zero XY Here** sets a work origin so jobs are authored around `X0 Y0`.

**Calibration** — counts-per-unit scale factor (fit from a logged
multi-mass test, or the two-point helper), abort retract, travel feeds,
safety limits, rod-change settings, and surface mesh probing / saving.

**Tuning** — live PID fields (Kp, Ki, Kd, deadband, eval period, max
step, averaging, ramp) with a step-response chart. **Auto-Tune** probes
contact stiffness with a rigid non-spinning rod and derives conservative
gains; **Sweep** mode fits stiffness per force band and saves a gain
schedule. Save tunings as named **mechtrode profiles**.

**Jobs** — upload, preflight-check (bounds, Z floor, max force, feeds),
run, pause/resume, abort, or start at a given step. Every run is logged
to `logs/run_<job>_<timestamp>.csv` with your **Run notes** in the header.

## Job files

**CSV** — `X, Y, Target_Force_Units, Feedrate`; one move per row.

**Annotated G-code** (`.gcode`, `.nc`, `.g`) — normal G-code plus comment
directives handled by the job runner:

| Directive | Meaning |
|---|---|
| `;FORCE=<units>` | Force target for everything that follows (0 = free air) |
| `;WAIT_FORCE TIMEOUT=<s>` | Creep to first contact, then wait until force is within the deadband |
| `;DWELL=<s>` | Hold while force control keeps regulating (use instead of `G4`) |
| `;ZREF=SURFACE` | Job Z coordinates are relative to the last touch-off (required by FS CAM output) |

```gcode
;FORCE=0
G1 Z45 F600            ; safe height
G1 X25 Y30 F600
G1 Z10 F300            ; park above surface
;FORCE=800
;WAIT_FORCE TIMEOUT=30
;DWELL=10
;FORCE=0
G1 Z45 F600
```

Long moves are sliced into ~0.15 s segments so force corrections can ride
along mid-move; `G4`, `G28` and `SET_GCODE_OFFSET` inside a job are
flagged at preflight.

## Troubleshooting

| Symptom | Check |
|---|---|
| Sensor pill red | I2C wiring; `i2cdetect -y 1` should show 0x2A |
| Sample rate well below ~77 Hz | Pi CPU load or another I2C user |
| Klipper pill red | Klipper/Moonraker down — check Mainsail |
| Force negative when pressing down | Negate the calibration factor |
| Force oscillates under AUTO Z | Lower Kp/Ki, raise avg window or deadband |
| `WAIT_FORCE timed out` | Park too high, target unreachable, or controller too slow |

Logs: `sudo journalctl -u dashboard -f`.

## License

MIT — see [LICENSE](LICENSE).
