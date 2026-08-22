"""Flask backend for the friction surfacing dashboard."""

import io
import os
import time
import zipfile

from flask import (Flask, jsonify, render_template, request, send_file,
                   send_from_directory)
from werkzeug.utils import secure_filename

import hardware

app = Flask(__name__)
# LAN app, frequently redeployed: make browsers revalidate static files
# so stale cached JS never wedges the UI against a newer API.
app.config["SEND_FILE_MAX_AGE_DEFAULT"] = 0
machine = hardware.Machine()


@app.route("/")
def index():
    # Cache-bust static assets per app start so browsers never run a
    # stale app.js against a newer backend.
    return render_template("index.html", asset_v=int(machine.start_time))


# ---------------------------------------------------------------------- #
# Live status

@app.get("/api/status")
def api_status():
    return jsonify({
        "time": time.time() - machine.start_time,
        "raw_adc": machine.raw_adc,
        "force_units": machine.force_units(),
        "x": machine.position[0],
        "y": machine.position[1],
        "z": machine.position[2],
        "z_offset": machine.z_offset,
        "xy_offset": machine.xy_offset,
        "klipper_state": machine.klipper_state,
        "homed_axes": machine.homed_axes,
        "sensor_ok": machine.sensor_ok,
        "sensor_error": machine.sensor_error,
        "sample_hz": machine.sample_hz,
        "is_tared": machine.is_tared,
        "logging": machine.logging,
        "log_file": os.path.basename(machine.log_path) if machine.log_path else None,
        "control_enabled": machine.control_enabled,
        "force_target": machine.force_target,
        "rod_consumption": machine.rod_consumption(),
        "fault": machine.fault,
        "job": machine.job,
        "config": machine.config,
        # points can grow to a few hundred pairs; fetch them from
        # /api/autotune instead of shipping them at 2 Hz
        "autotune": {k: v for k, v in machine.autotune.items()
                     if k != "points"},
        "mesh": {
            "exists": machine.mesh is not None,
            "enabled": float(machine.config.get("mesh_enabled", 1.0)) != 0.0,
            "name": machine.mesh.get("name") if machine.mesh else None,
            "created": machine.mesh.get("created") if machine.mesh else None,
            "range_mm": machine.mesh.get("range_mm") if machine.mesh else None,
            "probe": machine.mesh_probe,
        },
        "touchoff": machine.touchoff,
    })


# ---------------------------------------------------------------------- #
# Dashboard actions

@app.post("/api/log/start")
def api_log_start():
    path = machine.start_log()
    return jsonify({"status": "ok", "file": os.path.basename(path)})


@app.post("/api/log/stop")
def api_log_stop():
    machine.stop_log()
    return jsonify({"status": "ok"})


@app.post("/api/tare")
def api_tare():
    if not machine.tare():
        return jsonify({"status": "error", "message": "Sensor not available"}), 400
    return jsonify({"status": "ok", "tare_offset": machine.tare_offset})


@app.post("/api/tare/clear")
def api_tare_clear():
    machine.clear_tare()
    return jsonify({"status": "ok"})


@app.post("/api/control")
def api_control():
    data = request.get_json(force=True, silent=True) or {}
    if "enabled" in data:
        if data["enabled"] and machine.fault:
            return jsonify({"status": "error",
                            "message": "Safety fault active — clear it "
                                       "first"}), 400
        machine.control_enabled = bool(data["enabled"])
    if "target" in data:
        try:
            machine.force_target = float(data["target"])
        except (TypeError, ValueError):
            return jsonify({"status": "error", "message": "Bad target"}), 400
    return jsonify({"status": "ok",
                    "control_enabled": machine.control_enabled,
                    "force_target": machine.force_target})


@app.get("/api/force")
def api_force():
    """Lightweight fast-poll endpoint for the tuning chart (~10 Hz)."""
    return jsonify({
        "time": time.time() - machine.start_time,
        "force": machine.force_units(),
        "avg": machine.force_avg(),
        "target": machine.force_target,
        "z_offset": machine.z_offset,
        "control_enabled": machine.control_enabled,
        "pid": machine.pid_state,
    })


@app.post("/api/tuning")
def api_tuning():
    """Apply controller parameters to the running machine WITHOUT persisting.
    Pass persist=true to also save to config.json."""
    data = request.get_json(force=True, silent=True) or {}
    persist = bool(data.pop("persist", False))
    try:
        machine.update_config(data, persist=persist)
    except (TypeError, ValueError) as e:
        return jsonify({"status": "error", "message": f"Bad value: {e}"}), 400
    return jsonify({"status": "ok", "persisted": persist,
                    "config": machine.config})


@app.get("/api/tuning/saved")
def api_tuning_saved():
    """The last values saved to disk (for Revert)."""
    return jsonify(hardware.load_config())


@app.post("/api/autotune/start")
def api_autotune_start():
    data = request.get_json(force=True, silent=True) or {}
    ok, msg = machine.start_autotune(data)
    return jsonify({"status": "ok" if ok else "error", "message": msg}), \
        200 if ok else 400


@app.post("/api/autotune/abort")
def api_autotune_abort():
    was_running = machine.abort_autotune()
    return jsonify({"status": "ok", "was_running": was_running})


@app.get("/api/autotune")
def api_autotune_get():
    """Full auto-tune state including the load/unload curve points."""
    return jsonify(machine.autotune)


@app.post("/api/autotune/apply")
def api_autotune_apply():
    """Apply (and by default persist) the recommended controller settings
    from the last completed auto-tune run."""
    rec = machine.autotune.get("recommended")
    if not rec:
        return jsonify({"status": "error",
                        "message": "No auto-tune recommendations available"}), 400
    data = request.get_json(force=True, silent=True) or {}
    persist = bool(data.get("persist", True))
    machine.update_config(dict(rec), persist=persist)
    return jsonify({"status": "ok", "persisted": persist,
                    "config": machine.config})


# ---------------------------------------------------------------------- #
# Surface mesh

@app.get("/api/mesh")
def api_mesh_get():
    return jsonify({"mesh": machine.mesh, "probe": machine.mesh_probe})


@app.post("/api/mesh/start")
def api_mesh_start():
    data = request.get_json(force=True, silent=True) or {}
    ok, msg = machine.start_mesh(data)
    return jsonify({"status": "ok" if ok else "error", "message": msg}), \
        200 if ok else 400


@app.post("/api/mesh/abort")
def api_mesh_abort():
    was_running = machine.abort_mesh()
    return jsonify({"status": "ok", "was_running": was_running})


@app.post("/api/mesh/clear")
def api_mesh_clear():
    if machine.mesh_probing():
        return jsonify({"status": "error",
                        "message": "Mesh probing is running"}), 400
    machine.clear_mesh()
    return jsonify({"status": "ok"})


@app.get("/api/meshes")
def api_meshes_list():
    return jsonify({"meshes": machine.list_meshes()})


@app.post("/api/meshes/save")
def api_meshes_save():
    data = request.get_json(force=True, silent=True) or {}
    ok, msg = machine.save_mesh_as(data.get("name", ""))
    return jsonify({"status": "ok" if ok else "error",
                    "message": msg}), 200 if ok else 400


@app.post("/api/meshes/load")
def api_meshes_load():
    if machine.mesh_probing():
        return jsonify({"status": "error",
                        "message": "Mesh probing is running"}), 400
    data = request.get_json(force=True, silent=True) or {}
    ok, msg = machine.load_mesh_named(data.get("name", ""))
    return jsonify({"status": "ok" if ok else "error",
                    "message": msg}), 200 if ok else 400


@app.post("/api/meshes/delete")
def api_meshes_delete():
    data = request.get_json(force=True, silent=True) or {}
    ok, msg = machine.delete_mesh_named(data.get("name", ""))
    return jsonify({"status": "ok" if ok else "error",
                    "message": msg}), 200 if ok else 400


# ---------------------------------------------------------------------- #
# Single-point touch-off

@app.post("/api/touchoff/start")
def api_touchoff_start():
    data = request.get_json(force=True, silent=True) or {}
    ok, msg = machine.start_touchoff(data)
    return jsonify({"status": "ok" if ok else "error", "message": msg}), \
        200 if ok else 400


@app.post("/api/touchoff/abort")
def api_touchoff_abort():
    was_running = machine.abort_touchoff()
    return jsonify({"status": "ok", "was_running": was_running})


# ---------------------------------------------------------------------- #
# Mechtrode calibration profiles

@app.get("/api/profiles")
def api_profiles_list():
    return jsonify({"profiles": machine.list_profiles()})


@app.post("/api/profiles/save")
def api_profiles_save():
    data = request.get_json(force=True, silent=True) or {}
    source = data.get("source", "current")
    if source not in ("current", "autotune"):
        return jsonify({"status": "error", "message": "Bad source"}), 400
    ok, msg = machine.save_profile(data.get("name", ""), source)
    return jsonify({"status": "ok" if ok else "error",
                    "message": msg}), 200 if ok else 400


@app.post("/api/profiles/load")
def api_profiles_load():
    data = request.get_json(force=True, silent=True) or {}
    ok, msg, warnings = machine.load_profile(data.get("name", ""))
    if not ok:
        return jsonify({"status": "error", "message": msg}), 400
    return jsonify({"status": "ok", "message": msg,
                    "warnings": warnings, "config": machine.config})


@app.post("/api/profiles/delete")
def api_profiles_delete():
    data = request.get_json(force=True, silent=True) or {}
    ok, msg = machine.delete_profile(data.get("name", ""))
    return jsonify({"status": "ok" if ok else "error",
                    "message": msg}), 200 if ok else 400


@app.post("/api/jog")
def api_jog():
    data = request.get_json(force=True, silent=True) or {}
    try:
        ok, msg = machine.jog(data.get("axis", ""), data.get("dist", 0),
                              data.get("feed"))
    except (TypeError, ValueError):
        return jsonify({"status": "error", "message": "Bad jog request"}), 400
    return jsonify({"status": "ok" if ok else "error", "message": msg}), \
        200 if ok else 400


@app.post("/api/goto")
def api_goto():
    data = request.get_json(force=True, silent=True) or {}
    try:
        ok, msg = machine.goto(data.get("x"), data.get("y"), data.get("z"),
                               data.get("feed"))
    except (TypeError, ValueError):
        return jsonify({"status": "error", "message": "Bad coordinates"}), 400
    return jsonify({"status": "ok" if ok else "error", "message": msg}), \
        200 if ok else 400


@app.post("/api/workzero/set")
def api_workzero_set():
    ok, msg = machine.set_xy_zero()
    return jsonify({"status": "ok" if ok else "error", "message": msg}), \
        200 if ok else 400


@app.post("/api/workzero/clear")
def api_workzero_clear():
    ok, msg = machine.clear_xy_zero()
    return jsonify({"status": "ok" if ok else "error", "message": msg}), \
        200 if ok else 400


@app.post("/api/home")
def api_home():
    data = request.get_json(force=True, silent=True) or {}
    ok, msg = machine.home(data.get("axes", "all"))
    return jsonify({"status": "ok" if ok else "error", "message": msg}), \
        200 if ok else 400


# ---------------------------------------------------------------------- #
# Calibration

@app.get("/api/calibration")
def api_calibration_get():
    return jsonify(machine.config)


@app.post("/api/calibration")
def api_calibration_set():
    data = request.get_json(force=True, silent=True) or {}
    try:
        machine.update_config(data)
    except (TypeError, ValueError) as e:
        return jsonify({"status": "error", "message": f"Bad value: {e}"}), 400
    return jsonify({"status": "ok", "config": machine.config})


# ---------------------------------------------------------------------- #
# Jobs

@app.get("/api/jobs")
def api_jobs_list():
    return jsonify({"jobs": machine.list_jobs()})


@app.post("/api/jobs/upload")
def api_jobs_upload():
    f = request.files.get("file")
    if f is None or not f.filename:
        return jsonify({"status": "error", "message": "No file provided"}), 400
    name = secure_filename(f.filename)
    if not name.lower().endswith(hardware.JOB_EXTENSIONS):
        exts = ", ".join(hardware.JOB_EXTENSIONS)
        return jsonify({"status": "error",
                        "message": f"Accepted file types: {exts}"}), 400
    path = os.path.join(hardware.JOB_DIR, name)
    f.save(path)
    try:
        steps = hardware.parse_job_file(path)
    except ValueError as e:
        os.remove(path)
        return jsonify({"status": "error", "message": f"Invalid job file: {e}"}), 400
    return jsonify({"status": "ok", "name": name, "steps": len(steps),
                    "preflight": machine.check_job(steps)})


@app.post("/api/jobs/run")
def api_jobs_run():
    data = request.get_json(force=True, silent=True) or {}
    name = data.get("name", "")
    if not name:
        return jsonify({"status": "error", "message": "No job name provided"}), 400
    ok, msg = machine.start_job(name, notes=data.get("notes", ""),
                                start_step=data.get("start_step", 1))
    code = 200 if ok else 400
    return jsonify({"status": "ok" if ok else "error", "message": msg}), code


@app.post("/api/jobs/pause")
def api_jobs_pause():
    ok, msg = machine.request_pause()
    return jsonify({"status": "ok" if ok else "error",
                    "message": msg}), 200 if ok else 400


# ---------------------------------------------------------------------- #
# Logs (manual + auto-captured run logs)

@app.get("/api/logs")
def api_logs_list():
    return jsonify({"logs": machine.list_logs()})


@app.get("/api/logs/download/<path:name>")
def api_logs_download(name):
    name = os.path.basename(name)
    if not name.endswith(".csv"):
        return jsonify({"status": "error", "message": "Bad log name"}), 400
    return send_from_directory(hardware.LOG_DIR, name)


@app.post("/api/logs/delete")
def api_logs_delete():
    data = request.get_json(force=True, silent=True) or {}
    ok, msg = machine.delete_log(data.get("name", ""))
    return jsonify({"status": "ok" if ok else "error",
                    "message": msg}), 200 if ok else 400


@app.post("/api/jobs/confirm")
def api_jobs_confirm():
    ok, msg = machine.confirm_job()
    return jsonify({"status": "ok" if ok else "error",
                    "message": msg}), 200 if ok else 400


@app.post("/api/jobs/abort")
def api_jobs_abort():
    was_running = machine.abort_job()
    return jsonify({"status": "ok", "was_running": was_running})


@app.post("/api/jobs/clear")
def api_jobs_clear():
    ok, msg = machine.clear_job()
    return jsonify({"status": "ok" if ok else "error",
                    "message": msg}), 200 if ok else 400


@app.post("/api/estop")
def api_estop():
    ok = machine.estop()
    return jsonify({"status": "ok" if ok else "error"})


@app.post("/api/klipper/restart")
def api_klipper_restart():
    ok = machine.firmware_restart()
    return jsonify({"status": "ok" if ok else "error",
                    "message": "" if ok else "Moonraker unreachable"}), \
        200 if ok else 502


@app.post("/api/fault/clear")
def api_fault_clear():
    machine.clear_fault()
    return jsonify({"status": "ok"})


@app.get("/api/backup")
def api_backup():
    """Zip of all settings and calibration data (not logs)."""
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as z:
        for path in (hardware.CONFIG_PATH, hardware.MESH_PATH,
                     hardware.TOUCHOFF_PATH):
            if os.path.exists(path):
                z.write(path, os.path.basename(path))
        for dirpath, arcname in ((hardware.PROFILE_DIR, "profiles"),
                                 (hardware.MESH_DIR, "meshes"),
                                 (hardware.JOB_DIR, "jobs")):
            for fname in sorted(os.listdir(dirpath)):
                full = os.path.join(dirpath, fname)
                if os.path.isfile(full):
                    z.write(full, f"{arcname}/{fname}")
    buf.seek(0)
    return send_file(
        buf, mimetype="application/zip", as_attachment=True,
        download_name=time.strftime("dashboard_backup_%Y%m%d_%H%M%S.zip"))


if __name__ == "__main__":
    machine.start()
    print("\n[*] Friction surfacing dashboard: http://<pi-ip>:5000\n")
    # use_reloader=False: the reloader would spawn a second process and fight
    # over the I2C bus.
    app.run(host="0.0.0.0", port=5000, threaded=True, use_reloader=False)
