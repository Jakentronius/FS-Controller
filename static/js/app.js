/* Friction surfacing dashboard frontend. Polls /api/status at 2 Hz. */

const $ = (id) => document.getElementById(id);
const MAX_POINTS = 120;

let latestStatus = null;
let capZero = null;
let capLoad = null;
let calFormDirty = false;

// ---------------------------------------------------------------------- //
// Tabs

document.querySelectorAll(".tab-btn").forEach((btn) => {
  btn.addEventListener("click", () => {
    document.querySelectorAll(".tab-btn").forEach((b) => b.classList.remove("active"));
    document.querySelectorAll(".tab").forEach((t) => t.classList.remove("active"));
    btn.classList.add("active");
    $("tab-" + btn.dataset.tab).classList.add("active");
    if (btn.dataset.tab === "jobs") { refreshJobs(); refreshLogs(); }
    if (btn.dataset.tab === "tuning") refreshProfiles();
    if (btn.dataset.tab === "calibration") refreshMeshList();
    setTuningActive(btn.dataset.tab === "tuning");
  });
});

// ---------------------------------------------------------------------- //
// Charts

function makeChart(canvasId, label, color) {
  return new Chart($(canvasId).getContext("2d"), {
    type: "line",
    data: {
      labels: [],
      datasets: [{ label, borderColor: color, data: [], pointRadius: 0, tension: 0.1 }],
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      scales: {
        x: { display: false },
        y: { grid: { color: "#444" }, ticks: { color: "#bbb" } },
      },
      plugins: { legend: { labels: { color: "#fff" } } },
    },
  });
}

const forceChart = makeChart("forceChart", "Force", "#00bfff");
const zChart = makeChart("zChart", "Z-Height (mm)", "#4caf50");

function pushPoint(chart, label, value) {
  chart.data.labels.push(label);
  chart.data.datasets[0].data.push(value);
  if (chart.data.labels.length > MAX_POINTS) {
    chart.data.labels.shift();
    chart.data.datasets[0].data.shift();
  }
  chart.update();
}

// ---------------------------------------------------------------------- //
// API helpers

async function post(url, body) {
  const opts = { method: "POST" };
  if (body !== undefined) {
    opts.headers = { "Content-Type": "application/json" };
    opts.body = JSON.stringify(body);
  }
  const res = await fetch(url, opts);
  return res.json();
}

// ---------------------------------------------------------------------- //
// Status polling

async function poll() {
  let data;
  try {
    const res = await fetch("/api/status");
    data = await res.json();
  } catch (e) {
    setPill("pill-klipper", "Dashboard: CONNECTION LOST", "bad");
    return;
  }

  const firstPoll = latestStatus === null;
  latestStatus = data;

  // One rendering error must never freeze the whole UI (e.g. mixed
  // cached assets after a deploy): render each section independently.
  try {
    renderStatus(data, firstPoll);
  } catch (e) {
    console.error("render error (stale cache? hard-refresh):", e);
    setPill("pill-klipper", "UI ERROR — hard refresh the page", "bad");
  }
}

function renderStatus(data, firstPoll) {

  // Status pills
  const klipperOk = !["offline", "unknown", "error", "shutdown"].includes(data.klipper_state);
  setPill("pill-klipper", `Klipper: ${data.klipper_state.toUpperCase()}`,
    klipperOk ? "ok" : "bad");
  setPill("pill-sensor",
    data.sensor_ok
      ? `Sensor: ${data.sample_hz.toFixed(0)} Hz${data.is_tared ? " (tared)" : ""}`
      : `Sensor: FAULT — ${data.sensor_error}`,
    data.sensor_ok ? "ok" : "bad");
  setPill("pill-autoz",
    data.control_enabled
      ? `AUTO Z: ON @ ${fmt(data.force_target)} ${data.config.units_label}`
      : "AUTO Z: OFF",
    data.control_enabled ? "warn" : "");
  setPill("pill-log",
    data.logging ? `Log: RECORDING` : "Log: off",
    data.logging ? "warn" : "");
  const job = data.job;
  if (job && job.state === "running") {
    setPill("pill-job", `Job: ${job.file} ${job.row}/${job.total}`, "warn");
  } else if (job) {
    setPill("pill-job", `Job: ${job.state}`, job.state === "done" ? "ok" : "bad");
  } else {
    setPill("pill-job", "Job: none", "");
  }

  // Readouts
  $("ro-force").innerText = fmt(data.force_units);
  $("ro-raw").innerText = data.raw_adc;
  $("ro-x").innerText = data.x.toFixed(2);
  $("ro-y").innerText = data.y.toFixed(2);
  $("ro-z").innerText = data.z.toFixed(3);
  $("ro-zoff").innerText = data.z_offset.toFixed(3);
  ["x", "y", "z"].forEach((axis) => {
    const homed = (data.homed_axes || "").includes(axis);
    const badge = $("homed-" + axis);
    badge.innerText = homed ? "homed" : "unhomed";
    badge.className = "badge " + (homed ? "badge-ok" : "badge-warn");
    $("ro-" + axis).classList.toggle("dim", !homed);
  });
  // Work-origin display: machine coords of the origin + current work coords
  const [ox, oy] = data.xy_offset || [0, 0];
  if (ox || oy) {
    $("workzero-status").innerText =
      `origin at machine (${ox.toFixed(1)}, ${oy.toFixed(1)}) — ` +
      `head at work (${(data.x - ox).toFixed(2)}, ${(data.y - oy).toFixed(2)})`;
  } else {
    $("workzero-status").innerText = "machine coords (no work origin set)";
  }

  $("cal-live-raw").innerText = data.raw_adc;
  document.querySelectorAll(".units-label").forEach((el) => {
    el.innerText = data.config.units_label;
  });

  // Charts
  const t = data.time.toFixed(1);
  pushPoint(forceChart, t, data.force_units);
  pushPoint(zChart, t, data.z);
  forceChart.data.datasets[0].label = `Force (${data.config.units_label})`;

  // Buttons
  const btnLog = $("btn-log");
  btnLog.innerText = data.logging ? "Stop Log" : "Start Log";
  btnLog.className = data.logging ? "btn btn-red" : "btn btn-green";
  $("log-file").innerText = data.logging ? `logging → ${data.log_file}` : "";

  const btnCtrl = $("btn-control");
  btnCtrl.innerText = data.control_enabled ? "AUTO Z: ON" : "AUTO Z: OFF";
  btnCtrl.className = data.control_enabled ? "btn btn-red" : "btn btn-gray";

  // Job status
  updateJobStatus(data.job);

  // Auto-tune status
  updateAutotune(data.autotune);

  // Surface mesh status
  updateMesh(data.mesh);

  // Touch-off status
  updateTouchoff(data.touchoff);

  // Safety fault banner
  const banner = $("fault-banner");
  if (data.fault) {
    banner.hidden = false;
    $("fault-text").innerText = `SAFETY FAULT: ${data.fault.reason} ` +
      `(${new Date(data.fault.time * 1000).toLocaleTimeString()})`;
  } else {
    banner.hidden = true;
  }

  // Tare-drift hint: idle, tared, but force isn't reading zero
  const idle = !data.control_enabled && !(data.job && data.job.state === "running")
    && data.autotune.state !== "running" && data.mesh.probe.state !== "running"
    && data.touchoff.state !== "running";
  if (data.sensor_ok && data.is_tared && idle
      && Math.abs(data.force_units) > data.config.force_deadband) {
    setPill("pill-sensor",
      `Sensor: ${data.sample_hz.toFixed(0)} Hz — zero drift ${fmt(data.force_units)} ${data.config.units_label}? re-tare`,
      "warn");
  }

  // One-time initialization from server state
  if (firstPoll) {
    $("input-target").value = data.force_target;
    fillCalForm(data.config);
    fillTuneForm(data.config);
    refreshJobs();
  }
}

function fmt(v) {
  if (v === null || v === undefined) return "--";
  return Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(2);
}

function setPill(id, text, kind) {
  const el = $(id);
  el.innerText = text;
  el.className = "pill" + (kind ? " pill-" + kind : "");
}

setInterval(poll, 500);
poll();

// ---------------------------------------------------------------------- //
// Dashboard actions

$("btn-log").addEventListener("click", () =>
  post(latestStatus && latestStatus.logging ? "/api/log/stop" : "/api/log/start"));

$("btn-tare").addEventListener("click", () => post("/api/tare"));
$("btn-clear-tare").addEventListener("click", () => post("/api/tare/clear"));

$("btn-control").addEventListener("click", () => {
  const enabled = latestStatus ? latestStatus.control_enabled : false;
  post("/api/control", { enabled: !enabled });
});

$("btn-set-target").addEventListener("click", () => {
  const v = parseFloat($("input-target").value);
  if (!isNaN(v)) post("/api/control", { target: v });
});

// ---------------------------------------------------------------------- //
// Jogging & homing

let jogStep = 1;

document.querySelectorAll(".btn-step").forEach((btn) =>
  btn.addEventListener("click", () => {
    document.querySelectorAll(".btn-step").forEach((b) => b.classList.remove("active"));
    btn.classList.add("active");
    jogStep = parseFloat(btn.dataset.step);
  }));

document.querySelectorAll("[data-jog-axis]").forEach((btn) =>
  btn.addEventListener("click", async () => {
    const dist = jogStep * parseInt(btn.dataset.jogSign, 10);
    const res = await post("/api/jog", { axis: btn.dataset.jogAxis, dist });
    $("jog-status").innerText = res.status === "ok" ? "" : res.message;
  }));

async function home(axes, btn) {
  btn.disabled = true;
  $("jog-status").innerText = `Homing ${axes.toUpperCase()}...`;
  try {
    const res = await post("/api/home", { axes });
    $("jog-status").innerText = res.status === "ok" ? "" : res.message;
  } finally {
    btn.disabled = false;
  }
}

$("btn-xyzero").addEventListener("click", async () => {
  const res = await post("/api/workzero/set");
  $("jog-status").innerText = res.status === "ok" ? "" : res.message;
});

$("btn-xyzero-clear").addEventListener("click", async () => {
  const res = await post("/api/workzero/clear");
  $("jog-status").innerText = res.status === "ok" ? "" : res.message;
});

$("btn-goto").addEventListener("click", async () => {
  const payload = {};
  ["x", "y", "z"].forEach((a) => {
    const v = parseFloat($("goto-" + a).value);
    if (!isNaN(v)) payload[a] = v;
  });
  if (!Object.keys(payload).length) {
    $("jog-status").innerText = "Enter at least one coordinate.";
    return;
  }
  const btn = $("btn-goto");
  btn.disabled = true;
  $("jog-status").innerText = "Moving...";
  try {
    const res = await post("/api/goto", payload);
    $("jog-status").innerText = res.status === "ok" ? "" : res.message;
  } finally {
    btn.disabled = false;
  }
});

$("btn-home-all").addEventListener("click", (e) => home("all", e.target));
$("btn-home-xy").addEventListener("click", (e) => home("xy", e.target));
$("btn-home-z").addEventListener("click", (e) => home("z", e.target));

$("btn-estop").addEventListener("click", () => {
  if (confirm("EMERGENCY STOP: this halts Klipper (M112) and requires a firmware restart. Continue?")) {
    post("/api/estop");
  }
});

$("btn-fw-restart").addEventListener("click", async () => {
  if (!confirm("FIRMWARE_RESTART Klipper? Homing state will be lost.")) return;
  await post("/api/klipper/restart");
});

$("btn-fault-clear").addEventListener("click", () => post("/api/fault/clear"));

$("btn-park").addEventListener("click", async () => {
  if (!latestStatus) return;
  const cfg = latestStatus.config;
  const res = await post("/api/goto",
    { x: cfg.park_x, y: cfg.park_y, z: cfg.park_z });
  $("jog-status").innerText = res.status === "ok" ? "" : res.message;
});

// ---------------------------------------------------------------------- //
// Calibration

const CAL_FIELDS = [
  "counts_per_unit", "units_label", "retract_mm", "retract_feedrate",
  "max_rod_consumption", "rod_change_retract", "rod_change_induction_s",
  "max_force", "z_floor", "park_x", "park_y", "park_z",
  "travel_feedrate", "z_travel_feedrate",
];

function fillCalForm(cfg) {
  if (calFormDirty) return;
  CAL_FIELDS.forEach((f) => { $("cfg-" + f).value = cfg[f]; });
}

document.querySelectorAll("#cal-form input").forEach((el) =>
  el.addEventListener("input", () => { calFormDirty = true; }));

$("cal-form").addEventListener("submit", async (e) => {
  e.preventDefault();
  const payload = {};
  CAL_FIELDS.forEach((f) => {
    const v = $("cfg-" + f).value;
    payload[f] = f === "units_label" ? v : parseFloat(v);
  });
  const res = await post("/api/calibration", payload);
  $("cal-status").innerText =
    res.status === "ok" ? "Saved ✓" : `Error: ${res.message}`;
  calFormDirty = false;
  setTimeout(() => { $("cal-status").innerText = ""; }, 3000);
});

$("btn-cap-zero").addEventListener("click", () => {
  if (!latestStatus) return;
  capZero = latestStatus.raw_adc;
  $("cap-zero-val").innerText = capZero;
});

$("btn-cap-load").addEventListener("click", () => {
  if (!latestStatus) return;
  capLoad = latestStatus.raw_adc;
  $("cap-load-val").innerText = capLoad;
});

$("btn-compute-factor").addEventListener("click", () => {
  const mass = parseFloat($("input-known-mass").value);
  if (capZero === null || capLoad === null || !mass) {
    alert("Capture both points and enter the known mass first.");
    return;
  }
  const factor = (capLoad - capZero) / mass;
  $("cfg-counts_per_unit").value = factor;
  calFormDirty = true;
});

// ---------------------------------------------------------------------- //
// Tuning tab: live PID tuning with a fast chart

const TUNE_FIELDS = [
  "force_target", "pid_kp", "pid_ki", "pid_kd", "force_deadband",
  "control_cooldown", "z_step_max", "force_avg_samples",
  "force_ramp_rate", "backlash_comp",
];

const TUNE_MAX_POINTS = 300; // ~30 s at 10 Hz

const tuneChart = new Chart($("tuneChart").getContext("2d"), {
  type: "line",
  data: {
    labels: [],
    datasets: [
      { label: "Force", borderColor: "#00bfff", data: [], pointRadius: 0, tension: 0.1 },
      { label: "Target", borderColor: "#ff9800", borderDash: [6, 4], data: [], pointRadius: 0 },
      { label: "Z Offset (mm)", borderColor: "#4caf50", data: [], pointRadius: 0, yAxisID: "y2" },
    ],
  },
  options: {
    responsive: true,
    maintainAspectRatio: false,
    animation: false,
    scales: {
      x: { display: false },
      y: { grid: { color: "#444" }, ticks: { color: "#bbb" } },
      y2: { position: "right", grid: { drawOnChartArea: false }, ticks: { color: "#4caf50" } },
    },
    plugins: { legend: { labels: { color: "#fff" } } },
  },
});

let tuneTimer = null;

function setTuningActive(active) {
  if (active && tuneTimer === null) {
    tuneTimer = setInterval(tunePoll, 100);
  } else if (!active && tuneTimer !== null) {
    clearInterval(tuneTimer);
    tuneTimer = null;
  }
}

async function tunePoll() {
  let d;
  try {
    d = await (await fetch("/api/force")).json();
  } catch (e) {
    return;
  }

  const t = d.time.toFixed(1);
  tuneChart.data.labels.push(t);
  tuneChart.data.datasets[0].data.push(d.force);
  tuneChart.data.datasets[1].data.push(d.target);
  tuneChart.data.datasets[2].data.push(d.z_offset);
  if (tuneChart.data.labels.length > TUNE_MAX_POINTS) {
    tuneChart.data.labels.shift();
    tuneChart.data.datasets.forEach((ds) => ds.data.shift());
  }
  tuneChart.update();

  const btn = $("btn-tune-control");
  btn.innerText = d.control_enabled ? "AUTO Z: ON" : "AUTO Z: OFF";
  btn.className = d.control_enabled ? "btn btn-red" : "btn btn-gray";

  const pid = d.pid || {};
  if (!d.control_enabled || pid.error === undefined) {
    $("pid-readout").innerText = "controller idle";
  } else if (pid.in_deadband) {
    $("pid-readout").innerText =
      `IN DEADBAND — error ${pid.error.toFixed(1)}, I ${pid.i.toFixed(4)} mm (bleeding)` +
      (pid.mesh ? ` | mesh ff ${pid.mesh.toFixed(4)} mm` : "");
  } else {
    const ramping = pid.target_eff !== undefined
      && pid.target_eff < d.force_target;
    $("pid-readout").innerText =
      `error ${pid.error.toFixed(1)} | P ${pid.p.toFixed(4)} + I ${pid.i.toFixed(4)}` +
      ` + D ${pid.d.toFixed(4)} = out ${pid.out.toFixed(4)} mm/step` +
      (ramping ? ` | ramping to ${pid.target_eff.toFixed(0)}` : "") +
      (pid.mesh ? ` | mesh ff ${pid.mesh.toFixed(4)} mm` : "") +
      (pid.sched && pid.sched !== 1 ? ` | sched ×${pid.sched.toFixed(2)}` : "");
  }
}

function fillTuneForm(cfg) {
  TUNE_FIELDS.forEach((f) => { $("tune-" + f).value = cfg[f]; });
}

let tuneApplyTimer = null;

function applyTuning(immediate) {
  clearTimeout(tuneApplyTimer);
  const doApply = async () => {
    const payload = {};
    TUNE_FIELDS.forEach((f) => {
      const v = parseFloat($("tune-" + f).value);
      if (!isNaN(v)) payload[f] = v;
    });
    const res = await post("/api/tuning", payload);
    $("tune-status").innerText =
      res.status === "ok" ? "applied (not saved)" : `Error: ${res.message}`;
  };
  if (immediate) doApply();
  else tuneApplyTimer = setTimeout(doApply, 400);
}

TUNE_FIELDS.forEach((f) =>
  $("tune-" + f).addEventListener("input", () => applyTuning(false)));

$("btn-tune-save").addEventListener("click", async () => {
  const payload = { persist: true };
  TUNE_FIELDS.forEach((f) => {
    const v = parseFloat($("tune-" + f).value);
    if (!isNaN(v)) payload[f] = v;
  });
  const res = await post("/api/tuning", payload);
  $("tune-status").innerText =
    res.status === "ok" ? "saved to config ✓" : `Error: ${res.message}`;
});

$("btn-tune-revert").addEventListener("click", async () => {
  const saved = await (await fetch("/api/tuning/saved")).json();
  fillTuneForm(saved);
  applyTuning(true);
  $("tune-status").innerText = "reverted to saved values (applied)";
});

$("btn-tune-control").addEventListener("click", () => {
  const enabled = latestStatus ? latestStatus.control_enabled : false;
  post("/api/control", { enabled: !enabled });
});

function stepTarget(sign) {
  const step = parseFloat($("tune-step-size").value) || 0;
  const field = $("tune-force_target");
  field.value = (parseFloat(field.value) || 0) + sign * step;
  applyTuning(true);
}

$("btn-target-up").addEventListener("click", () => stepTarget(1));
$("btn-target-down").addEventListener("click", () => stepTarget(-1));

// ---------------------------------------------------------------------- //
// Auto-tune

let atChart = null;
let atPrevState = "idle";
let atRecommended = null;
let atCurveTimer = 0;

function renderAtChart(points) {
  const toXY = (pts) => (pts || []).map(([d, f]) => ({ x: d, y: f }));
  if (!atChart) {
    atChart = new Chart($("atChart").getContext("2d"), {
      type: "scatter",
      data: {
        datasets: [
          { label: "Loading", borderColor: "#00bfff", backgroundColor: "#00bfff",
            showLine: true, pointRadius: 2, data: [] },
          { label: "Unloading", borderColor: "#ff9800", backgroundColor: "#ff9800",
            showLine: true, pointRadius: 2, data: [] },
        ],
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        scales: {
          x: { type: "linear", grid: { color: "#444" }, ticks: { color: "#bbb" },
               title: { display: true, text: "descent from start (mm)", color: "#bbb" } },
          y: { grid: { color: "#444" }, ticks: { color: "#bbb" },
               title: { display: true, text: "force", color: "#bbb" } },
        },
        plugins: { legend: { labels: { color: "#fff" } } },
      },
    });
  }
  atChart.data.datasets[0].data = toXY(points.load);
  atChart.data.datasets[1].data = toXY(points.unload);
  atChart.update();
}

async function refreshAtCurve(showResults) {
  let at;
  try {
    at = await (await fetch("/api/autotune")).json();
  } catch (e) {
    return;
  }
  if (at.points) renderAtChart(at.points);
  if (showResults && at.recommended) {
    atRecommended = at.recommended;
    $("btn-profile-save-at").disabled = false;
    const r = at.results || {};
    const u = latestStatus ? latestStatus.config.units_label : "";
    const holds = r.holds || (r.hold ? [r.hold] : []);
    const holdTxt = holds.map((h) =>
      `@${h.target ?? "?"}: mean <b>${h.mean}</b>, RMS <b>${h.rms_error}</b>, ` +
      `σ <b>${h.std}</b>` + (h.note ? ` — <i>${h.note}</i>` : "")
    ).join(" &middot; ");
    const rec = at.recommended;
    const curveTxt = (rec.stiffness_curve || []).length
      ? `<br>Gain schedule S(F): ` + rec.stiffness_curve
          .map(([f, s]) => `${f}→<b>${s}</b>`).join(", ") +
        ` ${u}/mm (ref ${rec.schedule_ref_force}) — saved with Apply`
      : "";
    const sr = r.step_response;
    let srTxt;
    if (sr && sr.ok) {
      const relax = sr.relax_frac > 0.15
        ? `, <b>${Math.round(sr.relax_frac * 100)}%</b> relaxes away by 2 s`
        : "";
      const windup = sr.windup_mm
        ? `, reversal windup <b>${sr.windup_mm}</b> mm (reversals arrive ` +
          `${Math.round(sr.reversal_ratio * 100)}%-sized)`
        : "";
      srTxt =
        `Loop lag: dead time <b>${Math.round(sr.dead_time_s * 1000)}</b> ms` +
        ` &plusmn;${Math.round(sr.dead_time_std_s * 1000)}, ` +
        `T63 <b>${Math.round(sr.time_constant_s * 1000)}</b> ms ` +
        `(${sr.reps} steps of ${sr.dz_mm} mm) &rarr; correcting ` +
        `<b>${sr.corr_frac_per_cycle ?? "?"}</b> of the error per cycle` +
        relax + windup + `<br>`;
    } else {
      const why = sr && sr.steps
        ? sr.steps.map((s) =>
            `${s.dir}: ${s.fail || "ok"}` +
            (s.peak != null ? ` (peak ${s.peak}/${s.df_exp ?? "?"})` : "")
          ).join("; ")
        : "no data";
      srTxt = `Loop lag: <i>step test inconclusive — legacy 0.3/S gain ` +
        `used</i> — ${why}<br>`;
    }
    if (r.note) srTxt += `<i>${r.note}</i><br>`;
    $("at-results-text").innerHTML =
      `Sensor noise σ: <b>${r.noise_std}</b> ${u} &middot; ` +
      `Contact at Z <b>${r.contact_z}</b> mm &middot; ` +
      `Stiffness: <b>${r.stiffness}</b> ${u}/mm &middot; ` +
      `Hysteresis: <b>${r.hysteresis_mm ?? "?"}</b> mm<br>` +
      srTxt +
      `Hold (Kp ${holds.length ? holds[0].kp_used : "?"}): ${holdTxt}<br>` +
      `Recommended: Kp <b>${rec.pid_kp}</b>, Ki <b>${rec.pid_ki}</b>, ` +
      `Kd <b>0</b>, ` +
      `Deadband <b>${rec.force_deadband}</b> ${u}, ` +
      `Max step <b>${rec.z_step_max}</b> mm` +
      (rec.control_cooldown != null
        ? `, Eval period <b>${rec.control_cooldown}</b> s` : "") +
      (rec.backlash_comp != null
        ? `, Lash comp <b>${rec.backlash_comp}</b> mm` : "") +
      curveTxt;
    $("at-results").hidden = false;
  }
}

function updateAutotune(at) {
  if (!at) return;
  const running = at.state === "running";
  $("btn-at-start").disabled = running;
  $("btn-at-abort").disabled = !running;

  let txt = at.state;
  if (running) txt = `${at.phase}: ${at.message || "..."}`;
  else if (at.state === "error") txt = `error — ${at.error}`;
  else if (at.state === "aborted") txt = "aborted (retracted)";
  else if (at.state === "done") txt = "done ✓";
  $("at-status").innerText = txt;

  // Refresh the probe curve live (throttled to ~1 Hz) during load/unload
  if (running && ["load", "unload"].includes(at.phase)) {
    if (Date.now() - atCurveTimer > 1000) {
      atCurveTimer = Date.now();
      refreshAtCurve(false);
    }
  }

  // On completion, pull the full results once
  if (atPrevState === "running" && at.state !== "running") {
    refreshAtCurve(at.state === "done");
  }
  atPrevState = at.state;
}

$("btn-at-start").addEventListener("click", async () => {
  const checklist =
    "Start auto-tune?\n\n" +
    "Confirm before continuing:\n" +
    "  • Rigid NON-SPINNING rod in the collet\n" +
    "  • Head parked a few mm ABOVE the substrate, near build-area center\n" +
    "  • Scale calibrated (counts per unit saved)\n" +
    "  • Spindle OFF\n\n" +
    "The head will descend slowly until it feels contact.";
  if (!confirm(checklist)) return;
  const payload = { sweep: $("at-sweep").checked ? 1 : 0 };
  const pf = parseFloat($("at-probe_force").value);
  const mt = parseFloat($("at-max_travel").value);
  if (!isNaN(pf)) payload.probe_force = pf;
  if (!isNaN(mt)) payload.max_travel = mt;
  $("at-results").hidden = true;
  atRecommended = null;
  $("btn-profile-save-at").disabled = true;
  const res = await post("/api/autotune/start", payload);
  if (res.status !== "ok") $("at-status").innerText = `error — ${res.message}`;
});

$("btn-at-abort").addEventListener("click", () => post("/api/autotune/abort"));

$("btn-at-load").addEventListener("click", () => {
  if (!atRecommended) return;
  Object.entries(atRecommended).forEach(([k, v]) => {
    const el = $("tune-" + k);
    if (el) el.value = v;
  });
  applyTuning(true);
  $("tune-status").innerText = "auto-tune values applied (not saved)";
});

$("btn-at-apply").addEventListener("click", async () => {
  if (!atRecommended) return;
  const res = await post("/api/autotune/apply", { persist: true });
  if (res.status === "ok") {
    fillTuneForm(res.config);
    $("tune-status").innerText = "auto-tune values saved to config ✓";
  } else {
    $("tune-status").innerText = `Error: ${res.message}`;
  }
});

// ---------------------------------------------------------------------- //
// Surface mesh

let meshPrevState = "idle";
let meshGridLoaded = false;

function renderMeshGrid(mesh) {
  const wrap = $("mesh-grid");
  if (!mesh) {
    wrap.innerHTML = "";
    return;
  }
  const zvals = mesh.z.flat();
  const zmin = Math.min(...zvals);
  const range = Math.max(...zvals) - zmin || 1;
  let html = "<table><tr><th></th>";
  mesh.xs.forEach((x) => { html += `<th>X ${x.toFixed(1)}</th>`; });
  html += "</tr>";
  // Largest Y on top so the table matches the machine layout
  for (let iy = mesh.ys.length - 1; iy >= 0; iy--) {
    html += `<tr><th>Y ${mesh.ys[iy].toFixed(1)}</th>`;
    mesh.z[iy].forEach((z) => {
      const t = (z - zmin) / range;           // 0 = low spot, 1 = high spot
      const hue = 240 - 240 * t;              // blue -> red
      html += `<td style="background:hsl(${hue},45%,30%)">${z.toFixed(3)}</td>`;
    });
    html += "</tr>";
  }
  html += "</table>";
  wrap.innerHTML = html;
}

async function refreshMeshGrid() {
  try {
    const d = await (await fetch("/api/mesh")).json();
    renderMeshGrid(d.mesh);
  } catch (e) { /* ignore */ }
}

function updateMesh(m) {
  if (!m) return;
  const probe = m.probe || {};
  const running = probe.state === "running";
  $("btn-mesh-start").disabled = running;
  $("btn-mesh-abort").disabled = !running;
  $("btn-mesh-clear").disabled = running || !m.exists;

  let txt = probe.state || "idle";
  if (running) {
    txt = `${probe.phase}: ${probe.message || "..."}` +
      (probe.total ? ` — ${probe.done}/${probe.total} points` : "");
  } else if (probe.state === "error") txt = `error — ${probe.error}`;
  else if (probe.state === "aborted") txt = "aborted (retracted)";
  else if (probe.state === "done") txt = probe.message || "done ✓";
  $("mesh-status").innerText = txt;

  $("mesh-info").innerText = m.exists
    ? `Mesh${m.name ? ` '${m.name}'` : ""}: ` +
      `${new Date(m.created * 1000).toLocaleString()}, ` +
      `range ${m.range_mm.toFixed(3)} mm` +
      (m.enabled ? "" : " (DISABLED)")
    : "No mesh stored.";

  const cb = $("mesh-enabled");
  if (document.activeElement !== cb) cb.checked = m.enabled;

  $("btn-mesh-save").disabled = running || !m.exists;

  if ((meshPrevState === "running" && probe.state === "done") ||
      (m.exists && !meshGridLoaded)) {
    meshGridLoaded = true;
    refreshMeshGrid();
    refreshMeshList();
  }
  if (!m.exists) renderMeshGrid(null);
  meshPrevState = probe.state || "idle";
}

function meshLibMsg(text) {
  $("mesh-lib-status").innerText = text;
  setTimeout(() => { $("mesh-lib-status").innerText = ""; }, 6000);
}

async function refreshMeshList() {
  let data;
  try {
    data = await (await fetch("/api/meshes")).json();
  } catch (e) {
    return;
  }
  const tbody = document.querySelector("#mesh-table tbody");
  tbody.innerHTML = "";
  if (!data.meshes.length) {
    tbody.innerHTML =
      "<tr><td colspan='6' class='hint'>No saved meshes.</td></tr>";
    return;
  }
  data.meshes.forEach((m) => {
    const tr = document.createElement("tr");
    const probed = m.created
      ? new Date(m.created * 1000).toLocaleString() : "--";
    const grid = m.points_x && m.points_y
      ? `${m.points_x}×${m.points_y} / ${m.size_x_mm}×${m.size_y_mm} mm`
      : (m.points_per_side
         ? `${m.points_per_side}×${m.points_per_side} / ${m.size_mm} mm`
         : "--");
    const center = m.center
      ? `(${m.center[0]}, ${m.center[1]})` : "--";
    tr.innerHTML =
      `<td>${m.name}</td><td>${probed}</td><td>${grid}</td>` +
      `<td>${m.range_mm != null ? m.range_mm.toFixed(3) : "--"}</td>` +
      `<td>${center}</td><td></td>`;
    const cell = tr.lastElementChild;

    const btnLoad = document.createElement("button");
    btnLoad.className = "btn btn-green";
    btnLoad.innerText = "Load";
    btnLoad.addEventListener("click", async () => {
      const res = await post("/api/meshes/load", { name: m.name });
      if (res.status !== "ok") {
        meshLibMsg(`Error: ${res.message}`);
        return;
      }
      meshLibMsg(`mesh '${res.message}' is now active ✓`);
      refreshMeshGrid();
    });
    cell.appendChild(btnLoad);

    const btnDel = document.createElement("button");
    btnDel.className = "btn btn-gray";
    btnDel.innerText = "Delete";
    btnDel.addEventListener("click", async () => {
      if (!confirm(`Delete saved mesh '${m.name}'?`)) return;
      const res = await post("/api/meshes/delete", { name: m.name });
      if (res.status !== "ok") meshLibMsg(`Error: ${res.message}`);
      refreshMeshList();
    });
    cell.appendChild(btnDel);

    tbody.appendChild(tr);
  });
}

$("btn-mesh-save").addEventListener("click", async () => {
  const name = $("mesh-name").value.trim();
  if (!name) {
    meshLibMsg("Enter a mesh name first.");
    return;
  }
  const res = await post("/api/meshes/save", { name });
  if (res.status === "ok") {
    meshLibMsg(`saved mesh '${res.message}' ✓`);
    refreshMeshList();
  } else {
    meshLibMsg(`Error: ${res.message}`);
  }
});

// ---------------------------------------------------------------------- //
// Single-point touch-off

function updateTouchoff(to) {
  if (!to) return;
  const running = to.state === "running";
  $("btn-touchoff").disabled = running;
  $("btn-touchoff-abort").disabled = !running;

  const last = to.last;
  $("ro-surface").innerText =
    last && last.surface_z != null ? last.surface_z.toFixed(3) : "--";

  if (running) {
    $("touchoff-status").innerText = `${to.phase}: ${to.message || "..."}`;
  } else if (to.state === "error") {
    $("touchoff-status").innerText = `error — ${to.error}`;
  } else if (to.state === "aborted") {
    $("touchoff-status").innerText = "touch-off aborted (retracted)";
  } else if (to.state === "done" && last) {
    let msg = `Surface Z ${last.surface_z.toFixed(3)} mm at ` +
      `(${last.x}, ${last.y}), ${new Date(last.created * 1000).toLocaleTimeString()}`;
    if (last.delta != null) {
      msg += ` — ${last.delta >= 0 ? "+" : ""}${last.delta.toFixed(3)} mm vs previous` +
        ` (wear / new rod length)`;
    }
    $("touchoff-status").innerText = msg;
  } else if (last && last.surface_z != null) {
    $("touchoff-status").innerText =
      `Last touch-off: Z ${last.surface_z.toFixed(3)} mm at (${last.x}, ${last.y}), ` +
      new Date(last.created * 1000).toLocaleString();
  }
}

$("btn-touchoff").addEventListener("click", async () => {
  if (!confirm(
    "Touch off at the current XY?\n\n" +
    "The head will descend slowly until it feels the surface, then park " +
    "just above it. Rod/tool must be able to safely take light contact " +
    "(spindle off).")) return;
  const payload = {};
  const hv = parseFloat($("touchoff-hover").value);
  if (!isNaN(hv)) payload.hover_mm = hv;
  const res = await post("/api/touchoff/start", payload);
  if (res.status !== "ok") $("touchoff-status").innerText = `error — ${res.message}`;
});

$("btn-touchoff-abort").addEventListener("click", () => post("/api/touchoff/abort"));

$("btn-mesh-start").addEventListener("click", async () => {
  const checklist =
    "Start mesh probe?\n\n" +
    "Confirm before continuing:\n" +
    "  • Rigid NON-SPINNING rod in the collet, spindle OFF\n" +
    "  • Head a few mm ABOVE the substrate at the CENTER of the region\n" +
    "  • The whole grid square fits on the substrate and inside travel limits\n\n" +
    "The head will touch off every grid point (a few minutes).";
  if (!confirm(checklist)) return;
  const payload = {};
  [["mesh-size_x_mm", "size_x_mm"], ["mesh-size_y_mm", "size_y_mm"],
   ["mesh-points_x", "points_x"], ["mesh-points_y", "points_y"],
   ["mesh-max_travel", "max_travel"]].forEach(([id, key]) => {
    const v = parseFloat($(id).value);
    if (!isNaN(v)) payload[key] = v;
  });
  const res = await post("/api/mesh/start", payload);
  if (res.status !== "ok") $("mesh-status").innerText = `error — ${res.message}`;
});

$("btn-mesh-abort").addEventListener("click", () => post("/api/mesh/abort"));

$("btn-mesh-clear").addEventListener("click", async () => {
  if (!confirm("Delete the stored surface mesh?")) return;
  await post("/api/mesh/clear");
  meshGridLoaded = false;
});

$("mesh-enabled").addEventListener("change", (e) =>
  post("/api/tuning", { mesh_enabled: e.target.checked ? 1 : 0, persist: true }));

// ---------------------------------------------------------------------- //
// Mechtrode profiles

function profileMsg(text) {
  $("profile-status").innerText = text;
  setTimeout(() => { $("profile-status").innerText = ""; }, 6000);
}

async function refreshProfiles() {
  let data;
  try {
    data = await (await fetch("/api/profiles")).json();
  } catch (e) {
    return;
  }
  const u = latestStatus ? latestStatus.config.units_label : "";
  const tbody = document.querySelector("#profile-table tbody");
  tbody.innerHTML = "";
  if (!data.profiles.length) {
    tbody.innerHTML =
      "<tr><td colspan='7' class='hint'>No profiles saved yet.</td></tr>";
    return;
  }
  data.profiles.forEach((p) => {
    const tr = document.createElement("tr");
    const created = p.created
      ? new Date(p.created * 1000).toLocaleString() : "--";
    const stiff = p.stiffness != null ? `${p.stiffness} ${u}/mm` : "--";
    tr.innerHTML =
      `<td>${p.name}</td><td>${created}</td><td>${p.source || "--"}</td>` +
      `<td>${stiff}</td><td>${p.pid_kp ?? "--"}</td>` +
      `<td>${p.force_deadband ?? "--"}</td><td></td>`;
    const cell = tr.lastElementChild;

    const btnLoad = document.createElement("button");
    btnLoad.className = "btn btn-green";
    btnLoad.innerText = "Load";
    btnLoad.addEventListener("click", async () => {
      const res = await post("/api/profiles/load", { name: p.name });
      if (res.status !== "ok") {
        profileMsg(`Error: ${res.message}`);
        return;
      }
      fillTuneForm(res.config);
      let msg = `loaded '${res.message}' — applied & saved ✓`;
      if (res.warnings && res.warnings.length) {
        msg += ` — WARNING: ${res.warnings.join("; ")}`;
      }
      profileMsg(msg);
      $("tune-status").innerText = `profile '${res.message}' active`;
    });
    cell.appendChild(btnLoad);

    const btnDel = document.createElement("button");
    btnDel.className = "btn btn-gray";
    btnDel.innerText = "Delete";
    btnDel.addEventListener("click", async () => {
      if (!confirm(`Delete profile '${p.name}'?`)) return;
      const res = await post("/api/profiles/delete", { name: p.name });
      if (res.status !== "ok") profileMsg(`Error: ${res.message}`);
      refreshProfiles();
    });
    cell.appendChild(btnDel);

    tbody.appendChild(tr);
  });
}

async function saveProfile(source) {
  const name = $("profile-name").value.trim();
  if (!name) {
    profileMsg("Enter a profile name first.");
    return;
  }
  const res = await post("/api/profiles/save", { name, source });
  if (res.status === "ok") {
    profileMsg(`saved profile '${res.message}' ✓`);
    refreshProfiles();
  } else {
    profileMsg(`Error: ${res.message}`);
  }
}

$("btn-profile-save-at").addEventListener("click", () => saveProfile("autotune"));
$("btn-profile-save-cur").addEventListener("click", () => saveProfile("current"));

// ---------------------------------------------------------------------- //
// Jobs

async function refreshJobs() {
  let data;
  try {
    data = await (await fetch("/api/jobs")).json();
  } catch (e) {
    return;
  }
  const tbody = document.querySelector("#job-table tbody");
  tbody.innerHTML = "";
  if (!data.jobs.length) {
    tbody.innerHTML = "<tr><td colspan='4' class='hint'>No job files uploaded.</td></tr>";
    return;
  }
  data.jobs.forEach((j) => {
    const tr = document.createElement("tr");
    const mtime = new Date(j.mtime * 1000).toLocaleString();
    let info = j.valid ? String(j.steps) : `invalid: ${j.error}`;
    const pf = j.preflight || { errors: [], warnings: [] };
    if (j.valid && pf.errors.length) {
      info += ` <span class="pf-err" title="${esc(pf.errors.join("\n"))}">⛔ ${pf.errors.length}</span>`;
    }
    if (j.valid && pf.warnings.length) {
      info += ` <span class="pf-warn" title="${esc(pf.warnings.join("\n"))}">⚠ ${pf.warnings.length}</span>`;
    }
    tr.innerHTML =
      `<td>${j.name}</td><td>${info}</td><td>${mtime}</td><td></td>`;
    const btn = document.createElement("button");
    btn.className = "btn btn-green";
    btn.innerText = "Run";
    btn.disabled = !j.valid || pf.errors.length > 0;
    btn.title = pf.errors.length ? "Preflight errors — hover the ⛔ badge" : "";
    btn.addEventListener("click", () => runJob(j.name, pf));
    tr.lastElementChild.appendChild(btn);
    tbody.appendChild(tr);
  });
}

function esc(s) {
  return String(s).replace(/&/g, "&amp;").replace(/</g, "&lt;")
    .replace(/>/g, "&gt;").replace(/"/g, "&quot;");
}

async function runJob(name, preflight) {
  const startStep = parseInt($("job-start-step").value, 10) || 1;
  let msg = `Run job "${name}"? AUTO Z force control will be enabled and ` +
    "force data will be recorded to a run log.";
  if (preflight && preflight.warnings.length) {
    msg += "\n\nPREFLIGHT WARNINGS:\n• " + preflight.warnings.join("\n• ");
  }
  if (startStep > 1) {
    msg += `\n\nRESUMING AT STEP ${startStep}: the machine will move ` +
      "directly to that step's coordinates. Make sure the path there is clear.";
  }
  if (!confirm(msg)) return;
  const res = await post("/api/jobs/run",
    { name, notes: $("job-notes").value, start_step: startStep });
  if (res.status !== "ok") alert(`Could not start job: ${res.message}`);
  else $("job-start-step").value = 1;
}

// ---------------------------------------------------------------------- //
// Run logs

let logChart = null;
let jobPrevState = null;

async function refreshLogs() {
  let data;
  try {
    data = await (await fetch("/api/logs")).json();
  } catch (e) {
    return;
  }
  const tbody = document.querySelector("#log-table tbody");
  tbody.innerHTML = "";
  if (!data.logs.length) {
    tbody.innerHTML = "<tr><td colspan='6' class='hint'>No logs yet.</td></tr>";
    return;
  }
  data.logs.forEach((lg) => {
    const tr = document.createElement("tr");
    const date = new Date(lg.mtime * 1000).toLocaleString();
    const size = lg.size > 1048576
      ? (lg.size / 1048576).toFixed(1) + " MB"
      : Math.round(lg.size / 1024) + " kB";
    const noteHint = (lg.meta.notes || "").split("\n")[0].slice(0, 40);
    tr.innerHTML =
      `<td>${lg.name}${lg.active ? " <b class='accent'>(recording)</b>" : ""}</td>` +
      `<td>${lg.meta.job || (lg.is_run ? "?" : "manual")}</td>` +
      `<td>${date}</td><td>${size}</td><td class="hint">${noteHint}</td><td></td>`;
    const cell = tr.lastElementChild;

    const btnView = document.createElement("button");
    btnView.className = "btn btn-blue";
    btnView.innerText = "View";
    btnView.addEventListener("click", () => viewLog(lg.name));
    cell.appendChild(btnView);

    const aDl = document.createElement("a");
    aDl.href = "/api/logs/download/" + encodeURIComponent(lg.name);
    aDl.download = lg.name;
    const btnDl = document.createElement("button");
    btnDl.className = "btn btn-gray";
    btnDl.innerText = "Download";
    aDl.appendChild(btnDl);
    cell.appendChild(aDl);

    const btnDel = document.createElement("button");
    btnDel.className = "btn btn-gray";
    btnDel.innerText = "Delete";
    btnDel.disabled = lg.active;
    btnDel.addEventListener("click", async () => {
      if (!confirm(`Delete log '${lg.name}'?`)) return;
      await post("/api/logs/delete", { name: lg.name });
      refreshLogs();
    });
    cell.appendChild(btnDel);

    tbody.appendChild(tr);
  });
}

async function viewLog(name) {
  let text;
  try {
    text = await (await fetch("/api/logs/download/" + encodeURIComponent(name))).text();
  } catch (e) {
    return;
  }
  const metaLines = [];
  let header = null;
  const rows = [];
  for (const ln of text.split("\n")) {
    if (ln.startsWith("#")) { metaLines.push(ln.slice(1).trim()); continue; }
    if (!ln.trim()) continue;
    const parts = ln.split(",");
    if (!header) { header = parts.map((h) => h.trim()); continue; }
    rows.push(parts);
  }
  if (!header || !rows.length) return;

  const col = (n) => header.indexOf(n);
  const ti = col("Time_epoch_s");
  let fi = col("Force_units");
  const tgt = col("Force_target");
  const zo = col("Z_offset");
  let forceLabel = "Force";
  if (fi < 0) { fi = col("Raw_ADC"); forceLabel = "Raw ADC (old format)"; }

  const stride = Math.max(1, Math.floor(rows.length / 1500));
  const t0 = parseFloat(rows[0][ti]);
  const labels = [], force = [], target = [], zoff = [];
  for (let i = 0; i < rows.length; i += stride) {
    const r = rows[i];
    labels.push((parseFloat(r[ti]) - t0).toFixed(1));
    force.push(parseFloat(r[fi]));
    target.push(tgt >= 0 ? parseFloat(r[tgt]) : null);
    zoff.push(zo >= 0 ? parseFloat(r[zo]) : null);
  }

  $("log-viewer").hidden = false;
  $("log-viewer-title").innerText = name;
  $("log-viewer-meta").innerText = metaLines.join("\n") || "(no header metadata)";

  if (logChart) logChart.destroy();
  logChart = new Chart($("logChart").getContext("2d"), {
    type: "line",
    data: {
      labels,
      datasets: [
        { label: forceLabel, borderColor: "#00bfff", data: force, pointRadius: 0, tension: 0.1 },
        { label: "Target", borderColor: "#ff9800", borderDash: [6, 4], data: target, pointRadius: 0 },
        { label: "Z Offset (mm)", borderColor: "#4caf50", data: zoff, pointRadius: 0, yAxisID: "y2" },
      ],
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      scales: {
        x: { ticks: { color: "#bbb", maxTicksLimit: 12 },
             title: { display: true, text: "elapsed (s)", color: "#bbb" } },
        y: { grid: { color: "#444" }, ticks: { color: "#bbb" } },
        y2: { position: "right", grid: { drawOnChartArea: false }, ticks: { color: "#4caf50" } },
      },
      plugins: { legend: { labels: { color: "#fff" } } },
    },
  });
  $("log-viewer").scrollIntoView({ behavior: "smooth", block: "nearest" });
}

function updateJobStatus(job) {
  // Refresh the log list when a job just finished (its run log closed)
  const state = job ? job.state : null;
  if (jobPrevState === "running" && state !== "running") refreshLogs();
  jobPrevState = state;
  const txt = $("job-status-text");
  const errEl = $("job-error-text");
  const bar = $("job-progress");
  const btnAbort = $("btn-abort");
  const btnClear = $("btn-job-clear");

  if (!job) {
    txt.innerText = "No job running.";
    errEl.hidden = true;
    bar.style.width = "0";
    btnAbort.disabled = true;
    $("btn-pause").disabled = true;
    btnClear.hidden = true;
    return;
  }

  const pct = job.total ? Math.round((100 * job.row) / job.total) : 0;
  const paused = job.state === "running" && job.pause;
  bar.style.width = pct + "%";
  bar.style.background = paused ? "#ffb300"
    : job.state === "running" ? "#4caf50"
    : job.state === "done" ? "#007acc" : "#f44336";
  btnAbort.disabled = job.state !== "running";
  $("btn-pause").disabled = job.state !== "running" || !!job.pause;
  btnClear.hidden = job.state === "running";

  const manualStages = ["pausing", "waiting_resume", "resume"];
  const pauseKind = paused && manualStages.includes(job.pause.stage)
    ? "PAUSED" : "PAUSED (rod change)";
  let msg = `${job.file}: ${paused ? pauseKind : job.state.toUpperCase()}` +
    ` — step ${job.row}/${job.total}`;
  if (job.step_desc && job.state === "running" && !paused) msg += ` (${job.step_desc})`;
  if (job.state === "running" && latestStatus &&
      latestStatus.config.max_rod_consumption > 0) {
    msg += ` — rod used ${latestStatus.rod_consumption.toFixed(2)}` +
      `/${latestStatus.config.max_rod_consumption} mm`;
  }
  txt.innerText = msg;
  errEl.hidden = !job.error;
  errEl.innerText = job.error ? `⛔ ${job.error}` : "";

  // Rod-change pause prompt
  const box = $("job-pause-box");
  const btnConfirm = $("btn-job-confirm");
  if (paused) {
    box.hidden = false;
    $("job-pause-msg").innerText = job.pause.message || job.pause.stage;
    const waiting = String(job.pause.stage || "").startsWith("waiting");
    btnConfirm.disabled = !waiting;
    btnConfirm.innerText =
      job.pause.stage === "waiting_rod_change" ? "Rod Changed — Touch Off"
      : job.pause.stage === "waiting_spindle" ? "Spindle On — Re-engage"
      : job.pause.stage === "waiting_resume" ? "Resume Job"
      : "Working...";
  } else {
    box.hidden = true;
    btnConfirm.disabled = true;
  }
}

$("btn-job-confirm").addEventListener("click", async () => {
  $("btn-job-confirm").disabled = true;
  await post("/api/jobs/confirm");
});

$("btn-upload").addEventListener("click", async () => {
  const input = $("job-file");
  if (!input.files.length) {
    $("upload-status").innerText = "Choose a file first.";
    return;
  }
  const form = new FormData();
  form.append("file", input.files[0]);
  const res = await (await fetch("/api/jobs/upload", { method: "POST", body: form })).json();
  if (res.status === "ok") {
    const pf = res.preflight || { errors: [], warnings: [] };
    let txt = `Uploaded ${res.name} (${res.steps} steps) ✓`;
    if (pf.errors.length) txt += ` — ⛔ ${pf.errors.length} preflight error(s): ${pf.errors[0]}`;
    else if (pf.warnings.length) txt += ` — ⚠ ${pf.warnings.length} warning(s): ${pf.warnings[0]}`;
    $("upload-status").innerText = txt;
  } else {
    $("upload-status").innerText = `Error: ${res.message}`;
  }
  if (res.status === "ok") {
    input.value = "";
    refreshJobs();
  }
});

$("btn-abort").addEventListener("click", () => post("/api/jobs/abort"));
$("btn-pause").addEventListener("click", () => post("/api/jobs/pause"));
$("btn-job-clear").addEventListener("click", () => post("/api/jobs/clear"));
