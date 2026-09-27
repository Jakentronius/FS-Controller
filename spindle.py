"""Spindle speed control for the Gpenny spindle's H100 VFD over Modbus RTU.

Stage A control (see gpenny-vfd-logger/CONTROL.md): the drive's frequency
setpoint comes from this software (F002 = 2) while RUN/STOP stay on the VFD
keypad (F001 = 0). So this module can only choose the speed; it cannot
start the spindle, and the panel STOP key always works.

A background thread polls the drive at ~2 Hz (output frequency, current,
rpm, voltages, heatsink temperature, the commanded setpoint and the run
coils) and re-reads the control-mode parameters every few seconds, so the
dashboard can show whether Stage A is actually in force.

Bus sharing: every transaction takes the same advisory flock on the tty
that vfd_logger.py / vfd_control.py use, so those tools can run alongside
the dashboard without interleaving Modbus frames.
"""

import fcntl
import os
import threading
import time

try:
    import minimalmodbus
    import serial
    MODBUS_AVAILABLE = True
except ImportError:  # pragma: no cover - dev machines without the libs
    MODBUS_AVAILABLE = False

PORT = os.environ.get("SPINDLE_PORT", "/dev/ttyUSB0")   # DSD TECH SH-U11F
SLAVE_ADDRESS = 1          # F163
BAUDRATE = 19200           # F164 = 2
TIMEOUT_S = 0.3
POLE_PAIRS = 1             # 2-pole spindle (F143 = 2): rpm = 60 * Hz

# Input registers 0x0000.. (function 04H); the H100 answers at most 4 per read
FIELDS = ("freq_hz", "set_freq_hz", "current_a", "rpm",
          "dc_bus_v", "ac_v", "temp_c")
SCALES = (0.1, 0.1, 0.01, 1.0, 0.1, 0.1, 0.1)
REG_MAX_PER_READ = 4
REG_SETPOINT = 0x0201      # given frequency, 0.1 Hz, RAM; active when F002 = 2
COIL_STOP = 0x004B         # only honoured when F001 = 2
COIL_NAMES = ("operation", "jog", "reverse", "in_operation", "in_jog",
              "in_reverse", "braking", "freq_tracking")
RUN_SOURCE = {0: "keypad", 1: "terminals", 2: "communication"}
FREQ_SOURCE = {0: "keypad F003", 1: "AI1", 2: "communication",
               3: "keypad potentiometer", 4: "AI2", 5: "pulse input",
               6: "AI1+AI2"}

POLL_S = 0.5
CONFIG_REFRESH_S = 5.0
RETRY_OPEN_S = 5.0


def hz_to_rpm(hz):
    return hz * 60.0 / POLE_PAIRS


def rpm_to_hz(rpm):
    return rpm * POLE_PAIRS / 60.0


class SpindleController:
    def __init__(self, port=PORT):
        self.port = port
        self._inst = None
        self._lock = threading.Lock()      # one bus transaction at a time
        self._running = True
        self._errors = 0
        self._last_config_t = 0.0
        self.state = {
            "available": False, "error": None, "port": port,
            "freq_hz": 0.0, "set_freq_hz": 0.0, "current_a": 0.0,
            "rpm": 0.0, "dc_bus_v": 0.0, "ac_v": 0.0, "temp_c": 0.0,
            "setpoint_hz": 0.0, "setpoint_rpm": 0.0,
            "running": False, "coils": {},
            "run_source": None, "freq_source": None,
            "run_source_name": "?", "freq_source_name": "?",
            "min_hz": 0.0, "max_hz": 400.0, "accel_s": None, "decel_s": None,
            "stage_a": False, "last_update": None,
        }

    # ------------------------------------------------------------------ #
    def start(self):
        threading.Thread(target=self._poll_loop, daemon=True).start()

    def shutdown(self):
        self._running = False

    def status(self):
        return dict(self.state)

    # ------------------------------------------------------------------ #
    # Bus plumbing

    def _open(self):
        if not MODBUS_AVAILABLE:
            raise RuntimeError("minimalmodbus / pyserial not installed")
        inst = minimalmodbus.Instrument(self.port, SLAVE_ADDRESS,
                                        mode=minimalmodbus.MODE_RTU)
        inst.serial.baudrate = BAUDRATE
        inst.serial.bytesize = 8
        inst.serial.parity = serial.PARITY_NONE
        inst.serial.stopbits = 1
        inst.serial.timeout = TIMEOUT_S
        inst.clear_buffers_before_each_transaction = True
        return inst

    def _close(self):
        try:
            if self._inst is not None:
                self._inst.serial.close()
        except Exception:
            pass
        self._inst = None

    def _txn(self, fn):
        """Run fn(inst) as one locked bus transaction (thread + flock)."""
        with self._lock:
            if self._inst is None:
                self._inst = self._open()
            fd = self._inst.serial.fileno()
            fcntl.flock(fd, fcntl.LOCK_EX)
            try:
                return fn(self._inst)
            finally:
                fcntl.flock(fd, fcntl.LOCK_UN)

    def _fail(self, exc):
        self._errors += 1
        if self._errors >= 3 or not self.state["available"]:
            self.state["available"] = False
            self.state["error"] = f"{type(exc).__name__}: {exc}"[:160]
            with self._lock:
                self._close()

    # ------------------------------------------------------------------ #
    # Polling

    def _read_live(self):
        def go(inst):
            raw = []
            for start in range(0, len(FIELDS), REG_MAX_PER_READ):
                n = min(REG_MAX_PER_READ, len(FIELDS) - start)
                raw.extend(inst.read_registers(start, n, functioncode=4))
            sp = inst.read_registers(REG_SETPOINT, 1, functioncode=3)[0]
            bits = inst.read_bits(0, 8, functioncode=1)
            return raw, sp, bits
        raw, sp, bits = self._txn(go)
        for name, val, scale in zip(FIELDS, raw, SCALES):
            self.state[name] = round(val * scale, 3)
        self.state["setpoint_hz"] = sp / 10.0
        self.state["setpoint_rpm"] = round(hz_to_rpm(sp / 10.0), 1)
        coils = dict(zip(COIL_NAMES, (bool(b) for b in bits)))
        self.state["coils"] = coils
        self.state["running"] = bool(coils.get("in_operation"))

    def _read_config(self):
        def go(inst):
            f = inst.read_registers(0, 4, functioncode=3)      # F000..F003
            g = inst.read_registers(5, 1, functioncode=3)      # F005
            h = inst.read_registers(11, 4, functioncode=3)     # F011..F014
            i = inst.read_registers(15, 1, functioncode=3)     # F015
            return f, g, h, i
        f, g, h, i = self._txn(go)
        s = self.state
        s["run_source"], s["freq_source"] = f[1], f[2]
        s["run_source_name"] = RUN_SOURCE.get(f[1], "?")
        s["freq_source_name"] = FREQ_SOURCE.get(f[2], "?")
        s["max_hz"], s["min_hz"] = g[0] / 10.0, h[0] / 10.0
        s["accel_s"], s["decel_s"] = h[3] / 10.0, i[0] / 10.0
        s["stage_a"] = (f[2] == 2)

    def _poll_loop(self):
        while self._running:
            t0 = time.time()
            try:
                if (t0 - self._last_config_t) >= CONFIG_REFRESH_S:
                    self._read_config()
                    self._last_config_t = t0
                self._read_live()
                self._errors = 0
                self.state["available"] = True
                self.state["error"] = None
                self.state["last_update"] = t0
            except Exception as e:      # keep the thread alive on bus faults
                self._fail(e)
                if self._inst is None:
                    time.sleep(RETRY_OPEN_S)
                    continue
            time.sleep(max(0.05, POLL_S - (time.time() - t0)))

    # ------------------------------------------------------------------ #
    # Commands

    def set_hz(self, hz):
        """Write the frequency setpoint and verify it. (ok, message)."""
        try:
            hz = float(hz)
        except (TypeError, ValueError):
            return False, "Bad speed value"
        if not self.state["available"]:
            return False, f"Spindle drive offline ({self.state['error']})"
        if not self.state["stage_a"]:
            return False, ("Drive ignores the serial setpoint: F002 = "
                           f"{self.state['freq_source']} "
                           f"({self.state['freq_source_name']}); set F002 = 2")
        lo, hi = self.state["min_hz"], self.state["max_hz"]
        if hz != 0 and not (lo <= hz <= hi):
            return False, (f"{hz:g} Hz ({hz_to_rpm(hz):.0f} rpm) is outside "
                           f"the drive limits {lo:g}..{hi:g} Hz")
        counts = int(round(hz * 10))

        def go(inst):
            inst.write_register(REG_SETPOINT, counts, functioncode=6)
            return inst.read_registers(REG_SETPOINT, 1, functioncode=3)[0]
        try:
            back = self._txn(go)
        except Exception as e:
            self._fail(e)
            return False, f"Setpoint write failed: {e}"
        if back != counts:
            return False, f"Drive read back {back / 10:.1f} Hz, not {hz:g} Hz"
        self.state["setpoint_hz"] = back / 10.0
        self.state["setpoint_rpm"] = round(hz_to_rpm(back / 10.0), 1)
        if hz == 0:
            return True, "Setpoint 0 Hz: spindle ramping down"
        return True, f"Setpoint {hz:g} Hz ({hz_to_rpm(hz):.0f} rpm)"

    def set_rpm(self, rpm):
        try:
            return self.set_hz(rpm_to_hz(float(rpm)))
        except (TypeError, ValueError):
            return False, "Bad speed value"

    def stop(self):
        """Setpoint to 0 Hz (ramps down over F015) and try the STOP coil,
        which the drive only honours under comms run control."""
        ok, msg = self.set_hz(0.0)
        if not ok:
            return ok, msg
        try:
            self._txn(lambda inst: inst.write_bit(COIL_STOP, 1, functioncode=5))
            return True, "Setpoint 0 Hz and STOP accepted"
        except Exception:
            return True, ("Setpoint 0 Hz (ramping down over the drive's decel "
                          "time). Keypad still owns RUN/STOP: press STOP on "
                          "the panel for a full stop.")
