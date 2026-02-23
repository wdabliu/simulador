# -*- coding: ascii -*-
"""
grblHAL RTCP Testing Suite Unificado
=====================================
Consolida todos los tests de rtcp.c en un solo script con seleccion por CLI.
Requiere que grblHAL_sim.exe este compilado en build/.

Uso basico:
    python testing.py                    # Ejecuta todos los tests
    python testing.py -v                 # Modo verbose (muestra comunicacion TCP)

Ejecutar un grupo especifico:
    python testing.py -matematicas       # Cinematica inversa + formulas Python
    python testing.py -singularidad      # Gimbal Lock: proteccion A=90 (ALARM:12)
    python testing.py -funciones         # Diagnostico $RTCP + cache + settings
    python testing.py -mcodes            # M451/M450 toggle + warning
    python testing.py -bypass            # Bypass RTCP OFF = identidad
    python testing.py -coherencia        # Coherencia inv/directa + pivot Z
    python testing.py -feedrate          # Feedrate compensacion + segmentacion
    python testing.py -realtime          # Realtime report |RTCP:ON/OFF|
    python testing.py -l                 # Lista todos los grupos disponibles

Guardar resultado a archivo:
    python testing.py > resultado.txt 2>&1
    python testing.py -v > debug_completo.txt 2>&1
    python testing.py -singularidad > singularidad.txt 2>&1

Combinar grupos:
    python testing.py -matematicas -singularidad

Codigo de salida:
    0 = todos los tests pasaron
    1 = al menos un test fallo
    2 = error de ejecucion (simulador no arranca, conexion fallida, etc.)
"""

import argparse
import logging
import math
import os
import socket
import subprocess
import sys
import time

# =====================================================================
# CONFIG
# =====================================================================

SIM_EXE = r"c:\simulador\build\grblHAL_sim.exe"
CONFIG_FILE = r"c:\simulador\testing_config.ini"
PORT = 23
TOL = 0.05       # mm tolerancia general
TOL_MATH = 0.02  # mm tolerancia cinematica pura

log = logging.getLogger("rtcp_test")


def load_config(path):
    """Lee testing_config.ini y retorna lista de comandos + dict de settings."""
    commands = []
    settings = {}
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("["):
                continue
            # Separar comentario inline
            cmd = line.split("#")[0].strip()
            if not cmd:
                continue
            commands.append(cmd)
            # Guardar en dict si es $n=v
            if cmd.startswith("$") and "=" in cmd:
                key, val = cmd.split("=", 1)
                settings[key.strip()] = val.strip()
    return commands, settings


def get_pivot(settings):
    """Extrae pivot (X,Y,Z) del dict de settings."""
    px = float(settings.get("$640", "0"))
    py = float(settings.get("$641", "0"))
    pz = float(settings.get("$642", "150"))
    return (px, py, pz)


# Cargar config al importar para que PIVOT este disponible globalmente
_CONFIG_CMDS, _CONFIG_SETTINGS = load_config(CONFIG_FILE)
PIVOT = get_pivot(_CONFIG_SETTINGS)

# =====================================================================
# CONEXION TCP AL SIMULADOR
# =====================================================================

class Sim:
    def __init__(self):
        self.proc = None
        self.sock = None
        self.buf = b""

    def start(self):
        eeprom = os.path.join(os.path.dirname(SIM_EXE), "EEPROM.DAT")
        if os.path.exists(eeprom):
            os.remove(eeprom)
            log.info("EEPROM.DAT eliminado (defaults frescos)")

        cmd = [SIM_EXE, "-p", str(PORT), "-t", "0"]
        log.info("Lanzando: %s", " ".join(cmd))
        self.proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP
        )
        time.sleep(3.0)
        if self.proc.poll() is not None:
            err = self.proc.stderr.read().decode(errors="replace")
            raise RuntimeError("Simulador fallo: %s" % err)
        log.info("PID=%d", self.proc.pid)

        for attempt in range(5):
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(5.0)
                self.sock.connect(("127.0.0.1", PORT))
                log.info("TCP conectado")
                time.sleep(1.0)
                self._drain()
                return
            except (ConnectionRefusedError, socket.timeout, OSError):
                log.warning("Intento %d/5", attempt + 1)
                try:
                    self.sock.close()
                except Exception:
                    pass
                time.sleep(1.5)
        raise ConnectionError("No se pudo conectar al simulador")

    def _drain(self):
        self.sock.settimeout(1.0)
        try:
            while True:
                d = self.sock.recv(4096)
                if not d:
                    break
        except Exception:
            pass
        self.buf = b""

    def send(self, cmd):
        self.sock.sendall((cmd.strip() + "\r\n").encode())

    def recv(self, timeout=5.0):
        lines = []
        self.sock.settimeout(timeout)
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                d = self.sock.recv(4096)
                if not d:
                    break
                self.buf += d
                while b"\n" in self.buf:
                    raw, self.buf = self.buf.split(b"\n", 1)
                    line = raw.decode(errors="replace").strip().strip("\r")
                    if line:
                        lines.append(line)
                        log.debug("RX << %s", line)
                        if line.lower() == "ok" or line.lower().startswith("error"):
                            return lines
            except socket.timeout:
                break
        return lines

    def cmd(self, command, timeout=5.0, wait=0.2):
        if wait > 0:
            time.sleep(wait)
        self.send(command)
        time.sleep(0.1)
        return self.recv(timeout)

    def wait_stable(self, max_wait=20.0, interval=0.5):
        prev = None
        deadline = time.time() + max_wait
        while time.time() < deadline:
            time.sleep(interval)
            resp = self.cmd("$RTCP", timeout=5, wait=0.2)
            motor = None
            found = False
            for line in resp:
                if "Motor Position" in line:
                    found = True
                    continue
                if found and "X =" in line:
                    motor = line.strip()
                    break
            if motor and motor == prev:
                return True
            prev = motor
        return False

    def unlock(self):
        self._drain()
        # Soft reset (Ctrl+X) saca de ALARM
        self.sock.sendall(b"\x18")
        time.sleep(1.0)
        self._drain()
        # $X unlock por si queda en estado lock
        resp = self.cmd("$X", timeout=3, wait=0.3)
        log.debug("Unlock: %s", resp)
        time.sleep(0.3)
        self.cmd("G90 G21", wait=0.1)
        return resp

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        if self.proc:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=5)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass
            log.info("Proceso terminado")


# =====================================================================
# HELPERS DE PARSEO
# =====================================================================

def get_rtcp_data(lines):
    data = {
        "mode": None, "pivot": {}, "offsets": {},
        "tcp": None, "motor": None, "cache": None,
        "a_deg": None, "c_deg": None,
    }
    section = None
    for line in lines:
        if "RTCP Mode:" in line:
            data["mode"] = "ON" if "ON" in line else "OFF"
        elif "Pivot Point:" in line:
            section = "pivot"
        elif "Axis Offsets" in line:
            section = "offsets"
        elif "TCP Position" in line:
            section = "tcp"
        elif "Motor Position:" in line:
            section = "motor"
        elif "Rotary Axes:" in line:
            section = "rotary"
        elif "Trig Cache:" in line:
            data["cache"] = "Valid" if "Valid" in line else "Invalid"
        elif section == "pivot" and "$64" in line:
            for axis in ["X", "Y", "Z"]:
                if " %s = " % axis in line:
                    val = line.split("%s = " % axis)[1].split()[0]
                    data["pivot"][axis] = float(val)
        elif section == "offsets" and "$64" in line:
            for axis in ["Y", "Z"]:
                if " %s = " % axis in line:
                    val = line.split("%s = " % axis)[1].split()[0]
                    data["offsets"][axis] = float(val)
        elif section in ("tcp", "motor") and "X =" in line:
            vals = {}
            parts = line.split()
            for i, p in enumerate(parts):
                if p in ("X", "Y", "Z") and i + 2 < len(parts) and parts[i + 1] == "=":
                    try:
                        vals[p] = float(parts[i + 2])
                    except ValueError:
                        pass
            if len(vals) == 3:
                if section == "tcp":
                    data["tcp"] = vals
                else:
                    data["motor"] = vals
        elif section == "rotary":
            if "A =" in line:
                data["a_deg"] = float(line.split("A = ")[1].split()[0])
            elif "C =" in line:
                data["c_deg"] = float(line.split("C = ")[1].split()[0])
    return data


def get_motor(lines):
    motor_section = False
    for line in lines:
        if "Motor Position" in line:
            motor_section = True
            continue
        if motor_section and "X =" in line:
            parts = line.split()
            vals = {}
            for i, p in enumerate(parts):
                if p in ("X", "Y", "Z") and i + 2 < len(parts) and parts[i + 1] == "=":
                    try:
                        vals[p] = float(parts[i + 2])
                    except ValueError:
                        pass
            if len(vals) == 3:
                return (vals["X"], vals["Y"], vals["Z"])
    return None


def has_text(lines, text):
    return any(text in l for l in lines)


def rtcp_inverse(x, y, z, a_deg, c_deg, px=0, py=0, pz=150):
    a = math.radians(a_deg)
    c = math.radians(c_deg)
    ca, sa = math.cos(a), math.sin(a)
    cc, sc = math.cos(c), math.sin(c)
    dx, dy, dz = x - px, y - py, z - pz
    rx = cc * dx - sc * dy
    ry = sc * dx + cc * dy
    return (rx + px, ca * ry - sa * dz + py, sa * ry + ca * dz + pz)


# =====================================================================
# FRAMEWORK DE TESTS
# =====================================================================

class TestRunner:
    def __init__(self):
        self.results = []
        self.current_group = ""

    def group(self, name):
        self.current_group = name
        print("\n=== %s ===" % name)

    def test(self, name, passed, detail=""):
        status = "[PASS]" if passed else "[FAIL]"
        msg = "  %s %s" % (status, name)
        if detail:
            msg += "  -- %s" % detail
        print(msg)
        self.results.append((self.current_group, name, passed))
        return passed

    def summary(self):
        total = len(self.results)
        passed = sum(1 for _, _, p in self.results if p)
        failed = total - passed
        print("\n" + "=" * 55)
        if failed == 0:
            print("RESULTADO: %d/%d PASADOS OK" % (passed, total))
        else:
            print("RESULTADO: %d/%d pasados, %d fallidos" % (passed, total, failed))
            print("\nFallos:")
            for grp, name, p in self.results:
                if not p:
                    print("  [FAIL] [%s] %s" % (grp, name))
        print("=" * 55)
        return failed == 0


# =====================================================================
# SETUP
# =====================================================================

def setup(sim):
    print("\n=== SETUP ===")
    print("  Config: %s" % CONFIG_FILE)

    ok_count = 0
    err_count = 0
    for cmd in _CONFIG_CMDS:
        resp = sim.cmd(cmd, wait=0.1)
        if has_text(resp, "error"):
            log.warning("Error aplicando '%s': %s", cmd, resp)
            err_count += 1
        else:
            ok_count += 1
            log.debug("OK: %s", cmd)

    # Mover a origen
    sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.1)

    if err_count:
        print("  [WARN] %d settings con error, %d aplicados OK" % (err_count, ok_count))
    else:
        print("  %d settings aplicados OK" % ok_count)
    print("  Pivot: X=%.1f Y=%.1f Z=%.1f" % PIVOT)
    print("  Soft limits: %s | Hard limits: %s" % (
        "ON" if _CONFIG_SETTINGS.get("$20", "0") != "0" else "OFF",
        "ON" if _CONFIG_SETTINGS.get("$21", "0") != "0" else "OFF"))


def reset_position(sim, rtcp_on=True):
    if rtcp_on:
        sim.cmd("M451", wait=0.2)
    resp = sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
    if has_text(resp, "ALARM"):
        recover_alarm(sim)
        sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)


def recover_alarm(sim):
    sim.unlock()
    time.sleep(0.5)
    sim.cmd("$20=0", wait=0.1)
    sim.cmd("M451", wait=0.2)


# =====================================================================
# GRUPO: MATEMATICAS (cinematica pura)
# =====================================================================

def test_matematicas(sim, t):
    t.group("MATEMATICAS: Cinematica inversa")

    sim.cmd("M451", wait=0.2)

    # Tests de cinematica que ejecutan en el simulador (angulos sin singularidad)
    tests = [
        ("Identidad A0 C0",   "G0 X100 Y0 Z0 A0 C0",     100, 0, 0, 0, 0),
        ("C=90",              "G0 X100 Y0 Z0 A0 C90",     100, 0, 0, 0, 90),
        ("A=45",              "G0 X0 Y0 Z0 A45 C0",       0, 0, 0, 45, 0),
        ("A45 C30 XYZ",       "G0 X50 Y25 Z-10 A45 C30",  50, 25, -10, 45, 30),
        ("C=180 inv X",       "G0 X100 Y0 Z0 A0 C180",    100, 0, 0, 0, 180),
        ("A-30 C-45",         "G0 X30 Y20 Z-5 A-30 C-45", 30, 20, -5, -30, -45),
        ("C=360 ident",       "G0 X75 Y0 Z0 A0 C360",     75, 0, 0, 0, 360),
        ("A=80 extremo",      "G0 X10 Y10 Z0 A80 C60",    10, 10, 0, 80, 60),
    ]

    for i, (name, gcode, x, y, z, a, c) in enumerate(tests, 1):
        exp = rtcp_inverse(x, y, z, a, c, PIVOT[0], PIVOT[1], PIVOT[2])

        resp = sim.cmd(gcode, timeout=10, wait=0.3)
        if has_text(resp, "ALARM"):
            recover_alarm(sim)
            t.test("[%2d] %s" % (i, name), False, "ALARM recibido")
            reset_position(sim)
            continue

        sim.wait_stable(max_wait=15, interval=0.5)

        rtcp_resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
        motor = get_motor(rtcp_resp)

        if motor is None:
            t.test("[%2d] %s" % (i, name), False, "sin respuesta Motor")
            recover_alarm(sim)
            reset_position(sim)
            continue

        diffs = [abs(exp[j] - motor[j]) for j in range(3)]
        md = max(diffs)
        ax = ["X", "Y", "Z"][diffs.index(md)]

        if md <= TOL_MATH:
            t.test("[%2d] %s" % (i, name), True,
                   "Motor: X=%.3f Y=%.3f Z=%.3f" % motor)
        else:
            t.test("[%2d] %s" % (i, name), False,
                   "diff %.3fmm en %s | Esp: X=%.3f Y=%.3f Z=%.3f | Act: X=%.3f Y=%.3f Z=%.3f"
                   % (md, ax, exp[0], exp[1], exp[2], motor[0], motor[1], motor[2]))

        reset_position(sim)

    # Verificacion de formulas Python para angulos en singularidad
    t.group("MATEMATICAS: Formulas Python A=90 (Gimbal Lock)")

    # A=90, pivot Z=150: motor_Y=150 (dz pivotea a Y), motor_Z=150 (queda a altura pivot)
    m90 = rtcp_inverse(0, 0, 0, 90, 0, PIVOT[0], PIVOT[1], PIVOT[2])
    t.test("Formula Python A=90: motor_Y=150",
           abs(m90[1] - 150.0) < TOL_MATH,
           "calc: X=%.3f Y=%.3f Z=%.3f" % m90)
    t.test("Formula Python A=90: motor_Z=pz (150)",
           abs(m90[2] - PIVOT[2]) < TOL_MATH,
           "calc: Z=%.3f, pivot_z=%.3f" % (m90[2], PIVOT[2]))

    m90c90 = rtcp_inverse(50, 0, 0, 90, 90, PIVOT[0], PIVOT[1], PIVOT[2])
    t.test("Formula Python A=90 C=90: calculo coherente",
           abs(m90c90[1] - 150.0) < TOL_MATH,
           "calc: X=%.3f Y=%.3f Z=%.3f" % m90c90)


# =====================================================================
# GRUPO: FUNCIONES (diagnostico, cache, settings)
# =====================================================================

def test_diagnostico(sim, t):
    t.group("FUNCIONES: $RTCP diagnostico")

    sim.cmd("M451", wait=0.2)
    resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
    data = get_rtcp_data(resp)

    t.test("$RTCP reporta version", has_text(resp, "v17.1"))
    t.test("$RTCP reporta modo ON", data["mode"] == "ON")
    t.test("$RTCP reporta pivot Z=150", data["pivot"].get("Z") == 150.0,
           "pivot_z=%s" % data["pivot"].get("Z"))
    t.test("$RTCP reporta offsets en 0",
           data["offsets"].get("Y") == 0.0 and data["offsets"].get("Z") == 0.0)
    t.test("$RTCP reporta TCP Position", data["tcp"] is not None)
    t.test("$RTCP reporta Motor Position", data["motor"] is not None)
    t.test("$RTCP reporta Rotary Axes", data["a_deg"] is not None)
    t.test("$RTCP reporta Trig Cache", data["cache"] is not None,
           "cache=%s" % data["cache"])


def test_cache(sim, t):
    t.group("FUNCIONES: Cache trigonometrico")

    sim.cmd("M450", wait=0.2)
    sim.cmd("M451", wait=0.2)
    resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
    data = get_rtcp_data(resp)
    t.test("Cache Invalid tras M451 sin movimiento",
           data["cache"] == "Invalid", "cache=%s" % data["cache"])

    sim.cmd("G0 X5 A10", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)
    resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
    data = get_rtcp_data(resp)
    t.test("Cache Valid tras movimiento con angulo",
           data["cache"] == "Valid", "cache=%s" % data["cache"])

    sim.cmd("$642=151", wait=0.2)
    resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
    data = get_rtcp_data(resp)
    t.test("Cache Invalid tras cambio de setting",
           data["cache"] == "Invalid", "cache=%s" % data["cache"])
    sim.cmd("$642=150", wait=0.2)

    reset_position(sim)


def test_settings(sim, t):
    t.group("FUNCIONES: Settings $640-$644")

    sim.cmd("$640=25", wait=0.2)
    data = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.3))
    t.test("$640 cambia Pivot X", data["pivot"].get("X") == 25.0,
           "pivot_x=%s" % data["pivot"].get("X"))

    sim.cmd("$641=10", wait=0.2)
    data = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.3))
    t.test("$641 cambia Pivot Y", data["pivot"].get("Y") == 10.0,
           "pivot_y=%s" % data["pivot"].get("Y"))

    sim.cmd("$643=5", wait=0.2)
    data = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.3))
    t.test("$643 cambia Offset Y", data["offsets"].get("Y") == 5.0,
           "offset_y=%s" % data["offsets"].get("Y"))

    sim.cmd("$644=3", wait=0.2)
    data = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.3))
    t.test("$644 cambia Offset Z", data["offsets"].get("Z") == 3.0,
           "offset_z=%s" % data["offsets"].get("Z"))

    # Restaurar
    for s in ["$640=0", "$641=0", "$642=150", "$643=0", "$644=0"]:
        sim.cmd(s, wait=0.1)


# =====================================================================
# GRUPO: M-CODES (toggle, warning, conmutacion)
# =====================================================================

def test_mcodes(sim, t):
    t.group("M-CODES: M451/M450 toggle")

    sim.cmd("M451", wait=0.2)
    data = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.3))
    t.test("M451 activa RTCP", data["mode"] == "ON")

    sim.cmd("M450", wait=0.2)
    data = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.3))
    t.test("M450 desactiva RTCP", data["mode"] == "OFF")

    sim.cmd("M451", wait=0.2)
    data = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.3))
    t.test("M451 reactiva RTCP", data["mode"] == "ON")

    resp = sim.cmd("M451", wait=0.2)
    t.test("M451 doble no da error", any("ok" in l.lower() for l in resp))

    sim.cmd("M450", wait=0.2)
    resp = sim.cmd("M450", wait=0.2)
    t.test("M450 doble no da error", any("ok" in l.lower() for l in resp))

    # Warning con ejes rotados
    t.group("M-CODES: Warning rotary != 0")

    sim.cmd("M451", wait=0.2)
    sim.cmd("G0 X0 Y0 Z0 A45 C0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)

    resp = sim.cmd("M450", wait=0.3)
    t.test("M450 con A=45 genera warning",
           has_text(resp, "Warning") or has_text(resp, "rotary"),
           "resp=%s" % "|".join(resp[:3]))

    sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)

    sim.cmd("M451", wait=0.2)
    sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
    sim.wait_stable(max_wait=15, interval=0.5)
    resp = sim.cmd("M450", wait=0.3)
    t.test("M450 con A=0 NO genera warning",
           not has_text(resp, "Warning"), "resp=%s" % "|".join(resp[:3]))

    # Conmutacion mid-program
    t.group("M-CODES: Conmutacion mid-program")

    sim.cmd("M451", wait=0.2)
    reset_position(sim)

    sim.cmd("G0 X5 A0 C0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)

    sim.cmd("M450", wait=0.2)
    data = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.3))
    t.test("Conmutacion ON->OFF exitosa", data["mode"] == "OFF")

    sim.cmd("G0 X10 Y0 Z0 A0 C0", wait=0.2)
    time.sleep(2.0)
    sim.wait_stable(max_wait=15, interval=0.5)
    data = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.3))
    motor = data["motor"]
    t.test("Movimiento sin RTCP reporta modo OFF",
           data["mode"] == "OFF",
           "mode=%s, motor_x=%s" % (data["mode"], motor["X"] if motor else "N/A"))

    sim.cmd("M451", wait=0.2)
    data = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.3))
    t.test("Reactivacion OFF->ON exitosa", data["mode"] == "ON")

    reset_position(sim)


# =====================================================================
# GRUPO: BYPASS (identidad con RTCP OFF)
# =====================================================================

def test_bypass(sim, t):
    t.group("BYPASS: RTCP OFF = identidad pura")

    sim.cmd("M450", wait=0.2)
    sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)

    sim.cmd("G0 X5 Y0 Z0 A10 C0", wait=0.2)
    time.sleep(3.0)
    sim.wait_stable(max_wait=15, interval=0.5)
    data = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    motor = data["motor"]

    t.test("Bypass: Motor X reportado",
           motor is not None,
           "motor_x=%s" % (motor["X"] if motor else "N/A"))
    t.test("Bypass: Motor Y = 0 (sin transformacion)",
           motor and abs(motor["Y"]) < TOL,
           "motor_y=%s" % (motor["Y"] if motor else "N/A"))
    t.test("Bypass: Motor Z = 0 (sin transformacion)",
           motor and abs(motor["Z"]) < TOL,
           "motor_z=%s" % (motor["Z"] if motor else "N/A"))

    # Con RTCP ON la misma posicion da transformacion
    sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)
    sim.cmd("M451", wait=0.2)
    sim.cmd("G0 X5 A10", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)
    data = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    motor_on = data["motor"]

    t.test("RTCP ON: Motor Y != 0 (transformacion activa)",
           motor_on and abs(motor_on["Y"]) > 1.0,
           "motor_y=%s" % (motor_on["Y"] if motor_on else "N/A"))
    t.test("RTCP ON: Motor Z != 0 (pivot compensa)",
           motor_on and abs(motor_on["Z"]) > 1.0,
           "motor_z=%s" % (motor_on["Z"] if motor_on else "N/A"))

    reset_position(sim)


# =====================================================================
# GRUPO: COHERENCIA (inv/directa + pivot Z)
# =====================================================================

def test_coherencia(sim, t):
    t.group("COHERENCIA: Cinematica inversa/directa")

    sim.cmd("M451", wait=0.2)
    sim.cmd("G0 X5 Y3 Z-2 A20 C30", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)
    data = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))

    tcp = data["tcp"]
    t.test("TCP X reportado ~= 5", tcp and abs(tcp["X"] - 5.0) < TOL,
           "tcp_x=%s" % (tcp["X"] if tcp else "N/A"))
    t.test("TCP Y reportado ~= 3", tcp and abs(tcp["Y"] - 3.0) < TOL,
           "tcp_y=%s" % (tcp["Y"] if tcp else "N/A"))
    t.test("TCP Z reportado ~= -2", tcp and abs(tcp["Z"] - (-2.0)) < TOL,
           "tcp_z=%s" % (tcp["Z"] if tcp else "N/A"))
    t.test("A reportado ~= 20",
           data["a_deg"] is not None and abs(data["a_deg"] - 20.0) < 0.1,
           "a=%s" % data["a_deg"])
    t.test("C reportado ~= 30",
           data["c_deg"] is not None and abs(data["c_deg"] - 30.0) < 0.1,
           "c=%s" % data["c_deg"])

    motor = data["motor"]
    t.test("Motor != TCP (transformacion activa)",
           motor and tcp and (
               abs(motor["X"] - tcp["X"]) > 0.5 or
               abs(motor["Y"] - tcp["Y"]) > 0.5 or
               abs(motor["Z"] - tcp["Z"]) > 0.5),
           "motor=%s, tcp=%s" % (motor, tcp))

    reset_position(sim)

    # Pivot Z efecto con angulo valido (A=80, no singularidad)
    t.group("COHERENCIA: Pivot Z")

    # Verificar con A=80 (cerca del limite pero sin singularidad)
    sim.cmd("M451", wait=0.2)
    sim.cmd("G0 X0 Y0 Z0 A80 C0", wait=0.2)
    sim.wait_stable(max_wait=15, interval=0.5)
    data80 = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    motor80 = data80["motor"]
    exp80 = rtcp_inverse(0, 0, 0, 80, 0, PIVOT[0], PIVOT[1], PIVOT[2])

    t.test("A=80 pivot Z=150: motor_Y correcto",
           motor80 and abs(motor80["Y"] - exp80[1]) < TOL,
           "act=%.3f esp=%.3f" % (motor80["Y"] if motor80 else 0, exp80[1]))
    t.test("A=80 pivot Z=150: motor_Z correcto",
           motor80 and abs(motor80["Z"] - exp80[2]) < TOL,
           "act=%.3f esp=%.3f" % (motor80["Z"] if motor80 else 0, exp80[2]))

    reset_position(sim)

    # Tambien verificar formula Python para pivot Z en angulos de singularidad
    m150 = rtcp_inverse(0, 0, 0, 90, 0, 0, 0, 150)
    m100 = rtcp_inverse(0, 0, 0, 90, 0, 0, 0, 100)
    t.test("Formula: Pivot Z=150 motor_Y=150 con A=90",
           abs(m150[1] - 150.0) < TOL, "calc=%.3f" % m150[1])
    t.test("Formula: Cambio pivot proporcional",
           abs(m150[1] - m100[1] - 50.0) < TOL,
           "diff=%.3f" % (m150[1] - m100[1]))

    # Identidad con RTCP ON
    t.group("COHERENCIA: Identidad A=0 C=0")

    sim.cmd("M451", wait=0.2)
    sim.cmd("G0 X7 Y3 Z-1 A0 C0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)
    data = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    motor = data["motor"]

    t.test("A=0 C=0 -> Motor X = TCP X",
           motor and abs(motor["X"] - 7.0) < TOL,
           "motor_x=%s" % (motor["X"] if motor else "N/A"))
    t.test("A=0 C=0 -> Motor Y = TCP Y",
           motor and abs(motor["Y"] - 3.0) < TOL,
           "motor_y=%s" % (motor["Y"] if motor else "N/A"))
    t.test("A=0 C=0 -> Motor Z = TCP Z",
           motor and abs(motor["Z"] - (-1.0)) < TOL,
           "motor_z=%s" % (motor["Z"] if motor else "N/A"))

    reset_position(sim)


# =====================================================================
# GRUPO: FEEDRATE (compensacion, segmentacion)
# =====================================================================

def test_feedrate(sim, t):
    t.group("FEEDRATE: Compensacion y segmentacion")

    sim.cmd("M451", wait=0.2)
    reset_position(sim)

    # G0 a posicion con angulos
    sim.cmd("G0 X5 Y3 Z0 A30 C45", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)
    d_g0 = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    motor_g0 = d_g0["motor"]

    reset_position(sim)

    # G1 a la MISMA posicion
    sim.cmd("G1 X5 Y3 Z0 A30 C45 F5000", wait=0.2)
    sim.wait_stable(max_wait=15, interval=0.5)
    d_g1 = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    motor_g1 = d_g1["motor"]

    if motor_g0 and motor_g1:
        dx = abs(motor_g0["X"] - motor_g1["X"])
        dy = abs(motor_g0["Y"] - motor_g1["Y"])
        dz = abs(motor_g0["Z"] - motor_g1["Z"])
        max_diff = max(dx, dy, dz)
        t.test("G1 llega al mismo destino que G0",
               max_diff < TOL,
               "diff=%.4fmm" % max_diff)
    else:
        t.test("G1 llega al mismo destino que G0",
               False, "No se obtuvieron posiciones")

    # Motor dist > TCP dist
    tcp_g1 = d_g1["tcp"]
    if motor_g1 and tcp_g1:
        motor_dist = math.sqrt(sum(v ** 2 for v in motor_g1.values()))
        tcp_dist = math.sqrt(sum(v ** 2 for v in tcp_g1.values()))
        t.test("Motor dist > TCP dist (compensacion activa)",
               motor_dist > tcp_dist * 1.1,
               "motor=%.2f tcp=%.2f ratio=%.2f" % (motor_dist, tcp_dist, motor_dist / tcp_dist))
    else:
        t.test("Motor dist > TCP dist", False, "No se obtuvieron posiciones")

    # Segmentacion G1 largo
    reset_position(sim)
    sim.cmd("G1 X20 Y0 Z0 A45 C0 F5000", wait=0.2)
    sim.wait_stable(max_wait=15, interval=0.5)
    d_seg = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    tcp_seg = d_seg["tcp"]

    t.test("Segmentacion G1: TCP X llega a 20mm",
           tcp_seg and abs(tcp_seg["X"] - 20.0) < TOL,
           "tcp_x=%s" % (tcp_seg["X"] if tcp_seg else "N/A"))
    t.test("Segmentacion G1: A llega a 45 deg",
           d_seg["a_deg"] is not None and abs(d_seg["a_deg"] - 45.0) < 0.1,
           "a=%s" % d_seg["a_deg"])

    reset_position(sim)


# =====================================================================
# GRUPO: REALTIME REPORT
# =====================================================================

def test_realtime(sim, t):
    t.group("REALTIME: Report |RTCP:ON/OFF|")

    sim.cmd("M451", wait=0.2)
    time.sleep(0.3)

    # Enviar ? y leer respuesta
    sim._drain()
    sim.send("?")
    time.sleep(1.0)
    sim.sock.settimeout(3.0)
    raw = b""
    try:
        while True:
            d = sim.sock.recv(4096)
            if not d:
                break
            raw += d
            if b">" in raw:
                break
    except socket.timeout:
        pass
    rt_text = raw.decode(errors="replace")
    log.debug("Realtime ON: %s", rt_text)
    has_on = "RTCP:ON" in rt_text
    t.test("Status report contiene RTCP:ON",
           has_on,
           "resp=%s" % rt_text.strip()[:120])

    # Drenar y ahora M450
    sim._drain()
    sim.cmd("M450", wait=0.3)
    time.sleep(0.3)

    sim._drain()
    sim.send("?")
    time.sleep(1.0)
    sim.sock.settimeout(3.0)
    raw = b""
    try:
        while True:
            d = sim.sock.recv(4096)
            if not d:
                break
            raw += d
            if b">" in raw:
                break
    except socket.timeout:
        pass
    rt_text = raw.decode(errors="replace")
    log.debug("Realtime OFF: %s", rt_text)
    has_off = "RTCP:OFF" in rt_text
    t.test("Status report contiene RTCP:OFF",
           has_off,
           "resp=%s" % rt_text.strip()[:120])

    sim.cmd("M451", wait=0.2)
    reset_position(sim)


# =====================================================================
# GRUPO: SINGULARIDAD (Gimbal Lock A=90)
# =====================================================================

def test_singularidad(sim, t):
    """Verifica que la proteccion de Gimbal Lock funciona correctamente.

    Cuando A=90, cos(A)=0 y el eje C pierde un grado de libertad.
    rtcp.c:transform_from_cartesian() detecta esto y genera ALARM:12
    (Alarm_LimitsEngaged) como proteccion. Este es comportamiento CORRECTO.
    """
    t.group("SINGULARIDAD: Gimbal Lock (A=90)")

    sim.cmd("M451", wait=0.2)
    reset_position(sim)

    # Test 1: A=90 debe generar ALARM:12
    resp = sim.cmd("G0 X0 Y0 Z0 A90 C0", timeout=10, wait=0.3)
    alarm_90 = has_text(resp, "ALARM")
    t.test("A=90 genera ALARM (Gimbal Lock detectado)",
           alarm_90,
           "resp=%s" % "|".join(resp[:3]))
    if alarm_90:
        recover_alarm(sim)
        reset_position(sim)

    # Test 2: A=-90 tambien debe generar ALARM:12
    sim.cmd("M451", wait=0.2)
    resp = sim.cmd("G0 X0 Y0 Z0 A-90 C0", timeout=10, wait=0.3)
    alarm_n90 = has_text(resp, "ALARM")
    t.test("A=-90 genera ALARM (singularidad simetrica)",
           alarm_n90,
           "resp=%s" % "|".join(resp[:3]))
    if alarm_n90:
        recover_alarm(sim)
        reset_position(sim)

    # Test 3: A=89.6 (dentro del umbral SINGULARITY_COS_THRESHOLD=0.0087
    #         cos(89.6)=0.00698 < 0.0087) debe dar ALARM
    sim.cmd("M451", wait=0.2)
    resp = sim.cmd("G0 X0 Y0 Z0 A89.6 C0", timeout=10, wait=0.3)
    alarm_896 = has_text(resp, "ALARM")
    t.test("A=89.6 genera ALARM (cos=0.007 < umbral 0.0087)",
           alarm_896,
           "cos(89.6)=%.5f, umbral=0.0087" % math.cos(math.radians(89.6)))
    if alarm_896:
        recover_alarm(sim)
        reset_position(sim)

    # Test 4: A=89 (fuera del umbral: cos(89)=0.01745 > 0.0087)
    # debe funcionar sin ALARM
    sim.cmd("M451", wait=0.2)
    resp = sim.cmd("G0 X0 Y0 Z0 A89 C0", timeout=10, wait=0.3)
    alarm_89 = has_text(resp, "ALARM")
    if alarm_89:
        recover_alarm(sim)
        reset_position(sim)
    t.test("A=89 NO genera ALARM (fuera del umbral)",
           not alarm_89,
           "cos(89)=%.5f, umbral=0.0087" % math.cos(math.radians(89)))

    if not alarm_89:
        sim.wait_stable(max_wait=15, interval=0.5)
        reset_position(sim)

    # Test 5: A=80 debe funcionar sin problemas
    sim.cmd("M451", wait=0.2)
    resp = sim.cmd("G0 X0 Y0 Z0 A80 C0", timeout=10, wait=0.3)
    alarm_80 = has_text(resp, "ALARM")
    if alarm_80:
        recover_alarm(sim)
        reset_position(sim)
    t.test("A=80 NO genera ALARM (angulo seguro)",
           not alarm_80,
           "cos(80)=%.5f" % math.cos(math.radians(80)))

    if not alarm_80:
        sim.wait_stable(max_wait=15, interval=0.5)
        reset_position(sim)

    # Test 6: A=90 con C=90 tambien singularidad
    sim.cmd("M451", wait=0.2)
    resp = sim.cmd("G0 X50 Y0 Z0 A90 C90", timeout=10, wait=0.3)
    alarm_90c90 = has_text(resp, "ALARM")
    t.test("A=90 C=90 genera ALARM (Gimbal Lock)",
           alarm_90c90,
           "resp=%s" % "|".join(resp[:3]))
    if alarm_90c90:
        recover_alarm(sim)
        reset_position(sim)


# =====================================================================
# REGISTRO DE GRUPOS
# =====================================================================

GROUPS = {
    "matematicas":   ("Cinematica inversa + formulas Python", test_matematicas),
    "singularidad": ("Gimbal Lock: proteccion A=90 (ALARM:12)", test_singularidad),
    "funciones":     ("Diagnostico + Cache + Settings",
                      lambda s, t: (test_diagnostico(s, t), test_cache(s, t), test_settings(s, t))),
    "mcodes":        ("M451/M450 toggle + warning + conmutacion", test_mcodes),
    "bypass":        ("Bypass RTCP OFF = identidad", test_bypass),
    "coherencia":    ("Coherencia inv/directa + Pivot Z + Identidad", test_coherencia),
    "feedrate":      ("Feedrate compensacion + segmentacion", test_feedrate),
    "realtime":      ("Realtime report |RTCP:ON/OFF|", test_realtime),
}


# =====================================================================
# MAIN
# =====================================================================

def main():
    parser = argparse.ArgumentParser(
        description="grblHAL RTCP Testing Suite",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos:
    python testing.py                  Ejecuta todos los tests
    python testing.py -matematicas     Solo cinematica matematica
    python testing.py -singularidad    Solo tests de Gimbal Lock
    python testing.py -funciones       Solo diagnostico/cache/settings
    python testing.py -mcodes         Solo M-codes
    python testing.py -bypass         Solo bypass
    python testing.py -coherencia     Solo coherencia
    python testing.py -feedrate       Solo feedrate
    python testing.py -realtime       Solo realtime report
    python testing.py -l              Lista grupos disponibles
    python testing.py -v              Modo verbose
        """
    )

    for name, (desc, _) in GROUPS.items():
        parser.add_argument("-%s" % name, action="store_true", help=desc)

    parser.add_argument("-l", "--list", action="store_true",
                        help="Lista grupos disponibles")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Logging verbose (DEBUG)")

    args = parser.parse_args()

    if args.list:
        print("\nGrupos de tests disponibles:\n")
        for name, (desc, _) in GROUPS.items():
            print("  -%-15s %s" % (name, desc))
        print("\n  Sin argumentos: ejecuta TODOS los grupos")
        return

    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )

    # Determinar grupos a ejecutar
    selected = [name for name in GROUPS if getattr(args, name, False)]
    if not selected:
        selected = list(GROUPS.keys())

    sim = Sim()
    t = TestRunner()

    try:
        sim.start()
        setup(sim)

        for name in selected:
            desc, func = GROUPS[name]
            func(sim, t)

        all_passed = t.summary()
        sys.exit(0 if all_passed else 1)

    except Exception as e:
        print("\n[ERROR] %s" % e)
        import traceback
        traceback.print_exc()
        sys.exit(2)
    finally:
        sim.close()


if __name__ == "__main__":
    main()
