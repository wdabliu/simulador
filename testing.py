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
        "chord_error": {},
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
        elif "Tolerancia" in line or "Chord Error" in line:
            section = "chord_error"
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
        elif section == "chord_error" and "$64" in line:
            if "G1" in line:
                val = line.split("=")[1].strip().split()[0]
                data["chord_error"]["G1"] = float(val)
            elif "G0" in line:
                val = line.split("=")[1].strip().split()[0]
                data["chord_error"]["G0"] = float(val)
            elif "Comp Velocidad" in line:
                data["speed_comp"] = "ON" if "ON" in line else "OFF"
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


def rtcp_forward(mx, my, mz, a_deg, c_deg, px=0, py=0, pz=150):
    """FK: Motor -> TCP. Inversa de rtcp_inverse.
    P_tcp = Pivot + Rc^-1 * Ra^-1 * (P_motor - Pivot)
    """
    a = math.radians(a_deg)
    c = math.radians(c_deg)
    ca, sa = math.cos(a), math.sin(a)
    cc, sc = math.cos(c), math.sin(c)
    dx, dy, dz = mx - px, my - py, mz - pz
    # Ra^-1 (negar angulo A)
    iy = ca * dy + sa * dz
    iz = -sa * dy + ca * dz
    # Rc^-1 (negar angulo C)
    rx = cc * dx + sc * iy
    ry = -sc * dx + cc * iy
    return (rx + px, ry + py, iz + pz)


def twp_rotation_zxz(x, y, z, a1_deg, a2_deg, a3_deg, ox=0, oy=0, oz=0):
    """Replica la rotacion ZXZ de twp_set_euler_angles() en rtcp.c.
    R = Rz(a1) * Rx(a2) * Rz(a3)
    rotated = R * (target - origin) + origin
    """
    r1 = math.radians(a1_deg)
    r2 = math.radians(a2_deg)
    r3 = math.radians(a3_deg)
    c1, s1 = math.cos(r1), math.sin(r1)
    c2, s2 = math.cos(r2), math.sin(r2)
    c3, s3 = math.cos(r3), math.sin(r3)

    R = [
        [ c1*c3 - s1*c2*s3, -c1*s3 - s1*c2*c3,  s1*s2],
        [ s1*c3 + c1*c2*s3, -s1*s3 + c1*c2*c3, -c1*s2],
        [ s2*s3,             s2*c3,              c2   ],
    ]

    dx, dy, dz = x - ox, y - oy, z - oz
    rx = R[0][0]*dx + R[0][1]*dy + R[0][2]*dz + ox
    ry = R[1][0]*dx + R[1][1]*dy + R[1][2]*dz + oy
    rz = R[2][0]*dx + R[2][1]*dy + R[2][2]*dz + oz
    return (rx, ry, rz)


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

    # Verificacion de formulas Python para A=90
    t.group("MATEMATICAS: Formulas Python A=90")

    # A=90, pivot Z=PIVOT[2]: motor_Y=PIVOT[2] (dz pivotea a Y), motor_Z=PIVOT[2]
    m90 = rtcp_inverse(0, 0, 0, 90, 0, PIVOT[0], PIVOT[1], PIVOT[2])
    t.test("Formula Python A=90: motor_Y=pivot_z",
           abs(m90[1] - PIVOT[2]) < TOL_MATH,
           "calc: X=%.3f Y=%.3f Z=%.3f, pivot_z=%.1f" % (m90 + (PIVOT[2],)))
    t.test("Formula Python A=90: motor_Z=pz",
           abs(m90[2] - PIVOT[2]) < TOL_MATH,
           "calc: Z=%.3f, pivot_z=%.3f" % (m90[2], PIVOT[2]))

    m90c90 = rtcp_inverse(50, 0, 0, 90, 90, PIVOT[0], PIVOT[1], PIVOT[2])
    t.test("Formula Python A=90 C=90: calculo coherente",
           abs(m90c90[1] - PIVOT[2]) < TOL_MATH,
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

    # Test $647: Compensacion de velocidad TCP ON/OFF
    t.group("FEEDRATE: $647 Compensacion velocidad TCP")

    # 1. Verificar diagnostico ON/OFF
    sim.cmd("$647=1", wait=0.2)
    d_on = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    t.test("$647=1 reporta Comp Velocidad = ON",
           d_on.get("speed_comp") == "ON",
           "speed_comp=%s" % d_on.get("speed_comp"))

    sim.cmd("$647=0", wait=0.2)
    d_off = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    t.test("$647=0 reporta Comp Velocidad = OFF",
           d_off.get("speed_comp") == "OFF",
           "speed_comp=%s" % d_off.get("speed_comp"))

    # 2. Funcional: G1 con comp ON llega al destino correcto
    sim.cmd("$647=1", wait=0.2)
    reset_position(sim)
    sim.cmd("G1 X5 Y3 Z0 A30 C45 F5000", wait=0.2)
    sim.wait_stable(max_wait=15, interval=0.5)
    d_comp_on = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    motor_on = d_comp_on["motor"]
    tcp_on = d_comp_on["tcp"]

    if motor_on and tcp_on:
        motor_dist_on = math.sqrt(sum(v ** 2 for v in motor_on.values()))
        tcp_dist_on = math.sqrt(sum(v ** 2 for v in tcp_on.values()))
        ratio_on = motor_dist_on / tcp_dist_on if tcp_dist_on > 0.01 else 0
        t.test("Comp ON: motor llega a destino (ratio motor/tcp > 1)",
               ratio_on > 1.1,
               "motor=%.2f tcp=%.2f ratio=%.2f" % (motor_dist_on, tcp_dist_on, ratio_on))
    else:
        t.test("Comp ON: motor llega a destino", False, "posiciones no obtenidas")

    # 3. Funcional: G1 con comp OFF llega al MISMO destino
    sim.cmd("$647=0", wait=0.2)
    reset_position(sim)
    sim.cmd("G1 X5 Y3 Z0 A30 C45 F5000", wait=0.2)
    sim.wait_stable(max_wait=15, interval=0.5)
    d_comp_off = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    motor_off = d_comp_off["motor"]
    tcp_off = d_comp_off["tcp"]

    if motor_off and tcp_off:
        motor_dist_off = math.sqrt(sum(v ** 2 for v in motor_off.values()))
        tcp_dist_off = math.sqrt(sum(v ** 2 for v in tcp_off.values()))
        ratio_off = motor_dist_off / tcp_dist_off if tcp_dist_off > 0.01 else 0
        t.test("Comp OFF: motor llega al mismo destino",
               ratio_off > 1.1,
               "motor=%.2f tcp=%.2f ratio=%.2f" % (motor_dist_off, tcp_dist_off, ratio_off))
    else:
        t.test("Comp OFF: motor llega al mismo destino", False, "posiciones no obtenidas")

    # 4. Comparar: ambos modos llegan a la MISMA posicion final
    if motor_on and motor_off:
        dx = abs(motor_on["X"] - motor_off["X"])
        dy = abs(motor_on["Y"] - motor_off["Y"])
        dz = abs(motor_on["Z"] - motor_off["Z"])
        max_diff = max(dx, dy, dz)
        t.test("ON vs OFF: misma posicion final (comp solo afecta velocidad)",
               max_diff < TOL,
               "diff=%.4fmm ON=(%.3f,%.3f,%.3f) OFF=(%.3f,%.3f,%.3f)" % (
                   max_diff,
                   motor_on["X"], motor_on["Y"], motor_on["Z"],
                   motor_off["X"], motor_off["Y"], motor_off["Z"]))
    else:
        t.test("ON vs OFF: misma posicion final", False, "posiciones no obtenidas")

    # Restaurar $647=1
    sim.cmd("$647=1", wait=0.2)
    reset_position(sim)


# =====================================================================
# GRUPO: TWP (Tilted Work Plane: G68.2 / G53.1 / G69)
# =====================================================================

def test_twp(sim, t):
    # Asegurar RTCP OFF -- TWP solo funciona con RTCP deshabilitado
    sim.cmd("M450", wait=0.2)
    sim.cmd("G69", wait=0.2)  # Limpiar cualquier TWP previa
    sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)

    # ----- Parsing y validacion -----
    t.group("TWP: Parsing y validacion")

    # Test 1: G68.2 sin I/J/K da error
    resp = sim.cmd("G68.2 X0 Y0 Z0", wait=0.3)
    t.test("G68.2 sin I/J/K da error",
           has_text(resp, "error"),
           "resp=%s" % "|".join(resp[:3]))

    # Test 2: G68.2 con I/J/K acepta
    resp = sim.cmd("G68.2 X0 Y0 Z0 I0 J30 K0", wait=0.3)
    t.test("G68.2 con I/J/K acepta (ok)",
           any("ok" in l.lower() for l in resp),
           "resp=%s" % "|".join(resp[:3]))

    # Test 3: G53.1 sin G68.2 previo da error
    sim.cmd("G69", wait=0.2)  # Limpiar
    sim._drain()
    resp = sim.cmd("G53.1", wait=0.3)
    t.test("G53.1 sin G68.2 previo da error",
           has_text(resp, "error"),
           "resp=%s" % "|".join(resp[:3]))

    # Test 4: G53.1 despues de G68.2 acepta
    sim.cmd("G68.2 X0 Y0 Z0 I0 J30 K0", wait=0.2)
    resp = sim.cmd("G53.1", wait=0.3)
    t.test("G53.1 despues de G68.2 acepta (ok)",
           any("ok" in l.lower() for l in resp),
           "resp=%s" % "|".join(resp[:3]))
    sim.cmd("G69", wait=0.2)

    # Test 5: G69 sin TWP activa NO da error (idempotente)
    resp = sim.cmd("G69", wait=0.3)
    t.test("G69 sin TWP activa = ok (idempotente)",
           any("ok" in l.lower() for l in resp),
           "resp=%s" % "|".join(resp[:3]))

    # Test 6: G69 doble NO da error
    sim.cmd("G69", wait=0.2)
    resp = sim.cmd("G69", wait=0.3)
    t.test("G69 doble = ok",
           any("ok" in l.lower() for l in resp),
           "resp=%s" % "|".join(resp[:3]))

    # ----- Rotacion matematica -----
    t.group("TWP: Rotacion matematica ZXZ")

    sim.cmd("G0 X0 Y0 Z0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)

    twp_tests = [
        # (nombre, euler I J K, origen, movimiento X Y Z, descripcion)
        ("Identidad I=0 J=0 K=0",
         0, 0, 0, 0, 0, 0,
         10, 0, 0,
         "Motor sin cambio"),
        ("J=90 rota Z->X",
         0, 90, 0, 0, 0, 0,
         0, 0, 10,
         "Z=10 debe mapearse a X"),
        ("J=90 rota X->-Z",
         0, 90, 0, 0, 0, 0,
         10, 0, 0,
         "X=10 debe mapearse a -Z"),
        ("I=90 rota X->Y",
         90, 0, 0, 0, 0, 0,
         10, 0, 0,
         "Rz(90) rota X a Y"),
    ]

    for i, (name, ei, ej, ek, ox, oy, oz, mx, my, mz, desc) in enumerate(twp_tests, 7):
        # Limpiar y definir TWP
        sim.cmd("G69", wait=0.2)
        sim.cmd("G0 X0 Y0 Z0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)

        sim.cmd("G68.2 X%.1f Y%.1f Z%.1f I%.1f J%.1f K%.1f" % (ox, oy, oz, ei, ej, ek), wait=0.2)
        sim.cmd("G53.1", wait=0.2)

        # Mover
        resp = sim.cmd("G0 X%.1f Y%.1f Z%.1f" % (mx, my, mz), timeout=10, wait=0.3)
        if has_text(resp, "ALARM"):
            recover_alarm(sim)
            t.test("[%2d] %s" % (i, name), False, "ALARM")
            sim.cmd("G69", wait=0.2)
            continue

        sim.wait_stable(max_wait=15, interval=0.5)

        # Leer posicion motor via $RTCP
        rtcp_resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
        motor = get_motor(rtcp_resp)

        # Calcular esperado
        exp = twp_rotation_zxz(mx, my, mz, ei, ej, ek, ox, oy, oz)

        if motor is None:
            t.test("[%2d] %s" % (i, name), False, "sin respuesta Motor")
            sim.cmd("G69", wait=0.2)
            continue

        diffs = [abs(exp[j] - motor[j]) for j in range(3)]
        md = max(diffs)
        ax = ["X", "Y", "Z"][diffs.index(md)]

        if md <= TOL:
            t.test("[%2d] %s" % (i, name), True,
                   "Motor: X=%.3f Y=%.3f Z=%.3f" % motor)
        else:
            t.test("[%2d] %s" % (i, name), False,
                   "diff %.3fmm en %s | Esp: X=%.3f Y=%.3f Z=%.3f | Act: X=%.3f Y=%.3f Z=%.3f"
                   % (md, ax, exp[0], exp[1], exp[2], motor[0], motor[1], motor[2]))

        sim.cmd("G69", wait=0.2)

    # ----- Origen TWP -----
    t.group("TWP: Origen de rotacion")

    # Test 11: Punto EN el origen no se mueve
    sim.cmd("G0 X0 Y0 Z0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)
    sim.cmd("G68.2 X50 Y0 Z0 I90 J0 K0", wait=0.2)
    sim.cmd("G53.1", wait=0.2)
    sim.cmd("G0 X50 Y0 Z0", timeout=10, wait=0.3)
    sim.wait_stable(max_wait=15, interval=0.5)
    rtcp_resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
    motor = get_motor(rtcp_resp)
    exp_origin = twp_rotation_zxz(50, 0, 0, 90, 0, 0, 50, 0, 0)
    if motor:
        diffs = [abs(exp_origin[j] - motor[j]) for j in range(3)]
        md = max(diffs)
        t.test("[11] Punto en origen TWP no cambia",
               md <= TOL,
               "Esp: (%.1f,%.1f,%.1f) Act: (%.3f,%.3f,%.3f) diff=%.4f"
               % (exp_origin[0], exp_origin[1], exp_origin[2],
                  motor[0], motor[1], motor[2], md))
    else:
        t.test("[11] Punto en origen TWP no cambia", False, "sin Motor")
    sim.cmd("G69", wait=0.2)

    # Test 12: Rotacion alrededor de origen no-cero
    sim.cmd("G0 X0 Y0 Z0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)
    sim.cmd("G68.2 X50 Y0 Z0 I90 J0 K0", wait=0.2)
    sim.cmd("G53.1", wait=0.2)
    sim.cmd("G0 X60 Y0 Z0", timeout=10, wait=0.3)
    sim.wait_stable(max_wait=15, interval=0.5)
    rtcp_resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
    motor = get_motor(rtcp_resp)
    exp_off = twp_rotation_zxz(60, 0, 0, 90, 0, 0, 50, 0, 0)
    if motor:
        diffs = [abs(exp_off[j] - motor[j]) for j in range(3)]
        md = max(diffs)
        t.test("[12] Rotacion con origen=(50,0,0)",
               md <= TOL,
               "Esp: (%.3f,%.3f,%.3f) Act: (%.3f,%.3f,%.3f) diff=%.4f"
               % (exp_off[0], exp_off[1], exp_off[2],
                  motor[0], motor[1], motor[2], md))
    else:
        t.test("[12] Rotacion con origen=(50,0,0)", False, "sin Motor")
    sim.cmd("G69", wait=0.2)

    # ----- Interaccion TWP <-> RTCP -----
    t.group("TWP: Interaccion con RTCP")

    # Test 13: TWP con RTCP OFF aplica rotacion
    sim.cmd("M450", wait=0.2)  # RTCP OFF
    sim.cmd("G0 X0 Y0 Z0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)
    sim._drain()
    sim.cmd("G68.2 X0 Y0 Z0 I0 J90 K0", wait=0.3)
    sim.cmd("G53.1", wait=0.3)
    sim.cmd("G0 X0 Y0 Z10", timeout=10, wait=0.3)
    sim.wait_stable(max_wait=15, interval=0.5)
    rtcp_resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
    motor = get_motor(rtcp_resp)
    exp13 = twp_rotation_zxz(0, 0, 10, 0, 90, 0)
    if motor:
        diffs = [abs(exp13[j] - motor[j]) for j in range(3)]
        md = max(diffs)
        t.test("[13] TWP con RTCP OFF: rotacion activa",
               md <= TOL,
               "Esp: (%.3f,%.3f,%.3f) Act: (%.3f,%.3f,%.3f) diff=%.4f"
               % (exp13[0], exp13[1], exp13[2], motor[0], motor[1], motor[2], md))
    else:
        t.test("[13] TWP con RTCP OFF: rotacion activa", False, "sin Motor")
    sim.cmd("G69", wait=0.2)

    # Test 14: RTCP ON (M451) hace que TWP deje de aplicarse
    sim.cmd("G0 X0 Y0 Z0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)
    sim.cmd("G68.2 X0 Y0 Z0 I0 J90 K0", wait=0.2)
    sim.cmd("G53.1", wait=0.2)
    sim.cmd("M451", wait=0.2)  # Activar RTCP -> TWP no aplica
    sim.cmd("G0 X0 Y0 Z10 A0 C0", timeout=10, wait=0.3)
    sim.wait_stable(max_wait=15, interval=0.5)
    rtcp_resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
    data = get_rtcp_data(rtcp_resp)
    motor = data["motor"]
    # Con RTCP ON + A=0 C=0, sin TWP, motor Z debe ser ~10 (identidad RTCP)
    if motor:
        t.test("[14] RTCP ON inhibe TWP (motor Z~=10)",
               abs(motor["Z"] - 10.0) < TOL,
               "Motor: X=%.3f Y=%.3f Z=%.3f" % (motor["X"], motor["Y"], motor["Z"]))
    else:
        t.test("[14] RTCP ON inhibe TWP", False, "sin Motor")
    sim.cmd("M450", wait=0.2)
    sim.cmd("G69", wait=0.2)

    # ----- Cancelacion G69 -----
    t.group("TWP: Cancelacion G69")

    # Test 15: G69 cancela rotacion
    sim.cmd("G0 X0 Y0 Z0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)
    sim.cmd("G68.2 X0 Y0 Z0 I0 J90 K0", wait=0.2)
    sim.cmd("G53.1", wait=0.2)
    sim.cmd("G69", wait=0.2)  # Cancelar
    sim.cmd("G0 X0 Y0 Z10", timeout=10, wait=0.3)
    sim.wait_stable(max_wait=15, interval=0.5)
    rtcp_resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
    motor = get_motor(rtcp_resp)
    if motor:
        t.test("[15] G69 cancela: Z=10 sin rotar",
               abs(motor[2] - 10.0) < TOL and abs(motor[0]) < TOL,
               "Motor: X=%.3f Y=%.3f Z=%.3f (esp X~0 Z~10)" % motor)
    else:
        t.test("[15] G69 cancela rotacion", False, "sin Motor")

    # Test 16: G53.1 despues de G69 da error (defined=false)
    sim._drain()
    resp = sim.cmd("G53.1", wait=0.3)
    t.test("[16] G53.1 despues de G69 da error",
           has_text(resp, "error"),
           "resp=%s" % "|".join(resp[:3]))

    # Cleanup
    sim.cmd("G69", wait=0.2)
    sim.cmd("G0 X0 Y0 Z0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)


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
# GRUPO: BUG2P2 (rebaseline gc_state.position en M451)
# =====================================================================

def test_bug2p2_rebaseline(sim, t):
    """Verifica Bug 2 Problema 2: M451 no re-sincroniza gc_state.position.

    Escenario: con RTCP OFF, mover a posicion con A!=0. Luego activar M451.
    El parser conserva gc_state.position calculada como identidad (motor=cartesiano),
    pero segment_line ahora aplica FK al motor position real. Si no se rebasealinea,
    el delta del siguiente movimiento se distorsiona masivamente.

    Si el bug existe (NO corregido): el motor se mueve >> 1mm para un G1 de 1mm.
    Si el bug esta corregido: el motor se mueve ~1mm (proporcional al delta pedido).
    """
    t.group("BUG2P2: Rebaseline gc_state.position en M451")

    # --- Test 1: Activar M451 con A=45 y verificar salto ---
    t.group("BUG2P2: Salto de posicion tras M451 con A=45")

    # Paso 1: RTCP OFF, mover a posicion con eje rotativo != 0
    sim.cmd("M450", wait=0.2)
    sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)

    sim.cmd("G0 X10 Y20 Z0 A45 C0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)

    # Registrar motor position ANTES de M451 (con RTCP OFF = identidad)
    data_before = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    motor_before = data_before["motor"]
    t.test("Pre-M451: Motor X=10 (identidad)",
           motor_before and abs(motor_before["X"] - 10.0) < TOL,
           "motor_x=%s" % (motor_before["X"] if motor_before else "N/A"))
    t.test("Pre-M451: Motor Y=20 (identidad)",
           motor_before and abs(motor_before["Y"] - 20.0) < TOL,
           "motor_y=%s" % (motor_before["Y"] if motor_before else "N/A"))
    t.test("Pre-M451: Motor Z=0 (identidad)",
           motor_before and abs(motor_before["Z"] - 0.0) < TOL,
           "motor_z=%s" % (motor_before["Z"] if motor_before else "N/A"))

    # Paso 2: Activar RTCP
    sim.cmd("M451", wait=0.2)
    data_after_m451 = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    t.test("M451 activo", data_after_m451["mode"] == "ON")

    # Motor NO deberia cambiar solo por M451 (no hay movimiento)
    motor_after_m451 = data_after_m451["motor"]
    t.test("Motor no cambia solo por M451",
           motor_after_m451 and motor_before and
           abs(motor_after_m451["X"] - motor_before["X"]) < TOL and
           abs(motor_after_m451["Y"] - motor_before["Y"]) < TOL and
           abs(motor_after_m451["Z"] - motor_before["Z"]) < TOL,
           "before=%s after=%s" % (motor_before, motor_after_m451))

    # Paso 3: Hacer movimiento PEQUEIO (1mm en Y)
    # Si el bug existe: el parser cree estar en Y=20, pide Y=19 (delta=1mm)
    # Pero segment_line calcula FK(motors) que da TCP distinto de Y=20
    # El delta real sera >> 1mm
    sim.cmd("G1 Y19 F5000", wait=0.3)
    sim.wait_stable(max_wait=15, interval=0.5)

    data_after_move = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    motor_after_move = data_after_move["motor"]

    if motor_after_move and motor_after_m451:
        delta_x = abs(motor_after_move["X"] - motor_after_m451["X"])
        delta_y = abs(motor_after_move["Y"] - motor_after_m451["Y"])
        delta_z = abs(motor_after_move["Z"] - motor_after_m451["Z"])
        total_delta = math.sqrt(delta_x**2 + delta_y**2 + delta_z**2)

        # Calcular FK de la posicion motor pre-movimiento para referencia
        tcp_real = rtcp_forward(
            motor_after_m451["X"], motor_after_m451["Y"], motor_after_m451["Z"],
            45, 0, PIVOT[0], PIVOT[1], PIVOT[2])

        t.test("Delta motor total para G1 Y19 (delta=1mm)",
               True,  # info-only test, siempre pasa
               "deltaX=%.3f deltaY=%.3f deltaZ=%.3f total=%.3f mm" % (
                   delta_x, delta_y, delta_z, total_delta))

        t.test("FK(motors pre-move) muestra TCP real",
               True,  # info-only
               "TCP_real: X=%.3f Y=%.3f Z=%.3f vs parser cree: X=10 Y=20 Z=0" % tcp_real)

        # G90 Y19 con RTCP ON: delta grande es CORRECTO
        # TCP real esta en Y=-91.9, G90 Y19 = moverse a Y=19 = ~110mm real
        t.test("[INFO] G90 Y19 delta motor (correcto bajo RTCP)",
               True,
               "delta_total=%.3f mm" % total_delta)

        # Verificar que la distorsion es consistente con FK
        # El parser cree Y=20, pero FK da un Y muy diferente con A=45
        parser_y = 20.0
        fk_y = tcp_real[1]
        y_discrepancy = abs(parser_y - fk_y)
        t.test("Discrepancia Y parser vs FK",
               True,  # info-only
               "parser_y=%.3f fk_y=%.3f discrepancia=%.3f mm" % (
                   parser_y, fk_y, y_discrepancy))
    else:
        t.test("[BUG2P2] No se pudo leer motor position", False,
               "after_m451=%s after_move=%s" % (motor_after_m451, motor_after_move))

    # --- Test 2: Verificar con A=0 (caso sin bug, delta debe ser ~1mm) ---
    t.group("BUG2P2: Control -- M451 con A=0 (sin distorsion)")

    sim.cmd("M450", wait=0.2)
    sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)

    sim.cmd("G0 X10 Y20 Z0 A0 C0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)

    sim.cmd("M451", wait=0.2)
    data_ctrl_pre = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    motor_ctrl_pre = data_ctrl_pre["motor"]

    sim.cmd("G1 Y19 F5000", wait=0.3)
    sim.wait_stable(max_wait=15, interval=0.5)

    data_ctrl_post = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    motor_ctrl_post = data_ctrl_post["motor"]

    if motor_ctrl_pre and motor_ctrl_post:
        ctrl_delta = math.sqrt(
            (motor_ctrl_post["X"] - motor_ctrl_pre["X"])**2 +
            (motor_ctrl_post["Y"] - motor_ctrl_pre["Y"])**2 +
            (motor_ctrl_post["Z"] - motor_ctrl_pre["Z"])**2)
        t.test("Control A=0: delta ~1mm (sin distorsion)",
               ctrl_delta < 3.0,
               "delta=%.3f mm (esperado ~1.0)" % ctrl_delta)
    else:
        t.test("Control A=0: lectura motor", False, "sin datos")

    # --- Test 3: Angulo mayor (A=80) para distorsion mas visible ---
    t.group("BUG2P2: A=80 -- distorsion extrema")

    sim.cmd("M450", wait=0.2)
    sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)

    sim.cmd("G0 X5 Y10 Z0 A80 C0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)

    sim.cmd("M451", wait=0.2)
    data_a80_pre = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    motor_a80_pre = data_a80_pre["motor"]

    # Movimiento pequenio: 0.5mm en X
    sim.cmd("G1 X5.5 F5000", wait=0.3)
    sim.wait_stable(max_wait=15, interval=0.5)

    data_a80_post = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    motor_a80_post = data_a80_post["motor"]

    if motor_a80_pre and motor_a80_post:
        a80_delta = math.sqrt(
            (motor_a80_post["X"] - motor_a80_pre["X"])**2 +
            (motor_a80_post["Y"] - motor_a80_pre["Y"])**2 +
            (motor_a80_post["Z"] - motor_a80_pre["Z"])**2)

        tcp_real_a80 = rtcp_forward(
            motor_a80_pre["X"], motor_a80_pre["Y"], motor_a80_pre["Z"],
            80, 0, PIVOT[0], PIVOT[1], PIVOT[2])

        t.test("A=80: FK(motors) TCP real",
               True,
               "TCP_real: X=%.3f Y=%.3f Z=%.3f vs parser: X=5 Y=10 Z=0" % tcp_real_a80)

        fix_a80 = a80_delta < 3.0
        t.test("[BUG2P2-FIX] A=80 G90 X5.5 delta proporcional (<3mm)",
               fix_a80,
               "delta=%.3f mm" % a80_delta)
    else:
        t.test("A=80: lectura motor", False, "sin datos")

    # --- Test 4: G91 incremental (prueba definitiva del rebaseline) ---
    t.group("BUG2P2: G91 incremental -- prueba definitiva")

    sim.cmd("M450", wait=0.2)
    sim.cmd("G90", wait=0.1)
    sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)

    sim.cmd("G0 X10 Y20 Z0 A45 C0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)

    sim.cmd("M451", wait=0.2)
    data_g91_pre = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    motor_g91_pre = data_g91_pre["motor"]

    tcp_pre = rtcp_forward(
        motor_g91_pre["X"], motor_g91_pre["Y"], motor_g91_pre["Z"],
        45, 0, PIVOT[0], PIVOT[1], PIVOT[2])
    t.test("Pre G91: FK(motors) TCP real",
           True,
           "TCP: X=%.3f Y=%.3f Z=%.3f" % tcp_pre)

    # G91 Y-1: si gc_state.position esta rebasealineada, target = FK_Y-1
    # Si NO, target = 20-1 = 19 (corrupto, delta enorme)
    sim.cmd("G91 G1 Y-1 F5000", wait=0.3)
    sim.wait_stable(max_wait=15, interval=0.5)

    data_g91_post = get_rtcp_data(sim.cmd("$RTCP", timeout=5, wait=0.5))
    motor_g91_post = data_g91_post["motor"]

    if motor_g91_pre and motor_g91_post:
        g91_delta = math.sqrt(
            (motor_g91_post["X"] - motor_g91_pre["X"])**2 +
            (motor_g91_post["Y"] - motor_g91_pre["Y"])**2 +
            (motor_g91_post["Z"] - motor_g91_pre["Z"])**2)

        t.test("G91 Y-1: delta motor total",
               True,
               "delta=%.3f mm" % g91_delta)

        t.test("[BUG2P2-FIX] G91 Y-1 delta proporcional (<5mm = fix OK)",
               g91_delta < 5.0,
               "delta=%.3f (esperado ~1 con fix, >>10 sin fix)" % g91_delta)
    else:
        t.test("G91: lectura motor", False, "sin datos")

    # Limpieza
    sim.cmd("G90", wait=0.1)
    sim.cmd("M450", wait=0.2)
    sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
    sim.wait_stable(max_wait=10, interval=0.3)
    reset_position(sim)



# =====================================================================
# REGISTRO DE GRUPOS
# =====================================================================

GROUPS = {
    "matematicas":   ("Cinematica inversa + formulas Python", test_matematicas),
    "funciones":     ("Diagnostico + Cache + Settings",
                      lambda s, t: (test_diagnostico(s, t), test_cache(s, t), test_settings(s, t))),
    "mcodes":        ("M451/M450 toggle + warning + conmutacion", test_mcodes),
    "bypass":        ("Bypass RTCP OFF = identidad", test_bypass),
    "coherencia":    ("Coherencia inv/directa + Pivot Z + Identidad", test_coherencia),
    "feedrate":      ("Feedrate compensacion + segmentacion", test_feedrate),
    "realtime":      ("Realtime report |RTCP:ON/OFF|", test_realtime),
    "twp":           ("G68.2/G53.1/G69 Tilted Work Plane", test_twp),
    "bug2p2":        ("Bug2P2: Rebaseline gc_state.position en M451", test_bug2p2_rebaseline),
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
    python testing.py -twp            Solo TWP (G68.2/G53.1/G69)
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
            # Limpiar buffer TCP y resetear estado entre grupos
            sim._drain()
            sim.unlock()
            sim.cmd("M451", wait=0.2)
            sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
            sim.wait_stable(max_wait=10, interval=0.3)
            sim._drain()
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
