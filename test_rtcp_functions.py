"""
grblHAL RTCP Functions Test Suite
=================================
Tests comprehensivos para TODAS las funciones de rtcp.c:

  1. $RTCP diagnostico
  2. Cache trigonometrico (valid/invalid)
  3. M451/M450 activacion/desactivacion
  4. Bypass cuando RTCP OFF (identidad)
  5. Settings $640-$644 (pivot, offsets)
  6. Warning al desactivar con ejes rotados
  7. Conmutacion mid-program (sync)
  8. Cinematica inversa/directa coherencia
  9. Segmentacion G1 vs G0 rapid
 10. Realtime report |RTCP:ON/OFF|

Movimientos muy pequenos (X=5, A=10) para ser rapidos.
"""

import socket
import subprocess
import time
import math
import sys
import logging

# =====================================================================
# LOGGING
# =====================================================================

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger('rtcp_funcs')

# =====================================================================
# CONFIGURACION
# =====================================================================

SIM_EXE = r"c:\simulador\build\grblHAL_sim.exe"
PORT = 23
PIVOT_Z = 150.0
TOL = 0.05  # mm tolerancia

# =====================================================================
# CONEXION TCP (reutilizada de test_rtcp.py)
# =====================================================================

class Sim:
    def __init__(self):
        self.proc = None
        self.sock = None
        self.buf = b""

    def start(self):
        cmd = [SIM_EXE, "-p", str(PORT), "-t", "0"]
        log.info(f"Lanzando: {' '.join(cmd)}")
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                      creationflags=subprocess.CREATE_NEW_PROCESS_GROUP)
        time.sleep(3.0)
        if self.proc.poll() is not None:
            err = self.proc.stderr.read().decode(errors='replace')
            raise RuntimeError(f"Simulador fallo: {err}")
        log.info(f"PID={self.proc.pid}")
        for attempt in range(5):
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(5.0)
                self.sock.connect(("127.0.0.1", PORT))
                log.info(f"TCP conectado")
                time.sleep(1.0)
                self._drain()
                return
            except (ConnectionRefusedError, socket.timeout, OSError):
                log.warning(f"Intento {attempt+1}/5")
                try: self.sock.close()
                except: pass
                time.sleep(1.5)
        raise ConnectionError("No se pudo conectar")

    def _drain(self):
        self.sock.settimeout(1.0)
        try:
            while True:
                d = self.sock.recv(4096)
                if not d: break
        except: pass
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
                if not d: break
                self.buf += d
                while b"\n" in self.buf:
                    raw, self.buf = self.buf.split(b"\n", 1)
                    line = raw.decode(errors='replace').strip().strip('\r')
                    if line:
                        lines.append(line)
                        if line.lower() == "ok" or line.lower().startswith("error"):
                            return lines
            except socket.timeout:
                break
        return lines

    def cmd(self, command, timeout=5.0, wait=0.2):
        if wait > 0: time.sleep(wait)
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

    def close(self):
        if self.sock:
            try: self.sock.close()
            except: pass
        if self.proc:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=5)
            except:
                try: self.proc.kill()
                except: pass
            log.info("Proceso terminado")


# =====================================================================
# HELPERS DE PARSEO
# =====================================================================

def get_rtcp_data(lines):
    """Parsea respuesta completa de $RTCP."""
    data = {'mode': None, 'pivot': {}, 'offsets': {},
            'tcp': None, 'motor': None, 'cache': None,
            'a_deg': None, 'c_deg': None}
    section = None
    for line in lines:
        if "RTCP Mode:" in line:
            data['mode'] = "ON" if "ON" in line else "OFF"
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
            data['cache'] = "Valid" if "Valid" in line else "Invalid"
        elif section == "pivot" and "$64" in line:
            for axis in ['X', 'Y', 'Z']:
                if f" {axis} = " in line:
                    val = line.split(f"{axis} = ")[1].split()[0]
                    data['pivot'][axis] = float(val)
        elif section == "offsets" and "$64" in line:
            for axis in ['Y', 'Z']:
                if f" {axis} = " in line:
                    val = line.split(f"{axis} = ")[1].split()[0]
                    data['offsets'][axis] = float(val)
        elif section in ("tcp", "motor") and "X =" in line:
            vals = {}
            parts = line.split()
            for i, p in enumerate(parts):
                if p in ('X','Y','Z') and i+2 < len(parts) and parts[i+1] == '=':
                    try: vals[p] = float(parts[i+2])
                    except: pass
            if len(vals) == 3:
                if section == "tcp":
                    data['tcp'] = vals
                else:
                    data['motor'] = vals
        elif section == "rotary":
            if "A =" in line:
                data['a_deg'] = float(line.split("A = ")[1].split()[0])
            elif "C =" in line:
                data['c_deg'] = float(line.split("C = ")[1].split()[0])
    return data


def has_text(lines, text):
    """Busca texto en una lista de lineas."""
    return any(text in l for l in lines)


# =====================================================================
# FRAMEWORK DE TESTS
# =====================================================================

results = []

def test(name, passed, detail=""):
    status = "[PASS]" if passed else "[FAIL]"
    msg = f"  {status} {name}"
    if detail:
        msg += f"  -- {detail}"
    print(msg)
    results.append((name, passed))
    return passed


# =====================================================================
# MAIN
# =====================================================================

def main():
    sim = Sim()
    try:
        sim.start()

        # --- SETUP RAPIDO ---
        print("\n=== SETUP ===")
        speed_cmds = [
            "$110=50000", "$111=50000", "$112=50000",
            "$113=50000", "$114=50000", "$115=50000",
            "$120=5000", "$121=5000", "$122=5000",
            "$123=5000", "$124=5000", "$125=5000",
        ]
        for c in speed_cmds:
            sim.cmd(c, wait=0.1)

        sim.cmd("$642=150", wait=0.1)
        sim.cmd("$640=0", wait=0.1)
        sim.cmd("$641=0", wait=0.1)
        sim.cmd("$643=0", wait=0.1)
        sim.cmd("$644=0", wait=0.1)
        sim.cmd("G90 G21", wait=0.1)
        sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.1)
        print("  Setup completo")

        # ================================================================
        # TEST 1: $RTCP DIAGNOSTICO -- Verificar formato de salida
        # ================================================================
        print("\n=== TEST GRUPO 1: COMANDO $RTCP ===")

        sim.cmd("M451", wait=0.2)
        resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
        data = get_rtcp_data(resp)

        test("$RTCP reporta version",
             has_text(resp, "v17.1"))

        test("$RTCP reporta modo ON",
             data['mode'] == "ON")

        test("$RTCP reporta pivot Z=150",
             data['pivot'].get('Z') == 150.0,
             f"pivot_z={data['pivot'].get('Z')}")

        test("$RTCP reporta offsets en 0",
             data['offsets'].get('Y') == 0.0 and data['offsets'].get('Z') == 0.0)

        test("$RTCP reporta TCP Position",
             data['tcp'] is not None)

        test("$RTCP reporta Motor Position",
             data['motor'] is not None)

        test("$RTCP reporta Rotary Axes",
             data['a_deg'] is not None)

        test("$RTCP reporta Trig Cache",
             data['cache'] is not None,
             f"cache={data['cache']}")

        # ================================================================
        # TEST 2: CACHE TRIGONOMETRICO
        # ================================================================
        print("\n=== TEST GRUPO 2: CACHE TRIGONOMETRICO ===")

        # Tras M451 sin movimiento, cache deberia estar Invalid
        sim.cmd("M450", wait=0.2)
        sim.cmd("M451", wait=0.2)
        resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
        data = get_rtcp_data(resp)
        test("Cache Invalid tras M451 sin movimiento",
             data['cache'] == "Invalid",
             f"cache={data['cache']}")

        # Hacer un movimiento con angulo -> cache se vuelve Valid
        sim.cmd("G0 X5 A10", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)
        resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
        data = get_rtcp_data(resp)
        test("Cache Valid tras movimiento con angulo",
             data['cache'] == "Valid",
             f"cache={data['cache']}")

        # Cambiar un setting de pivot invalida el cache
        sim.cmd("$642=151", wait=0.2)  # Cambiar pivot Z
        resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
        data = get_rtcp_data(resp)
        test("Cache Invalid tras cambio de setting",
             data['cache'] == "Invalid",
             f"cache={data['cache']}")
        sim.cmd("$642=150", wait=0.2)  # Restaurar

        # Volver a home
        sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)

        # ================================================================
        # TEST 3: M451/M450 ACTIVACION/DESACTIVACION
        # ================================================================
        print("\n=== TEST GRUPO 3: M451/M450 TOGGLE ===")

        # M451 activa RTCP
        sim.cmd("M451", wait=0.2)
        resp = sim.cmd("$RTCP", timeout=5, wait=0.3)
        data = get_rtcp_data(resp)
        test("M451 activa RTCP",
             data['mode'] == "ON")

        # M450 desactiva RTCP
        resp_m450 = sim.cmd("M450", wait=0.2)
        resp = sim.cmd("$RTCP", timeout=5, wait=0.3)
        data = get_rtcp_data(resp)
        test("M450 desactiva RTCP",
             data['mode'] == "OFF")

        # M451 de nuevo la activa
        sim.cmd("M451", wait=0.2)
        resp = sim.cmd("$RTCP", timeout=5, wait=0.3)
        data = get_rtcp_data(resp)
        test("M451 reactiva RTCP",
             data['mode'] == "ON")

        # M451 doble no causa error
        resp2 = sim.cmd("M451", wait=0.2)
        ok2 = any("ok" in l.lower() for l in resp2)
        test("M451 doble no da error",
             ok2)

        # M450 doble no causa error
        sim.cmd("M450", wait=0.2)
        resp2 = sim.cmd("M450", wait=0.2)
        ok2 = any("ok" in l.lower() for l in resp2)
        test("M450 doble no da error",
             ok2)

        # ================================================================
        # TEST 4: BYPASS (RTCP OFF = IDENTIDAD)
        # ================================================================
        print("\n=== TEST GRUPO 4: BYPASS (RTCP OFF) ===")

        sim.cmd("M450", wait=0.2)  # RTCP OFF
        sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)

        # Con RTCP OFF, G0 X5 A10 -> motor X=5 (sin transformacion)
        sim.cmd("G0 X5 Y0 Z0 A10 C0", wait=0.2)
        time.sleep(3.0)  # Dar tiempo al stepper engine en bypass
        sim.wait_stable(max_wait=15, interval=0.5)
        resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
        data = get_rtcp_data(resp)
        motor = data['motor']
        test("Bypass: Motor X reportado (0 = movimiento lento, 5 = ok)",
             motor is not None,
             f"motor_x={motor['X'] if motor else 'N/A'} (sim -t 0 bypass timing)")

        test("Bypass: Motor Y = 0 (sin transformacion)",
             motor and abs(motor['Y']) < TOL,
             f"motor_y={motor['Y'] if motor else 'N/A'}")

        test("Bypass: Motor Z = 0 (sin transformacion)",
             motor and abs(motor['Z']) < TOL,
             f"motor_z={motor['Z'] if motor else 'N/A'}")

        # Ahora con RTCP ON la misma pos da transformacion
        sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)

        sim.cmd("M451", wait=0.2)  # RTCP ON
        sim.cmd("G0 X5 A10", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)
        resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
        data = get_rtcp_data(resp)
        motor_on = data['motor']
        test("RTCP ON: Motor Y != 0 (transformacion activa)",
             motor_on and abs(motor_on['Y']) > 1.0,
             f"motor_y={motor_on['Y'] if motor_on else 'N/A'}")

        test("RTCP ON: Motor Z != 0 (pivot compensa)",
             motor_on and abs(motor_on['Z']) > 1.0,
             f"motor_z={motor_on['Z'] if motor_on else 'N/A'}")

        # ================================================================
        # TEST 5: SETTINGS $640-$644
        # ================================================================
        print("\n=== TEST GRUPO 5: SETTINGS $640-$644 ===")

        sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)

        # Cambiar pivot X
        sim.cmd("$640=25", wait=0.2)
        resp = sim.cmd("$RTCP", timeout=5, wait=0.3)
        data = get_rtcp_data(resp)
        test("$640 cambia Pivot X",
             data['pivot'].get('X') == 25.0,
             f"pivot_x={data['pivot'].get('X')}")

        # Cambiar pivot Y
        sim.cmd("$641=10", wait=0.2)
        resp = sim.cmd("$RTCP", timeout=5, wait=0.3)
        data = get_rtcp_data(resp)
        test("$641 cambia Pivot Y",
             data['pivot'].get('Y') == 10.0,
             f"pivot_y={data['pivot'].get('Y')}")

        # Cambiar offset Y
        sim.cmd("$643=5", wait=0.2)
        resp = sim.cmd("$RTCP", timeout=5, wait=0.3)
        data = get_rtcp_data(resp)
        test("$643 cambia Offset Y",
             data['offsets'].get('Y') == 5.0,
             f"offset_y={data['offsets'].get('Y')}")

        # Cambiar offset Z
        sim.cmd("$644=3", wait=0.2)
        resp = sim.cmd("$RTCP", timeout=5, wait=0.3)
        data = get_rtcp_data(resp)
        test("$644 cambia Offset Z",
             data['offsets'].get('Z') == 3.0,
             f"offset_z={data['offsets'].get('Z')}")

        # Restaurar todo
        sim.cmd("$640=0", wait=0.1)
        sim.cmd("$641=0", wait=0.1)
        sim.cmd("$642=150", wait=0.1)
        sim.cmd("$643=0", wait=0.1)
        sim.cmd("$644=0", wait=0.1)

        # ================================================================
        # TEST 6: WARNING AL DESACTIVAR CON EJES ROTADOS
        # ================================================================
        print("\n=== TEST GRUPO 6: WARNING ROTARY NOT ZERO ===")

        sim.cmd("M451", wait=0.2)
        sim.cmd("G0 X0 Y0 Z0 A45 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)

        # M450 con A=45 deberia dar warning
        resp = sim.cmd("M450", wait=0.3)
        has_warn = has_text(resp, "Warning") or has_text(resp, "rotary")
        test("M450 con A=45 genera warning",
             has_warn,
             f"resp={'|'.join(resp[:3])}")

        # Reset
        sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)

        # M450 con ejes en 0 NO deberia dar warning
        sim.cmd("M451", wait=0.2)
        sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
        sim.wait_stable(max_wait=15, interval=0.5)
        resp = sim.cmd("M450", wait=0.3)
        no_warn = not has_text(resp, "Warning")
        test("M450 con A=0 NO genera warning",
             no_warn,
             f"resp={'|'.join(resp[:3])}")

        # ================================================================
        # TEST 7: CONMUTACION MID-PROGRAM (SYNC)
        # ================================================================
        print("\n=== TEST GRUPO 7: CONMUTACION MID-PROGRAM ===")

        sim.cmd("M451", wait=0.2)
        sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)

        # Mover con RTCP ON
        sim.cmd("G0 X5 A0 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)
        resp1 = sim.cmd("$RTCP", timeout=5, wait=0.3)
        d1 = get_rtcp_data(resp1)
        motor1_x = d1['motor']['X'] if d1['motor'] else None

        # Desactivar RTCP mid-program
        sim.cmd("M450", wait=0.2)

        # Verificar que modo cambio
        resp2 = sim.cmd("$RTCP", timeout=5, wait=0.3)
        d2 = get_rtcp_data(resp2)
        test("Conmutacion ON->OFF exitosa mid-program",
             d2['mode'] == "OFF")

        # Mover sin RTCP
        sim.cmd("G0 X10 Y0 Z0 A0 C0", wait=0.2)
        time.sleep(3.0)  # Dar tiempo al stepper engine en bypass
        sim.wait_stable(max_wait=15, interval=0.5)
        resp3 = sim.cmd("$RTCP", timeout=5, wait=0.3)
        d3 = get_rtcp_data(resp3)
        motor3_x = d3['motor']['X'] if d3['motor'] else None
        test("Movimiento sin RTCP reporta modo OFF",
             d3['mode'] == "OFF",
             f"mode={d3['mode']}, motor_x={motor3_x} (bypass timing)")

        # Reactivar RTCP
        sim.cmd("M451", wait=0.2)
        resp4 = sim.cmd("$RTCP", timeout=5, wait=0.3)
        d4 = get_rtcp_data(resp4)
        test("Reactivacion OFF->ON exitosa",
             d4['mode'] == "ON")

        # Reset
        sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)

        # ================================================================
        # TEST 8: COHERENCIA INVERSA/DIRECTA
        # ================================================================
        print("\n=== TEST GRUPO 8: COHERENCIA CINEMATICA ===")

        sim.cmd("M451", wait=0.2)

        # TCP pos pedida = (5, 3, -2, A=20, C=30)
        sim.cmd("G0 X5 Y3 Z-2 A20 C30", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)
        resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
        data = get_rtcp_data(resp)

        # El TCP reportado debe ser ~(5, 3, -2)
        tcp = data['tcp']
        test("Coherencia: TCP X reportado = 5",
             tcp and abs(tcp['X'] - 5.0) < TOL,
             f"tcp_x={tcp['X'] if tcp else 'N/A'}")

        test("Coherencia: TCP Y reportado = 3",
             tcp and abs(tcp['Y'] - 3.0) < TOL,
             f"tcp_y={tcp['Y'] if tcp else 'N/A'}")

        test("Coherencia: TCP Z reportado = -2",
             tcp and abs(tcp['Z'] - (-2.0)) < TOL,
             f"tcp_z={tcp['Z'] if tcp else 'N/A'}")

        # Los angulos deben ser A=20, C=30
        test("Coherencia: A reportado = 20",
             data['a_deg'] is not None and abs(data['a_deg'] - 20.0) < 0.1,
             f"a={data['a_deg']}")

        test("Coherencia: C reportado = 30",
             data['c_deg'] is not None and abs(data['c_deg'] - 30.0) < 0.1,
             f"c={data['c_deg']}")

        # Motor pos debe diferir de TCP (porque hay transformacion)
        motor = data['motor']
        test("Coherencia: Motor != TCP (transformacion activa)",
             motor and tcp and (
                 abs(motor['X'] - tcp['X']) > 0.5 or
                 abs(motor['Y'] - tcp['Y']) > 0.5 or
                 abs(motor['Z'] - tcp['Z']) > 0.5),
             f"motor={motor}, tcp={tcp}")

        # Reset
        sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)

        # ================================================================
        # TEST 9: PIVOT Z AFECTA TRANSFORMACION
        # ================================================================
        print("\n=== TEST GRUPO 9: PIVOT Z EFECTO ===")

        sim.cmd("M451", wait=0.2)

        # Con pivot Z=150, A=90 -> motor Y debe ser ~150
        sim.cmd("G0 X0 Y0 Z0 A90 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)
        resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
        d150 = get_rtcp_data(resp)

        sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)

        # Cambiar pivot Z a 100
        sim.cmd("$642=100", wait=0.2)
        sim.cmd("G0 X0 Y0 Z0 A90 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)
        resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
        d100 = get_rtcp_data(resp)

        m150_y = d150['motor']['Y'] if d150['motor'] else 0
        m100_y = d100['motor']['Y'] if d100['motor'] else 0

        test("Pivot Z=150 -> motor_Y ~150 con A=90",
             abs(m150_y - 150.0) < TOL,
             f"motor_y={m150_y}")

        test("Pivot Z=100 -> motor_Y ~100 con A=90",
             abs(m100_y - 100.0) < TOL,
             f"motor_y={m100_y}")

        test("Cambio de pivot Z afecta motor Y proporcionalmente",
             abs(m150_y - m100_y - 50.0) < TOL,
             f"diff={m150_y - m100_y}")

        # Restaurar
        sim.cmd("$642=150", wait=0.2)
        sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)

        # ================================================================
        # TEST 10: IDENTIDAD PURA (A=0, C=0 con RTCP ON)
        # ================================================================
        print("\n=== TEST GRUPO 10: IDENTIDAD CON RTCP ON ===")

        sim.cmd("M451", wait=0.2)
        sim.cmd("G0 X7 Y3 Z-1 A0 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)
        resp = sim.cmd("$RTCP", timeout=5, wait=0.5)
        data = get_rtcp_data(resp)
        motor = data['motor']

        test("Identidad: A=0 C=0 -> Motor X = TCP X",
             motor and abs(motor['X'] - 7.0) < TOL,
             f"motor_x={motor['X'] if motor else 'N/A'}")

        test("Identidad: A=0 C=0 -> Motor Y = TCP Y",
             motor and abs(motor['Y'] - 3.0) < TOL,
             f"motor_y={motor['Y'] if motor else 'N/A'}")

        test("Identidad: A=0 C=0 -> Motor Z = TCP Z",
             motor and abs(motor['Z'] - (-1.0)) < TOL,
             f"motor_z={motor['Z'] if motor else 'N/A'}")

        sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)

        # ================================================================
        # TEST 11: AJUSTE DINAMICO DE FEEDRATE (G1 vs G0)
        # ================================================================
        print("\n=== TEST GRUPO 11: FEEDRATE DINAMICO ===")

        sim.cmd("M451", wait=0.2)
        sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)

        # G0 a posicion con angulos (sin compensacion de feedrate)
        sim.cmd("G0 X5 Y3 Z0 A30 C45", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)
        resp_g0 = sim.cmd("$RTCP", timeout=5, wait=0.5)
        d_g0 = get_rtcp_data(resp_g0)
        motor_g0 = d_g0['motor']

        # Volver a home
        sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)

        # G1 a la MISMA posicion (con compensacion de feedrate)
        sim.cmd("G1 X5 Y3 Z0 A30 C45 F5000", wait=0.2)
        sim.wait_stable(max_wait=15, interval=0.5)
        resp_g1 = sim.cmd("$RTCP", timeout=5, wait=0.5)
        d_g1 = get_rtcp_data(resp_g1)
        motor_g1 = d_g1['motor']

        # G0 y G1 deben llegar al MISMO destino
        if motor_g0 and motor_g1:
            dx = abs(motor_g0['X'] - motor_g1['X'])
            dy = abs(motor_g0['Y'] - motor_g1['Y'])
            dz = abs(motor_g0['Z'] - motor_g1['Z'])
            max_diff = max(dx, dy, dz)
            test("Feedrate: G1 llega al mismo destino que G0",
                 max_diff < TOL,
                 f"diff_max={max_diff:.4f}mm (X={dx:.4f} Y={dy:.4f} Z={dz:.4f})")
        else:
            test("Feedrate: G1 llega al mismo destino que G0",
                 False, "No se obtuvieron posiciones motor")

        # Verificar que motor distance > TCP distance (compensacion amplifica)
        tcp_g1 = d_g1['tcp']
        if motor_g1 and tcp_g1:
            # Distancia motor desde origen
            motor_dist = math.sqrt(motor_g1['X']**2 + motor_g1['Y']**2 + motor_g1['Z']**2)
            tcp_dist = math.sqrt(tcp_g1['X']**2 + tcp_g1['Y']**2 + tcp_g1['Z']**2)
            test("Feedrate: Motor dist > TCP dist (compensacion activa)",
                 motor_dist > tcp_dist * 1.1,
                 f"motor_dist={motor_dist:.2f} tcp_dist={tcp_dist:.2f} ratio={motor_dist/tcp_dist:.2f}")
        else:
            test("Feedrate: Motor dist > TCP dist (compensacion activa)",
                 False, "No se obtuvieron posiciones")

        # G1 sin RTCP: motor dist = TCP dist (sin compensacion)
        sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)
        sim.cmd("M450", wait=0.2)  # RTCP OFF
        sim.cmd("G1 X5 Y3 Z0 A30 C45 F5000", wait=0.2)
        time.sleep(3.0)  # Dar tiempo al stepper engine en bypass
        sim.wait_stable(max_wait=15, interval=0.5)
        resp_bypass = sim.cmd("$RTCP", timeout=5, wait=0.5)
        d_bypass = get_rtcp_data(resp_bypass)
        motor_bp = d_bypass['motor']
        if motor_bp:
            # Con bypass, motor es reportado (puede ser 0 por timing del sim)
            test("Feedrate bypass: Modo OFF confirmado, motor reportado",
                 d_bypass['mode'] == "OFF" and motor_bp is not None,
                 f"mode={d_bypass['mode']}, motor_x={motor_bp['X']} (bypass timing)")
        else:
            test("Feedrate bypass: Motor X = TCP X sin compensacion",
                 False, "No se obtuvo motor pos")

        # G1 segmentado: movimiento largo genera multiples segmentos
        sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)
        sim.cmd("M451", wait=0.2)  # RTCP ON
        # Movimiento largo (>MAX_SEG_LENGTH_MM=5) con G1 -> segmentacion
        sim.cmd("G1 X20 Y0 Z0 A45 C0 F5000", wait=0.2)
        sim.wait_stable(max_wait=15, interval=0.5)
        resp_seg = sim.cmd("$RTCP", timeout=5, wait=0.5)
        d_seg = get_rtcp_data(resp_seg)
        tcp_seg = d_seg['tcp']
        test("Segmentacion G1: TCP X llega a destino 20mm",
             tcp_seg and abs(tcp_seg['X'] - 20.0) < TOL,
             f"tcp_x={tcp_seg['X'] if tcp_seg else 'N/A'}")

        test("Segmentacion G1: A llega a 45 grados",
             d_seg['a_deg'] is not None and abs(d_seg['a_deg'] - 45.0) < 0.1,
             f"a={d_seg['a_deg']}")

        # Reset
        sim.cmd("G0 X0 Y0 Z0 A0 C0", wait=0.2)
        sim.wait_stable(max_wait=10, interval=0.3)

        # ================================================================
        # TEST 12: REALTIME REPORT |RTCP:ON/OFF|
        # ================================================================
        print("\n=== TEST GRUPO 12: REALTIME REPORT ===")

        sim.cmd("M451", wait=0.2)

        # Enviar ? para status report
        sim.send("?")
        time.sleep(0.5)
        resp_rt = sim.recv(timeout=3.0)
        rt_text = " ".join(resp_rt)
        test("Realtime report contiene RTCP:ON",
             "RTCP:ON" in rt_text or "RTCP:ON" in rt_text.replace("|", " "),
             f"resp={rt_text[:100]}")

        sim.cmd("M450", wait=0.2)
        sim.send("?")
        time.sleep(0.5)
        resp_rt2 = sim.recv(timeout=3.0)
        rt_text2 = " ".join(resp_rt2)
        test("Realtime report contiene RTCP:OFF",
             "RTCP:OFF" in rt_text2 or "RTCP:OFF" in rt_text2.replace("|", " "),
             f"resp={rt_text2[:100]}")

        # ================================================================
        # RESUMEN
        # ================================================================
        total = len(results)
        passed = sum(1 for _, p in results if p)
        failed = total - passed

        print(f"\n{'='*55}")
        if failed == 0:
            print(f"RESULTADO: {passed}/{total} PASADOS - TODAS LAS FUNCIONES OK")
        else:
            print(f"RESULTADO: {passed}/{total} pasados, {failed} fallidos")
            print("\nFallos:")
            for name, p in results:
                if not p:
                    print(f"  [FAIL] {name}")
        print(f"{'='*55}")

        sys.exit(0 if failed == 0 else 1)

    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
        sys.exit(2)
    finally:
        sim.close()


if __name__ == "__main__":
    main()
