"""
grblHAL Simulator RTCP Tester - Via TCP con -t 0

Lanza grblHAL_sim.exe con:
  -p 23   : puerto TCP para comunicacion
  -t 0    : velocidad maxima (sin simulacion de tiempo real)
"""

import socket
import subprocess
import logging
import time
import math
import sys

# =====================================================================
# LOGGING
# =====================================================================

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
log = logging.getLogger('rtcp')

# =====================================================================
# CONFIGURACION
# =====================================================================

SIM_EXE = r"c:\simulador\build\grblHAL_sim.exe"
PORT = 23
PIVOT = (0.0, 0.0, 150.0)
TOLERANCE_MM = 0.02


# =====================================================================
# CONEXION TCP
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
        log.info("Esperando 3s para inicializacion...")
        time.sleep(3.0)
        if self.proc.poll() is not None:
            err = self.proc.stderr.read().decode(errors='replace')
            raise RuntimeError(f"Simulador fallo: {err}")
        log.info(f"Simulador iniciado PID={self.proc.pid}")

        # Conectar TCP
        for attempt in range(5):
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(5.0)
                self.sock.connect(("127.0.0.1", PORT))
                log.info(f"TCP conectado a 127.0.0.1:{PORT}")
                time.sleep(1.0)
                self._drain()
                return
            except (ConnectionRefusedError, socket.timeout, OSError):
                log.warning(f"TCP intento {attempt+1}/5 fallido")
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
        raw = cmd.strip() + "\r\n"
        log.debug(f"TX >> {cmd.strip()}")
        self.sock.sendall(raw.encode())

    def recv(self, timeout=5.0):
        """Recibe lineas hasta ok/error o timeout."""
        lines = []
        self.sock.settimeout(timeout)
        deadline = time.time() + timeout
        t0 = time.time()
        while time.time() < deadline:
            try:
                d = self.sock.recv(4096)
                if not d:
                    log.debug("RX: socket cerrado")
                    break
                log.debug(f"RX raw ({len(d)}b): {d[:100]}")
                self.buf += d
                while b"\n" in self.buf:
                    raw, self.buf = self.buf.split(b"\n", 1)
                    line = raw.decode(errors='replace').strip().strip('\r')
                    if line:
                        lines.append(line)
                        log.debug(f"RX << {line}")
                        if line.lower() == "ok" or line.lower().startswith("error"):
                            log.debug(f"RX completado en {time.time()-t0:.2f}s ({len(lines)} lineas)")
                            return lines
            except socket.timeout:
                log.debug(f"RX timeout ({timeout}s), {len(lines)} lineas recibidas")
                break
        return lines

    def cmd(self, command, timeout=5.0, wait=0.0):
        """Envia comando, espera, y recibe respuesta."""
        if wait > 0:
            time.sleep(wait)
        self.send(command)
        time.sleep(0.2)
        return self.recv(timeout)

    def wait_stable(self, max_wait=60.0, interval=2.0):
        """Espera polling $RTCP hasta que Motor Position no cambie."""
        prev = None
        deadline = time.time() + max_wait
        while time.time() < deadline:
            time.sleep(interval)
            resp = self.cmd("$RTCP", timeout=5, wait=0.3)
            motor = None
            found = False
            for line in resp:
                if "Motor Position" in line:
                    found = True
                    continue
                if found and "X =" in line:
                    motor = line.strip()
                    break
            log.debug(f"wait_stable: motor={motor}  prev={prev}")
            if motor and motor == prev:
                log.info(f"Posicion estable: {motor}")
                return True
            prev = motor
        log.warning(f"wait_stable: timeout {max_wait}s")
        return False

    def close(self):
        log.info("Cerrando conexion y proceso...")
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
# CINEMATICA DE REFERENCIA
# =====================================================================

def rtcp_inv(x, y, z, a_deg, c_deg, px=0, py=0, pz=150):
    a, c = math.radians(a_deg), math.radians(c_deg)
    ca, sa, cc, sc = math.cos(a), math.sin(a), math.cos(c), math.sin(c)
    dx, dy, dz = x - px, y - py, z - pz
    rx = cc*dx - sc*dy
    ry = sc*dx + cc*dy
    return (rx + px, ca*ry - sa*dz + py, sa*ry + ca*dz + pz)


# =====================================================================
# PARSEO
# =====================================================================

def get_motor(lines):
    """Extrae Motor Position de respuesta $RTCP."""
    motor_section = False
    for line in lines:
        if "Motor Position" in line:
            motor_section = True
            continue
        if motor_section and "X =" in line:
            parts = line.split()
            vals = {}
            for i, p in enumerate(parts):
                if p in ('X','Y','Z') and i+2 < len(parts) and parts[i+1] == '=':
                    try: vals[p] = float(parts[i+2])
                    except: pass
            if len(vals) == 3:
                return (vals['X'], vals['Y'], vals['Z'])
    return None


# =====================================================================
# MAIN
# =====================================================================

def main():
    sim = Sim()
    try:
        sim.start()

        # --- CONFIGURACION ---
        print("\n=== CONFIGURACION ===")
        setup = [
            # Velocidades maximas altas para tests rapidos
            "$110=50000", "$111=50000", "$112=50000", "$113=50000", "$114=50000", "$115=50000",
            # Aceleraciones altas
            "$120=5000", "$121=5000", "$122=5000", "$123=5000", "$124=5000", "$125=5000",
            # Pivot RTCP
            "$642=150", "$640=0", "$641=0", "$643=0", "$644=0",
            # Modo
            "G90 G21", "G0 X0 Y0 Z0 A0 C0", "M451"
        ]
        for c in setup:
            r = sim.cmd(c, wait=0.2)
            ok = "ok" if any("ok" in x.lower() for x in r) else str(r)
            print(f"  {c:35s} {ok}")

        # Verificar RTCP
        time.sleep(1.0)
        r = sim.cmd("$RTCP", timeout=5, wait=1.0)
        mode = "ON" if any("ON" in l for l in r) else "OFF"
        print(f"\n  RTCP Mode: {mode}")
        if mode != "ON":
            print("  RTCP no se activo. Abortando.")
            return

        # --- TESTS ---
        print(f"\n=== TESTS RTCP (Pivot Z={PIVOT[2]}mm, Tol={TOLERANCE_MM}mm) ===\n")

        tests = [
            ("Identidad A0 C0",        "G0 X100 Y0 Z0 A0 C0",      100,0,0, 0,0),
            ("C=90",                    "G0 X100 Y0 Z0 A0 C90",     100,0,0, 0,90),
            ("A=90 pivot",              "G0 X0 Y0 Z0 A90 C0",       0,0,0, 90,0),
            ("A=45",                    "G0 X0 Y0 Z0 A45 C0",       0,0,0, 45,0),
            ("A45 C30 XYZ",             "G0 X50 Y25 Z-10 A45 C30",  50,25,-10, 45,30),
            ("C=180 inv X",             "G0 X100 Y0 Z0 A0 C180",    100,0,0, 0,180),
            ("A90 C90",                 "G0 X50 Y0 Z0 A90 C90",     50,0,0, 90,90),
            ("A-30 C-45",               "G0 X30 Y20 Z-5 A-30 C-45", 30,20,-5, -30,-45),
            ("C=360 ident",             "G0 X75 Y0 Z0 A0 C360",     75,0,0, 0,360),
            ("A=80 extremo",            "G0 X10 Y10 Z0 A80 C60",    10,10,0, 80,60),
        ]

        passed = 0
        for i, (name, gcode, x, y, z, a, c) in enumerate(tests, 1):
            # Calcular referencia
            exp = rtcp_inv(x, y, z, a, c, PIVOT[0], PIVOT[1], PIVOT[2])

            # Enviar movimiento y esperar que termine
            sim.cmd(gcode, timeout=10, wait=0.3)
            sim.wait_stable(max_wait=30, interval=0.5)

            # Leer posicion
            resp = sim.cmd("$RTCP", timeout=5, wait=1.0)
            motor = get_motor(resp)

            if motor is None:
                print(f"  [{i:2d}] [FAIL] {name}: sin respuesta Motor")
                # Reset
                sim.cmd("G0 X0 Y0 Z0 A0 C0", timeout=10, wait=0.5)
                time.sleep(SETTLE_TIME)
                continue

            diffs = [abs(exp[j] - motor[j]) for j in range(3)]
            md = max(diffs)
            ax = ['X','Y','Z'][diffs.index(md)]

            if md <= TOLERANCE_MM:
                print(f"  [{i:2d}] [PASS] {name}")
                print(f"         Motor: X={motor[0]:.3f} Y={motor[1]:.3f} Z={motor[2]:.3f}")
                passed += 1
            else:
                print(f"  [{i:2d}] [FAIL] {name}  (diff {md:.3f}mm en {ax})")
                print(f"         Esperado: X={exp[0]:.3f} Y={exp[1]:.3f} Z={exp[2]:.3f}")
                print(f"         Actual:   X={motor[0]:.3f} Y={motor[1]:.3f} Z={motor[2]:.3f}")

            # Reset
            sim.cmd("G0 X0 Y0 Z0 A0 C0", timeout=10, wait=0.3)
            sim.wait_stable(max_wait=30, interval=0.5)

        # Resumen
        total = len(tests)
        print(f"\n{'='*50}")
        if passed == total:
            print(f"RESULTADO: {passed}/{total} PASADOS - CINEMATICA VERIFICADA")
        else:
            print(f"RESULTADO: {passed}/{total} pasados")
        print(f"{'='*50}")

    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
    finally:
        sim.close()

if __name__ == "__main__":
    main()
