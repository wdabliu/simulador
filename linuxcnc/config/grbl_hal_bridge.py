#!/usr/bin/env python3
"""
grbl_hal_bridge.py - Componente HAL para LinuxCNC

Recibe posiciones del bridge grblHAL via TCP y las pasa
a LinuxCNC como motor-pos-fb via HAL pins.

LinuxCNC es un VISUALIZADOR PASIVO: no ejecuta G-code,
no entiende M451/M450/$RTCP. Solo muestra posiciones.

Uso (desde LinuxCNC):
  loadusr -Wn grbl_remote python3 grbl_hal_bridge.py --ip 192.168.1.76

Argumentos:
  --ip    IP de la maquina Windows con grbl_capture.py (default: 192.168.1.76)
  --port  Puerto TCP del bridge (default: 5007)
"""
import argparse
import socket
import sys
import time

WINDOWS_IP = "192.168.1.76"
BRIDGE_PORT = 5007


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default=WINDOWS_IP, help="IP del bridge Windows")
    parser.add_argument("--port", type=int, default=BRIDGE_PORT, help="Puerto TCP")
    args = parser.parse_args()

    try:
        import hal
        h = hal.component("grbl_remote")
        h.newpin("axis.x.pos", hal.HAL_FLOAT, hal.HAL_OUT)
        h.newpin("axis.y.pos", hal.HAL_FLOAT, hal.HAL_OUT)
        h.newpin("axis.z.pos", hal.HAL_FLOAT, hal.HAL_OUT)
        h.newpin("axis.a.pos", hal.HAL_FLOAT, hal.HAL_OUT)
        h.newpin("axis.b.pos", hal.HAL_FLOAT, hal.HAL_OUT)
        h.newpin("axis.c.pos", hal.HAL_FLOAT, hal.HAL_OUT)
        h.newpin("connected", hal.HAL_BIT, hal.HAL_OUT)
        h.ready()
        use_hal = True
        print("[HAL] Componente grbl_remote listo")
    except ImportError:
        print("[WARN] modulo hal no disponible, modo standalone")
        use_hal = False
        h = None

    def update_pos(x, y, z, a, b, c):
        if use_hal:
            h["axis.x.pos"] = x
            h["axis.y.pos"] = y
            h["axis.z.pos"] = z
            h["axis.a.pos"] = a
            h["axis.b.pos"] = b
            h["axis.c.pos"] = c
            h["connected"] = True

    def set_disconnected():
        if use_hal:
            h["connected"] = False

    while True:
        sock = None
        try:
            print(f"[TCP] Conectando a {args.ip}:{args.port}...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((args.ip, args.port))
            sock.settimeout(None)
            print(f"[TCP] Conectado a {args.ip}:{args.port}")

            buf = b""
            count = 0
            while True:
                data = sock.recv(4096)
                if not data:
                    print("[TCP] Conexion cerrada")
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    msg = line.decode(errors="replace").strip()
                    if not msg.startswith("POS "):
                        continue
                    parts = msg.split()
                    if len(parts) != 7:
                        continue
                    try:
                        vals = [float(p) for p in parts[1:]]
                    except ValueError:
                        continue
                    update_pos(*vals)
                    count += 1
                    if count % 100 == 0:
                        print(f"[DATA] #{count} X={vals[0]:.2f} Y={vals[1]:.2f} Z={vals[2]:.2f} A={vals[3]:.2f}")

        except KeyboardInterrupt:
            print("\n[EXIT]")
            break
        except (ConnectionRefusedError, socket.timeout, OSError) as e:
            print(f"[TCP] Error: {e}")
        finally:
            set_disconnected()
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass

        print("[TCP] Reconectando en 3s...")
        time.sleep(3.0)


if __name__ == "__main__":
    main()
