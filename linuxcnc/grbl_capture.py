#!/usr/bin/env python3
"""
grbl_capture.py - Captura salida paso a paso del simulador grblHAL y retransmite via TCP.

Lanza grblHAL_sim.exe como subproceso, lee stderr (print_steps) y envia
posiciones convertidas a mm/deg a todos los clientes TCP conectados.

Flujo:
  ioSender --TCP:23--> grblHAL_sim.exe --stderr--> grbl_capture.py --TCP:5007--> LinuxCNC

Arquitectura interna:
  stderr_reader (hilo) --> Queue --> main loop --> broadcast TCP

Uso:
  python grbl_capture.py
  python grbl_capture.py --port 5007 --sim-port 23 --rate 0.02
"""
import argparse
import os
import queue
import socket
import subprocess
import sys
import threading
import time

SIM_EXE = r"c:\simulador\build\grblHAL_sim.exe"
CONFIG_FILE = r"c:\simulador\testing_config.ini"
IOSENDER_EXE = r"C:\Users\diseño\Downloads\ioSender.2.0.46\ioSender 2.0.46\ioSender.exe"

DEFAULT_STEPS_PER_MM = [250.0, 250.0, 250.0, 250.0, 250.0, 250.0]


def load_steps_per_mm(config_path):
    steps = list(DEFAULT_STEPS_PER_MM)
    setting_map = {"$100": 0, "$101": 1, "$102": 2, "$103": 3, "$104": 4, "$105": 5}
    try:
        with open(config_path, "r") as f:
            for line in f:
                line = line.split("#")[0].strip()
                if not line or not line.startswith("$"):
                    continue
                if "=" in line:
                    key, val = line.split("=", 1)
                    key = key.strip()
                    if key in setting_map:
                        steps[setting_map[key]] = float(val.strip())
    except FileNotFoundError:
        print(f"[WARN] Config no encontrado: {config_path}, usando defaults")
    return steps


class BridgeServer:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.clients = []
        self.lock = threading.Lock()
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((host, port))
        self.server.listen(5)
        self.running = True

    def accept_loop(self):
        self.server.settimeout(1.0)
        while self.running:
            try:
                conn, addr = self.server.accept()
                with self.lock:
                    self.clients.append(conn)
                print(f"[BRIDGE] Cliente conectado: {addr}")
            except socket.timeout:
                continue

    def broadcast(self, message):
        data = (message + "\n").encode()
        dead = []
        with self.lock:
            for c in self.clients:
                try:
                    c.sendall(data)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    dead.append(c)
            for c in dead:
                self.clients.remove(c)
                try:
                    c.close()
                except Exception:
                    pass

    def close(self):
        self.running = False
        with self.lock:
            for c in self.clients:
                try:
                    c.close()
                except Exception:
                    pass
        self.server.close()


def parse_step_line(line, steps_per_mm):
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    parts = line.split()
    if len(parts) < 7:
        return None
    try:
        timestamp = float(parts[0])
        steps = [int(p) for p in parts[1:7]]
    except (ValueError, IndexError):
        return None
    pos = [steps[i] / steps_per_mm[i] for i in range(6)]
    return timestamp, pos


def send_initial_config(sim_port, config_path):
    print(f"[CONFIG] Enviando parametros al simulador en TCP:{sim_port}...")
    try:
        with open(config_path, "r") as f:
            lines = f.readlines()
    except Exception as e:
        print(f"[CONFIG] Error leyendo {config_path}: {e}")
        return

    # Esperar a que el puerto del simulador abra
    connected = False
    for attempt in range(20):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1.0)
            sock.connect(("127.0.0.1", sim_port))
            connected = True
            break
        except Exception:
            time.sleep(0.5)
            
    if not connected:
        print("[CONFIG] Error: Simulador no abrio el puerto a tiempo.")
        return

    try:
        sock.recv(2048) # limpiar mensaje de bienvenida
    except:
        pass

    sock.settimeout(0.5)
    count = 0
    for line in lines:
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("["):
            continue
        cmd = line.split("#")[0].strip()
        if not cmd:
            continue
            
        sock.sendall((cmd + "\r\n").encode())
        time.sleep(0.05)
        try:
            sock.recv(1024)
        except:
            pass
        count += 1

    sock.close()
    print(f"[CONFIG] {count} comandos enviados exitosamente. Socket liberado.")
    time.sleep(0.5) # pausa para asegurar que el simulador cierre su socket internamente


def stderr_reader(proc, steps_per_mm, msg_queue, stop_event):
    """Hilo dedicado: drena stderr lo mas rapido posible y encola mensajes."""
    try:
        for raw in proc.stderr:
            if stop_event.is_set():
                break
            line = raw.decode(errors="replace").strip()
            result = parse_step_line(line, steps_per_mm)
            if result is None:
                continue
            timestamp, pos = result
            msg = f"POS {pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f} {pos[3]:.4f} {pos[4]:.4f} {pos[5]:.4f}"
            msg_queue.put((timestamp, pos, msg))
    except Exception as e:
        if not stop_event.is_set():
            print(f"[READER] Error: {e}")
    finally:
        msg_queue.put(None)  # sentinel


def main():
    parser = argparse.ArgumentParser(description="Captura grblHAL sim → retransmite a LinuxCNC")
    parser.add_argument("--port", type=int, default=5007, help="Puerto TCP para clientes (default: 5007)")
    parser.add_argument("--sim-port", type=int, default=23, help="Puerto TCP del simulador para ioSender (default: 23)")
    parser.add_argument("--rate", type=float, default=0.02, help="Intervalo de print_steps en seg (default: 0.02)")
    parser.add_argument("--speed", type=float, default=1.0, help="Factor de velocidad del simulador (default: 1.0)")
    args = parser.parse_args()

    steps_per_mm = load_steps_per_mm(CONFIG_FILE)
    print(f"[CONFIG] Steps/mm: {steps_per_mm}")

    eeprom = os.path.join(os.path.dirname(SIM_EXE), "EEPROM.DAT")
    if os.path.exists(eeprom):
        os.remove(eeprom)
        print("[CONFIG] EEPROM.DAT eliminado")

    sim_cmd = [SIM_EXE, "-p", str(args.sim_port), "-r", str(args.rate), "-t", str(args.speed)]
    print(f"[SIM] Lanzando: {' '.join(sim_cmd)}")
    proc = subprocess.Popen(
        sim_cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        creationflags=subprocess.CREATE_NEW_PROCESS_GROUP,
    )
    time.sleep(2.0)
    if proc.poll() is not None:
        err = proc.stderr.read().decode(errors="replace")
        print(f"[ERROR] Simulador fallo: {err}")
        sys.exit(1)

    print(f"[SIM] PID={proc.pid}, puerto TCP={args.sim_port}")
    
    # NUEVO: Enviar configuración MIENTRAS ioSender todavía no acapara el puerto 23
    send_initial_config(args.sim_port, CONFIG_FILE)
    
    print(f"[SIM] Esperando conexion de ioSender en TCP:{args.sim_port}...")

    if os.path.exists(IOSENDER_EXE):
        print(f"[IOSENDER] Lanzando ioSender desde: {IOSENDER_EXE}")
        try:
            subprocess.Popen([IOSENDER_EXE], cwd=os.path.dirname(IOSENDER_EXE), creationflags=subprocess.CREATE_NEW_PROCESS_GROUP)
        except Exception as e:
            print(f"[ERROR] No se pudo iniciar ioSender: {e}")
    else:
        print(f"[WARN] ioSender no encontrado en: {IOSENDER_EXE}")

    bridge = BridgeServer("0.0.0.0", args.port)
    accept_thread = threading.Thread(target=bridge.accept_loop, daemon=True)
    accept_thread.start()
    print(f"[BRIDGE] Escuchando en 0.0.0.0:{args.port}")

    msg_queue = queue.Queue()
    stop_event = threading.Event()
    reader_thread = threading.Thread(
        target=stderr_reader,
        args=(proc, steps_per_mm, msg_queue, stop_event),
        daemon=True,
    )
    reader_thread.start()
    print("[READER] Hilo lector de stderr iniciado")

    line_count = 0
    try:
        while True:
            try:
                item = msg_queue.get(timeout=0.5)
            except queue.Empty:
                if proc.poll() is not None:
                    print("[SIM] Proceso terminado")
                    break
                continue

            if item is None:  # sentinel del reader
                break

            timestamp, pos, msg = item
            bridge.broadcast(msg)
            line_count += 1
            if line_count % 50 == 0:
                print(f"[DATA] t={timestamp:.3f}s X={pos[0]:.2f} Y={pos[1]:.2f} Z={pos[2]:.2f} A={pos[3]:.2f} B={pos[4]:.2f} C={pos[5]:.2f}")
    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl+C")
    finally:
        stop_event.set()
        bridge.close()
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        print(f"[EXIT] {line_count} posiciones procesadas")


if __name__ == "__main__":
    main()
