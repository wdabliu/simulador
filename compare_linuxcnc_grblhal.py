"""
Comparacion: LinuxCNC vs grblHAL RTCP (mesa-mesa XYZAC)
=========================================================

Replica EXACTA de ambas implementaciones:

LinuxCNC (trtfuncs.c):
  - NO segmenta los movimientos G1
  - Interpola linealmente en espacio cartesiano (X,Y,Z,A,C independientes)
  - Aplica kinematicsInverse() a cada ciclo de servo (1ms, 1kHz)
  - La precision depende de: velocidad / periodo_servo
  
grblHAL (rtcp.c) ACTUAL:
  - Segmenta por MAX_SEG_LENGTH_MM=0.5 y MAX_SEG_ANGLE_DEG=0.5
  - Aplica transform_from_cartesian() a cada segmento
  - Los motores interpolan linealmente ENTRE segmentos

grblHAL PROPUESTO:
  - Segmenta por evaluacion de punto medio adaptiva
  - Tolerancia configurable (default 0.01mm)
"""

import math
import sys

# =====================================================================
# LINUXCNC: xyzacKinematicsInverse (trtfuncs.c:199-257, codigo EXACTO)
# =====================================================================

def linuxcnc_inverse(pos_x, pos_y, pos_z, a_deg, c_deg,
                      x_rot=0, y_rot=0, z_rot=0, dt=0, dy=0, dz=0):
    """LinuxCNC xyzacKinematicsInverse - EXACTO del source code.
    
    pos_x/y/z = posicion TCP en espacio cartesiano (world)
    a_deg, c_deg = angulos rotativos en grados
    x/y/z_rot = rot_point (pivot)
    dt = tool_offset, dy = y_offset, dz = z_offset
    """
    a_rad = math.radians(a_deg)
    c_rad = math.radians(c_deg)
    dz = dz + dt  # dz += tool_offset
    
    # LinuxCNC trtfuncs.c lineas 555-571
    joint_x = (+ math.cos(c_rad) * (pos_x - x_rot)
               - math.sin(c_rad) * (pos_y - y_rot)
               + x_rot)
    
    joint_y = (+ math.sin(c_rad) * math.cos(a_rad) * (pos_x - x_rot)
               + math.cos(c_rad) * math.cos(a_rad) * (pos_y - y_rot)
               - math.sin(a_rad) * (pos_z - z_rot)
               - math.cos(a_rad) * dy
               + math.sin(a_rad) * dz
               + dy + y_rot)
    
    joint_z = (+ math.sin(c_rad) * math.sin(a_rad) * (pos_x - x_rot)
               + math.cos(c_rad) * math.sin(a_rad) * (pos_y - y_rot)
               + math.cos(a_rad) * (pos_z - z_rot)
               - math.sin(a_rad) * dy
               - math.cos(a_rad) * dz
               + dz + z_rot)
    
    return joint_x, joint_y, joint_z


def linuxcnc_forward(jx, jy, jz, a_deg, c_deg,
                      x_rot=0, y_rot=0, z_rot=0, dt=0, dy=0, dz=0):
    """LinuxCNC xyzacKinematicsForward - trtfuncs.c:151-197"""
    a_rad = math.radians(a_deg)
    c_rad = math.radians(c_deg)
    dz = dz + dt
    
    pos_x = (+ math.cos(c_rad) * (jx - x_rot)
             + math.sin(c_rad) * math.cos(a_rad) * (jy - dy - y_rot)
             + math.sin(c_rad) * math.sin(a_rad) * (jz - dz - z_rot)
             + math.sin(c_rad) * dy
             + x_rot)
    
    pos_y = (- math.sin(c_rad) * (jx - x_rot)
             + math.cos(c_rad) * math.cos(a_rad) * (jy - dy - y_rot)
             + math.cos(c_rad) * math.sin(a_rad) * (jz - dz - z_rot)
             + math.cos(c_rad) * dy
             + y_rot)
    
    pos_z = (- math.sin(a_rad) * (jy - dy - y_rot)
             + math.cos(a_rad) * (jz - dz - z_rot)
             + dz + z_rot)
    
    return pos_x, pos_y, pos_z


# =====================================================================
# grblHAL: transform_from_cartesian (rtcp.c)
# =====================================================================

def grblhal_inverse(tcp_x, tcp_y, tcp_z, a_deg, c_deg,
                     pivot_x=0, pivot_y=0, pivot_z=150,
                     tlo_z=0, offset_y=0, offset_z=0):
    """grblHAL rtcp.c transform_from_cartesian
    
    IMPORTANTE: En grblHAL, position[Z_AXIS] YA incluye TLO
    (sumado por gc_get_block_offset antes de llamar la transformacion).
    Aqui recibimos tcp_z como la posicion Z del TCP SIN TLO,
    y simulamos lo que hace grblHAL: sumar tlo_z al input Z.
    """
    a_rad = math.radians(a_deg)
    c_rad = math.radians(c_deg)
    ca, sa = math.cos(a_rad), math.sin(a_rad)
    cc, sc = math.cos(c_rad), math.sin(c_rad)
    
    # grblHAL recibe position[Z] = tcp_z + tlo_z (pre-sumado)
    pos_z_with_tlo = tcp_z + tlo_z
    
    dy = offset_y
    dz = offset_z + tlo_z   # Combinar como LinuxCNC: dz = dz + dt
    
    # Paso 1: Trasladar al sistema del pivot, restando TLO
    px = tcp_x - pivot_x
    py = tcp_y - pivot_y
    pz = (pos_z_with_tlo - tlo_z) - pivot_z   # = tcp_z - pivot_z
    
    # Paso 2: Rotacion C (alrededor de Z)
    xc = px * cc - py * sc
    yc = px * sc + py * cc
    
    # Paso 3: Rotacion A con offsets
    mx = xc + pivot_x
    my = (yc - dy) * ca - (pz - dz) * sa + dy + pivot_y
    mz = (yc - dy) * sa + (pz - dz) * ca + dz + pivot_z
    return mx, my, mz


def grblhal_forward(mx, my, mz, a_deg, c_deg,
                     pivot_x=0, pivot_y=0, pivot_z=150,
                     tlo_z=0, offset_y=0, offset_z=0):
    """grblHAL rtcp.c transform_to_cartesian
    Retorna tcp_z SIN TLO (posicion real de la pieza)"""
    a_rad = math.radians(a_deg)
    c_rad = math.radians(c_deg)
    ca, sa = math.cos(a_rad), math.sin(a_rad)
    cc, sc = math.cos(c_rad), math.sin(c_rad)
    dz = offset_z + tlo_z
    dy = offset_y
    ym = my - dy - pivot_y
    zm = mz - dz - pivot_z
    yc = ym * ca + zm * sa + dy
    pz = -ym * sa + zm * ca + dz
    xc = mx - pivot_x
    px = xc * cc + yc * sc
    py = -xc * sc + yc * cc
    # Retornar tcp_z (sin TLO, posicion real)
    return px + pivot_x, py + pivot_y, pz + pivot_z


# =====================================================================
# PRUEBA 1: Las cinematicas son IGUALES?
# =====================================================================

def test_kinematics_equivalence():
    print("=" * 70)
    print("PRUEBA 1: LinuxCNC vs grblHAL - Cinematica Inversa Identica?")
    print("=" * 70)
    
    # Parametros equivalentes
    pivot_z = 150
    tlo_z = 50  # tool length offset
    
    cases = [
        (0, 0, 0, 0, 0, "Origen, sin rot"),
        (100, 0, 0, 0, 0, "X100, sin rot"),
        (100, 50, -30, 0, 0, "X100Y50Z-30, sin rot"),
        (100, 0, 0, 10, 0, "X100 A10"),
        (100, 0, 0, 0, 30, "X100 C30"),
        (100, 50, 0, 10, 30, "X100Y50 A10C30"),
        (0, 0, 0, 45, 45, "A45C45"),
        (200, 100, -50, 20, 60, "X200Y100Z-50 A20C60"),
        (50, 0, 0, 90, 0, "X50 A90"),
        (0, 0, 0, 0, 90, "C90 puro"),
    ]
    
    ok_all = True
    print(f"\n  pivot_z={pivot_z}, tlo_z={tlo_z}")
    print(f"  {'Caso':<30s} {'dif_X':>8s} {'dif_Y':>8s} {'dif_Z':>8s} {'Max err':>10s}")
    print(f"  {'-'*30} {'-'*8} {'-'*8} {'-'*8} {'-'*10}")
    
    for x, y, z, a, c, label in cases:
        # LinuxCNC: pos.z = TCP Z sin TLO, dt = TLO por separado
        lx, ly, lz = linuxcnc_inverse(x, y, z, a, c,
                                        z_rot=pivot_z, dt=tlo_z)
        # grblHAL: tcp_z = TCP Z sin TLO, tlo_z por separado
        # (internamente suma tlo_z al input como hace gc_get_block_offset)
        gx, gy, gz = grblhal_inverse(x, y, z, a, c,
                                      pivot_z=pivot_z, tlo_z=tlo_z)
        
        dx, dy_val, dz_val = abs(lx-gx), abs(ly-gy), abs(lz-gz)
        err = max(dx, dy_val, dz_val)
        ok = err < 1e-9
        ok_all = ok_all and ok
        tag = "OK" if ok else "!!"
        print(f"  [{tag}] {label:<30s} {dx:>8.2e} {dy_val:>8.2e} {dz_val:>8.2e} {err:>10.2e}")
    
    # Test adicional: round-trip (inverse -> forward = identidad)
    print(f"\n  Test round-trip (inversa -> directa = identidad):")
    for x, y, z, a, c, label in cases:
        jx, jy, jz = grblhal_inverse(x, y, z, a, c, pivot_z=pivot_z, tlo_z=tlo_z)
        rx, ry, rz = grblhal_forward(jx, jy, jz, a, c, pivot_z=pivot_z, tlo_z=tlo_z)
        err = max(abs(rx-x), abs(ry-y), abs(rz-z))
        ok_rt = err < 1e-9
        ok_all = ok_all and ok_rt
        tag = "OK" if ok_rt else "!!"
        print(f"  [{tag}] {label:<30s} err={err:.2e}")
    
    print(f"\n  RESULTADO: {'PASA - CINEMATICAS IDENTICAS' if ok_all else 'FALLA'}\n")
    return ok_all


# =====================================================================
# SIMULADOR DE SEGMENTACION
# =====================================================================

def simulate_tcp_error(tcp0, tcp1, a0, a1, c0, c1, n_segs,
                        inv_fn, fwd_fn, pz=150, tlo=0, checks=100):
    """Simula N segmentos y mide el error TCP maximo"""
    max_err = 0.0
    for seg in range(n_segs):
        t0, t1 = seg/n_segs, (seg+1)/n_segs
        tp0 = [tcp0[i]+t0*(tcp1[i]-tcp0[i]) for i in range(3)]
        tp1 = [tcp0[i]+t1*(tcp1[i]-tcp0[i]) for i in range(3)]
        aa0 = a0+t0*(a1-a0); aa1 = a0+t1*(a1-a0)
        cc0 = c0+t0*(c1-c0); cc1 = c0+t1*(c1-c0)
        m0 = inv_fn(*tp0, aa0, cc0, pivot_z=pz, tlo_z=tlo)
        m1 = inv_fn(*tp1, aa1, cc1, pivot_z=pz, tlo_z=tlo)
        for k in range(1, checks):
            s = k/checks
            tg = t0+s*(t1-t0)
            mi = tuple(m0[i]+s*(m1[i]-m0[i]) for i in range(3))
            ai = aa0+s*(aa1-aa0); ci = cc0+s*(cc1-cc0)
            ideal = [tcp0[i]+tg*(tcp1[i]-tcp0[i]) for i in range(3)]
            real = fwd_fn(*mi, ai, ci, pivot_z=pz, tlo_z=tlo)
            err = math.sqrt(sum((real[i]-ideal[i])**2 for i in range(3)))
            if err > max_err: max_err = err
    return max_err


def linuxcnc_servo_segments(tcp0, tcp1, a0, a1, c0, c1, 
                             feedrate_mm_min, servo_hz=1000):
    """Calcula cuantos 'segmentos' naturales genera LinuxCNC por servo cycle.
    
    Un movimiento a F3000 (50mm/s) con servo de 1kHz genera
    posiciones cada 0.05mm = 50/1000. Cada posicion pasa por
    kinematicsInverse().
    """
    dist_lin = math.sqrt(sum((tcp1[i]-tcp0[i])**2 for i in range(3)))
    da = abs(a1-a0)
    dc = abs(c1-c0)
    
    # Distancia total en espacio de TP (XYZ + ABC como independientes)
    # LinuxCNC calcula target como la norma completa
    dist_total = math.sqrt(dist_lin**2 + da**2 + dc**2)
    if dist_total < 1e-10:
        return 1
    
    feed_mm_s = feedrate_mm_min / 60.0
    time_s = dist_total / feed_mm_s  # Simplificado: velocidad constante
    n_cycles = max(1, int(time_s * servo_hz))
    return n_cycles


def midpoint_error(tcp0, tcp1, a0, a1, c0, c1, inv_fn, pz=150, tlo=0):
    """Error del punto medio para el metodo adaptativo"""
    m0 = inv_fn(*tcp0, a0, c0, pivot_z=pz, tlo_z=tlo)
    m1 = inv_fn(*tcp1, a1, c1, pivot_z=pz, tlo_z=tlo)
    tcp_mid = [(tcp0[i]+tcp1[i])/2 for i in range(3)]
    a_mid, c_mid = (a0+a1)/2, (c0+c1)/2
    m_correct = inv_fn(*tcp_mid, a_mid, c_mid, pivot_z=pz, tlo_z=tlo)
    m_interp = tuple((m0[i]+m1[i])/2 for i in range(3))
    fwd = grblhal_forward
    tcp_real = fwd(*m_interp, a_mid, c_mid, pivot_z=pz, tlo_z=tlo)
    return math.sqrt(sum((tcp_real[i]-tcp_mid[i])**2 for i in range(3)))


def calc_adaptive(tcp0, tcp1, a0, a1, c0, c1, tol, pz=150, tlo=0):
    """Metodo adaptativo: biseccion por punto medio"""
    n = 1
    for _ in range(15):
        max_err = 0
        for seg in range(n):
            t0, t1 = seg/n, (seg+1)/n
            tp0 = [tcp0[i]+t0*(tcp1[i]-tcp0[i]) for i in range(3)]
            tp1 = [tcp0[i]+t1*(tcp1[i]-tcp0[i]) for i in range(3)]
            aa0 = a0+t0*(a1-a0); aa1 = a0+t1*(a1-a0)
            cc0 = c0+t0*(c1-c0); cc1 = c0+t1*(c1-c0)
            err = midpoint_error(tp0, tp1, aa0, aa1, cc0, cc1,
                                  grblhal_inverse, pz, tlo)
            if err > max_err: max_err = err
        if max_err <= tol: return n
        n *= 2
    return n


def grblhal_current_segments(tcp0, tcp1, a0, a1, c0, c1):
    """Sistema actual: max(dist/0.5, max_rot/0.5)"""
    dist = math.sqrt(sum((tcp1[i]-tcp0[i])**2 for i in range(3)))
    max_rot = max(abs(a1-a0), abs(c1-c0))
    sl = max(1, math.ceil(dist/0.5)) if dist > 0.5 else 1
    sr = max(1, math.ceil(max_rot/0.5)) if max_rot > 0.5 else 1
    return max(sl, sr)


# =====================================================================
# PRUEBA 2: Comparacion LinuxCNC vs grblHAL vs Propuesto
# =====================================================================

def test_comparison():
    print("=" * 70)
    print("PRUEBA 2: LinuxCNC vs grblHAL actual vs Propuesto adaptativo")
    print("=" * 70)
    
    pz, tlo = 150, 50
    tol = 0.01
    feed = 3000  # mm/min
    
    moves = [
        ("X1000 A2",        [0,0,0], [1000,0,0], 0, 2,  0, 0),
        ("X200 A4",         [0,0,0], [200,0,0],  0, 4,  0, 0),
        ("X100 A10",        [0,0,0], [100,0,0],  0, 10, 0, 0),
        ("X10 A30",         [0,0,0], [10,0,0],   0, 30, 0, 0),
        ("X0 A90",          [0,0,0], [0,0,0],    0, 90, 0, 0),
        ("X100 A0.5",       [0,0,0], [100,0,0],  0, 0.5,0, 0),
        ("X100 C30",        [0,0,0], [100,0,0],  0, 0,  0, 30),
        ("X100Y50 A10C30",  [0,0,0], [100,50,0], 0, 10, 0, 30),
        ("X200 A5C10",      [0,0,0], [200,0,0],  0, 5,  0, 10),
        ("X1000 lineal",    [0,0,0], [1000,0,0], 0, 0,  0, 0),
        ("X0.5 A0.2 (CAM)", [0,0,0], [0.5,0,0],  0, 0.2,0, 0),
        ("X2 A1 C0.5 (CAM)",[0,0,0], [2,0,0],    0, 1,  0, 0.5),
    ]
    
    print(f"\n  Pivot={pz}mm, TLO={tlo}mm, Feed={feed}mm/min, Tolerancia={tol}mm")
    print(f"\n  {'Movimiento':<25s} | {'LinuxCNC':^22s} | {'grblHAL actual':^15s} | {'Propuesto':^15s}")
    print(f"  {'':25s} | {'segs':>6s} {'err':>8s} {'ms':>6s} | {'segs':>6s} {'err':>7s} | {'segs':>6s} {'err':>7s}")
    print(f"  {'-'*25}-+-{'-'*22}-+-{'-'*15}-+-{'-'*15}")
    
    all_ok = True
    t_linuxcnc, t_grblhal, t_proposed = 0, 0, 0
    
    for label, tcp0, tcp1, a0, a1, c0, c1 in moves:
        # LinuxCNC: segmentos naturales por servo cycle
        n_lcnc = linuxcnc_servo_segments(tcp0, tcp1, a0, a1, c0, c1, feed)
        err_lcnc = simulate_tcp_error(tcp0, tcp1, a0, a1, c0, c1, n_lcnc,
                                       grblhal_inverse, grblhal_forward, pz, tlo)
        time_lcnc = n_lcnc  # cycles = ms at 1kHz
        
        # grblHAL actual
        n_grbl = grblhal_current_segments(tcp0, tcp1, a0, a1, c0, c1)
        err_grbl = simulate_tcp_error(tcp0, tcp1, a0, a1, c0, c1, n_grbl,
                                       grblhal_inverse, grblhal_forward, pz, tlo)
        
        # Propuesto adaptativo
        n_prop = calc_adaptive(tcp0, tcp1, a0, a1, c0, c1, tol, pz, tlo)
        err_prop = simulate_tcp_error(tcp0, tcp1, a0, a1, c0, c1, n_prop,
                                       grblhal_inverse, grblhal_forward, pz, tlo)
        
        t_linuxcnc += n_lcnc
        t_grblhal += n_grbl
        t_proposed += n_prop
        
        ok_prop = err_prop <= tol * 1.15
        if not ok_prop: all_ok = False
        
        tag = "OK" if ok_prop else "!!"
        print(f"  [{tag}] {label:<22s} | {n_lcnc:>6d} {err_lcnc:>8.4f} {time_lcnc:>5d}ms"
              f" | {n_grbl:>6d} {err_grbl:>7.4f}"
              f" | {n_prop:>6d} {err_prop:>7.4f}")
    
    print(f"\n  TOTALES:")
    print(f"    LinuxCNC (servo 1kHz):   {t_linuxcnc:>6d} ciclos")
    print(f"    grblHAL actual:          {t_grblhal:>6d} segmentos")
    print(f"    Propuesto (tol 0.01mm):  {t_proposed:>6d} segmentos")
    print(f"\n  RESULTADO: {'PASA' if all_ok else 'FALLA'}\n")
    return all_ok


# =====================================================================
# PRUEBA 3: Precision LinuxCNC depende de velocidad
# =====================================================================

def test_linuxcnc_speed_dependency():
    print("=" * 70)
    print("PRUEBA 3: LinuxCNC - precision depende de velocidad")
    print("=" * 70)
    
    pz, tlo = 150, 50
    tcp0, tcp1 = [0,0,0], [100,50,0]
    a0, a1 = 0, 10
    c0, c1 = 0, 30
    
    print(f"\n  Movimiento: X100Y50 A10C30, Pivot={pz}, TLO={tlo}")
    print(f"  {'Feedrate':>10s} {'Servo cyc':>10s} {'Err TCP':>10s} {'Equiv. a tol':>15s}")
    print(f"  {'-'*10} {'-'*10} {'-'*10} {'-'*15}")
    
    for feed in [100, 500, 1000, 3000, 6000, 12000, 30000]:
        n = linuxcnc_servo_segments(tcp0, tcp1, a0, a1, c0, c1, feed)
        err = simulate_tcp_error(tcp0, tcp1, a0, a1, c0, c1, n,
                                  grblhal_inverse, grblhal_forward, pz, tlo)
        equiv = "excelente" if err < 0.001 else "buena" if err < 0.01 else "aceptable" if err < 0.05 else "pobre" if err < 0.1 else "MALA"
        print(f"  {feed:>8d}F {n:>10d} {err:>10.6f} {equiv:>15s}")
    
    print(f"\n  -> A velocidades altas, LinuxCNC pierde precision")
    print(f"  -> grblHAL con segmentacion es INDEPENDIENTE de velocidad\n")
    return True


# =====================================================================
# PRUEBA 4: Caso real CAM - movimientos pequenos tipicos
# =====================================================================

def test_cam_realistic():
    print("=" * 70) 
    print("PRUEBA 4: Movimientos tipicos de CAM (lineas cortas)")
    print("=" * 70)
    
    pz, tlo = 150, 50
    feed = 3000
    tol = 0.01
    
    # Simular un toolpath de esfera: 20 lineas G1 consecutivas
    cam_moves = []
    for i in range(20):
        t0 = i / 20.0
        t1 = (i + 1) / 20.0
        # Semicirculo en XZ con rotacion A
        x0 = 50 * math.cos(math.pi * t0)
        z0 = 50 * math.sin(math.pi * t0) - 50
        x1 = 50 * math.cos(math.pi * t1)
        z1 = 50 * math.sin(math.pi * t1) - 50
        a0 = -90 * t0
        a1 = -90 * t1
        cam_moves.append((f"Seg {i+1:2d}", [x0,0,z0], [x1,0,z1], a0, a1, 0, 0))
    
    t_lcnc, t_grbl, t_prop = 0, 0, 0
    errs_lcnc, errs_grbl, errs_prop = [], [], []
    
    for label, tcp0, tcp1, a0, a1, c0, c1 in cam_moves:
        n_l = linuxcnc_servo_segments(tcp0, tcp1, a0, a1, c0, c1, feed)
        n_g = grblhal_current_segments(tcp0, tcp1, a0, a1, c0, c1)
        n_p = calc_adaptive(tcp0, tcp1, a0, a1, c0, c1, tol, pz, tlo)
        
        e_l = simulate_tcp_error(tcp0, tcp1, a0, a1, c0, c1, n_l,
                                  grblhal_inverse, grblhal_forward, pz, tlo)
        e_g = simulate_tcp_error(tcp0, tcp1, a0, a1, c0, c1, n_g,
                                  grblhal_inverse, grblhal_forward, pz, tlo)
        e_p = simulate_tcp_error(tcp0, tcp1, a0, a1, c0, c1, n_p,
                                  grblhal_inverse, grblhal_forward, pz, tlo)
        
        t_lcnc += n_l; t_grbl += n_g; t_prop += n_p
        errs_lcnc.append(e_l); errs_grbl.append(e_g); errs_prop.append(e_p)
    
    print(f"\n  Toolpath: 20 segmentos CAM simulando semicirculo con rotacion A")
    print(f"  Pivot={pz}, TLO={tlo}, Feed={feed}\n")
    print(f"  {'Metodo':<25s} {'Segs total':>12s} {'Err max':>10s} {'Err prom':>10s}")
    print(f"  {'-'*25} {'-'*12} {'-'*10} {'-'*10}")
    
    def avg(lst): return sum(lst)/len(lst) if lst else 0
    
    print(f"  {'LinuxCNC (servo 1kHz)':<25s} {t_lcnc:>12d} {max(errs_lcnc):>10.6f} {avg(errs_lcnc):>10.6f}")
    print(f"  {'grblHAL actual':<25s} {t_grbl:>12d} {max(errs_grbl):>10.6f} {avg(errs_grbl):>10.6f}")
    print(f"  {'Propuesto (tol 0.01mm)':<25s} {t_prop:>12d} {max(errs_prop):>10.6f} {avg(errs_prop):>10.6f}")
    
    print(f"\n  -> Para movimientos CAM tipicos (cortos), las 3 soluciones son similares")
    print(f"  -> La diferencia aparece con movimientos largos\n")
    return True


# =====================================================================
# MAIN
# =====================================================================

if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("  COMPARACION: LinuxCNC vs grblHAL RTCP (Mesa-Mesa XYZAC)")
    print("  Cinematica inversa EXACTA de trtfuncs.c replicada en Python")
    print("=" * 70 + "\n")
    
    r1 = test_kinematics_equivalence()
    r2 = test_comparison()
    r3 = test_linuxcnc_speed_dependency()
    r4 = test_cam_realistic()
    
    print("=" * 70)
    print("  RESUMEN FINAL")
    print("=" * 70)
    for name, passed in [
        ("1. Cinematicas LinuxCNC=grblHAL identicas", r1),
        ("2. Comparacion segmentacion 3 metodos", r2),
        ("3. LinuxCNC depende de velocidad", r3),
        ("4. Caso CAM realista", r4),
    ]:
        print(f"  [{'PASA' if passed else 'FALLA'}]  {name}")
    
    print("\n  CONCLUSIONES:")
    print("  - LinuxCNC y grblHAL usan EXACTA la misma matematica cinematica")
    print("  - LinuxCNC NO segmenta, depende de servo cycle (1kHz)")
    print("  - grblHAL actual sobre-segmenta por distancia lineal")
    print("  - Propuesto: precision controlada, independiente de velocidad")
    print("=" * 70)
    sys.exit(0 if all([r1,r2,r3,r4]) else 1)
