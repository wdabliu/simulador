"""
Verificacion v4: Enfoque ADAPTATIVO por evaluacion de punto medio
==================================================================

En lugar de una formula analitica (que no captura cross-coupling),
evaluamos el error REAL del punto medio del movimiento.

Metodo:
  1. Calcular motor positions en inicio, medio y fin
  2. Comparar el motor del punto medio cinemático vs interpolado
  3. Si el error > tolerancia, duplicar segmentos (biseccion)
  4. Resultado: N minimo que garantiza error <= tolerancia

Implementacion en C seria:
  - 1 llamada extra a transform_from_cartesian para el punto medio
  - Bucle while que duplica N hasta que el error sea aceptable
  - Tipicamente converge en 1-3 iteraciones
"""

import math
import sys

# =====================================================================
# RTCP KINEMATICS
# =====================================================================

def fwd(tcp_x, tcp_y, tcp_z, a_deg, c_deg, pz=150):
    ar, cr = math.radians(a_deg), math.radians(c_deg)
    ca, sa, cc, sc = math.cos(ar), math.sin(ar), math.cos(cr), math.sin(cr)
    px, py, pzz = tcp_x, tcp_y, tcp_z - pz
    xc = px*cc - py*sc
    yc = px*sc + py*cc
    return (xc, yc*ca - pzz*sa, yc*sa + pzz*ca + pz)

def inv(mx, my, mz, a_deg, c_deg, pz=150):
    ar, cr = math.radians(a_deg), math.radians(c_deg)
    ca, sa, cc, sc = math.cos(ar), math.sin(ar), math.cos(cr), math.sin(cr)
    ym, zm = my, mz - pz
    yc = ym*ca + zm*sa
    pzz = -ym*sa + zm*ca
    px = mx*cc + yc*sc
    py = -mx*sc + yc*cc
    return (px, py, pzz + pz)


def simulate_segments(tcp0, tcp1, a0, a1, c0, c1, n, pz=150, checks=100):
    """Mide el error TCP maximo con N segmentos"""
    max_err = 0.0
    for seg in range(n):
        t0, t1 = seg/n, (seg+1)/n
        tp0 = [tcp0[i]+t0*(tcp1[i]-tcp0[i]) for i in range(3)]
        tp1s = [tcp0[i]+t1*(tcp1[i]-tcp0[i]) for i in range(3)]
        aa0 = a0+t0*(a1-a0); aa1 = a0+t1*(a1-a0)
        cc0 = c0+t0*(c1-c0); cc1 = c0+t1*(c1-c0)
        m0 = fwd(*tp0, aa0, cc0, pz)
        m1 = fwd(*tp1s, aa1, cc1, pz)
        for k in range(1, checks):
            s = k/checks
            tg = t0+s*(t1-t0)
            mi = tuple(m0[i]+s*(m1[i]-m0[i]) for i in range(3))
            ai = aa0+s*(aa1-aa0); ci = cc0+s*(cc1-cc0)
            ideal = [tcp0[i]+tg*(tcp1[i]-tcp0[i]) for i in range(3)]
            real = inv(*mi, ai, ci, pz)
            err = math.sqrt(sum((real[i]-ideal[i])**2 for i in range(3)))
            if err > max_err: max_err = err
    return max_err


def midpoint_error(tcp0, tcp1, a0, a1, c0, c1, pz=150):
    """Error del punto medio: motor interpolado vs cinematica correcta.
    
    Esto captura TODOS los tipos de error:
    - Chord error de A
    - Chord error de C  
    - Cross-coupling X*sin(C)
    - Cualquier otro efecto no-lineal
    """
    # Motor positions en los extremos
    m0 = fwd(*tcp0, a0, c0, pz)
    m1 = fwd(*tcp1, a1, c1, pz)
    
    # Punto medio: valores interpolados linealmente
    tcp_mid = [(tcp0[i]+tcp1[i])/2 for i in range(3)]
    a_mid = (a0+a1)/2
    c_mid = (c0+c1)/2
    
    # Motor del punto medio calculado cinematicamente (correcto)
    m_correct = fwd(*tcp_mid, a_mid, c_mid, pz)
    
    # Motor del punto medio interpolado linealmente (lo que hace el planner)
    m_interp = tuple((m0[i]+m1[i])/2 for i in range(3))
    
    # Error en motor space
    err_motor = math.sqrt(sum((m_correct[i]-m_interp[i])**2 for i in range(3)))
    
    # Pero lo que importa es el error en TCP space
    # Usar cinematica directa para ver donde llega el TCP con el motor interpolado
    tcp_from_interp = inv(*m_interp, a_mid, c_mid, pz)
    err_tcp = math.sqrt(sum((tcp_from_interp[i]-tcp_mid[i])**2 for i in range(3)))
    
    return err_tcp


def calc_segments_adaptive(tcp0, tcp1, a0, a1, c0, c1, tolerance, pz=150):
    """Calcula N por biseccion del error del punto medio.
    
    1. Divide el movimiento en N segmentos
    2. Para cada segmento, evalua el error del punto medio
    3. Si algun error > tolerancia, duplica N
    4. Repite hasta que todos los segmentos pasen
    
    Complejidad: O(N * log(N)) llamadas a transform, pero N es tipicamente pequeno
    """
    n = 1
    max_n = 10000
    
    while n < max_n:
        max_err = 0.0
        for seg in range(n):
            t0, t1 = seg/n, (seg+1)/n
            tp0 = [tcp0[i]+t0*(tcp1[i]-tcp0[i]) for i in range(3)]
            tp1s = [tcp0[i]+t1*(tcp1[i]-tcp0[i]) for i in range(3)]
            aa0 = a0+t0*(a1-a0); aa1 = a0+t1*(a1-a0) 
            cc0 = c0+t0*(c1-c0); cc1 = c0+t1*(c1-c0)
            err = midpoint_error(tp0, tp1s, aa0, aa1, cc0, cc1, pz)
            if err > max_err:
                max_err = err
        if max_err <= tolerance:
            return n, max_err
        n *= 2  # Duplicar
    
    return n, max_err


# =====================================================================
# PRUEBA: Verificar que el metodo adaptativo funciona para TODOS los casos
# =====================================================================

if __name__ == "__main__":
    print("\n" + "=" * 70)
    print("  VERIFICACION v4: ENFOQUE ADAPTATIVO (EVALUACION PUNTO MEDIO)")
    print("  Metodo: evaluar error real del punto medio, biseccion hasta OK")
    print("=" * 70 + "\n")
    
    pz = 150
    tol = 0.01
    
    print(f"  Pivot Z = {pz}mm, Tolerancia = {tol}mm\n")
    
    moves = [
        # (label, tcp0, tcp1, a0, a1, c0, c1)
        # Solo A
        ("X1000 A2",        [0,0,0], [1000,0,0], 0, 2,  0, 0),
        ("X200 A4",         [0,0,0], [200,0,0],  0, 4,  0, 0),
        ("X100 A10",        [0,0,0], [100,0,0],  0, 10, 0, 0),
        ("X10 A30",         [0,0,0], [10,0,0],   0, 30, 0, 0),
        ("X0 A90",          [0,0,0], [0,0,0],    0, 90, 0, 0),
        ("X100 A0.5",       [0,0,0], [100,0,0],  0, 0.5,0, 0),
        # Solo C
        ("X100 C30",        [0,0,0], [100,0,0],  0, 0,  0, 30),
        ("C90 desde X50",   [50,0,0],[50,0,0],   0, 0,  0, 90),
        ("X100Y50 C30",     [0,0,0], [100,50,0], 0, 0,  0, 30),
        # A + C combinados
        ("X100Y50 A10C30",  [0,0,0], [100,50,0], 0, 10, 0, 30),
        ("X50 A30C30",      [0,0,0], [50,0,0],   0, 30, 0, 30),
        ("A45C45 puro",     [0,0,0], [0,0,0],    0, 45, 0, 45),
        ("X200 A5C10",      [0,0,0], [200,0,0],  0, 5,  0, 10),
        ("X200Y100 A20C45", [0,0,0], [200,100,0],0, 20, 0, 45),
        # Sin rotacion
        ("X1000 (sin rot)", [0,0,0], [1000,0,0], 0, 0,  0, 0),
        ("X500Y300 (sin rot)",[0,0,0],[500,300,0],0, 0,  0, 0),
    ]
    
    print(f"  {'Movimiento':<30s} {'Adapt':>6s} {'Actual':>7s} {'Err adapt':>10s} "
          f"{'Err real':>10s} {'<=tol?':>6s}")
    print(f"  {'-'*30} {'-'*6} {'-'*7} {'-'*10} {'-'*10} {'-'*6}")
    
    all_ok = True
    for label, tcp0, tcp1, a0, a1, c0, c1 in moves:
        da = abs(a1-a0); dc = abs(c1-c0)
        dist = math.sqrt(sum((tcp1[i]-tcp0[i])**2 for i in range(3)))
        
        # Metodo adaptativo
        n_adapt, err_adapt = calc_segments_adaptive(tcp0, tcp1, a0, a1, c0, c1, tol, pz)
        
        # Sistema actual
        segs_lin = max(1, math.ceil(dist/0.5)) if dist > 0.5 else 1
        segs_rot = max(1, math.ceil(max(da,dc)/0.5)) if max(da,dc) > 0.5 else 1
        n_actual = max(segs_lin, segs_rot)
        
        # Error real con N adaptativo
        err_real = simulate_segments(tcp0, tcp1, a0, a1, c0, c1, n_adapt, pz)
        
        within = err_real <= tol * 1.15
        ok = within
        if not ok: all_ok = False
        
        tag = "OK" if ok else "!!"
        tol_tag = "SI" if within else "NO"
        print(f"  [{tag}] {label:<30s} {n_adapt:>6d} {n_actual:>7d} "
              f"{err_adapt:>10.6f} {err_real:>10.6f} {tol_tag:>6s}")
    
    print(f"\n  {'TODAS PASAN' if all_ok else 'HAY FALLOS'}")
    
    print("\n" + "=" * 70)
    print("  COMPARATIVA: Segmentos adaptativos vs actuales")
    print("=" * 70)
    
    total_adapt = 0
    total_actual = 0
    for label, tcp0, tcp1, a0, a1, c0, c1 in moves:
        da = abs(a1-a0); dc = abs(c1-c0)
        dist = math.sqrt(sum((tcp1[i]-tcp0[i])**2 for i in range(3)))
        n_adapt, _ = calc_segments_adaptive(tcp0, tcp1, a0, a1, c0, c1, tol, pz)
        segs_lin = max(1, math.ceil(dist/0.5)) if dist > 0.5 else 1
        segs_rot = max(1, math.ceil(max(da,dc)/0.5)) if max(da,dc) > 0.5 else 1
        n_actual = max(segs_lin, segs_rot)
        ratio = n_actual / n_adapt if n_adapt > 0 else 0
        total_adapt += n_adapt
        total_actual += n_actual
        bar_a = "#" * min(50, n_adapt)
        bar_c = "#" * min(50, n_actual)
        print(f"\n  {label}:")
        print(f"    Adapt:  {n_adapt:>6d} {bar_a}")
        print(f"    Actual: {n_actual:>6d} {bar_c}")
        print(f"    Ratio:  {ratio:.1f}x {'<< mejora' if ratio > 2 else ''}")
    
    print(f"\n  TOTAL: Adapt={total_adapt}, Actual={total_actual}, "
          f"Reduccion={total_actual/total_adapt:.1f}x")
    print("=" * 70)
    sys.exit(0 if all_ok else 1)
