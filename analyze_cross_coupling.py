"""
Analisis profundo: por que falla la formula cuando X y C cambian juntos
=======================================================================

Vamos a examinar exactamente que pasa fisicamente en un solo segmento
de X0->X100 C0->C30 para entender el error.
"""

import math

def transform_from_cartesian(tcp_x, tcp_y, tcp_z, a_deg, c_deg, pivot_z=150):
    a_rad = math.radians(a_deg)
    c_rad = math.radians(c_deg)
    cos_a, sin_a = math.cos(a_rad), math.sin(a_rad)
    cos_c, sin_c = math.cos(c_rad), math.sin(c_rad)
    px = tcp_x
    py = tcp_y
    pz = tcp_z - pivot_z
    xc = px * cos_c - py * sin_c
    yc = px * sin_c + py * cos_c
    motor_x = xc
    motor_y = yc * cos_a - pz * sin_a
    motor_z = yc * sin_a + pz * cos_a + pivot_z
    return motor_x, motor_y, motor_z


def transform_to_cartesian(motor_x, motor_y, motor_z, a_deg, c_deg, pivot_z=150):
    a_rad = math.radians(a_deg)
    c_rad = math.radians(c_deg)
    cos_a, sin_a = math.cos(a_rad), math.sin(a_rad)
    cos_c, sin_c = math.cos(c_rad), math.sin(c_rad)
    ym = motor_y
    zm = motor_z - pivot_z
    yc = ym * cos_a + zm * sin_a
    pz = -ym * sin_a + zm * cos_a
    xc = motor_x
    px = xc * cos_c + yc * sin_c
    py = -xc * sin_c + yc * cos_c
    tcp_x = px
    tcp_y = py
    tcp_z = pz + pivot_z
    return tcp_x, tcp_y, tcp_z


print("=" * 70)
print("ANALISIS: X0->X100 C0->C30 en 1 segmento")
print("=" * 70)

# Endpoints
tcp0 = (0, 0, 0)
tcp1 = (100, 0, 0)
c0, c1 = 0, 30

m0 = transform_from_cartesian(*tcp0, 0, c0)
m1 = transform_from_cartesian(*tcp1, 0, c1)

print(f"\nEndpoints:")
print(f"  TCP inicio:  ({tcp0[0]:7.2f}, {tcp0[1]:7.2f}, {tcp0[2]:7.2f})  C={c0}")
print(f"  TCP fin:     ({tcp1[0]:7.2f}, {tcp1[1]:7.2f}, {tcp1[2]:7.2f})  C={c1}")
print(f"  Motor inicio: ({m0[0]:7.2f}, {m0[1]:7.2f}, {m0[2]:7.2f})")
print(f"  Motor fin:    ({m1[0]:7.2f}, {m1[1]:7.2f}, {m1[2]:7.2f})")

print(f"\nPuntos intermedios (motor interpolado linealmente vs cinemática correcta):")
print(f"  {'t':>5s} | {'X_motor':>8s} {'Y_motor':>8s} | {'X_tcp_real':>10s} {'Y_tcp_real':>10s} | "
      f"{'X_tcp_ideal':>11s} {'Y_tcp_ideal':>11s} | {'Error':>8s}")
print(f"  {'-'*5} | {'-'*8} {'-'*8} | {'-'*10} {'-'*10} | {'-'*11} {'-'*11} | {'-'*8}")

for i in range(11):
    t = i / 10
    # Motor interpolado linealmente
    m_interp = tuple(m0[j] + t * (m1[j] - m0[j]) for j in range(3))
    # Angulo C interpolado
    c_interp = c0 + t * (c1 - c0)
    # TCP real (desde motor interpolado)
    tcp_real = transform_to_cartesian(*m_interp, 0, c_interp)
    # TCP ideal (interpolacion lineal en TCP space)
    tcp_ideal = tuple(tcp0[j] + t * (tcp1[j] - tcp0[j]) for j in range(3))
    
    err = math.sqrt(sum((tcp_real[j] - tcp_ideal[j])**2 for j in range(3)))
    
    print(f"  {t:5.2f} | {m_interp[0]:8.2f} {m_interp[1]:8.2f} | "
          f"{tcp_real[0]:10.4f} {tcp_real[1]:10.4f} | "
          f"{tcp_ideal[0]:11.4f} {tcp_ideal[1]:11.4f} | {err:8.4f}")

# Ahora analizamos POR QUE el error es tan grande
print("\n" + "=" * 70)
print("ANALISIS DE LA FUENTE DE ERROR")
print("=" * 70)

print("""
El motor_x en el destino es: tcp_x * cos(C) = 100 * cos(30) = 86.60
El motor_y en el destino es: tcp_x * sin(C) = 100 * sin(30) = 50.00

A mitad del camino (t=0.5):
  Motor interpolado: mx = (0 + 86.60)/2 = 43.30, my = (0+50)/2 = 25.00
  Angulo C = 15 grados
  
  TCP_x real = mx*cos(C) + my*sin(C) = 43.30*0.966 + 25.00*0.259 = 48.31
  TCP_y real = -mx*sin(C) + my*cos(C) = -43.30*0.259 + 25.00*0.966 = 12.94
  
  TCP ideal: x=50, y=0
  
  Error en X = 50 - 48.31 = 1.69
  Error en Y = 0 - 12.94 = -12.94  <-- !!ENORME!!
  
El problema: motor_y se interpola de 0 a 50 linealmente.
A t=0.5, motor_y = 25.
Pero ese motor_y de 25 con C=15 produce:
  tcp_y = -mx*sin(15) + my*cos(15) = -43.30*0.259 + 25*0.966 = 12.94

El tcp_y DEBERIA ser 0 en todo el camino!
El error viene de que motor_y acumula un valor (sin(30)*100 = 50)
que es proporcional a X*sin(C), y esto NO es lineal en el tiempo.
""")

print("\nEn formula: motor_y(t) = tcp_x(t) * sin(C(t)) = (100*t) * sin(30*t)")
print("Esto es un PRODUCTO de dos funciones que cambian: t * sin(t)")
print("La interpolacion lineal asume que es lineal, pero t*sin(t) no lo es.")
print()

# Cuantificar el error como funcion de la posicion
print("=" * 70)
print("CASO COMPARATIVO: C30 PURO desde X=50 FIJO (sin mover X)")
print("=" * 70)

tcp0b = (50, 0, 0)
tcp1b = (50, 0, 0)
m0b = transform_from_cartesian(*tcp0b, 0, 0)
m1b = transform_from_cartesian(*tcp1b, 0, 30)

print(f"  Motor inicio: ({m0b[0]:7.2f}, {m0b[1]:7.2f})")
print(f"  Motor fin:    ({m1b[0]:7.2f}, {m1b[1]:7.2f})")

for i in [5]:
    t = i / 10
    m_interp = tuple(m0b[j] + t * (m1b[j] - m0b[j]) for j in range(3))
    c_interp = t * 30
    tcp_real = transform_to_cartesian(*m_interp, 0, c_interp)
    tcp_ideal = (50, 0, 0)
    err = math.sqrt(sum((tcp_real[j] - tcp_ideal[j])**2 for j in range(3)))
    print(f"  t=0.5: TCP real = ({tcp_real[0]:.4f}, {tcp_real[1]:.4f}), "
          f"TCP ideal = (50, 0), err = {err:.6f}mm")
    # Chord error prediction: 50 * (1-cos(15)) = 50 * 0.03407 = 1.704
    pred = 50 * (1 - math.cos(math.radians(15)))
    print(f"  Formula cuerda: 50 * (1-cos(15)) = {pred:.6f}mm")
    print(f"  Ratio: {err/pred:.4f}x")

print("""
CONCLUSION:
  - Cuando X es FIJO y solo C rota: la formula de cuerda es EXACTA (ratio=1.0)
  - Cuando X CAMBIA mientras C rota: hay un error ADICIONAL del cross-coupling
  - El cross-coupling viene de que motor_y = X*sin(C), producto de dos variables
  - Este efecto es INDEPENDIENTE del pivot_z, es puramente de la rotacion C
  - La formula de cuerda NO captura este efecto combinado

SOLUCION PROPUESTA:
  Para el termino de C, el brazo efectivo debe incluir el DESPLAZAMIENTO
  MAXIMO del TCP desde el eje de rotacion C (eje Z).
  
  Pero incluso con el brazo correcto, la formula de cuerda pura no captura
  el cross-coupling. La solucion mas segura es:
  
  1. Usar evaluacion directa: calcular motor positions en inicio, medio y fin
  2. Si la no-linealidad del punto medio > tolerancia, subdividir
  3. Esto es un enfoque ADAPTATIVO, no analitico
""")
