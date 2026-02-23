#!/usr/bin/env python3
"""
vismach_5axis.py - Modelo Trunnion AC para grblHAL simulator
Basado en xyzac-trt-gui.py de LinuxCNC (Rudy du Preez)
Configuracion: Cabezal fijo + mesa basculante A sobre plato C
"""
from vismach import *
import hal
import sys

# ===================== HAL COMPONENT =====================

c = hal.component("grbl_visualizer")
c.newpin("axis_x", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("axis_y", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("axis_z", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("axis_a", hal.HAL_FLOAT, hal.HAL_IN)
c.newpin("axis_c", hal.HAL_FLOAT, hal.HAL_IN)
c.ready()

# ===================== HERRAMIENTA =====================

tooltip = Capture()

tool = Collection([
    tooltip,
    CylinderZ(0, 0.2, 6, 3),   # punta
    CylinderZ(6, 3, 70, 3)     # cuerpo
])
tool = Translate([tool], 0, 0, -20)
tool = Color([1, 0, 0, 0], [tool])

# ===================== SPINDLE =====================

spindle = Collection([
    Color([0, 0.5, 0.5, 0], [CylinderZ(0, 10, 20, 15)]),   # nariz
    CylinderZ(20, 20, 135, 20),                               # carcasa
])
spindle = Color([0, 0.5, 0.5, 0], [spindle])
spindle = Collection([tool, spindle])
spindle = Translate([spindle], 0, 0, 20)

# Motor del spindle
motor = Collection([
    Color([0, 0.5, 0.5, 0], [CylinderZ(135, 30, 200, 30)])
])
motor = Translate([motor], 0, 200, 0)

# Cabezal completo
head = Collection([
    spindle,
    Color([0, 1, 0, 0], [Box(-30, -30, 60, 30, 240, 135)]),
    motor
])
head = Translate([head], 0, 0, 205)

# ===================== MESA ROTATIVA C =====================

work = Capture()

ctable = Collection([
    work,
    CylinderZ(-9, 50, 0, 50),
    Color([1, 1, 1, 0], [CylinderX(-50, 1, 50, 1)]),  # cruz X
    Color([1, 1, 1, 0], [CylinderY(-50, 1, 50, 1)]),  # cruz Y
    Color([1, 1, 1, 0], [Box(42, -4, -20, 51, 4, 5)]) # marca orientacion
])
ctable = HalRotate([ctable], c, "axis_c", 1, 0, 0, 1)
ctable = Color([1, 0, 1, 0], [ctable])

crotary = Collection([
    ctable,
    Color([0.3, 0.5, 1, 0], [Box(-50, -50, -30, 50, 50, -18)])
])

# ===================== TRUNNION (EJE A) =====================

# Yugo (placa base + laterales)
yoke = Collection([
    Color([1, 0.5, 0, 0], [Box(-65, -40, -35, 65, 40, -25)]),  # placa
    Color([1, 0.5, 0, 0], [Box(-65, -40, -35, -55, 40, 0)]),   # lateral izq
    Color([1, 0.5, 0, 0], [Box(55, -40, -35, 65, 40, 0)])      # lateral der
])

# Cilindros de soporte del trunnion
trunnion = Collection([
    Color([1, 0.5, 0, 0], [CylinderX(-78, 20, -55, 20)]),
    Color([1, 0.5, 0, 0], [CylinderX(55, 15, 70, 15)]),
    Color([1, 1, 1, 0], [Box(-80, -20, -1, -78, 20, 1)])  # marca lado motor
])

# Ensamble rotativo A
arotary = Collection([crotary, yoke, trunnion])
arotary = HalRotate([arotary], c, "axis_a", 1, 1, 0, 0)

# Brackets de montaje
brackets = Collection([
    Box(-77, -40, -50, -67, 40, 0),   # bracket izq
    Box(77, -40, -50, 67, 40, 0),     # bracket der
    Box(77, 40, -52, -77, -40, -40)   # placa de montaje
])

# ===================== MESA PRINCIPAL (solo X) =====================

table = Collection([
    arotary,
    brackets,
    Box(-150, -50, -69, 150, 50, -52),   # cuerpo de mesa
    Box(-150, -40, -75, 150, 40, -69)    # guias
])
table = HalTranslate([table], c, "axis_x", -1, 0, 0)
table = Color([0.4, 0.4, 0.4, 0], [table])

# ===================== SILLA (solo Y) =====================

saddle = Collection([
    table,
    Box(-75, -53, -105, 75, 53, -73),      # cuerpo silla
    Box(-60, -60, -200, 60, 60, -105),     # soporte vertical hasta la base
])
saddle = HalTranslate([saddle], c, "axis_y", 0, -1, 0)
saddle = Color([0.8, 0.8, 0.8, 0], [saddle])
saddle = Translate([saddle], 0, 0, 200)    # elevar mesa sobre la base

# ===================== CARRO Z + CABEZAL =====================
# El spindle se mueve en Z deslizandose sobre la columna

zcarriage = Color([1, 1, 0, 0], [Box(-45, 100, 250, 45, 195, 370)])

head_z = Collection([head, zcarriage])
head_z = HalTranslate([head_z], c, "axis_z", 0, 0, 1)

# ===================== BASE Y COLUMNA =====================

base = Collection([
    Box(-150, -120, -80, 150, 200, 0),       # base (ancha y plana)
    Box(-55, 100, 0, 55, 200, 450),          # columna (alta)
])
base = Color([0, 1, 0, 0], [base])

# ===================== MODELO FINAL =====================

model = Collection([head_z, saddle, base])

myhud = Hud()
myhud.show("grblHAL 5-Axis AC Trunnion")

main(model, tooltip, work, size=500, hud=myhud, lat=-60, lon=25)


