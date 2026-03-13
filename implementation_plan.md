# Plan de Implementación: G68.2 / G53.1 / G69 en grblHAL

Agregar soporte para Tilted Work Plane (TWP) al firmware grblHAL del simulador 5 ejes, permitiendo que G-code generado por postprocesadores Fanuc de Fusion 360 se ejecute correctamente.

## User Review Required

> [!IMPORTANT]
> **Tu grblHAL local tiene extensiones (`ROTATION_ENABLE`) que NO existen en el upstream oficial.** El plan modifica [gcode.c](file:///c:/simulador/src/grbl/gcode.c) y [gcode.h](file:///c:/simulador/src/grbl/gcode.h) del core directamente — estos cambios no serán compatibles con actualizaciones upstream sin merge manual.

> [!NOTE]
> **✅ Sin conflicto enum**: Valores existentes de `non_modal_t`: 0,4,10,28,30,38,40,53,65,66,67,92,98,102,112,122,187. Los valores **69, 531, y 682** están libres.

> [!NOTE]
> **✅ Convención Euler confirmada**: [fanuc.cps](file:///c:/simulador/config_cam/fanuc.cps) línea 358 usa `EULER_ZXZ_R` (Q=313). Solo se implementa ZXZ. `cancelTiltFirst: true` y `eulerCalculationMethod: "standard"`.

> [!IMPORTANT]
> **Descubrimiento clave en [fanuc.cps](file:///c:/simulador/config_cam/fanuc.cps) líneas 2915-2919**: El postprocesador genera `G68.2 X_ Y_ Z_ I_ J_ K_` seguido de `G53.1`, **sin parámetros P ni Q explícitos**. Los valores I,J,K son ángulos Euler ZXZ directos. El parser debe aceptar G68.2 sin P/Q (defaults P=0, Q=313).
>
> ```javascript
> // fanuc.cps línea 2915-2919:
> writeBlock(gRotationModal.format(68.2), "X" + workOrigin.x, "Y" + workOrigin.y, "Z" + workOrigin.z,
>   "I" + abc.x, "J" + abc.y, "K" + abc.z);
> writeBlock(gFormat.format(53.1));  // activa TWP
> ```
>
> LinuxCNC `remap.py:985` ([g682()](file:///C:/Users/dise%C3%B1o/Desktop/repos/linuxcnc/configs/sim/axis/vismach/5axis/table-rotary_spindle-rotary-nutating/python/remap.py#985-1328)) confirma: P default = 0, Q default = 313.

---

## Proposed Changes

### Parser grblHAL

Modificaciones al parser de G-codes para reconocer G68.2, G53.1 y G69 como comandos non-modal válidos.

#### [MODIFY] [gcode.h](file:///c:/simulador/src/grbl/gcode.h)

Agregar 3 nuevos valores al enum `non_modal_t` (después de línea 72):
```c
NonModal_TiltedWorkPlane = 682,        // G68.2
NonModal_ActivateTWP = 531,            // G53.1
NonModal_CancelTiltedWorkPlane = 69,   // G69
```

#### [MODIFY] [gcode.c](file:///c:/simulador/src/grbl/gcode.c)

**STEP 2 (~25 líneas)** — Reconocimiento de G-codes:

1. **Agregar `case 68:`** en el switch de G-codes (~línea 1462):
   - Si `mantissa == 20` → G68.2: `non_modal_command = NonModal_TiltedWorkPlane`
   - Si `mantissa == 0` y `ROTATION_ENABLE` → dejar para G68 2D existente (si se implementa)
   - Else → error
   - **NOTA**: No confundir con M68 (`case 68:` de M-codes en línea 1706, que es I/O analógico)

2. **Agregar `case 69:`** (~después de case 68):
   - `non_modal_command = NonModal_CancelTiltedWorkPlane`
   - **Sin mantissa** — G69 es siempre entero

3. **Modificar `case 4: case 53:`** (línea 1377) para detectar G53.1:
   - Si `int_value == 53 && mantissa == 10` → G53.1
   - **CUIDADO**: El `case 4: case 53:` actual incluye fall-through de `case 10: case 28: case 30: case 92:`. La lógica de G28.1/G30.1/G92.x NO debe romperse. El check de G53.1 debe ir ANTES del fall-through existente, con `mantissa = 0` para marcar como válido.

**STEP 3 (~10 líneas)** — Validación:
- **G68.2**: Requiere I, J, K (ángulos Euler). X, Y, Z opcionales (default 0). **P y Q opcionales** (P default=0, Q no usado — ZXZ fijo)
- **G53.1**: Requiere que G68.2 haya sido ejecutado (`twp.defined == true`). Es activación de estado, no override de coordenadas (a diferencia de G53)
- **G69**: Sin validación especial. Idempotente: si TWP no está activo, G69 es NOP silencioso ([1001.nc](file:///c:/simulador/config_cam/1001.nc) línea 10 lo envía como safety)

**STEP 4 (~15 líneas)** — Ejecución:
- **G68.2**: Llama `twp_set_euler_angles(xyz, ijk)` → calcula matriz ZXZ 3x3
- **G53.1**: Llama `twp_activate()` → `twp.active = true`
- **G69**: Llama `twp_deactivate()` → `twp.active = false`, `twp.defined = false`

---

### Cinemática TWP

Lógica de rotación 3D dentro del módulo de cinemática existente.

#### [MODIFY] [rtcp.c](file:///c:/simulador/src/grbl/kinematics/rtcp.c)

**Estructura de estado TWP (~15 líneas)**:
- `twp_state_t` con: `active`, `defined`, `origin[3]`, `euler[3]`, `R[3][3]`, `R_inv[3][3]`

**Función `twp_set_euler_angles()` (~40 líneas)**:
- Recibe origen (X,Y,Z) y ángulos Euler (α1,α2,α3) en grados
- Convierte a radianes con `RADDEG` (constante ya existente en `nuts_bolts.h:45`)
- Calcula matriz ZXZ (Q=313): `R = Rz(α1) × Rx(α2) × Rz(α3)`
- Calcula inversa = transpuesta (R es ortogonal)
- Setea `twp.defined = true`

**Modificar [rtcp_segment_line()](file:///c:/simulador/src/grbl/kinematics/rtcp.c#1114-1399) (~15 líneas)**:
- Si `twp.active`: aplicar `rotated = R × (target - origin) + origin` solo a XYZ
- Ejes A,B,C pasan sin modificar
- **Punto de inserción**: en `init=true`, línea ~1140, ANTES del bypass `if (!rtcp_enabled)` (línea 1142). TWP funciona con RTCP OFF — `1001.nc` usa G68.2/G53.1 con RTCP desactivado
- Continuar con IK normal si RTCP está habilitado

**Modificar [transform_steps_to_cartesian()](file:///c:/simulador/src/grbl/kinematics/rtcp.c#729-769) (~10 líneas)**:
- Si `twp.active`: aplicar rotación inversa `R_inv` para DRO correcto

**Funciones `twp_activate()` / `twp_deactivate()` (~10 líneas)**:
- Activar/desactivar flag, sincronizar planner con `protocol_buffer_synchronize()`
- **`twp_deactivate` debe resetear ambos**: `active = false` y `defined = false` (como G69)

**Reset TWP**:
- TWP solo se cancela con G69 explícito — NO se resetea automáticamente en soft reset ni abort
- Consistente con Marlin (DerAndere1) y LinuxCNC donde el estado TWP persiste para permitir retomar mecanizado

#### [NEW] [twp.h](file:///c:/simulador/src/grbl/kinematics/twp.h)

Header con prototipos para que [gcode.c](file:///c:/simulador/src/grbl/gcode.c) pueda llamar funciones de rtcp.c:
```c
void twp_set_euler_angles(float ox, float oy, float oz, float a1, float a2, float a3);
void twp_activate(void);
void twp_deactivate(void);
bool twp_is_active(void);
bool twp_is_defined(void);
```

---

## Notas de Consistencia (Manual vs Plan)

| Tema | Manual | Plan | Estado |
|---|---|---|---|
| Enum G69 | Sección 16: valor 69 | ✅ 69 (verificado libre) | OK |
| Euler convention | Sección 13.3: ZXZ Q=313 | ✅ Solo ZXZ | OK |
| TLO handling | Sección 14.5: TLO ya en target | ✅ No se maneja TLO separado | OK |
| TWP antes de RTCP | Sección 16.6: orden correcto | ✅ Secuenciales, no simultáneos | OK |
| G68 2D existente | Sección 15.2: ROTATION_ENABLE | ✅ case 68 protege G68 con `#ifdef` | OK |
| Modos P0-P3 | Sección 3: 4 modos | Solo P0 (Fase 1) | OK — Fase 2 futura |
| Parámetro R (pre-rotación) | Sección 2.5 | No implementado Fase 1 | OK — Fase 2 futura |
| Reset en homing | Sección 8.4 | ✅ Incluido | OK |
| Estimación líneas | Sección 12: 450-500 total | 250 Fase 1 | OK — consistente |
| case 53 fall-through | Sección 14.3 | ✅ Nota sobre no romper G28/G30/G92 | OK |

---

## Verification Plan

### Automated Tests

1. **Compilación**: Build del proyecto — sin errores ni warnings
2. **Test de parsing**:
   - `G68.2 X0 Y0 Z0 I0 J-90 K0` → OK
   - `G68.3` → error (`Status_GcodeUnsupportedCommand`)
   - `G53.1` después de G68.2 → OK
   - `G53.1` sin G68.2 previo → error
   - `G69` → OK
   - `G28 G91 Z0` después de G69 → OK (no se rompió G28)
3. **Test de rotación matemática**:
   - `G68.2 I0 J0 K0` → matriz identidad, sin cambio
   - `G68.2 I0 J0 K-21.045` → solo Rz(-21.045°), coincide con [1001.nc](file:///c:/simulador/config_cam/1001.nc)
   - `G68.2 I0 J90 K0` → rotación X 90°, Y→Z

### Manual Verification

1. Ejecutar secuencia de [1001.nc](file:///c:/simulador/config_cam/1001.nc): G69→G68.2→G53.1→G00→G69→M451→corte→M450
2. Verificar DRO con TWP activo muestra coordenadas en plano inclinado
3. Verificar que M451 (RTCP) sigue funcionando después de G69
4. Verificar que G28 cancela TWP automáticamente
