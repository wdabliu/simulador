/*
  carteciana.c - explicit cartesian kinematics implementation

  Part of grblHAL

  This file implements the standard Cartesian kinematics explicity
  through the kinematics API, although grblHAL defaults to this
  behavior natively if no kinematic module is loaded.
*/

#include "../grbl.h"

// Define un flag para habilitar esta cinemática. 
// Normalmente grblHAL es cartesiano por defecto.
#ifndef CARTESIAN
#define CARTESIAN 1
#endif

#if CARTESIAN

#include <math.h>

#include "../hal.h"
#include "../settings.h"
#include "../planner.h"
#include "../kinematics.h"

static on_report_options_ptr on_report_options;

// Convierte array de "steps" a posición de máquina (mm)
static float *cartesian_convert_array_steps_to_mpos (float *position, int32_t *steps)
{
    uint_fast8_t idx;

    for(idx = X_AXIS; idx < N_AXIS; idx++)
        position[idx] = steps[idx] / settings.axis[idx].steps_per_mm;

    return position;
}

// Transforma posición del sistema Cartesiano a Cartesiano (Identidad, 1:1)
static inline float *transform_from_cartesian (float *target, float *position)
{
    uint_fast8_t idx;

    for(idx = X_AXIS; idx < N_AXIS; idx++)
        target[idx] = position[idx];

    return target;
}

// Transforma posición del sistema motor a Cartesiano (Identidad, 1:1)
static inline float *transform_to_cartesian (float *target, float *position)
{
    uint_fast8_t idx;

    for(idx = X_AXIS; idx < N_AXIS; idx++)
        target[idx] = position[idx];

    return target;
}

// Obtiene la máscara del eje para límites
static uint_fast8_t cartesian_limits_get_axis_mask (uint_fast8_t idx)
{
    return bit(idx); // Mapeo 1:1
}

// Establece la posición objetivo
static void cartesian_limits_set_target_pos (uint_fast8_t idx)
{
    sys.position[idx] = 0;
}

// Establece las posiciones de máquina para los finales de carrera después de hacer homing.
static void cartesian_limits_set_machine_positions (axes_signals_t cycle)
{
    uint_fast8_t idx = N_AXIS;

    if(settings.homing.flags.force_set_origin) {
        do {
            if(cycle.mask & bit(--idx)) {
                sys.position[idx] = 0;
            }
        } while (idx);
    } else do {

         coord_data_t *pulloff = limits_homing_pulloff(NULL);

         if(cycle.mask & bit(--idx)) {
             sys.position[idx] = bit_istrue(settings.homing.dir_mask.value, bit(idx))
                                          ? lroundf((settings.axis[idx].max_travel + pulloff->values[idx]) * settings.axis[idx].steps_per_mm)
                                          : lroundf(-pulloff->values[idx] * settings.axis[idx].steps_per_mm);
         }
    } while(idx);
}

// Segmentación en líneas rectas, no hace falta subdividir un movimiento directo cartesiano
static float *kinematics_segment_line (float *target, float *position, plan_line_data_t *pl_data, bool init)
{
    static uint_fast8_t iterations;
    static float trsf[N_AXIS];

    if(init) {
        iterations = 2; // Solo el inicio y fin, sin subdivisiones complejas
        transform_from_cartesian(trsf, target);
        // Para cartesianas puras, la velocidad de avance (feedrate) es lineal, sin escala no-lineal
    }

    return iterations-- == 0 ? NULL : trsf;
}

// Validación de ciclo de homing
static bool homing_cycle_validate (axes_signals_t cycle)
{
    return true; // Cualquier ciclo es válido en cinemática pura cartesiana
}

// Obtener velocidad de homing (sin multiplicador en ejes ortogonales)
static float homing_cycle_get_feedrate (axes_signals_t cycle, float feedrate, homing_mode_t mode)
{
    return feedrate;
}

// Reportar la óptica cargada
static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[KINEMATICS:Cartesiana Identidad]" ASCII_EOL);
}

// Inicializar API (pointers a las funciones de cinemática) de la Cartesiana Clásica
void cartesian_init (void)
{
    kinematics.limits_set_target_pos = cartesian_limits_set_target_pos;
    kinematics.limits_get_axis_mask = cartesian_limits_get_axis_mask;
    kinematics.limits_set_machine_positions = cartesian_limits_set_machine_positions;
    kinematics.transform_from_cartesian = transform_from_cartesian;
    kinematics.transform_steps_to_cartesian = cartesian_convert_array_steps_to_mpos;
    kinematics.segment_line = kinematics_segment_line;
    kinematics.homing_cycle_validate = homing_cycle_validate;
    kinematics.homing_cycle_get_feedrate = homing_cycle_get_feedrate;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;
}

#endif
