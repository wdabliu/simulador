/*
  carteciano2.c - cinemática cartesiana explícita extraída del comportamiento
                  por defecto implícito de grblHAL

  Cuando grblHAL no tiene KINEMATICS_API definido, las funciones cinemáticas
  están dispersas en:
    - system.c:          system_convert_array_steps_to_mpos()  (steps → mm)
    - machine_limits.c:  limits_set_machine_positions()        (posiciones homing)
    - machine_limits.c:  step_pin[idx] = bit(idx)              (máscara de ejes)
    - machine_limits.c:  sys.position[idx] = 0                 (target pos homing)
    - motion_control.c:  sin KINEMATICS_API, mc_line() pasa directo al planner

  Este archivo reúne todo ese comportamiento implícito como un módulo cinemático
  explícito que se conecta a la API kinematics_t de grblHAL.

  Part of grblHAL simulator

  Copyright (c) 2019-2023 Terje Io (código original implícito)

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
*/

#include "../grbl.h"

#ifndef CARTESIAN2
#define CARTESIAN2 1
#endif

#if CARTESIAN2

#include <math.h>

#include "../hal.h"
#include "../settings.h"
#include "../planner.h"
#include "../kinematics.h"

static on_report_options_ptr on_report_options;

/* ============================================================================
 * transform_steps_to_cartesian  (extraída de system.c líneas 1216-1220)
 *
 * Convierte el array de pasos del motor a posición en milímetros.
 * En cartesiano puro es una simple división: mm = steps / steps_per_mm
 * ========================================================================= */
static float *cartesian2_transform_steps_to_cartesian (float *position, int32_t *steps)
{
    uint_fast8_t idx = N_AXIS;

    do {
        idx--;
        position[idx] = steps[idx] / settings.axis[idx].steps_per_mm;
    } while(idx);

    return position;
}

/* ============================================================================
 * transform_from_cartesian  (implícita — identidad)
 *
 * Transforma coordenadas del sistema cartesiano de trabajo al sistema motor.
 * En cinemática cartesiana es la función identidad: motor == cartesiano.
 *
 * Usada por machine_limits.c en el homing para transformar el target antes
 * de enviarlo al planner (líneas 209, 383).
 * ========================================================================= */
static float *cartesian2_transform_from_cartesian (float *target, float *position)
{
    uint_fast8_t idx = N_AXIS;

    do {
        idx--;
        target[idx] = position[idx];
    } while(idx);

    return target;
}

/* ============================================================================
 * segment_line  (extraída de motion_control.c líneas 84-201)
 *
 * En modo cartesiano sin KINEMATICS_API, mc_line() pasa el target directamente
 * al planner sin subdivisiones. Cuando se usa KINEMATICS_API, segment_line
 * se llama en un bucle while hasta que retorne NULL.
 *
 * Implementación: 2 iteraciones (init + 1 paso), sin subdivisiones.
 * La primera llamada (init=true) copia el target y retorna el transformado.
 * La segunda llamada retorna el target final.
 * La tercera retorna NULL para terminar el bucle.
 * ========================================================================= */
static float *cartesian2_segment_line (float *target, float *position, plan_line_data_t *pl_data, bool init)
{
    static uint_fast8_t iterations;
    static float trsf[N_AXIS];

    if(init) {
        iterations = 2;
        /* Transformación identidad: copiar target tal cual */
        cartesian2_transform_from_cartesian(trsf, target);
        /* En cartesiano puro no hay escalado de feed rate */
    }

    return iterations-- == 0 ? NULL : trsf;
}

/* ============================================================================
 * limits_get_axis_mask  (extraída de machine_limits.c línea 302)
 *
 * Retorna la máscara de bits para el eje dado.
 * En cartesiano es mapeo 1:1: eje 0 → bit 0, eje 1 → bit 1, etc.
 * ========================================================================= */
static uint_fast8_t cartesian2_limits_get_axis_mask (uint_fast8_t idx)
{
    return bit(idx);
}

/* ============================================================================
 * limits_set_target_pos  (extraída de machine_limits.c línea 354)
 *
 * Restablece la posición del eje a 0 durante el homing.
 * En cartesiano simplemente pone sys.position[idx] = 0.
 * ========================================================================= */
static void cartesian2_limits_set_target_pos (uint_fast8_t idx)
{
    sys.position[idx] = 0;
}

/* ============================================================================
 * limits_set_machine_positions  (extraída de machine_limits.c líneas 146-165)
 *
 * Establece las posiciones de máquina después de completar el homing.
 * Si force_set_origin está activo, pone la posición en 0.
 * Si no, calcula la posición basándose en max_travel y pulloff.
 * ========================================================================= */
static void cartesian2_limits_set_machine_positions (axes_signals_t cycle)
{
    uint_fast8_t idx = N_AXIS;

    if(settings.homing.flags.force_set_origin) {
        do {
            if(cycle.mask & bit(--idx)) {
                sys.position[idx] = 0;
            }
        } while(idx);
    } else do {

        coord_data_t *pulloff = limits_homing_pulloff(NULL);

        if(cycle.mask & bit(--idx)) {
            sys.position[idx] = bit_istrue(settings.homing.dir_mask.value, bit(idx))
                                     ? lroundf((settings.axis[idx].max_travel + pulloff->values[idx]) * settings.axis[idx].steps_per_mm)
                                     : lroundf(-pulloff->values[idx] * settings.axis[idx].steps_per_mm);
        }
    } while(idx);
}

/* ============================================================================
 * homing_cycle_validate  (implícita — siempre válido)
 *
 * En mode por defecto, grblHAL comprueba si kinematics.homing_cycle_validate
 * es != NULL antes de llamarla. Si es NULL, el ciclo siempre es válido.
 * Aquí la exponemos explícitamente retornando true siempre.
 * ========================================================================= */
static bool cartesian2_homing_cycle_validate (axes_signals_t cycle)
{
    return true;
}

/* ============================================================================
 * homing_cycle_get_feedrate  (implícita — sin modificación)
 *
 * En modo por defecto, grblHAL comprueba si kinematics.homing_cycle_get_feedrate
 * es != NULL. Si es NULL, usa el feedrate sin modificar.
 * Aquí exponemos la versión identidad: retorna el feedrate tal cual.
 * ========================================================================= */
static float cartesian2_homing_cycle_get_feedrate (axes_signals_t cycle, float feedrate, homing_mode_t mode)
{
    return feedrate;
}

/* ============================================================================
 * report_options  (reporta la cinemática cargada)
 * ========================================================================= */
static void cartesian2_report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[KINEMATICS:Cartesiana2 (por defecto explícita)]" ASCII_EOL);
}

/* ============================================================================
 * cartesian2_init  —  Punto de entrada del módulo
 *
 * Conecta todas las funciones cinemáticas cartesianas explícitas a la
 * estructura kinematics_t de grblHAL.
 *
 * Equivale a hacer explícito todo lo que grblHAL hace implícitamente
 * cuando KINEMATICS_API no está definido.
 * ========================================================================= */
void cartesian2_init (void)
{
    kinematics.transform_steps_to_cartesian = cartesian2_transform_steps_to_cartesian;
    kinematics.transform_from_cartesian     = cartesian2_transform_from_cartesian;
    kinematics.segment_line                 = cartesian2_segment_line;
    kinematics.limits_get_axis_mask         = cartesian2_limits_get_axis_mask;
    kinematics.limits_set_target_pos        = cartesian2_limits_set_target_pos;
    kinematics.limits_set_machine_positions = cartesian2_limits_set_machine_positions;
    kinematics.homing_cycle_validate        = cartesian2_homing_cycle_validate;
    kinematics.homing_cycle_get_feedrate    = cartesian2_homing_cycle_get_feedrate;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = cartesian2_report_options;
}

#endif

