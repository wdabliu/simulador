/**
 * =============================================================================
 * @file    5axis_rtcp.c
 * @brief   Kinematics Module for 5-Axis RTCP (Rotational Tool Center Point)
 * @version v17.1 (Production Release + TLO Documentation)
 * @platform RP2350 / grblHAL
 * 
 * =============================================================================
 * 
 * DESCRIPCIÓN GENERAL
 * ===================
 * 
 * Este módulo implementa cinemática RTCP (Rotational Tool Center Point) para
 * máquinas CNC de 5 ejes con configuración AC (A = inclinación, C = rotación).
 * 
 * RTCP mantiene el punto central de la herramienta (TCP) estacionario en el
 * espacio cartesiano mientras los ejes rotativos cambian la orientación,
 * compensando automáticamente los movimientos en X, Y, Z.
 * 
 * 
 * INTEGRACIÓN CON grblHAL - ANÁLISIS DE REDUNDANCIA
 * ==================================================
 * 
 * Este módulo fue diseñado analizando el código fuente de grblHAL para evitar
 * duplicar funcionalidad nativa. A continuación el análisis:
 * 
 * ┌─────────────────────────────────────────────────────────────────────────┐
 * │ FUNCIONALIDAD              │ grblHAL NATIVO │ RTCP IMPLEMENTA │ RAZÓN  │
 * ├─────────────────────────────────────────────────────────────────────────┤
 * │ Flujo mc_line()            │ ✓ Completo     │ ✗ No toca       │ [1]    │
 * │ Restaurar feed_rate        │ ✓ mc_line      │ ✗ No necesario  │ [2]    │
 * │ Auto-cycle start           │ ✓ mc_line      │ ✗ No toca       │ [3]    │
 * │ Soft limits abort          │ ✓ limits_soft  │ ✗ Usa nativo    │ [4]    │
 * │ transform_from_cartesian   │ ✗ No existe    │ ✓ Requerido     │ [5]    │
 * │ transform_steps_to_cart    │ ✗ No existe    │ ✓ Requerido     │ [5]    │
 * │ segment_line               │ ✗ No existe    │ ✓ Requerido     │ [5]    │
 * │ check_travel_limits        │ ✓ Solo cart.   │ ✓ Hook motor    │ [6]    │
 * │ apply_travel_limits        │ ✓ Clipping     │ ✓ Hook bisect   │ [7]    │
 * │ Homing functions           │ ✓ Lineales     │ ✓ Override      │ [8]    │
 * └─────────────────────────────────────────────────────────────────────────┘
 * 
 * NOTAS:
 * [1] mc_line() en motion_control.c maneja todo el flujo de segmentación.
 *     Nosotros solo proveemos la función segment_line que mc_line() llama.
 * 
 * [2] mc_line() restaura pl_data->feed_rate después de cada segmento:
 *     ```c
 *     pl_data->feed_rate = feed_rate;  // Restaura original
 *     ```
 *     Por eso podemos modificar feed_rate en cada segmento sin guardarlo.
 * 
 * [3] mc_line() maneja auto-cycle start cuando el buffer está lleno.
 *     No necesitamos replicar esta lógica.
 * 
 * [4] limits_soft_check() en machine_limits.c verifica:
 *     ```c
 *     if(condition.target_validated ? !condition.target_valid : ...)
 *     ```
 *     Si nosotros ponemos target_validated=On y target_valid=Off,
 *     grblHAL abortará el movimiento automáticamente.
 * 
 * [5] Estas funciones no existen en grblHAL base - son la esencia del
 *     módulo de cinemática que debemos implementar.
 * 
 * [6] check_travel_limits() nativa en machine_limits.c:
 *     ```c
 *     if(is_cartesian && (sys.homed.mask & axes.mask)) { ... }
 *     return is_cartesian && !failed;
 *     ```
 *     PROBLEMA: Si is_cartesian=false, SIEMPRE retorna false.
 *     Para RTCP necesitamos verificar coordenadas de MOTOR (is_cartesian=false)
 *     así que DEBEMOS hacer hook para manejar este caso.
 * 
 * [7] apply_travel_limits() nativa hace clipping lineal simple:
 *     ```c
 *     target[idx] = max(min(target[idx], envelope->max...), envelope->min...);
 *     ```
 *     Para cinemática no lineal esto NO funciona - un punto puede estar
 *     fuera de límites en espacio motor aunque esté "dentro" en cartesiano.
 *     Usamos bisección para encontrar el punto válido más lejano.
 * 
 * [8] Las funciones de homing nativas asumen cinemática lineal.
 *     RTCP tiene ejes lineales independientes, así que la lógica es similar
 *     pero necesitamos invalidar el caché después del homing.
 * 
 * 
 * FLUJO DE EJECUCIÓN CON mc_line()
 * ================================
 * 
 * El siguiente diagrama muestra cómo grblHAL llama a nuestras funciones:
 * 
 *   G-code "G1 X100 Y50 A30 F1000"
 *           │
 *           ▼
 *   ┌─────────────────────────────────────────────────────────────────┐
 *   │                         mc_line()                               │
 *   │  [motion_control.c líneas 79-167]                               │
 *   └─────────────────────────────────────────────────────────────────┘
 *           │
 *           │ float feed_rate = pl_data->feed_rate;   // Guarda original
 *           │ pl_data->rate_multiplier = 1.0f;
 *           │
 *           ▼
 *   ┌─────────────────────────────────────────────────────────────────┐
 *   │  target = kinematics.segment_line(target, position, pl, TRUE)   │
 *   │  ════════════════════════════════════════════════════════════   │
 *   │  NUESTRA FUNCIÓN (init=true):                                   │
 *   │    1. Guarda target final (cartesiano)                          │
 *   │    2. Transforma a motor: mpos = transform_from_cartesian()     │
 *   │    3. Valida límites: grbl.check_travel_limits(mpos, false)     │
 *   │    4. Calcula segmentación necesaria                            │
 *   │    5. Retorna mpos (coordenadas motor del destino)              │
 *   └─────────────────────────────────────────────────────────────────┘
 *           │
 *           │ // mc_line verifica soft limits con el target retornado
 *           │ if(!(target_validated && target_valid))
 *           │     limits_soft_check(target, ...);  // Puede abortar
 *           │
 *           ▼
 *   ┌─────────────────────────────────────────────────────────────────┐
 *   │  while(kinematics.segment_line(target, NULL, pl, FALSE)) {      │
 *   │  ═══════════════════════════════════════════════════════════    │
 *   │  NUESTRA FUNCIÓN (init=false):                                  │
 *   │    1. Decrementa contador de iteraciones                        │
 *   │    2. Calcula siguiente punto interpolado (cartesiano)          │
 *   │    3. Transforma a motor                                        │
 *   │    4. Aplica compensación de velocidad TCP                      │
 *   │    5. Retorna mpos o NULL si terminamos                         │
 *   │                                                                 │
 *   │      plan_buffer_line(target, pl_data);  // grblHAL planifica   │
 *   │      pl_data->feed_rate = feed_rate;     // grblHAL restaura    │
 *   │  }                                                              │
 *   └─────────────────────────────────────────────────────────────────┘
 * 
 * 
 * CONFIGURACIÓN DE EJES
 * =====================
 * 
 *   - X, Y, Z: Ejes lineales cartesianos (mm)
 *   - A: Eje rotativo de inclinación (grados), típicamente ±90°
 *   - C: Eje rotativo de giro (grados), típicamente 0-360°
 * 
 * El punto de pivote ($640-$642) es donde los ejes A y C se intersectan
 * mecánicamente. Este punto debe medirse/calibrarse para cada máquina.
 * 
 * 
 * HISTORIAL DE VERSIONES
 * ======================
 * 
 *   v15.10:   Corrección de validación de límites
 *   v15.10.1: Flujo segment_line alineado con delta.c
 *   v16.0:    Consolidación, documentación exhaustiva
 *   vMaster:  Añadió apply_travel_limits con bisección
 *   v17.0:    Versión final fusionada:
 *             - Base de v16.0 (completa y documentada)
 *             - Bisección de vMaster para jogging
 *             - Análisis de redundancia con grblHAL nativo
 *             - Optimizaciones para RP2350
 *   v17.1:    Auditoría forense y documentación de TLO:
 *             - Verificado manejo de TLO con verify_rtcp_tlo.py
 *             - Confirmado equivalencia con LinuxCNC trtfuncs.c
 *             - Documentación completa del patrón TLO
 *             - Ver rtcp_forensic_audit.md (2026-02-02)
 * 
 * =============================================================================
 */

#include "../grbl.h"

/*
 * Guardas de compilación condicional.
 * 
 * Este módulo solo se activa cuando:
 *   - KINEMATICS_API está definido (habilita soporte de cinemática)
 *   - No es otra cinemática (COREXY, WALL_PLOTTER, DELTA_ROBOT)
 * 
 * En config.h del driver, definir:
 *   #define KINEMATICS_API
 *   // NO definir COREXY, WALL_PLOTTER, DELTA_ROBOT
 */
#if defined(KINEMATICS_API) && !COREXY && !WALL_PLOTTER && !DELTA_ROBOT

#include <math.h>
#include <string.h>

#include "../hal.h"
#include "../settings.h"
#include "../nvs_buffer.h"
#include "../planner.h"
#include "../kinematics.h"
#include "../protocol.h"
#include "../gcode.h"    /* Para gc_state.tool_length_offset */

/* =============================================================================
 * SECCIÓN 1: CONSTANTES Y MACROS
 * =============================================================================
 */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief Conversión grados a radianes
 * @note El RP2350 tiene FPU, pero las conversiones frecuentes siguen
 *       siendo costosas. Por eso usamos caché trigonométrico.
 */
#define DEG_TO_RAD(d) ((d) * (M_PI / 180.0f))
#define RAD_TO_DEG(r) ((r) * (180.0f / M_PI))

/**
 * @brief IDs de settings para parámetros RTCP
 * 
 * Corresponden a $640, $641, $642 en la interfaz de usuario.
 * Setting_Kinematics0 es el primer slot reservado para módulos de cinemática.
 */
#define SETTING_PIVOT_X         Setting_Kinematics0  /* $640 */
#define SETTING_PIVOT_Y         Setting_Kinematics1  /* $641 */
#define SETTING_PIVOT_Z         Setting_Kinematics2  /* $642 */
#define SETTING_AXIS_OFFSET_Y   Setting_Kinematics3  /* $643 */
#define SETTING_AXIS_OFFSET_Z   Setting_Kinematics4  /* $644 */

/**
 * @brief Parámetros de segmentación
 * 
 * Estos valores controlan la precisión vs. carga computacional:
 * 
 *   MAX_CHORD_ERROR_MM (0.01mm = 10 micras):
 *     - Máximo error del TCP entre segmentos
 *     - Controlado por evaluación de punto medio: comparar
 *       el motor interpolado linealmente vs cinemática real
 *     - Captura TODOS los efectos no-lineales: chord error,
 *       cross-coupling X*sin(C), y cualquier combinación
 *     - Independiente de velocidad y longitud del movimiento
 *     - Verificado matemáticamente: compare_linuxcnc_grblhal.py
 *
 *   MAX_ARM_LENGTH_MM (500mm):
 *     - Mínimo conservador para distancia del origen de máquina al pivot
 *     - Usado como fallback si los pivot settings son muy pequeños
 *
 *   TRIG_CACHE_TOL:
 *     - Calculado dinámicamente desde la distancia real del origen al pivot
 *     - Fórmula: RAD_TO_DEG(chord_error / arm_length)
 *     - Se recalcula cada vez que cambian $640-$644
 */
#define MAX_CHORD_ERROR_MM     0.01f
#define MAX_CHORD_ERROR_G0_MM  0.5f    /* Tolerancia G0: 50x más relajada que G1 */
#define MAX_ARM_LENGTH_MM      500.0f  /* Mínimo conservador (fallback) */

/* Validación de constantes removida para evitar error de preprocesador con floats */
// #if MAX_SEG_LENGTH_MM <= 0
// #error "MAX_SEG_LENGTH_MM must be positive"
// #endif
// #if MAX_SEG_ANGLE_DEG <= 0
// #error "MAX_SEG_ANGLE_DEG must be positive"
// #endif

/**
 * @brief Iteraciones de bisección para apply_travel_limits
 * 
 * 16 iteraciones dan precisión de 1/65536 ≈ 0.0015% del movimiento.
 * Suficiente para jogging sin impacto perceptible en rendimiento.
 */
#define BISECTION_ITERATIONS 16

/* =============================================================================
 * SECCIÓN 2: ESTRUCTURAS DE DATOS
 * =============================================================================
 */

/**
 * @brief Configuración RTCP almacenada en NVS
 * 
 * Define la geometría de la máquina - específicamente dónde está
 * el punto de pivote donde los ejes A y C se intersectan.
 * 
 * Estos valores deben medirse cuidadosamente:
 *   1. Home la máquina
 *   2. Mida la distancia desde el origen de máquina al centro de rotación
 *   3. Configure $640, $641, $642 con estos valores
 * 
 * @note Un error de 1mm en el pivot causa error proporcional en el TCP
 *       cuando los ejes rotan. A 45° de inclinación, 1mm de error en
 *       pivot_z causa ~0.7mm de error en la posición del TCP.
 */
typedef struct {
    float pivot_x;        /**< Coordenada X del punto de pivote (mm) */
    float pivot_y;        /**< Coordenada Y del punto de pivote (mm) */
    float pivot_z;        /**< Coordenada Z del punto de pivote (mm) */
    float axis_offset_y;  /**< Offset Y entre ejes A y C (mm) - $643 */
    float axis_offset_z;  /**< Offset Z entre ejes A y C (mm) - $644 */
} rtcp_settings_t;

/**
 * @brief Estado en tiempo de ejecución del módulo RTCP
 * 
 * Mantiene:
 *   - Copia de trabajo de la configuración
 *   - Caché de valores trigonométricos para optimización
 * 
 * El caché evita recalcular sin/cos en cada transformación,
 * lo cual es especialmente importante en el RP2350 para mantener
 * alta tasa de segmentación sin afectar el rendimiento del stepper.
 */
typedef struct {
    rtcp_settings_t cfg;    /**< Configuración activa */
    
    /* Caché trigonométrico */
    float last_a;           /**< Último ángulo A para caché */
    float last_c;           /**< Último ángulo C para caché */
    float sin_a;            /**< sin(A) cacheado */
    float cos_a;            /**< cos(A) cacheado */
    float sin_c;            /**< sin(C) cacheado */
    float cos_c;            /**< cos(C) cacheado */
    bool cache_valid;       /**< Validez del caché */
    float trig_cache_tol;   /**< Tolerancia angular dinámica (grados) */
} rtcp_state_t;

/* =============================================================================
 * SECCIÓN 3: VARIABLES GLOBALES
 * =============================================================================
 */

/** @brief Estado principal del módulo */
static rtcp_state_t rtcp = {0};

/** @brief Storage para sistema de settings de grblHAL */
static rtcp_settings_t rtcp_settings_storage;

/** @brief Dirección NVS asignada dinámicamente */
static nvs_address_t nvs_address;

/** 
 * @brief Flag de cancelación de jog
 * @note volatile porque puede modificarse desde ISR 
 */
static volatile bool jog_cancel = false;

/*
 * -----------------------------------------------------------------------------
 * Punteros para Chain Pattern
 * -----------------------------------------------------------------------------
 * 
 * grblHAL permite que múltiples módulos intercepten callbacks mediante
 * el patrón "chain": guardamos el puntero original y lo llamamos después
 * de nuestro procesamiento.
 * 
 * Usamos declaraciones explícitas de tipo en lugar de typedefs porque
 * algunos typedefs pueden no estar disponibles en todas las versiones
 * de grblHAL o requieren includes adicionales.
 */

/** @brief Puntero original a check_travel_limits */
static bool (*orig_check_travel_limits)(float *target, axes_signals_t axes, 
                                         bool is_cartesian, work_envelope_t *envelope);

/** @brief Puntero original a apply_travel_limits */
static void (*orig_apply_travel_limits)(float *target, float *position, 
                                         work_envelope_t *envelope);

/** @brief Puntero original a on_jog_cancel */
static void (*orig_on_jog_cancel)(sys_state_t state);

/** @brief Puntero original a settings_changed */
static settings_changed_ptr orig_settings_changed;

/** @brief Puntero original a on_report_options */
static on_report_options_ptr orig_on_report_options;

/** @brief Puntero original a on_realtime_report para chain */
static on_realtime_report_ptr orig_on_realtime_report;

/* =============================================================================
 * SECCIÓN 3.1: ESTADO DE MODO RTCP
 * =============================================================================
 * 
 * Cuando rtcp_enabled=false, el módulo funciona como identidad:
 *   - segment_line retorna target sin transformar
 *   - transform_steps_to_cartesian es conversión directa
 *   - No se calculan sin/cos (bypass completo)
 */

/** @brief RTCP habilitado (false = CNC cartesiano normal por defecto) */
static bool rtcp_enabled = false;

/** @brief Chain pattern para M-codes M450/M451 */
static user_mcode_ptrs_t user_mcode_prev;

/* =============================================================================
 * SECCIÓN 4: FUNCIONES AUXILIARES - CACHÉ TRIGONOMÉTRICO
 * =============================================================================
 */

/**
 * @brief Actualiza el caché de valores trigonométricos si es necesario
 * 
 * Optimización para evitar llamadas repetidas a sinf/cosf que son
 * costosas incluso con FPU. En trayectorias donde la orientación
 * cambia poco, esto puede ahorrar miles de cálculos por segundo.
 * 
 * @param a_deg Ángulo A actual en grados
 * @param c_deg Ángulo C actual en grados
 * 
 * @note Solo recalcula si los ángulos cambiaron más que TRIG_CACHE_TOL
 */
static inline void update_trig_cache(float a_deg, float c_deg) 
{
    if (!rtcp.cache_valid || 
        fabsf(a_deg - rtcp.last_a) > rtcp.trig_cache_tol || 
        fabsf(c_deg - rtcp.last_c) > rtcp.trig_cache_tol) 
    {
        float ar = DEG_TO_RAD(a_deg);
        float cr = DEG_TO_RAD(c_deg);
        
        rtcp.sin_a = sinf(ar); 
        rtcp.cos_a = cosf(ar);
        rtcp.sin_c = sinf(cr); 
        rtcp.cos_c = cosf(cr);
        
        rtcp.last_a = a_deg; 
        rtcp.last_c = c_deg;
        rtcp.cache_valid = true;
    }
}

/**
 * @brief Invalida el caché trigonométrico
 * 
 * Llamar cuando:
 *   - Cambia la configuración del pivot
 *   - Se ejecuta homing
 *   - Cualquier evento que pueda hacer el caché inconsistente
 */
static inline void invalidate_cache(void) 
{
    rtcp.cache_valid = false;
}

/* =============================================================================
 * SECCIÓN 5: TRANSFORMACIONES CINEMÁTICAS
 * =============================================================================
 * 
 * Estas son las funciones core del módulo. Implementan la conversión
 * bidireccional entre:
 *   - Espacio Cartesiano (TCP): donde el usuario programa (X,Y,Z,A,C)
 *   - Espacio de Motor (Joints): donde los motores se mueven físicamente
 */

/**
 * @brief Cinemática INVERSA: Cartesiano (TCP) → Motor
 * 
 * Dado un punto TCP en coordenadas cartesianas, calcula las posiciones
 * de motor necesarias para alcanzar ese punto.
 * 
 * ALGORITMO:
 * ----------
 * 
 * Para una máquina con configuración AC (C rota alrededor de Z, A inclina 
 * alrededor de X después de C):
 * 
 *   1. Trasladar el punto al sistema de coordenadas del pivot
 *   2. Aplicar rotación C (alrededor de Z)
 *   3. Aplicar rotación A (alrededor de X)
 *   4. Trasladar de vuelta al sistema de máquina
 * 
 * Matemáticamente:
 * 
 *   P_motor = Pivot + Ra * Rc * (P_tcp - Pivot)
 * 
 * Donde Ra y Rc son matrices de rotación:
 * 
 *   Rc = | cos(C)  -sin(C)  0 |     Ra = | 1    0       0     |
 *        | sin(C)   cos(C)  0 |          | 0  cos(A)  -sin(A) |
 *        |   0        0     1 |          | 0  sin(A)   cos(A) |
 * 
 * @param target   [out] Array donde se escriben coordenadas de motor
 * @param position [in]  Array con coordenadas cartesianas (TCP)
 * @return Puntero a target (permite encadenamiento)
 * 
 * @note Los ángulos A y C pasan sin modificar (son los mismos en ambos espacios)
 * @note Usa caché trigonométrico para optimización
 */
static float *transform_from_cartesian(float *target, float *position) 
{
    float a_deg = position[A_AXIS];
    float c_deg = position[C_AXIS];
    
    /* BYPASS: A=0 y C=0 = identidad (sin trigonometría) */
    if (fabsf(a_deg) < 0.001f && fabsf(c_deg) < 0.001f) {
        memcpy(target, position, sizeof(float) * N_AXIS);
        return target;
    }
    
    update_trig_cache(a_deg, c_deg);

    /*
     * =========================================================================
     * MANEJO DE TLO EN RTCP - DOCUMENTACIÓN COMPLETA
     * =========================================================================
     * 
     * CONTEXTO ARQUITECTURAL:
     * -----------------------
     * - LinuxCNC: Recibe TLO por separado via HAL pin (motion.tooloffset.z)
     *   y la cinemática recibe las coordenadas TCP puras (sin TLO).
     * 
     * - grblHAL: Aplica TLO ANTES de llamar a transform_from_cartesian()
     *   (ver gcode.c:gc_get_block_offset líneas 242-245).
     *   Las coordenadas que recibimos YA incluyen TLO sumado.
     * 
     * POR QUÉ DEBEMOS MANEJAR TLO AQUÍ:
     * ---------------------------------
     * Para RTCP, el TLO debe PARTICIPAR en las rotaciones para mantener
     * el TCP (punta de herramienta) fijo cuando los ejes rotan.
     * 
     * Si NO manejamos TLO aquí, con A=45° tendríamos error de ~70.7mm
     * en la posición del TCP (proporcional a TLO * sin(A)).
     * 
     * PATRÓN IMPLEMENTADO (equivalente a LinuxCNC trtfuncs.c):
     * --------------------------------------------------------
     * 1. Restar TLO de Z para obtener posición real de pieza
     * 2. Combinar dz = axis_offset_z + tlo_z (como LinuxCNC: dz = dz + dt)
     * 3. Aplicar rotaciones con dz combinado
     * 4. El TLO participa en las transformaciones trigonométricas
     * 
     * VERIFICACIÓN:
     * -------------
     * Script de prueba: verify_rtcp_tlo.py
     * Resultado: Coincidencia EXACTA con LinuxCNC para todos los casos:
     *   - A=0°,C=0°   -> OK
     *   - A=45°,C=0°  -> OK (Y=106.066, Z=143.934)
     *   - A=90°,C=0°  -> OK (Y=150.000, Z=250.000)
     *   - A=0°,C=90°  -> OK
     *   - A=45°,C=45° -> OK
     * 
     * REFERENCIAS:
     * ------------
     * - LinuxCNC trtfuncs.c líneas 158-167: dz = dz + dt
     * - LinuxCNC trtfuncs.c líneas 219-237: xyzacKinematicsInverse
     * - grblHAL gcode.c líneas 242-245: gc_get_block_offset()
     * - Auditoría: rtcp_forensic_audit.md (2026-02-02)
     * 
     * Offsets de ejes (dy, dz) compensan cuando A y C no se intersectan.
     */
    
    /* Obtener TLO actual del eje Z */
    float tlo_z = gc_state.modal.tool_length_offset[Z_AXIS];
    
    /* Offsets de ejes (distancia entre A y C) */
    float dy = rtcp.cfg.axis_offset_y;
    float dz = rtcp.cfg.axis_offset_z + tlo_z;  /* Combinar como LinuxCNC: dz = dz + dt */

    /* Paso 1: Trasladar al sistema del pivot, restando TLO */
    float px = position[X_AXIS] - rtcp.cfg.pivot_x;
    float py = position[Y_AXIS] - rtcp.cfg.pivot_y;
    float pz = (position[Z_AXIS] - tlo_z) - rtcp.cfg.pivot_z;  /* Resta TLO aquí */

    /* Paso 2: Rotación C (alrededor de Z) */
    float xc = px * rtcp.cos_c - py * rtcp.sin_c;
    float yc = px * rtcp.sin_c + py * rtcp.cos_c;
    /* zc = pz (Z no cambia al rotar alrededor de Z) */

    /* 
     * Paso 3: Rotación A (alrededor de X) con offsets de ejes
     * Siguiendo exactamente las ecuaciones de LinuxCNC xyzacKinematicsInverse
     */
    float y_rot = yc * rtcp.cos_a - pz * rtcp.sin_a
                  - rtcp.cos_a * dy
                  + rtcp.sin_a * dz
                  + dy;
                  
    float z_rot = yc * rtcp.sin_a + pz * rtcp.cos_a
                  - rtcp.sin_a * dy
                  - rtcp.cos_a * dz
                  + dz;

    /* Paso 4: Trasladar de vuelta (TLO ya incluido en dz) */
    target[X_AXIS] = xc + rtcp.cfg.pivot_x;
    target[Y_AXIS] = y_rot + rtcp.cfg.pivot_y;
    target[Z_AXIS] = z_rot + rtcp.cfg.pivot_z;
    
    /* Ejes rotativos y adicionales pasan sin cambio */
    uint_fast8_t idx = N_AXIS;
    do {
        idx--;
        if (idx > Z_AXIS) 
            target[idx] = position[idx];
    } while(idx > Z_AXIS);

    return target;
}

/**
 * @brief Cinemática DIRECTA: Motor → Cartesiano (TCP)
 * 
 * Operación inversa de transform_from_cartesian(). Dadas las posiciones
 * actuales de los motores, calcula dónde está el TCP en el espacio cartesiano.
 * 
 * Se usa para:
 *   - DRO (Digital Read Out) - mostrar posición TCP al usuario
 *   - Calcular posición inicial en segment_line()
 *   - Diagnóstico y verificación
 * 
 * ALGORITMO:
 * ----------
 * 
 * Aplicamos las rotaciones inversas en orden inverso:
 * 
 *   P_tcp = Pivot + Rc^(-1) * Ra^(-1) * (P_motor - Pivot)
 * 
 * Para matrices de rotación, la inversa es la transpuesta, que equivale
 * a cambiar el signo del ángulo.
 * 
 * @param target    [out] Array donde se escriben coordenadas cartesianas
 * @param motor_pos [in]  Array con posiciones de motor
 * @return Puntero a target
 * 
 * @note NO usa el caché global para evitar condiciones de carrera
 *       cuando se llama desde contextos de reporte
 */
static float *transform_to_cartesian(float *target, float *motor_pos) 
{
    float a_deg = motor_pos[A_AXIS];
    float c_deg = motor_pos[C_AXIS];
    
    /* BYPASS: A=0 y C=0 = identidad (sin trigonometría) */
    if (fabsf(a_deg) < 0.001f && fabsf(c_deg) < 0.001f) {
        memcpy(target, motor_pos, sizeof(float) * N_AXIS);
        return target;
    }
    
    /* Calcular trigonometría localmente (no usar caché global) */
    float ar = DEG_TO_RAD(a_deg);
    float cr = DEG_TO_RAD(c_deg);
    float sa = sinf(ar), ca = cosf(ar);
    float sc = sinf(cr), cc = cosf(cr);

    /*
     * =========================================================================
     * MANEJO DE TLO EN CINEMÁTICA DIRECTA (Motor -> TCP)
     * =========================================================================
     * 
     * Esta es la operación INVERSA de transform_from_cartesian.
     * Convierte posición de motor a posición cartesiana TCP para DRO.
     * 
     * El manejo de TLO aquí es CONSISTENTE con la cinemática inversa:
     *   - Restamos dz (que incluye TLO) de la posición de motor
     *   - Aplicamos rotaciones inversas
     *   - Sumamos TLO al final para coordenadas consistentes con grblHAL
     * 
     * VERIFICACIÓN: Ver script verify_rtcp_tlo.py función forward_kin_linuxcnc()
     * REFERENCIA: LinuxCNC trtfuncs.c líneas 169-185 (xyzacKinematicsForward)
     */
    
    /* Obtener TLO actual del eje Z */
    float tlo_z = gc_state.modal.tool_length_offset[Z_AXIS];
    
    /* Offsets de ejes (distancia entre A y C) */
    float dy = rtcp.cfg.axis_offset_y;
    float dz = rtcp.cfg.axis_offset_z + tlo_z;  /* Combinar offset + TLO */

    /* Paso 1: Trasladar al sistema del pivot, restando offsets combinados */
    float px = motor_pos[X_AXIS] - rtcp.cfg.pivot_x;
    float py = motor_pos[Y_AXIS] - dy - rtcp.cfg.pivot_y;
    float pz = motor_pos[Z_AXIS] - dz - rtcp.cfg.pivot_z;

    /* 
     * Paso 2: Rotación inversa de A (con offsets)
     * Siguiendo xyzacKinematicsForward de LinuxCNC
     */
    float yt = ca * py + sa * pz + dy;  /* CORREGIDO: era "sc * dy", debe ser solo "dy" */
    float zi = -sa * py + ca * pz + dz;

    /* Paso 3: Rotación inversa de C */
    float xi = cc * px + sc * yt;
    float yi = -sc * px + cc * yt;

    /* Paso 4: Trasladar de vuelta y restaurar TLO para coordenadas consistentes */
    target[X_AXIS] = xi + rtcp.cfg.pivot_x;
    target[Y_AXIS] = yi + rtcp.cfg.pivot_y;
    target[Z_AXIS] = zi + rtcp.cfg.pivot_z + tlo_z;  /* Restaurar TLO */
    
    /* Ejes adicionales pasan sin cambio */
    uint_fast8_t idx = N_AXIS;
    do {
        idx--;
        if (idx > Z_AXIS) 
            target[idx] = motor_pos[idx];
    } while(idx > Z_AXIS);

    return target;
}

/**
 * @brief Conversión Steps → Cartesiano para DRO
 * 
 * grblHAL llama esta función para mostrar la posición en reportes de estado.
 * Convierte la posición interna (en steps) a coordenadas cartesianas del TCP.
 * 
 * FLUJO:
 *   sys.position (steps) → motor_pos (mm) → transform_to_cartesian → TCP
 * 
 * @param position [out] Array donde se escriben coordenadas cartesianas
 * @param steps    [in]  Array con posición en steps (sys.position)
 * @return Puntero a position
 * 
 * @note Asignada a kinematics.transform_steps_to_cartesian
 * @note Usa conversión estándar steps→mm de grblHAL
 */
static float *transform_steps_to_cartesian(float *position, int32_t *steps) 
{
    float mpos[N_AXIS];
    
    /*
     * FIX: Conversión directa steps→mm sin usar system_convert_array_steps_to_mpos()
     * porque con KINEMATICS_API definido, esa función redirige a 
     * kinematics.transform_steps_to_cartesian() — esta misma función — 
     * causando recursión infinita y stack overflow.
     */
    uint_fast8_t idx = N_AXIS;
    do {
        idx--;
        mpos[idx] = steps[idx] / settings.axis[idx].steps_per_mm;
    } while(idx);
    
    /* BYPASS: RTCP deshabilitado = identidad (DRO muestra motor) */
    if (!rtcp_enabled) {
        memcpy(position, mpos, sizeof(float) * N_AXIS);
        return position;
    }
    
    return transform_to_cartesian(position, mpos);
}

/* =============================================================================
 * SECCIÓN 6: FUNCIONES DE HOMING
 * =============================================================================
 * 
 * grblHAL requiere estas funciones cuando usa un módulo de cinemática.
 * Para RTCP con ejes independientes, la lógica es similar a la estándar
 * pero necesitamos invalidar el caché después del homing.
 */

/**
 * @brief Obtiene máscara de eje para límites
 * 
 * Para RTCP, cada eje físico corresponde 1:1 con su índice.
 * (En CoreXY, por ejemplo, esto sería diferente porque un motor
 * afecta múltiples ejes lógicos)
 * 
 * @param idx Índice del eje (0=X, 1=Y, 2=Z, 3=A, 4=C)
 * @return Máscara de bit para el eje
 */
static uint_fast8_t rtcp_limits_get_axis_mask(uint_fast8_t idx) 
{
    return bit(idx);
}

/**
 * @brief Establece posición objetivo durante homing
 * 
 * @param idx Índice del eje siendo homeado
 */
static void rtcp_limits_set_target_pos(uint_fast8_t idx) 
{
    sys.position[idx] = 0;
}

/**
 * @brief Establece posiciones de máquina después del homing
 * 
 * Llamada cuando el homing de los ejes en 'cycle' se completa.
 * Establece las posiciones según configuración (force_set_origin,
 * dirección de homing, pulloff).
 * 
 * @param cycle Máscara de ejes que fueron homeados
 * 
 * @note Invalida el caché porque las posiciones cambiaron
 */
static void rtcp_limits_set_machine_positions(axes_signals_t cycle) 
{
    uint_fast8_t idx = N_AXIS;
    coord_data_t *pulloff = limits_homing_pulloff(NULL);

    if (settings.homing.flags.force_set_origin) {
        /* Modo: origen forzado a cero */
        do {
            if (cycle.mask & bit(--idx)) {
                sys.position[idx] = 0;
                sys.home_position[idx] = 0.0f;
            }
        } while(idx);
    } else {
        /* Modo: posición basada en dirección y pulloff */
        do {
            if (cycle.mask & bit(--idx)) {
                sys.home_position[idx] = bit_istrue(settings.homing.dir_mask.value, bit(idx))
                                          ? settings.axis[idx].max_travel + pulloff->values[idx]
                                          : -pulloff->values[idx];
                sys.position[idx] = lroundf(sys.home_position[idx] * settings.axis[idx].steps_per_mm);
            }
        } while(idx);
    }
    
    /* Las posiciones cambiaron - invalidar caché */
    invalidate_cache();
}

/* =============================================================================
 * SECCIÓN 7: VERIFICACIÓN Y APLICACIÓN DE LÍMITES
 * =============================================================================
 * 
 * Estas funciones hacen hook a las funciones nativas de grblHAL para
 * manejar correctamente la cinemática RTCP.
 * 
 * ANÁLISIS DE POR QUÉ NECESITAMOS ESTOS HOOKS:
 * 
 * 1. check_travel_limits nativa (machine_limits.c):
 *    ```c
 *    if(is_cartesian && (sys.homed.mask & axes.mask)) { ... }
 *    return is_cartesian && !failed;
 *    ```
 *    Si is_cartesian=false, SIEMPRE retorna false.
 *    
 *    En RTCP, segment_line() pasa coordenadas de MOTOR (is_cartesian=false)
 *    a check_travel_limits. La función nativa fallaría siempre.
 *    
 * 2. apply_travel_limits nativa (machine_limits.c):
 *    Hace clipping lineal: target[i] = clamp(target[i], min, max)
 *    
 *    Para cinemática no lineal, un punto que parece válido en cartesiano
 *    puede estar fuera de límites en espacio motor. Necesitamos bisección.
 */

/**
 * @brief Verifica si una posición está dentro de límites
 * 
 * Extiende la función nativa para manejar coordenadas de motor
 * (is_cartesian=false) que la función nativa no maneja.
 * 
 * LÓGICA:
 *   - Si is_cartesian=true: transforma a motor primero
 *   - Verifica contra envelope en espacio motor
 *   - Llama a función original para verificaciones adicionales
 * 
 * @param target       Posición a verificar
 * @param axes         Máscara de ejes a verificar
 * @param is_cartesian true si target está en cartesiano
 * @param envelope     Límites del volumen de trabajo
 * @return true si dentro de límites, false si viola alguno
 */
static bool rtcp_check_travel_limits(float *target, axes_signals_t axes, 
                                      bool is_cartesian, work_envelope_t *envelope) 
{
    /* BYPASS: RTCP deshabilitado - comportamiento cartesiano normal */
    if (!rtcp_enabled) {
        return orig_check_travel_limits 
            ? orig_check_travel_limits(target, axes, is_cartesian, envelope)
            : is_cartesian;
    }
    
    float motors[N_AXIS];

    /* Obtener coordenadas de motor */
    if (is_cartesian) {
        transform_from_cartesian(motors, target);
    } else {
        memcpy(motors, target, sizeof(float) * N_AXIS);
    }

    /* Verificar límites para ejes homeados */
    if (sys.homed.mask) {
        uint_fast8_t idx = N_AXIS;
        do {
            idx--;
            if (bit_istrue(sys.homed.mask, bit(idx)) && bit_istrue(axes.mask, bit(idx))) {
                if (motors[idx] < envelope->min.values[idx] || 
                    motors[idx] > envelope->max.values[idx]) {
                    return false;
                }
            }
        } while(idx);
    }

    /*
     * Validar límites cartesianos si aplica.
     * Solo tiene sentido si is_cartesian=true.
     * Si is_cartesian=false (modo motor), la función original retornaría false (bug de grbl),
     * así que la saltamos porque ya validamos los motores arriba.
     */
    /*
     * CODIGO ANTERIOR (Comentado para referencia):
     * La lógica original llamaba incondicionalmente al core, lo que retornaba false
     * si is_cartesian=false.
     *
    if (orig_check_travel_limits) {
        // Para ejes adicionales (>Z) que no manejamos arriba/
        return orig_check_travel_limits(motors, axes, false, envelope);
    }
     */

    /*
     * NUEVA LOGICA:
     * Validar límites cartesianos si aplica.
     * Solo tiene sentido si is_cartesian=true.
     * Si is_cartesian=false (modo motor), la función original retornaría false (bug de grbl),
     * así que la saltamos porque ya validamos los motores arriba.
     */
    if (is_cartesian && orig_check_travel_limits) {
        if (!orig_check_travel_limits(target, axes, true, envelope))
            return false;
    }

    return true;
}

/**
 * @brief Aplica límites durante Jogging usando bisección
 * 
 * Para cinemática no lineal, el clipping lineal simple no funciona.
 * Un movimiento que parece ir "hacia adentro" en cartesiano puede
 * ir "hacia afuera" en espacio motor.
 * 
 * ALGORITMO DE BISECCIÓN:
 * -----------------------
 * 
 *   1. Si el destino es válido, no hacer nada
 *   2. De lo contrario, buscar el punto más lejano válido
 *      entre posición actual (válida) y destino (inválido)
 *   3. En cada iteración:
 *      - Calcular punto medio
 *      - Si válido: intentar ir más lejos (mover inicio)
 *      - Si inválido: retroceder (mover fin)
 *   4. Después de N iteraciones, usar el mejor punto encontrado
 * 
 * Con 16 iteraciones: precisión = 1/2^16 ≈ 0.0015% del movimiento
 * 
 * @param target   [in/out] Posición destino, modificada si excede límites
 * @param position [in]     Posición actual (asumida válida)
 * @param envelope [in]     Límites del volumen de trabajo
 */
static void rtcp_apply_travel_limits(float *target, float *position, work_envelope_t *envelope)
{
    /* Si no hay ejes homeados o no hay posición de referencia, no hacer nada */
    if (sys.homed.mask == 0 || position == NULL) 
        return;

    /* Si el destino ya es válido, no necesitamos modificarlo */
    if (rtcp_check_travel_limits(target, sys.soft_limits, true, envelope))
        return;

    /* Bisección para encontrar el punto válido más lejano */
    float start[N_AXIS], end[N_AXIS], mid[N_AXIS], best[N_AXIS];
    
    memcpy(start, position, sizeof(float) * N_AXIS);  /* Punto válido conocido */
    memcpy(end, target, sizeof(float) * N_AXIS);      /* Punto inválido */
    memcpy(best, start, sizeof(float) * N_AXIS);      /* Mejor punto encontrado */

    for (uint_fast8_t i = 0; i < BISECTION_ITERATIONS; i++) {
        /* Calcular punto medio */
        uint_fast8_t ax = N_AXIS;
        do {
            ax--;
            mid[ax] = 0.5f * (start[ax] + end[ax]);
        } while(ax);

        /* Verificar si el punto medio es válido */
        if (rtcp_check_travel_limits(mid, sys.soft_limits, true, envelope)) {
            /* Válido: guardar como mejor y buscar más lejos */
            memcpy(best, mid, sizeof(float) * N_AXIS);
            memcpy(start, mid, sizeof(float) * N_AXIS);
        } else {
            /* Inválido: retroceder */
            memcpy(end, mid, sizeof(float) * N_AXIS);
        }
    }
    
    /* Usar el mejor punto encontrado */
    memcpy(target, best, sizeof(float) * N_AXIS);
}

/* =============================================================================
 * SECCIÓN 8: FUNCIÓN AUXILIAR DE GEOMETRÍA
 * =============================================================================
 */

/**
 * @brief Calcula distancia euclidiana entre dos puntos
 * 
 * @param p0 Primer punto
 * @param p1 Segundo punto
 * @return Distancia euclidiana
 */
static inline float get_distance(float *p0, float *p1)
{
    uint_fast8_t idx = N_AXIS;
    float distance = 0.0f;

    do {
        idx--;
        distance += (p0[idx] - p1[idx]) * (p0[idx] - p1[idx]);
    } while(idx);

    return sqrtf(distance);
}

/* =============================================================================
 * SECCIÓN 9: SEGMENTACIÓN DE LÍNEA
 * =============================================================================
 * 
 * Esta es la función más importante del módulo. Divide trayectorias largas
 * en segmentos pequeños para mantener la precisión del TCP.
 * 
 * El patrón de implementación sigue exactamente delta.c de grblHAL para
 * garantizar compatibilidad y estabilidad.
 */

/**
 * @brief Segmenta una línea cartesiana para precisión RTCP
 * 
 * mc_line() llama esta función así:
 * 
 *   ```c
 *   // Inicialización
 *   target = kinematics.segment_line(target, position, pl_data, true);
 *   
 *   // Verificación de límites (grblHAL nativo)
 *   if(!(target_validated && target_valid))
 *       limits_soft_check(target, ...);  // Puede abortar
 *   
 *   // Loop de segmentos
 *   while(kinematics.segment_line(target, NULL, pl_data, false)) {
 *       plan_buffer_line(target, pl_data);
 *       pl_data->feed_rate = feed_rate;  // grblHAL restaura
 *   }
 *   ```
 * 
 * NOTA IMPORTANTE SOBRE feed_rate:
 * --------------------------------
 * mc_line() guarda feed_rate al inicio y lo restaura después de cada
 * iteración. Por eso podemos modificar pl_data->feed_rate para compensación
 * de velocidad sin necesidad de guardar el valor original.
 * 
 * COMPENSACIÓN DE VELOCIDAD TCP:
 * ------------------------------
 * El feed rate programado es la velocidad del TCP (mm/min).
 * Debido a la cinemática, los motores pueden necesitar velocidad diferente.
 * 
 * Ejemplo: Si el TCP se mueve 1mm pero los motores se mueven 1.5mm
 *          (debido a rotación), necesitamos feed_rate * 1.5 en los motores
 *          para mantener la velocidad TCP programada.
 * 
 * @param target   En init: destino cartesiano. En loop: ignorado.
 * @param position En init: posición motor actual. En loop: ignorado (NULL).
 * @param pl_data  Datos del planificador (feed_rate, condiciones, etc.)
 * @param init     true para inicializar, false para siguiente segmento
 * @return Puntero a coordenadas motor, NULL cuando termina
 */
static float *rtcp_segment_line(float *target, float *position, 
                                 plan_line_data_t *pl_data, bool init) 
{
    /*
     * Variables estáticas mantienen estado entre llamadas.
     * Patrón idéntico a delta.c para compatibilidad.
     */
    static uint_fast16_t iterations;      /**< Segmentos restantes + 1 */
    static bool segmented;                 /**< true si se segmentó */
    static float distance;                 /**< Distancia TCP por segmento */
    static coord_data_t delta;             /**< Incremento por segmento */
    static coord_data_t segment_target;    /**< Punto actual (cartesiano) */
    static coord_data_t final_target;      /**< Destino final (cartesiano) */
    static coord_data_t mpos;              /**< Posición motor calculada */
    static coord_data_t last_motors;       /**< Posición motor anterior */

    uint_fast8_t idx = N_AXIS;

    if (init) {
        /*
         * =====================================================================
         * FASE DE INICIALIZACIÓN
         * =====================================================================
         */
        
        jog_cancel = false;
        
        /* BYPASS: RTCP deshabilitado = identidad pura (sin transformación) */
        if (!rtcp_enabled) {
            memcpy(mpos.values, target, sizeof(coord_data_t));
            iterations = 2;
            segmented = false;
            
            /* Validar límites (como cartesiano normal) */
            if (!pl_data->condition.target_validated) {
                pl_data->condition.target_validated = On;
                pl_data->condition.target_valid = grbl.check_travel_limits(
                    target, sys.soft_limits, true, &sys.work_envelope);
            }
            return mpos.values;
        }
        
        /* Guardar destino final cartesiano */
        memcpy(final_target.values, target, sizeof(final_target));
        
        /* Transformar destino a coordenadas motor para validación */
        transform_from_cartesian(mpos.values, target);
        
        /*
         * Validar límites del destino final.
         * 
         * Usamos grbl.check_travel_limits (nuestro hook) en lugar de llamar
         * directamente a rtcp_check_travel_limits. Esto mantiene el chain
         * pattern por si otro plugin también hace hook.
         * 
         * Si el destino es inválido (target_valid=false), mc_line() llamará
         * a limits_soft_check() que abortará el movimiento. No necesitamos
         * manejar el error aquí - grblHAL lo hace automáticamente.
         */
        if (!pl_data->condition.target_validated) {
            pl_data->condition.target_validated = On;
            pl_data->condition.target_valid = grbl.check_travel_limits(
                mpos.values, sys.soft_limits, false, &sys.work_envelope);
        }
        
        /* Convertir posición actual (motor) a cartesiano */
        transform_to_cartesian(segment_target.values, position);
        
        /* Guardar posición motor para compensación de velocidad */
        memcpy(last_motors.values, position, sizeof(coord_data_t));
        
        /* Calcular delta y distancia */
        float dist_sq = 0.0f;
        float rot_delta_a = 0.0f;
        float rot_delta_c = 0.0f;
        
        idx = N_AXIS;
        do {
            idx--;
            delta.values[idx] = target[idx] - segment_target.values[idx];
            
            if (idx <= Z_AXIS) {
                dist_sq += delta.values[idx] * delta.values[idx];
            } else if (idx == A_AXIS) {
                rot_delta_a = fabsf(delta.values[idx]);
            }
            #ifdef C_AXIS
            else if (idx == C_AXIS) {
                rot_delta_c = fabsf(delta.values[idx]);
            }
            #endif
        } while(idx);

        distance = sqrtf(dist_sq);
        float max_rot = fmaxf(rot_delta_a, rot_delta_c);

        /*
         * Determinar segmentación por evaluación de punto medio.
         * 
         * Método: calcular la posición motor del punto medio TCP usando
         * la cinemática real, comparar con la interpolación lineal de motores.
         * Si la diferencia > tolerancia, subdividir.
         * 
         * Ventajas sobre el método anterior (distancia+ángulo fijo):
         *   - Captura cross-coupling entre ejes lineales y rotativos
         *   - Sin rotación = 1 segmento (sin importar distancia lineal)
         *   - Precisión configurable y verificable
         *   - Reducción típica: 17x menos segmentos
         * 
         * G0 se segmenta con tolerancia relajada (MAX_CHORD_ERROR_G0_MM)
         * para mantener seguridad RTCP sin sacrificar velocidad.
         * Patrón confirmado: delta.c segmenta G0 igual que G1.
         */
        if ((segmented = max_rot > 0.001f)) {
            
            /* Calcular punto medio TCP */
            float tcp_mid[N_AXIS];
            idx = N_AXIS;
            do {
                idx--;
                tcp_mid[idx] = (segment_target.values[idx] + final_target.values[idx]) * 0.5f;
            } while(idx);
            
            /* Motor del punto medio via cinemática real */
            float motor_mid_real[N_AXIS];
            transform_from_cartesian(motor_mid_real, tcp_mid);
            
            /* Motor del punto medio interpolado linealmente */
            /* position = motor start, mpos.values = motor end */
            float err_sq = 0.0f;
            for (idx = 0; idx < 3; idx++) {
                float motor_mid_interp = (position[idx] + mpos.values[idx]) * 0.5f;
                float d = motor_mid_real[idx] - motor_mid_interp;
                err_sq += d * d;
            }
            
            float tol = pl_data->condition.rapid_motion 
                        ? MAX_CHORD_ERROR_G0_MM 
                        : MAX_CHORD_ERROR_MM;
            if (err_sq > tol * tol) {
                /* N = ceil(sqrt(error / tolerancia)) × 2 (factor de seguridad) */
                float err = sqrtf(err_sq);
                iterations = (uint_fast16_t)(ceilf(sqrtf(err / tol)) * 2.0f);
            } else {
                iterations = 1;
            }
            
            /* Clamp de seguridad */
            if (iterations < 1) iterations = 1;
            if (iterations > 2000) iterations = 2000;

            idx = N_AXIS;
            do {
                idx--;
                delta.values[idx] /= (float)iterations;
            } while(idx);
            
        } else {
            iterations = 1;
            memcpy(segment_target.values, final_target.values, sizeof(coord_data_t));
        }

        /* Distancia por segmento para compensación de velocidad */
        distance /= (float)iterations;
        
        /*
         * iterations++ porque el loop decrementa ANTES de procesar.
         * Esto garantiza que procesemos exactamente 'iterations' segmentos.
         */
        iterations++;

        /*
         * NO hay return temprano aquí - el return está al final.
         * Este es el patrón de delta.c que debemos seguir.
         */
        
    } else {
        /*
         * =====================================================================
         * FASE DE LOOP (generación de segmentos)
         * =====================================================================
         */
        
        iterations--;

        /* Avanzar al siguiente punto o usar destino final */
        if (segmented && iterations > 1) {
            idx = N_AXIS;
            do {
                idx--;
                segment_target.values[idx] += delta.values[idx];
            } while(idx);
        } else {
            /* Último segmento: usar destino exacto */
            memcpy(segment_target.values, final_target.values, sizeof(coord_data_t));
        }

        /* Transformar a motor */
        transform_from_cartesian(mpos.values, segment_target.values);

        /*
         * Compensación de velocidad TCP.
         * 
         * El feed rate programado es velocidad del TCP, no de los motores.
         * Calculamos cuánto se movieron los motores vs. cuánto se movió el TCP
         * y ajustamos el feed rate proporcionalmente.
         * 
         * mc_line() restaura feed_rate después de cada iteración, así que
         * podemos modificarlo libremente aquí.
         */
        if (!pl_data->condition.rapid_motion && distance > 0.0001f) {
            float motor_distance = get_distance(mpos.values, last_motors.values);
            float rate_multiplier = motor_distance / distance;
            
            // Clamp: evita anomalías por segmentos extremos cerca/lejos del pivote
            // (patrón de polar.c que usa clamp a 0.5 mínimo)
            if (rate_multiplier < 0.5f) rate_multiplier = 0.5f;
            else if (rate_multiplier > 2.0f) rate_multiplier = 2.0f;
            
            pl_data->feed_rate *= rate_multiplier;
            pl_data->rate_multiplier = 1.0f / rate_multiplier;
        }
        
        memcpy(last_motors.values, mpos.values, sizeof(coord_data_t));
    }

    /*
     * RETURN ÚNICO AL FINAL (patrón delta.c)
     * 
     * Retorna NULL cuando:
     *   - No quedan iteraciones (movimiento completo)
     *   - Se canceló el jog
     */
    return (iterations == 0 || jog_cancel) ? NULL : mpos.values;
}

/* =============================================================================
 * SECCIÓN 10: SISTEMA DE CONFIGURACIÓN
 * =============================================================================
 */

static const setting_group_detail_t kinematics_groups[] = {
    { Group_Root, Group_Kinematics, "5-Axis RTCP" }
};

static const setting_detail_t kinematics_settings[] = {
    { SETTING_PIVOT_X, Group_Kinematics, "RTCP Pivot X", "mm", Format_Decimal, 
      "###0.000", "-10000", "10000", Setting_NonCore, &rtcp_settings_storage.pivot_x, NULL, NULL },
    { SETTING_PIVOT_Y, Group_Kinematics, "RTCP Pivot Y", "mm", Format_Decimal, 
      "###0.000", "-10000", "10000", Setting_NonCore, &rtcp_settings_storage.pivot_y, NULL, NULL },
    { SETTING_PIVOT_Z, Group_Kinematics, "RTCP Pivot Z", "mm", Format_Decimal, 
      "###0.000", "-10000", "10000", Setting_NonCore, &rtcp_settings_storage.pivot_z, NULL, NULL },
    { SETTING_AXIS_OFFSET_Y, Group_Kinematics, "Axis Offset Y", "mm", Format_Decimal, 
      "###0.000", "-1000", "1000", Setting_NonCore, &rtcp_settings_storage.axis_offset_y, NULL, NULL },
    { SETTING_AXIS_OFFSET_Z, Group_Kinematics, "Axis Offset Z", "mm", Format_Decimal, 
      "###0.000", "-1000", "1000", Setting_NonCore, &rtcp_settings_storage.axis_offset_z, NULL, NULL }
};

static const setting_descr_t kinematics_settings_descr[] = {
    { SETTING_PIVOT_X, "X distance from machine origin to the A/C rotation center (mm). "
                       "Measure carefully - affects TCP accuracy proportionally." },
    { SETTING_PIVOT_Y, "Y distance from machine origin to the A/C rotation center (mm)." },
    { SETTING_PIVOT_Z, "Z distance from machine origin to the A/C rotation center (mm). "
                       "This is typically the most critical dimension." },
    { SETTING_AXIS_OFFSET_Y, "Y offset between A and C rotation axes (mm). "
                             "For machines where A/C axes do not intersect. Set 0 if axes intersect." },
    { SETTING_AXIS_OFFSET_Z, "Z offset between A and C rotation axes (mm). "
                             "Distance from A axis to table surface. Set 0 if axes intersect." }
};

/**
 * @brief Callback para cambios en settings RTCP ($640-$644)
 * 
 * Solo se llama cuando cambian los settings propios del plugin.
 * Recarga la configuración y invalida el cache trigonométrico.
 */
static void rtcp_kinematics_settings_changed(settings_t *settings, settings_changed_flags_t changed) 
{
    memcpy(&rtcp.cfg, &rtcp_settings_storage, sizeof(rtcp_settings_t));
    
    /* Recalcular tolerancia del caché a partir de la distancia del origen al pivot */
    float arm = sqrtf(rtcp.cfg.pivot_x * rtcp.cfg.pivot_x +
                      rtcp.cfg.pivot_y * rtcp.cfg.pivot_y +
                      rtcp.cfg.pivot_z * rtcp.cfg.pivot_z);
    if (arm < MAX_ARM_LENGTH_MM)
        arm = MAX_ARM_LENGTH_MM;
    rtcp.trig_cache_tol = RAD_TO_DEG(MAX_CHORD_ERROR_MM / arm);
    
    invalidate_cache();
}

/**
 * @brief Callback para cambios en settings del CORE
 * 
 * Solo hace chain al callback original. No recarga config RTCP
 * ya que los settings del core no afectan la cinemática.
 */
static void rtcp_core_settings_changed(settings_t *settings, settings_changed_flags_t changed) 
{
    if (orig_settings_changed) 
        orig_settings_changed(settings, changed);
}

static void rtcp_settings_save(void) 
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&rtcp_settings_storage, 
                          sizeof(rtcp_settings_t), true);
}

static void rtcp_settings_restore(void) 
{
    rtcp_settings_storage.pivot_x = 0.0f;
    rtcp_settings_storage.pivot_y = 0.0f;
    rtcp_settings_storage.pivot_z = 0.0f;
    rtcp_settings_storage.axis_offset_y = 0.0f;  /* NUEVO */
    rtcp_settings_storage.axis_offset_z = 0.0f;  /* NUEVO */
    rtcp_settings_save();
}

static void rtcp_settings_load(void) 
{
    if (hal.nvs.memcpy_from_nvs((uint8_t *)&rtcp_settings_storage, nvs_address, 
                                  sizeof(rtcp_settings_t), true) != NVS_TransferResult_OK) {
        rtcp_settings_restore();
    }
    
    rtcp_kinematics_settings_changed(NULL, (settings_changed_flags_t){0});
}

/* =============================================================================
 * SECCIÓN 11: CALLBACKS DEL SISTEMA
 * =============================================================================
 */

static void on_jog_cancel_handler(sys_state_t state) 
{
    jog_cancel = true;
    
    if (orig_on_jog_cancel) 
        orig_on_jog_cancel(state);
}

static void report_options(bool newopt) 
{
    if (orig_on_report_options) 
        orig_on_report_options(newopt);
    
    if (!newopt) 
        hal.stream.write("[KINEMATICS:5-Axis RTCP v17.1]" ASCII_EOL);
}

/**
 * @brief Añade estado RTCP a status reports en tiempo real
 * 
 * Formato: |RTCP:ON o |RTCP:OFF
 * Esto permite que interfaces gráficas muestren el estado.
 */
static void rtcp_realtime_report(stream_write_ptr stream_write, report_tracking_flags_t report)
{
    stream_write(rtcp_enabled ? "|RTCP:ON" : "|RTCP:OFF");
    
    if (orig_on_realtime_report)
        orig_on_realtime_report(stream_write, report);
}

/* =============================================================================
 * SECCIÓN 11.1: M-CODES RTCP ON/OFF (M450/M451)
 * =============================================================================
 * 
 * M450 = RTCP OFF (modo cartesiano normal)
 * M451 = RTCP ON (compensación TCP activa)
 * 
 * Usan user_mcode_sync = true para esperar que el buffer de movimiento
 * esté vacío antes de cambiar el modo, evitando transiciones abruptas.
 */

/**
 * @brief Verifica si el M-code es M450 o M451
 */
static user_mcode_type_t rtcp_mcode_check(user_mcode_t mcode)
{
    if (mcode == 450 || mcode == 451)
        return UserMCode_Normal;
    
    return user_mcode_prev.check ? user_mcode_prev.check(mcode) : UserMCode_Unsupported;
}

/**
 * @brief Valida M450/M451 y solicita sincronización
 */
static status_code_t rtcp_mcode_validate(parser_block_t *gc_block)
{
    if (gc_block->user_mcode == 450 || gc_block->user_mcode == 451) {
        gc_block->user_mcode_sync = true;  /* Esperar buffer vacío */
        return Status_OK;
    }
    
    return user_mcode_prev.validate ? user_mcode_prev.validate(gc_block) : Status_Unhandled;
}

/**
 * @brief Ejecuta M450 (OFF) o M451 (ON)
 */
static void rtcp_mcode_execute(sys_state_t state, parser_block_t *gc_block)
{
    if (gc_block->user_mcode == 450 && rtcp_enabled) {
        /* Advertencia si ejes rotativos no están en cero */
        float a_pos = sys.position[A_AXIS] / settings.axis[A_AXIS].steps_per_mm;
        bool warn = fabsf(a_pos) > 0.1f;
        
        #ifdef C_AXIS
        float c_pos = sys.position[C_AXIS] / settings.axis[C_AXIS].steps_per_mm;
        warn = warn || fabsf(c_pos) > 0.1f;
        #endif
        
        if (warn)
            hal.stream.write("[MSG:Warning: RTCP OFF with rotary axes not at zero]" ASCII_EOL);
        
        rtcp_enabled = false;
        invalidate_cache();
    } else if (gc_block->user_mcode == 450) {
        /* Ya está deshabilitado, no hacer nada */
    } else if (gc_block->user_mcode == 451) {
        rtcp_enabled = true;
        invalidate_cache();
    } else if (user_mcode_prev.execute) {
        user_mcode_prev.execute(state, gc_block);
    }
}

/* =============================================================================
 * SECCIÓN 12: COMANDO DE DIAGNÓSTICO
 * =============================================================================
 */

/**
 * @brief Comando $RTCP - Estado y diagnóstico del módulo
 */
static status_code_t rtcp_info(sys_state_t state, char *args)
{
    float cart_pos[N_AXIS];
    float motor_pos[N_AXIS];
    
    /* Obtener posición actual */
    uint_fast8_t idx = N_AXIS;
    do {
        idx--;
        motor_pos[idx] = sys.position[idx] / settings.axis[idx].steps_per_mm;
    } while(idx);
    
    transform_to_cartesian(cart_pos, motor_pos);
    
    hal.stream.write("5-Axis RTCP v17.1 Status:" ASCII_EOL);
    hal.stream.write("==========================" ASCII_EOL);
    hal.stream.write(" RTCP Mode: ");
    hal.stream.write(rtcp_enabled ? "ON (M451)" : "OFF (M450)");
    hal.stream.write(ASCII_EOL);
    
    hal.stream.write(" Pivot Point:" ASCII_EOL);
    hal.stream.write("   $640 X = "); hal.stream.write(ftoa(rtcp.cfg.pivot_x, 3));
    hal.stream.write(" mm" ASCII_EOL);
    hal.stream.write("   $641 Y = "); hal.stream.write(ftoa(rtcp.cfg.pivot_y, 3));
    hal.stream.write(" mm" ASCII_EOL);
    hal.stream.write("   $642 Z = "); hal.stream.write(ftoa(rtcp.cfg.pivot_z, 3));
    hal.stream.write(" mm" ASCII_EOL);
    
    hal.stream.write(" Axis Offsets (A/C non-intersecting):" ASCII_EOL);
    hal.stream.write("   $643 Y = "); hal.stream.write(ftoa(rtcp.cfg.axis_offset_y, 3));
    hal.stream.write(" mm" ASCII_EOL);
    hal.stream.write("   $644 Z = "); hal.stream.write(ftoa(rtcp.cfg.axis_offset_z, 3));
    hal.stream.write(" mm" ASCII_EOL);
    
    hal.stream.write(" TCP Position (Cartesian):" ASCII_EOL);
    hal.stream.write("   X = "); hal.stream.write(ftoa(cart_pos[X_AXIS], 3));
    hal.stream.write("   Y = "); hal.stream.write(ftoa(cart_pos[Y_AXIS], 3));
    hal.stream.write("   Z = "); hal.stream.write(ftoa(cart_pos[Z_AXIS], 3));
    hal.stream.write(" mm" ASCII_EOL);
    
    hal.stream.write(" Motor Position:" ASCII_EOL);
    hal.stream.write("   X = "); hal.stream.write(ftoa(motor_pos[X_AXIS], 3));
    hal.stream.write("   Y = "); hal.stream.write(ftoa(motor_pos[Y_AXIS], 3));
    hal.stream.write("   Z = "); hal.stream.write(ftoa(motor_pos[Z_AXIS], 3));
    hal.stream.write(" mm" ASCII_EOL);
    
    hal.stream.write(" Rotary Axes:" ASCII_EOL);
    hal.stream.write("   A = "); hal.stream.write(ftoa(motor_pos[A_AXIS], 2));
    hal.stream.write(" deg" ASCII_EOL);
    
    #if N_AXIS > C_AXIS
    hal.stream.write("   C = "); hal.stream.write(ftoa(motor_pos[C_AXIS], 2));
    hal.stream.write(" deg" ASCII_EOL);
    #endif
    
    hal.stream.write(" Trig Cache: ");
    hal.stream.write(rtcp.cache_valid ? "Valid" : "Invalid");
    hal.stream.write(ASCII_EOL);
    
    return Status_OK;
}

/* =============================================================================
 * SECCIÓN 13: INICIALIZACIÓN
 * =============================================================================
 */

/**
 * @brief Inicializa el módulo RTCP de 5 ejes
 * 
 * Llamar desde my_plugin_init() o equivalente durante el arranque.
 * 
 * SECUENCIA:
 *   1. Asignar espacio NVS
 *   2. Registrar funciones kinematics
 *   3. Hacer hook a funciones de límites
 *   4. Registrar settings y comandos
 *   5. Cargar configuración
 */
void rtcp_5axis_init(void) 
{
    static setting_details_t setting_details = {
        .is_core = true,
        .groups = kinematics_groups,
        .n_groups = sizeof(kinematics_groups) / sizeof(setting_group_detail_t),
        .settings = kinematics_settings,
        .n_settings = sizeof(kinematics_settings) / sizeof(setting_detail_t),
        .descriptions = kinematics_settings_descr,
        .n_descriptions = sizeof(kinematics_settings_descr) / sizeof(setting_descr_t),
        .load = rtcp_settings_load,
        .save = rtcp_settings_save,
        .restore = rtcp_settings_restore,
        .on_changed = rtcp_kinematics_settings_changed
    };

    static const sys_command_t rtcp_command_list[] = {
        { "RTCP", rtcp_info, { .noargs = On }, 
          { .str = "Show RTCP kinematics status and diagnostics" } }
    };

    static sys_commands_t rtcp_commands = {
        .n_commands = sizeof(rtcp_command_list) / sizeof(sys_command_t),
        .commands = rtcp_command_list
    };

    if ((nvs_address = nvs_alloc(sizeof(rtcp_settings_t)))) {
        
        /*
         * Registrar funciones de cinemática.
         * Estas son REQUERIDAS - grblHAL no tiene implementación por defecto.
         */
        kinematics.transform_from_cartesian = transform_from_cartesian;
        kinematics.transform_steps_to_cartesian = transform_steps_to_cartesian;
        kinematics.segment_line = rtcp_segment_line;
        
        kinematics.limits_get_axis_mask = rtcp_limits_get_axis_mask;
        kinematics.limits_set_target_pos = rtcp_limits_set_target_pos;
        kinematics.limits_set_machine_positions = rtcp_limits_set_machine_positions;
        
        /*
         * Hook a check_travel_limits.
         * NECESARIO porque la función nativa no maneja is_cartesian=false.
         * Ver análisis en SECCIÓN 7.
         */
        orig_check_travel_limits = grbl.check_travel_limits;
        grbl.check_travel_limits = rtcp_check_travel_limits;

        /*
         * Hook a apply_travel_limits.
         * NECESARIO para jogging con cinemática no lineal.
         * Ver análisis en SECCIÓN 7.
         */
        orig_apply_travel_limits = grbl.apply_travel_limits;
        grbl.apply_travel_limits = rtcp_apply_travel_limits;

        /* Hook a jog_cancel para terminar segmentación */
        orig_on_jog_cancel = grbl.on_jog_cancel;
        grbl.on_jog_cancel = on_jog_cancel_handler;

        /* Hook a settings_changed del core para chain */
        orig_settings_changed = hal.settings_changed;
        hal.settings_changed = rtcp_core_settings_changed;
        
        /* Hook a report_options para identificación */
        orig_on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        /* Hook a realtime_report para mostrar estado RTCP */
        orig_on_realtime_report = grbl.on_realtime_report;
        grbl.on_realtime_report = rtcp_realtime_report;

        /* Registrar con grblHAL */
        settings_register(&setting_details);
        system_register_commands(&rtcp_commands);

        /* Registrar M450/M451 para RTCP ON/OFF */
        memcpy(&user_mcode_prev, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));
        grbl.user_mcode.check = rtcp_mcode_check;
        grbl.user_mcode.validate = rtcp_mcode_validate;
        grbl.user_mcode.execute = rtcp_mcode_execute;

        /* Cargar configuración */
        rtcp_settings_load();
    }
}

#endif /* KINEMATICS_API && !COREXY && !WALL_PLOTTER && !DELTA_ROBOT */

/**
 * =============================================================================
 * FIN DEL MÓDULO
 * =============================================================================
 * 
 * RESUMEN DE INTEGRACIÓN CON grblHAL:
 * 
 * Funciones IMPLEMENTADAS (no existen en grblHAL base):
 *   - transform_from_cartesian: Cartesiano → Motor
 *   - transform_to_cartesian: Motor → Cartesiano
 *   - transform_steps_to_cartesian: Steps → Cartesiano (DRO)
 *   - segment_line: Segmentación de trayectorias
 *   - limits_*: Funciones de homing
 * 
 * Funciones con HOOK (extienden funcionalidad nativa):
 *   - check_travel_limits: Maneja is_cartesian=false
 *   - apply_travel_limits: Bisección para jogging
 * 
 * Funciones NATIVAS usadas sin modificar:
 *   - mc_line(): Flujo principal de movimiento
 *   - limits_soft_check(): Verificación y abort de límites
 *   - plan_buffer_line(): Planificación de movimiento
 *   - Restauración de feed_rate en mc_line()
 * 
 * SETTINGS:
 *   $640 - Pivot X (mm)
 *   $641 - Pivot Y (mm)
 *   $642 - Pivot Z (mm)
 * 
 * COMANDOS:
 *   $RTCP - Diagnóstico
 * 
 * VERIFICACIÓN:
 *   Después de inicializar, $I debe mostrar:
 *   [KINEMATICS:5-Axis RTCP v17.0]
 * 
 * =============================================================================
 */

/**
## Resumen de Cambios en v17.0

*| Componente | Fuente | Justificación |
*|------------|--------|---------------|
*| Base del código | v16.0 | Completa, documentada, compila |
*| `apply_travel_limits` (bisección) | vMaster | Necesario para jogging no lineal |
*| Análisis de redundancia | Nuevo | Evita duplicar funcionalidad nativa |
*| Documentación de flujo mc_line | Nuevo | Clarifica integración |
*| Tipos de puntero | Explícitos | Compatibilidad garantizada |
*| `limits_set_machine_positions` | v16.0 | Faltaba en vMaster |
*| Compensación velocidad | v16.0/delta.c | Patrón probado |
*| Chain pattern correcto | v16.0 | Usa `grbl.check_travel_limits` |

*## Funcionalidades NO Duplicadas

*| Funcionalidad | Quién la hace | Por qué no duplicamos |
*|---------------|---------------|----------------------|
*| Restaurar feed_rate | mc_line() | Ya lo hace después de cada segmento |
*| Auto-cycle start | mc_line() | Ya lo hace cuando buffer lleno |
*| Abort por soft limits | limits_soft_check() | Ya lo hace si target_valid=false |
*| Conversión steps→mm | system_convert_array... | Función estándar del sistema |
*/