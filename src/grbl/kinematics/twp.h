/**
 * @file    twp.h
 * @brief   Tilted Work Plane (G68.2 / G53.1 / G69) interface
 * 
 * Prototipos para que gcode.c pueda llamar funciones TWP de rtcp.c.
 * La implementación vive en rtcp.c junto con la cinemática RTCP.
 */

#ifndef _TWP_H_
#define _TWP_H_

#include <stdbool.h>

void twp_set_euler_angles(float ox, float oy, float oz, float a1, float a2, float a3);
void twp_activate(void);
void twp_deactivate(void);
bool twp_is_active(void);
bool twp_is_defined(void);

#endif /* _TWP_H_ */
