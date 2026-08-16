#ifndef ROBOT_PARAMS_H
#define ROBOT_PARAMS_H

// =====================================================
// Parámetros físicos del robot diferencial
// =====================================================
// ENCODER_PPR:
// Pulsos efectivos por revolución de la rueda o del eje medido.
// Este valor debe corresponder al conteo real usado por el firmware.
#define ENCODER_PPR          2112.0f

// Radio de la rueda [m]
#define WHEEL_RADIUS_M       0.0217f

// Distancia entre ruedas [m]
#define WHEEL_BASE_M         0.10144f

// Límite de velocidad angular de rueda usado por software [rad/s]
#define WHEEL_MAX_RAD_S      13.0f

#endif
