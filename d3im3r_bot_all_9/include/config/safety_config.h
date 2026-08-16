#ifndef SAFETY_CONFIG_H
#define SAFETY_CONFIG_H

// =====================================================
// Configuración de seguridad por sensores ToF
// =====================================================
//
// Esta capa protege el robot ante obstáculos cercanos.
// Actúa sobre el comando /cmd_vel antes de convertirlo
// a velocidades de rueda.
//
// La seguridad NO reemplaza la navegación autónoma.
// La seguridad NO reemplaza el PI.
// La seguridad filtra comandos peligrosos.
//
// =====================================================

#define SAFETY_ENABLE                      true

// =====================================================
// Seguridad frontal
// =====================================================
//
// Si el robot avanza y el obstáculo frontal está dentro
// de la zona de parada, se bloquea la velocidad lineal.
//
// Si el obstáculo frontal está dentro de la zona de
// reducción, se limita progresivamente la velocidad.
//
// =====================================================

// Distancia frontal mínima permitida para avanzar [m]
#define SAFETY_FRONT_STOP_M                0.12f

// Distancia desde la cual se empieza a reducir velocidad [m]
#define SAFETY_FRONT_SLOW_M                0.30f

// Velocidad mínima permitida dentro de la zona lenta.
// Evita que el robot siga empujando fuerte cerca del obstáculo.
#define SAFETY_MIN_SLOW_FACTOR             0.20f

// =====================================================
// Seguridad lateral
// =====================================================
//
// Convención usada:
// - angular.z > 0  giro hacia la izquierda
// - angular.z < 0  giro hacia la derecha
//
// Si hay obstáculo muy cerca a un lado, se bloquea el giro
// hacia ese mismo lado.
//
// =====================================================

#define SAFETY_SIDE_STOP_M                 0.10f
#define SAFETY_SIDE_SLOW_M                 0.18f

// =====================================================
// Comportamiento ante sensores inválidos
// =====================================================
//
// Si el sensor frontal no está disponible:
// - true  -> Bloquea avance por seguridad.
// - false -> Permite avanzar, pero sin protección frontal.
//
// Recomendado: true.
//
// =====================================================

#define SAFETY_FAILSAFE_ON_FRONT_INVALID   true

// Si un sensor lateral no está disponible:
// - true  -> Bloquea giro hacia ese lado.
// - false -> Permite giro hacia ese lado.
//
// Recomendado: false para evitar que el robot quede demasiado
// limitado si solo falla un sensor lateral.
//
#define SAFETY_FAILSAFE_ON_SIDE_INVALID    false

// =====================================================
// Límites absolutos por seguridad
// =====================================================
//
// Estos límites se aplican al comando resultante.
// Sirven para evitar comandos excesivos desde ROS 2.
//
// =====================================================

#define SAFETY_MAX_LINEAR_M_S              0.22f//0.15f
#define SAFETY_MAX_ANGULAR_RAD_S           2.50f//1.20f

// Si se activa parada de emergencia frontal,
// se permite girar en el sitio para escapar.
#define SAFETY_ALLOW_TURN_ON_FRONT_STOP    true

// Si se activa parada de emergencia frontal,
// este es el máximo giro permitido durante escape.
#define SAFETY_FRONT_STOP_MAX_TURN_RAD_S   0.60f

#endif