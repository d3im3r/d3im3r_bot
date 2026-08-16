#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

// =====================================================
// PWM
// =====================================================
#define PWM_CH_LEFT           0
#define PWM_CH_RIGHT          1

#define PWM_FREQ_HZ           1000
#define PWM_RES_BITS          8

#define PWM_LIMIT_PERCENT       100.0f
#define PWM_DEADBAND_PERCENT    12.0f
#define MOTOR_TIMEOUT_MS        500

// Dominio PWM real usado por el controlador
#define CTRL_PWM_MAX          255.0f

// =====================================================
// Control PI incremental en dominio PWM
// =====================================================
// Ecuación discreta implementada:
//
// u[k] = u[k-1] + Kp*(e[k] - e[k-1]) + Ki*Ts*e[k]
//
// Donde:
// - e[k]     = w_ref[k] - w_medida[k]
// - u[k]     = acción de control en dominio PWM [-255, 255]
// - Ts       = periodo real de muestreo en segundos, calculado con millis()
//
// Ventaja frente al PI posicional/no incremental:
// - No se almacena una integral acumulada independiente.
// - La acción anterior u[k-1] contiene el efecto integral.
// - La saturación directa de u[k] ayuda a reducir windup.
// =====================================================
#define CTRL_LEFT_KP_PWM       9.90f
#define CTRL_RIGHT_KP_PWM      9.6667f

#define CTRL_LEFT_KI_PWM       9.20f
#define CTRL_RIGHT_KI_PWM      9.0f

#define CTRL_OUTPUT_LIMIT_PERCENT   100.0f

// Para detener limpio cuando la referencia sea cero
#define CTRL_REF_ZERO_EPSILON       0.05f

#endif
