#include "control.h"
#include "app_config.h"
#include "encoders.h"
#include "motors.h"

#include <Arduino.h>
#include <math.h>

static uint32_t g_last_control_ms = 0;

static float g_left_ref_rad_s  = 0.0f;
static float g_right_ref_rad_s = 0.0f;

// Watchdog de referencia
static uint32_t g_last_left_ref_ms  = 0;
static uint32_t g_last_right_ref_ms = 0;

// Estados rueda izquierda
static float e_l = 0.0f;
static float e_l_prev = 0.0f;
static float u_l = 0.0f;      // Porcentaje aplicado al motor [-100, 100]
static float u_l_pwm = 0.0f;  // Salida interna en PWM [-255, 255]

// Estados rueda derecha
static float e_r = 0.0f;
static float e_r_prev = 0.0f;
static float u_r = 0.0f;      // Porcentaje aplicado al motor [-100, 100]
static float u_r_pwm = 0.0f;  // Salida interna en PWM [-255, 255]

static float clamp_pwm(float u)
{
    if (u > CTRL_PWM_MAX) return CTRL_PWM_MAX;
    if (u < -CTRL_PWM_MAX) return -CTRL_PWM_MAX;
    return u;
}

static float pwm_to_percent(float pwm)
{
    return (pwm / CTRL_PWM_MAX) * 100.0f;
}

static void reset_left_controller()
{
    e_l = 0.0f;
    e_l_prev = 0.0f;
    u_l = 0.0f;
    u_l_pwm = 0.0f;
}

static void reset_right_controller()
{
    e_r = 0.0f;
    e_r_prev = 0.0f;
    u_r = 0.0f;
    u_r_pwm = 0.0f;
}

bool control_init()
{
    g_left_ref_rad_s = 0.0f;
    g_right_ref_rad_s = 0.0f;

    g_last_control_ms = millis();
    g_last_left_ref_ms = g_last_control_ms;
    g_last_right_ref_ms = g_last_control_ms;

    reset_left_controller();
    reset_right_controller();

    return true;
}

void control_set_left_ref(float ref_rad_s)
{
    g_left_ref_rad_s = ref_rad_s;
    g_last_left_ref_ms = millis();
}

void control_set_right_ref(float ref_rad_s)
{
    g_right_ref_rad_s = ref_rad_s;
    g_last_right_ref_ms = millis();
}

float control_get_left_ref()    { return g_left_ref_rad_s; }
float control_get_right_ref()   { return g_right_ref_rad_s; }

float control_get_left_u()      { return u_l; }
float control_get_right_u()     { return u_r; }

float control_get_left_error()  { return e_l; }
float control_get_right_error() { return e_r; }

static void update_left_incremental_pi(float ref_rad_s, float measured_rad_s, float dt)
{
    if (fabs(ref_rad_s) < CTRL_REF_ZERO_EPSILON) {
        reset_left_controller();
        motors_set_left(0.0f);
        return;
    }

    e_l = ref_rad_s - measured_rad_s;

    // PI incremental:
    // u[k] = u[k-1] + Kp*(e[k] - e[k-1]) + Ki*Ts*e[k]
    const float delta_e = e_l - e_l_prev;
    const float delta_u_pwm = (CTRL_LEFT_KP_PWM * delta_e) +
                              (CTRL_LEFT_KI_PWM * dt * e_l);

    u_l_pwm = clamp_pwm(u_l_pwm + delta_u_pwm);
    u_l = pwm_to_percent(u_l_pwm);

    motors_set_left(u_l);

    e_l_prev = e_l;
}

static void update_right_incremental_pi(float ref_rad_s, float measured_rad_s, float dt)
{
    if (fabs(ref_rad_s) < CTRL_REF_ZERO_EPSILON) {
        reset_right_controller();
        motors_set_right(0.0f);
        return;
    }

    e_r = ref_rad_s - measured_rad_s;

    // PI incremental:
    // u[k] = u[k-1] + Kp*(e[k] - e[k-1]) + Ki*Ts*e[k]
    const float delta_e = e_r - e_r_prev;
    const float delta_u_pwm = (CTRL_RIGHT_KP_PWM * delta_e) +
                              (CTRL_RIGHT_KI_PWM * dt * e_r);

    u_r_pwm = clamp_pwm(u_r_pwm + delta_u_pwm);
    u_r = pwm_to_percent(u_r_pwm);

    motors_set_right(u_r);

    e_r_prev = e_r;
}

void control_update()
{
    const uint32_t now = millis();

    if ((now - g_last_control_ms) < CONTROL_PERIOD_MS) {
        return;
    }

    const float dt = (now - g_last_control_ms) / 1000.0f;
    g_last_control_ms = now;

    if (dt <= 0.0f) {
        return;
    }

    // Watchdog de referencias. Si dejan de llegar comandos,
    // las referencias vuelven a cero y los controladores se reinician.
    if ((now - g_last_left_ref_ms) > MOTOR_TIMEOUT_MS) {
        g_left_ref_rad_s = 0.0f;
    }

    if ((now - g_last_right_ref_ms) > MOTOR_TIMEOUT_MS) {
        g_right_ref_rad_s = 0.0f;
    }

    const float w_left  = encoders_get_left_rad_s();
    const float w_right = encoders_get_right_rad_s();

    update_left_incremental_pi(g_left_ref_rad_s, w_left, dt);
    update_right_incremental_pi(g_right_ref_rad_s, w_right, dt);
}
