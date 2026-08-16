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
static float i_l = 0.0f;
static float u_l = 0.0f;      // porcentaje aplicado al motor
static float u_l_pwm = 0.0f;  // salida interna en PWM [-255,255]

// Estados rueda derecha
static float e_r = 0.0f;
static float e_r_prev = 0.0f;
static float i_r = 0.0f;
static float u_r = 0.0f;      // porcentaje aplicado al motor
static float u_r_pwm = 0.0f;  // salida interna en PWM [-255,255]

static float clamp_pwm(float u)
{
    if (u > CTRL_PWM_MAX) return CTRL_PWM_MAX;
    if (u < -CTRL_PWM_MAX) return -CTRL_PWM_MAX;
    return u;
}

static float clamp_integral(float i)
{
    if (i > CTRL_INTEGRAL_LIMIT) return CTRL_INTEGRAL_LIMIT;
    if (i < -CTRL_INTEGRAL_LIMIT) return -CTRL_INTEGRAL_LIMIT;
    return i;
}

static float pwm_to_percent(float pwm)
{
    return (pwm / CTRL_PWM_MAX) * 100.0f;
}

static void reset_left_controller()
{
    e_l = 0.0f;
    e_l_prev = 0.0f;
    i_l = 0.0f;
    u_l = 0.0f;
    u_l_pwm = 0.0f;
}

static void reset_right_controller()
{
    e_r = 0.0f;
    e_r_prev = 0.0f;
    i_r = 0.0f;
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

void control_update()
{
    uint32_t now = millis();

    if ((now - g_last_control_ms) < CONTROL_PERIOD_MS) {
        return;
    }

    float dt = (now - g_last_control_ms) / 1000.0f;
    g_last_control_ms = now;

    if (dt <= 0.0f) {
        return;
    }

    // Watchdog de referencias
    if ((now - g_last_left_ref_ms) > MOTOR_TIMEOUT_MS) {
        g_left_ref_rad_s = 0.0f;
    }

    if ((now - g_last_right_ref_ms) > MOTOR_TIMEOUT_MS) {
        g_right_ref_rad_s = 0.0f;
    }

    const float w_left  = encoders_get_left_rad_s();
    const float w_right = encoders_get_right_rad_s();

    // =========================
    // RUEDA IZQUIERDA
    // =========================
    if (fabs(g_left_ref_rad_s) < CTRL_REF_ZERO_EPSILON) {
        reset_left_controller();
        motors_set_left(0.0f);
    } else {
        e_l = g_left_ref_rad_s - w_left;

        // Integral acumulada
        i_l += e_l * dt;
        i_l = clamp_integral(i_l);

        // PI no incremental en PWM
        u_l_pwm = (CTRL_LEFT_KP_PWM * e_l) + (CTRL_LEFT_KI_PWM * i_l);
        u_l_pwm = clamp_pwm(u_l_pwm);

        // Anti-windup simple: si satura y el error empuja en la misma dirección,
        // se deshace la última integración.
        if ((u_l_pwm >= CTRL_PWM_MAX && e_l > 0.0f) ||
            (u_l_pwm <= -CTRL_PWM_MAX && e_l < 0.0f)) {
            i_l -= e_l * dt;
            i_l = clamp_integral(i_l);
            u_l_pwm = (CTRL_LEFT_KP_PWM * e_l) + (CTRL_LEFT_KI_PWM * i_l);
            u_l_pwm = clamp_pwm(u_l_pwm);
        }

        u_l = pwm_to_percent(u_l_pwm);
        motors_set_left(u_l);

        e_l_prev = e_l;
    }

    // =========================
    // RUEDA DERECHA
    // =========================
    if (fabs(g_right_ref_rad_s) < CTRL_REF_ZERO_EPSILON) {
        reset_right_controller();
        motors_set_right(0.0f);
    } else {
        e_r = g_right_ref_rad_s - w_right;

        // Integral acumulada
        i_r += e_r * dt;
        i_r = clamp_integral(i_r);

        // PI no incremental en PWM
        u_r_pwm = (CTRL_RIGHT_KP_PWM * e_r) + (CTRL_RIGHT_KI_PWM * i_r);
        u_r_pwm = clamp_pwm(u_r_pwm);

        // Anti-windup simple
        if ((u_r_pwm >= CTRL_PWM_MAX && e_r > 0.0f) ||
            (u_r_pwm <= -CTRL_PWM_MAX && e_r < 0.0f)) {
            i_r -= e_r * dt;
            i_r = clamp_integral(i_r);
            u_r_pwm = (CTRL_RIGHT_KP_PWM * e_r) + (CTRL_RIGHT_KI_PWM * i_r);
            u_r_pwm = clamp_pwm(u_r_pwm);
        }

        u_r = pwm_to_percent(u_r_pwm);
        motors_set_right(u_r);

        e_r_prev = e_r;
    }
}