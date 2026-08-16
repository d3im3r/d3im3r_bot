#include "control.h"
#include "app_config.h"
#include "encoders.h"
#include "motors.h"

#include <Arduino.h>
#include <math.h>

struct PIController {
    float kp;
    float ki;
    float ref_rad_s;
    float e;
    float e_prev;
    float u_pwm;
    float u_percent;
    uint32_t last_ref_ms;

    void reset() {
        e = 0.0f;
        e_prev = 0.0f;
        u_pwm = 0.0f;
        u_percent = 0.0f;
    }
};

static PIController ctrl_left  = { CTRL_LEFT_KP_PWM,  CTRL_LEFT_KI_PWM,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0 };
static PIController ctrl_right = { CTRL_RIGHT_KP_PWM, CTRL_RIGHT_KI_PWM, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0 };

static uint32_t g_last_control_ms = 0;

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

static void update_pi_controller(PIController &ctrl, float measured_rad_s, float dt, void (*motor_set)(float))
{
    if (fabs(ctrl.ref_rad_s) < CTRL_REF_ZERO_EPSILON) {
        ctrl.reset();
        motor_set(0.0f);
        return;
    }

    ctrl.e = ctrl.ref_rad_s - measured_rad_s;

    // PI incremental: u[k] = u[k-1] + Kp*(e[k] - e[k-1]) + Ki*Ts*e[k]
    const float delta_e = ctrl.e - ctrl.e_prev;
    const float delta_u_pwm = (ctrl.kp * delta_e) + (ctrl.ki * dt * ctrl.e);

    ctrl.u_pwm = clamp_pwm(ctrl.u_pwm + delta_u_pwm);
    ctrl.u_percent = pwm_to_percent(ctrl.u_pwm);

    motor_set(ctrl.u_percent);
    ctrl.e_prev = ctrl.e;
}

bool control_init()
{
    ctrl_left.kp = CTRL_LEFT_KP_PWM;
    ctrl_left.ki = CTRL_LEFT_KI_PWM;
    ctrl_left.ref_rad_s = 0.0f;
    ctrl_left.reset();

    ctrl_right.kp = CTRL_RIGHT_KP_PWM;
    ctrl_right.ki = CTRL_RIGHT_KI_PWM;
    ctrl_right.ref_rad_s = 0.0f;
    ctrl_right.reset();

    g_last_control_ms = millis();
    ctrl_left.last_ref_ms = g_last_control_ms;
    ctrl_right.last_ref_ms = g_last_control_ms;

    return true;
}

void control_set_left_ref(float ref_rad_s)
{
    ctrl_left.ref_rad_s = ref_rad_s;
    ctrl_left.last_ref_ms = millis();
}

void control_set_right_ref(float ref_rad_s)
{
    ctrl_right.ref_rad_s = ref_rad_s;
    ctrl_right.last_ref_ms = millis();
}

float control_get_left_ref()    { return ctrl_left.ref_rad_s; }
float control_get_right_ref()   { return ctrl_right.ref_rad_s; }

float control_get_left_u()      { return ctrl_left.u_percent; }
float control_get_right_u()     { return ctrl_right.u_percent; }

float control_get_left_error()  { return ctrl_left.e; }
float control_get_right_error() { return ctrl_right.e; }

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

    // Watchdog de referencias
    if ((now - ctrl_left.last_ref_ms) > MOTOR_TIMEOUT_MS) {
        ctrl_left.ref_rad_s = 0.0f;
    }

    if ((now - ctrl_right.last_ref_ms) > MOTOR_TIMEOUT_MS) {
        ctrl_right.ref_rad_s = 0.0f;
    }

    const float w_left  = encoders_get_left_rad_s();
    const float w_right = encoders_get_right_rad_s();

    update_pi_controller(ctrl_left, w_left, dt, motors_set_left);
    update_pi_controller(ctrl_right, w_right, dt, motors_set_right);
}
