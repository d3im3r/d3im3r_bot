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

static float e_l = 0.0f;
static float e_l_prev = 0.0f;
static float u_l = 0.0f;

static float e_r = 0.0f;
static float e_r_prev = 0.0f;
static float u_r = 0.0f;

static float clamp_percent(float u)
{
    if (u > CTRL_OUTPUT_LIMIT_PERCENT) return CTRL_OUTPUT_LIMIT_PERCENT;
    if (u < -CTRL_OUTPUT_LIMIT_PERCENT) return -CTRL_OUTPUT_LIMIT_PERCENT;
    return u;
}

static void reset_left_controller()
{
    e_l = 0.0f;
    e_l_prev = 0.0f;
    u_l = 0.0f;
}

static void reset_right_controller()
{
    e_r = 0.0f;
    e_r_prev = 0.0f;
    u_r = 0.0f;
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

float control_get_left_ref()   { return g_left_ref_rad_s; }
float control_get_right_ref()  { return g_right_ref_rad_s; }

float control_get_left_u()     { return u_l; }
float control_get_right_u()    { return u_r; }

float control_get_left_error() { return e_l; }
float control_get_right_error(){ return e_r; }

void control_update()
{
    uint32_t now = millis();

    if ((now - g_last_control_ms) < CONTROL_PERIOD_MS) {
        return;
    }



    const float w_left  = encoders_get_left_rad_s();
    const float w_right = encoders_get_right_rad_s();

    // RUEDA IZQUIERDA
    if (fabs(g_left_ref_rad_s) < CTRL_REF_ZERO_EPSILON) {
        reset_left_controller();
        motors_set_left(0.0f);
    } else {
        e_l = g_left_ref_rad_s - w_left;

    #if USE_INCREMENTAL_PI
        u_l = u_l + CTRL_LEFT_Q0 * e_l + CTRL_LEFT_Q1 * e_l_prev;
    #else
        u_l = CTRL_LEFT_KP * e_l;
    #endif

        e_l_prev = e_l;
        u_l = clamp_percent(u_l);
        motors_set_left(u_l);
    }

    // RUEDA DERECHA
    if (fabs(g_right_ref_rad_s) < CTRL_REF_ZERO_EPSILON) {
        reset_right_controller();
        motors_set_right(0.0f);
    } else {
        e_r = g_right_ref_rad_s - w_right;

    #if USE_INCREMENTAL_PI
        u_r = u_r + CTRL_RIGHT_Q0 * e_r + CTRL_RIGHT_Q1 * e_r_prev;
    #else
        u_r = CTRL_RIGHT_KP * e_r;
    #endif

        e_r_prev = e_r;
        u_r = clamp_percent(u_r);
        motors_set_right(u_r);
    }
}