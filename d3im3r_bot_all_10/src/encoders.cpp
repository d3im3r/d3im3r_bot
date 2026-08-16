#include "encoders.h"
#include "app_config.h"

#include <Arduino.h>
#include <math.h>

static volatile int32_t g_left_ticks = 0;
static volatile int32_t g_right_ticks = 0;

static int32_t g_last_left_ticks = 0;
static int32_t g_last_right_ticks = 0;
static uint32_t g_last_update_ms = 0;

static float g_left_rad_s = 0.0f;
static float g_right_rad_s = 0.0f;

static portMUX_TYPE mux_left  = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE mux_right = portMUX_INITIALIZER_UNLOCKED;

static inline void IRAM_ATTR process_encoder_isr(int pinA, int pinB, volatile int32_t &ticks, bool inverted, portMUX_TYPE &mux)
{
    bool a = digitalRead(pinA);
    bool b = digitalRead(pinB);
    int step = (a != b) ? (inverted ? -1 : 1) : (inverted ? 1 : -1);

    portENTER_CRITICAL_ISR(&mux);
    ticks += step;
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR left_encoder_isr()
{
    process_encoder_isr(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN, g_left_ticks, ENCODER_LEFT_INVERTED, mux_left);
}

void IRAM_ATTR right_encoder_isr()
{
    process_encoder_isr(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN, g_right_ticks, ENCODER_RIGHT_INVERTED, mux_right);
}

bool encoders_init()
{
    pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B_PIN, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), left_encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A_PIN), right_encoder_isr, CHANGE);

    encoders_reset();
    return true;
}

void encoders_update()
{
    uint32_t now = millis();
    uint32_t dt_ms = now - g_last_update_ms;

    if (dt_ms < ENCODER_VEL_UPDATE_PERIOD_MS) {
        return;
    }

    float dt = dt_ms / 1000.0f;
    if (dt <= 0.0f) {
        return;
    }

    int32_t left_now, right_now;

    portENTER_CRITICAL(&mux_left);
    left_now = g_left_ticks;
    portEXIT_CRITICAL(&mux_left);

    portENTER_CRITICAL(&mux_right);
    right_now = g_right_ticks;
    portEXIT_CRITICAL(&mux_right);

    int32_t delta_left = left_now - g_last_left_ticks;
    int32_t delta_right = right_now - g_last_right_ticks;

    g_last_left_ticks = left_now;
    g_last_right_ticks = right_now;
    g_last_update_ms = now;

    g_left_rad_s  = (delta_left * 2.0f * PI) / (ENCODER_PPR * dt);
    g_right_rad_s = (delta_right * 2.0f * PI) / (ENCODER_PPR * dt);
}

int32_t encoders_get_left_ticks()
{
    int32_t value;
    portENTER_CRITICAL(&mux_left);
    value = g_left_ticks;
    portEXIT_CRITICAL(&mux_left);
    return value;
}

int32_t encoders_get_right_ticks()
{
    int32_t value;
    portENTER_CRITICAL(&mux_right);
    value = g_right_ticks;
    portEXIT_CRITICAL(&mux_right);
    return value;
}

void encoders_set_left_ticks(int32_t value)
{
    portENTER_CRITICAL(&mux_left);
    g_left_ticks = value;
    portEXIT_CRITICAL(&mux_left);
    g_last_left_ticks = value;
}

void encoders_set_right_ticks(int32_t value)
{
    portENTER_CRITICAL(&mux_right);
    g_right_ticks = value;
    portEXIT_CRITICAL(&mux_right);
    g_last_right_ticks = value;
}

void encoders_reset()
{
    encoders_set_left_ticks(0);
    encoders_set_right_ticks(0);
    g_left_rad_s = 0.0f;
    g_right_rad_s = 0.0f;
    g_last_update_ms = millis();
}

float encoders_get_left_rad_s()  { return g_left_rad_s; }
float encoders_get_left_rpm()    { return (g_left_rad_s * 60.0f) / (2.0f * PI); }
float encoders_get_left_m_s()    { return g_left_rad_s * WHEEL_RADIUS_M; }

float encoders_get_right_rad_s() { return g_right_rad_s; }
float encoders_get_right_rpm()   { return (g_right_rad_s * 60.0f) / (2.0f * PI); }
float encoders_get_right_m_s()   { return g_right_rad_s * WHEEL_RADIUS_M; }