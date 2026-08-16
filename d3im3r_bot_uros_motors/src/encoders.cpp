#include "encoders.h"
#include "app_config.h"

#include <Arduino.h>

// =====================================================
// Contadores
// =====================================================
static volatile int32_t g_left_ticks = 0;
static volatile int32_t g_right_ticks = 0;

// =====================================================
// Mutex ESP32
// =====================================================
static portMUX_TYPE mux_left  = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE mux_right = portMUX_INITIALIZER_UNLOCKED;

// =====================================================
// ISR encoder izquierdo
// =====================================================
void IRAM_ATTR left_encoder_isr()
{
    bool a = digitalRead(ENCODER_LEFT_A_PIN);
    bool b = digitalRead(ENCODER_LEFT_B_PIN);

    portENTER_CRITICAL_ISR(&mux_left);
    if (a != b) {
        g_left_ticks += (ENCODER_LEFT_INVERTED ? -1 : 1);
    } else {
        g_left_ticks += (ENCODER_LEFT_INVERTED ? 1 : -1);
    }
    portEXIT_CRITICAL_ISR(&mux_left);
}

// =====================================================
// ISR encoder derecho
// =====================================================
void IRAM_ATTR right_encoder_isr()
{
    bool a = digitalRead(ENCODER_RIGHT_A_PIN);
    bool b = digitalRead(ENCODER_RIGHT_B_PIN);

    portENTER_CRITICAL_ISR(&mux_right);
    if (a != b) {
        g_right_ticks += (ENCODER_RIGHT_INVERTED ? -1 : 1);
    } else {
        g_right_ticks += (ENCODER_RIGHT_INVERTED ? 1 : -1);
    }
    portEXIT_CRITICAL_ISR(&mux_right);
}

// =====================================================
// Init
// =====================================================
bool encoders_init()
{
    pinMode(ENCODER_LEFT_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B_PIN, INPUT_PULLUP);

    pinMode(ENCODER_RIGHT_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B_PIN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A_PIN), left_encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A_PIN), right_encoder_isr, CHANGE);

    g_left_ticks = 0;
    g_right_ticks = 0;

    return true;
}

// =====================================================
// Getters seguros
// =====================================================
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

// =====================================================
// Setters seguros
// =====================================================
void encoders_set_left_ticks(int32_t value)
{
    portENTER_CRITICAL(&mux_left);
    g_left_ticks = value;
    portEXIT_CRITICAL(&mux_left);
}

void encoders_set_right_ticks(int32_t value)
{
    portENTER_CRITICAL(&mux_right);
    g_right_ticks = value;
    portEXIT_CRITICAL(&mux_right);
}

void encoders_reset()
{
    encoders_set_left_ticks(0);
    encoders_set_right_ticks(0);
}