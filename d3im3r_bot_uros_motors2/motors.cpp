#include "motors.h"
#include "app_config.h"
#include <math.h>

static float last_pwm_left  = 0.0f;
static float last_pwm_right = 0.0f;

static uint32_t last_left_cmd_ms  = 0;
static uint32_t last_right_cmd_ms = 0;

static inline uint32_t duty_to_ledc(float duty_percent) {
    return (uint32_t)((duty_percent / 100.0f) * ((1 << PWM_RES) - 1));
}

static float clamp_and_apply_deadband(float cmd) {
    float mag = fabs(cmd);

    if (mag < 0.001f) {
        return 0.0f;
    }

    if (mag < PWM_DEADBAND_PERCENT) {
        mag = PWM_DEADBAND_PERCENT;
    }

    if (mag > PWM_LIMIT_PERCENT) {
        mag = PWM_LIMIT_PERCENT;
    }

    return (cmd < 0.0f) ? -mag : mag;
}

static void apply_motor_hw(
    float cmd,
    int pin_in1, int pin_in2,
    uint8_t pwm_channel,
    bool forward_in1_high, bool forward_in2_high,
    float &last_pwm_store
) {
    bool reverse = (cmd < 0.0f);
    float mag = fabs(cmd);

    if (mag < 0.001f) {
        ledcWrite(pwm_channel, 0);
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, LOW);
        last_pwm_store = 0.0f;
        return;
    }

    bool in1 = forward_in1_high;
    bool in2 = forward_in2_high;

    if (!reverse) {
        in1 = !in1;
        in2 = !in2;
    }

    ledcWrite(pwm_channel, 0);
    delayMicroseconds(500);

    digitalWrite(pin_in1, in1 ? HIGH : LOW);
    digitalWrite(pin_in2, in2 ? HIGH : LOW);

    ledcWrite(pwm_channel, duty_to_ledc(mag));
    last_pwm_store = reverse ? -mag : mag;
}

void motors_init() {
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_CH_LEFT,  PWM_FREQ, PWM_RES);

    ledcAttachPin(pwmA, PWM_CH_RIGHT);
    ledcAttachPin(pwmB, PWM_CH_LEFT);

    motors_stop_all();

    last_left_cmd_ms = millis();
    last_right_cmd_ms = millis();
}

void motors_set_left(float cmd_percent) {
    float cmd = clamp_and_apply_deadband(cmd_percent);

    // LEFT: forward = BIN1 LOW, BIN2 HIGH
    apply_motor_hw(cmd, BIN1, BIN2, PWM_CH_LEFT, false, true, last_pwm_left);

    last_left_cmd_ms = millis();
}

void motors_set_right(float cmd_percent) {
    float cmd = clamp_and_apply_deadband(cmd_percent);

    // RIGHT: forward = AIN1 HIGH, AIN2 LOW
    apply_motor_hw(cmd, AIN1, AIN2, PWM_CH_RIGHT, true, false, last_pwm_right);

    last_right_cmd_ms = millis();
}

float motors_get_left_applied() {
    return last_pwm_left;
}

float motors_get_right_applied() {
    return last_pwm_right;
}

void motors_stop_all() {
    ledcWrite(PWM_CH_RIGHT, 0);
    ledcWrite(PWM_CH_LEFT, 0);

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);

    last_pwm_left = 0.0f;
    last_pwm_right = 0.0f;
}

void motors_update() {
    uint32_t now = millis();

    if ((now - last_left_cmd_ms) > MOTOR_TIMEOUT_MS) {
        motors_set_left(0.0f);
    }

    if ((now - last_right_cmd_ms) > MOTOR_TIMEOUT_MS) {
        motors_set_right(0.0f);
    }
}