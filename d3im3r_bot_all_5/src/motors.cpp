#include "motors.h"
#include "app_config.h"

#include <Arduino.h>
#include <math.h>

static float last_pwm_left  = 0.0f;
static float last_pwm_right = 0.0f;

static uint32_t last_left_cmd_ms  = 0;
static uint32_t last_right_cmd_ms = 0;

static inline uint32_t duty_to_ledc(float duty_percent)
{
    if (duty_percent < 0.0f) duty_percent = 0.0f;
    if (duty_percent > 100.0f) duty_percent = 100.0f;

    return (uint32_t)((duty_percent / 100.0f) * ((1 << PWM_RES_BITS) - 1));
}

static float clamp_and_apply_deadband(float cmd)
{
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

static void motor_write(
    float cmd,
    int pin_in1,
    int pin_in2,
    uint8_t pwm_channel,
    bool forward_in1,
    bool forward_in2,
    float &last_pwm_store
)
{
    float mag = fabs(cmd);

    if (mag < 0.001f) {
        ledcWrite(pwm_channel, 0);
        digitalWrite(pin_in1, LOW);
        digitalWrite(pin_in2, LOW);
        last_pwm_store = 0.0f;
        return;
    }

    bool dir_forward = (cmd >= 0.0f);

    bool in1 = forward_in1;
    bool in2 = forward_in2;

    if (!dir_forward) {
        in1 = !forward_in1;
        in2 = !forward_in2;
    }

    ledcWrite(pwm_channel, 0);
    delayMicroseconds(300);

    digitalWrite(pin_in1, in1 ? HIGH : LOW);
    digitalWrite(pin_in2, in2 ? HIGH : LOW);

    ledcWrite(pwm_channel, duty_to_ledc(mag));
    last_pwm_store = dir_forward ? mag : -mag;
}

bool motors_init()
{
    pinMode(MOTOR_LEFT_IN1_PIN, OUTPUT);
    pinMode(MOTOR_LEFT_IN2_PIN, OUTPUT);
    pinMode(MOTOR_RIGHT_IN1_PIN, OUTPUT);
    pinMode(MOTOR_RIGHT_IN2_PIN, OUTPUT);

    ledcSetup(PWM_CH_LEFT, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcSetup(PWM_CH_RIGHT, PWM_FREQ_HZ, PWM_RES_BITS);

    ledcAttachPin(MOTOR_LEFT_PWM_PIN, PWM_CH_LEFT);
    ledcAttachPin(MOTOR_RIGHT_PWM_PIN, PWM_CH_RIGHT);

    motors_stop_all();

    last_left_cmd_ms = millis();
    last_right_cmd_ms = millis();

    return true;
}

void motors_set_left(float cmd_percent)
{
    float cmd = clamp_and_apply_deadband(cmd_percent);

    // LEFT: avance = IN1 LOW, IN2 HIGH
    motor_write(
        cmd,
        MOTOR_LEFT_IN1_PIN,
        MOTOR_LEFT_IN2_PIN,
        PWM_CH_LEFT,
        true,
        false,
        last_pwm_left
    );

    last_left_cmd_ms = millis();
}

void motors_set_right(float cmd_percent)
{
    float cmd = clamp_and_apply_deadband(cmd_percent);

    // RIGHT: avance = IN1 HIGH, IN2 LOW
    motor_write(
        cmd,
        MOTOR_RIGHT_IN1_PIN,
        MOTOR_RIGHT_IN2_PIN,
        PWM_CH_RIGHT,
        false,
        true,
        last_pwm_right
    );

    last_right_cmd_ms = millis();
}

float motors_get_left_applied()
{
    return last_pwm_left;
}

float motors_get_right_applied()
{
    return last_pwm_right;
}

void motors_stop_all()
{
    ledcWrite(PWM_CH_LEFT, 0);
    ledcWrite(PWM_CH_RIGHT, 0);

    digitalWrite(MOTOR_LEFT_IN1_PIN, LOW);
    digitalWrite(MOTOR_LEFT_IN2_PIN, LOW);
    digitalWrite(MOTOR_RIGHT_IN1_PIN, LOW);
    digitalWrite(MOTOR_RIGHT_IN2_PIN, LOW);

    last_pwm_left = 0.0f;
    last_pwm_right = 0.0f;
}

void motors_update()
{
    uint32_t now = millis();

    if ((now - last_left_cmd_ms) > MOTOR_TIMEOUT_MS) {
        motors_set_left(0.0f);
    }

    if ((now - last_right_cmd_ms) > MOTOR_TIMEOUT_MS) {
        motors_set_right(0.0f);
    }
}