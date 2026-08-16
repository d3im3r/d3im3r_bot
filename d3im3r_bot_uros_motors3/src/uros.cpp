#include "uros.h"
#include "app_config.h"
#include "encoders.h"
#include "motors.h"

#include <micro_ros_platformio.h>
#include <WiFi.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

// =====================================================
// Objetos micro-ROS
// =====================================================
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_timer_t timer;
static rclc_executor_t executor;

// =====================================================
// Publicadores ticks
// =====================================================
static rcl_publisher_t left_ticks_pub;
static rcl_publisher_t right_ticks_pub;

// =====================================================
// Publicadores velocidad angular
// =====================================================
static rcl_publisher_t left_rad_s_pub;
static rcl_publisher_t right_rad_s_pub;

// =====================================================
// Publicadores rpm
// =====================================================
static rcl_publisher_t left_rpm_pub;
static rcl_publisher_t right_rpm_pub;

// =====================================================
// Publicadores velocidad lineal
// =====================================================
static rcl_publisher_t left_m_s_pub;
static rcl_publisher_t right_m_s_pub;

// =====================================================
// Publicadores PWM aplicado
// =====================================================
static rcl_publisher_t left_pwm_applied_pub;
static rcl_publisher_t right_pwm_applied_pub;

// =====================================================
// Subscriptores
// =====================================================
static rcl_subscription_t load_left_sub;
static rcl_subscription_t load_right_sub;
static rcl_subscription_t cmd_left_pwm_sub;
static rcl_subscription_t cmd_right_pwm_sub;

// =====================================================
// Mensajes Int32
// =====================================================
static std_msgs__msg__Int32 left_ticks_msg;
static std_msgs__msg__Int32 right_ticks_msg;
static std_msgs__msg__Int32 load_left_msg;
static std_msgs__msg__Int32 load_right_msg;

// =====================================================
// Mensajes Float32
// =====================================================
static std_msgs__msg__Float32 left_rad_s_msg;
static std_msgs__msg__Float32 right_rad_s_msg;
static std_msgs__msg__Float32 left_rpm_msg;
static std_msgs__msg__Float32 right_rpm_msg;
static std_msgs__msg__Float32 left_m_s_msg;
static std_msgs__msg__Float32 right_m_s_msg;
static std_msgs__msg__Float32 left_pwm_applied_msg;
static std_msgs__msg__Float32 right_pwm_applied_msg;

static std_msgs__msg__Float32 cmd_left_pwm_msg;
static std_msgs__msg__Float32 cmd_right_pwm_msg;

// =====================================================
// WiFi
// =====================================================
static bool wifi_init()
{
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        if ((millis() - t0) > 15000) {
            return false;
        }
    }

    set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
    delay(500);

    return true;
}

// =====================================================
// Callbacks de subscripción
// =====================================================
static void load_left_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    encoders_set_left_ticks(msg->data);
}

static void load_right_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    encoders_set_right_ticks(msg->data);
}

static void cmd_left_pwm_callback(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    motors_set_left(msg->data);
}

static void cmd_right_pwm_callback(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    motors_set_right(msg->data);
}

// =====================================================
// Timer callback
// =====================================================
static void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;

    if (timer == NULL) {
        return;
    }

    encoders_update();
    motors_update();

    // -------------------------
    // Ticks
    // -------------------------
    left_ticks_msg.data  = encoders_get_left_ticks();
    right_ticks_msg.data = encoders_get_right_ticks();

    rcl_publish(&left_ticks_pub, &left_ticks_msg, NULL);
    rcl_publish(&right_ticks_pub, &right_ticks_msg, NULL);

    // -------------------------
    // Velocidades
    // -------------------------
    left_rad_s_msg.data  = encoders_get_left_rad_s();
    right_rad_s_msg.data = encoders_get_right_rad_s();

    left_rpm_msg.data  = encoders_get_left_rpm();
    right_rpm_msg.data = encoders_get_right_rpm();

    left_m_s_msg.data  = encoders_get_left_m_s();
    right_m_s_msg.data = encoders_get_right_m_s();

    rcl_publish(&left_rad_s_pub, &left_rad_s_msg, NULL);
    rcl_publish(&right_rad_s_pub, &right_rad_s_msg, NULL);

    rcl_publish(&left_rpm_pub, &left_rpm_msg, NULL);
    rcl_publish(&right_rpm_pub, &right_rpm_msg, NULL);

    rcl_publish(&left_m_s_pub, &left_m_s_msg, NULL);
    rcl_publish(&right_m_s_pub, &right_m_s_msg, NULL);

    // -------------------------
    // PWM aplicado
    // -------------------------
    left_pwm_applied_msg.data  = motors_get_left_applied();
    right_pwm_applied_msg.data = motors_get_right_applied();

    rcl_publish(&left_pwm_applied_pub, &left_pwm_applied_msg, NULL);
    rcl_publish(&right_pwm_applied_pub, &right_pwm_applied_msg, NULL);
}

// =====================================================
// Init micro-ROS
// =====================================================
bool uros_init()
{
    if (!wifi_init()) {
        return false;
    }

    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "d3im3r_base_node", "", &support) != RCL_RET_OK) return false;

    // =================================================
    // Publicadores
    // =================================================
    if (rclc_publisher_init_default(
            &left_ticks_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "left_ticks") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &right_ticks_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "right_ticks") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &left_rad_s_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "left_wheel_rad_s") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &right_rad_s_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "right_wheel_rad_s") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &left_rpm_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "left_wheel_rpm") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &right_rpm_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "right_wheel_rpm") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &left_m_s_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "left_wheel_m_s") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &right_m_s_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "right_wheel_m_s") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &left_pwm_applied_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "left_pwm_applied") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &right_pwm_applied_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "right_pwm_applied") != RCL_RET_OK) return false;

    // =================================================
    // Subscriptores
    // =================================================
    if (rclc_subscription_init_default(
            &load_left_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "load_count_left") != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(
            &load_right_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "load_count_right") != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(
            &cmd_left_pwm_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "cmd_left_pwm") != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(
            &cmd_right_pwm_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "cmd_right_pwm") != RCL_RET_OK) return false;

    // =================================================
    // Timer
    // =================================================
    if (rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(UROS_ENCODER_PUBLISH_PERIOD_MS),
            timer_callback) != RCL_RET_OK) return false;

    // =================================================
    // Executor
    // 1 timer + 4 subscripciones = 5 handles
    // =================================================
    if (rclc_executor_init(&executor, &support.context, 5, &allocator) != RCL_RET_OK) return false;

    if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK) return false;

    if (rclc_executor_add_subscription(
            &executor,
            &load_left_sub,
            &load_left_msg,
            &load_left_callback,
            ON_NEW_DATA) != RCL_RET_OK) return false;

    if (rclc_executor_add_subscription(
            &executor,
            &load_right_sub,
            &load_right_msg,
            &load_right_callback,
            ON_NEW_DATA) != RCL_RET_OK) return false;

    if (rclc_executor_add_subscription(
            &executor,
            &cmd_left_pwm_sub,
            &cmd_left_pwm_msg,
            &cmd_left_pwm_callback,
            ON_NEW_DATA) != RCL_RET_OK) return false;

    if (rclc_executor_add_subscription(
            &executor,
            &cmd_right_pwm_sub,
            &cmd_right_pwm_msg,
            &cmd_right_pwm_callback,
            ON_NEW_DATA) != RCL_RET_OK) return false;

    return true;
}

void uros_spin()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}