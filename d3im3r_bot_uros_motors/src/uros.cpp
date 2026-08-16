#include "uros.h"
#include "app_config.h"
#include "encoders.h"

#include <micro_ros_platformio.h>
#include <WiFi.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// =====================================================
// Objetos micro-ROS
// =====================================================
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_timer_t timer;
static rclc_executor_t executor;

static rcl_publisher_t left_ticks_pub;
static rcl_publisher_t right_ticks_pub;

static rcl_subscription_t load_left_sub;
static rcl_subscription_t load_right_sub;

static std_msgs__msg__Int32 left_ticks_msg;
static std_msgs__msg__Int32 right_ticks_msg;

static std_msgs__msg__Int32 load_left_msg;
static std_msgs__msg__Int32 load_right_msg;

// =====================================================
// Callbacks
// =====================================================
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;

    if (timer == NULL) {
        return;
    }

    left_ticks_msg.data = encoders_get_left_ticks();
    right_ticks_msg.data = encoders_get_right_ticks();

    rcl_publish(&left_ticks_pub, &left_ticks_msg, NULL);
    rcl_publish(&right_ticks_pub, &right_ticks_msg, NULL);
}

void load_left_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

    encoders_set_left_ticks(msg->data);
    left_ticks_msg.data = msg->data;
    rcl_publish(&left_ticks_pub, &left_ticks_msg, NULL);
}

void load_right_callback(const void * msgin)
{
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

    encoders_set_right_ticks(msg->data);
    right_ticks_msg.data = msg->data;
    rcl_publish(&right_ticks_pub, &right_ticks_msg, NULL);
}

// =====================================================
// WiFi
// =====================================================
static bool wifi_init()
{
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        if (millis() - t0 > 15000) {
            return false;
        }
    }

    set_microros_wifi_transports(WIFI_SSID, WIFI_PASS, AGENT_IP, AGENT_PORT);
    delay(500);

    return true;
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

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        return false;
    }

    if (rclc_node_init_default(&node, "d3im3r_encoders", "", &support) != RCL_RET_OK) {
        return false;
    }

    if (rclc_publisher_init_default(
            &left_ticks_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "left_ticks") != RCL_RET_OK) {
        return false;
    }

    if (rclc_publisher_init_default(
            &right_ticks_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "right_ticks") != RCL_RET_OK) {
        return false;
    }

    if (rclc_subscription_init_default(
            &load_left_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "load_count_left") != RCL_RET_OK) {
        return false;
    }

    if (rclc_subscription_init_default(
            &load_right_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "load_count_right") != RCL_RET_OK) {
        return false;
    }

    if (rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(UROS_ENCODER_PUBLISH_PERIOD_MS),
            timer_callback) != RCL_RET_OK) {
        return false;
    }

    if (rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK) {
        return false;
    }

    if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK) {
        return false;
    }

    if (rclc_executor_add_subscription(
            &executor,
            &load_left_sub,
            &load_left_msg,
            &load_left_callback,
            ON_NEW_DATA) != RCL_RET_OK) {
        return false;
    }

    if (rclc_executor_add_subscription(
            &executor,
            &load_right_sub,
            &load_right_msg,
            &load_right_callback,
            ON_NEW_DATA) != RCL_RET_OK) {
        return false;
    }

    return true;
}

void uros_spin()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}