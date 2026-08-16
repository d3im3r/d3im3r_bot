#include "uros.h"
#include "app_config.h"
#include "encoders.h"
#include "control.h"
#include "sensors.h"

#include <micro_ros_platformio.h>
#include <WiFi.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_timer_t timer;
static rclc_executor_t executor;

// Publishers
static rcl_publisher_t left_rad_s_pub;
static rcl_publisher_t right_rad_s_pub;
static rcl_publisher_t left_control_pub;
static rcl_publisher_t right_control_pub;
static rcl_publisher_t imu_yaw_pub;
static rcl_publisher_t tof_center_pub;
static rcl_publisher_t tof_left_pub;
static rcl_publisher_t tof_right_pub;

// Subscribers
static rcl_subscription_t left_ref_sub;
static rcl_subscription_t right_ref_sub;

// Messages
static std_msgs__msg__Float32 left_rad_s_msg;
static std_msgs__msg__Float32 right_rad_s_msg;
static std_msgs__msg__Float32 left_control_msg;
static std_msgs__msg__Float32 right_control_msg;
static std_msgs__msg__Float32 imu_yaw_msg;
static std_msgs__msg__Float32 tof_center_msg;
static std_msgs__msg__Float32 tof_left_msg;
static std_msgs__msg__Float32 tof_right_msg;
static std_msgs__msg__Float32 left_ref_msg;
static std_msgs__msg__Float32 right_ref_msg;

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

static void left_ref_callback(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    control_set_left_ref(msg->data);
}

static void right_ref_callback(const void * msgin)
{
    const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
    control_set_right_ref(msg->data);
}

static void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;

    if (timer == NULL) {
        return;
    }

    left_rad_s_msg.data = encoders_get_left_rad_s();
    right_rad_s_msg.data = encoders_get_right_rad_s();

    left_control_msg.data = control_get_left_u();
    right_control_msg.data = control_get_right_u();

    rcl_publish(&left_rad_s_pub, &left_rad_s_msg, NULL);
    rcl_publish(&right_rad_s_pub, &right_rad_s_msg, NULL);
    rcl_publish(&left_control_pub, &left_control_msg, NULL);
    rcl_publish(&right_control_pub, &right_control_msg, NULL);
}

bool uros_init()
{
    if (!wifi_init()) {
        return false;
    }

    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "d3im3r_robot_node", "", &support) != RCL_RET_OK) return false;

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
            &left_control_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "left_control_action") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &right_control_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "right_control_action") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &imu_yaw_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "imu_yaw_rad") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &tof_center_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "tof_center_m") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &tof_left_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "tof_left_m") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &tof_right_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "tof_right_m") != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(
            &left_ref_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "left_wheel_ref_rad_s") != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(
            &right_ref_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "right_wheel_ref_rad_s") != RCL_RET_OK) return false;

    if (rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(UROS_PUBLISH_PERIOD_MS),
            timer_callback) != RCL_RET_OK) return false;

    // 1 timer + 2 subs = 3 handles
    if (rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK) return false;

    if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK) return false;

    if (rclc_executor_add_subscription(
            &executor,
            &left_ref_sub,
            &left_ref_msg,
            &left_ref_callback,
            ON_NEW_DATA) != RCL_RET_OK) return false;

    if (rclc_executor_add_subscription(
            &executor,
            &right_ref_sub,
            &right_ref_msg,
            &right_ref_callback,
            ON_NEW_DATA) != RCL_RET_OK) return false;

    return true;
}

void uros_spin()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}