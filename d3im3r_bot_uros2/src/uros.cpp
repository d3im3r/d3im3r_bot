#include "uros.h"

#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/u_int16.h>
#include <geometry_msgs/msg/vector3.h>

// ===== Config simple =====
static char ssid[] = "Turtlebot";
static char pass[] = "carlitosbot";
static IPAddress agent_ip(192, 168, 1, 100);
static uint16_t agent_port = 8888;

// ===== Estado =====
bool uros_transport_ok = false;
bool uros_support_ok = false;
bool uros_node_ok = false;
bool uros_publishers_ok = false;

// ===== micro-ROS =====
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;

static rcl_publisher_t pub_imu;
static rcl_publisher_t pub_cal;
static rcl_publisher_t pub_tof;

static sensor_msgs__msg__Imu imu_msg;
static std_msgs__msg__UInt16 cal_msg;
static geometry_msgs__msg__Vector3 tof_msg;

#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

bool uros_init()
{
    set_microros_wifi_transports(ssid, pass, agent_ip, agent_port);
    uros_transport_ok = true;

    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        return false;
    }
    uros_support_ok = true;

    if (rclc_node_init_default(&node, "esp32_sensor_node", "", &support) != RCL_RET_OK) {
        return false;
    }
    uros_node_ok = true;

    if (rclc_publisher_init_default(
            &pub_imu,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "/imu/data") != RCL_RET_OK) {
        return false;
    }

    if (rclc_publisher_init_default(
            &pub_cal,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
            "/bno055/calibration_status") != RCL_RET_OK) {
        return false;
    }

    if (rclc_publisher_init_default(
            &pub_tof,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
            "/tof/distances") != RCL_RET_OK) {
        return false;
    }

    uros_publishers_ok = true;
    return true;
}

void uros_publish_imu(float x, float y, float z, float w)
{
    if (!uros_publishers_ok) return;

    imu_msg.orientation.x = x;
    imu_msg.orientation.y = y;
    imu_msg.orientation.z = z;
    imu_msg.orientation.w = w;

    // Simplificado: marcamos covarianza no disponible
    imu_msg.orientation_covariance[0] = -1.0;
    imu_msg.angular_velocity_covariance[0] = -1.0;
    imu_msg.linear_acceleration_covariance[0] = -1.0;

    RCSOFTCHECK(rcl_publish(&pub_imu, &imu_msg, NULL));
}

void uros_publish_calibration(uint16_t status)
{
    if (!uros_publishers_ok) return;

    cal_msg.data = status;
    RCSOFTCHECK(rcl_publish(&pub_cal, &cal_msg, NULL));
}

void uros_publish_tof(float d1, float d2, float d3)
{
    if (!uros_publishers_ok) return;

    tof_msg.x = d1;
    tof_msg.y = d2;
    tof_msg.z = d3;

    RCSOFTCHECK(rcl_publish(&pub_tof, &tof_msg, NULL));
}

void uros_spin()
{
    delay(5);
}