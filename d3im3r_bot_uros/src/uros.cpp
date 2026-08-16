#include "uros.h"

#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/u_int16.h>
#include <geometry_msgs/msg/vector3.h>

char ssid[] = "Turtlebot";
char pass[] = "carlitosbot";

IPAddress agent_ip(192,168,1,100);
uint16_t agent_port = 8888;

rcl_node_t node;

rcl_publisher_t pub_imu;
rcl_publisher_t pub_cal;
rcl_publisher_t pub_tof;

rcl_allocator_t allocator;
rclc_support_t support;

sensor_msgs__msg__Imu imu_msg;
std_msgs__msg__UInt16 cal_msg;
geometry_msgs__msg__Vector3 tof_msg;

#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void uros_init()
{
    set_microros_wifi_transports(
        ssid,
        pass,
        agent_ip,
        agent_port);

    allocator = rcl_get_default_allocator();

    rclc_support_init(&support,0,NULL,&allocator);

    rclc_node_init_default(
        &node,
        "esp32_sensor_node",
        "",
        &support);

    rclc_publisher_init_default(
        &pub_imu,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs,msg,Imu),
        "/imu/data");

    rclc_publisher_init_default(
        &pub_cal,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,UInt16),
        "/bno055/calibration_status");

    rclc_publisher_init_default(
        &pub_tof,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Vector3),
        "/tof/distances");
}

void uros_publish_imu(float x,float y,float z,float w)
{
    imu_msg.orientation.x = x;
    imu_msg.orientation.y = y;
    imu_msg.orientation.z = z;
    imu_msg.orientation.w = w;

    RCSOFTCHECK(rcl_publish(&pub_imu,&imu_msg,NULL));
}

void uros_publish_calibration(uint16_t status)
{
    cal_msg.data = status;

    RCSOFTCHECK(rcl_publish(&pub_cal,&cal_msg,NULL));
}

void uros_publish_tof(float d1,float d2,float d3)
{
    tof_msg.x = d1;
    tof_msg.y = d2;
    tof_msg.z = d3;

    RCSOFTCHECK(rcl_publish(&pub_tof,&tof_msg,NULL));
}

void uros_spin()
{
    delay(5);
}