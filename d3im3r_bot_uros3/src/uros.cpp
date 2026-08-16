#include "uros.h"

#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/u_int16.h>

#include <builtin_interfaces/msg/time.h>

char ssid[] = "Turtlebot";
char pass[] = "carlitosbot";

IPAddress agent_ip(192,168,1,100);
uint16_t agent_port = 8888;

rcl_node_t node;

rcl_publisher_t pub_pose;
rcl_publisher_t pub_yaw;
rcl_publisher_t pub_cal;
rcl_publisher_t pub_tof;

rcl_allocator_t allocator;
rclc_support_t support;

geometry_msgs__msg__PoseStamped pose_msg;
std_msgs__msg__Float32 yaw_msg;
std_msgs__msg__UInt16 cal_msg;
geometry_msgs__msg__Vector3 tof_msg;

#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

// =======================================
// Timestamp helper
// =======================================

static void set_time(builtin_interfaces__msg__Time * t)
{
    int64_t now = rmw_uros_epoch_millis();

    t->sec = now / 1000;
    t->nanosec = (now % 1000) * 1000000;
}

// =======================================
// Init microROS
// =======================================

bool uros_init()
{
    set_microros_wifi_transports(
        ssid,
        pass,
        agent_ip,
        agent_port);

    allocator = rcl_get_default_allocator();

    if(rclc_support_init(&support,0,NULL,&allocator) != RCL_RET_OK)
        return false;

    if(rclc_node_init_default(
        &node,
        "esp32_sensor_node",
        "",
        &support) != RCL_RET_OK)
        return false;

    // Sync clock with ROS agent
    rmw_uros_sync_session(1000);

    rclc_publisher_init_default(
        &pub_pose,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,PoseStamped),
        "/imu/orientation");

    rclc_publisher_init_default(
        &pub_yaw,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs,msg,Float32),
        "/imu/yaw");

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

    return true;
}

// =======================================
// Orientation
// =======================================

void uros_publish_orientation(float qx,float qy,float qz,float qw)
{
    set_time(&pose_msg.header.stamp);

    pose_msg.header.frame_id.data = (char*)"imu_link";
    pose_msg.header.frame_id.size = strlen("imu_link");
    pose_msg.header.frame_id.capacity = pose_msg.header.frame_id.size + 1;

    pose_msg.pose.orientation.x = qx;
    pose_msg.pose.orientation.y = qy;
    pose_msg.pose.orientation.z = qz;
    pose_msg.pose.orientation.w = qw;

    RCSOFTCHECK(rcl_publish(&pub_pose,&pose_msg,NULL));
}

// =======================================
// Yaw
// =======================================

void uros_publish_yaw(float yaw)
{
    yaw_msg.data = yaw;

    RCSOFTCHECK(rcl_publish(&pub_yaw,&yaw_msg,NULL));
}

// =======================================
// Calibration
// =======================================

void uros_publish_calibration(uint16_t status)
{
    cal_msg.data = status;

    RCSOFTCHECK(rcl_publish(&pub_cal,&cal_msg,NULL));
}

// =======================================
// TOF
// =======================================

void uros_publish_tof(float c,float l,float r)
{
    tof_msg.x = c;
    tof_msg.y = l;
    tof_msg.z = r;

    RCSOFTCHECK(rcl_publish(&pub_tof,&tof_msg,NULL));
}

void uros_spin()
{
    delay(5);
}