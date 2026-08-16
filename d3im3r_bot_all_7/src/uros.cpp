#include "uros.h"
#include "app_config.h"
#include "encoders.h"
#include "control.h"
#include "sensors.h"
#include "kinematics.h"
#include "sync.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "network_config.h"

#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/twist.h>

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_timer_t timer;
static rclc_executor_t executor;

// =========================
// Publishers
// =========================
static rcl_publisher_t wheel_vel_pub;
static rcl_publisher_t left_control_pub;
static rcl_publisher_t right_control_pub;
static rcl_publisher_t imu_yaw_pub;
static rcl_publisher_t tof_distances_pub;

// =========================
// Subscribers
// =========================
static rcl_subscription_t wheel_refs_sub;
static rcl_subscription_t cmd_vel_sub;

// =========================
// Messages
// =========================
static geometry_msgs__msg__Vector3 wheel_vel_msg;
static std_msgs__msg__Float32 left_control_msg;
static std_msgs__msg__Float32 right_control_msg;
static std_msgs__msg__Float32 imu_yaw_msg;
static geometry_msgs__msg__Vector3 tof_distances_msg;
static geometry_msgs__msg__Vector3 wheel_refs_msg;
static geometry_msgs__msg__Twist cmd_vel_msg;

// =========================
// Callback referencias ruedas
// x = left
// y = right
// z = reservado
// =========================
static void wheel_refs_callback(const void * msgin)
{
    const geometry_msgs__msg__Vector3 * msg =
        (const geometry_msgs__msg__Vector3 *)msgin;

    control_set_left_ref((float)msg->x);
    control_set_right_ref((float)msg->y);
}

// =========================
// Callback cmd_vel
// linear.x  = v [m/s]
// angular.z = w [rad/s]
// =========================
static void cmd_vel_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg =
        (const geometry_msgs__msg__Twist *)msgin;

    const float v_m_s = (float)msg->linear.x;
    const float w_rad_s = (float)msg->angular.z;

    WheelSpeedRefs refs = kinematics_vw_to_wheels(v_m_s, w_rad_s);

    SyncOutput sync_out = sync_apply_hybrid_straight_correction(
        refs.left_rad_s,
        refs.right_rad_s,
        encoders_get_left_rad_s(),
        encoders_get_right_rad_s(),
        v_m_s,
        w_rad_s,
        imu_yaw
    );

    control_set_left_ref(sync_out.left_rad_s);
    control_set_right_ref(sync_out.right_rad_s);
}

// =========================
// Timer publicación
// =========================
static void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;

    if (timer == NULL) {
        return;
    }

    // Velocidades ruedas [rad/s]
    wheel_vel_msg.x = encoders_get_left_rad_s();
    wheel_vel_msg.y = encoders_get_right_rad_s();
    wheel_vel_msg.z = 0.0f;

    rcl_publish(&wheel_vel_pub, &wheel_vel_msg, NULL);

    // Acciones de control [%]
    left_control_msg.data = control_get_left_u();
    right_control_msg.data = control_get_right_u();

    rcl_publish(&left_control_pub, &left_control_msg, NULL);
    rcl_publish(&right_control_pub, &right_control_msg, NULL);

    // IMU yaw [rad]
    if (bno_ok) {
        imu_yaw_msg.data = imu_yaw;
        rcl_publish(&imu_yaw_pub, &imu_yaw_msg, NULL);
    }

    // Distancias ToF [m]
    tof_distances_msg.x = tof_center_ok ? tof_distances[0] : -1.0f;
    tof_distances_msg.y = tof_left_ok   ? tof_distances[1] : -1.0f;
    tof_distances_msg.z = tof_right_ok  ? tof_distances[2] : -1.0f;

    rcl_publish(&tof_distances_pub, &tof_distances_msg, NULL);
}

// =========================
// Init micro-ROS
// =========================
bool uros_init()
{
    if (!setupMicroRosNetwork()) {
        return false;
    }

    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;

    if (rclc_node_init_default(
            &node,
            "d3im3r_robot_node",
            "",
            &support
        ) != RCL_RET_OK) return false;

    // =========================
    // Publishers
    // =========================
    if (rclc_publisher_init_default(
            &wheel_vel_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
            "wheel_velocities_rad_s"
        ) != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &left_control_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "left_control_action"
        ) != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &right_control_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "right_control_action"
        ) != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &imu_yaw_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "imu_yaw_rad"
        ) != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(
            &tof_distances_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
            "tof_distances_m"
        ) != RCL_RET_OK) return false;

    // =========================
    // Subscriber referencias directas de rueda
    // =========================
    if (rclc_subscription_init_default(
            &wheel_refs_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
            "wheel_refs_rad_s"
        ) != RCL_RET_OK) return false;

    // =========================
    // Subscriber cmd_vel
    // =========================
    if (rclc_subscription_init_default(
            &cmd_vel_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "cmd_vel"
        ) != RCL_RET_OK) return false;

    // =========================
    // Timer
    // =========================
    if (rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(UROS_PUBLISH_PERIOD_MS),
            timer_callback
        ) != RCL_RET_OK) return false;

    // 1 timer + 2 subscriptions = 3 handles
    if (rclc_executor_init(
            &executor,
            &support.context,
            3,
            &allocator
        ) != RCL_RET_OK) return false;

    if (rclc_executor_add_timer(
            &executor,
            &timer
        ) != RCL_RET_OK) return false;

    if (rclc_executor_add_subscription(
            &executor,
            &wheel_refs_sub,
            &wheel_refs_msg,
            &wheel_refs_callback,
            ON_NEW_DATA
        ) != RCL_RET_OK) return false;

    if (rclc_executor_add_subscription(
            &executor,
            &cmd_vel_sub,
            &cmd_vel_msg,
            &cmd_vel_callback,
            ON_NEW_DATA
        ) != RCL_RET_OK) return false;

    return true;
}

// =========================
// Spin
// =========================
void uros_spin()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}