#include "uros.h"
#include "app_config.h"
#include "encoders.h"
#include "control.h"
#include "sensors.h"
#include "kinematics.h"
#include "sync.h"
#include "odometry.h"
#include "safety.h"
#include "network_config.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/twist.h>

// Objetos micro-ROS
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_timer_t timer;
static rclc_executor_t executor;

// Publishers
static rcl_publisher_t wheel_vel_pub;
static rcl_publisher_t left_control_pub;
static rcl_publisher_t right_control_pub;
static rcl_publisher_t imu_yaw_pub;
static rcl_publisher_t tof_distances_pub;
static rcl_publisher_t odom_pose_pub;
static rcl_publisher_t odom_twist_pub;
static rcl_publisher_t safety_status_pub;

// Subscribers
static rcl_subscription_t wheel_refs_sub;
static rcl_subscription_t cmd_vel_sub;

// Messages
static geometry_msgs__msg__Vector3 wheel_vel_msg;
static std_msgs__msg__Float32 left_control_msg;
static std_msgs__msg__Float32 right_control_msg;
static std_msgs__msg__Float32 imu_yaw_msg;
static geometry_msgs__msg__Vector3 tof_distances_msg;
static geometry_msgs__msg__Vector3 odom_pose_msg;
static geometry_msgs__msg__Vector3 odom_twist_msg;
static geometry_msgs__msg__Vector3 safety_status_msg;
static geometry_msgs__msg__Vector3 wheel_refs_msg;
static geometry_msgs__msg__Twist cmd_vel_msg;

typedef enum {
    MOTION_MODE_NONE = 0,
    MOTION_MODE_WHEEL_REFS,
    MOTION_MODE_CMD_VEL
} MotionMode;

static MotionMode g_motion_mode = MOTION_MODE_NONE;
static float g_cmd_v_m_s = 0.0f;
static float g_cmd_w_rad_s = 0.0f;

static uint32_t g_last_cmd_vel_ms = 0;
static uint32_t g_last_wheel_refs_ms = 0;

static void stop_motion_command()
{
    g_cmd_v_m_s = 0.0f;
    g_cmd_w_rad_s = 0.0f;
    g_motion_mode = MOTION_MODE_NONE;
    sync_reset();
    control_set_left_ref(0.0f);
    control_set_right_ref(0.0f);
}

static void wheel_refs_callback(const void * msgin)
{
    const auto * msg = (const geometry_msgs__msg__Vector3 *)msgin;
    g_motion_mode = MOTION_MODE_WHEEL_REFS;
    g_last_wheel_refs_ms = millis();
    g_cmd_v_m_s = 0.0f;
    g_cmd_w_rad_s = 0.0f;
    sync_reset();

    control_set_left_ref((float)msg->x);
    control_set_right_ref((float)msg->y);
}

static void cmd_vel_callback(const void * msgin)
{
    const auto * msg = (const geometry_msgs__msg__Twist *)msgin;
    g_cmd_v_m_s = (float)msg->linear.x;
    g_cmd_w_rad_s = (float)msg->angular.z;
    g_motion_mode = MOTION_MODE_CMD_VEL;
    g_last_cmd_vel_ms = millis();
}

void uros_update_motion_command()
{
    const uint32_t now = millis();

    if (g_motion_mode == MOTION_MODE_CMD_VEL) {
        if ((now - g_last_cmd_vel_ms) > MOTOR_TIMEOUT_MS) {
            stop_motion_command();
            return;
        }

        SafeCmdVel safe_cmd = safety_filter_cmd_vel(
            g_cmd_v_m_s, g_cmd_w_rad_s,
            tof_distances[0], tof_distances[1], tof_distances[2],
            tof_center_ok, tof_left_ok, tof_right_ok
        );

        WheelSpeedRefs refs = kinematics_inverse(safe_cmd.v_m_s, safe_cmd.w_rad_s);

        SyncOutput sync_out = sync_apply_hybrid_straight_correction(
            refs.left_rad_s, refs.right_rad_s,
            encoders_get_left_rad_s(), encoders_get_right_rad_s(),
            safe_cmd.v_m_s, safe_cmd.w_rad_s,
            imu_yaw, bno_ok
        );

        control_set_left_ref(sync_out.left_rad_s);
        control_set_right_ref(sync_out.right_rad_s);
        return;
    }

    if (g_motion_mode == MOTION_MODE_WHEEL_REFS) {
        if ((now - g_last_wheel_refs_ms) > MOTOR_TIMEOUT_MS) {
            g_motion_mode = MOTION_MODE_NONE;
            sync_reset();
        }
        return;
    }

    sync_reset();
}

static void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;
    if (timer == NULL) return;

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

    // Odometría pose
    OdomPose2D pose = odometry_get_pose();
    odom_pose_msg.x = pose.x_m;
    odom_pose_msg.y = pose.y_m;
    odom_pose_msg.z = pose.theta_rad;
    rcl_publish(&odom_pose_pub, &odom_pose_msg, NULL);

    // Odometría twist
    BodyTwist2D twist = odometry_get_twist();
    odom_twist_msg.x = twist.v_m_s;
    odom_twist_msg.y = twist.w_rad_s;
    odom_twist_msg.z = 0.0f;
    rcl_publish(&odom_twist_pub, &odom_twist_msg, NULL);

    // Estado de seguridad
    SafeCmdVel safe_cmd = safety_get_last_cmd();
    safety_status_msg.x = safe_cmd.limited ? 1.0f : 0.0f;
    safety_status_msg.y = safe_cmd.emergency_stop ? 1.0f : 0.0f;
    safety_status_msg.z = (float)safe_cmd.reason;
    rcl_publish(&safety_status_pub, &safety_status_msg, NULL);
}

bool uros_init()
{
    if (!setupMicroRosNetwork()) return false;

    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, UROS_NODE_NAME, UROS_NODE_NAMESPACE, &support) != RCL_RET_OK) return false;

    // Publishers
    const rosidl_message_type_support_t * vec3_ts  = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3);
    const rosidl_message_type_support_t * float_ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32);
    const rosidl_message_type_support_t * twist_ts = ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist);

    if (rclc_publisher_init_default(&wheel_vel_pub, &node, vec3_ts, TOPIC_WHEEL_VELOCITIES_RAD_S) != RCL_RET_OK ||
        rclc_publisher_init_default(&left_control_pub, &node, float_ts, TOPIC_LEFT_CONTROL_ACTION) != RCL_RET_OK ||
        rclc_publisher_init_default(&right_control_pub, &node, float_ts, TOPIC_RIGHT_CONTROL_ACTION) != RCL_RET_OK ||
        rclc_publisher_init_default(&imu_yaw_pub, &node, float_ts, TOPIC_IMU_YAW_RAD) != RCL_RET_OK ||
        rclc_publisher_init_default(&tof_distances_pub, &node, vec3_ts, TOPIC_TOF_DISTANCES_M) != RCL_RET_OK ||
        rclc_publisher_init_default(&odom_pose_pub, &node, vec3_ts, TOPIC_ODOM_POSE) != RCL_RET_OK ||
        rclc_publisher_init_default(&odom_twist_pub, &node, vec3_ts, TOPIC_ODOM_TWIST) != RCL_RET_OK ||
        rclc_publisher_init_default(&safety_status_pub, &node, vec3_ts, TOPIC_SAFETY_STATUS) != RCL_RET_OK) {
        return false;
    }

    // Subscribers
    if (rclc_subscription_init_default(&wheel_refs_sub, &node, vec3_ts, TOPIC_WHEEL_REFS_RAD_S) != RCL_RET_OK ||
        rclc_subscription_init_default(&cmd_vel_sub, &node, twist_ts, TOPIC_CMD_VEL) != RCL_RET_OK) {
        return false;
    }

    // Timer & Executor
    if (rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(UROS_PUBLISH_PERIOD_MS), timer_callback) != RCL_RET_OK) return false;
    if (rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK) return false;

    if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK ||
        rclc_executor_add_subscription(&executor, &wheel_refs_sub, &wheel_refs_msg, &wheel_refs_callback, ON_NEW_DATA) != RCL_RET_OK ||
        rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA) != RCL_RET_OK) {
        return false;
    }

    return true;
}

void uros_spin()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}