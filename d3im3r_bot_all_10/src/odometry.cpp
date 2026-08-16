#include "odometry.h"
#include "app_config.h"

#include <math.h>

static OdomState2D g_odom;
static bool g_initialized = false;
static uint32_t g_last_update_ms = 0;

static bool g_imu_yaw_offset_locked = false;
static float g_imu_yaw_offset_rad = 0.0f;

static inline float wrap_to_pi_odom(float angle)
{
    while (angle > PI)  angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

static inline float clamp_dt(float dt_s)
{
    if (dt_s < ODOM_MIN_DT_S) return 0.0f;
    if (dt_s > ODOM_MAX_DT_S) return ODOM_MAX_DT_S;
    return dt_s;
}

void odometry_init()
{
    odometry_reset();
}

void odometry_reset()
{
    g_odom.pose.x_m = 0.0f;
    g_odom.pose.y_m = 0.0f;
    g_odom.pose.theta_rad = 0.0f;
    g_odom.twist.v_m_s = 0.0f;
    g_odom.twist.w_rad_s = 0.0f;

    g_initialized = false;
    g_last_update_ms = millis();
    g_imu_yaw_offset_locked = false;
    g_imu_yaw_offset_rad = 0.0f;
}

void odometry_update(
    float left_meas_rad_s,
    float right_meas_rad_s,
    float imu_yaw_rad,
    bool imu_available
)
{
    const uint32_t now = millis();
    const BodyTwist2D measured_twist = kinematics_forward(left_meas_rad_s, right_meas_rad_s);
    g_odom.twist = measured_twist;

    if (!g_initialized) {
        g_last_update_ms = now;
        g_initialized = true;

        if (ODOM_USE_IMU_YAW && imu_available) {
            g_imu_yaw_offset_rad = ODOM_ZERO_YAW_ON_INIT ? imu_yaw_rad : 0.0f;
            g_imu_yaw_offset_locked = true;
            g_odom.pose.theta_rad = wrap_to_pi_odom(imu_yaw_rad - g_imu_yaw_offset_rad);
        }
        return;
    }

    float dt_s = clamp_dt((now - g_last_update_ms) / 1000.0f);
    g_last_update_ms = now;

    if (dt_s <= 0.0f) {
        return;
    }

    // Orientación
    if (ODOM_USE_IMU_YAW && imu_available) {
        if (!g_imu_yaw_offset_locked) {
            g_imu_yaw_offset_rad = ODOM_ZERO_YAW_ON_INIT ? imu_yaw_rad : 0.0f;
            g_imu_yaw_offset_locked = true;
        }
        g_odom.pose.theta_rad = wrap_to_pi_odom(imu_yaw_rad - g_imu_yaw_offset_rad);
    } else {
        g_odom.pose.theta_rad = wrap_to_pi_odom(g_odom.pose.theta_rad + measured_twist.w_rad_s * dt_s);
    }

    // Integración de posición
    const float theta = g_odom.pose.theta_rad;
    g_odom.pose.x_m += measured_twist.v_m_s * cos(theta) * dt_s;
    g_odom.pose.y_m += measured_twist.v_m_s * sin(theta) * dt_s;
}

OdomState2D odometry_get_state()   { return g_odom; }
OdomPose2D odometry_get_pose()     { return g_odom.pose; }
BodyTwist2D odometry_get_twist()   { return g_odom.twist; }

float odometry_get_x_m()           { return g_odom.pose.x_m; }
float odometry_get_y_m()           { return g_odom.pose.y_m; }
float odometry_get_theta_rad()     { return g_odom.pose.theta_rad; }
float odometry_get_v_m_s()         { return g_odom.twist.v_m_s; }
float odometry_get_w_rad_s()       { return g_odom.twist.w_rad_s; }