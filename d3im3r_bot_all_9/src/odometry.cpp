#include "odometry.h"
#include "app_config.h"

#include <math.h>

// =====================================================
// Estado interno
// =====================================================
static OdomState2D g_odom;

static bool g_initialized = false;

static uint32_t g_last_update_ms = 0;

// Offset de yaw para que la odometría pueda iniciar en theta = 0
static bool g_imu_yaw_offset_locked = false;
static float g_imu_yaw_offset_rad = 0.0f;

// =====================================================
// Utilidades
// =====================================================
static float wrap_to_pi_odom(float angle)
{
    while (angle > PI) {
        angle -= 2.0f * PI;
    }

    while (angle < -PI) {
        angle += 2.0f * PI;
    }

    return angle;
}

static float clamp_dt(float dt_s)
{
    if (dt_s < ODOM_MIN_DT_S) {
        return 0.0f;
    }

    if (dt_s > ODOM_MAX_DT_S) {
        return ODOM_MAX_DT_S;
    }

    return dt_s;
}

// =====================================================
// Inicialización
// =====================================================
void odometry_init()
{
    odometry_reset();
}

// =====================================================
// Reset
// =====================================================
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

// =====================================================
// Actualización de odometría
// =====================================================
void odometry_update(
    float left_meas_rad_s,
    float right_meas_rad_s,
    float imu_yaw_rad,
    bool imu_available
)
{
    const uint32_t now = millis();

    // Calcular velocidad real del cuerpo con cinemática directa
    const BodyTwist2D measured_twist = kinematics_forward(
        left_meas_rad_s,
        right_meas_rad_s
    );

    g_odom.twist = measured_twist;

    // Primera actualización: solo sincroniza tiempo y orientación
    if (!g_initialized) {
        g_last_update_ms = now;
        g_initialized = true;

        if (ODOM_USE_IMU_YAW && imu_available) {
            if (ODOM_ZERO_YAW_ON_INIT) {
                g_imu_yaw_offset_rad = imu_yaw_rad;
            } else {
                g_imu_yaw_offset_rad = 0.0f;
            }

            g_imu_yaw_offset_locked = true;

            g_odom.pose.theta_rad = wrap_to_pi_odom(
                imu_yaw_rad - g_imu_yaw_offset_rad
            );
        }

        return;
    }

    float dt_s = (now - g_last_update_ms) / 1000.0f;
    g_last_update_ms = now;

    dt_s = clamp_dt(dt_s);

    if (dt_s <= 0.0f) {
        return;
    }

    // =================================================
    // Orientación
    // =================================================
    //
    // Modo 1:
    // Usar yaw de IMU como orientación de odometría.
    //
    // Modo 2:
    // Integrar orientación desde encoders.
    //
    // =================================================
    float theta_for_integration = g_odom.pose.theta_rad;

    if (ODOM_USE_IMU_YAW && imu_available) {
        if (!g_imu_yaw_offset_locked) {
            if (ODOM_ZERO_YAW_ON_INIT) {
                g_imu_yaw_offset_rad = imu_yaw_rad;
            } else {
                g_imu_yaw_offset_rad = 0.0f;
            }

            g_imu_yaw_offset_locked = true;
        }

        g_odom.pose.theta_rad = wrap_to_pi_odom(
            imu_yaw_rad - g_imu_yaw_offset_rad
        );

        theta_for_integration = g_odom.pose.theta_rad;
    } else {
        g_odom.pose.theta_rad = wrap_to_pi_odom(
            g_odom.pose.theta_rad + measured_twist.w_rad_s * dt_s
        );

        theta_for_integration = g_odom.pose.theta_rad;
    }

    // =================================================
    // Integración de posición
    // =================================================
    //
    // x[k+1] = x[k] + v*cos(theta)*dt
    // y[k+1] = y[k] + v*sin(theta)*dt
    //
    // =================================================
    g_odom.pose.x_m += measured_twist.v_m_s *
                       cos(theta_for_integration) *
                       dt_s;

    g_odom.pose.y_m += measured_twist.v_m_s *
                       sin(theta_for_integration) *
                       dt_s;

#if ODOM_DEBUG_SERIAL
    Serial.print("[ODOM] x=");
    Serial.print(g_odom.pose.x_m, 4);
    Serial.print(" y=");
    Serial.print(g_odom.pose.y_m, 4);
    Serial.print(" th=");
    Serial.print(g_odom.pose.theta_rad, 4);
    Serial.print(" v=");
    Serial.print(g_odom.twist.v_m_s, 4);
    Serial.print(" w=");
    Serial.println(g_odom.twist.w_rad_s, 4);
#endif
}

// =====================================================
// Getters
// =====================================================
OdomState2D odometry_get_state()
{
    return g_odom;
}

OdomPose2D odometry_get_pose()
{
    return g_odom.pose;
}

BodyTwist2D odometry_get_twist()
{
    return g_odom.twist;
}

float odometry_get_x_m()
{
    return g_odom.pose.x_m;
}

float odometry_get_y_m()
{
    return g_odom.pose.y_m;
}

float odometry_get_theta_rad()
{
    return g_odom.pose.theta_rad;
}

float odometry_get_v_m_s()
{
    return g_odom.twist.v_m_s;
}

float odometry_get_w_rad_s()
{
    return g_odom.twist.w_rad_s;
}