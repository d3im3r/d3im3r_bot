#include "sensors.h"
#include "app_config.h"

#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L0X.h>
#include <utility/imumaths.h>
#include <math.h>

static Adafruit_BNO055 bno(55, 0x28);

static Adafruit_VL53L0X sensor_center;
static Adafruit_VL53L0X sensor_right;
static Adafruit_VL53L0X sensor_left;

// Datos exportados
float imu_qx = 0.0f;
float imu_qy = 0.0f;
float imu_qz = 0.0f;
float imu_qw = 1.0f;

float imu_yaw = 0.0f;

uint16_t calibration_status = 0;

uint8_t cal_sys = 0;
uint8_t cal_gyro = 0;
uint8_t cal_accel = 0;
uint8_t cal_mag = 0;

float tof_distances[3] = {-1.0f, -1.0f, -1.0f};

bool bno_ok = false;
bool tof_center_ok = false;
bool tof_left_ok = false;
bool tof_right_ok = false;

bool sensors_init()
{
    Serial.println("[SENSORS] Iniciando BNO055...");
    bno_ok = bno.begin();

    if (!bno_ok) {
        Serial.println("[SENSORS] ERROR BNO055");
    } else {
        delay(1000);
        bno.setExtCrystalUse(true);
        Serial.println("[SENSORS] BNO055 OK");
    }

    Serial.println("[SENSORS] Inicializacion VL53L0X x3");

    pinMode(XSHUT_CENTER_PIN, OUTPUT);
    pinMode(XSHUT_RIGHT_PIN, OUTPUT);
    pinMode(XSHUT_LEFT_PIN, OUTPUT);

    digitalWrite(XSHUT_CENTER_PIN, LOW);
    digitalWrite(XSHUT_RIGHT_PIN, LOW);
    digitalWrite(XSHUT_LEFT_PIN, LOW);

    Serial.println("[TOF] Sensores en reset");
    delay(500);

    // DERECHO
    Serial.println("[TOF] Activando DERECHO...");
    digitalWrite(XSHUT_RIGHT_PIN, HIGH);
    delay(300);

    if (!sensor_right.begin()) {
        Serial.println("[TOF] ERROR sensor derecho");
        tof_right_ok = false;
    } else {
        sensor_right.setAddress(TOF_ADDR_RIGHT);
        tof_right_ok = true;
        Serial.print("[TOF] Direccion DERECHO -> 0x");
        Serial.println(TOF_ADDR_RIGHT, HEX);
    }

    delay(300);

    // IZQUIERDO
    Serial.println("[TOF] Activando IZQUIERDO...");
    digitalWrite(XSHUT_LEFT_PIN, HIGH);
    delay(300);

    if (!sensor_left.begin()) {
        Serial.println("[TOF] ERROR sensor izquierdo");
        tof_left_ok = false;
    } else {
        sensor_left.setAddress(TOF_ADDR_LEFT);
        tof_left_ok = true;
        Serial.print("[TOF] Direccion IZQUIERDO -> 0x");
        Serial.println(TOF_ADDR_LEFT, HEX);
    }

    delay(300);

    // CENTRAL
    Serial.println("[TOF] Activando CENTRAL...");
    digitalWrite(XSHUT_CENTER_PIN, HIGH);
    delay(300);

    if (!sensor_center.begin()) {
        Serial.println("[TOF] ERROR sensor central");
        tof_center_ok = false;
    } else {
        sensor_center.setAddress(TOF_ADDR_CENTER);
        tof_center_ok = true;
        Serial.print("[TOF] Direccion CENTRAL -> 0x");
        Serial.println(TOF_ADDR_CENTER, HEX);
    }

    delay(300);

    if (tof_center_ok) sensor_center.startRangeContinuous();
    if (tof_right_ok)  sensor_right.startRangeContinuous();
    if (tof_left_ok)   sensor_left.startRangeContinuous();

    Serial.println("[TOF] Sensores listos");

    return (bno_ok || tof_center_ok || tof_left_ok || tof_right_ok);
}

void sensors_update_imu()
{
    if (!bno_ok) return;

    imu::Quaternion q = bno.getQuat();

    imu_qx = q.x();
    imu_qy = q.y();
    imu_qz = q.z();
    imu_qw = q.w();

    imu_yaw = atan2(
        2.0f * (imu_qw * imu_qz + imu_qx * imu_qy),
        1.0f - 2.0f * (imu_qy * imu_qy + imu_qz * imu_qz)
    );

    bno.getCalibration(&cal_sys, &cal_gyro, &cal_accel, &cal_mag);

    calibration_status =
        static_cast<uint16_t>(cal_sys * 1000 + cal_gyro * 100 + cal_accel * 10 + cal_mag);
}

void sensors_update_tof()
{
    if (tof_center_ok && sensor_center.isRangeComplete()) {
        tof_distances[0] = sensor_center.readRange() / 1000.0f;
    }

    if (tof_left_ok && sensor_left.isRangeComplete()) {
        tof_distances[1] = sensor_left.readRange() / 1000.0f;
    }

    if (tof_right_ok && sensor_right.isRangeComplete()) {
        tof_distances[2] = sensor_right.readRange() / 1000.0f;
    }
}