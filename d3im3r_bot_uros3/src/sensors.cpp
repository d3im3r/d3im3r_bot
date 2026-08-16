#include "sensors.h"

#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L0X.h>
#include <utility/imumaths.h>
#include <math.h>

#define XSHUT_CENTER 5
#define XSHUT_RIGHT  23
#define XSHUT_LEFT   14

#define ADDR_CENTER 0x30
#define ADDR_RIGHT  0x31
#define ADDR_LEFT   0x32

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
        return false;
    }

    delay(1000);
    bno.setExtCrystalUse(true);
    Serial.println("[SENSORS] BNO055 OK");

    Serial.println("[SENSORS] Inicializacion VL53L0X x3");

    pinMode(XSHUT_CENTER, OUTPUT);
    pinMode(XSHUT_RIGHT, OUTPUT);
    pinMode(XSHUT_LEFT, OUTPUT);

    digitalWrite(XSHUT_CENTER, LOW);
    digitalWrite(XSHUT_RIGHT, LOW);
    digitalWrite(XSHUT_LEFT, LOW);

    Serial.println("[TOF] Sensores en reset");
    delay(500);

    // DERECHO
    Serial.println("[TOF] Activando DERECHO...");
    digitalWrite(XSHUT_RIGHT, HIGH);
    delay(300);

    if (!sensor_right.begin()) {
        Serial.println("[TOF] ERROR sensor derecho");
        tof_right_ok = false;
    } else {
        sensor_right.setAddress(ADDR_RIGHT);
        tof_right_ok = true;
        Serial.println("[TOF] Direccion DERECHO -> 0x31");
    }

    delay(300);

    // IZQUIERDO
    Serial.println("[TOF] Activando IZQUIERDO...");
    digitalWrite(XSHUT_LEFT, HIGH);
    delay(300);

    if (!sensor_left.begin()) {
        Serial.println("[TOF] ERROR sensor izquierdo");
        tof_left_ok = false;
    } else {
        sensor_left.setAddress(ADDR_LEFT);
        tof_left_ok = true;
        Serial.println("[TOF] Direccion IZQUIERDO -> 0x32");
    }

    delay(300);

    // CENTRAL
    Serial.println("[TOF] Activando CENTRAL...");
    digitalWrite(XSHUT_CENTER, HIGH);
    delay(300);

    if (!sensor_center.begin()) {
        Serial.println("[TOF] ERROR sensor central");
        tof_center_ok = false;
    } else {
        sensor_center.setAddress(ADDR_CENTER);
        tof_center_ok = true;
        Serial.println("[TOF] Direccion CENTRAL -> 0x30");
    }

    delay(300);

    if (tof_center_ok) sensor_center.startRangeContinuous();
    if (tof_right_ok)  sensor_right.startRangeContinuous();
    if (tof_left_ok)   sensor_left.startRangeContinuous();

    Serial.println("[TOF] Sensores listos");

    return true;
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