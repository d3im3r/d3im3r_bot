#include "sensors.h"

#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L0X.h>
#include <utility/imumaths.h>

// ================================
// Pines XSHUT
// ================================
#define XSHUT_CENTER 5
#define XSHUT_RIGHT  23
#define XSHUT_LEFT   14

// ================================
// Direcciones nuevas
// ================================
#define ADDR_CENTER 0x30
#define ADDR_RIGHT  0x31
#define ADDR_LEFT   0x32

// ================================
// Sensores
// ================================
static Adafruit_BNO055 bno_sensor(55, 0x28);

static Adafruit_VL53L0X sensor_center;
static Adafruit_VL53L0X sensor_right;
static Adafruit_VL53L0X sensor_left;

// ================================
// Estado
// ================================
bool bno_ok = false;
bool tof_center_ok = false;
bool tof_left_ok = false;
bool tof_right_ok = false;

// ================================
// Datos exportados
// ================================
float imu_qx = 0.0f;
float imu_qy = 0.0f;
float imu_qz = 0.0f;
float imu_qw = 1.0f;

uint16_t calibration_status = 0;

float tof_distances[3] = {-1.0f,-1.0f,-1.0f};

// ================================
// Inicialización sensores
// ================================
bool sensors_init()
{
    Serial.println("[SENSORS] Iniciando BNO055...");

    bno_ok = bno_sensor.begin();

    if (bno_ok)
    {
        delay(1000);
        bno_sensor.setExtCrystalUse(true);
        Serial.println("[SENSORS] BNO055 OK");
    }
    else
    {
        Serial.println("[SENSORS] ERROR BNO055");
    }

    Serial.println("[SENSORS] Inicializacion VL53L0X x3");

    Wire.begin(21,22);
    Wire.setClock(100000);
    delay(200);

    pinMode(XSHUT_CENTER, OUTPUT);
    pinMode(XSHUT_RIGHT, OUTPUT);
    pinMode(XSHUT_LEFT, OUTPUT);

    // Reset sensores
    digitalWrite(XSHUT_CENTER, LOW);
    digitalWrite(XSHUT_RIGHT, LOW);
    digitalWrite(XSHUT_LEFT, LOW);

    delay(500);

    // ======================
    // DERECHO
    // ======================

    Serial.println("[TOF] Activando DERECHO");

    digitalWrite(XSHUT_RIGHT, HIGH);
    delay(300);

    if (!sensor_right.begin())
    {
        Serial.println("[TOF] ERROR derecho");
        tof_right_ok = false;
    }
    else
    {
        sensor_right.setAddress(ADDR_RIGHT);
        tof_right_ok = true;
        Serial.println("[TOF] DERECHO -> 0x31");
    }

    delay(300);

    // ======================
    // IZQUIERDO
    // ======================

    Serial.println("[TOF] Activando IZQUIERDO");

    digitalWrite(XSHUT_LEFT, HIGH);
    delay(300);

    if (!sensor_left.begin())
    {
        Serial.println("[TOF] ERROR izquierdo");
        tof_left_ok = false;
    }
    else
    {
        sensor_left.setAddress(ADDR_LEFT);
        tof_left_ok = true;
        Serial.println("[TOF] IZQUIERDO -> 0x32");
    }

    delay(300);

    // ======================
    // CENTRAL
    // ======================

    Serial.println("[TOF] Activando CENTRAL");

    digitalWrite(XSHUT_CENTER, HIGH);
    delay(300);

    if (!sensor_center.begin())
    {
        Serial.println("[TOF] ERROR central");
        tof_center_ok = false;
    }
    else
    {
        sensor_center.setAddress(ADDR_CENTER);
        tof_center_ok = true;
        Serial.println("[TOF] CENTRAL -> 0x30");
    }

    delay(300);

    if (tof_center_ok) sensor_center.startRangeContinuous();
    if (tof_right_ok)  sensor_right.startRangeContinuous();
    if (tof_left_ok)   sensor_left.startRangeContinuous();

    Serial.println("[TOF] Sensores listos");

    return bno_ok;
}

// ================================
// Lectura sensores
// ================================
void sensors_update()
{
    // ===== IMU =====
    if (bno_ok)
    {
        imu::Quaternion quat = bno_sensor.getQuat();

        imu_qx = quat.x();
        imu_qy = quat.y();
        imu_qz = quat.z();
        imu_qw = quat.w();

        uint8_t sys,gyro,accel,mag;
        bno_sensor.getCalibration(&sys,&gyro,&accel,&mag);

        calibration_status =
            sys*1000 + gyro*100 + accel*10 + mag;
    }

    // ===== TOF CENTRAL =====
    if (tof_center_ok && sensor_center.isRangeComplete())
    {
        tof_distances[0] = sensor_center.readRange() / 1000.0;
    }

    // ===== TOF IZQUIERDO =====
    if (tof_left_ok && sensor_left.isRangeComplete())
    {
        tof_distances[1] = sensor_left.readRange() / 1000.0;
    }

    // ===== TOF DERECHO =====
    if (tof_right_ok && sensor_right.isRangeComplete())
    {
        tof_distances[2] = sensor_right.readRange() / 1000.0;
    }
}