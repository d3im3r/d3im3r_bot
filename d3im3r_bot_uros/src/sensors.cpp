#include "sensors.h"

#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L0X.h>

// ===== BNO055 =====

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ===== TOF =====

Adafruit_VL53L0X tof;

// ===== Variables exportadas =====

float imu_qx = 0;
float imu_qy = 0;
float imu_qz = 0;
float imu_qw = 1;

uint16_t calibration_status = 0;

float tof_distances[3] = {0,0,0};

// ===== Init =====

bool sensors_init()
{
    if(!bno.begin())
        return false;

    delay(1000);

    bno.setExtCrystalUse(true);

    if(!tof.begin())
        return false;

    return true;
}

// ===== Update =====

void sensors_update()
{
    imu::Quaternion quat = bno.getQuat();

    imu_qx = quat.x();
    imu_qy = quat.y();
    imu_qz = quat.z();
    imu_qw = quat.w();

    uint8_t sys,gyro,accel,mag;

    bno.getCalibration(&sys,&gyro,&accel,&mag);

    calibration_status = sys*1000 + gyro*100 + accel*10 + mag;

    VL53L0X_RangingMeasurementData_t measure;

    tof.rangingTest(&measure,false);

    if(measure.RangeStatus != 4)
        tof_distances[0] = measure.RangeMilliMeter / 1000.0;
}