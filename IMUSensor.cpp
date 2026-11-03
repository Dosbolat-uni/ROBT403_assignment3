#include "IMUSensor.h"
#include <iostream>
#include <unistd.h>

namespace exploringRPi {

short IMUSensor::combineRegisters(unsigned char msb, unsigned char lsb) {
    return (short)((msb << 8) | lsb); //[cite: 4]
}

IMUSensor::IMUSensor(unsigned int bus, unsigned int devAddress)
    : I2CDevice(bus, devAddress) {
    
    // LSM6DS33 Datasheet Exact Sensitivities:
    // Gyro @ +/-500 dps  --> 17.50 mdps/LSB = 0.0175 dps/LSB
    gyroScale = 0.0175f * (M_PI / 180.0f); // Converts LSB to rad/s
    
    // Accel @ +/-2g      --> 0.061 mg/LSB = 0.000061 g/LSB
    accelScale = 0.000061f;                // Converts LSB to g
    
    magScale = 1.0f;
}

IMUSensor::~IMUSensor() {}

int IMUSensor::initSensors() {
    // CTRL1_XL (0x10): Accel = 104Hz, +/-2g   (was 0x20 = 26 Hz)
    if (this->writeRegister(0x10, 0x40) != 0) return 1;

    // CTRL2_G (0x11): Gyro = 104Hz, +/-500dps  (was 0x24 = 26 Hz)
    if (this->writeRegister(0x11, 0x44) != 0) return 1;

    usleep(50000); // Allow sensor outputs to stabilize
    calibrateGyro();

    return 0;
}

void IMUSensor::calibrateGyro() {
    std::cout << "Calibrating Gyroscope offset... KEEP IMU STILL!" << std::endl;
    long sumX = 0, sumY = 0, sumZ = 0;
    const int samples = 200;

    for (int i = 0; i < samples; ++i) {
        readRawData();
        sumX += rawGyro[0];
        sumY += rawGyro[1];
        sumZ += rawGyro[2];
        usleep(5000); // 5ms delay between samples
    }

    gyroOffsetX = (float)sumX / samples;
    gyroOffsetY = (float)sumY / samples;
    gyroOffsetZ = (float)sumZ / samples;

    std::cout << "Calibration Complete. Offsets (LSB) - X: " 
              << gyroOffsetX << " Y: " << gyroOffsetY << " Z: " << gyroOffsetZ << std::endl;
}

int IMUSensor::readRawData() {
    // Read Gyro OUTX_L_G (0x22) to OUTZ_H_G (0x27)
    unsigned char* gyroData = this->readRegisters(6, 0x22); //[cite: 11]
    if (gyroData == NULL) return -1;

    rawGyro[0] = combineRegisters(gyroData[1], gyroData[0]);
    rawGyro[1] = combineRegisters(gyroData[3], gyroData[2]);
    rawGyro[2] = combineRegisters(gyroData[5], gyroData[4]);
    delete[] gyroData;

    // Read Accel OUTX_L_XL (0x28) to OUTZ_H_XL (0x2D)
    unsigned char* accelData = this->readRegisters(6, 0x28);
    if (accelData == NULL) return -1;

    rawAccel[0] = combineRegisters(accelData[1], accelData[0]);
    rawAccel[1] = combineRegisters(accelData[3], accelData[2]);
    rawAccel[2] = combineRegisters(accelData[5], accelData[4]);
    delete[] accelData;

    rawMag[0] = 0; rawMag[1] = 0; rawMag[2] = 0;
    return 0;
}

void IMUSensor::convertToPhysicalValues() {
    // Accelerometer physical units
    ax = rawAccel[0] * accelScale;
    ay = rawAccel[2] * accelScale; // Physical Z (rawAccel[2]) -> GL Y
    az = rawAccel[1] * accelScale; // Physical Y (rawAccel[1]) -> GL Z

    // Gyroscope physical units (remap raw data AND corresponding offsets)
    gx = (rawGyro[0] - gyroOffsetX) * gyroScale;
    gy = (rawGyro[2] - gyroOffsetZ) * gyroScale; // Physical Z (rawGyro[2]) -> GL Y
    gz = (rawGyro[1] - gyroOffsetY) * gyroScale; // Physical Y (rawGyro[1]) -> GL Z

    mx = rawMag[0] * magScale;
    my = rawMag[2] * magScale;
    mz = rawMag[1] * magScale;
}

Quaternion IMUSensor::updateAndGetQuaternion() {
    readRawData();
    convertToPhysicalValues();

    if (mx == 0.0f && my == 0.0f && mz == 0.0f) {
        MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az); //[cite: 9]
    } else {
        MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz); //[cite: 9]
    }

    Quaternion q;
    q.q0 = q0;
    q.q1 = q1;
    q.q2 = q2;
    q.q3 = q3;
    return q;
}

} /* namespace exploringRPi */
