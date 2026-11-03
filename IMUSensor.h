#ifndef IMUSENSOR_H_
#define IMUSENSOR_H_

#include "I2CDevice.h"
#include <cmath>

extern "C" {
    #include "MadgwickAHRS.h"
}

namespace exploringRPi {

struct Quaternion {
    float q0, q1, q2, q3;
};

class IMUSensor : protected I2CDevice {
private:
    float gyroScale;   
    float accelScale;  
    float magScale;    

    // Gyroscope Zero-Offset Variables
    float gyroOffsetX = 0.0f;
    float gyroOffsetY = 0.0f;
    float gyroOffsetZ = 0.0f;

    short rawAccel[3];
    short rawGyro[3];
    short rawMag[3];

    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    short combineRegisters(unsigned char msb, unsigned char lsb);

public:
    IMUSensor(unsigned int bus, unsigned int devAddress);
    virtual ~IMUSensor();

    virtual int initSensors();
    virtual void calibrateGyro(); // Zero-bias calibration
    virtual int readRawData();
    virtual void convertToPhysicalValues();
    virtual Quaternion updateAndGetQuaternion();
};

} /* namespace exploringRPi */

#endif
