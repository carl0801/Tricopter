#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "I2Cdev.h"
#include <Wire.h>
#include <VL53L0X.h>
#include "MPU6050.h"
#include "HMC5883L.h"

#define SAMPLE_PERIOD (0.01f) // 20ms sample period (50Hz)

class IMU {
    private:
        // Constants
        const double ANGLE_THRESHOLD = 0.05; // threshold for change in angle
        const double ALPHA = 0.8; // alpha value for the complementary filter

        // Structures
        struct RawData {
            int16_t ax, ay, az;
            int16_t gx, gy, gz;
            int16_t mx, my, mz;
        };

        struct Mag {
            double x = 0;
            double y = 0;
            double z = 0;
            double calX = 0;
            double calY = 0;
            double calZ = 0;
        };

    public:
        // Variables
        static const double STEP_TIME; // timeInterval

        // Functions
        void init_sensors();
        void getEulerRotation(double *roll, double *pitch, double *yaw);
        void getQuaternionRotation(double *w, double *x, double *y, double *z);
        void getAltitude(double *altitude);
        void getPressure(double *pressure);
        void getTemperature(double *temperature);
        void test(float *gx, float *gy, float *gz, float *ax, float *ay, float *az, float *mx, float *my, float *mz);
};

#endif // MY_SENSORS_H
