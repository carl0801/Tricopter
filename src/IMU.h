#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include "Fusion.h"

#define SAMPLE_PERIOD (0.01f) // 20ms sample period (50Hz)

#define MPU9250_ADDRESS 0x69 // Device address when AD0 pin is connected to pin 2

//#define HW_290_ADDRESS 0x68 // Device address 

//#define VL53L0X_ADDRESS 0x29 // Device address


#ifdef HW_290_ADDRESS

    #include <Wire.h>
    #include "MPU6050.h"
    #include "HMC5883L.h"
    #include <Adafruit_BMP085.h>

#endif //HW_290_ADDRESS


#ifdef MPU9250_ADDRESS

    #include "MPU9250.h"
    #define AD0 2 // AD0 pin is connected to pin 2

#endif //MPU9250_ADDRESS 

#ifdef VL53L0X_ADDRESS

    #include "VL53L0X.h"

#endif //VL53L0X_ADDRESS


class IMU {
    private:
        // Structures
        struct RawData {
            int16_t ax, ay, az;
            int16_t gx, gy, gz;
            int16_t mx, my, mz;
        };
        RawData raw;

        struct Mag {
            double x = 0;
            double y = 0;
            double z = 0;
            double calX = 0;
            double calY = 0;
            double calZ = 0;
        };
        Mag mag;

        struct filterData{
            float gx, gy, gz;
            float ax, ay, az;
            float mx, my, mz;
            float gx1, gy1, gz1;
            float ax1, ay1, az1;
            float mx1, my1, mz1;
        };
        filterData data;
        
        void getRead();
        void transformation();

    public:
        // Variables
        static const double STEP_TIME; // timeInterval

        // Functions

        void init_sensors();
        void readSensor();
        void getEulerRotation(double *roll, double *pitch, double *yaw);
        void getQuaternionRotation(double *w, double *x, double *y, double *z);
        void getAltitude(double *altitude);
        void getPressure(double *pressure);
        void getTemperature(double *temperature);
        void test(float *gx, float *gy, float *gz, float *ax, float *ay, float *az, float *mx, float *my, float *mz, int board);
};

#endif // MY_SENSORS_H
