#ifndef IMU_H
#define IMU_H

#include "Fusion.h"
#include "MPU9250.h"

#define SAMPLE_RATE (100) // Hz
#define GRAVITY 9.81f // m/s^2


#define MPU9250_ADDRESS 0x69 // Device address when AD0 pin is connected to pin 2

//#define VL53L0X_CONNECT true// Device address


#ifdef MPU9250_ADDRESS

    #include "MPU9250.h"
    #define AD0_PIN 17 // AD0 pin is connected to pin 2

#endif //MPU9250_ADDRESS 

#ifdef VL53L0X_CONNECT

    #include "VL53L0X.h"

#endif //VL53L0X_ADDRESS


class IMU {
    private:
        // Functions
        void read_sensors();

        // Variables
        float time_now;
        float time_former;
        float deltat;

        // Sensor variables
        float accel[3]; 
        float magnetom[3];
        float gyro[3];

        // Data variables
        float euler_rad[3];
        float quaternians[4];
        float position[3];



    public:
        // Functions
        void init();
        void update_IMU();
        void sendToPC(float* data1, float* data2, float* data3);

        // get data functions
        void getEulerRad(float* roll, float* pitch, float* yaw);
        void getQuaternians(float* w, float* x, float* y, float* z);
        void getEarthAcceleration(float* x, float* y, float* z);
        void getLidarData(float* data1, float* data2);

        
};

#endif // MY_SENSORS_H
