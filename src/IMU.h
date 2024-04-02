#ifndef IMU_H
#define IMU_H

#include "Fusion.h"
#include "MPU9250.h"

#define SAMPLE_RATE (2000) // Hz
#define GRAVITY 9.81f // m/s^2


#define MPU9250_ADDRESS 0x69 // Device address when AD0 pin is connected to pin 2

//#define VL53L0X_ADDRESS 0x29 // Device address


#ifdef MPU9250_ADDRESS

    #include "MPU9250.h"
    #define AD0_PIN 2 // AD0 pin is connected to pin 2

#endif //MPU9250_ADDRESS 

#ifdef VL53L0X_ADDRESS

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

        float yaw_offset = (2.61f - M_PI_2);

        // Data variables
        float euler_rad[3];
        float quaternians[4];
        double position[3];



    public:
        // Functions
        void init_IMU();
        void update_IMU();
        void sendToPC(float* data1, float* data2, float* data3);

        // get data functions
        void getEulerRad(float* roll, float* pitch, float* yaw);
        void getQuaternians(float* w, float* x, float* y, float* z);
        void getPosition(float* x, float* y, float* z);

        
};

#endif // MY_SENSORS_H
