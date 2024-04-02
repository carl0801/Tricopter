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
        double time_now;
        double time_former;
        double deltat;

        // Sensor variables
        double accel[3]; 
        double magnetom[3];
        double gyro[3];

        double yaw_offset = (2.61f - M_PI_2);

        // Data variables
        double euler_rad[3];
        double quaternians[4];
        double position[3];



    public:
        // Functions
        void init_IMU();
        void update_IMU();
        void sendToPC(double* data1, double* data2, double*);

        // get data functions
        void getEulerRad(double* roll, double* pitch, double* yaw);
        void getQuaternians(double* w, double* x, double* y, double* z);
        void getPosition(double* x, double* y, double* z);

        
};

#endif // MY_SENSORS_H
