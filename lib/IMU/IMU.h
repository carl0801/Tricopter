#ifndef IMU_H
#define IMU_H

#include "Fusion.h"
#include "MPU9250.h"
#include "ArduinoEigen.h"


//#define SAMPLE_RATE (100) // Hz
#define GRAVITY 9.81f // m/s^2


#define MPU9250_ADDRESS 0x69 // Device address when AD0 pin is connected to pin 2

#define VL53L0X_CONNECT true// Device address


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
        u_int SAMPLE_RATE;

        // Variables
        double time_now;
        double time_former;
        double deltat;

        // Sensor variables
        float accel[3]; 
        float magnetom[3];
        float gyro[3];

        // Data variables
        double euler_rad[3];
        double quaternians[4];
        double position[3];
        double velocity[3];
        double acceleration[3];
        double angular_velocity[3];



    public:
        // Functions
        IMU(double dt);
        void init();
        void update_IMU();
        void sendToPC(double* data1, double* data2, double* data3);

        // get data functions
        void getEulerRad(double* roll, double* pitch, double* yaw);
        void getYaw(double* yaw);
        void getQuaternians(double* w, double* x, double* y, double* z);
        void getPos(double* x, double* y, double* z);
        void getLidarData(double* data1, double* data2);
        void getAngularVelocity(double* x, double* y, double* z);

        
};

#endif // MY_SENSORS_H
