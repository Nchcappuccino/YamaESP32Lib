//実験したら25分で7度ずれた

#ifndef MPU6050_H_
#define MPU6050_H_

#include <Arduino.h>
#include <Wire.h>
#include "esp_log.h"
namespace mpu6050{

typedef struct{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temp;       //温度
    int16_t gx;
    int16_t gy;
    int16_t gz;
}RawData_t;

typedef struct{
    double ax;          //[m/s^2]
    double ay;          //[m/s^2]
    double az;          //[m/s^2]
    double gx;          //[rad/s]
    double gy;          //[rad/s]
    double gz;          //[rad/s]
    double acc_angle_x;     //-pi~pi[rad], 加速度センサーから逆三角関数を使って算出
    double acc_angle_y;     //-pi~pi[rad], 加速度センサーから逆三角関数を使って算出
    double yaw;         //-pi~pi[rad]
    double pitch;       //-pi~pi[rad]
    double roll;        //-pi~pi[rad]
    double temperature;
}IMUData_t;

typedef struct{
    double ax;          //[m/s^2]
    double ay;          //[m/s^2]
    double az;          //[m/s^2]
    double gx;          //[rad/s]
    double gy;          //[rad/s]
    double gz;          //[rad/s]
}Offset_t;

typedef struct{
    double w;  
    double x;
    double y;
    double z;
}Quaternion_t;          //unit less

class MPU6050{
    private:
        static constexpr uint8_t DEVICE_ID = 0x68;
        static constexpr uint8_t SMPLRT_DIV = 0x19;
        static constexpr uint8_t CONFIG = 0x1a;
        static constexpr uint8_t GYRO_CONFIG = 0x1b;
        static constexpr uint8_t ACCEL_CONFIG = 0x1c;
        static constexpr uint8_t WHO_AM_I = 0x75;
        static constexpr uint8_t PWR_MGMT_1 = 0x6b;
        static constexpr uint8_t ACCEL_XOUT_H = 0x3b;

        static constexpr uint8_t LPF_260HZ = 0x00;      //delay 0.98ms
        static constexpr uint8_t LPF_184HZ = 0x01;      //delay 1.9ms
        static constexpr uint8_t LPF_94HZ = 0x02;       //delay 2.8ms
        static constexpr uint8_t LPF_44HZ = 0x03;       //delay 4.8ms
        static constexpr uint8_t LPF_21HZ = 0x04;       //delay 8.3ms
        static constexpr uint8_t LPF_10HZ = 0x05;       //delay 13.4ms
        static constexpr uint8_t LPF_5HZ = 0x06;        //delay 18.6ms
        static constexpr uint8_t LPF_DISABLE = 0x07;    //delay 0.0ms

/*GYRO_SET_xxxとACCEL_SET_xGの範囲は-xから+xまでなの注意*/
        static constexpr uint8_t GYRO_SET_250DPS = 0x00;
        static constexpr uint8_t GYRO_SET_500DPS = 0x08;
        static constexpr uint8_t GYRO_SET_1000DPS = 0x10;
        static constexpr uint8_t GYRO_SET_2000DPS = 0x18;

        static constexpr uint8_t ACCEL_SET_2G = 0x00;
        static constexpr uint8_t ACCEL_SET_4G = 0x02;
        static constexpr uint8_t ACCEL_SET_8G = 0x10;
        static constexpr uint8_t ACCEL_SET_16G = 0x18;

        static constexpr double GYRO_FACTOR_250DPS = 131.0;
        static constexpr double GYRO_FACTOR_500DPS = 65.5;
        static constexpr double GYRO_FACTOR_1000DPS = 32.8;
        static constexpr double GYRO_FACTOR_2000DPS = 16.4;

        static constexpr double ACCEL_FACTOR_2G = 16384.0;
        static constexpr double ACCEL_FACTOR_4G = 8192.0;
        static constexpr double ACCEL_FACTOR_8G = 4096.0;
        static constexpr double ACCEL_FACTOR_16G = 2048.0;

        static constexpr double g = 9.798;      //重力加速度(東京)
        static constexpr double PI2 = PI * 2.0;

        static constexpr int _offset_times = 10000;   //オフセットの実行回数
        RawData_t _raw;
        Offset_t _offset;
        IMUData_t _imudata;
        Quaternion_t _q;
        double _prev_t;
        double _interval;
        
        void _writeMPU6050(uint8_t reg, uint8_t data);
        uint8_t _readMPU6050(uint8_t reg);
        void _inAngleRange(double &data);
        void _getMPU6050Data();
        void _getOffset();
    public:
        MPU6050(){}
        void init();
        void update();
        void printData();
        const RawData_t& getRawData() const     {return _raw;}
        const Offset_t& getOffsetData() const   {return _offset;}
        const IMUData_t& getIMUData() const     {return _imudata;}
        const Quaternion_t& getQuaternion()const{return _q;}
        const int& getOffsetTimes() const       {return _offset_times;}
};

}/*mpu6050*/
#endif/*MPU6050_H_*/