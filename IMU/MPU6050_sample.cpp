/*Wire.beginとsetClockはMPU6050のメンバ関数内で行ってないので各自行ってください*/
#include <Arduino.h>
#include "MPU6050.h"

mpu6050::MPU6050 imu;
void setup(){
    Serial.begin(115200);
    Wire.begin(26,27);
    Wire.setClock(400000);
    imu.init();
}

void loop(){
    imu.update();
    imu.printData();

}