#include "MPU6050.h"

namespace mpu6050{

void MPU6050::_writeMPU6050(uint8_t reg, uint8_t data){
    Wire.beginTransmission(DEVICE_ID);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t MPU6050::_readMPU6050(uint8_t reg){
    Wire.beginTransmission(DEVICE_ID);
    Wire.write(reg);
    Wire.endTransmission(true);
    Wire.requestFrom(DEVICE_ID,1);
    uint8_t data = Wire.read();
    return data;
}

void MPU6050::_inAngleRange(double &data){
    if(data > PI)
        data = (PI2 * -1.0) + data;
    else if(data <= PI * -1.0)
        data = PI2 - data;    
}


void MPU6050::_getMPU6050Data(){
    double angle[3];
    Wire.beginTransmission(DEVICE_ID);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(DEVICE_ID, 14, true);

    _raw.ax = Wire.read() << 8 | Wire.read();
    _raw.ay = Wire.read() << 8 | Wire.read();
    _raw.az = Wire.read() << 8 | Wire.read();
    _raw.temp = Wire.read() << 8 | Wire.read();
    _raw.gx = Wire.read() << 8 | Wire.read();
    _raw.gy = Wire.read() << 8 | Wire.read();
    _raw.gz = Wire.read() << 8 | Wire.read();

    _imudata.temperature = _raw.temp / 340.0f + 36.53f;

    _imudata.ax = _raw.ax / ACCEL_FACTOR_2G * g - _offset.ax;
    _imudata.ay = _raw.ay / ACCEL_FACTOR_2G * g - _offset.ay;
    _imudata.az = _raw.az / ACCEL_FACTOR_2G * g - _offset.az;

    _interval = (micros() - _prev_t) * 0.000001;
    _prev_t = micros();

    _imudata.gx = ((double)_raw.gx / GYRO_FACTOR_250DPS * DEG_TO_RAD) - _offset.gx;
    _imudata.gy = ((double)_raw.gy / GYRO_FACTOR_250DPS * DEG_TO_RAD) - _offset.gy;
    _imudata.gz = ((double)_raw.gz / GYRO_FACTOR_250DPS * DEG_TO_RAD) - _offset.gz;

    angle[0] = _imudata.gx * _interval;
    angle[1] = _imudata.gy * _interval;
    angle[2] = _imudata.gz * _interval;

    //加速度から角度を算出
    _imudata.acc_angle_x = atan2(_imudata.ay, _imudata.az + abs(_imudata.ax)) * 1.0;
    _imudata.acc_angle_y = atan2(_imudata.ax, _imudata.az + abs(_imudata.ay)) * -1.0;

    //相補フィルタ
    // _imudata.roll += (0.996 * angle[0]) + (0.004 * _imudata.ax);
    // _imudata.pitch += (0.996 * angle[1]) + (0.004 * _imudata.ay);
    _imudata.yaw += angle[2];
    _imudata.pitch += angle[1];
    _imudata.roll += angle[0];
    _inAngleRange(_imudata.yaw);
    _inAngleRange(_imudata.pitch);
    _inAngleRange(_imudata.roll);
}


void MPU6050::_getOffset(){
    delay(2500);
    for(int i = 0; i  < _offset_times; i++){
        
        Wire.beginTransmission(DEVICE_ID);
        Wire.write(ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(DEVICE_ID, 14, true);

        _raw.ax = Wire.read() << 8 | Wire.read();
        _raw.ay = Wire.read() << 8 | Wire.read();
        _raw.az = Wire.read() << 8 | Wire.read();
        _raw.temp = Wire.read() << 8 | Wire.read();
        _raw.gx = Wire.read() << 8 | Wire.read();
        _raw.gy = Wire.read() << 8 | Wire.read();
        _raw.gz = Wire.read() << 8 | Wire.read();

        _offset.ax += (double)_raw.ax / ACCEL_FACTOR_2G * g;
        _offset.ay += (double)_raw.ay / ACCEL_FACTOR_2G * g;
        _offset.az += (double)_raw.az / ACCEL_FACTOR_2G * g - g;   //-gは重力加速度を考慮.
        _offset.gx += (double)_raw.gx / GYRO_FACTOR_250DPS * DEG_TO_RAD;
        _offset.gy += (double)_raw.gy / GYRO_FACTOR_250DPS * DEG_TO_RAD;
        _offset.gz += (double)_raw.gz / GYRO_FACTOR_250DPS * DEG_TO_RAD;
    }
    _offset.ax /= _offset_times;
    _offset.ay /= _offset_times;
    _offset.az /= _offset_times;
    _offset.gx /= _offset_times;
    _offset.gy /= _offset_times;
    _offset.gz /= _offset_times;
    Serial.printf("IMU Offset Task Finished\r\n");
}


void MPU6050::init(){
    /*Wire.beginはMPU6050のメンバ関数内では呼び出していないので、別途呼び出す必要あり*/
    _writeMPU6050(PWR_MGMT_1, 0x00);
    _writeMPU6050(SMPLRT_DIV, 0x00);
    _writeMPU6050(CONFIG, LPF_44HZ);         //gyro output rate 1khz
    _writeMPU6050(GYRO_CONFIG, GYRO_SET_250DPS);
    _writeMPU6050(ACCEL_CONFIG, ACCEL_SET_2G);
    _writeMPU6050(PWR_MGMT_1,0x01);     //disable sleep mode, PLL with X gyro
    while(_readMPU6050(WHO_AM_I) != DEVICE_ID){
        log_e("MPU6050 WHO_AM_I Failed WHO_AM_I is %d\r\n",_readMPU6050(WHO_AM_I));
        delay(500);
    }
    Serial.printf("IMU OK\r\n");
    _getOffset();

}

void MPU6050::update(){
    _getMPU6050Data();
}

/**
 * @brief 
 * 全部表示したくないときは自由にコメントアウトしてください。
 * もちろん追加もOKです。
 */
void MPU6050::printData(){
    /*
    Serial.print("interval");
    Serial.print(_interval);
    Serial.print("offset times");
    Serial.print(_offset_times);
    Serial.print("offset_ax ");
    Serial.print(_offset.ax);
    Serial.print("offset_ay ");
    Serial.print(_offset.ay);
    Serial.print("offset_az ");
    Serial.print(_offset.az);
    Serial.print("offset_gx ");
    Serial.print(_offset.gx);
    Serial.print("offset_gy ");
    Serial.print(_offset.gy);
    Serial.print("offset_gz ");
    Serial.print(_offset.gz);
    */
    Serial.printf("ax %+4.3f ",_imudata.ax);
    Serial.printf("ay %+4.3f ",_imudata.ay);
    Serial.printf("az %+4.3f ",_imudata.az);
    /*
    Serial.printf("gx %+4.3f ",_imudata.gx);
    Serial.printf("gy %+4.3f ",_imudata.gy);
    Serial.printf("gz %+4.3f ",_imudata.gz);
    Serial.printf("temp %+4.3f ",_imudata.temperature);
    */
    Serial.printf("acc_angle_x %+4.3f ",_imudata.acc_angle_x);
    Serial.printf("acc_angle_y %+4.3f ",_imudata.acc_angle_y);
    Serial.printf("yaw %+4.3f ",degrees(_imudata.yaw));
    Serial.printf("pitch %+4.3f ",degrees(_imudata.pitch));
    Serial.printf("roll %+4.3f ",degrees(_imudata.roll));


    Serial.printf("\r\n");

}
}/*mpu6050*/