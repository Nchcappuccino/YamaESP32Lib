#include "AS5601.h"
namespace as5601{

void AS5601::_write(uint8_t reg, uint8_t data){
    Wire.beginTransmission(DEVICE_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t AS5601::_read(uint8_t reg){
    Wire.beginTransmission(DEVICE_ADDR);
    Wire.write(reg);
    Wire.endTransmission(true);
    Wire.requestFrom(DEVICE_ADDR,1);
    uint8_t data = Wire.read();
    return data;
}

uint16_t AS5601::_read2Byte(uint8_t reg){
    Wire.beginTransmission(DEVICE_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(DEVICE_ADDR, 2);
    uint16_t data;
    _i2c_buff[0] = Wire.read();
    _i2c_buff[1] = Wire.read();
    data = ((uint16_t)_i2c_buff[0] << 8) & 0x0F00;
    data |= (uint16_t)_i2c_buff[1];
    return data;
    // printf("data %d ",data);
}

void AS5601::_getRawAngle(){
    if(_noise == false)     _prev_raw_angle = _raw_angle;
    _raw_angle = _read2Byte(RAW_ANGLE);

    //ノイズ処理,ノイズがのっていた場合は前回値を使用する.
    if(_raw_angle < 0 || _raw_angle > 4096){
        _raw_angle = _prev_raw_angle;
        _noise = true;
        return;
    }else if(_noise == true){
        //_noiseがtrueの時に下のノイズ処理のプログラムに行くとバグるので_noiseをfalseにしてreturn
        _noise = false;
        return;
    }
    if(_prev_raw_angle > _range_th && _prev_raw_angle < 4096 - _range_th){
        if(_raw_angle - _prev_raw_angle > _range_th || _raw_angle - _prev_raw_angle < _range_th * -1){
            _raw_angle = _prev_raw_angle;
            _noise = true;
        }else{
            _noise = false;
        }
    }else{
        _noise = false;
    }
}

void AS5601::_calculation(){
    //角度を-pi~pi[rad]に変換.
    _angle = ((float)_raw_angle - 2048.0f) / 4096.0f * 360.0f * DEG_TO_RAD
                 * static_cast<float>(_dir);
    _time = (float)millis();
    _dt = (_time - _prev_time) * 0.001f;
    //角度が4096から0に移り変わったときに角速度がおかしくならないようにする処理.
    //角度が4096から0または逆に移り変わったときは角速度は前回値を使用する.
    if(_raw_angle < _range_th && _prev_raw_angle > 4096 - _range_th){
        _angular_v = _prev_angular_v;
    }else if(_raw_angle > 4096 - _range_th && _prev_raw_angle < _range_th){
        _angular_v = _prev_angular_v;
    }else{
        _angular_v = (_angle - _prev_angle) / _dt;
    }
    _rpm = _angular_v * 30.0f / PI;

    _prev_time = _time;
    _prev_angle = _angle;
    _prev_angular_v = _angular_v;
}

void AS5601::update(){
    _getRawAngle();
    _calculation();
}

void AS5601::print(){
    //使うときは自由にコメントアウトしてください.
    Serial.printf("wireH %d\t",_i2c_buff[0]);
    Serial.printf("wireL %d\t",_i2c_buff[1]);
    Serial.printf("raw angle %d ", _raw_angle);
    Serial.printf("angle %f[rad] %f[deg] ", _angle,degrees(_angle));
    Serial.printf("angular_v %f ",_angular_v);
    Serial.printf("RPM %f ",_rpm);
    Serial.printf("noise flag %d ",_noise);
    Serial.printf("\r\n");
}

}/*as5601*/