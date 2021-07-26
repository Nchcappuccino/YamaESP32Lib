#ifndef AS5601_H_
#define AS5601_H_
#include <Arduino.h>
#include <Wire.h>
#include "esp_log.h"
#ifndef ROTATIONDIR

#define ROTATIONDIR
enum class RotationDir{
	REGULAR_DIR = 1,	//通常の方向.
	NEGATIVE_DIR = -1		//通常とは逆.
};
#endif

namespace as5601{

class AS5601{
    private:
        static constexpr uint8_t ZMCO = 0x00;
        static constexpr uint8_t ZPOS = 0x01;
        static constexpr uint8_t CONF = 0x07;
        static constexpr uint8_t ABN = 0x09;
        static constexpr uint8_t PUSHTHR = 0x0a;
        static constexpr uint8_t RAW_ANGLE = 0x0c;
        static constexpr uint8_t ANGLE = 0x0e;
        static constexpr uint8_t STATUS = 0x0b;
        static constexpr uint8_t AGC = 0x1a;
        static constexpr uint8_t MAGNITUDE = 0x1b;
        static constexpr uint8_t BURN = 0xff;
        static constexpr uint8_t DEVICE_ADDR = 0x36;
        void _write(uint8_t reg, uint8_t data);
        uint8_t _read(uint8_t reg);
        uint16_t _read2Byte(uint8_t reg);
        void _getRawAngle();
        void _calculation();
        uint8_t _i2c_buff[2];
        uint16_t _raw_angle;
        uint16_t _prev_raw_angle;
        float _time;             //[ms]
        float _prev_time;
        float _dt;
        float _angle;           //[rad], -pi~pi
        float _prev_angle;
        float _angular_v;        //[rad/s]
        float _prev_angular_v;
        float _rpm;
        bool _noise;    //ループの中でノイズがのっているかの判別用   
        bool _alive;    //AS5601と通信できてるかのフラグ,falseだと失敗してる. 
    public:
        AS5601(RotationDir dir, uint16_t range_th)
            :_dir(dir), _range_th(range_th){
                _noise = false;
            };
        void update();
        void print();
        const uint16_t& getRawAngle() const {return _raw_angle;}
        const float& getAngle() const {return _angle;}
        const float& getAngularV() const {return _angular_v;}
        const float& getRPM() const {return _rpm;}
        const bool& getAS5601Alive() const {return _alive;}
    private:
        RotationDir _dir;
        //角度が4096から0に移り変わったときに角速度がおかしくならないようにする閾値.
        const uint16_t _range_th;        
};
}

#endif/*AS5601_H_*/
