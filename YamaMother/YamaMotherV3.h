#ifndef YAMAMOTHERV3_H_
#define YAMAMOTHERV3_H_

#include <Arduino.h>
#include <SD.h>
#include "FastLED.h"
#include "MPU6050.h"
#include "CANDriver.h"
#include "esp_log.h"

namespace yamamotherv3{

const double C4 = 261.6;
const double CS4 = 277.18;  //C#4
const double D4 = 293.665;
const double DS4 = 311.127;  //D#4
const double E4 = 329.63;
const double F4 = 349.228;
const double FS4 = 369.994;  //F#4
const double G4 = 391.995;
const double GS4 = 415.305;  //G#4
const double A4 = 440;
const double AS4 = 466.164;  //A#4
const double B4 = 493.883;
const double C5 = 523.251;
const double BUZZER_STOP = 0.0;

enum class IMUSerect{
    DISABLE,
    MPU6050,
    BNO055,
};

enum class MotherWireSpeed{
    DISABLE,
    SPEED100K,
    SPEED400K,
};

class YamaMotherV3{
    private:
        const IMUSerect _imuserect = IMUSerect::MPU6050;
        const MotherWireSpeed _wirespeed = MotherWireSpeed::SPEED400K;
        uint8_t _mother_num;                        //DIPスイッチから読む都合上constにはできない.
        //マザーに搭載してるRGBLEDの明るさ.
        static constexpr uint8_t _mother_led_brightness = 15;
        //マザー上のRGBLEDの数.
        static constexpr uint8_t _mother_led_num = 8;
        int _usersw_state;
        int _dipsw1_state;
        int _dipsw2_state;
        bool _buzzer_disable_state = false;     //trueならブザーオフ
    public:
        //GPIO系の定数がpublicな理由はハードウェアの不良時にアクセスしやすくするため.
        static constexpr uint8_t can_txd = 4;
        static constexpr uint8_t can_rxd = 2;
        static constexpr uint8_t rgbled_pin = 32;
        static constexpr uint8_t buzzer_pin = 12;
        static constexpr uint8_t mosi_pin = 23;
        static constexpr uint8_t miso_pin = 19;
        static constexpr uint8_t clk_pin = 18;
        static constexpr uint8_t sd_cs_pin = 5;
        static constexpr uint8_t sda_pin = 26;
        static constexpr uint8_t scl_pin = 27;
        static constexpr uint8_t usersw_pin = 35;
        static constexpr uint8_t dipsw1_pin = 39;
        static constexpr uint8_t dipsw2_pin = 34;
        static constexpr uint8_t uart2_tx_pin = 17;
        static constexpr uint8_t uart2_rx_pin = 16;
        static constexpr uint8_t buzzer_channel = 0;
        //マザーに接続されているすべてのLEDの数.
        const uint8_t all_led_num = 8;
        CRGB *leds;     //メモリの動的確保用.
        mpu6050::MPU6050 mpu6050;
        can_device::CANDriver can_driver{can_device::WROOM32_BAUDRATE, can_rxd, can_txd};
        YamaMotherV3(IMUSerect imuserect, MotherWireSpeed wirespeed, uint8_t led_num)
            :_imuserect(imuserect),_wirespeed(wirespeed),all_led_num(led_num){
                while(led_num < 8){
                    log_e("_all_led_num is less than 8.");
                    delay(1000);
                }
                leds = new CRGB[all_led_num];
                //WS2812Bを使うときは第3引数はRGBではなくGRBにする.
                FastLED.addLeds<WS2812B, rgbled_pin, GRB>(this->leds, all_led_num);
        }

        //8bitを8個のLEDがいい感じに表現してくれる、第二引数で色の指定が可能.
        void LED(uint8_t array, CRGB::HTMLColorCode color);
        void buzzerDisable(){_buzzer_disable_state = true;}
        void buzzer(double tone);
        void init();
        void update();
        const uint8_t& getMotherNum() const {return _mother_num;}
        const int& getUserSW()        const {return _usersw_state;}
        const int& getDIPSW1()        const {return _dipsw1_state;}
        const int& getDIPSW2()        const {return _dipsw2_state;}
        const int& getBuzzerDisableState() const {return _buzzer_disable_state;}
};
}
#endif /*YAMAMOTHERV3_H_*/