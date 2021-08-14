/**
 * @file YamaMotherV3.h
 * @author Yamaguchi Yudai
 * @brief 
 * @date 2021-08-15
 * 
 */

#ifndef YAMAMOTHERV3_H_
#define YAMAMOTHERV3_H_

#include <Arduino.h>
#include <SD.h>
#include "FastLED.h"
#include "MPU6050.h"
#include "CANDriver.h"
#include "esp_log.h"

/**
 * @brief YamaMotherV3用の名前空間
 */
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

/**
 * @brief 
 * 使用したいIMUを選ぶ。使用しない場合はDISABLE
 */
enum class IMUSerect{
    DISABLE,
    MPU6050,
    BNO055,
};

/**
 * @brief 
 * 使用したい速度を選ぶ。使用する場合は100K推奨。
 * 使わない場合はDISABLEにする。
 */
enum class MotherWireSpeed{
    DISABLE,
    SPEED100K,
    SPEED400K,
};

class YamaMotherV3{
    private:
        const IMUSerect _imuserect = IMUSerect::MPU6050;
        const MotherWireSpeed _wirespeed = MotherWireSpeed::SPEED400K;

        /**
         * @brief 
         * マザーの番号。DIPスイッチで設定可能。
         * DIPスイッチに書いてある数字で1が2進数でいう1bit目、2が2bit目に対応している。
         * 数字の範囲は0~3。
         * DIPスイッチから読む都合上constにはできない。
         */
        uint8_t _mother_num;
        
        /**
         * @brief 
         * マザーに搭載してるRGBLEDの明るさ。
         * 明るさを変えたい場合は自由に変えてください。
         */
        static constexpr uint8_t _mother_led_brightness = 15;
        
        /**
         * @brief 
         * マザーに搭載されてるRGBLEDの数。
         * ロボットに搭載されてる数とは違うので注意。
         */
        static constexpr uint8_t _mother_led_num = 8;
        int _usersw_state;
        int _dipsw1_state;
        int _dipsw2_state;

        /**
         * @brief trueならブザーオフ
         */
        bool _buzzer_disable_state = false;
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
        
        /**
         * @brief 
         * ロボットに搭載されてるすべてのRGBLEDの数
         * classのコンストラクタの引数でall_led_numの数は変更できる。
         */
        const uint8_t all_led_num = 8;
        
        /**
         * @brief メモリの動的確保を行ってる。
         */
        CRGB *leds;
        mpu6050::MPU6050 mpu6050;
        can_device::CANDriver can_driver{can_device::WROOM32_BAUDRATE, can_rxd, can_txd};
        YamaMotherV3(IMUSerect imuserect, MotherWireSpeed wirespeed, uint8_t led_num)
            :_imuserect(imuserect),_wirespeed(wirespeed),all_led_num(led_num){
                while(led_num < 8){
                    log_e("_all_led_num is less than 8.");
                    delay(1000);
                }
                leds = new CRGB[all_led_num];
                
                /**
                 * @brief WS2812Bを使うときは第3引数はRGBではなくGRBにする
                 */
                FastLED.addLeds<WS2812B, rgbled_pin, GRB>(this->leds, all_led_num);
        }

        /**
         * @brief 8bitを8個のLEDがいい感じに表現してくれる、第二引数で色の指定が可能
         * 
         * @param array 表示させたい数字(array)
         * @param color 色。CRGB::HTMLColorCodeから好きな色を選ぶ。
         */
        void LED(uint8_t array, const CRGB::HTMLColorCode& color);

        /**
         * @brief ブザーをオフにする。 
         */
        void buzzerDisable(){_buzzer_disable_state = true;}

        /**
         * @brief 音を鳴らす。buzzerDisableが呼び出されてるときは音はならない。
         * 
         * @param tone 鳴らしたい音の周波数。周波数に関してはこのヘッダファイルの名前空間内に定義済み。 
         */
        void buzzer(const double& tone);

        /**
         * @brief ペリフェラル関係(Serial.begin)などは全てこの関数で行う
         * 
         */
        void init();

        /**
         * @brief
         * IMUの値とスイッチの状況を更新する。
         * SDカード関係
         */
        void update();
        
        const uint8_t& getMotherNum() const {return _mother_num;}
        const uint8_t& getUserSW()        const {return _usersw_state;}
        const uint8_t& getDIPSW1()        const {return _dipsw1_state;}
        const uint8_t& getDIPSW2()        const {return _dipsw2_state;}
        const uint8_t& getBuzzerDisableState() const {return _buzzer_disable_state;}
};
}
#endif /*YAMAMOTHERV3_H_*/