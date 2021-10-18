#include "YamaMotherv3.h"
namespace yamamotherv3{
void YamaMotherV3::LED(uint8_t array, const CRGB::HTMLColorCode& color){
    if(array < 0 || array > 255){
        log_e("YamaMotherV3 LED array is out of range.\r\n");
        return;
    }
    uint8_t led_judg;
    FastLED.setBrightness(_mother_led_brightness);
    for(int i = 0; i < 8; i++){
        led_judg = array % 2;
        array = array / 2;
        if(led_judg == 0)
            leds[i] = (0,0,0);
        else
            leds[i] = color;
    }
    FastLED.show();
}

void YamaMotherV3::LED(CRGB *led, uint8_t crgb_size){
    if(all_led_num*3 == crgb_size){
        memcpy(leds,led,crgb_size);
        FastLED.show();
    }else{
        log_e("Data Size is error size:%d",sizeof(*leds));
    }
}

void YamaMotherV3::buzzer(const double& tone){
    if(_buzzer_disable_state == false)
        ledcWriteTone(buzzer_channel, tone);
}

void YamaMotherV3::init(){
    //Serial.beginとWire.beginはこのメンバ関数で行う.
    pinMode(usersw_pin, INPUT);
    pinMode(dipsw1_pin, INPUT);
    pinMode(dipsw2_pin, INPUT);
    _dipsw1_state = digitalRead(dipsw1_pin);
    _dipsw2_state = digitalRead(dipsw2_pin);
    _mother_num = _dipsw2_state << 1 || _dipsw1_state;
    Serial.begin(115200);
    Wire.begin(sda_pin,scl_pin);
    if(_wirespeed == MotherWireSpeed::SPEED100K)
        Wire.setClock(100000);
    else if(_wirespeed == MotherWireSpeed::SPEED400K)
        Wire.setClock(400000);
    SD.begin(sd_cs_pin,SPI ,24000000, "/sd");
    if(_imuserect == IMUSerect::MPU6050)
        mpu6050.init();
    can_driver.init();
    ledcSetup(buzzer_channel,5000,8);
    ledcAttachPin(buzzer_pin,buzzer_channel);
    Serial.printf("YamaMotherV3 init finished\r\n");
}

void YamaMotherV3::update(){
    _usersw_state = digitalRead(_usersw_state);
    _dipsw1_state = digitalRead(dipsw1_pin);
    _dipsw2_state = digitalRead(dipsw2_pin);
    if(_imuserect == IMUSerect::MPU6050)
        mpu6050.update();
}

}/*yamamotherv3*/