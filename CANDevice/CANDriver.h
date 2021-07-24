
#ifndef CANDRIVER_H_
#define CANDRIVER_H_
#include <Arduino.h>
#include <CAN.h>
#include <vector>

#define CAN_DEBUG_ON

extern ESP32SJA1000Class esp32can;

namespace can_device{

/*WrOOM32の罠対策、面倒なのでとりあえずはボーレートの選択は手動で*/
const long WROOM32_BAUDRATE = 1000e3;

typedef struct{
    struct {
        const uint32_t SET_KP_ID = 0x001;
        const uint32_t SET_KI_ID = 0x002;
        const uint32_t SET_KD_ID = 0x003;
        const uint32_t SET_DT_AND_MODE_ID = 0x004; //ここはdt以外にもいろんなモードのデータも入れる予定
        const uint32_t UPDATE_TARGET_ID = 0x005;	//angleかspeed
        const uint32_t MD_STATE_ID = 0x006;
    } md;
    //ここにstructで他のIDも追加する
} CAN_ID_t;
extern CAN_ID_t can_id;

class CANDriver{
    private:
        long _baudrate;
        uint32_t _receive_id;
        std::vector<uint8_t> _receive_buff;
    public:
        bool receive_task_flag;
        CANDriver(long baudrate);
        void init();
        void send(uint32_t id,std::vector<uint8_t> &buff);
        void receive(uint32_t id, std::vector<uint8_t> buff);
        uint32_t getID() const;
        std::vector<uint8_t> getBuff() const;
};

}/*can_device*/
#endif /*CANDRIVER_H_*/
