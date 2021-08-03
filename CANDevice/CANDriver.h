
#ifndef CANDRIVER_H_
#define CANDRIVER_H_
#include <Arduino.h>
#include <CAN.h>
#include <vector>
#include "esp_log.h"

// #define CAN_DEBUG_ON

namespace can_device{

const long WROOM32_BAUDRATE = 1000e3;

typedef struct{
    uint32_t id;
    std::vector<uint8_t> buff;
    uint8_t dlc;
}CANReceiveData_t;

extern CANReceiveData_t can_receive_data;

class CANDriver{
    private:
        long _baudrate;
        const uint8_t _rxd = 0;
        const uint8_t _txd = 0;
    public:
        CANDriver(long baudrate, uint8_t rxd, uint8_t txd)
            :_baudrate{baudrate}, _rxd{rxd} , _txd{txd}{}
        void init();
        void send(uint32_t id, std::vector<uint8_t>& buff);
        void receive(uint32_t id, std::vector<uint8_t> buff, uint8_t dlc);
};

}/*can_device*/
#endif /*CANDRIVER_H_*/
