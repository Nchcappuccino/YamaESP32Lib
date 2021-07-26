
#ifndef CANDRIVER_H_
#define CANDRIVER_H_
#include <Arduino.h>
#include <CAN.h>
#include <vector>

// #define CAN_DEBUG_ON

namespace can_device{

const long WROOM32_BAUDRATE = 1000e3;

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
