/**
 * @file CANDriver.h
 * @author Yamaguchi Yudai
 * @brief CANの送受信をする関数。一部main.cppに記述する必要あり
 * @date 2021-08-15
 */


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

/**
 * @brief 
 * CANの受信データ
 */
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
        void send(const uint32_t& id, const std::vector<uint8_t>& buff);
        /**
         * @brief 
         * この関数は受信時の割り込みタスクで回す。
         * 
         * @param id        CANの割り込みで受信したid 
         * @param buff      CANの割り込みで受信したbuff(配列ではなくvector)
         * @param dlc       CANの割り込みで受信したdlc(データ長)
         */
        void receive(const uint32_t& id, const std::vector<uint8_t>& buff, const uint8_t& dlc);
};

}/*can_device*/
#endif /*CANDRIVER_H_*/

/*
CAN受信のプログラム例(イメージ)

void onReceive(int packetSize){
	std::vector<uint8_t>buff;
	while(CAN.available()){
		buff.push_back(static_cast<uint8_t>(CAN.read()));
	}
	sample.yama.can_driver.receive(static_cast<uint32_t>(CAN.packetId()), buff, static_cast<uint8_t>(CAN.packetDlc()));
	sample.canInterruptTask();
}
*/