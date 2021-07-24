#include "CANDriver.h"

ESP32SJA1000Class esp32can;

namespace can_device{

CAN_ID_t can_id;

CANDriver::CANDriver(long baudrate):_baudrate{baudrate}{}

void CANDriver::init(){
    esp32can.setPins(2, 4);
    if (!esp32can.begin(_baudrate)){
        Serial.println("Starting CAN failed!");
        while (1);
    }
    Serial.printf("CAN START!\r\n");
}

void CANDriver::send(uint32_t id,std::vector<uint8_t> &buff){
    /*new演算子によるメモリの動的確保を行ってる*/
    esp32can.beginPacket(id);
    uint8_t len = (uint8_t)buff.size();
    uint8_t *tx_buff;
    tx_buff = new uint8_t[len];
    for (uint8_t i = 0; i < len; i++)
        tx_buff[i] = buff[i];
    esp32can.write(tx_buff, buff.size());
    esp32can.endPacket();

    #ifdef CAN_DEBUG_ON
    // Serial.printf("CAN send ID = %d, len = %d\r\n",id,len);
    // for (int i = 0; i < len; i++)
        // Serial.printf("Data[%d] = %d,", i, tx_buff[i]);
    // Serial.printf("\r\n");
    #endif /*CAN_DEBUG_ON*/

    delete[] tx_buff;
}

void CANDriver::receive(uint32_t id,std::vector<uint8_t> buff){
    _receive_id = id;
    _receive_buff = buff;
    #ifdef CAN_DEBUG_ON
    uint8_t len = (uint8_t)buff.size();
    Serial.printf("CAN receive ID = %d, len = %d\r\n",id,len);
    for (int i = 0; i < len; i++)
        Serial.printf("Data[%d] = %d,", i, buff[i]);
    Serial.printf("\r\n");
    #endif /*CAN_DEBUG_ON*/
}

uint32_t CANDriver::getID() const{
    return _receive_id;
}

std::vector<uint8_t> CANDriver::getBuff() const{
    return _receive_buff;
}

}/*can_device*/