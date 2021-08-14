#include "CANDriver.h"

namespace can_device{

CANReceiveData_t can_receive_data;

void CANDriver::init(){
    CAN.setPins(_rxd, _txd);
    if (!CAN.begin(_baudrate)){
        while (1) log_e("Starting CAN failed!\r\n");
    }
    Serial.printf("CAN START!\r\n");
}

void CANDriver::send(const uint32_t& id, const std::vector<uint8_t>& buff){
    /*new演算子によるメモリの動的確保を行ってる*/
    CAN.beginPacket(id);
    uint8_t len = (uint8_t)buff.size();
    uint8_t *tx_buff;
    tx_buff = new uint8_t[len];
    for (uint8_t i = 0; i < len; i++)
        tx_buff[i] = buff[i];
    CAN.write(tx_buff, buff.size());
    CAN.endPacket();

    #ifdef CAN_DEBUG_ON
    // Serial.printf("CAN send ID = %d, dlc = %d\r\n",id,len);
    // for (int i = 0; i < len; i++)
        // Serial.printf("Data[%d] = %d,", i, tx_buff[i]);
    // Serial.printf("\r\n");
    #endif /*CAN_DEBUG_ON*/

    delete[] tx_buff;
}

void CANDriver::receive(const uint32_t& id, const std::vector<uint8_t>& buff, const uint8_t& dlc){
    can_receive_data.id = id;
    can_receive_data.buff = buff;
    can_receive_data.dlc = dlc;
    #ifdef CAN_DEBUG_ON
    uint8_t len = (uint8_t)buff.size();
    Serial.printf("CAN receive ID = %d, dlc = %d\r\n",id,dlc);
    for (int i = 0; i < len; i++)
        Serial.printf("Data[%d] = %d,", i, buff[i]);
    Serial.printf("\r\n");
    #endif /*CAN_DEBUG_ON*/
}


}/*can_device*/