/**
 * @file YamaSolenoidValve.h
 * @author Yamaguchi Yudai
 * @brief
 * @version 1.0
 * @date 2022-03-25
 */

#ifndef YAMASOLENOIDVALVE_H_
#define YAMASOLENOIDVALVE_H_

#include "CANDriver.h"

namespace yamasol {

const uint32_t SOL_TARGET_ID = 0x351;

class YamaSolenoidValve {
private:
    can_device::CANDriver& _can_driver;
    const uint8_t _num;

public:
    YamaSolenoidValve(can_device::CANDriver can_driver, uint8_t num=0)
        : _can_driver(can_driver),_num(num) {}
    /*** @brief MSBを1にするとポート8がオンに、LSBを1にするとポート1がオンになる。 */
    void send(uint8_t data) {
        std::vector<uint8_t> send_data{_num,data};
        _can_driver.send(SOL_TARGET_ID, send_data);
    }
};
}  // namespace sol

#endif  // YAMASOLENOIDVALVE_H_