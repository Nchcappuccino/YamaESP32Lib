#include "YamaMDv3.h"

namespace yamamdv3{

void YamaMDv3::_sendInitData(){
    //send reset signal
    std::vector<uint8_t> tx_reset(1);
    tx_reset[0] = _md_num;
    _can_driver.send(MD_RESET_ID, tx_reset);
    delay(5);
    //send kp
    std::vector<uint8_t> tx_kp;
    tx_kp.reserve(5);
    tx_kp.push_back(_md_num);
    for (int i = 0; i < 4;i++)
        tx_kp.push_back(_init.kp.data[i]);
    _can_driver.send(SET_KP_ID, tx_kp);
    delay(5);
    //send ki
    std::vector<uint8_t> tx_ki;
    tx_ki.reserve(5);
    tx_ki.push_back(_md_num);
    for (int i = 0; i < 4;i++)
        tx_ki.push_back(_init.ki.data[i]);
    _can_driver.send(SET_KI_ID, tx_ki);
    delay(5);
    //send kd
    std::vector<uint8_t> tx_kd;
    tx_kd.reserve(5);
    tx_kd.push_back(_md_num);
    for (int i = 0; i < 4;i++)
        tx_kd.push_back(_init.kd.data[i]);
    _can_driver.send(SET_KD_ID, tx_kd);
    delay(5);
    //send mode etc
    std::vector<uint8_t> tx_mode(8);
    tx_mode[0] = _md_num;
    tx_mode[1] = _init.dt;
    tx_mode[2] = static_cast<uint8_t>(_init.pid_limit >> 8);
    tx_mode[3] = static_cast<uint8_t>(_init.pid_limit);
    tx_mode[4] = static_cast<uint8_t>(_init.pid_min_and_max_abs * 250.0f);
    tx_mode[5] = static_cast<uint8_t>(_init.origin_angle >> 8);
    tx_mode[6] = static_cast<uint8_t>(_init.origin_angle);
    tx_mode[7] = static_cast<uint8_t>(_init.select_md_send_mode);
    tx_mode[7] |= (static_cast<uint8_t>(_init.enc_mode)) << 3;
    if(static_cast<int8_t>(_init.enc_dir) == 1)
        tx_mode[7] |= 0b01000000;
    else
        tx_mode[7] |= 0b00000000;
    if(static_cast<int8_t>(_init.motor_dir) == 1)
        tx_mode[7] |= 0b10000000;
    else
        tx_mode[7] |= 0b00000000;
    _can_driver.send(SET_DT_AND_MODE_ID, tx_mode);
    Serial.printf("md%d init finished!\r\n",_md_num);
    delay(5);
}

void YamaMDv3::_sendTarget(){
    //目標値が前回値と同じときはデータを送らない.
    if(_prev_enable_duty == _send.enable_duty && _prev_target == _send.target)      return;
    switch(_init.enc_mode){
        case EncoderMode::ANGLE_MODE:{   
            std::vector<uint8_t> angle_buff;
            if(_send.enable_duty == true){
                if(_send.target < -1.0f || _send.target > 1.0f){
                    log_e("MD%d's target is out of range\r\n",_md_num);
                    return;
                }
                uint16_t duty = static_cast<uint16_t>((_send.target * 2 * LENGTH10BIT) + LENGTH11BIT);
                if(duty > LENGTH12BIT - 1)     duty = LENGTH12BIT - 1;
                angle_buff.push_back((static_cast<uint8_t>(duty >> 4) & 0b11110000) | _md_num);
                angle_buff.push_back(static_cast<uint8_t>(duty));
                angle_buff.push_back(255);      //ここの数字はuint8_tの範囲内のものであればなんでもいい.
            }else{
                if(_send.target <= PI * -1.0 || _send.target > PI){
                    log_e("MD%d's target is out of range\r\n",_md_num);
                    return;
                }
                int16_t angle = static_cast<int16_t>((_send.target * RAD_TO_DEG * 4096.0f / 360.0f) -1 + LENGTH11BIT);
                if(angle > 4095)    angle = 4095;
                else if(angle < 0)  angle = 0;
                angle_buff.push_back((static_cast<uint8_t>(angle >> 4) & 0b11110000) | _md_num);
                angle_buff.push_back(static_cast<uint8_t>(angle));
            }
            _can_driver.send(UPDATE_TARGET_ID, angle_buff);
            break;
        }
        case EncoderMode::SPEED_MODE:{
            std::vector<uint8_t> speed_buff(3);
            if(_send.enable_duty == true){
                if(_send.target < -1.0f || _send.target > 1.0f){
                    log_e("MD%d's target is out of range\r\n",_md_num);
                    return;
                }
                uint32_t duty = static_cast<uint32_t>((_send.target * LENGTH18BIT) + LENGTH18BIT);
                if(duty > LENGTH19BIT - 1)      duty = LENGTH19BIT - 1;
                speed_buff[0] = (static_cast<uint8_t>(duty >> 11) & 0b11100000) | _md_num | 0b00010000;      //0b00010000のときはenable_dutyがtrueになる
                speed_buff[1] = static_cast<uint8_t>(duty >> 8);
                speed_buff[2] = static_cast<uint8_t>(duty);
            }else{
                //角速度を送るときはrad/sをdeg/sに変換して送る.
                uint32_t speed = static_cast<uint32_t>((degrees(_send.target) * LENGTH18BIT) + LENGTH18BIT);
                if(speed > LENGTH19BIT - 1)     speed = LENGTH19BIT - 1;
                speed_buff[0] = (static_cast<uint8_t>(speed >> 11) & 0b11100000) | _md_num | 0b0000000;      //0b00010000のときはenable_dutyがtrueになる
                speed_buff[1] = static_cast<uint8_t>(speed >> 8);
                speed_buff[2] = static_cast<uint8_t>(speed);
            }
            _can_driver.send(UPDATE_TARGET_ID, speed_buff);
            break;
        }
        case EncoderMode::DUTY_MODE:{
            std::vector<uint8_t> duty_buff(2);
            if(_send.target < -1.0f || _send.target > 1.0f){
                log_e("MD%d's target is out of range\r\n",_md_num);
                return;
            }
            uint16_t duty = static_cast<uint16_t>(_send.target * LENGTH10BIT) + LENGTH10BIT;
            if(duty > LENGTH11BIT - 1)      duty = LENGTH11BIT - 1;
            duty_buff[0] = (static_cast<uint8_t>(duty >> 3) & 0b11100000) | _md_num;    //5bit目が使われてない理由はlimitSWのデータが入るようにするため(実際に入ることはなかった).
            duty_buff[1] = static_cast<uint8_t>(duty);
            _can_driver.send(UPDATE_TARGET_ID, duty_buff);
            break;
        }
        case EncoderMode::POV_MODE:{
            //処理の内容はspeed_modeと同じ.
            std::vector<uint8_t> speed_buff(3);
            if(_send.enable_duty == true){
                if(_send.target < -1.0f || _send.target > 1.0f){
                    log_e("MD%d's target is out of range\r\n",_md_num);
                    return;
                }
                uint32_t duty = static_cast<uint32_t>((_send.target * LENGTH18BIT) + LENGTH18BIT);
                if(duty > LENGTH19BIT - 1)      duty = LENGTH19BIT - 1;
                speed_buff[0] = (static_cast<uint8_t>(duty >> 11) & 0b11100000) | _md_num | 0b00010000;      //0b00010000のときはenable_dutyがtrueになる
                speed_buff[1] = static_cast<uint8_t>(duty >> 8);
                speed_buff[2] = static_cast<uint8_t>(duty);
            }else{
                //角速度を送るときはrad/sをdeg/sに変換して送る.
                uint32_t speed = static_cast<uint32_t>((degrees(_send.target) * LENGTH18BIT) + LENGTH18BIT);
                if(speed > LENGTH19BIT - 1)     speed = LENGTH19BIT - 1;
                speed_buff[0] = (static_cast<uint8_t>(speed >> 11) & 0b11100000) | _md_num | 0b0000000;      //0b00010000のときはenable_dutyがtrueになる
                speed_buff[1] = static_cast<uint8_t>(speed >> 8);
                speed_buff[2] = static_cast<uint8_t>(speed);
            }
            _can_driver.send(UPDATE_TARGET_ID, speed_buff);
            break;
        }
    }
    _prev_enable_duty = _send.target;
    _prev_target = _send.target;
}

void YamaMDv3::_receiveTargetANDLimit(){
    switch(_init.enc_mode){
        case EncoderMode::ANGLE_MODE:{
            int16_t angle = static_cast<int16_t>(_my_receive_data.buff[0] & 0b11110000) << 4;
            angle |= _my_receive_data.buff[1];
            angle = angle - 1 + LENGTH11BIT;
            _receive.md_state = static_cast<float>(angle) / 4096.0f * 360.0f * DEG_TO_RAD;
            _receive.limit_sw_state = static_cast<bool>(_my_receive_data.buff[2]);
            break;
        }
        case EncoderMode::SPEED_MODE:{
            int32_t speed = static_cast<int32_t>(_my_receive_data.buff[0] & 0b11100000) << 11;
            speed |= static_cast<int32_t>(_my_receive_data.buff[1] << 8) | _my_receive_data.buff[2];
            speed = speed - LENGTH18BIT + 1;
            _receive.md_state = static_cast<float>(speed) * DEG_TO_RAD;
            if(_my_receive_data.buff[0] & 0b00010000)
                _receive.limit_sw_state = true;
            else
                _receive.limit_sw_state = false;
            break;
        }
    }
}

void YamaMDv3::_receiveOnlyTarget(){
    switch(_init.enc_mode){
        case EncoderMode::ANGLE_MODE:{
            int16_t angle = static_cast<int16_t>(_my_receive_data.buff[0] & 0b11110000) << 4;
            angle |= _my_receive_data.buff[1];
            angle = angle - 2047;
            _receive.md_state = static_cast<float>(angle) / 4096.0f * 360.0f * DEG_TO_RAD;
            break;
        }
        case EncoderMode::SPEED_MODE:{
            int32_t speed = static_cast<int32_t>(_my_receive_data.buff[0] & 0b11100000) << 11;
            speed |= static_cast<int32_t>(_my_receive_data.buff[1] << 8) | _my_receive_data.buff[2];
            speed = speed - LENGTH18BIT + 1;
            _receive.md_state = static_cast<float>(speed) * DEG_TO_RAD;
            break;
        }
        case EncoderMode::POV_MODE:{
            //処理はANGLE_MODEと同じ.
            int16_t angle = static_cast<int16_t>(_my_receive_data.buff[0] & 0b11110000) << 4;
            angle |= _my_receive_data.buff[1];
            angle = angle - 2047;
            _receive.md_state = static_cast<float>(angle) / 4096.0f * 360.0f * DEG_TO_RAD;
            break;
        }
    }
}

void YamaMDv3::init(PIDInit_t& pid_init, uint8_t dt, SelectMDSendMode select_md_send_mode, EncoderMode enc_mode, RotationDir motor_dir, RotationDir enc_dir){
    _init.kp.kp = pid_init.kp;
    _init.ki.ki = pid_init.ki;
    _init.kd.kd = pid_init.kd;
    _init.pid_limit = pid_init.pid_limit;
    _init.pid_min_and_max_abs = pid_init.pid_min_and_max_abs;
    _init.dt = dt;
    _init.select_md_send_mode = select_md_send_mode;
    _init.enc_mode = enc_mode;
    _init.motor_dir = motor_dir;
    _init.enc_dir = enc_dir;
    _sendInitData();
    stop();
}

void YamaMDv3::move(float target){
    _send.enable_duty = false;
    _send.target = target;
    _sendTarget();
}

void YamaMDv3::dutyMove(float target){
    _send.enable_duty = true;
    _send.target = target;
    _sendTarget();
}

void YamaMDv3::stop(){
    dutyMove(0.0f);
}

bool YamaMDv3::interruptReceiveTask(const can_device::CANReceiveData_t& rx_data){
    if(rx_data.id != MD_STATE_ID || (rx_data.buff[0] & 0b00001111) != _md_num){
        return true;
    }else{
        _my_receive_data.id = rx_data.id;
        _my_receive_data.buff = rx_data.buff;
        _my_receive_data.dlc = rx_data.dlc;
        _interrupt_flag = true;
        return false;
    }
}

void YamaMDv3::receiveTask(){
    if(_interrupt_flag == false || _init.select_md_send_mode == SelectMDSendMode::DATA_DISABLE)     return;
    switch(_init.select_md_send_mode){
        case SelectMDSendMode::TARGET_AND_LIMIT_SW:
            _receiveTargetANDLimit();
            break;
        case SelectMDSendMode::ONLY_TARGET:
            _receiveOnlyTarget();
            break;
        case SelectMDSendMode::ONLY_LIMIT_SW:
            if((_my_receive_data.buff[0] & 0b00010000) == 0b00010000)
                _receive.limit_sw_state = true;
            else
                _receive.limit_sw_state = false;
            break;
    }
}

}/*yamamdv3*/