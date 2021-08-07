#ifndef YAMAMDV3_H_
#define YAMAMDV3_H_

#include <Arduino.h>
#include <vector>
#include "CANDriver.h"
#include "esp_log.h"

namespace yamamdv3{

//0x000はなんとなく使いたくないからなし
const uint32_t SET_KP_ID = 0x001;
const uint32_t SET_KI_ID = 0x002;
const uint32_t SET_KD_ID = 0x003;
const uint32_t SET_DT_AND_MODE_ID = 0x004;
const uint32_t UPDATE_TARGET_ID = 0x005;
const uint32_t MD_STATE_ID = 0x006;
const uint32_t MD_RESET_ID = 0x007;

enum class EncoderMode{
	ANGLE_MODE = 0,
	SPEED_MODE = 1,
	DUTY_MODE = 2,
    POV_MODE = 3,
    STEER_MODE = 4
};

enum class RotationDir{
	REGULAR_DIR = 1,	//通常の方向
	NEGATIVE_DIR = -1	//通常とは逆
};

enum class SelectMDSendMode{
	TARGET_AND_LIMIT_SW = 0,
	ONLY_TARGET = 1,
	ONLY_LIMIT_SW = 2,
	DATA_DISABLE = 3
};

typedef struct{
    float kp;
    float ki;
    float kd;
    float pid_min_and_max_abs;
    uint16_t pid_limit;
}PIDInit_t;

typedef struct{
	union{
		float kp;
		uint8_t data[4];
	}kp;
	union{
		float ki;
		uint8_t data[4];
	}ki;
	union{
		float kd;
		uint8_t data[4];
	}kd;
    float pid_min_and_max_abs;
    uint16_t pid_limit;         //ワインドアップ対策用
	uint8_t dt;		//[ms]
    uint16_t origin_angle;      //raw_angle時の角度(0~4095)
	RotationDir motor_dir;
	RotationDir enc_dir;
	EncoderMode enc_mode;
	SelectMDSendMode select_md_send_mode;		//MDがmotherに送信するデータを選択.
}MDInitData_t;

typedef struct{
	float target;
	bool enable_duty;   //trueなら強制的にdutyをon,falseならoff
}MotherSendData_t;

typedef struct{
    float md_state;
    bool limit_sw_state;
}MotherReceiveData_t;

class YamaMDv3{
    private:
        can_device::CANDriver& _can_driver;
        can_device::CANReceiveData_t _my_receive_data;
        MDInitData_t _init;
        MotherSendData_t _send;
        MotherReceiveData_t _receive;
        const uint8_t _md_num = 0;
        std::vector<uint8_t> _prev_buff;
        float _prev_target;
        bool _prev_enable_duty;
        bool _interrupt_flag;   //割り込みの処理が行われてReceiveTaskを実行する必要があるときはTrueになる
        static constexpr uint16_t A3921_PWM_RESOLUTION = 2048;
        static constexpr uint16_t A3921_PWM_HALF_RESOLUTION = 1024;
        static constexpr uint16_t LENGTH10BIT = 1024;
        static constexpr uint16_t LENGTH11BIT = 2048;
        static constexpr uint16_t LENGTH12BIT = 4096;
        static constexpr uint32_t LENGTH18BIT = 262144;
        static constexpr uint32_t LENGTH19BIT = 524288;
        void _sendInitData();
        void _sendTarget();
        void _receiveTargetANDLimit();
        void _receiveOnlyTarget();
    public:
        YamaMDv3(can_device::CANDriver& can_driver, uint8_t md_num)
            :_can_driver{can_driver}, _md_num{md_num}{_prev_target = 1234.5678f;}
        //initの関数でencの方向があってないとmdでデータを受け取ったときに方向が反転するはず.
        void init(PIDInit_t& pid_init, uint8_t dt, uint16_t origin_angle, SelectMDSendMode select_md_send_mode, EncoderMode enc_mode, RotationDir motor_dir, RotationDir enc_dir);
        void move(float target);
        void dutyMove(float target);
        void stop();
        bool interruptReceiveTask(const can_device::CANReceiveData_t& rx_data);
        void receiveTask();
        const float getMDState() const {return _receive.md_state;}
        const bool getLimitSWState() const {return _receive.limit_sw_state;}
};


}/*yamamdv3*/

#endif /*YAMAMDV3_H_*/