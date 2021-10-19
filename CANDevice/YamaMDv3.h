/**
 * @file YamaMDv3.h
 * @author Yamaguchi Yudai
 * @brief CAN通信を用いたYamaMDとの通信プログラム
 * @date 2021-08-15
 */

#ifndef YAMAMDV3_H_
#define YAMAMDV3_H_

#include <Arduino.h>
#include <vector>
#include "CANDriver.h"
#include "esp_log.h"

/**
 * @brief 
 * YamaMDv3用の名前空間
 */
namespace yamamdv3{

//0x000はなんとなく使いたくないからなし
const uint32_t SET_KP_ID = 0x001;
const uint32_t SET_KI_ID = 0x002;
const uint32_t SET_KD_ID = 0x003;
const uint32_t SET_DT_AND_MODE_ID = 0x004;
const uint32_t UPDATE_TARGET_ID = 0x005;
const uint32_t MD_STATE_ID = 0x006;
const uint32_t MD_RESET_ID = 0x007;

/**
 * @brief 
 * ANGLE_MODEの場合
 * 目標値は角度[rad]で目標値を与えると、MD側でいい感じに角度を合わせてくれる。
 * SelectMDSendModeでTargetを有効にした場合は、MDからエンコーダの角度が送られてくる。(-pi~pi)
 * SPEED_MODEの場合
 * 目標値は角速度[rad/s]で目標値を与えると、MD側でいい感じに角速度を合わせてくれる。
 * SelectMDSendModeでTargetを有効にした場合は、MDから角速度が送られてくる。
 * DUTY_MODEの場合
 * 目標値はduty比(-1.0~1.0)で目標値を与えると、MD側ではdutyに応じてモーターを動かしてくれる。
 * SelectMDSendModeはDataDisableのみ対応。
 * POV_MODEの場合
 * 目標値は角度[rad/s]で目標値を与えると、MD側でいい感じに角速度を合わせてくれる。
 * SelectMDSendModeはONLY_TARGETにのみ対応していて、MDからはモータの角度が送られてくる。
 */
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

/**
 * @brief 
 * MDがマザーに送ってくるデータを選択する。     
 * 基本的にはDATA_DISABLE推奨
 */
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

    /**
     * @brief 
     * ワインドアップ対策用
     */
    uint16_t pid_limit;
	uint8_t dt;		//[ms]

    /**
     * @brief 
     * raw_angle時の角度(0~4095)
     */
    uint16_t origin_angle;
	RotationDir motor_dir;
	RotationDir enc_dir;
	EncoderMode enc_mode;

    /**
     * @brief 
     * MDがmotherに送信するデータを選択
     */
	SelectMDSendMode select_md_send_mode;
}MDInitData_t;

typedef struct{
	float target;

    /**
     * @brief 
     * trueならモードに関わらず強制的にdutyをon,falseならoff
     */
	bool enable_duty;
}MotherSendData_t;

/**
 * @brief 
 * md_stateにはSelectMDSendModeでtargetを有効にした場合、角度や角速度などEncoderModeにあった値が入る
 */
typedef struct{
    float md_state;
    bool limit_sw_state;
}MotherReceiveData_t;

class YamaMDv3{
    private:
        can_device::CANDriver& _can_driver;

        /**
         * @brief 
         * CANで受信した生の値が入る。
         * 新しいデータがMDから送られてくると処理が正業に行われていれば更新される。
         */
        can_device::CANReceiveData_t _my_receive_data;
        MDInitData_t _init;
        MotherSendData_t _send;
        MotherReceiveData_t _receive;

        const uint8_t _md_num = 0;
        std::vector<uint8_t> _prev_buff;
        float _prev_target;
        bool _prev_enable_duty;

        uint32_t _last_update_time;

        //doubleの理由はハードウェア割り込み中に動かすため
        float _prev_mdstate;

        /**
         * @brief 
         * 割り込みの処理が行われてReceiveTaskを実行する必要があるときはTrueになる
         */
        bool _interrupt_flag;
        
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

        inline float checkOutOfRange(float target, float min_range, float max_range){
            
            if(target > max_range)
                target = max_range;
            else if(target < min_range)
                target = min_range;

            return target;
        }
    public:

        YamaMDv3(can_device::CANDriver& can_driver, uint8_t md_num)
            :_can_driver{can_driver}, _md_num{md_num}{_prev_target = 1234.5678f;}
        
        /**
         * @brief initの関数でencの方向があってないとmdでデータを受け取ったときに方向が反転するはず.
         * 
         * @param pid_init pidのゲインなどはこの構造体
         * @param dt ms,基本的には5か10が入る。
         * @param origin_angle origin_angleを設定する必要がないときは0にする
         * @param select_md_send_mode 
         * @param enc_mode 
         * @param motor_dir 
         * @param enc_dir 関係ない場合は、REGULAR_DIRにする。
         */
        void init(PIDInit_t& pid_init, uint8_t dt, uint16_t origin_angle, SelectMDSendMode select_md_send_mode, EncoderMode enc_mode, RotationDir motor_dir, RotationDir enc_dir);
        
        /**
         * @brief 
         * ANGLE_MODE:pi~pi[rad]の範囲で角度を与える。      
         * SPEED_MODE:角速度[rad/s]を与える。       
         * DUTY_MODE:1.0~1.0の範囲で動かしたいdutyを与える。        
         * POV_MODE:角速度[rad/s]を与える。     
         * @param target 目標値 
         */
        void move(const float& target);

        /**
         * @brief 
         * MODEを問わず、強制的にdutyで動かす
         * @param target duty-1.0~1.0
         */
        void dutyMove(const float& target);

        /**
         * @brief 
         * モータを止める関数
         * モータにはトルクがかかってるので、無理やり動かそうとしても動かないので注意
         */
        void stop();

        /**
         * @brief 
         * CANの割り込み発生時に行う関数
         * @param rx_data 
         * @return true     trueの場合は受信したデータが関係ないとき
         * @return false    falseの場合は受信したデータが自分のものであるとき
         */
        bool interruptReceiveTask(const can_device::CANReceiveData_t& rx_data);

        /**
         * @brief 
         * メインループで回す関数。
         * CANの割り込み発生時に生じたデータの処理を行う。
         * 
         */
        void receiveTask();

        const uint32_t& getLastUpdateTime() const {return _last_update_time;}
        const float& getPrevMDState() const {return _prev_mdstate;}
        const float& getMDState() const {return _receive.md_state;}
        const bool& getLimitSWState() const {return _receive.limit_sw_state;}
};

}/*yamamdv3*/

#endif /*YAMAMDV3_H_*/