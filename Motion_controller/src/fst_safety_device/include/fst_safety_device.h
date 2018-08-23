#ifndef FST_SAFETY_DEVICE_H
#define FST_SAFETY_DEVICE_H

#include "base_device.h"
#include "fst_safety_device_param.h"
#include "common_log.h"
#include <thread>
#include <mutex>

namespace fst_hal
{
	
typedef unsigned int U32;
typedef unsigned long long int U64;
typedef struct _Core1Status
{
	char id:4;
	char status:4;
}Core1Status;
//qianjin update with new register def
typedef struct _InputByte5
{
	char brake1:1;//brake base
	char brake2:1;//brake aux1
	char brake3:1;//brake aux2
	char D5:1;
	char D6:1;
	char D7:1;
	char outage0:1;
	char outage1:1;
}InputByte5;

typedef struct _InputByte6
{
	char core1_reset:1;
	char user_reset:1;
	char cabinet_reset:1;
	char decelerate:1;
	char usermode_man:1;
	char usermode_limit:1;
	char usermode_auto:1;
	//char manual:1;
	//char slowdown:1;
	//char lmt_manual:1;
	//char D5:1;
	//char automatic:1;
	//char D6:1;
	char D7:1;
}InputByte6;

typedef struct _InputByte7
{
	char tp_estop:1;
	char safetydoor_stop:1;
	char lmt_stop:1;
	char ext_estop:1;
	char deadman_normal:1;
	char deadman_panic:1;
	char mode_signal:1;
	char contactor:1;
}InputByte7;

typedef struct _InputByte8
{
	char core0_alarm:1;
	char core1_alarm:1;
	char D2:1;
	char D3:1;
	char D4:1;
	char D5:1;
	char D6:1;
	char alarming:1;
}InputByte8;



typedef struct _OutputByte5
{
	char core0_sw0:1;
	char core0_sw1:1;
	char core0_sw2:1;
	char safedoor_stop_config:1;
	char ext_estop_config:1;
	char lmt_stop_config:1;
	char D6:1;
	char D7:1;
}OutputByte5;


typedef struct _SafetyBoardDIFrm1
{
	char		start_data;
	Core1Status core1_status;
	char		heart_beat;
	char		crc_data;
}SafetyBoardDIFrm1;

typedef struct _SafetyBoardDIFrm2
{
	InputByte5	byte5;
	InputByte6	byte6;
	InputByte7	byte7;
	InputByte8	byte8;
}SafetyBoardDIFrm2;

typedef struct _SafetyBoardDOFrm1
{
	char		start_data;
	char		byte2;
	char		heart_beat;
	char		crc_data;
}SafetyBoardDOFrm1;

typedef struct _SafetyBoardDOFrm2
{
	OutputByte5 byte5;
	char		byte6;
	char		byte7;
	char		byte8;
}SafetyBoardDOFrm2;
	
#define RESET_SAFETY_DELAY      (200)   //delay for reset safety board  (ms)
	


class FstSafetyDevice : public BaseDevice
{
public:
    FstSafetyDevice(int address);
    ~FstSafetyDevice();

    virtual bool init();

    //various API to control or monitor...

    
    /**
     * @brief: get input frame 2
     *
     * @return 
     */
    U32 getDIFrm2();
    
    /**
     * @brief 
     *
     * @return 
     */
    bool isDIFrmChanged();
    /**
     * @brief: get output frame 2
     *
     * @return 
     */
    U32 getDOFrm2();

    /**
     * @brief: motor deceleration state(class 0 pause) 
     *
     * @return
     */
    char getDIDec();

    /**
     * @brief:mother line contactor1  
     *
     * @return 
     */
    char getDIOutage1();

    /**
     * @brief: mother line contactor0
     * when contactor0 and contactor1 are both 1
     * the feadback should be 1
     *
     * @return 
     */
    char getDIOutage0();

    /**
     * @brief:the eighth axis brake 
     *
     * @return 
     */
    char getDIBrake3();

    /**
     * @brief: the seventh axis brake 
     *
     * @return 
     */
    char getDIBrake2();

    /**
     * @brief: first six axises brake 
     *
     * @return 
     */
    char getDIBrake1();

    /**
     * @brief: deadman in deepest position 
     *
     * @return 
     */
    char getDIDeadmanPanic();

    /**
     * @brief: deadman in normal position 
     *
     * @return 
     */
    char getDIDeadmanNormal();

    /**
     * @brief: TP in manual state 
     *
     * @return 
     */
    char getDITPManual();

    /**
     * @brief: TP in speed limited state 
     *
     * @return 
     */
    char getDITPLimitedManual();

    /**
     * @brief: TP in auto state 
     *
     * @return 
     */
    char getDITPAuto();
    
    /*
     * qianjin:add for mode 
     *
     *
     */
    int getDITPUserMode();
    /**
     * @brief: TP estop
     *
     * @return 
     */
    char getDITPEStop();

    /**
     * @brief: in hardware limit
     *
     * @return 
     */
    char getDILimitedStop();

    /**
     * @brief: external estop 
     *
     * @return 
     */
    char getDIExtEStop();

    /**
     * @brief: safety board reset 
     *
     * @return 
     */
    char getDICore1Reset();

    /**
     * @brief: Servo alarm 
     *
     * @return 
     */
    char getDICore1Alarm();

    /**
     * @brief: feadback of another outage1 and outage0  
     *
     * @return 
     */
    char getDIAlarm();
     
    /**
     * @brief: safety door stop 
     *
     * @return 
     */
    char getDISafetyDoorStop();

    /**
     * @brief: type 0 stop from core0 
     *
     * @return 
     */
    char getDOType0Stop();

    /**
     * @brief: estop 0 
     *
     * @param data
     *
     * @return 
     */
    U64 setDOType0Stop(char data);

    /**
     * @brief: type 0 stop from core0 
     *
     * @return 
     */
    char getDOType1Stop();

    /**
     * @brief: estop 1 
     *
     * @param data
     *
     * @return 
     */
    U64 setDOType1Stop(char data);

    /**
     * @brief: type 0 stop from core0 
     *
     * @return 
     */
    char getDOType2Stop();

    /**
     * @brief 
     *
     * @param data
     *
     * @return 
     */
    U64 setDOType2Stop(char data);
    
    /**
     * @brief: safety board stop config 
     * 0: class 0 stop; 1:class 1 stop
     *
     * @return 
     */
    char getDOSafetyStopConf();

    /**
     * @brief 
     *
     * @param data
     *
     * @return 
     */
    U64 setDOSafetyStopConf(char data);

    /**
     * @brief: ex_estop config 
     * 0: class 0 stop; 1:class 1 stop
     *
     * @return 
     */
    char getDOExtEStopConf();

    /**
     * @brief 
     *
     * @param data
     *
     * @return 
     */
    U64 setDOExtEStopConf(char data);

    /**
     * @brief: speed limited config
     * 0: class 0 stop; 1:class 1 stop
     * 
     * @return 
     */
    char getDOLmtStopConf();

    /**
     * @brief 
     *
     * @param data
     *
     * @return 
     */
    U64 setDOLmtStopConf(char data);

    /**
     * @brief 
     */
    void reset();

    /**
     * @brief: heart_beat with safety board 
     *
     * @return 
     */
    U64 setSafetyHeartBeat();

    /**
     * @brief 
     *
     * @return 
     */
    bool isSafetyValid();

    /**
     * @brief 
     *
     * @return 
     */
    bool isSafetyAlarm();
private:
    FstSafetyDeviceParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    int input[8];   //maybe 8 bytes?
    int output[8];  //maybe 8 bytes?

    std::thread update_thread_;
    std::mutex mutex_;  // data protection

    FstSafetyDevice();
    void updateThreadFunc();    // data exchange 
  private:
    std::atomic<bool>               valid_flag_;

    std::atomic<SafetyBoardDIFrm2>  din_frm2_;
    std::atomic<SafetyBoardDOFrm2>  dout_frm2_;
};

}

#endif

