/**
 * @file safety_interface.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-12-05
 */

#ifndef TP_INTERFACE_SAFETY_INTERFACE_H_
#define TP_INTERFACE_SAFETY_INTERFACE_H_
#include "fst_error.h"
#include <boost/thread/mutex.hpp>


typedef struct _InputByte4
{
    char brake1:1;
    char brake2:1;
    char brake3:1;
    char outage0:1;
    char outage1:1;
    char decelerate:1;
    char D6:1;
    char D7:1;
}InputByte4;

typedef struct _InputByte5
{
    char deadman_panic:1;
    char deadman_normal:1;
    char manual:1;
    char lmt_manual:1;
    char automatic:1;
    char estop:1;
    char lmt_stop:1;
    char ext_estop:1;
}InputByte5;

typedef struct _InputByte6
{
    char safety_door_stop:1;
    char contactor_feadback:1;
    char servo_alarm:1;
    char hw_reset:1;
    char D4:1;
    char D5:1;
    char D6:1;
    char D7:1;
}InputByte6;


typedef struct _OutputByte4
{
    char sw_alarm:1;
    char disble_brake:1;
    char D2:1;
    char D3:1;
    char D4:1;
    char D5:1;
    char D6:1;
    char D7:1;
}OutputByte4;

typedef struct _OutputByte5
{
    char safety_stop_config:1;
    char ext_estop_config:1;
    char lmt_stop_config:1;
    char sw_reset:1;
    char D4:1;
    char D5:1;
    char D6:1;
    char D7:1;
}OutputByte5;

typedef struct _SafetyBoardDIFrm1
{
    char        start_data;
    char        heart_beat;
    char        crc_data;
    InputByte4  byte4; 
}SafetyBoardDIFrm1;

typedef struct _SafetyBoardDIFrm2
{
    InputByte5  byte5;
    InputByte6  byte6;
    char        byte7;
    char        byte8;
}SafetyBoardDIFrm2;

typedef struct _SafetyBoardDOFrm1
{
    char        start_data;
    char        heart_beat;
    char        crc_data;
    OutputByte4 byte4; 
}SafetyBoardDOFrm1;

typedef struct _SafetyBoardDOFrm2
{
    OutputByte5 byte5;
    char        byte6;
    char        byte7;
    char        byte8;
}SafetyBoardDOFrm2;


class SafetyInterface
{
  public:
    SafetyInterface();
    ~SafetyInterface();

    /**
     * @brief: get input frame 1 
     *
     * @return 
     */
    U32 getDIFrm1();
    /**
     * @brief: get input frame 2
     *
     * @return 
     */
    U32 getDIFrm2();
    /**
     * @brief: get output frame 1 
     *
     * @return 
     */
    U32 getDOFrm1();
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
    
    /**
     * @brief: TP estop
     *
     * @return 
     */
    char getDIEStop();

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
    char getDIHWReset();

    /**
     * @brief: Servo alarm 
     *
     * @return 
     */
    char getDIServoAlarm();

    /**
     * @brief: feadback of another outage1 and outage0  
     *
     * @return 
     */
    char getDIContactorFeadback();
     
    /**
     * @brief: safety door stop 
     *
     * @return 
     */
    char getDISafetyDoorStop();

    /**
     * @brief: sofeware alarm from core0 
     *
     * @return 
     */
    char getDOSWAlarm();
    U64 setDOSWAlarm(char data);
    
    U64 setBrakeOn();  
    U64 setBrakeOff();
    /**
     * @brief: safety board stop config 
     * 0: class 0 stop; 1:class 1 stop
     *
     * @return 
     */
    char getDOSafetyStopConf();
    U64 setDOSafetyStopConf(char data);

    /**
     * @brief: ex_estop config 
     * 0: class 0 stop; 1:class 1 stop
     *
     * @return 
     */
    char getDOExtEStopConf();
    U64 setDOExtEStopConf(char data);

    /**
     * @brief: speed limited config
     * 0: class 0 stop; 1:class 1 stop
     * 
     * @return 
     */
    char getDOLmtStopConf();
    U64 setDOLmtStopConf(char data);

    /**
     * @brief: safety board sofeware reset 
     * 1s pulse high
     * 
     * @return 
     */
    U64 setSoftwareReset(char data);

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
  private:
    bool                valid_flag_;

    SafetyBoardDIFrm1    din_frm1_;
    SafetyBoardDIFrm2    din_frm2_;
    SafetyBoardDOFrm1    dout_frm1_;
    SafetyBoardDOFrm2    dout_frm2_;

    boost::mutex        mutex_;
};

#endif

