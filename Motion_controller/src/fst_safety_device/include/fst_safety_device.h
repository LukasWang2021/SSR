#ifndef FST_SAFETY_DEVICE_H
#define FST_SAFETY_DEVICE_H

#include "base_device.h"
#include "fst_safety_device_param.h"
#include "common_log.h"
#include "base_datatype.h"
#include <thread>
#include <mutex>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace fst_hal
{
	
//qianjin update with new register def
typedef struct _InputByte1
{
	char brake1:1;//brake base
	char brake2:1;//brake aux1
	char brake3:1;//brake aux2
	char contactor0:1;
	char contactor1:1;
	char D5:1;
	char D6:1;
	char D7:1;
}InputByte1;

typedef struct _InputByte2
{
	char core1_reset:1;
	char cabinet_reset:1;
	char user_reset:1;
	char decelerate:1;
	char usermode_man:1;
	char usermode_limit:1;
	char usermode_auto:1;
	char D7:1;
}InputByte2;

typedef struct _InputByte3
{
	char tp_estop:1;
	char lmt_stop:1;
	char ext_estop:1;
	char safetydoor_stop:1;
    char cabinet_f_estop:1;
	char deadman_normal:1;
	char deadman_panic:1;
    char dual_faulty:1;
}InputByte3;

typedef struct _InputByte4
{
	char mode_faulty:1;
    char brake1_relay_faulty:1;
    char brake2_relay_faulty:1;
    char brake3_relay_faulty:1;
    char contactor0_relay_faulty:1;
    char contactor1_relay_faulty:1;
	char contactor_faulty:1;
	char excitor_stop:1;
}InputByte4;

typedef struct _OutputByte1
{
	char core0_sw0:1;
	char core0_sw1:1;
	char core0_sw2:1;
	char safedoor_stop_config:1;
	char ext_estop_config:1;
	char lmt_stop_config:1;
	char D6:1;
	char D7:1;
}OutputByte1;


typedef struct _SafetyBoardDIFrm1
{
	InputByte1	byte1;
	InputByte2	byte2;
	InputByte3	byte3;
	InputByte4	byte4;
}SafetyBoardDIFrm1;

typedef struct _SafetyBoardDIFrm2
{
	char reserve1;
	char reserve2;
	char reserve3;
	char reserve4;
}SafetyBoardDIFrm2;

typedef struct _SafetyBoardDOFrm1
{
	OutputByte1 byte1;
	char		reserve2;
	char		reserve3;
	char		reserve4;
}SafetyBoardDOFrm1;

typedef struct _SafetyBoardDOFrm2
{
	char reserve1;
	char reserve2;
	char reserve3;
	char reserve4;
}SafetyBoardDOFrm2;
	
//#define RESET_SAFETY_DELAY      (200)   //delay for reset safety board  (ms)

class FstSafetyDevice : public BaseDevice
{
public:
    FstSafetyDevice(int address);
    ~FstSafetyDevice();

    virtual bool init();

    //  -----------------------------------------------------------------------
    //  Function:		getDIFrm1
    //  Description: get the DI status from the first 4 bytes.
    //  -----------------------------------------------------------------------
    uint32_t getDIFrm1(void);

    //  -----------------------------------------------------------------------
	//  Function:		isDIFrmChanged
	//  Description: check change of DI status.
	//  -----------------------------------------------------------------------
    bool isDIFrmChanged(void);

    //  -----------------------------------------------------------------------
	//  Function:		getDOFrm1
	//  Description: get the DO status from the first 4 bytes.
	//  -----------------------------------------------------------------------
    uint32_t getDOFrm1(void);

    //  -----------------------------------------------------------------------
	//  Function:		getDIBrake1
	//  Description: the brake on main axles 1-6
	//  -----------------------------------------------------------------------
    char getDIBrake1(void);

    //  -----------------------------------------------------------------------
	//  Function:		getDIBrake2
	//  Description: the brake on aux axle 1
	//  -----------------------------------------------------------------------
    char getDIBrake2(void);

    //  -----------------------------------------------------------------------
	//  Function:		getDIBrake3
	//  Description: the brake on aux axle 2
	//  -----------------------------------------------------------------------
    char getDIBrake3(void);

    //  -----------------------------------------------------------------------
	//  Function:		getDIContactor0
	//  Description: the contactor0 status.
	//  -----------------------------------------------------------------------
    char getDIContactor0(void);

	//  -----------------------------------------------------------------------
	//  Function:		getDIContactor1
	//  Description: the contactor1 status.
	//  -----------------------------------------------------------------------
    char getDIContactor1(void);

	//  -----------------------------------------------------------------------
	//  Function:		getDICore1Reset
	//  Description: the action Core1 reset.
	//  -----------------------------------------------------------------------
    char getDICore1Reset(void);

	//  -----------------------------------------------------------------------
	//  Function:		getCabinetReset
	//  Description: the reset from cabinet.
	//  -----------------------------------------------------------------------
    char getCabinetReset(void);

	//  -----------------------------------------------------------------------
	//  Function:		getUserReset
	//  Description: the action from user of cabinet.
	//  -----------------------------------------------------------------------
    char getUserReset(void);

    //  -----------------------------------------------------------------------
	//  Function:		getDIDec
	//  Description: the decelerating status.
	//  -----------------------------------------------------------------------
    char getDIDec(void);

	//  -----------------------------------------------------------------------
	//  Function:		getDITPManual
	//  Description: the TP mode is manual.
	//  -----------------------------------------------------------------------
    char getDITPManual(void);

	//  -----------------------------------------------------------------------
	//  Function:		getDITPLimitedManual
	//  Description: the TP mode is slowly manual.
	//  -----------------------------------------------------------------------
    char getDITPLimitedManual(void);

	//  -----------------------------------------------------------------------
	//  Function:		getDITPAuto
	//  Description: the TP mode is auto.
	//  -----------------------------------------------------------------------
    char getDITPAuto(void);

     //qianjin:add for mode
	//  -----------------------------------------------------------------------
	//  Function:		getDITPUserMode
	//  Description: the mode according the priority.
	//  Return: 1  ->  auto mode.
	//          3  ->  manual.
	//          2  ->  limited manual
	//  -----------------------------------------------------------------------
    int getDITPUserMode(void);

	//  -----------------------------------------------------------------------
	//  Function:		getDITPEStop
	//  Description: E-stop from TP.
	//  -----------------------------------------------------------------------
    char getDITPEStop(void);

	//  -----------------------------------------------------------------------
	//  Function:		getDILimitedStop
	//  Description: Stop from the limited position.
	//  -----------------------------------------------------------------------
    char getDILimitedStop(void);

	//  -----------------------------------------------------------------------
	//  Function:		getDIExtEStop
	//  Description: Stop from external signal.
	//  -----------------------------------------------------------------------
    char getDIExtEStop(void);

	//  -----------------------------------------------------------------------
	//  Function:		getDISafetyDoorStop
	//  Description: Stop from the safety door.
	//  -----------------------------------------------------------------------
    char getDISafetyDoorStop(void);

	//  -----------------------------------------------------------------------
	//  Function:		getDIDeadmanNormal
	//  Description: the deadman is in release status.
	//  -----------------------------------------------------------------------
    char getDIDeadmanNormal(void);

	//  -----------------------------------------------------------------------
	//  Function:		getDIDeadmanPanic
	//  Description: the deadman is in pressed status.
	//  -----------------------------------------------------------------------
    char getDIDeadmanPanic(void);

	//  -----------------------------------------------------------------------
	//  Function:		getDualFaulty
	//  Description: Differences in dual circuit.
	//  -----------------------------------------------------------------------
    char getDualFaulty(void);

	//  -----------------------------------------------------------------------
	//  Function:		getModeFaulty
	//  Description: the faulty status according auto/manual/limited manual.
	//  -----------------------------------------------------------------------
    char getModeFaulty(void);

	//  -----------------------------------------------------------------------
	//  Function:		getBrake1RelayFaulty
	//  Description: the faulty status from brake1 relay.
	//  -----------------------------------------------------------------------
    char getBrake1RelayFaulty(void);

	//  -----------------------------------------------------------------------
	//  Function:		getBrake2RelayFaulty
	//  Description: the faulty status from brake2 relay.
	//  -----------------------------------------------------------------------
    char getBrake2RelayFaulty(void);

	//  -----------------------------------------------------------------------
	//  Function:		getBrake3RelayFaulty
	//  Description: the faulty status from brake3 relay.
	//  -----------------------------------------------------------------------
    char getBrake3RelayFaulty(void);

	//  -----------------------------------------------------------------------
	//  Function:		getContactor0RelayFaulty
	//  Description: the faulty status from Contactor0.
	//  -----------------------------------------------------------------------
    char getContactor0RelayFaulty(void);

	//  -----------------------------------------------------------------------
	//  Function:		getContactor1RelayFaulty
	//  Description: the faulty status from Contactor1.
	//  -----------------------------------------------------------------------
    char getContactor1RelayFaulty(void);

	//  -----------------------------------------------------------------------
	//  Function:		getDIContactorFaulty
	//  Description: the contactor status.
	//  -----------------------------------------------------------------------
	char getDIContactorFaulty(void);

    //  -----------------------------------------------------------------------
	//  Function:		getExcitorStop
	//  Description: the request to stop excitor of motors.
	//  -----------------------------------------------------------------------
    char getExcitorStop(void);

    char getDIAlarm(void);//depressed soon

    //  -----------------------------------------------------------------------
	//  Function:		getDOType0Stop
	//  Description: get the value of type0 stop.
	//  -----------------------------------------------------------------------
    char getDOType0Stop(void);

	//  -----------------------------------------------------------------------
	//  Function:		setDOType0Stop
	//  Description: set type0 stop.
	//  -----------------------------------------------------------------------
    ErrorCode setDOType0Stop(char data);

	//  -----------------------------------------------------------------------
	//  Function:		getDOType1Stop
	//  Description: get the value of type1 stop.
	//  -----------------------------------------------------------------------
    char getDOType1Stop(void);

    //  -----------------------------------------------------------------------
	//  Function:		setDOType1Stop
	//  Description: set type1 stop.
	//  -----------------------------------------------------------------------
    ErrorCode setDOType1Stop(char data);

	//  -----------------------------------------------------------------------
	//  Function:		getDOType2Stop
	//  Description: get the value of type2 stop.
	//  -----------------------------------------------------------------------
    char getDOType2Stop(void);

	//  -----------------------------------------------------------------------
	//  Function:		setDOType2Stop
	//  Description: set type2 stop.
	//  ----------------------------------------------------------------------
    ErrorCode setDOType2Stop(char data);

	//  -----------------------------------------------------------------------
	//  Function:		getDOSafetyDoorStopConf
	//  Description: the configuration value of safety door stop.
	//  ----------------------------------------------------------------------
    char getDOSafetyDoorStopConf(void);

	//  -----------------------------------------------------------------------
	//  Function:		setDOSafetyDoorStopConf
	//  Description: set the configuration of safety door stop.
	//  ----------------------------------------------------------------------
    ErrorCode setDOSafetyDoorStopConf(char data);

	//  -----------------------------------------------------------------------
	//  Function:		getDOExtEStopConf
	//  Description: the configuration value of external stop.
	//  ----------------------------------------------------------------------
    char getDOExtEStopConf(void);

	//  -----------------------------------------------------------------------
	//  Function:		getDOExtEStopConf
	//  Description: the configuration value of external stop.
	//  ----------------------------------------------------------------------
    ErrorCode setDOExtEStopConf(char data);

	//  -----------------------------------------------------------------------
	//  Function:		getDOLmtStopConf
	//  Description: the configuration value of limited position stop.
	//  ----------------------------------------------------------------------
    char getDOLmtStopConf(void);

	//  -----------------------------------------------------------------------
	//  Function:		setDOLmtStopConf
	//  Description: set the configuration of limited position stop.
	//  ----------------------------------------------------------------------
    ErrorCode setDOLmtStopConf(char data);

	//  -----------------------------------------------------------------------
	//  Function:		reset
	//  Description: reset by clear stop commands.
	//  ----------------------------------------------------------------------
    void reset(void);

	//  -----------------------------------------------------------------------
	//  Function:	    isSafetyVirtual
	//  Description: is safety board existed.
	//  Return: false  -> safety board is existed.
	//          true -> virtual board.
	//  ----------------------------------------------------------------------
    bool isSafetyVirtual(void);

	//  -----------------------------------------------------------------------
	//  Function:		isExcitorStopRequest
	//  Description: is the excitor stop request valid?
	//  return:  true  -> the excitor stop request is valid.
	//           false -> invalid stop request
	//  ----------------------------------------------------------------------
	bool isExcitorStopRequest(void);
    //bool isSafetyAlarm();

	void getSafetyBoardVersion(int &version);


	
private:
    FstSafetyDeviceParam* param_ptr_;
    fst_log::Logger* log_ptr_;

    boost::thread update_thread_;
    std::mutex mutex_;  // data protection
    void startThread(void);
    void runThread(void); 
    void updateThreadFunc(void);      // calling data exchange
    ErrorCode updateSafetyData(void); // data exchange

    FstSafetyDevice();
	bool is_virtual_;
    std::atomic<SafetyBoardDIFrm1>  din_frm1_;
    std::atomic<SafetyBoardDOFrm1>  dout_frm1_;
};

}

#endif

