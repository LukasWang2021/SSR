#ifndef FST_SAFETY_DEVICE_H
#define FST_SAFETY_DEVICE_H

#include "base_device.h"
#include "fst_safety_device_param.h"
#include "common_log.h"
#include <mutex>
#include "thread_help.h"

namespace fst_hal
{
	
//qianjin update with new register def
typedef struct _InputByte1
{
	char main_brake:1;//main brake
	char brake1:1;//brake aux1
	char brake2:1;//brake aux2
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
	char comm_err:1; //communication between FPGA and MCU
}InputByte2;

typedef struct _InputByte3
{
	char tp_estop:1;
	char lmt_stop:1;
	char ext_estop:1;
	char safetydoor_stop:1;
    char cabinet_estop:1;
	char deadman_normal:1;
	char deadman_panic:1;
    char dual_faulty:1;
}InputByte3;

typedef struct _InputByte4
{
	char mode_faulty:1;
    char main_brake_relay_faulty:1;
    char brake1_relay_faulty:1;
    char brake2_relay_faulty:1;
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
	uint32_t data;
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

typedef enum
{
    USER_OP_MODE_NONE             = 0,
    USER_OP_MODE_AUTO             = 1,
    USER_OP_MODE_SLOWLY_MANUAL    = 2,
    USER_OP_MODE_MANUAL = 3,
}UserOpMode;
	

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

	uint32_t getDIFrm2(void);

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
	//  Function:		getDIMainBrake
	//  Description: the brake on main axles 1-6
	//  -----------------------------------------------------------------------
    char getDIMainBrake(void);

    //  -----------------------------------------------------------------------
	//  Function:		getDIBrake1
	//  Description: the brake on aux axle 1
	//  -----------------------------------------------------------------------
    char getDIBrake1(void);

    //  -----------------------------------------------------------------------
	//  Function:		getDIBrake2
	//  Description: the brake on aux axle 2
	//  -----------------------------------------------------------------------
    char getDIBrake2(void);

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

	char getDICommError(void);

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

	char getDICabinetStop(void);

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
	//  Function:		getMainBrakeRelayFaulty
	//  Description: the faulty status from main brake relay.
	//  -----------------------------------------------------------------------
    char getMainBrakeRelayFaulty(void);

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
	//  Function:		isCabinetResetRequest
	//  Description: is the request valid?
	//  return:  true  -> the request is valid.
	//           false -> invalid request
	//  ----------------------------------------------------------------------
	bool isCabinetResetRequest(void);

    //  -----------------------------------------------------------------------
	//  Function:		getSafetyBoardVersion
	//  Description: get safety_board version
	//  return:  None
	//  ----------------------------------------------------------------------
	void getSafetyBoardVersion(int &version);

	//  -----------------------------------------------------------------------
	//  Function:		get*ModeDo
	//  Description: get the DO output according to the user mode.
	//  return:  true -> DO is set.
	//           bool -> DO is not set.
	//  ----------------------------------------------------------------------
	bool getAutoModeDo(uint32_t &port_offset, uint8_t &value);
	bool getLimitedManualModeDo(uint32_t &port_offset, uint8_t &value);
	bool getManualModeDo(uint32_t &port_offset, uint8_t &value);

    //check safety_board
	bool checkSafetyBoardAlarm(void);

	//check deadman normal
	ErrorCode checkDeadmanNormal(void);

    void routineThreadFunc(void);      // calling data exchange
	bool isRunning();
	
private:
    bool isRisingEdge(char value, ErrorCode code, char &pre_value);

    FstSafetyDeviceParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    fst_base::ThreadHelp routine_thread_;
	bool is_running_;
    std::mutex mutex_;  // data protection
    ErrorCode updateSafetyData(void); // data exchange

    FstSafetyDevice();
	bool is_virtual_;
    std::atomic<SafetyBoardDIFrm1>  din_frm1_;
    std::atomic<SafetyBoardDOFrm1>  dout_frm1_;
	std::atomic<SafetyBoardDIFrm2>  din_frm2_;

	//safety_alarm
	char pre_dual_faulty_;
	char pre_ext_estop_;
	char pre_door_stop_;
	char pre_limited_stop_;
	char pre_deadman_normal_;
	char pre_deadman_panic_;
	char pre_tp_estop_;
	char pre_mode_faulty_;
	char pre_contactor_faulty_;
	char pre_main_brake_relay_;
	char pre_brake1_relay_;
	char pre_brake2_relay_;
	char pre_contactor0_relay_;
	char pre_contactor1_relay_;
	char pre_cabinet_stop_;

	//comm error safety_alarm
	char pre_comm_err_;
};
}

void safetyDeviceRoutineThreadFunc(void* arg);

#endif

