/**********************************************
Copyright 漏 2016 Foresight-Robotics Ltd. All rights reserved.
File:       fst_safety_device.cpp
Author:     Feng.Wu 
Create:     25-Sep-2018
Modify:     22-Oct-2018
Summary:    dealing with safety board
**********************************************/
#include <unistd.h>
#include <memory.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "fst_safety_mem.h"
#include "fst_safety_device.h"
#include "error_monitor.h"

using namespace fst_hal;
using namespace fst_base;

FstSafetyDevice::FstSafetyDevice(int address):
    BaseDevice(address, fst_hal::DEVICE_TYPE_FST_SAFETY),
    log_ptr_(NULL),
    param_ptr_(NULL),
    is_virtual_(false),
    pre_dual_faulty_(0),
	pre_ext_estop_(0),
	pre_door_stop_(0),
	pre_limited_stop_(0),
	pre_deadman_normal_(1),//for initial 
	pre_deadman_panic_(0),
	pre_tp_estop_(0),
	pre_mode_faulty_(0),
	pre_contactor_faulty_(0),
	pre_main_brake_relay_(0),
	pre_brake1_relay_(0),
	pre_brake2_relay_(0),
	pre_contactor0_relay_(0),
	pre_contactor1_relay_(0),
	pre_cabinet_stop_(0),
    pre_comm_err_(1)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new FstSafetyDeviceParam();
    FST_LOG_INIT("fst_safety_device");
}

FstSafetyDevice::~FstSafetyDevice()
{
    if (is_virtual_ == false)
    {
        update_thread_.interrupt();
        update_thread_.join();
        closeSafety();
    } 

    if(log_ptr_ != NULL) {
        delete log_ptr_;
        log_ptr_ = NULL;
    }

    if(param_ptr_ != NULL) {
        delete param_ptr_;
        param_ptr_ = NULL;
    }
}

bool FstSafetyDevice::init()
{
    memset((char*)&din_frm1_, 0, sizeof(din_frm1_));
    memset((char*)&dout_frm1_, 0, sizeof(dout_frm1_));
    memset((char*)&din_frm2_, 0, sizeof(din_frm2_));

    ErrorCode ret = 0;
    if(!param_ptr_->loadParam()){
        FST_WARN("Failed to load safety component parameters");
    } else {
        FST_LOG_SET_LEVEL((fst_log::MessageLevel)param_ptr_->log_level_);
        is_virtual_ = param_ptr_->is_virtual_;
    }
   
    if (is_virtual_ == false){
        ret = openSafety();
        FST_INFO("Use True Safety");
        if (ret != 0){
            FST_ERROR("init FstSafetyDevice failed");
            ErrorMonitor::instance()->add(ret);
            return false; 
        }
        startThread();
        setValid(true);
    } else {
        FST_INFO("Use Fake Safety");
        return true;
    }
	
    return true;
}

FstSafetyDevice::FstSafetyDevice():
    BaseDevice(0, fst_hal::DEVICE_TYPE_INVALID)
{
}

uint32_t FstSafetyDevice::getDIFrm1()
{
    return *(uint32_t*)&din_frm1_;
}

uint32_t FstSafetyDevice::getDIFrm2()
{
    return *(uint32_t*)&din_frm2_;
}

bool FstSafetyDevice::isDIFrmChanged()
{
    static uint32_t pre_frame_data;
    uint32_t frm_data = getDIFrm1();
    if (pre_frame_data == frm_data)
    {
        return false;
    }
    
    pre_frame_data = frm_data;
    return true;
}

uint32_t FstSafetyDevice::getDOFrm1()
{
    return *(uint32_t*)&dout_frm1_;
}

//------------- input byte1------------------//
char FstSafetyDevice::getDIMainBrake()
{
    return din_frm1_.load().byte1.main_brake;
}
char FstSafetyDevice::getDIBrake1()
{
    return din_frm1_.load().byte1.brake1;
}
char FstSafetyDevice::getDIBrake2()
{
    return din_frm1_.load().byte1.brake2;
}
char FstSafetyDevice::getDIContactor0()
{
    return din_frm1_.load().byte1.contactor0;
}
char FstSafetyDevice::getDIContactor1()
{
    return din_frm1_.load().byte1.contactor1;
}

//------------input byte2---------------------//
char FstSafetyDevice::getDICore1Reset()
{
    return din_frm1_.load().byte2.core1_reset;
}
char FstSafetyDevice::getCabinetReset()
{
    return din_frm1_.load().byte2.cabinet_reset;
}
char FstSafetyDevice::getUserReset()
{
    return din_frm1_.load().byte2.user_reset;
}
char FstSafetyDevice::getDIDec()
{
    return din_frm1_.load().byte2.decelerate;
}
char FstSafetyDevice::getDITPManual()
{
    return din_frm1_.load().byte2.usermode_man;
}
char FstSafetyDevice::getDITPLimitedManual()
{
    return din_frm1_.load().byte2.usermode_limit;
}
char FstSafetyDevice::getDITPAuto()
{
    return din_frm1_.load().byte2.usermode_auto;
}
//qianjin add for user mode
int FstSafetyDevice::getDITPUserMode()
{
    int val = 0;
    if( din_frm1_.load().byte2.usermode_auto) val |= 0x1;
    if( din_frm1_.load().byte2.usermode_limit) val |= 0x2;
    if( din_frm1_.load().byte2.usermode_man) val |= 0x4;
    

    //FST_INFO("getDITPUserMode: safety_interface_ :: din_frm1_: %08X", *(uint32_t*)&din_frm1_);
	//FST_INFO("getDITPUserMode: safety_interface_ :: val: %d", val);

    if(val == 0x1) return USER_OP_MODE_AUTO; //auto mode
    if(val == 0x2) return USER_OP_MODE_SLOWLY_MANUAL; //limit manual mode
    if(val == 0x4) return USER_OP_MODE_MANUAL; //unlimit manual mode
    return USER_OP_MODE_NONE;
}

char FstSafetyDevice::getDICommError(void)
{
    return din_frm1_.load().byte2.comm_err;
}

//----------------input byte3----------------------//
char FstSafetyDevice::getDITPEStop()
{
    return din_frm1_.load().byte3.tp_estop;
}
char FstSafetyDevice::getDILimitedStop()
{
    return din_frm1_.load().byte3.lmt_stop;
}
char FstSafetyDevice::getDIExtEStop()
{
    return din_frm1_.load().byte3.ext_estop;
}
char FstSafetyDevice::getDISafetyDoorStop()
{
    return din_frm1_.load().byte3.safetydoor_stop;
}

char FstSafetyDevice::getDICabinetStop(void)
{
    return din_frm1_.load().byte3.cabinet_estop;
}
char FstSafetyDevice::getDIDeadmanNormal()
{
    return din_frm1_.load().byte3.deadman_normal;
}
char FstSafetyDevice::getDIDeadmanPanic()
{
    return din_frm1_.load().byte3.deadman_panic;
}
char FstSafetyDevice::getDualFaulty()
{
    return din_frm1_.load().byte3.dual_faulty;
}

//----------------input byte4----------------//
char FstSafetyDevice::getModeFaulty()
{
    return din_frm1_.load().byte4.mode_faulty; 
}
char FstSafetyDevice::getMainBrakeRelayFaulty()
{
    return din_frm1_.load().byte4.main_brake_relay_faulty;
}
char FstSafetyDevice::getBrake1RelayFaulty()
{
    return din_frm1_.load().byte4.brake1_relay_faulty;
}
char FstSafetyDevice::getBrake2RelayFaulty()
{
    return din_frm1_.load().byte4.brake2_relay_faulty;
}
char FstSafetyDevice::getContactor0RelayFaulty()
{
    return din_frm1_.load().byte4.contactor0_relay_faulty;
}
char FstSafetyDevice::getContactor1RelayFaulty()
{
    return din_frm1_.load().byte4.contactor1_relay_faulty;
}
char FstSafetyDevice::getDIContactorFaulty(void)
{
    return din_frm1_.load().byte4.contactor_faulty;
}
char FstSafetyDevice::getExcitorStop()
{
    return din_frm1_.load().byte4.excitor_stop; 
}

//---------------output byte1----------------//
char FstSafetyDevice::getDOType0Stop()
{
    return dout_frm1_.load().byte1.core0_sw0;
}
ErrorCode FstSafetyDevice::setDOType0Stop(char data)
{
    if (isSafetyVirtual() == true){
        FST_WARN("setDOType0Stop set 0x%x", data);
        return SUCCESS;
    }
    SafetyBoardDOFrm1 out = dout_frm1_;
    out.byte1.core0_sw0 = data;
    FST_WARN("setDOType0Stop set 0x%X", *(int*)&out);
    return setSafety(*(int*)&out, SAFETY_OUTPUT_FIRSTFRAME);
}

char FstSafetyDevice::getDOType1Stop()
{
    return dout_frm1_.load().byte1.core0_sw1;
}
ErrorCode FstSafetyDevice::setDOType1Stop(char data)
{
    if (isSafetyVirtual() == true){
        FST_WARN("setDOType1Stop set 0x%x", data);
        return SUCCESS;
    }
    SafetyBoardDOFrm1 out = dout_frm1_;
    out.byte1.core0_sw1 = data;
    FST_WARN("setDOType1Stop set 0x%X", *(int*)&out);
    return setSafety(*(int*)&out, SAFETY_OUTPUT_FIRSTFRAME);
}

char FstSafetyDevice::getDOType2Stop()
{
    return dout_frm1_.load().byte1.core0_sw2;
}
ErrorCode FstSafetyDevice::setDOType2Stop(char data)
{
    if (isSafetyVirtual() == true){
        FST_WARN("setDOType2Stop set 0x%x", data);
        return SUCCESS;
    }
    SafetyBoardDOFrm1 out = dout_frm1_;
    out.byte1.core0_sw2 = data;
    FST_WARN("setDOType2Stop set 0x%X", *(int*)&out);
    return setSafety(*(int*)&out, SAFETY_OUTPUT_FIRSTFRAME);
}

char FstSafetyDevice::getDOSafetyDoorStopConf()
{
    return dout_frm1_.load().byte1.safedoor_stop_config;
}
ErrorCode FstSafetyDevice::setDOSafetyDoorStopConf(char data)
{
    if (isSafetyVirtual() == true){
        // FST_INFO("setDOSafetyStopConf Error set %c", data);
        return SUCCESS;
    }
    SafetyBoardDOFrm1 out = dout_frm1_;
    out.byte1.safedoor_stop_config = data;
    FST_INFO("setDOSafetyDoorStopConf set 0x%X", *(int*)&out);
    return setSafety(*(int*)&out, SAFETY_OUTPUT_FIRSTFRAME);
}

char FstSafetyDevice::getDOExtEStopConf()
{
    return dout_frm1_.load().byte1.ext_estop_config;
}
ErrorCode FstSafetyDevice::setDOExtEStopConf(char data)
{
    if (isSafetyVirtual() == true){
        // FST_INFO("setDOExtEStopConf Error set %c", data);
        return SUCCESS;
    }
    SafetyBoardDOFrm1 out = dout_frm1_;
    out.byte1.ext_estop_config = data;
    FST_INFO("setDOExtEStopConf set 0x%X", *(int*)&out);
    return setSafety(*(int*)&out, SAFETY_OUTPUT_FIRSTFRAME);
}

char FstSafetyDevice::getDOLmtStopConf()
{
    return dout_frm1_.load().byte1.lmt_stop_config;
}
ErrorCode FstSafetyDevice::setDOLmtStopConf(char data)
{
    if (isSafetyVirtual() == true){
        // FST_INFO("setDOLmtStopConf Error set %c", data);
        return SUCCESS;
    }
    SafetyBoardDOFrm1 out = dout_frm1_;
    out.byte1.lmt_stop_config = data;
    FST_INFO("setDOLmtStopConf set 0x%X", *(int*)&out);
    return setSafety(*(int*)&out, SAFETY_OUTPUT_FIRSTFRAME);
}

void FstSafetyDevice::reset()
{
    setDOType0Stop(0);
    setDOType1Stop(0);
    setDOType2Stop(0);
    pre_dual_faulty_ = 0;
	pre_ext_estop_ = 0;
	pre_door_stop_ = 0;
	pre_limited_stop_ = 0;
	pre_deadman_normal_ = 0;
	pre_deadman_panic_ = 0;
	pre_tp_estop_ = 0;
	pre_mode_faulty_ = 0;
	pre_contactor_faulty_ = 0;
	pre_main_brake_relay_ = 0;
	pre_brake1_relay_ = 0;
	pre_brake2_relay_ = 0;
	pre_contactor0_relay_ = 0;
	pre_contactor1_relay_ = 0;
	pre_cabinet_stop_ = 0;
}

bool FstSafetyDevice::isSafetyVirtual()
{
    return is_virtual_;
}
bool FstSafetyDevice::isCabinetResetRequest()
{
    static char pre_value = 1;
    char current_value = getCabinetReset();    
    if ((pre_value == 0) && (current_value == 1))
    {
        pre_value = current_value;
        return true;
    }

    pre_value = current_value;
    return false;
}


void FstSafetyDevice::getSafetyBoardVersion(int &version)
{
    if (is_virtual_ == false)
    {
        getSafetyBoardVersionFromMem(&version);
    }
    else
    {
        version = 0;
    }
}


bool FstSafetyDevice::getAutoModeDo(uint32_t &port_offset, uint8_t &value)
{
    // if setting is 0, do not set DO output.
    if (param_ptr_->auto_mode_DO_ == 0)
    {
        return false;
    }

    port_offset = param_ptr_->auto_mode_DO_;

    int mode = getDITPUserMode();
    if (mode == USER_OP_MODE_AUTO)
    {
        value = 1;
    }
    else
    {
        value = 0;
    }
    return true;
}

bool FstSafetyDevice::getLimitedManualModeDo(uint32_t &port_offset, uint8_t &value)
{
    // if setting is 0, do not set DO output.
    if (param_ptr_->limited_manual_mode_DO_ == 0)
    {
        return false;
    }

    port_offset = param_ptr_->limited_manual_mode_DO_;

    int mode = getDITPUserMode();
    if (mode == USER_OP_MODE_SLOWLY_MANUAL)
    {
        value = 1;
    }
    else
    {
        value = 0;
    }
    return true;
}

bool FstSafetyDevice::getManualModeDo(uint32_t &port_offset, uint8_t &value)
{
    // if setting is 0, do not set DO output.
    if (param_ptr_->manual_mode_DO_ == 0)
    {
        return false;
    }

    port_offset = param_ptr_->manual_mode_DO_;

    int mode = getDITPUserMode();
    if (mode == USER_OP_MODE_MANUAL)
    {
        value = 1;
    }
    else
    {
        value = 0;
    }
    return true;
}


//generate the error codes from the external component(safety_board)
bool FstSafetyDevice::checkSafetyBoardAlarm(void)
{
    char current_value = 0;
    bool ret = false;

    current_value = getDualFaulty();
    ret |= isRisingEdge(current_value, SAFETY_BOARD_RELAY_DUAL_FAULTY, pre_dual_faulty_);

    current_value = getDIExtEStop();
    ret |= isRisingEdge(current_value, SAFETY_BOARD_EXTERNAL_STOP, pre_ext_estop_);
    
    current_value = getDISafetyDoorStop();
    ret |= isRisingEdge(current_value, SAFETY_BOARD_SAFETY_DOOR_STOP, pre_door_stop_);

    current_value = getDILimitedStop();
    ret |= isRisingEdge(current_value, SAFETY_BOARD_LIMITED_STOP, pre_limited_stop_);

    current_value = getDIDeadmanPanic();
    ret |= isRisingEdge(current_value, SAFETY_BOARD_DEADMAN_PANIC, pre_deadman_panic_);

    current_value = getDITPEStop();
    ret |= isRisingEdge(current_value, SAFETY_BOARD_TP_ESTOP, pre_tp_estop_);

    //current_value = getModeFaulty();
    //ret |= isRisingEdge(current_value, SAFETY_BOARD_OP_MODE_FAULTY, pre_mode_faulty_);

    current_value = getDIContactorFaulty();
    ret |= isRisingEdge(current_value, SAFETY_BOARD_MAIN_CONTACTOR_FAULTY, pre_contactor_faulty_);

    current_value = getMainBrakeRelayFaulty();
    ret |= isRisingEdge(current_value, SAFETY_BOARD_MAIN_BRAKE_RELAY_FAULTY, pre_main_brake_relay_);

    current_value = getBrake1RelayFaulty();
    ret |= isRisingEdge(current_value, SAFETY_BOARD_AUX_BRAKE_RELAY_ONE_FAULTY, pre_brake1_relay_);

    current_value = getBrake2RelayFaulty();
    ret |= isRisingEdge(current_value, SAFETY_BOARD_AUX_BRAKE_RELAY_TWO_FAULTY, pre_brake2_relay_);

    current_value = getContactor0RelayFaulty();
    ret |= isRisingEdge(current_value, SAFETY_BOARD_CONTACTOR_RELAY_ZERO_FAULTY, pre_contactor0_relay_);
    
    current_value = getContactor1RelayFaulty();
    ret |= isRisingEdge(current_value, SAFETY_BOARD_CONTACTOR_RELAY_ONE_FAULTY, pre_contactor1_relay_);
    
    current_value = getDICabinetStop();
    ret |= isRisingEdge(current_value, SAFETY_BOARD_CABINET_STOP, pre_cabinet_stop_);

    //comm error 
    current_value = getDICommError();
    ret |= isRisingEdge(current_value, SAFETY_BOARD_COMM_ERROR, pre_comm_err_);

    return ret;
}

ErrorCode FstSafetyDevice::checkDeadmanNormal(void)
{
    char current_value = getDIDeadmanNormal();
    if (pre_deadman_normal_ == 0 && current_value == 1)
    {
        pre_deadman_normal_ = current_value;
        return SAFETY_BOARD_DEADMAN_NORMAL_FAULTY;
    }
    pre_deadman_normal_ = current_value;
    return SUCCESS;
}

bool FstSafetyDevice::isRisingEdge(char value, ErrorCode code, char &pre_value)
{
    if (pre_value == 0 && value == 1)
    {
        pre_value = value;
        ErrorMonitor::instance()->add(code);
        return true;
    }
    pre_value = value;
    return false;
}

//------------------------------------------------------------
// Function:    startThread
// Summary: start a thread.
// In:      None.
// Out:     None.
// Return:  None.
//------------------------------------------------------------
void FstSafetyDevice::startThread(void)
{
    update_thread_ = boost::thread(boost::bind(&FstSafetyDevice::runThread, this));
    //update_thread_.timed_join(boost::posix_time::milliseconds(100)); // check the thread running or not.
}

//------------------------------------------------------------
// Function:    runThread
// Summary: main function of io thread.
// In:      None.
// Out:     None.
// Return:  None.
//------------------------------------------------------------
void FstSafetyDevice::runThread(void)
{
    try
    {   FST_INFO("safety thread running...");
        while (true)
        {
            updateThreadFunc();

            // set interruption point.
            boost::this_thread::sleep(boost::posix_time::microseconds(param_ptr_->cycle_time_));// add  using  thread_help...
        }
    }
    catch (boost::thread_interrupted &)
    {
        std::cout<<"~Stop safety update thread Thread Safely.~"<<std::endl;
    }
}

void FstSafetyDevice::updateThreadFunc()
{
    updateSafetyData();
}

ErrorCode FstSafetyDevice::updateSafetyData()
{
    static ErrorCode pre_err = SUCCESS;
    uint32_t data;
    uint32_t data2;
    ErrorCode result = autorunSafetyData(); 
    if (result == SUCCESS){
        result = getSafety(&data, SAFETY_INPUT_FIRSTFRAME);
        if (result == SUCCESS)
            memcpy((char*)&din_frm1_, (char*)&data, sizeof(uint32_t));

        result = getSafety(&data2, SAFETY_INPUT_SECONDFRAME);
        if (result == SUCCESS)
            memcpy((char*)&din_frm2_, (char*)&data2, sizeof(uint32_t));
    }else{
        if (pre_err != result){
            ErrorMonitor::instance()->add(result);
        }
    }
    pre_err = result;
    return result;
}
