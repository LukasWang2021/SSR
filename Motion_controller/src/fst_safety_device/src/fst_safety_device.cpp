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
    is_virtual_(false)
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
char FstSafetyDevice::getDIBrake1()
{
    return din_frm1_.load().byte1.brake1;
}
char FstSafetyDevice::getDIBrake2()
{
    return din_frm1_.load().byte1.brake2;
}
char FstSafetyDevice::getDIBrake3()
{
    return din_frm1_.load().byte1.brake3;
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
    int val;
    val = 0;
    if( din_frm1_.load().byte2.usermode_auto) val |= 0x1;
    if( din_frm1_.load().byte2.usermode_man) val |= 0x2;
    if( din_frm1_.load().byte2.usermode_limit) val |= 0x4;

    //FST_INFO("getDITPUserMode: safety_interface_ :: din_frm1_: %08X", *(uint32_t*)&din_frm1_);
	//FST_INFO("getDITPUserMode: safety_interface_ :: val: %d", val);

    if(val == 0x1) return 1; //auto mode
    if(val == 0x2) return 3; //manual mode
    if(val == 0x4) return 2; //limit mode
    return 0;
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
char FstSafetyDevice::getBrake1RelayFaulty()
{
    return din_frm1_.load().byte4.brake1_relay_faulty;
}
char FstSafetyDevice::getBrake2RelayFaulty()
{
    return din_frm1_.load().byte4.brake2_relay_faulty;
}
char FstSafetyDevice::getBrake3RelayFaulty()
{
    return din_frm1_.load().byte4.brake3_relay_faulty;
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
        // FST_INFO("setDOType0Stop Error set %c\n", data);
        return SUCCESS;
    }
    SafetyBoardDOFrm1 out = dout_frm1_;
    out.byte1.core0_sw0 = data;
    FST_INFO("setDOType0Stop set 0x%X\n", *(int*)&out);
    return setSafety(*(int*)&out, SAFETY_OUTPUT_FIRSTFRAME);
}

char FstSafetyDevice::getDOType1Stop()
{
    return dout_frm1_.load().byte1.core0_sw1;
}
ErrorCode FstSafetyDevice::setDOType1Stop(char data)
{
    if (isSafetyVirtual() == true){
        // FST_INFO("setDOType1Stop Error set %c\n", data);
        return SUCCESS;
    }
    SafetyBoardDOFrm1 out = dout_frm1_;
    out.byte1.core0_sw1 = data;
    FST_INFO("setDOType1Stop set 0x%X\n", *(int*)&out);
    return setSafety(*(int*)&out, SAFETY_OUTPUT_FIRSTFRAME);
}

char FstSafetyDevice::getDOType2Stop()
{
    return dout_frm1_.load().byte1.core0_sw2;
}
ErrorCode FstSafetyDevice::setDOType2Stop(char data)
{
    if (isSafetyVirtual() == true){
        // FST_INFO("setDOType2Stop Error set %c\n", data);
        return SUCCESS;
    }
    SafetyBoardDOFrm1 out = dout_frm1_;
    out.byte1.core0_sw2 = data;
    FST_INFO("setDOType2Stop set 0x%X\n", *(int*)&out);
    return setSafety(*(int*)&out, SAFETY_OUTPUT_FIRSTFRAME);
}

char FstSafetyDevice::getDOSafetyDoorStopConf()
{
    return dout_frm1_.load().byte1.safedoor_stop_config;
}
ErrorCode FstSafetyDevice::setDOSafetyDoorStopConf(char data)
{
    if (isSafetyVirtual() == true){
        // FST_INFO("setDOSafetyStopConf Error set %c\n", data);
        return SUCCESS;
    }
    SafetyBoardDOFrm1 out = dout_frm1_;
    out.byte1.safedoor_stop_config = data;
    FST_INFO("setDOSafetyDoorStopConf set 0x%X\n", *(int*)&out);
    return setSafety(*(int*)&out, SAFETY_OUTPUT_FIRSTFRAME);
}

char FstSafetyDevice::getDOExtEStopConf()
{
    return dout_frm1_.load().byte1.ext_estop_config;
}
ErrorCode FstSafetyDevice::setDOExtEStopConf(char data)
{
    if (isSafetyVirtual() == true){
        // FST_INFO("setDOExtEStopConf Error set %c\n", data);
        return SUCCESS;
    }
    SafetyBoardDOFrm1 out = dout_frm1_;
    out.byte1.ext_estop_config = data;
    FST_INFO("setSafety set 0x%X\n", *(int*)&out);
    return setSafety(*(int*)&out, SAFETY_OUTPUT_FIRSTFRAME);
}

char FstSafetyDevice::getDOLmtStopConf()
{
    return dout_frm1_.load().byte1.lmt_stop_config;
}
ErrorCode FstSafetyDevice::setDOLmtStopConf(char data)
{
    if (isSafetyVirtual() == true){
        // FST_INFO("setDOLmtStopConf Error set %c\n", data);
        return SUCCESS;
    }
    SafetyBoardDOFrm1 out = dout_frm1_;
    out.byte1.lmt_stop_config = data;
    FST_INFO("setSafety set 0x%X\n", *(int*)&out);
    return setSafety(*(int*)&out, SAFETY_OUTPUT_FIRSTFRAME);
}

void FstSafetyDevice::reset()
{
    setDOType0Stop(0);
    setDOType1Stop(0);
    setDOType2Stop(0);
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

//generate the error codes from the external component(safety_board)
void FstSafetyDevice::checkSafetyBoardAlarm(void)
{
    char current_value = 0;

    static char pre_dual_faulty = 0;
    current_value = getDualFaulty();
    isRisingEdge(current_value, SAFETY_BOARD_RELAY_DUAL_FAULTY, pre_dual_faulty);

    static char pre_ext_estop = 0;
    current_value = getDIExtEStop();
    isRisingEdge(current_value, SAFETY_BOARD_EXTERNAL_STOP, pre_ext_estop);
    
    static char pre_door_stop = 0;
    current_value = getDISafetyDoorStop();
    isRisingEdge(current_value, SAFETY_BOARD_SAFETY_DOOR_STOP, pre_door_stop);

    static char pre_limited_stop = 0;
    current_value = getDILimitedStop();
    isRisingEdge(current_value, SAFETY_BOARD_LIMITED_STOP, pre_limited_stop);

    static char pre_deadman_normal = 0;
    current_value = getDIDeadmanNormal();
    isRisingEdge(current_value, SAFETY_BOARD_DEADMAN_NORMAL_FAULTY, pre_deadman_normal);

    static char pre_deadman_panic = 0;
    current_value = getDIDeadmanPanic();
    isRisingEdge(current_value, SAFETY_BOARD_DEADMAN_PANIC, pre_deadman_panic);

    static char pre_tp_estop = 0;
    current_value = getDITPEStop();
    isRisingEdge(current_value, SAFETY_BOARD_TP_ESTOP, pre_tp_estop);

    static char pre_mode_faulty = 0;
    current_value = getModeFaulty();
    isRisingEdge(current_value, SAFETY_BOARD_OP_MODE_FAULTY, pre_mode_faulty);

    static char pre_contactor_faulty = 0;
    current_value = getDIContactorFaulty();
    isRisingEdge(current_value, SAFETY_BOARD_MAIN_CONTACTOR_FAULTY, pre_contactor_faulty);

    static char pre_brake1_relay = 0;
    current_value = getBrake1RelayFaulty();
    isRisingEdge(current_value, SAFETY_BOARD_MAIN_BRAKE_RELAY_FAULTY, pre_brake1_relay);

    static char pre_brake2_relay = 0;
    current_value = getBrake2RelayFaulty();
    isRisingEdge(current_value, SAFETY_BOARD_AUX_BRAKE_RELAY_ONE_FAULTY, pre_brake2_relay);

    static char pre_brake3_relay = 0;
    current_value = getBrake3RelayFaulty();
    isRisingEdge(current_value, SAFETY_BOARD_AUX_BRAKE_RELAY_TWO_FAULTY, pre_brake3_relay);

    static char pre_contactor0_relay = 0;
    current_value = getContactor0RelayFaulty();
    isRisingEdge(current_value, SAFETY_BOARD_CONTACTOR_RELAY_ZERO_FAULTY, pre_contactor0_relay);
    
    static char pre_contactor1_relay = 0;
    current_value = getContactor1RelayFaulty();
    isRisingEdge(current_value, SAFETY_BOARD_CONTACTOR_RELAY_ONE_FAULTY, pre_contactor1_relay);
    
    static char pre_cabinet_stop = 0;
    current_value = getDICabinetStop();
    isRisingEdge(current_value, SAFETY_BOARD_CABINET_STOP, pre_cabinet_stop);
}

inline void FstSafetyDevice::isRisingEdge(char value, ErrorCode code, char &pre_value)
{
    if (pre_value == 0 && value == 1)
    {
        ErrorMonitor::instance()->add(code);
    }
    pre_value = value;
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
    // for test only.
    //static int count = 0;
    ErrorCode result = autorunSafetyData(); 
    if (result == SUCCESS){
        result = getSafety(&data, SAFETY_INPUT_FIRSTFRAME);
        if (result == SUCCESS)
            memcpy((char*)&din_frm1_, (char*)&data, sizeof(uint32_t));

        // for test only.
        /* count++;
        if (count >= 5000)
            count = 0;
        if (count == 0){
            printf("\nwrite safety[0] = 0x%x.\n", data);
        }
        data = getSafety(SAFETY_INPUT_FIRSTFRAME, &result);
        if (count == 0){
            printf("recv safety[0] = 0x%x.\n", data);
        }
        data = getSafety(SAFETY_INPUT_SECONDFRAME, &result);
        if (count == 0){
            printf("recv safety[1] = 0x%x.\n", data);
        }
        */
    }else{
        if (pre_err != result){
            ErrorMonitor::instance()->add(result);
            pre_err = result;
        }
    }
    return result;
}
