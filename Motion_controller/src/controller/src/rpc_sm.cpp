#include "controller_rpc.h"
#include "error_monitor.h"

using namespace fst_ctrl;


// "/rpc/controller/getUserOpMode"
void ControllerRpc::handleRpc0x00000C05(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Int32* data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getUserOpMode();
    data_ptr->error_code.data = SUCCESS;
    //recordLog(CONTROLLER_LOG, data_ptr->error_code.data, std::string("/rpc/controller/getUserOpMode"));
}

// "/rpc/controller/getRunningStatus"
void ControllerRpc::handleRpc0x00000AB3(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Int32* data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getRunningState();
    data_ptr->error_code.data = SUCCESS;
    //recordLog(CONTROLLER_LOG, data_ptr->error_code.data, std::string("/rpc/controller/getRunningStatus"));
}

// "/rpc/controller/getInterpreterStatus"
void ControllerRpc::handleRpc0x00016483(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Int32* data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getInterpreterState();
    data_ptr->error_code.data = SUCCESS;
    //recordLog(CONTROLLER_LOG, data_ptr->error_code.data, std::string("/rpc/controller/getInterpreterStatus"));
}

// "/rpc/controller/getRobotStatus"
void ControllerRpc::handleRpc0x00006F83(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Int32* data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getRobotState();
    data_ptr->error_code.data = SUCCESS;
    //recordLog(CONTROLLER_LOG, data_ptr->error_code.data, std::string("/rpc/controller/getRobotStatus"));
}

// "/rpc/controller/getCtrlStatus"
void ControllerRpc::handleRpc0x0000E9D3(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Int32* data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getCtrlState();
    data_ptr->error_code.data = SUCCESS;
    //recordLog(CONTROLLER_LOG, data_ptr->error_code.data, std::string("/rpc/controller/getCtrlStatus"));
}

// "/rpc/controller/getServoStatus"
void ControllerRpc::handleRpc0x0000D113(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Int32* data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getServoState();
    data_ptr->error_code.data = SUCCESS;
    //recordLog(CONTROLLER_LOG, data_ptr->error_code.data, std::string("/rpc/controller/getServoStatus"));
}

// "/rpc/controller/getSafetyAlarm"
void ControllerRpc::handleRpc0x0000C00D(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64_Int32* data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    data_ptr->data.data = (int32_t)state_machine_ptr_->getSafetyAlarm();
    data_ptr->error_code.data = SUCCESS;
    //recordLog(CONTROLLER_LOG, data_ptr->error_code.data, std::string("/rpc/controller/getSafetyAlarm"));
}

// "/rpc/controller/callEstop"
void ControllerRpc::handleRpc0x00013940(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    data_ptr->data.data = state_machine_ptr_->callEstop();
    //recordLog(CONTROLLER_LOG, data_ptr->data.data, std::string("/rpc/controller/callEstop"));
}

// "/rpc/controller/callReset"
void ControllerRpc::handleRpc0x000161E4(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
/*    if (state_machine_ptr_->getCtrlState() == CTRL_ENGAGED)
    {
        data_ptr->data.data = SUCCESS;
        fst_base::ErrorMonitor::instance()->add(INFO_RESET_SUCCESS);
        return;
    }*/
    data_ptr->data.data = state_machine_ptr_->callReset();
    //recordLog(CONTROLLER_LOG, data_ptr->data.data, std::string("/rpc/controller/callReset"));
}

// "/rpc/controller/setUserOpMode"
void ControllerRpc::handleRpc0x00002ED5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = state_machine_ptr_->setUserOpMode((fst_hal::UserOpMode)rq_data_ptr->data.data);

    //recordLog(CONTROLLER_LOG, rs_data_ptr->data.data, std::string("/rpc/controller/setUserOpMode"));
}

// "/rpc/controller/shutdown"
void ControllerRpc::handleRpc0x0000899E(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    rs_data_ptr->data.data = state_machine_ptr_->callShutdown();
    //recordLog(CONTROLLER_LOG, rs_data_ptr->data.data, std::string("/rpc/controller/shutdown"));
}

//"/rpc/controller/isBackupAvailable"	
void ControllerRpc::handleRpc0x00003EB5(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if(state_machine_ptr_->getCtrlState() != CTRL_ESTOP)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        return;
    }
    state_machine_ptr_->setState(false);
    rs_data_ptr->data.data = SUCCESS;
}

//"/rpc/controller/backupDone"	
void ControllerRpc::handleRpc0x000143E5(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    state_machine_ptr_->setState(true);
    rs_data_ptr->data.data = SUCCESS;
}

//"/rpc/controller/isRestoreAvailable"	
void ControllerRpc::handleRpc0x0000C7A5(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    if(state_machine_ptr_->getCtrlState() != CTRL_ESTOP)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        return;
    }
    state_machine_ptr_->setState(false);
    rs_data_ptr->data.data = SUCCESS;
}

//"/rpc/controller/restoreDone"	
void ControllerRpc::handleRpc0x000079F5(void* request_data_ptr, void* response_data_ptr)
{
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    state_machine_ptr_->setState(true);
    rs_data_ptr->data.data = SUCCESS;
}

//"/rpc/controller/setControllerIp"	
void ControllerRpc::handleRpc0x0000B090(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    //check state machine
    if(state_machine_ptr_->getCtrlState() != CTRL_ESTOP)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_ERROR("rpc-setControllerIp: invalid status = %d", state_machine_ptr_->getCtrlState());
        return;
    }

    //check value
    for (size_t i = 0; i < rq_data_ptr->data.data_count; ++i)
    {
        if (rq_data_ptr->data.data[i] > 255 || rq_data_ptr->data.data[i] < 0)
        {
            rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
            FST_ERROR("rpc-setControllerIp: invalid value[%d] = %d", i, rq_data_ptr->data.data[i]);
            return;
        }
    }
    char set_ip[256];
    if (rq_data_ptr->data.data_count == 4)
    {     
        sprintf(set_ip, "sh /root/install/lib/controller/setIP %d.%d.%d.%d", 
         rq_data_ptr->data.data[0], rq_data_ptr->data.data[1], rq_data_ptr->data.data[2], rq_data_ptr->data.data[3]);
    }
    else if (rq_data_ptr->data.data_count == 8)
    {
        sprintf(set_ip, "sh /root/install/lib/controller/setIP %d.%d.%d.%d %d.%d.%d.%d",
         rq_data_ptr->data.data[0], rq_data_ptr->data.data[1], rq_data_ptr->data.data[2], rq_data_ptr->data.data[3],
         rq_data_ptr->data.data[4], rq_data_ptr->data.data[5], rq_data_ptr->data.data[6], rq_data_ptr->data.data[7]);
    }
    else if (rq_data_ptr->data.data_count == 12)
    {
        sprintf(set_ip, "sh /root/install/lib/controller/setIP %d.%d.%d.%d %d.%d.%d.%d %d.%d.%d.%d",
         rq_data_ptr->data.data[0], rq_data_ptr->data.data[1], rq_data_ptr->data.data[2], rq_data_ptr->data.data[3],
         rq_data_ptr->data.data[4], rq_data_ptr->data.data[5], rq_data_ptr->data.data[6], rq_data_ptr->data.data[7], 
         rq_data_ptr->data.data[8], rq_data_ptr->data.data[9], rq_data_ptr->data.data[10], rq_data_ptr->data.data[11]);
    }
    else
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_ERROR("rpc-setControllerIp: invalid value number = %d", rq_data_ptr->data.data_count);
        return;
    }

    int res = system(set_ip);

    if (res != 0)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        FST_ERROR("rpc-setControllerIp: Failed with %d (%s)", res, set_ip);
        return;
    }

    rs_data_ptr->data.data = SUCCESS;
    FST_INFO("rpc-setControllerIp success: %s", set_ip);
}