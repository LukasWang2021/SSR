#include "system/servo_comm_base.h"
#include <assert.h>
#include <string.h>

using namespace std;
using namespace servo_comm_space;

ServoCommBase::ServoCommBase(int32_t controller_id, int32_t servo_id, int32_t servo_index)
{
    comm_ptr_ = createServoCommByController(controller_id, servo_id, servo_index);
}
                  
ServoCommBase::~ServoCommBase()
{
    freeServoComm(comm_ptr_);
}

bool ServoCommBase::prepareInit2PreOp(CommBlockData_t* from_block_ptr, size_t from_block_number, CommBlockData_t* to_block_ptr, size_t to_block_number)
{
    return initServoCommInit2PreOpByController(comm_ptr_, from_block_ptr, from_block_number, to_block_ptr, to_block_number);
}

bool ServoCommBase::preparePreOp2SafeOp(CommBlockData_t* to_block_ptr, size_t to_block_number, int32_t fdb_pdo_app_id)
{
    return initServoCommPreOp2SafeOpByController(comm_ptr_, to_block_ptr, to_block_number, fdb_pdo_app_id);
}

bool ServoCommBase::prepareSafeOp2Op(CommBlockData_t* from_block_ptr, size_t from_block_number, int32_t ctrl_pdo_app_id)
{
    return initServoCommSafeOp2OpByController(comm_ptr_, from_block_ptr, from_block_number, ctrl_pdo_app_id);
}

CoreCommState_e ServoCommBase::getCommState()
{
    return getServoCommState(comm_ptr_);    
}

bool ServoCommBase::emitServoCmdShutDown()
{
    CoreProcessCallAppData1000_t req_data;
    memset(&req_data, 0, sizeof(req_data));
    req_data.cmd = SERVO_CMD_SHUT_DOWN;
    return doServoCmdFastService(comm_ptr_->service_ptr, &req_data);
}

bool ServoCommBase::emitServoCmdSwitchOn()
{
    CoreProcessCallAppData1000_t req_data;
    memset(&req_data, 0, sizeof(req_data));
    req_data.cmd = SERVO_CMD_SWITCH_ON;
    return doServoCmdFastService(comm_ptr_->service_ptr, &req_data);
}

bool ServoCommBase::emitServoCmdDisableVoltage()
{
    CoreProcessCallAppData1000_t req_data;
    memset(&req_data, 0, sizeof(req_data));
    req_data.cmd = SERVO_CMD_DISABLE_VOLTAGE;
    return doServoCmdFastService(comm_ptr_->service_ptr, &req_data);
}

bool ServoCommBase::emitServoCmdEnableOperation()
{
    CoreProcessCallAppData1000_t req_data;
    memset(&req_data, 0, sizeof(req_data));
    req_data.cmd = SERVO_CMD_ENABLE_OPERATION;
    return doServoCmdFastService(comm_ptr_->service_ptr, &req_data);
}

bool ServoCommBase::emitServoCmdSwitchOnAndEnableOperation()
{
    CoreProcessCallAppData1000_t req_data;
    memset(&req_data, 0, sizeof(req_data));
    req_data.cmd = SERVO_CMD_SWITCH_ON_AND_ENABLE_OPERATION;
    return doServoCmdFastService(comm_ptr_->service_ptr, &req_data);
}

bool ServoCommBase::emitServoCmdDisableOperation()
{
    CoreProcessCallAppData1000_t req_data;
    memset(&req_data, 0, sizeof(req_data));
    req_data.cmd = SERVO_CMD_DISABLE_OPERATION;
    return doServoCmdFastService(comm_ptr_->service_ptr, &req_data);
}

bool ServoCommBase::emitServoCmdQuickStop()
{
    CoreProcessCallAppData1000_t req_data;
    memset(&req_data, 0, sizeof(req_data));
    req_data.cmd = SERVO_CMD_QUICK_STOP;
    return doServoCmdFastService(comm_ptr_->service_ptr, &req_data);
}

bool ServoCommBase::emitServoCmdFaultReset()
{
    CoreProcessCallAppData1000_t req_data;
    memset(&req_data, 0, sizeof(req_data));
    req_data.cmd = SERVO_CMD_FAULT_RESET;
    return doServoCmdFastService(comm_ptr_->service_ptr, &req_data);
}

bool ServoCommBase::emitServoCmdResetFaultReset()
{
    CoreProcessCallAppData1000_t req_data;
    memset(&req_data, 0, sizeof(req_data));
    req_data.cmd = SERVO_CMD_RESET_FAULT_RESET;
    return doServoCmdFastService(comm_ptr_->service_ptr, &req_data);
}

ErrorCode ServoCommBase::doServoCmdTransCommState(CoreCommState_e expected_state)
{
    CoreProcessCallAppData1000_t req_data, res_data;
    memset(&req_data, 0, sizeof(req_data));
    memset(&res_data, 0, sizeof(res_data));
    req_data.cmd = SERVO_CMD_TRANS_COMM_STATE;
    req_data.param1 = (int32_t)expected_state;
    bool ret = doServoCmdNormalService(comm_ptr_->service_ptr, &req_data, &res_data);
    if(!ret)
    {
        return CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED;
    }
    if(res_data.param1 != 0)
    {
        return CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED;
    }
    return SUCCESS;
}

ErrorCode ServoCommBase::doServoCmdReadParameter(int32_t param_index, int32_t* param_value_ptr)
{
    CoreProcessCallAppData1000_t req_data, res_data;
    memset(&req_data, 0, sizeof(req_data));
    memset(&res_data, 0, sizeof(res_data));
    req_data.cmd = SERVO_CMD_READ_PARAMETER;
    req_data.param1 = param_index;
    bool ret = doServoCmdNormalService(comm_ptr_->service_ptr, &req_data, &res_data);
    if(!ret)
    {
        return CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED;
    }
    if(res_data.param1 != 0)
    {
        return CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED;
    }
    *param_value_ptr = res_data.param2;
    return SUCCESS;
}

ErrorCode ServoCommBase::doServoCmdWriteParameter(int32_t param_index, int32_t param_value)
{
    CoreProcessCallAppData1000_t req_data, res_data;
    memset(&req_data, 0, sizeof(req_data));
    memset(&res_data, 0, sizeof(res_data));
    req_data.cmd = SERVO_CMD_WRITE_PARAMETER;
    req_data.param1 = param_index;
    req_data.param2 = param_value;
    bool ret = doServoCmdNormalService(comm_ptr_->service_ptr, &req_data, &res_data);
    if(!ret)
    {
        return CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED;
    }
    if(res_data.param1 != 0)
    {
        return CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED;
    }
    return SUCCESS;
}

ErrorCode ServoCommBase::doServoCmdMoveVelocity(int32_t target_velocity, int32_t acc, int32_t dec, int32_t jerk, int32_t direction)
{
    CoreProcessCallAppData1000_t req_data, res_data;
    memset(&req_data, 0, sizeof(req_data));
    memset(&res_data, 0, sizeof(res_data));
    req_data.cmd = SERVO_CMD_MOVE_VELOCITY;
    req_data.param1 = target_velocity;
    req_data.param2 = acc;
    req_data.param3 = dec;
    req_data.param4 = jerk;
    req_data.param5 = direction;
    bool ret = doServoCmdNormalService(comm_ptr_->service_ptr, &req_data, &res_data);
    if(!ret)
    {
        return CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED;
    }
    if(res_data.param1 != 0)
    {
        return CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED;
    }
    return SUCCESS;
}

ErrorCode ServoCommBase::doServoCmdMoveAbsolute(int64_t target_position, int32_t target_velocity, int32_t acc, int32_t dec, int32_t jerk)
{
    CoreProcessCallAppData1000_t req_data, res_data;
    memset(&req_data, 0, sizeof(req_data));
    memset(&res_data, 0, sizeof(res_data));
    req_data.cmd = SERVO_CMD_MOVE_ABSOLUTE;
    req_data.param1 = (target_position & 0xFFFFFFFF);//lsb
    req_data.param2 = ((target_position >> 32) & 0xFFFFFFFF);//msb
    req_data.param3 = target_velocity;
    req_data.param4 = acc;
    req_data.param5 = dec;
    req_data.param6 = jerk;
    bool ret = doServoCmdNormalService(comm_ptr_->service_ptr, &req_data, &res_data);
    if(!ret)
    {
        return CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED;
    }
    if(res_data.param1 != 0)
    {
        return CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED;
    }
    return SUCCESS;
}

ErrorCode ServoCommBase::doServoCmdMoveRelative(int64_t target_position, int32_t target_velocity, int32_t acc, int32_t dec, int32_t jerk)
{
    CoreProcessCallAppData1000_t req_data, res_data;
    memset(&req_data, 0, sizeof(req_data));
    memset(&res_data, 0, sizeof(res_data));
    req_data.cmd = SERVO_CMD_MOVE_RELATIVE;
    req_data.param1 = (target_position & 0xFFFFFFFF);//lsb
    req_data.param2 = ((target_position >> 32) & 0xFFFFFFFF);//msb
    req_data.param3 = target_velocity;
    req_data.param4 = acc;
    req_data.param5 = dec;
    req_data.param6 = jerk;
    bool ret = doServoCmdNormalService(comm_ptr_->service_ptr, &req_data, &res_data);
    if(!ret)
    {
        return CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED;
    }
    if(res_data.param1 != 0)
    {
        return CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED;
    }
    return SUCCESS;
}

ErrorCode ServoCommBase::doServoCmdHoming(void)
{
    return CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED;
}

ErrorCode ServoCommBase::doServoCmdAbortHoming(void)
{
    return CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED;
}

ErrorCode ServoCommBase::doServoCmdHalt(void)
{
    CoreProcessCallAppData1000_t req_data, res_data;
    memset(&req_data, 0, sizeof(req_data));
    memset(&res_data, 0, sizeof(res_data));
    req_data.cmd = SERVO_CMD_HALT;
    bool ret = doServoCmdNormalService(comm_ptr_->service_ptr, &req_data, &res_data);
    if(!ret)
    {
        return CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED;
    }
    if(res_data.param1 != 0)
    {
        return CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED;
    }
    return SUCCESS;
}

ErrorCode ServoCommBase::doServoCmdResetEncoder(void)
{
    CoreProcessCallAppData1000_t req_data, res_data;
    memset(&req_data, 0, sizeof(req_data));
    memset(&res_data, 0, sizeof(res_data));
    req_data.cmd = SERVO_CMD_RESET_ENCODER;
    bool ret = doServoCmdNormalService(comm_ptr_->service_ptr, &req_data, &res_data);
    if(!ret)
    {
        return CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED;
    }
    if(res_data.param1 != 0)
    {
        return CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED;
    }
    return SUCCESS;
}

ErrorCode ServoCommBase::doServoCmdGetServoDefinedInfo(int32_t* req_data_ptr, int32_t* res_data_ptr)
{
    CoreProcessCallAppData1000_t req_data, res_data;
    memset(&req_data, 0, sizeof(req_data));
    memset(&res_data, 0, sizeof(res_data));
    req_data.cmd = SERVO_CMD_SERVO_DEFINED;
    req_data.param1 = req_data_ptr[0];
    req_data.param2 = req_data_ptr[1];
    req_data.param3 = req_data_ptr[2];
    req_data.param4 = req_data_ptr[3];
    req_data.param5 = req_data_ptr[4];
    req_data.param6 = req_data_ptr[5];
    req_data.param7 = req_data_ptr[6];
    req_data.param8 = req_data_ptr[7];
    bool ret = doServoCmdNormalService(comm_ptr_->service_ptr, &req_data, &res_data);
    if(!ret)
    {
        return CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED;
    }
    if(res_data.param1 != 0)
    {
        return CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED;
    }
    res_data_ptr[0] = res_data.param1;
    res_data_ptr[1] = res_data.param2;
    res_data_ptr[2] = res_data.param3;
    res_data_ptr[3] = res_data.param4;
    res_data_ptr[4] = res_data.param5;
    res_data_ptr[5] = res_data.param6;
    res_data_ptr[6] = res_data.param7;
    res_data_ptr[7] = res_data.param8;
    return SUCCESS;
}

ErrorCode ServoCommBase::triggerServoCmdUploadParameters(int32_t** async_ack_ptr_ptr)
{
    CoreProcessCallAppData1000_t req_data, res_data;
    memset(&req_data, 0, sizeof(req_data));
    memset(&res_data, 0, sizeof(res_data));
    req_data.cmd = SERVO_CMD_UPLOAD_PARAMETERS;
    bool ret = doServoCmdAsyncService(comm_ptr_->service_ptr, &req_data, &res_data, async_ack_ptr_ptr);
    if(!ret)
    {
        return CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED;
    }
    if(res_data.param1 != 0)
    {
        return CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED;
    }
    return SUCCESS;  
}

bool ServoCommBase::uploadServoParameters(BufferAppData2002_t* params_ptr)
{
    if(params_ptr == NULL)
    {
        return false;
    }
    int32_t data_size;
    bool ret = getBufferType1(comm_ptr_->upload_param_buffer_ptr, (uint8_t*)params_ptr, &data_size);
    if(!ret || data_size != sizeof(BufferAppData2002_t))
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool ServoCommBase::downloadServoParameters(const BufferAppData2001_t* params_ptr)
{
    if(params_ptr == NULL)
    {
        return false;
    }
    bool ret = setBufferType1(comm_ptr_->download_param_buffer_ptr, (uint8_t*)params_ptr, sizeof(BufferAppData2001_t));
    if(!ret)
    {
        return false;
    }
    else
    {
        return true;
    }
}

ErrorCode ServoCommBase::triggerServoCmdDownloadParameters(int32_t** async_ack_ptr_ptr)
{
    CoreProcessCallAppData1000_t req_data, res_data;
    memset(&req_data, 0, sizeof(req_data));
    memset(&res_data, 0, sizeof(res_data));
    req_data.cmd = SERVO_CMD_DOWNLOAD_PARAMETERS;
    bool ret = doServoCmdAsyncService(comm_ptr_->service_ptr, &req_data, &res_data, async_ack_ptr_ptr);
    if(!ret)
    {
        return CORE_COMM_SEND_CORE_PROCESS_CALL_FAILED;
    }
    if(res_data.param1 != 0)
    {
        return CORE_COMM_EXEC_CORE_PROCESS_CALL_FAILED;
    }
    return SUCCESS;  
}

bool ServoCommBase::isServoAsyncServiceFinish(int32_t* async_ack_ptr)
{
    return isAsyncServiceDone(async_ack_ptr);
}

void ServoCommBase::processFdbPdoCurrent(uint8_t* pdo_data_ptr, uint32_t* current_time_stamp_ptr)
{
    assert(pdo_data_ptr != NULL);
    assert(current_time_stamp_ptr != NULL);
    assert(comm_ptr_->fdb_pdo_ptr != NULL);
    pullCircleBufferType2ByHead(comm_ptr_->fdb_pdo_ptr, pdo_data_ptr, current_time_stamp_ptr);
}

void ServoCommBase::processFdbPdoSync(uint8_t* pdo_data_ptr, uint32_t expect_time_stamp)
{
    assert(pdo_data_ptr != NULL);
    assert(expect_time_stamp >= 0);
    assert(comm_ptr_->fdb_pdo_ptr != NULL);
    pullCircleBufferType2ByTimeStamp(comm_ptr_->fdb_pdo_ptr, pdo_data_ptr, expect_time_stamp);
}

void ServoCommBase::processCtrlPdoBufferMode(uint8_t* pdo_data_ptr, int32_t expect_element_number, int32_t* actual_element_number_ptr)
{
    assert(pdo_data_ptr != NULL);
    assert(actual_element_number_ptr != NULL);
    assert(comm_ptr_->ctrl_pdo_ptr != NULL);
    int32_t spare_element_number = getSpareElementNumber(comm_ptr_->ctrl_pdo_ptr);
    *actual_element_number_ptr = (spare_element_number >= expect_element_number) ? expect_element_number : spare_element_number;
    pushCircleBufferType1(comm_ptr_->ctrl_pdo_ptr, pdo_data_ptr, *actual_element_number_ptr);
}

ServoSm_e ServoCommBase::getServoState(ServoState_u& state_word)
{
    if(state_word.bit.ready_to_switch_on == 0)  // XXXX XXX0
    {
        if(state_word.bit.switched_on == 0) // XXXX XX00
        {
            if(state_word.bit.operation_enabled == 0)   // XXXX X000
            {
                if(state_word.bit.fault == 0)    // XXXX 0000
                {
                    if(state_word.bit.switch_on_disabled == 1)   // X1XX 0000
                    {
                        return SERVO_SM_SWITCH_ON_DISABLED;
                    }
                    else    // X0XX 0000
                    {
                        return SERVO_SM_NOT_READY_TO_SWITCH_ON;
                    }
                }
                else if(state_word.bit.fault == 1
                    && state_word.bit.switch_on_disabled == 0)  // X0XX 1000
                {
                    return SERVO_SM_FAULT;
                }
            }
        }        
    }
    else    // XXXX XXX1
    {
        if(state_word.bit.switch_on_disabled == 0)  // X0XX XXX1
        {
            if(state_word.bit.switched_on == 0) // X0XX XX01
            {
                if(state_word.bit.operation_enabled == 0
                    && state_word.bit.fault == 0
                    && state_word.bit.quick_stop == 1)  // X01X 0001
                {
                    return SERVO_SM_READY_TO_SWITCH_ON;
                }
            }
            else    // X0XX XX11
            {
                if(state_word.bit.operation_enabled == 0)   // X0XX X011
                {
                    if(state_word.bit.fault == 0
                        && state_word.bit.quick_stop == 1)  // X01X 0011
                    {
                        return SERVO_SM_SWITCHED_ON;
                    }
                }
                else    // X0XX X111
                {
                    if(state_word.bit.fault == 0)   // X0XX 0111
                    {
                        if(state_word.bit.quick_stop == 0)  // X00X 0111
                        {
                            return SERVO_SM_QUICK_STOP_ACTIVE;
                        }
                        else    // X01X 0111
                        {
                            return SERVO_SM_OPERATION_ENABLED;
                        }
                    }
                    else    // X0XX 1111
                    {
                        return SERVO_SM_FAULT_REACTION_ACTIVE;
                    }
                }
                
            }
        }
    }

    return SERVO_SM_UNKNOWN;
}

std::string ServoCommBase::getServoStateString(ServoSm_e sm)
{
    switch(sm)
    {
        case SERVO_SM_START:                    return std::string("Start");
        case SERVO_SM_NOT_READY_TO_SWITCH_ON:   return std::string("NotReadyToSwitchOn");
        case SERVO_SM_SWITCH_ON_DISABLED:       return std::string("SwitchOnDisabled");
        case SERVO_SM_READY_TO_SWITCH_ON:       return std::string("ReadyToSwitchOn");
        case SERVO_SM_SWITCHED_ON:              return std::string("SwitchedOn");
        case SERVO_SM_OPERATION_ENABLED:        return std::string("OperationEnabled");
        case SERVO_SM_QUICK_STOP_ACTIVE:        return std::string("QuickStopActive");
        case SERVO_SM_FAULT_REACTION_ACTIVE:    return std::string("FaultReactionActive");
        case SERVO_SM_FAULT:                    return std::string("Fault");
        default:                                return std::string("Unknown");
    }
}

void ServoCommBase::getServoCommInfo(ServoCommInfo_t* info)
{
    assert(comm_ptr_ != NULL);

    info->from = comm_ptr_->from;
    info->to = comm_ptr_->to;
    info->servo_index = comm_ptr_->servo_index;

    if (comm_ptr_->comm_reg_ptr == NULL)
    {
        info->comm_reg_id = -1;
    }else
    {
        info->comm_reg_id = comm_ptr_->comm_reg_ptr->application_id;
    }

    if (comm_ptr_->service_ptr == NULL)
    {
        info->service_id = -1;
    }else
    {
        info->service_id = comm_ptr_->service_ptr->application_id;
    }

    if (comm_ptr_->download_param_buffer_ptr == NULL)
    {
        info->download_param_id = -1;
    }else
    {
        info->download_param_id = comm_ptr_->download_param_buffer_ptr->application_id;
    }

    if (comm_ptr_->upload_param_buffer_ptr == NULL)
    {
        info->upload_param_id = -1;
    }else
    {
        info->upload_param_id = comm_ptr_->upload_param_buffer_ptr->application_id;
    }

    if (comm_ptr_->ctrl_pdo_ptr == NULL)
    {
        info->ctrl_pdo_id = -1;
    }else
    {
        info->ctrl_pdo_id = comm_ptr_->ctrl_pdo_ptr->application_id;
    }

    if (comm_ptr_->fdb_pdo_ptr == NULL)
    {
        info->fdb_pdo_id = -1;
    }else
    {
        info->fdb_pdo_id = comm_ptr_->fdb_pdo_ptr->application_id;
    }
}


ServoCommBase::ServoCommBase():
    comm_ptr_(NULL)
{

}


