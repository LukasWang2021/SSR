#include "system/servo1001_comm.h"
#include "log_manager_producer.h"

using namespace servo_comm_space;
using namespace log_space;

Servo1001::Servo1001(int32_t controller_id, int32_t servo_id)
{
    is_valid_ = true;
    cpu_ptr_ = new ServoCpuCommBase(controller_id, servo_id);
    if(cpu_ptr_ == NULL)
    {
        is_valid_ = false;
    }

    for(size_t i = 0; i < SERVO_NUMBER; ++i)
    {
        servo_ptr_[i] = new ServoCommBase(controller_id, servo_id, i);
        if(servo_ptr_[i] == NULL)
        {
            is_valid_ = false;
        }
    }
}

Servo1001::~Servo1001()
{
    if(cpu_ptr_ != NULL)
    {
        delete cpu_ptr_;
        cpu_ptr_ = NULL;
    }

    for(size_t i = 0; i < SERVO_NUMBER; ++i)
    {
        if(servo_ptr_[i] != NULL)
        {
            delete servo_ptr_[i];
            servo_ptr_[i] = NULL;
        }
    }
}

bool Servo1001::isValid()
{
    return is_valid_;
}

ServoCpuCommBase* Servo1001::getCpuCommPtr()
{
    return cpu_ptr_;
}

ServoCommBase* Servo1001::getServoCommPtr(size_t servo_index)
{
    if(servo_index >= 0 && servo_index < SERVO_NUMBER)
    {
        return servo_ptr_[servo_index];
    }
    else
    {
        return NULL;
    }
}

Servo1001::Servo1001()
{
    is_valid_ = false;
    cpu_ptr_ = NULL;
    for(size_t i = 0; i < SERVO_NUMBER; ++i)
    {
        servo_ptr_[i] = NULL;
    }
}

bool Servo1001::prepareInit2PreOp(CommBlockData_t* from_block_ptr, size_t from_block_number, CommBlockData_t* to_block_ptr, size_t to_block_number)
{
    for(uint32_t i = 0; i < SERVO_NUMBER; ++i)
    {
        if(!servo_ptr_[i]->prepareInit2PreOp(from_block_ptr, from_block_number, to_block_ptr, to_block_number))
        {
            LogProducer::error("servo1001", "servo_ptr_[%d] prepareInit2PreOp failed", i);
            return false;
        }
    }
    return true;
}

bool Servo1001::preparePreOp2SafeOp(CommBlockData_t* to_block_ptr, size_t to_block_number)
{
    for(uint32_t i = 0; i < SERVO_NUMBER; ++i)
    {
        int32_t fdb_pdo_app_id = i < PMSM_NUMBER ? 3001 : 3002; 
        if(!servo_ptr_[i]->preparePreOp2SafeOp(to_block_ptr, to_block_number, fdb_pdo_app_id))
        {
            LogProducer::error("servo1001", "servo_ptr_[%d] preparePreOp2SafeOp failed", i);
            return false;
        }
    }
    return true;
}

bool Servo1001::prepareSafeOp2Op(CommBlockData_t* to_block_ptr, size_t to_block_number)
{
    for(uint32_t i = 0; i < SERVO_NUMBER; ++i)
    {
        if(!servo_ptr_[i]->prepareSafeOp2Op(to_block_ptr, to_block_number, 4000))
        {
            LogProducer::error("servo1001", "servo_ptr_[%d] prepareSafeOp2Op failed", i);
            return false;
        }
    }
    return true;
}

bool Servo1001::initServoCpuComm(CommBlockData_t* from_block_ptr, size_t from_block_number, CommBlockData_t* to_block_ptr, size_t to_block_number)
{
    return cpu_ptr_->init(from_block_ptr, from_block_number, to_block_ptr, to_block_number);
}

bool Servo1001::doServoCmdTransCommState(CoreCommState_e expected_state)
{
    for(uint32_t i = 0; i < SERVO_NUMBER; ++i)
    {
        if(servo_ptr_[i]->doServoCmdTransCommState(expected_state) != SUCCESS)
        {
            LogProducer::error("servo1001", "servo_ptr_[%d] trans state to %d failed", i, expected_state);
            return false;
        }
    }
    return true;
}


bool Servo1001::isAllServosInExpectedCommState(CoreCommState_e expected_comm_state)
{
    CoreCommState_e current_comm_state;
    for(size_t i = 0; i < SERVO_NUMBER; ++i)
    {
        current_comm_state = servo_ptr_[i]->getCommState();
        LogProducer::info("servo1001", "servo[%d] is in state %d", i, current_comm_state);
        if(current_comm_state < expected_comm_state 
            || current_comm_state > CORE_COMM_STATE_OP)
        {
            return false;
        }
    }
    return true;
}
