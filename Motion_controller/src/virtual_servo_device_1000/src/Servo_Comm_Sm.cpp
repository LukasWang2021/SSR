#include "Servo_Comm_Sm.h"
#include "common/core_comm_datatype.h"

using namespace virtual_servo_device;


void virtual_servo_device::initCommSm(CommSm_t* comm_sm_ptr, ServoComm_t* comm_ptr)
{
    comm_sm_ptr->expect_state = CORE_COMM_STATE_INIT;
    comm_sm_ptr->comm_ptr = comm_ptr;
    comm_sm_ptr->run = &doCommSmInit;
}

void virtual_servo_device::doCommSmInit(void* sm_ptr)
{
    CommSm_t* comm_sm_ptr = (CommSm_t*)sm_ptr;
    if(comm_sm_ptr->expect_state == CORE_COMM_STATE_PREOP)
    {
        setServoCommState(comm_sm_ptr->comm_ptr, CORE_COMM_STATE_PREOP);
        comm_sm_ptr->run = &doCommSmPreOP;
    }
}

void virtual_servo_device::doCommSmPreOP(void* sm_ptr)
{
    CommSm_t* comm_sm_ptr = (CommSm_t*)sm_ptr;
    if(comm_sm_ptr->expect_state == CORE_COMM_STATE_SAFEOP)
    {
        setServoCommState(comm_sm_ptr->comm_ptr, CORE_COMM_STATE_SAFEOP);
        comm_sm_ptr->run = &doCommSmSafeOP;
    }
}

void virtual_servo_device::doCommSmSafeOP(void* sm_ptr)
{
    CommSm_t* comm_sm_ptr = (CommSm_t*)sm_ptr;
    if(comm_sm_ptr->expect_state == CORE_COMM_STATE_OP)
    {
        setServoCommState(comm_sm_ptr->comm_ptr, CORE_COMM_STATE_OP);
        comm_sm_ptr->run = &doCommSmOP;
    }
}

void virtual_servo_device::doCommSmOP(void* sm_ptr)
{
    // do nothing
}



