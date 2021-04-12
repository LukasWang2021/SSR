#ifndef COMM_SM_H
#define COMM_SM_H


#include "Servo_Common_Def.h"
#include "common/servo_interface.h"

namespace virtual_servo_device{

typedef struct
{
    int32_t expect_state;
    ServoComm_t* comm_ptr;  // actual state is saved on comm_reg shm
    runSm run;
}CommSm_t;    

void initCommSm(CommSm_t* comm_sm_ptr, ServoComm_t* comm_ptr);
void doCommSmInit(void* sm_ptr);
void doCommSmPreOP(void* sm_ptr);
void doCommSmSafeOP(void* sm_ptr);
void doCommSmOP(void* sm_ptr);


}
#endif
