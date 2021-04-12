/*
 *Servo_CoreProcess.h
 *
 *Created on: 2019年7月19日
 *Author: qichao.wang
 */

 #ifndef SERVO_COREPROCESS_H_
 #define SERVO_COREPROCESS_H_

 #include <stdint.h>
 #include "common/servo_cpu_interface.h"
 #include "common/core_process_call_1000.h"
 #include "bare/core_comm_bare.h"
 #include "common/core_comm_datatype.h"
 #include "Servo_Comm_Sm.h"
 #include "Servo_Sm.h"
 #include "system/core_comm_system.h"

 #define UNKNOW_CMD 0x4004
 #define RESP_FAIL   0x3003


using namespace core_comm_space;

namespace virtual_servo_device{

 typedef struct
 {
    CoreCommSystem core_comm;
    bool core_comm_config_ready;
    CommBlockData_t* from_block_ptr;
    size_t from_block_number;
    CommBlockData_t* to_block_ptr;
    size_t to_block_number;

 }VirtualServoConfig_t;

 typedef struct
 {
    int32_t servo_id;
    int32_t isr_count;
    uint32_t ctrl_pdo_sync_reg[8];
    uint32_t sampling_sync_reg;
    uint32_t sampling_sync_reg_back;
    ServoCpuComm_t* servo_cpu_comm_ptr;


 }VirtualServoCpu_t;

 typedef struct
 {
    ServoComm_t* servo_comm_ptr;
    CommSm_t core_comm_sm;
    ServoSm_t servo_sm;
    CircleBufferAppData3000_t fdb_pdo_position_mode;
    CircleBufferAppData4000_t ctrl_pdo_position_mode;
    int32_t *servo_mode;
    int32_t ctrl_pdo_sync;
    int32_t actual_position_lsb;
    int32_t actual_position_msb;
    int32_t actual_velocity;
    int32_t actual_torque;
    bool upload_param_running;
    bool download_param_running;


 }VirtualServo_t;

 typedef struct
 {
    VirtualServoConfig_t config;
    VirtualServoCpu_t cpu;
    VirtualServo_t servo[8];
    uint32_t time_stamp;


 }VirtualServo1000_t;

 void servo_cmd_init(void);

 bool servo_cmd_get_gservo_ptr(VirtualServo1000_t** g_servo_ptr);

 void process_core_process_call(ServoComm_t* comm_ptr);

 void Servo_Cmd_Get_SamplePtr(ServoCpuComm_t** get_ptr);
 }
 #endif /* SERVO_COREPROCESS_H_ */

