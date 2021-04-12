/*
 * Servo_General.h
 *
 *  Created on: 2019年7月15日
 *      Author: qichao.wang
 */

#ifndef SERVO_GENERAL_H_
#define SERVO_GENERAL_H_

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include "common/core_comm_servo_datatype.h"
#include "Reg_Address.h"
#include "log_manager_producer_bare.h"

#define IS_BIT_SET(word, bit)  (((word) & (1<<(bit))) != 0 ? true : false)
#define RESET_BIT(word, bit)    ((word) & (~(1<<(bit))))

/*拼接错误码*/
#define CONV_SIGNEL_ERR_CODE(type,head_type,middle_type,err_code,temp_code) {type head_type_ = head_type; \
																	 	 	 type middle_type_ = middle_type; \
																	 	 	 type err_code_ = err_code;	\
																	 	 	 temp_code = (type)(head_type_<<32)|(middle_type_<<16)|(err_code_);}

#define CONV_ERR_CODE(type,head_type,middle_type,axis_num,err_code,temp_code) {type head_type_ = head_type; \
																	 	 	 type middle_type_ = middle_type; \
																	 	 	 type axis_num_ = axis_num; \
																	 	 	 type err_code_ = err_code;	\
																	 	 	 temp_code = (type)((head_type_<<24)|(middle_type_<<16)|(axis_num_<<8)|(err_code_));}
/*FPGA地址计算*/
#define FPGA_RAM_ADD_CAL(temp_add,axis_id,var_id) do{										    \
													temp_add = FPGA_CON_BASE+FPGA_RAM_OFFSET	\
																+axis_id*FPGA_CURRENT_OFFSET;   \
													temp_add += var_id*FPGA_PARA_OFFSET;}while(0)

#define FPGA_REG_ADD_CAL(temp_add,reg_id) do{temp_add = FPGA_CON_BASE + reg_id*FPGA_PARA_OFFSET;}while(0)

namespace virtual_servo_device{

typedef enum
{
	AXIS_0,
	AXIS_1,
	AXIS_2,
	AXIS_3,
	AXIS_4,
	AXIS_5,
	AXIS_6,
	AXIS_7,

	AXIS_MAX = AXIS_7 + 1,

}SERVO_AXIS_ENUM;

/*获取轴计算中间变量*/
typedef struct
{
	int64_t ctl_err;
	int64_t ki_out;
	int64_t kp_out;
	int64_t kd_out;

	int64_t ki_sum;
	int64_t pre_dout;

	int64_t con_pid_out;

}SERVO_AXIS_INF_STRUCT;

typedef struct
{
	int32_t reservd:32;

}SERVO_PARA_BIT_INFO;

typedef union
{
	int32_t reservd;
	SERVO_PARA_BIT_INFO bit_value;

}SERVO_VAR_VALID_UNION;

typedef struct
{
	int32_t para_value;
	int32_t default_value;
	int32_t value_up_limit;
	int32_t value_low_limit;
	SERVO_VAR_VALID_UNION vailid;
	SERVO_VAR_VALID_UNION attrite;
	char unit[16];

}SERVO_PARA_COMMO_INFO;


typedef struct
{
    uint32_t switch_on:1;
    uint32_t enable_voltage:1;
    uint32_t quick_stop:1;
    uint32_t enable_operation:1;
    uint32_t operation_mode_spec1:3;
    uint32_t fault_reset:1;
    uint32_t halt:1;
    uint32_t operation_mode_spec2:1;
    uint32_t rsvd:22;

}SERVO_CONTROLLER_BIT_STRUCT;


typedef union
{
    uint32_t all;
    SERVO_CONTROLLER_BIT_STRUCT bit;

}SERVO_CONTROLLER_UNION;

typedef struct
{
	uint32_t ready_to_switch_on:1;
	uint32_t switched_on:1;
	uint32_t operation_enabled:1;
	uint32_t fault:1;
	uint32_t voltage_enabled:1;
	uint32_t quick_stop:1;
	uint32_t switch_on_disabled:1;
    uint32_t warning:1;
    uint32_t rsvd1:1;
    uint32_t remote:1;
    uint32_t target_reached:1;
    uint32_t internal_limit_active:1;
    uint32_t operation_mode_spec:2;
    uint32_t rsvd:18;

}SERVO_AXIS_STATE_BIT_STRUCT;

typedef union
{
	uint32_t all;
	SERVO_AXIS_STATE_BIT_STRUCT bit;

}SERVO_AXIS_STATSE_UNION;

typedef enum
{
	NORMAL_MODE = 0x1,
	POSITION_MODE,
	SPEED_MODE,
	TORQUE_MODE

}SERVO_AXIS_MODE_ENUM;

typedef enum
{
	WORK_MODE,
	STUDY_MODE

}SERVO_FPGA_MODE_ENUM;
/*控制寄存器枚举*/
typedef enum
{
	FPGA_ID,
	CURRENT_CFG_CON,
	FPGA_MODE,
	PWM_CON,
	DA0_CON,
	DA1_CON,
	DA2_CON,
	DA3_CON,

	FPGA_REG_MAX = DA3_CON + 1

}SERVO_FPGA_REGISTER_ENUM;
}

#endif /* SERVO_GENERAL_H_ */

