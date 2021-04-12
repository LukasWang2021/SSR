/*
 * error_process.c
 *
 *  Created on: 2019年10月8日
 *      Author: qichao.wang
 */
#include "error_process.h"

#define HISTORY_ERR_CODE_LENGTH 7

using namespace virtual_servo_device;

/*--------------------static data--------------------*/
static PARA_AXIS_LOCAL_INFO *err_para_local_info[AXIS_MAX];
static PARA_AXIS_READ_INFO *err_para_read_info[AXIS_MAX];

static uint32_t *current_err_code[AXIS_MAX];
static uint32_t *history_err_code[AXIS_MAX];
/*---------------------static function------------------*/
static void error_process_pushfifo(SERVO_AXIS_ENUM axis_id,
							uint32_t error_code);
/*------------------------------------------------------------------------
 * 函数名称:void error_process_init(void)
 * 函数输入:none
 * 函数输出:none
 * 函数备注:框架数据指针初始化
 * ---------------------------------------------------------------------*/
 void virtual_servo_device::error_process_init(void)
{
	for(uint32_t i=0; i<AXIS_MAX; i++)
	{
		Servo_Para_Get_LocalPtr(static_cast<SERVO_AXIS_ENUM>(i),&err_para_local_info[static_cast<SERVO_AXIS_ENUM>(i)]);
		Servo_Para_Get_ReadPtr(static_cast<SERVO_AXIS_ENUM>(i),&err_para_read_info[static_cast<SERVO_AXIS_ENUM>(i)]);

		current_err_code[i] = (uint32_t*)&err_para_local_info[i]->heck_err_info.current_error_code;
		history_err_code[i] = (uint32_t*)&err_para_local_info[i]->heck_err_info.history_error_code1;

	}
}
/*------------------------------------------------------------------------
 * 函数名称:void error_process_report(SERVO_AXIS_ENUM axis_id,uint32_t error_code)
 * 函数输入:轴枚举，错误码
 * 函数输出:none
 * 函数备注:错误码上传
 * ---------------------------------------------------------------------*/
 void virtual_servo_device::error_process_report(SERVO_AXIS_ENUM axis_id,uint32_t error_code)
{
	if(!(*current_err_code[axis_id]))
	{
		*current_err_code[axis_id] = error_code;
	}else{
		error_process_pushfifo(axis_id,error_code);
	}
}
/*------------------------------------------------------------------------
 * 函数名称:void error_process_pushfifo(SERVO_AXIS_ENUM axis_id,
							uint32_t* push_add,
							uint32_t push_length)
 * 函数输入:轴枚举，压栈长度
 * 函数输出:none
 * 函数备注:保存历史错误码
 * ---------------------------------------------------------------------*/
 void error_process_pushfifo(SERVO_AXIS_ENUM axis_id,uint32_t error_code)
{
	uint32_t temp_code = error_code;

	for(uint32_t i=HISTORY_ERR_CODE_LENGTH; i>0; i--)
	{
		*(history_err_code[axis_id]+i) = *(history_err_code[axis_id]+i-1);
	}

	*history_err_code[axis_id] = temp_code;
}
/*------------------------------------------------------------------------
 * 函数名称:void error_process_reset_errcode(SERVO_AXIS_ENUM axis_id)
 * 函数输入:轴枚举
 * 函数输出:none
 * 函数备注:清除当前错误码
 * ---------------------------------------------------------------------*/
 void virtual_servo_device::error_process_reset_errcode(SERVO_AXIS_ENUM axis_id)
{
	*current_err_code[axis_id] = 0;
}

