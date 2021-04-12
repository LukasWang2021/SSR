/*
 * Servo_Safety.c
 *
 *  Created on: 2019年7月26日
 *      Author: qichao.wang
 */


#include "Servo_Safety.h"

#define MAX_COUNT 15
/*10ms读取一次安全板数据*/
#define DELAY_COUNT 50

using namespace virtual_servo_device;

static MOSAFE_FRAME_SET_INFO_STRUCT safety_send_info;

static MOSAFE_FREAM_RES_INFO_STRUCT *safety_resp_info = NULL;

static MOSAFE_FRAME_PROTO_INFO_STRUCT *safety_proto_info = NULL;

static void Servo_Safety_s_WriteSafety(void);

static void Servo_Safety_s_ReadSafety(void);

static uint32_t safety_board_status = 0;

static uint32_t safety_err_code = 0;
/*-------------------------------------------------------------
 *函数名称 :void MOSAFE_Init(void)
 *函数输入 :
 *函数输出 :
 *函数描述 :安全办数据初始化
--------------------------------------------------------------*/
void virtual_servo_device::Servo_Safety_Init(void)
{
	safety_resp_info = (MOSAFE_FREAM_RES_INFO_STRUCT *)(&safety_board_status);
	safety_proto_info = (MOSAFE_FRAME_PROTO_INFO_STRUCT *)(SAFE_BASE_ADD + SAFE_CON_OFFSET);
}
/*----------------------------------------------------------------------------------------------
 *函数名称:int MOSAFE_s_Get_Contator(void)
 *函数输入:无
 *函数输出:无
 *函数备注:检测接触器信号，减速信号，急停信号，抱闸开断信号
-----------------------------------------------------------------------------------------------*/
int virtual_servo_device::Servo_Safety_s_Get_Contator(int check_signal)
{
	int temp_data = 0;

	switch(check_signal)
	{
		case(CONTACTOR_SIGNAL):
		{
			temp_data = (safety_resp_info->frame_1&0x18)>>3;

			if(temp_data == 3)
			{
				temp_data = 1;
			}

			break;
		}
		case(DEC_SIGNAL):
		{
			temp_data = (safety_resp_info->frame_2&0x8)>>3;

			if(temp_data == 1)
			{
				temp_data = 1;
			}
			break;
		}
		case(IMDE_STOP_SIGNAL):
		{
			/*是否有急停发生*/
			temp_data = (safety_resp_info->frame_3&0x1);

			if(temp_data == 1)
			{
				temp_data = 1;
			}

			break;
		}
		case(BRAKER_STATUS):
		{
			temp_data = (safety_resp_info->frame_1&0x01);
			/*抱闸打开时为1？*/
			if(temp_data==0)
			{
				temp_data = 1;
			}
			else
			{
				temp_data = 0;
			}
			break;
		}
		case(SAFETY_PROTO):
		{
			temp_data = (safety_proto_info->err_status)&0x01;

			if(!temp_data)
			{
				temp_data = 1;
			}else{
				temp_data = 0;
			}

			break;
		}
		case(SAFETY_ALARM):
		{
			temp_data = (safety_resp_info->frame_4)&0x80;

			if(temp_data)
			{
				temp_data = 1;
			}else{
				temp_data = 0;
			}
			break;
		}
		default:break;

	}
	return temp_data;
}
/*-----------------------------------------------------------------------------------------
 * 函数名称:void MOSAFE_b_Write_Cmd_Ana(MOSAFE_SAFE_CMD_ENUM cmd_id)
 * 函数输入:
 * 函数输出:
 * 函数备注:缩减函数接口，四个接口函数整合为一个
 *-------------------------------------------------------------------------------------- */
 void virtual_servo_device::Servo_Safety_b_Write_Cmd_Ana(MOSAFE_SAFE_CMD_ENUM cmd_id)
{
	switch(cmd_id)
	{
		case(RESET_SAFETY):
		{
			safety_send_info.frame_1 = 0x00;
			safety_send_info.frame_2 = 0x00;

			break;
		}
		case(ZERO_STOP):
		{
			safety_send_info.frame_1 = 0x01;
			safety_send_info.frame_2 = 0x00;

			break;
		}
		case(EXCITATION):
		{
			safety_send_info.frame_1 = 0x00;
			safety_send_info.frame_2 = 0x01;

			break;
		}
		default:break;
	}
}

#define HEARTBEAT_INTENAL 100
/*----------------------------------------------------------------------------
 * 函数名称:void MOSAFE_p_RwSafety(void)
 * 函数输入:
 * 函数输出:
 * 函数备注:产生安全板心跳信号
 * --------------------------------------------------------------------------*/
 void virtual_servo_device::Servo_Safety_p_RwSafety(void)
{
	static int delay_count = 0;

	static uint8_t heart_beat = 0;

	if(delay_count<=HEARTBEAT_INTENAL)
	{
		delay_count++;
	}else{
		/*心跳加一*/
		safety_send_info.frame_2 &= 0x0f;
		safety_send_info.frame_2 |= heart_beat<<4;
		/*20ms写一次安全板，读一次安全板*/
		Servo_Safety_s_WriteSafety();

		Servo_Safety_s_ReadSafety();

		delay_count = 0;
		/*由于只有4位数据为用于产生心跳,需要将心跳数据或到第二Byte上，所以计数限制到16*/
		if(heart_beat<16)
		{
			heart_beat++;

		}else{
			heart_beat = 0;
		}
	}
}
/*----------------------------------------------------------------------------
 * 函数名称:void MOSAFE_s_WriteSafety(void)
 * 函数输入:
 * 函数输出:
 * 函数备注:写安全板数据
 * --------------------------------------------------------------------------*/
void Servo_Safety_s_WriteSafety(void)
{
	uint32_t temp_add = 0;

	uint32_t temp_data = 0;

	temp_data = SAFE_FRMAE_COMB(safety_send_info.frame_1,
				safety_send_info.frame_2,
				safety_send_info.resever1,
				safety_send_info.resever2);

	temp_add = SAFE_BASE_ADD + SAFE_WRITE_OFFSET;

//	alt_write_word(temp_add,temp_data);
}
/*----------------------------------------------------------------------------
 * 函数名称:void MOSAFE_s_ReadSafety(void)
 * 函数输入:
 * 函数输出:
 * 函数备注:读安全板数据
 * --------------------------------------------------------------------------*/
void Servo_Safety_s_ReadSafety(void)
{
	uint32_t temp_add = 0;

	temp_add = SAFE_BASE_ADD + SAFE_READ_OFFSET;

//	safety_board_status = alt_read_word(temp_add);

	temp_add = SAFE_BASE_ADD + SAFE_ERR_CODE;

//	safety_err_code = alt_read_word(temp_add);
}
/*----------------------------------------------------------------------------
 * 函数名称:uint32_t MOSAFE_p_Process_SafetBoard_Status(void)
 * 函数输入:
 * 函数输出:
 * 函数备注:上位机检测安全板使用信号
 * --------------------------------------------------------------------------*/
 uint32_t virtual_servo_device::Servo_Safety_p_Process_SafetBoard_Status(void)
{
	return safety_board_status;
}
/*----------------------------------------------------------------------------
 * 函数名称:uint32_t MOSAFE_p_Process_SafetBoard_ErrCode(void)
 * 函数输入:
 * 函数输出:
 * 函数备注:返回安全板错误码
 * --------------------------------------------------------------------------*/
 uint32_t virtual_servo_device::Servo_Safety_p_Process_SafetBoard_ErrCode(void)
{
	return safety_err_code;
}


