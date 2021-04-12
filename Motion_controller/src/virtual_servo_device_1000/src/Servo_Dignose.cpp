/*
 * Servo_Dignose.c
 *
 *  Created on: 2019年7月26日
 *      Author: qichao.wang
 */
#include "Servo_Dignose.h"
#include "Servo_General.h"
#include "common/servo_cpu_interface.h"
#include "common/comm_reg_1.h"
#include <string.h>
#include <assert.h>


using namespace virtual_servo_device;

static int32_t *dignose_var_ptr[AXIS_MAX][VAR_MAX];
static SERVO_DIGNOSE_PTR_INFO_STRUCT cfg_var_ptr;
static ServoCpuComm_t* sample_info_ptr = NULL;
static CommRegAppData1_t local_sample_info;

static SERVO_DIGNOSE_VAR_STRUCT dignose_data;
static SERVO_DIGNOSE_CON_SAMPLE_STRUCT sample_controll;
/*-------------------------------------------------------------
 *函数名称 :void MOGR_p_Write_FPGA(void)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :HPS控制FPGA保护操作
--------------------------------------------------------------*/
void virtual_servo_device::Servo_Dignose_Init(void)
{
	Servo_Cmd_Get_SamplePtr(&sample_info_ptr);
}
/*-------------------------------------------------------------
 *函数名称 :void MOGR_p_Write_FPGA(void)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :HPS控制FPGA保护操作
--------------------------------------------------------------*/
void virtual_servo_device::Servo_Dignose_SetPara(SERVO_AXIS_ENUM axis_id,
		SERVO_DIGNOSE_VAR_ENUM var_id,
		SERVO_CHANNEL_ENUM channel_id)
{
	SERVO_DIGNOSE_VAR_PROCESS_STRUCT* temp_cfg_var_ptr =
			(SERVO_DIGNOSE_VAR_PROCESS_STRUCT*)&cfg_var_ptr;
	(temp_cfg_var_ptr+channel_id)->var_ptr =
			dignose_var_ptr[axis_id][var_id];
}
/*-------------------------------------------------------------
 *函数名称 :void MOGR_p_Write_FPGA(void)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :HPS控制FPGA保护操作
--------------------------------------------------------------*/
int32_t virtual_servo_device::Servo_Dignose_Register_Var(SERVO_AXIS_ENUM axis_id
									,int32_t* var_ptr,
									SERVO_DIGNOSE_VAR_ENUM var_id)
{
	int32_t ret_val = 0;

	assert(var_ptr!=NULL);

	if(var_id<VAR_MAX)
	{
		dignose_var_ptr[axis_id][var_id] = var_ptr;

	}else{

		ret_val = -1;
	}

	return ret_val;
}
/*-------------------------------------------------------------
 *函数名称 :void MOGR_p_Write_FPGA(void)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :HPS控制FPGA保护操作
--------------------------------------------------------------*/
void virtual_servo_device::Servo_Dignose_p_GetVar(void)
{
	int32_t *temp_dig_ptr = (int32_t *)(&dignose_data);
	SERVO_DIGNOSE_VAR_PROCESS_STRUCT *temp_var_ptr =
			(SERVO_DIGNOSE_VAR_PROCESS_STRUCT *)(&cfg_var_ptr);

	memcpy((char*)&local_sample_info,sample_info_ptr->comm_reg_ptr->memory_ptr
									,sizeof(CommRegAppData1_t));

	if(local_sample_info.sampling_sync)
	{
		if(sample_controll.sample_count<local_sample_info.sampling_max_times)
		{
			for(int i=0; i<VAR_PTR_MAX; i++)
			{
					*(temp_dig_ptr+i)
							= *((temp_var_ptr+i)->var_ptr);
			}
			memcpy((sample_info_ptr->sampling_buffer_ptr->memory_ptr+8+sample_controll.sample_count*64)
					,(char*)&dignose_data,64);
			sample_controll.sample_count++;
		}else{
			sample_controll.sample_count = 0;
			local_sample_info.sampling_sync = 0;
			*(uint32_t*)(sample_info_ptr->sampling_buffer_ptr->memory_ptr) = 64*(local_sample_info.sampling_max_times);
			*(uint32_t*)(sample_info_ptr->comm_reg_ptr->memory_ptr+8) = 0;
			printf("sample over\r\n");
		}
	}else{
		sample_controll.sample_count = 0;
	}
}
/*-------------------------------------------------------------
 *函数名称 :void MOGR_p_Write_FPGA(void)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :HPS控制FPGA保护操作
--------------------------------------------------------------*/
void virtual_servo_device::Servo_Dignose_b_AnaInfo(void)
{
	uint32_t temp_val = 0;
	uint32_t temp_high_edge = 0;

	static int32_t pre_value = 0;

	if(sample_info_ptr->comm_reg_ptr->application_id==1)
	{
		temp_val = *(uint32_t*)(sample_info_ptr->comm_reg_ptr->memory_ptr+12);

		temp_high_edge = temp_val&(~pre_value);

		pre_value = temp_val;

		if(temp_high_edge)
		{
			memcpy((char*)&local_sample_info,sample_info_ptr->comm_reg_ptr->memory_ptr
								,sizeof(CommRegAppData1_t));

			for(int32_t i=0; i<CHANNEL_MAX; i++)
			{
				Servo_Dignose_SetPara(static_cast<SERVO_AXIS_ENUM>((local_sample_info.sampling_channel[i]&0xffff0000)>>16),
						static_cast<SERVO_DIGNOSE_VAR_ENUM>(local_sample_info.sampling_channel[i]&0xffff),static_cast<SERVO_CHANNEL_ENUM>(i));

				printf("sampling_sync:%x\r\n",local_sample_info.sampling_channel[i]);
			}

			printf("sampling_max:%u\r\n",local_sample_info.sampling_max_times);

			sample_controll.sample_count = 0;
		}
	}

}

