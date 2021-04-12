/*
 * Servo_Encode.c
 *
 *  Created on: 2019年7月26日
 *      Author: qichao.wang
 */
#include "Servo_Fpga.h"
#include "Servo_Encode.h"
#include "Servo_General.h"


using namespace virtual_servo_device;

static SERVO_ENCODE_OUTPUT_STRUCT pos_out[AXIS_MAX];
static SERVO_FPGA_ENCODE_BACK_STRUCT *encode_out[AXIS_MAX];

static PARA_ENCODE_CFG_READ_INFO encode_alg_info[AXIS_MAX];

/*-------------------------------------------------------------
 *函数名称 :void MOGR_p_Write_FPGA(void)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :HPS控制FPGA保护操作
--------------------------------------------------------------*/
void virtual_servo_device::Servo_Encode_Init(void)
 {
	 Servo_Fpga_Get_EncPtr(&encode_out[AXIS_0],AXIS_0);
	 Servo_Fpga_Get_EncPtr(&encode_out[AXIS_1],AXIS_1);
	 Servo_Fpga_Get_EncPtr(&encode_out[AXIS_2],AXIS_2);
	 Servo_Fpga_Get_EncPtr(&encode_out[AXIS_3],AXIS_3);
	 Servo_Fpga_Get_EncPtr(&encode_out[AXIS_4],AXIS_4);
	 Servo_Fpga_Get_EncPtr(&encode_out[AXIS_5],AXIS_5);
	 Servo_Fpga_Get_EncPtr(&encode_out[AXIS_6],AXIS_6);
	 Servo_Fpga_Get_EncPtr(&encode_out[AXIS_7],AXIS_7);

	 encode_alg_info[AXIS_0].encode_class.para_value = 256;
	 encode_alg_info[AXIS_1].encode_class.para_value = 256;
	 encode_alg_info[AXIS_2].encode_class.para_value = 256;
	 encode_alg_info[AXIS_3].encode_class.para_value = 256;
	 encode_alg_info[AXIS_4].encode_class.para_value = 256;
	 encode_alg_info[AXIS_5].encode_class.para_value = 256;
 }
 /*-------------------------------------------------------------
  *函数名称 :void MOGR_p_Write_FPGA(void)
  *函数输入 :无
  *函数输出 :无
  *函数描述 :HPS控制FPGA保护操作
 --------------------------------------------------------------*/
 void virtual_servo_device::Servo_Encode_SetPara(SERVO_AXIS_ENUM axis_id,
 		const PARA_ENCODE_CFG_READ_INFO *para_ptr)
 {
//	 encode_alg_info[axis_id].encode_class.para_value=
//			 para_ptr->encode_class.para_value;

	 Servo_Fpga_Write(axis_id,ENC_FILTER_REG
			 ,2);
 }
 /*-------------------------------------------------------------
  *函数名称 :void MOGR_p_Write_FPGA(void)
  *函数输入 :无
  *函数输出 :无
  *函数描述 :HPS控制FPGA保护操作
 --------------------------------------------------------------*/
 void virtual_servo_device::Servo_Encode_GetPara(SERVO_AXIS_ENUM axis_id,
 		PARA_ENCODE_CFG_READ_INFO *para_ptr,
 		int32_t copy_num)
 {

 }
 /*-------------------------------------------------------------
  *函数名称 :void MOGR_p_Write_FPGA(void)
  *函数输入 :无
  *函数输出 :无
  *函数描述 :HPS控制FPGA保护操作
 --------------------------------------------------------------*/
 void virtual_servo_device::Servo_Encode_Get_EncodePtr(SERVO_ENCODE_OUTPUT_STRUCT **get_ptr
		 ,SERVO_AXIS_ENUM axis_id)
 {
	 *get_ptr = &pos_out[axis_id];
 }
 /*-------------------------------------------------------------
  *函数名称 :void MOGR_p_Write_FPGA(void)
  *函数输入 :无
  *函数输出 :无
  *函数描述 :HPS控制FPGA保护操作
 --------------------------------------------------------------*/
 void virtual_servo_device::Servo_Encode_p_CalPos(SERVO_AXIS_ENUM axis_id)
 {
	 pos_out[axis_id].position_back = (int64_t)(encode_out[axis_id]->encode_total
			 	 	 	 	 *encode_alg_info[axis_id].encode_class.para_value);

 }

