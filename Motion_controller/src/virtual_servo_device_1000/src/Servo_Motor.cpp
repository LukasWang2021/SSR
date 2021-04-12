/*
 * Servo_Motor.c
 *
 *  Created on: 2019年7月26日
 *      Author: qichao.wang
 */
/*
 * MOTOR.c
 *
 *  Created on: 2016年8月1日
 *      Author: qichao.wang
 */
#include "Reg_Address.h"
#include "Servo_General.h"
#include "Servo_Motor.h"
#include "Servo_Fpga.h"

#define PI 3.141592654

#define DUMPING_LIMIT 16777216

#define MOTOR_CAL(type,current) ({type _temp_data=current;(type)(_temp_data/16777216);})

#define CAL_FILTER_FACTOR(type,conv_data,cut_off_freq,sample_t) ({type _temp_data;			\
											_temp_data = 1/(2*PI*cut_off_freq)/(1/(2*PI*cut_off_freq)+sample_t)	\
											;conv_data = (int)(_temp_data*16777216);})

using namespace virtual_servo_device;


static PARA_MOTOR_CFG_READ_INFO motor_cfg_info[AXIS_MAX];
static SERVO_MOTOR_TORQUE2CURRENT_FACTOR_STRUCT tor2current_factor[AXIS_MAX];

/*----------------------------------框架静态函数-----------------------*/
static void Servo_Motor_Cal_CurrentFactor(SERVO_AXIS_ENUM axis_id);

static void Servo_Motor_Current_Cfg(void);

static void Servo_Motor_DmaCfg(void);

static void Servo_Motor_Build_DmaLink(uint32_t index,
		   uint32_t sou_addr,
		   uint32_t des_addr,
		   uint32_t linu_num);

static void Servo_Motor_Switch_Mode(SERVO_AXIS_ENUM axis_id
						,int fpga_mode);
/*-------------------------------------------------------------
 *函数名称 :
 *函数输入 :
 *函数输出 :
 *函数描述 :初始化光栅尺factor
--------------------------------------------------------------*/
void virtual_servo_device::Servo_Motor_Init(void)
{
	/*计算电机变量*/
	Servo_Motor_Current_Cfg();

	//Servo_Print("fpga current num:%x\r\n",*(volatile uint32_t*)(0xc0000004));

	Servo_Motor_Switch_Mode(AXIS_0,WORK_MODE);
	Servo_Motor_Switch_Mode(AXIS_1,WORK_MODE);
	Servo_Motor_Switch_Mode(AXIS_2,WORK_MODE);
	Servo_Motor_Switch_Mode(AXIS_3,WORK_MODE);
	Servo_Motor_Switch_Mode(AXIS_4,WORK_MODE);
	Servo_Motor_Switch_Mode(AXIS_5,WORK_MODE);

	Servo_Motor_DmaCfg();
}
/*-------------------------------------------------------------
 *函数名称 :
 *函数输入 :
 *函数输出 :
 *函数描述 :初始化光栅尺factor
--------------------------------------------------------------*/
void virtual_servo_device::Servo_Motor_SetPara(SERVO_AXIS_ENUM axis_id,
				const PARA_MOTOR_CFG_READ_INFO *para_ptr)
{
	motor_cfg_info[axis_id].motor_pole_pairs.para_value
										=5;
	motor_cfg_info[axis_id].motor_polo_angle.para_value
										=49032;
	motor_cfg_info[axis_id].sample_resistence.para_value
										=20;
	motor_cfg_info[axis_id].motor_rated_current.para_value
										=46137344;
	motor_cfg_info[axis_id].motor_kt.para_value
										=para_ptr->motor_kt.para_value;

	Servo_Motor_Cal_CurrentFactor(axis_id);

	Servo_Fpga_Write(axis_id,POLAR_NUM_RAM,motor_cfg_info[axis_id].motor_pole_pairs.para_value);
	Servo_Fpga_Write(axis_id,POLAR_ALIGN_RAM,motor_cfg_info[axis_id].motor_polo_angle.para_value);
	Servo_Fpga_Write(axis_id,PHA_SATURE_RAM,50331648);
}
/*-------------------------------------------------------------
 *函数名称 :void MOTOR_b_CalFpgaFactor(MO_AXIS_ENUM axis_id)
 *函数输入 :
 *函数输出 :
 *函数描述 :计算电机参数，下发到FPGA
--------------------------------------------------------------*/
void Servo_Motor_Cal_CurrentFactor(SERVO_AXIS_ENUM axis_id)
{

	float temp_data = 0.;

	uint32_t cfg_data = 0;

	temp_data = MOTOR_CAL(float,motor_cfg_info[axis_id].motor_rated_current.para_value);

	temp_data = (CAL_CURRENT_FACTOR
			/(motor_cfg_info[axis_id].sample_resistence.para_value*temp_data));

	cfg_data = (uint32_t)temp_data;

	Servo_Fpga_Write(axis_id,CUR_SAMPLE_RAM,cfg_data);
}
/*------------------------------------------------------------------
 *函数名称 :void Servo_Motor_Cal_Tor2Current(SERVO_AXIS_ENUM axis_id)
 *函数输入 :
 *函数输出 :
 *函数描述 :计算电机参数，下发到FPGA
-------------------------------------------------------------------*/
void Servo_Motor_Cal_Tor2Current(SERVO_AXIS_ENUM axis_id)
{
	double temp_torque_kt = (double)motor_cfg_info[axis_id].motor_kt.para_value;
	double temp_rate_current = (double)motor_cfg_info[axis_id].motor_rated_current.para_value;
	temp_torque_kt = temp_torque_kt/16777216.0;
	temp_rate_current = temp_rate_current/16777216.0;

	tor2current_factor[axis_id].conv_factor = (int64_t)(16777216/(temp_torque_kt*temp_rate_current));
}
/*-----------------------------------------------------------------
 * 函数名称:void MOTOR_b_Current_Cfg(void)
 * 函数输入:
 * 函数输出:
 * 函数备注:配置电流环个数和默认工作模式
 * ---------------------------------------------------------------*/
void Servo_Motor_Current_Cfg(void)
{
	uint32_t temp_add = 0;

	uint32_t temp_data = 0;

	FPGA_REG_ADD_CAL(temp_add,CURRENT_CFG_CON);

	temp_data = 0;//alt_read_word(temp_add);

	temp_data = temp_data|(MAX_CURRENT_NUM<<CURRENT_NUM_SHIFT);

	Servo_Fpga_Gen_Write(temp_add,temp_data);

	temp_data = 0;//alt_read_word(temp_add);

	temp_data = temp_data|GLOBAL_ENABLE;

	Servo_Fpga_Gen_Write(temp_add,temp_data);
}

#define ENABLE_SHIFT 16
/*----------------------------------------------------------------------
 * 函数名称void MOTOR_s_Con_Motor(MO_AXIS_ENUM axis_id
					,int enable)
 * 函数输入:
 * 函数输出:
 * 函数备注:使能电机
 * --------------------------------------------------------------------*/
 void virtual_servo_device::Servo_Motor_ConMotor(SERVO_AXIS_ENUM axis_id
					,int enable)
{
	uint32_t temp_add = 0;

	uint32_t temp_data = 0;

	FPGA_REG_ADD_CAL(temp_add,CURRENT_CFG_CON);

	temp_data = Servo_Fpga_Gen_Read(temp_add);

	if(enable)
	{
		temp_data |= (1<<(axis_id+ENABLE_SHIFT));
	}
	else
	{
		temp_data &= ~(1<<(axis_id+ENABLE_SHIFT));
	}

	Servo_Fpga_Gen_Write(temp_add,temp_data);
}
/*----------------------------------------------------------------------
 * 函数名称:void MOTOR_b_Switch_Mode(MO_AXIS_ENUM axis_id
						,int fpga_mode)
 * 函数输入:
 * 函数输出:
 * 函数备注:切换FPGA到正常工作模式或者磁极学习模式
 * -------------------------------------------------------------------*/
 void Servo_Motor_Switch_Mode(SERVO_AXIS_ENUM axis_id
						,int fpga_mode)
{
	uint32_t temp_add = 0;

	uint32_t temp_data = 0;

	FPGA_REG_ADD_CAL(temp_add,FPGA_MODE);

	temp_data = Servo_Fpga_Gen_Read(temp_add);

	if(fpga_mode)
	{
		temp_data |= (1<<axis_id);
	}
	else
	{
		temp_data &= ~(1<<(axis_id));
	}

	Servo_Fpga_Gen_Write(temp_add,temp_data);
}

#define ADC_ENC_OFFSET 0x08
#define MAX_ACR        0x08
/*----------------------------------------------------------------------
 * 函数名称:void MOTOR_b_DmaCfg(int link_cmd,
					int sou_addr,
					int des_addr)
 * 函数输入:
 * 函数输出:
 * 函数备注:配置DMA功能链表
 * -------------------------------------------------------------------*/
void Servo_Motor_DmaCfg(void)
{
	uint32_t temp_des_addr = 0;

	uint32_t temp_sou_addr = 0;

	for(int i=0; i<MAX_ACR; i++)
	{
		temp_sou_addr = ADDR_BASE_ENC \
						+ (SOURCE_OFFSET*i) \
						+ ENC_SOU_OFFSET;

		temp_des_addr = ADDR_BASE_ACR \
						+ ACR_FACTOR*i 	\
						+ ENC_ACR_OFFSET \
						+ DES_OFFSET;

		Servo_Motor_Build_DmaLink(i,temp_sou_addr,temp_des_addr,0);

	}

	for(int i=8; i<(MAX_ACR+8); i++)
	{
		temp_sou_addr = ADDR_BASE_ADC \
						+ SOURCE_OFFSET*(i-MAX_ACR) \
					    + ADC_SOU_OFFSET;

		temp_des_addr = ADDR_BASE_ACR \
						+ ACR_FACTOR*(i-MAX_ACR) 	\
						+ ADC_ACR_OFFSET \
						+ DES_OFFSET;

		Servo_Motor_Build_DmaLink(i,temp_sou_addr,temp_des_addr,0);

	}

	Servo_Fpga_Gen_Write(ADDR_BASE_DMA+DMA_REG_OFFSET,0xfffff);

}
/*----------------------------------------------------------------------
 * 函数名称:void MOTOR_b_Build_DmaLink(int index,
						  int sou_addr,
						  int des_addr,
						  int linu_num)
 * 函数输入:
 * 函数输出:
 * 函数备注:配置DMA功能链表
 * -------------------------------------------------------------------*/
void Servo_Motor_Build_DmaLink(uint32_t index,
						   uint32_t sou_addr,
						   uint32_t des_addr,
						   uint32_t linu_num)
{
	uint32_t temp_cmd_addr = 0;

	temp_cmd_addr =  (index*INDEX_OFFSET) + ADDR_BASE_DMA \
							 + LINK_ADD_OFFSET;

	Servo_Fpga_Gen_Write(temp_cmd_addr,SIGNLE_CMD);

	Servo_Fpga_Gen_Write(temp_cmd_addr+SOU_REG_OFFSET,sou_addr);

	Servo_Fpga_Gen_Write(temp_cmd_addr+DES_REG_OFFSET,des_addr);

	Servo_Fpga_Gen_Write(temp_cmd_addr+LINK_REG_OFFSET,linu_num);

}
/*----------------------------------------------------------------------
 *void Servo_Motor_TorqueFeed(SERVO_AXIS_ENUM axis_id,
							int64_t torque_feed,
							int64_t* output)
 * 函数输入:
 * 函数输出:
 * 函数备注:配置DMA功能链表
 * -------------------------------------------------------------------*/
 void virtual_servo_device::Servo_Motor_TorqueFeed(SERVO_AXIS_ENUM axis_id,
							int64_t torque_feed,
							int64_t* output)
{
	*output = torque_feed*tor2current_factor[axis_id].conv_factor;
}

