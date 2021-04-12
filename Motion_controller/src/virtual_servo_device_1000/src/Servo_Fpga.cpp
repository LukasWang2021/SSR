/*
 * Servo_Fpga.c
 *
 *  Created on: 2019年7月26日
 *      Author: qichao.wang
 */

#include "Reg_Address.h"
#include "Servo_Fpga.h"


using namespace virtual_servo_device;

/*----------------------框架静态数据-----------------------*/
static SERVO_FPGA_REG_CFG_STRUCT axis_reg_info[AXIS_MAX];
static SERVO_FPGA_IQID_STRUCT current_dq[AXIS_MAX];
static SERVO_FPGA_ENCODE_BACK_STRUCT encode_back[AXIS_MAX];
static SERVO_FPGA_VEL_BACK_STRUCT vel_back[AXIS_MAX];
/*----------------------框架静态函数-----------------------*/
static void Servo_Fpga_LedCfg(void);

void virtual_servo_device::Servo_Fpga_Init(void)
{
	for(int i=0; i<AXIS_MAX; i++)
	{
		/*1轴寄存器*/
		AXIS_REG_CFG(axis_reg_info[i].enc_reg,ENACODE_TOTAL_BASE,i);
		AXIS_REG_CFG(axis_reg_info[i].mult_rap_reg,ENCODE_RAP_BASE,i);
		AXIS_REG_CFG(axis_reg_info[i].vel_reg,SP_CNT_ERR,i);
		AXIS_REG_CFG(axis_reg_info[i].encode_filter_reg,ENCODE_FILTER_REG,i);
		AXIS_REG_CFG(axis_reg_info[i].encode_err_ram,ENC_ERR_CHECK,i);
		AXIS_REG_CFG(axis_reg_info[i].reset_fpga_ram,REST_FPGA,i);
		AXIS_REG_CFG(axis_reg_info[i].dead_comp_ram,DEAD_ZONE_COMPEN,i);
		AXIS_REG_CFG(axis_reg_info[i].current_filter_reg,CURRENT_FILTER_CON,i);
		AXIS_RAM_CFG(axis_reg_info[i].cur_kp_ram,i,CURR_LOOP_KP);
		AXIS_RAM_CFG(axis_reg_info[i].cur_ki_ram,i,CURR_LOOP_KI);
		AXIS_RAM_CFG(axis_reg_info[i].iq_ram,i,IQ_CURRENT);
		AXIS_RAM_CFG(axis_reg_info[i].id_ram,i,ID_CURRENT);
		AXIS_RAM_CFG(axis_reg_info[i].motor_angle_ram,i,STUDY_THETA);
		AXIS_RAM_CFG(axis_reg_info[i].polar_nums_ram,i,POLAR_NUMS);
		AXIS_RAM_CFG(axis_reg_info[i].cur_sample_ram,i,CURR_FACTOR);
		AXIS_RAM_CFG(axis_reg_info[i].polar_angle_ram,i,POLAR_ANG_COM);
		AXIS_RAM_CFG(axis_reg_info[i].iq_inter_ram,i,PID_IQ_INT);
		AXIS_RAM_CFG(axis_reg_info[i].id_inter_ram,i,PID_ID_INT);
		AXIS_RAM_CFG(axis_reg_info[i].iq_dump_ram,i,VOL_OUT_VQ);
		AXIS_RAM_CFG(axis_reg_info[i].iq_fback_ram,i,FEEDBAK_IQ);
		AXIS_RAM_CFG(axis_reg_info[i].vq_out,i,VOL_OUT_VQ);
		AXIS_RAM_CFG(axis_reg_info[i].vd_out,i,VOL_OUT_VD);
		AXIS_RAM_CFG(axis_reg_info[i].current_loop_sa_reg,i,CURRENT_LOOP_SAT);

	}

	Servo_Fpga_LedCfg();
}
/*--------------------------------------------------------------------------------
 *函数名称 :void MOGR_Get_Mea_Ptr(MOGR_MEA_DATA_STRUCT **sn_data,MO_SN_ENUM axis_id)
 *函数输入 :
 *函数输出 :
 *函数描述 :获取GR框架传感器数据指针
--------------------------------------------------------------------------------*/
void virtual_servo_device::Servo_Fpga_Get_EncPtr(SERVO_FPGA_ENCODE_BACK_STRUCT **encode_ptr
					  ,SERVO_AXIS_ENUM axis_id)
{
	*encode_ptr = &encode_back[axis_id];
}
/*--------------------------------------------------------------------------------
 *函数名称 :void MOGR_Get_Mea_Ptr(MOGR_MEA_DATA_STRUCT **sn_data,MO_SN_ENUM axis_id)
 *函数输入 :
 *函数输出 :
 *函数描述 :获取GR框架传感器数据指针
--------------------------------------------------------------------------------*/
void virtual_servo_device::Servo_Fpga_Get_VelPtr(SERVO_FPGA_VEL_BACK_STRUCT **vel_ptr
					  ,SERVO_AXIS_ENUM axis_id)
{
	*vel_ptr = &vel_back[axis_id];
}
/*----------------------------------------------------------------------------------
 *函数名称 :void MOGR_Get_Out_Ptr(MOGR_OUTPUT_DATA_STRUCT **ac_data,MO_AXIS_ENUM ac_id)
 *函数输入 :
 *函数输出 :
 *函数描述 :获取GR框架传感器数据指针
-----------------------------------------------------------------------------------*/
void virtual_servo_device::Servo_Fpga_Get_IqdPtr(SERVO_FPGA_IQID_STRUCT **iqd_data
					 ,SERVO_AXIS_ENUM axis_id)
{
	*iqd_data = &current_dq[axis_id];
}
/*-------------------------------------------------------------
 *函数名称 :void MOGR_p_Write_FPGA(void)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :HPS控制FPGA保护操作
--------------------------------------------------------------*/
void virtual_servo_device::Servo_Write_p_FPGA(SERVO_AXIS_ENUM axis_id)
{
	Servo_Fpga_Write(axis_id,IQ_RAM
			,current_dq[axis_id].iq_out);
}
/*-------------------------------------------------------------
 *函数名称 :void Servo_Read_p_FPGA(SERVO_AXIS_ENUM axis_id)
 *函数输入 :
 *函数输出 :
 *函数描述 :读地址为外部各轴状态位
--------------------------------------------------------------*/
void virtual_servo_device::Servo_Read_p_FPGA(SERVO_AXIS_ENUM axis_id)
{
	encode_back[axis_id].encode_total =
			(int64_t)Servo_Fpga_Read(axis_id,ENC_REG);

	vel_back[axis_id].vel_act =
			Servo_Fpga_Read(axis_id,VEL_REG);
}
/*----------------------------------------------------------------
 * 函数名称:void MOTB_gpio_start(void)
 * 函数输入:
 * 函数输出:
 * 函数备注:GPIO初始化
 * ----------------------------------------------------------------*/
 void Servo_Fpga_LedCfg(void)
{
//	WRITE_HPS(CORE_LIGNT_CON_BASE,LED_CONFIG);
//	WRITE_HPS(CORE_LIGHT_DAT_BASE,LED_CONFIG);
}
#define LED_SHINK_COUNT 200
/*---------------------------------------------------------------------------
 *函数名称:void MOGR_HeartBeat(void)
 *函数输出:
 *函数输入:
 *函数备注:void MOGR_HeartBeat(void)
 *-------------------------------------------------------------------------- */
 void virtual_servo_device::Servo_Fpga_Heartbeat(void)
{
	static int led_count = 0;

	uint32_t temp_data = 0;

	static uint8_t led_val = 1;

	READ_HPS(CORE_LIGHT_DAT_BASE,temp_data);

	if(led_count == LED_SHINK_COUNT)
	{

		temp_data = temp_data&LED_MASK;

		led_val = ~led_val;

		temp_data |= led_val<<LED_SHIFT;

		WRITE_HPS(CORE_LIGHT_DAT_BASE,temp_data);

		led_count = 0;

	}else{
		led_count++;
	}

}
/*----------------------------------------------------------------
 * 函数名称:void MOGR_s_Reset_Fpga(MO_AXIS_ENUM axis_id)
 * 函数输入:???
 * 函数输出:
 * 函数备注:复位FPGA工作模式，复位FPGA错误
 * --------------------------------------------------------------*/
 void virtual_servo_device::Servo_Fpga_Reset(SERVO_AXIS_ENUM axis_id)
{
	unsigned int temp_add;

	FPGA_REG_ADD_CAL(temp_add,FPGA_MODE);
	/*磁极对准状态恢复到正常工作状态*/
	Servo_Fpga_Gen_Write(temp_add,FPGA_CLEAR);

	for(int i=0; i<AXIS_MAX; i++)
	{
		temp_add = axis_reg_info[i].reset_fpga_ram;
		/*清除FPGA错误标志位*/
//		alt_write_word(REST_FPGA+i*FPGA_AXIS_OFFSET,FPGA_RESET);
	}
}
/*-------------------------------------------------------------------
 * 函数名称:void MOGR_s_FeedDog(int input_data)
 * 函数输入:无
 * 函数输出:无
 * 函数备注:和FPGA握手喂狗
 * ------------------------------------------------------------------*/
 void virtual_servo_device::Servo_Fpga_FeedDog(int32_t input_data)
{
//	alt_write_word(FEED_DOG,input_data);
}
/*---------------------------------------------------------------------------
 *函数名称:void MOGR_s_Write_Fpga(MO_AXIS_ENUM axis_id,
					   SERVO_FPGA_REG_CFG_ENUM map_id
					   ,int out2fpga)
 *函数输出:
 *函数输入:
 *函数备注:获取编码器多圈信息
 *-------------------------------------------------------------------------- */
 void virtual_servo_device::Servo_Fpga_Write(SERVO_AXIS_ENUM axis_id,
						SERVO_FPGA_REG_CFG_ENUM map_id
					   ,int32_t out2fpga)
{
	uint32_t temp_add = 0;

	int32_t *temp_ptr = (int32_t *)(&axis_reg_info[axis_id]);

	temp_add = *(temp_ptr + map_id);

//	alt_write_word(temp_add,out2fpga);
}
/*---------------------------------------------------------------------------
 *函数名称:void MOGR_s_Gen_WriteFpga(uint32_t gen_addr,
						  int input_data)
 *函数输出:
 *函数输入:
 *函数备注:跟轴无关的写fpga通用寄存器可以调用接口
 *-------------------------------------------------------------------------- */
 void virtual_servo_device::Servo_Fpga_Gen_Write(uint32_t gen_addr,
						  int32_t input_data)
{
//	alt_write_word(gen_addr,input_data);
}
/*---------------------------------------------------------------------------
 *函数名称:void MOGR_s_Gen_WriteFpga(uint32_t gen_addr,
						  int input_data)
 *函数输出:
 *函数输入:
 *函数备注:读取fpga和轴无关的参数数据
 *-------------------------------------------------------------------------- */
 uint32_t virtual_servo_device::Servo_Fpga_Gen_Read(uint32_t gen_addr)
{
	uint32_t ret_val = 0;

//	ret_val = alt_read_word(gen_addr);

	return ret_val;
}
/*---------------------------------------------------------------------------
 *函数名称:int32_t Servo_Fpga_Read(SERVO_AXIS_ENUM axis_id,
					SERVO_FPGA_REG_CFG_ENUM map_id)
 *函数输出:
 *函数输入:
 *函数备注:获取编码器多圈信息
 *-------------------------------------------------------------------------- */
 int32_t virtual_servo_device::Servo_Fpga_Read(SERVO_AXIS_ENUM axis_id,
					SERVO_FPGA_REG_CFG_ENUM map_id)
{
	uint32_t temp_add = 0;

	int32_t ret_val = 0;

	int32_t *temp_ptr = (int32_t *)(&axis_reg_info[axis_id]);

	temp_add = *(temp_ptr + map_id);

//	ret_val = (int32_t)alt_read_word(temp_add);

	return ret_val;
}

