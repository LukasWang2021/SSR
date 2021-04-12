/*
 * Servo_CheckErr.c
 *
 *  Created on: 2019年7月26日
 *      Author: qichao.wang
 */
#include "Servo_CheckErr.h"
#include "Servo_Para.h"
#include "Servo_General.h"
#include "Servo_Motor.h"
#include "Servo_Fpga.h"
#include "error_process.h"
#include "Servo_Axis.h"
#include "Servo_Safety.h"
#include "Reg_Address.h"


using namespace virtual_servo_device;


//错误框架内部变量
static PARA_ERR_DATA_STRUCT check_err_data[AXIS_MAX];
//编码器信号MASK
static SERVO_ENCODE_MASK_SHIFT_STRUCT encode_mask_shift[ENCODE_FRAME_LENGTH] = {
	{ 0x01,0x00 },
	{ 0x02,0x01 },
	{ 0x04,0x02 },
	{ 0x08,0x03 },
	{ 0x10,0x04 },
	{ 0x20,0x05 },
	{ 0x40,0x06 },
	{ 0x80,0x07 },
	{ 0x100,0x08 },
	{ 0x200,0x09 }
};

static SERVO_FPGA_EXUTE_STRUCT ipm_mask[AXIS_MAX] = {
	{ 0x01,0x00 },
	{ 0x02,0x01 },
	{ 0x04,0x02 },
	{ 0x08,0x03 },
	{ 0x10,0x04 },
	{ 0x20,0x05 },
	{ 0x40,0x06 },
	{ 0x80,0x07 }
};

static DC_BUS_LIMIT_STRUCT dc_bus_limit = { 400,270,30 };//母线电压保护值
static uint64_t load_sum[AXIS_MAX] = { 1871704703661,1871704703661,1871704703661,1871704703661  \
,1871704703661, 1871704703661,1871704703661, 1871704703661
};//过载保护值
static SERVO_GEN_ERR_STRUCT ipm_curr[AXIS_MAX];//ipm_over_curr 错误位
static SERVO_GEN_ERR_STRUCT ipm_temp[AXIS_MAX]; //motor_over_temp 错误位
static SERVO_ENCODE_ERR_STRUCT encode_err[AXIS_MAX]; //encoder error 错误位

static volatile uint32_t* err_code = (volatile uint32_t*)(0x3a000000);
/*-------------------------------------------------------------
*函数名称 :void Servo_CheckErr_Init(void)
*函数输入 :
*函数输出 :
*函数描述 :母线电压检测，分为正常，过压、欠压三种工作模式
--------------------------------------------------------------*/
void virtual_servo_device::Servo_CheckErr_Init(void)
{
	error_process_init();
}
/*-------------------------------------------------------------
*函数名称 :void Servo_DCBUS_Vol_Check(void)
*函数输入 :
*函数输出 :
*函数描述 :母线电压检测，分为正常，过压、欠压三种工作模式
--------------------------------------------------------------*/
int virtual_servo_device::Servo_DCBUS_Vol_Check(void)
{
	static int vol_over_count = 0;

	static int vol_owe_count = 0;

	uint32_t temp_code = 0;

	int temp_data = 0;

	int temp_flag;

	temp_data = Get_DCBUS_Vol();

	if ((temp_data<dc_bus_limit.max_value) && (temp_data>dc_bus_limit.min_value))
	{
		/*设置标志位*/
		temp_flag = 0;

		vol_over_count = 0;

		vol_owe_count = 0;
	}
	else if (temp_data >dc_bus_limit.max_value)
	{
		vol_over_count++;

		if (vol_over_count == dc_bus_limit.count)
		{
			for (int axis_num = 0; axis_num<AXIS_MAX; axis_num++)
			{
				Servo_Motor_ConMotor(static_cast<SERVO_AXIS_ENUM>(axis_num), MOTOR_DISABLE);
				Servo_Axis_Set_AxisErr(static_cast<SERVO_AXIS_ENUM>(axis_num));

				CONV_ERR_CODE(uint32_t, SEVER01, DC_BUS_ENUM, axis_num,dcbus_over, temp_code);
				error_process_report(static_cast<SERVO_AXIS_ENUM>(axis_num),temp_code);
				Servo_Safety_b_Write_Cmd_Ana(ZERO_STOP);
				*err_code = temp_code;
			}

			vol_over_count = 0;
		}

		temp_flag = 1;//过压
	}
	else if (temp_data <dc_bus_limit.min_value)
	{
		vol_owe_count++;

		if (vol_owe_count == dc_bus_limit.count)
		{
			for (int axis_num = 0; axis_num<AXIS_MAX; axis_num++)
			{
				Servo_Motor_ConMotor(static_cast<SERVO_AXIS_ENUM>(axis_num), MOTOR_DISABLE);
				Servo_Axis_Set_AxisErr(static_cast<SERVO_AXIS_ENUM>(axis_num));

				CONV_ERR_CODE(uint32_t, SEVER01, DC_BUS_ENUM,axis_num, dcbus_owe, temp_code);
				error_process_report(static_cast<SERVO_AXIS_ENUM>(axis_num),temp_code);
				Servo_Safety_b_Write_Cmd_Ana(ZERO_STOP);
				*err_code = temp_code;
			}

			vol_owe_count = 0;
		}

		temp_flag = 2;//欠压
	}

	return temp_flag;

}

/*--------------------------------------------------------------------------------
*函数名称;int Get_DCBUS_Vol(void)
*函数输入:
*函数输出:
*函数备注:获取母线电压值
* ------------------------------------------------------------------------------*/
int virtual_servo_device::Get_DCBUS_Vol(void)
{
	static int32_t vol_filter[FILTER_MAX] = { 0 };

	int16_t temp_vol1 = 0;

	int32_t temp_vol2 = 0;

	temp_vol1 = 0;//(int16_t)alt_read_word(VD_WATCHWINDOW);

	temp_vol2 = (int32_t)(temp_vol1*VOL_FATCTOR);

	temp_vol2 = temp_vol2 + VOL_OFFSET;

	vol_filter[0] = temp_vol2;

	for (int i = (FILTER_MAX - 1); i>0; i--)
	{
		vol_filter[i] = vol_filter[i - 1];
	}

	temp_vol2 = 0;

	for (int i = 0; i<FILTER_MAX; i++)
	{
		temp_vol2 += vol_filter[i];
	}

	temp_vol2 = temp_vol2 >> 3;//求8次平均值

	return temp_vol2;
}
/*-------------------------------------------------------------
*函数名称 :void Servo_Ipm_OverCur_Check(void)
*函数输入 :
*函数输出 :
*函数描述 ：上升沿检测IPM过流信号，检测到关断PWM使能
--------------------------------------------------------------*/
void virtual_servo_device::Servo_Ipm_OverCur_Check(void)
{
	int temp_data = 0;
	int temp_status = 0;
	int err_high = 0;

	uint32_t temp_code = 0;

	temp_status = 0;//Servo_Fpga_Gen_Read(MOTOR_STATUS);//高8位

	temp_status = temp_status >> 8;

	for (int axis_num = 0; axis_num<AXIS_MAX; axis_num++)
	{
		temp_data = (temp_status&ipm_mask[axis_num].reg_mask) >> ipm_mask[axis_num].reg_shift;

		{

			ipm_curr[axis_num].current_data = temp_data;

			err_high = ipm_curr[axis_num].current_data&(~ipm_curr[axis_num].pre_data);

			ipm_curr[axis_num].pre_data = ipm_curr[axis_num].current_data;

			if (err_high)
			{
				 Servo_Motor_ConMotor(static_cast<SERVO_AXIS_ENUM>(axis_num),MOTOR_DISABLE);
				 Servo_Axis_Set_AxisErr(static_cast<SERVO_AXIS_ENUM>(axis_num));

				 CONV_ERR_CODE(uint32_t, SEVER01, IPM_MOD_ENUM,axis_num, ipm_over_curr, temp_code);
				 error_process_report(static_cast<SERVO_AXIS_ENUM>(axis_num),temp_code);
				 Servo_Safety_b_Write_Cmd_Ana(ZERO_STOP);
				 *err_code = temp_code;

			}

		}

	}
}

/*-------------------------------------------------------------
*函数名称 : void Servo_IQ_Overload_Check(MO_AXIS_ENUM axis_id)
*函数输入 :轴枚举
*函数输出 :
*函数描述 :
1.电机过流保护，IQ最大峰值不超过2.5倍的额定
2.电机过载保护，累积IQ值超过限制值则报错
--------------------------------------------------------------*/
void virtual_servo_device::Servo_IQ_Overload_Check(SERVO_AXIS_ENUM axis_id)
{
	 int64_t temp_data = 0;
	 uint64_t temp_var_data = 0;

	 uint32_t temp_code = 0;

	 static uint64_t motor_over_load[AXIS_MAX] = {0};

	 temp_data = Servo_Fpga_Read(axis_id,IQ_FBACK_RAM);

	 Servo_IQ_PeakCur_Check(axis_id,temp_data);

	 temp_var_data = (uint64_t)((temp_data>0)?(temp_data):(0-temp_data));
	 /*----------------------过载计算和退饱和------------*/
	 if(temp_var_data>16777216)
	 {
		 motor_over_load[axis_id] += (temp_var_data*temp_var_data)>>24;//超过额定直接累加

	 }else{

//		 if(motor_over_load[axis_id]<16777216)
//		 {
//			 motor_over_load[axis_id] = 0;		  //低于额定的累计,积分数值清零
//
//		 }else{
//			 motor_over_load[axis_id] += (temp_var_data-16777216);//否则退饱和
//		 }
	 }
	 /*-------------------出错处理-------------------*/
	 if(motor_over_load[axis_id]>load_sum[axis_id])
	 {
		 for(int i=0; i<AXIS_MAX; i++)
		 {
	 		Servo_Motor_ConMotor(static_cast<SERVO_AXIS_ENUM>(i),MOTOR_DISABLE);
	 		Servo_Axis_Set_AxisErr(static_cast<SERVO_AXIS_ENUM>(axis_id));
		 }

	 	motor_over_load[axis_id] = 0;

	 	CONV_ERR_CODE(uint32_t, SEVER01, MOTOR_ENUM,axis_id, motor_over_current, temp_code);
	 	error_process_report(axis_id,temp_code);
	 	Servo_Safety_b_Write_Cmd_Ana(ZERO_STOP);
	 	*err_code = temp_code;

	 }else{

	 }
}

/*--------------------------------------------------------------------------------------------------
* 函数名称:    void Servo_IQ_PeakCur_Check(MO_AXIS_ENUM axis_id,
int iq_data)
* 函数输入:轴号，当前IQ值
* 函数输出:无
* 函数备注:框架静态函数,在过载函数中调用,峰值超过最大值的上升沿报警
* ------------------------------------------------------------------------------------------------*/
void virtual_servo_device::Servo_IQ_PeakCur_Check(SERVO_AXIS_ENUM axis_id,
	int iq_data)
{
	static int peak_status[AXIS_MAX][2];

	uint32_t temp_code = 0;

	int temp_data = 0;

	temp_data = (iq_data>0) ? iq_data : (0 - iq_data);

	peak_status[axis_id][0] = ((temp_data - 50331648) >= 0) ? 1 : 0;

	temp_data = peak_status[axis_id][0] & (~peak_status[axis_id][1]);//上升沿触发

	peak_status[axis_id][1] = peak_status[axis_id][0];

	if (temp_data)
	{

        for(uint32_t i=0; i<AXIS_MAX; i++)
        {
		    Servo_Motor_ConMotor(static_cast<SERVO_AXIS_ENUM>(i), MOTOR_DISABLE);
		    Servo_Axis_Set_AxisErr(static_cast<SERVO_AXIS_ENUM>(i));
        }
		CONV_ERR_CODE(uint32_t, SEVER01, MOTOR_ENUM, axis_id,motor_peak_current,temp_code);
		error_process_report(axis_id,temp_code);
		Servo_Safety_b_Write_Cmd_Ana(ZERO_STOP);

		*err_code = temp_code;
	}
}


/*-------------------------------------------------------------
*函数名称 :void Servo_Temp_Check(void)
*函数输入 :
*函数输出 :
*函数描述 :过温检测
--------------------------------------------------------------*/
void virtual_servo_device::Servo_Temp_Check(void)
{
	int temp_data = 0;
	int temp_status = 0;
	int err_high = 0;

	uint32_t temp_code = 0;

	temp_status = 0;//Servo_Fpga_Gen_Read(MOTOR_STATUS);//低8位

	for (int axis_num = 0; axis_num<AXIS_MAX; axis_num++)
	{
		temp_data = (temp_status&ipm_mask[axis_num].reg_mask) >> ipm_mask[axis_num].reg_shift;

		if (temp_data)
		{
			ipm_temp[axis_num].current_data = temp_data;

			err_high = ipm_temp[axis_num].current_data&(~ipm_temp[axis_num].pre_data);

			ipm_temp[axis_num].pre_data = ipm_temp[axis_num].current_data;

			if (err_high)
			{
				Servo_Motor_ConMotor(static_cast<SERVO_AXIS_ENUM>(axis_num), MOTOR_DISABLE);
				Servo_Axis_Set_AxisErr(static_cast<SERVO_AXIS_ENUM>(axis_num));

				CONV_ERR_CODE(uint32_t, SEVER01, IPM_MOD_ENUM, axis_num,motor_over_temp, temp_code);
				error_process_report(static_cast<SERVO_AXIS_ENUM>(axis_num),temp_code);
				Servo_Safety_b_Write_Cmd_Ana(ZERO_STOP);

				*err_code = temp_code;
			}
		}
	}
}

/*-------------------------------------------------------------
*函数名称 : void Servo_OverSpeed_Check(int set_speed,
int mea_speed,
MO_AXIS_ENUM axis_id)
*函数输入 :设定速度、反馈速度，轴枚举
*函数输出 :
*函数描述 :1.检测电机速度大于最大速度值的115%，持续200ms报错
--------------------------------------------------------------*/
void virtual_servo_device::Servo_OverSpeed_Check(int set_speed,
	int mea_speed,
	SERVO_AXIS_ENUM axis_id)
{
	uint32_t temp_code = 0;

	static int over_speed_count[AXIS_MAX] = { 0 };

	//over_speed_limit为额定的115%，过保护
	if (mea_speed>check_err_data[axis_id].over_speed_limit)
	{
		over_speed_count[axis_id]++;

		if (over_speed_count[axis_id] == check_err_data[axis_id].over_speed_limit_time)
		{
			Servo_Motor_ConMotor(axis_id, MOTOR_DISABLE);
			Servo_Axis_Set_AxisErr(axis_id);

			CONV_ERR_CODE(uint32_t, SEVER01, CON_ENUM,axis_id, motor_over_speed, temp_code);
			error_process_report(axis_id,temp_code);

			Servo_Safety_b_Write_Cmd_Ana(ZERO_STOP);

			over_speed_count[axis_id] = 0;
		}
	}
	else
	{
		over_speed_count[axis_id] = 0;
	}
}

/*---------------------------------------------------------------------------------
* 函数名称:void Servo_OweSpeed_Check(MO_AXIS_ENUM axis_id
,int vel_err )
* 函数输入:
* 函数输出:
* 函数备注:失速报警
* -------------------------------------------------------------------------------*/
void virtual_servo_device::Servo_OweSpeed_Check(SERVO_AXIS_ENUM axis_id
	, int64_t vel_err)
{
	static int32_t vel_err_count[AXIS_MAX] = { 0 };

	int64_t temp_data = 0;

	uint32_t temp_code = 0;

	temp_data = (vel_err>0) ? vel_err : (0 - vel_err);

	if (temp_data >= check_err_data[axis_id].vel_err_limit) {

		if (vel_err_count[axis_id] <= check_err_data[axis_id].vel_setting_time)
		{
			vel_err_count[axis_id]++;

		}
		else {
			vel_err_count[axis_id] = 0;

			Servo_Motor_ConMotor(axis_id, MOTOR_DISABLE);
			Servo_Axis_Set_AxisErr(axis_id);

			CONV_ERR_CODE(uint32_t, SEVER01, CON_ENUM, axis_id,motor_owe_speed, temp_code);
			error_process_report(axis_id,temp_code);

			Servo_Safety_b_Write_Cmd_Ana(ZERO_STOP);
		}
	}
	else
	{
		vel_err_count[axis_id] = 0;
	}
}

#define VEL_LIMIT_PTHREAD 279620
/*---------------------------------------------------------------------------------
* 函数名称:void Servo_Block_Motion_Check(MO_AXIS_ENUM axis_id
,int vel_err )
* 函数输入:
* 函数输出:
* 函数备注:检测速度超差稳定时间
* -------------------------------------------------------------------------------*/
void virtual_servo_device::Servo_Block_Motion_Check(SERVO_AXIS_ENUM axis_id
	, int64_t back_vel,
	int64_t vel_err)
{
	static int32_t block_err_count[AXIS_MAX] = { 0 };

	int64_t temp_back_vel = 0;
	int64_t temp_vel_err = 0;
	uint32_t temp_code = 0;

	temp_back_vel = (back_vel>0) ? (back_vel) : (0 - back_vel);
	temp_vel_err = (vel_err>0) ? (vel_err) : (0 - vel_err);

	if ((temp_back_vel <= check_err_data[axis_id].block_speed) &&
		(temp_vel_err>check_err_data[axis_id].block_speed_err_limit))
	{
		if (block_err_count[axis_id] <= check_err_data[axis_id].block_time)
		{
			block_err_count[axis_id]++;
		}
		else {

			block_err_count[axis_id] = 0;

			Servo_Motor_ConMotor(axis_id, MOTOR_DISABLE);
			Servo_Axis_Set_AxisErr(axis_id);

			CONV_ERR_CODE(uint32_t, SEVER01, CON_ENUM, axis_id,motor_block_err, temp_code);
			error_process_report(axis_id,temp_code);

			Servo_Safety_b_Write_Cmd_Ana(ZERO_STOP);
		}
	}
	else
	{

		block_err_count[axis_id] = 0;
	}
}
/*-----------------------------------------------------------------------------------
* 函数名称:void Servo_Encode_Err_Check(MO_AXIS_ENUM axis_id)
* 函数输入:
* 函数输出:
* 函数备注:检测编码器报错
* --------------------------------------------------------------------------------*/
void virtual_servo_device::Servo_Encode_Err_Check(SERVO_AXIS_ENUM axis_id)
{
	int err_high = 0;//错误发生
	uint32_t temp_encode = 0;
	int temp_add = 0;
	uint32_t temp_code = 0;

	temp_add = ENC_ERR_CHECK + axis_id*FPGA_AXIS_OFFSET;

	temp_encode = 0;// alt_read_word(temp_add);

	temp_encode = temp_encode & 0x3C0;//低4位无效

	if (temp_encode)
	{
		for (int encode_num = 0; encode_num<ENCODE_FRAME_LENGTH; encode_num++)
		{
			encode_err[axis_id].current_data[encode_num] = \
				((temp_encode&encode_mask_shift[encode_num].mask_encode) >> encode_mask_shift[encode_num].shift_encode);

			err_high = encode_err[axis_id].current_data[encode_num]													\
				&(~encode_err[axis_id].pre_data[encode_num]);

			encode_err[axis_id].pre_data[encode_num] = encode_err[axis_id].current_data[encode_num];
			/*错误是否发生*/
			if (err_high)
			{
				switch (encode_num)
				{
				case OVER_SPEED:
				case FULL_ABS_STATUS:
				case COUNT_ERR:
				case COUNTER_OVERFLOAW:
				case MULTI_ERR:
				case BATERRY_ERR:
				case BATERRY_ALM:
				{
					CONV_ERR_CODE(uint32_t, INFO, ENCODE_ENUM, axis_id, (encode_over_speed + encode_num), temp_code);

					error_process_report(axis_id,temp_code);

					break;
				}
				case PROTOCAL_ABNOMAL:
				case PROTOCAL_DISCONNECT:
				{
					Servo_Motor_ConMotor(axis_id, MOTOR_DISABLE);

					Servo_Axis_Set_AxisErr(axis_id);

					CONV_ERR_CODE(uint32_t, SEVER01, ENCODE_ENUM, axis_id, (encode_over_speed + encode_num), temp_code);

					error_process_report(axis_id,temp_code);

					Servo_Safety_b_Write_Cmd_Ana(ZERO_STOP);
					*err_code = temp_code;

					break;
				}
				default:break;

				}
			}
		}
	}
	else
	{
		for (int encode_num = 0; encode_num<ENCODE_FRAME_LENGTH; encode_num++)
		{
			encode_err[axis_id].current_data[encode_num] = 0;

			encode_err[axis_id].pre_data[encode_num] = 0;

		}
	}
}
/*---------------------------------------------------------------------------------
* 函数名称:void Servo_PosErr_Check(MO_AXIS_ENUM axis_id)
* 函数输入:
* 函数输出:
* 函数备注:检测位置超差
* -------------------------------------------------------------------------------*/
void virtual_servo_device::Servo_PosErr_Check(SERVO_AXIS_ENUM axis_id
	, int64_t pos_err)
{
	static int32_t pos_err_count[AXIS_MAX] = { 0 };

	int64_t temp_data = 0;

	uint32_t temp_code = 0;

	temp_data = (pos_err>0) ? pos_err : (0 - pos_err);

	if (temp_data >= check_err_data[axis_id].pos_err_limit) {

		if (pos_err_count[axis_id] <= check_err_data[axis_id].pos_setting_time)
		{
			pos_err_count[axis_id]++;

		}
		else {
			pos_err_count[axis_id] = 0;

			Servo_Motor_ConMotor(axis_id, MOTOR_DISABLE);

			Servo_Axis_Set_AxisErr(axis_id);

			CONV_ERR_CODE(uint32_t, INFO, CON_ENUM, axis_id, pos_over_limit, temp_code);

			error_process_report(axis_id,temp_code);

			Servo_Safety_b_Write_Cmd_Ana(ZERO_STOP);
		}
	}
	else {

		pos_err_count[axis_id] = 0;
	}

}

/*----------------------------------------------------------------
*函数名称 : void Servo_s_Check_PhaseErr(MO_AXIS_ENUM axis_id)
*函数输入 :轴枚举
*函数输出 :
*函数描述 :电机过流保护，FPGA相电流超过限制值报错
-----------------------------------------------------------------*/
void virtual_servo_device::Servo_PhaseErr_Check(SERVO_AXIS_ENUM axis_id)
{
	uint32_t temp_add = 0;
	uint32_t temp_data = 0;

	uint32_t temp_high = 0;

	uint32_t temp_code = 0;

	static int edge_check[AXIS_MAX][2] = {
		{ 0,0 },
		{ 0,0 },
		{ 0,0 },
		{ 0,0 },
		{ 0,0 },
		{ 0,0 }
	};

	temp_add = PHASE_CURRENT_ERR + axis_id*FPGA_CURRENT_OFFSET;

	temp_data = 0;//alt_read_word(temp_add);

	temp_data = temp_data & 0x01;

	edge_check[axis_id][0] = temp_data;

	temp_high = edge_check[axis_id][0] & (~edge_check[axis_id][1]);

	edge_check[axis_id][1] = edge_check[axis_id][0];

	if (temp_high)
	{
		Servo_Motor_ConMotor(axis_id, MOTOR_DISABLE);

		Servo_Axis_Set_AxisErr(axis_id);

		CONV_ERR_CODE(uint32_t, SEVER01, MOTOR_ENUM, axis_id, pha_curr_err, temp_code);

		error_process_report(axis_id,temp_code);

		Servo_Safety_b_Write_Cmd_Ana(ZERO_STOP);

		*err_code = temp_code;

	}
}

void virtual_servo_device::test_err(void)
{
	if(*err_code==0XAA55AA55)
	{
		error_process_report(AXIS_0,0X70D10001);
		error_process_report(AXIS_0,0X70D10102);
		error_process_report(AXIS_0,0X70D10203);
		error_process_report(AXIS_0,0X70D10304);
		error_process_report(AXIS_0,0X70D10405);
		error_process_report(AXIS_0,0X70D10506);
		error_process_report(AXIS_0,0X70D10607);
		error_process_report(AXIS_0,0X70D10708);

		printf("error success\r\n");

		*err_code = 0;
	}


}

