/*
 * Servo_CheckErr.h
 *
 *  Created on: 2019年7月15日
 *      Author: qichao.wang
 */

#ifndef SERVO_CHECKERR_H_
#define SERVO_CHECKERR_H_

#include "Servo_Para.h"
#include "Servo_General.h"

#define ENCODE_FRAME_LENGTH 0x0A
#define MOTOR_RATED 16777216
#define VOL_FATCTOR 0.009155
#define VOL_OFFSET 300
#define FILTER_MAX 8
#define MAX_ERR_LIMIT 400
#define MAX_ERR_COM    3

namespace virtual_servo_device{

typedef struct
{
	int32_t max_value;
	int32_t count;
	int32_t min_value;
}DC_BUS_LIMIT_STRUCT;

typedef struct
{
	int iq_current_limit;
	int over_speed_limit;
	int over_speed_limit_time;
	int vel_err_limit;
	int vel_setting_time;
	int block_time;
	int block_speed_err_limit;
	int block_speed;
	int motor_current_limit;
	int pos_err_limit;
	int pos_setting_time;
	int filter_data[6];

}PARA_ERR_DATA_STRUCT;

typedef enum
{
	NONE = 0x01,
	INFO = 0x02,
	PAUSE_L = 0x03,
	PAUSE_G = 0x04,
	STOP_L = 0x05,
	STOP_G = 0x06,
	SEVER01 = 0x07,
	ABORTL = 0x08,
	ABORTG = 0x09,
	SEVER02 = 0x0A,
	SYSTEM = 0x0B

}SERVO_ERR_CODE_ENUM;

typedef enum
{
	DC_BUS_ENUM = 0xD2,//直流母线电压错误
	IPM_MOD_ENUM = 0xD3,//IPM故障
	ENCODE_ENUM = 0xD4,//编码器错误
	MOTOR_ENUM = 0xD5,//电机错误
	CON_ENUM = 0xD6, //控制错误
	SAFETY_ENUM = 0xD7

}SERVO_ALA_TYPE_ENUM;

typedef struct
{
	int fpga_status;
	int pos_ctlerr;
	int speed_ctlerr;
	int current_ctlerr;

}SERVO_ALG_VAR_CHECK_DATA_STRUCT;

typedef enum
{
	dcbus_over = 0x01,//dc过压
	dcbus_owe = 0x02,//dc欠压
	ipm_over_curr = 0x03,//IPM过流
	motor_over_current = 0x04,//电机过载
	motor_over_temp = 0x05,//IPM过温
	motor_over_speed = 0x06,//电机过速
	motor_owe_speed = 0x07,//电机欠速
	motor_block_err = 0x08,//电机堵转
	motor_align_load = 0x09,//磁极对准过载
	motor_align_phase = 0x0A,//磁极对准相许错误
	encode_over_speed = 0x0B,//编码器过速
	encode_full_abs = 0x0C,//编码器绝对值错误
	encode_count_err = 0x0D,//编码器计数错误
	encode_overflow = 0x0E,//编码器溢出错误
	encode_unexpect = 0x0F,//编码器未知错误
	encode_mouti_err = 0x10,//编码器多圈错误
	encode_baterry_err = 0x11,//编码器电池错误
	encode_baterry_alam = 0x12,//编码器电池报警
	encode_lost_connect = 0x13,//编码器丢失连接
	encode_protocl_err = 0x14,//编码器通信错误
	pos_over_limit = 0x15,//电机位置超差
	vel_over_limit = 0x16, //电机速度超差
	motor_peak_current = 0x17, //超过峰值电流3倍，瞬间报错
	safety_alarm = 0x18,	//安全板急停信号
	safety_dec_vel = 0x19,	//安全板减速信号
	safety_protol = 0x1a, //安全板通信错误
	pos_give_over_limit = 0x1b, //给定位置超限
	vel_give_over_limit = 0x1c,//给定速度超限
	acc_give_over_limit = 0x1d,//给定加速度超限
	gra_give_over_limit = 0x1e, //给定重力超限
	backup1 = 0x1f, //备用1
	backup2 = 0x20,	//备用2
	reset_vol_timeout = 0x21, //reset 母线电压检测超时
	reset_con_timeout = 0x22, //reset接触器检测超时
	reset_bra_timeout = 0x23, //reset 抱闸检测超时
	pha_curr_err = 0x24

}SERVO_AXIS_ERR_ENUM;
//过载状态机
typedef enum
{
	LOAD_NORMAL,
	LOAD_ADD,
	LOAD_RETREAD,

}SERVO_OVER_LOAD_ENUM;

typedef struct
{
	uint32_t mask_encode;
	uint32_t shift_encode;

}SERVO_ENCODE_MASK_SHIFT_STRUCT;

typedef struct
{
	uint32_t current_data;
	uint32_t pre_data;

}SERVO_GEN_ERR_STRUCT;

typedef struct
{
	uint32_t current_data[ENCODE_FRAME_LENGTH];
	uint32_t pre_data[ENCODE_FRAME_LENGTH];

}SERVO_ENCODE_ERR_STRUCT;

typedef struct
{
	int64_t value;
}SERVO_CURRENT_OVERLOAD_LIMIT_STRUCT;


typedef struct
{
	int64_t cur_pos;
	int64_t pre_pos;
	int64_t pos_rate;

}SERVO_SET_POS_VAR_STRUCT;

typedef struct
{
	int ploar_num;
	float ploar_res_num;//极对数的倒数，浮点运算

}SERVO_POLAR_ALIGN_STRUCT;

//编码器错误枚举
typedef enum
{
	OVER_SPEED,
	FULL_ABS_STATUS,
	COUNT_ERR,
	COUNTER_OVERFLOAW,
	RESERVED,
	MULTI_ERR,
	BATERRY_ERR,
	BATERRY_ALM,
	PROTOCAL_ABNOMAL,
	PROTOCAL_DISCONNECT,
	ENCODE_MAX = PROTOCAL_DISCONNECT + 1

}SERVO_ENCODE_ERR_REGISTER_ENUM;

typedef struct
{
	int reg_mask;
	int reg_shift;

}SERVO_FPGA_EXUTE_STRUCT;

void Servo_CheckErr_Init(void);
int Get_DCBUS_Vol(void);
int Servo_DCBUS_Vol_Check(void);
void Servo_Ipm_OverCur_Check(void);
void Servo_IQ_Overload_Check(SERVO_AXIS_ENUM axis_id);
void Servo_IQ_PeakCur_Check(SERVO_AXIS_ENUM axis_id, int iq_data);
void Servo_Temp_Check(void);
void Servo_OverSpeed_Check(int set_speed, int mea_speed, SERVO_AXIS_ENUM axis_id);
void Servo_OweSpeed_Check(SERVO_AXIS_ENUM axis_id, int64_t vel_err);
void Servo_Block_Motion_Check(SERVO_AXIS_ENUM axis_id, int64_t back_vel, int64_t vel_err);
void Servo_PolarAlign_Angle_Cal(SERVO_AXIS_ENUM axis_id, int polar_num);
int16_t CAL_POLAR(float polar);
void Servo_Align_Check(SERVO_AXIS_ENUM ac_id, int *get_data, int *align_result);
void Servo_Encode_Err_Check(SERVO_AXIS_ENUM axis_id);
void Servo_SafetyBoaed_Check(void);
void Servo_PosErr_Check(SERVO_AXIS_ENUM axis_id, int64_t pos_err);
void Servo_PhaseErr_Check(SERVO_AXIS_ENUM axis_id);
void test_err(void);
}
#endif /* SERVO_CHECKERR_H_ */

