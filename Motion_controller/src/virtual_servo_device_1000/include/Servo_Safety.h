/*
 * Servo_Safety.h
 *
 *  Created on: 2019年7月15日
 *      Author: qichao.wang
 */

#ifndef SERVO_SAFETY_H_
#define SERVO_SAFETY_H_

#include "Servo_General.h"

#define SAFE_FRMAE_COMB(data1,data2,data3,data4)	((data4<<24)|(data3<<16)|(data2<<8)|data1)
namespace virtual_servo_device{

typedef struct
{
	 uint8_t dec_signal_flag;
	 uint8_t stop_signal_flag;

}MOSAFE_STOP_SIGNAL_STRUCT;

typedef enum
{
	CONTACTOR_SIGNAL,
	DEC_SIGNAL,
	IMDE_STOP_SIGNAL,
	BRAKER_STATUS,
	SAFETY_ALARM,
	SAFETY_PROTO

}MOSAFE_CHECK_SIGNAL_ENUM;

typedef enum
{
	RESET_SAFETY,
	ZERO_STOP,
	EXCITATION,
	CLEAR_RESET//告知安全板已经励磁

}MOSAFE_SAFE_CMD_ENUM;

typedef struct
{
	uint8_t frame_1;
	uint8_t frame_2;
	uint8_t resever1;
	uint8_t resever2;

}MOSAFE_FRAME_SET_INFO_STRUCT;

typedef struct
{
	uint8_t frame_1;
	uint8_t frame_2;
	uint8_t frame_3;
	uint8_t frame_4;

}MOSAFE_FREAM_RES_INFO_STRUCT;

typedef struct
{
	uint8_t err_status;
	uint8_t err_beat;
	uint8_t reserve1;
	uint8_t reserve2;

}MOSAFE_FRAME_PROTO_INFO_STRUCT;

void Servo_Safety_Init(void);

int Servo_Safety_s_Get_Contator(int check_signal);

void Servo_Safety_b_Write_Cmd_Ana(MOSAFE_SAFE_CMD_ENUM cmd_id);

uint32_t Servo_Safety_p_Process_SafetBoard_Status(void);

uint32_t Servo_Safety_p_Process_SafetBoard_ErrCode(void);

void Servo_Safety_p_RwSafety(void);
}

#endif /* MOSAFE_H_ */

