/*
 * Servo_Inter_PosTraj.c
 *
 *  Created on: 2019年7月26日
 *      Author: qichao.wang
 */
#include "Servo_Inter_PosTraj.h"
#include  <math.h>
#include <assert.h>
#include <stdio.h>

#define TIME_FACTOR 0.0002
#define VEL_FACTOR 335544//335544*0.0002/2
#define DIVED_TWO 0.5
#define DIVED_THREE 0.33333333
#define POS_CONV_FACTOR 16777216


using namespace virtual_servo_device;

static SERVO_POS_TRAJ_PARA_INFO para_info[AXIS_MAX];
static SERVO_POS_TRAJ_UPDATE_INFO update_info[AXIS_MAX];
static SERVO_POS_TRAJ_CONV_UPDATE_INFO update_cmd_info[AXIS_MAX];
static SERVO_POS_TRAJ_CONV_TIME_INFO time_conv_info[AXIS_MAX];
static SERVO_POS_TRAJ_SOU_TIME_INFO time_sou_info[AXIS_MAX];
static SERVO_POS_TARJ_VAR_TIME_INFO time_var_info[AXIS_MAX];

static uint32_t acc_line_style[AXIS_MAX] = {TRIANGLE_NO_VEL};
static uint32_t traj_state[AXIS_MAX] = {TRAJ_IDLE};
static SERVO_POS_TRAJ_DOUBLE_PARA_INFO local_para_info[AXIS_MAX];

static int32_t Servo_PosTraj_CalTime(SERVO_AXIS_ENUM axis_id);
/*-----------------------------------------------------------------------------
 *function_name:void Servo_PosTraj_Init(void)
 *input:
 *output:
 *description:
 * --------------------------------------------------------------------------*/
 void virtual_servo_device::Servo_PosTraj_Init(void)
{
	//Servo_PosTraj_SetPara(AXIS_0,test_para);
}
#define COMPARE_LIMIT 0.00000001
/*-----------------------------------------------------------------------------
 *function_name:int32_t Servo_PosTraj_CalTime(SERVO_AXIS_ENUM axis_id)
 *input:
 *output:
 *description:
 * --------------------------------------------------------------------------*/
 int32_t Servo_PosTraj_CalTime(SERVO_AXIS_ENUM axis_id)
{
	int32_t ret_val = 0;
	double temp_target_pos = local_para_info[axis_id].target_pos;//
	double temp_constrain_vel = local_para_info[axis_id].constrain_vel;
	double temp_constrain_acc = local_para_info[axis_id].constrain_acc;
	double temp_constrain_jerk = local_para_info[axis_id].constrain_jerk;

	double temp_data1 = 0.;
	double temp_data2 = 0.;


	time_var_info[axis_id].t_jerk_max = pow((0.5*temp_target_pos/temp_constrain_jerk),1.0/3.0);//minimal move distance
	time_var_info[axis_id].t_vel_max = sqrt(temp_constrain_vel/temp_constrain_jerk);//exist minimal constant velocity
	time_var_info[axis_id].t_acc_max = temp_constrain_acc/temp_constrain_jerk;//arrive max accelerate time

	if((time_var_info[axis_id].t_acc_max-time_var_info[axis_id].t_jerk_max)>COMPARE_LIMIT)//acc时间需要大于最小三角形正负不相连的距离,所以没有达到最大加速度值
	{/*加速度三角形曲线*/

		if((time_var_info[axis_id].t_vel_max-time_var_info[axis_id].t_jerk_max)>COMPARE_LIMIT)
		{
			time_sou_info[axis_id].t_jerk = time_var_info[axis_id].t_jerk_max;
			time_sou_info[axis_id].t_acc = 0;
			time_sou_info[axis_id].t_vel = 0;

			time_conv_info[axis_id].t_jerk = (uint32_t)(time_sou_info[axis_id].t_jerk*5000);
			time_conv_info[axis_id].t_acc = 0;
			time_conv_info[axis_id].t_vel = 0;

			acc_line_style[axis_id] = TRIANGLE_NO_VEL;
			traj_state[axis_id] = TRAJ_JERK_S1;
			update_info[axis_id].motion_flag = 1;

		}else{

			time_sou_info[axis_id].t_jerk = time_var_info[axis_id].t_vel_max;//临界到速度时间
			time_sou_info[axis_id].t_vel = temp_target_pos/temp_constrain_vel-2*time_sou_info[axis_id].t_jerk;
			time_sou_info[axis_id].t_acc = 0;

			time_conv_info[axis_id].t_jerk = (uint32_t)(time_sou_info[axis_id].t_jerk*5000);
			time_conv_info[axis_id].t_vel = (uint32_t)(time_sou_info[axis_id].t_vel*5000);
			time_conv_info[axis_id].t_acc = 0;

			acc_line_style[axis_id] = TRIANGLE_VEL;
			traj_state[axis_id] = TRAJ_JERK_S1;
			update_info[axis_id].motion_flag = 1;

		}

	}else{/*加速度梯形曲线*/

		temp_data1 = -1.5*time_var_info[axis_id].t_acc_max +sqrt((0.25*time_var_info[axis_id].t_acc_max*time_var_info[axis_id].t_acc_max)
													+(temp_target_pos/temp_constrain_acc));

		temp_data2 = (temp_constrain_vel/temp_constrain_acc)-time_var_info[axis_id].t_acc_max;

		if((temp_data1>COMPARE_LIMIT)&&(temp_data2>COMPARE_LIMIT))
		{

			if((temp_data1-temp_data2)>COMPARE_LIMIT)//有匀速段
			{

				time_sou_info[axis_id].t_jerk = time_var_info[axis_id].t_acc_max;
//				time_sou_info[axis_id].t_acc = (temp_constrain_vel-temp_constrain_jerk*time_sou_info[axis_id].t_jerk*time_sou_info[axis_id].t_jerk)/temp_constrain_acc;
//				time_sou_info[axis_id].t_vel = (temp_target_pos-2*(0.5*temp_constrain_jerk*time_sou_info[axis_id].t_jerk*time_sou_info[axis_id].t_jerk*time_sou_info[axis_id].t_acc
//						+0.5*temp_constrain_acc*time_sou_info[axis_id].t_acc*time_sou_info[axis_id].t_acc
//						+0.5*temp_constrain_jerk*time_sou_info[axis_id].t_jerk*time_sou_info[axis_id].t_jerk*time_sou_info[axis_id].t_jerk
//						+temp_constrain_acc*time_sou_info[axis_id].t_jerk*time_sou_info[axis_id].t_acc
//						+0.5*temp_constrain_acc*time_sou_info[axis_id].t_jerk*time_sou_info[axis_id].t_jerk))/temp_constrain_vel;

				time_sou_info[axis_id].t_acc = temp_data2;
				time_sou_info[axis_id].t_vel = (temp_target_pos/temp_constrain_vel)-time_var_info[axis_id].t_acc_max
												-(temp_constrain_vel/temp_constrain_acc);


				time_conv_info[axis_id].t_jerk = (uint32_t)(time_sou_info[axis_id].t_jerk*5000);
				time_conv_info[axis_id].t_acc = (uint32_t)(time_sou_info[axis_id].t_acc*5000);
				time_conv_info[axis_id].t_vel = (uint32_t)(time_sou_info[axis_id].t_vel*5000);

				acc_line_style[axis_id] = TRAPOZID_VEL;
				traj_state[axis_id] = TRAJ_JERK_S1;
				update_info[axis_id].motion_flag = 1;

			}else{

				time_sou_info[axis_id].t_jerk = time_var_info[axis_id].t_acc_max;
				time_sou_info[axis_id].t_acc = temp_data1;
				time_sou_info[axis_id].t_vel = 0;

				time_conv_info[axis_id].t_jerk = (uint32_t)(time_sou_info[axis_id].t_jerk*5000);
				time_conv_info[axis_id].t_acc = (uint32_t)(time_sou_info[axis_id].t_acc*5000);
				time_conv_info[axis_id].t_vel = 0;

				acc_line_style[axis_id] = TRAPOZID_NO_VEL;
				traj_state[axis_id] = TRAJ_JERK_S1;
				update_info[axis_id].motion_flag = 1;
			}
		}else{
			printf("parameter valid\r\n");
			return -1;
		}
	}
	printf("tj=%f\r\n",time_sou_info[axis_id].t_jerk);
	printf("ta=%f\r\n",time_sou_info[axis_id].t_acc);
	printf("tv=%f\r\n",time_sou_info[axis_id].t_vel);

	printf("tj=%u\r\n",time_conv_info[axis_id].t_jerk);
	printf("ta=%u\r\n",time_conv_info[axis_id].t_acc);
	printf("tv=%u\r\n",time_conv_info[axis_id].t_vel);

	return ret_val;
}
/*-----------------------------------------------------------------------------
 *function_name:void Servo_PosTraj_Init(void)
 *input:
 *output:
 *description:
 * --------------------------------------------------------------------------*/
 void virtual_servo_device::Servo_PosTraj_SetPara(SERVO_AXIS_ENUM axis_id,
							const int32_t* set_para)
{
	assert(set_para!=NULL);

	para_info[axis_id].target_pos = *(set_para+1);//target
	para_info[axis_id].constrain_vel = *(set_para+3);//vel
	para_info[axis_id].constrain_acc = *(set_para+4);//acc
	para_info[axis_id].constrain_dec = *(set_para+5);//dec
	para_info[axis_id].constrain_jerk = *(set_para+6);//jerk

	local_para_info[axis_id].target_pos = (double)(para_info[axis_id].target_pos/65536.0);
	local_para_info[axis_id].constrain_acc = (double)(para_info[axis_id].constrain_acc);
	local_para_info[axis_id].constrain_dec = (double)(para_info[axis_id].constrain_dec);
	local_para_info[axis_id].constrain_jerk = (double)(para_info[axis_id].constrain_jerk);
	local_para_info[axis_id].constrain_vel = (double)(para_info[axis_id].constrain_vel);

	if(local_para_info[axis_id].constrain_jerk<10000)
	{
		local_para_info[axis_id].constrain_jerk = 10000;//avoid data too small
	}

	Servo_PosTraj_CalTime(axis_id);
}
/*-----------------------------------------------------------------------------
 *function_name:void Servo_PosTraj_Init(void)
 *input:
 *output:
 *description:
 * --------------------------------------------------------------------------*/
static void Servo_PosTraj_TriangleNoVel(SERVO_AXIS_ENUM axis_id)
{
	static int32_t triangle_vel[AXIS_MAX] = {0};
	static double temp_data_acc[AXIS_MAX] = {0};
	static double temp_data_vel[AXIS_MAX] = {0};
	static double temp_data_pos[AXIS_MAX] = {0};

	double temp_data = 0;

	triangle_vel[axis_id]++;

	switch(traj_state[axis_id])
	{
		case(TRAJ_JERK_S1):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_jerk)
			{
				temp_data = (double)(local_para_info[axis_id].constrain_jerk
						 *triangle_vel[axis_id]
						 *TIME_FACTOR);
				update_info[axis_id].acc_update = temp_data;

				update_info[axis_id].vel_update = (double)(update_info[axis_id].acc_update*
						triangle_vel[axis_id]*TIME_FACTOR*DIVED_TWO);//1/2jt^2

				update_info[axis_id].pos_update = (double)(update_info[axis_id].vel_update*triangle_vel[axis_id]
				                                           *TIME_FACTOR*DIVED_THREE);//1/6JT^3
			}else{
				temp_data_acc[axis_id] = update_info[axis_id].acc_update;
				temp_data_vel[axis_id] = update_info[axis_id].vel_update;
				temp_data_pos[axis_id] = update_info[axis_id].pos_update;
				triangle_vel[axis_id] = 0;
				traj_state[axis_id] = TRAJ_JERK_S3;
			}
			break;
		}
		case(TRAJ_JERK_S3):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_jerk)
			{
				temp_data = (double)(local_para_info[axis_id].constrain_jerk
						 *triangle_vel[axis_id]
						 *TIME_FACTOR);

				update_info[axis_id].acc_update = -temp_data;

				update_info[axis_id].vel_update = (double)((temp_data_acc[axis_id] + update_info[axis_id].acc_update*
						DIVED_TWO)*triangle_vel[axis_id]*TIME_FACTOR+temp_data_vel[axis_id]);//v+(a-jt)积分

				update_info[axis_id].pos_update = (double)((temp_data_vel[axis_id]+temp_data_acc[axis_id]*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR
						+update_info[axis_id].acc_update*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR*DIVED_THREE)
						*triangle_vel[axis_id]*TIME_FACTOR+temp_data_pos[axis_id]);//v+at+1/6*j*t^2

			}else{
				temp_data_acc[axis_id] = 0.;
				temp_data_vel[axis_id] = update_info[axis_id].vel_update;
				temp_data_pos[axis_id] = update_info[axis_id].pos_update;
				triangle_vel[axis_id] = 0;
				traj_state[axis_id] = TRAJ_JERK_S5;
			}
			break;
		}
		case(TRAJ_JERK_S5):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_jerk)
			{
				temp_data = (double)(local_para_info[axis_id].constrain_jerk
						 *triangle_vel[axis_id]
						 *TIME_FACTOR);

				update_info[axis_id].acc_update = -temp_data;

				update_info[axis_id].vel_update = (double)((update_info[axis_id].acc_update*
						DIVED_TWO)*triangle_vel[axis_id]*TIME_FACTOR+temp_data_vel[axis_id]);//v+(a-jt)积分
				update_info[axis_id].pos_update = (double)((temp_data_vel[axis_id]+
						update_info[axis_id].acc_update*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR*DIVED_THREE)
						*triangle_vel[axis_id]*TIME_FACTOR+temp_data_pos[axis_id]);//1/6JT^3

			}else{
				temp_data_acc[axis_id] = update_info[axis_id].acc_update;
				temp_data_vel[axis_id] = update_info[axis_id].vel_update;
				temp_data_pos[axis_id] = update_info[axis_id].pos_update;
				triangle_vel[axis_id] = 0;
				traj_state[axis_id] = TRAJ_JERK_S7;
			}
			break;
		}
		case(TRAJ_JERK_S7):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_jerk)
			{
				temp_data = (double)(local_para_info[axis_id].constrain_jerk
									 *triangle_vel[axis_id]
									 *TIME_FACTOR);

				update_info[axis_id].acc_update = temp_data;

				update_info[axis_id].vel_update = (double)((temp_data_acc[axis_id]+update_info[axis_id].acc_update*
									DIVED_TWO)*triangle_vel[axis_id]*TIME_FACTOR+temp_data_vel[axis_id]);//v+(a-jt)积分

				update_info[axis_id].pos_update = (double)((temp_data_vel[axis_id] + temp_data_acc[axis_id]*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR
									+update_info[axis_id].acc_update*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR*DIVED_THREE)
									*triangle_vel[axis_id]*TIME_FACTOR+temp_data_pos[axis_id]);//1/6JT^3

			}else{
				update_info[axis_id].pos_update = local_para_info[axis_id].target_pos;

				temp_data_acc[axis_id] = 0;
				temp_data_vel[axis_id] = 0;
				triangle_vel[axis_id] = 0;
				update_info[axis_id].motion_flag = 0;
			}
			break;
		}
	}

	update_cmd_info[axis_id].pos_update = (int64_t)(update_info[axis_id].pos_update*POS_CONV_FACTOR);
	update_cmd_info[axis_id].vel_update = (int64_t)(update_info[axis_id].vel_update*VEL_FACTOR);

	printf("pos_update:%lld\r\n",update_cmd_info[axis_id].pos_update);
	printf("vel_update:%lld\r\n",update_cmd_info[axis_id].vel_update);
}
/*-----------------------------------------------------------------------------
 *function_name:void Servo_PosTraj_Init(void)
 *input:
 *output:
 *description:
 * --------------------------------------------------------------------------*/
static void Servo_PosTraj_TriangleVel(SERVO_AXIS_ENUM axis_id)
{
	static int32_t triangle_vel[AXIS_MAX] = {0};
	static double temp_data_acc[AXIS_MAX] = {0};
	static double temp_data_vel[AXIS_MAX] = {0};
	static double temp_data_pos[AXIS_MAX] = {0};

	double temp_data = 0;

	triangle_vel[axis_id]++;

	switch(traj_state[axis_id])
	{
		case(TRAJ_JERK_S1):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_jerk)
			{
				temp_data = (double)(local_para_info[axis_id].constrain_jerk
						 *triangle_vel[axis_id]
						 *TIME_FACTOR);
				update_info[axis_id].acc_update = temp_data;

				update_info[axis_id].vel_update = (double)(update_info[axis_id].acc_update*
						triangle_vel[axis_id]*TIME_FACTOR*DIVED_TWO);//1/2jt^2

				update_info[axis_id].pos_update = (double)(update_info[axis_id].vel_update*triangle_vel[axis_id]
				                                           *TIME_FACTOR*DIVED_THREE);//1/6JT^3
			}else{
				temp_data_acc[axis_id] = update_info[axis_id].acc_update;
				temp_data_vel[axis_id] = update_info[axis_id].vel_update;
				temp_data_pos[axis_id] = update_info[axis_id].pos_update;
				triangle_vel[axis_id] = 0;
				traj_state[axis_id] = TRAJ_JERK_S3;
			}
			break;
		}
		case(TRAJ_JERK_S3):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_jerk)
			{
				temp_data = (double)(local_para_info[axis_id].constrain_jerk
						 *triangle_vel[axis_id]
						 *TIME_FACTOR);

				update_info[axis_id].acc_update = -temp_data;

				update_info[axis_id].vel_update = (double)((temp_data_acc[axis_id] + update_info[axis_id].acc_update*
						DIVED_TWO)*triangle_vel[axis_id]*TIME_FACTOR+temp_data_vel[axis_id]);//v+(a-jt)积分

				update_info[axis_id].pos_update = (double)((temp_data_vel[axis_id]+temp_data_acc[axis_id]*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR
						+update_info[axis_id].acc_update*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR*DIVED_THREE)
						*triangle_vel[axis_id]*TIME_FACTOR+temp_data_pos[axis_id]);//v+at+1/6*j*t^2

			}else{
				temp_data_acc[axis_id] = 0.;
				temp_data_vel[axis_id] = update_info[axis_id].vel_update;
				temp_data_pos[axis_id] = update_info[axis_id].pos_update;
				triangle_vel[axis_id] = 0;
				traj_state[axis_id] = TRAJ_CONS_VEL_S4;
			}
			break;
		}
		case(TRAJ_CONS_VEL_S4):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_vel)
			{

				update_info[axis_id].pos_update =
						(double)(temp_data_vel[axis_id]*triangle_vel[axis_id]*TIME_FACTOR+temp_data_pos[axis_id]);//1/6JT^3
			}else{
				triangle_vel[axis_id] = 0;
				traj_state[axis_id] = TRAJ_JERK_S5;
				temp_data_pos[axis_id] = update_info[axis_id].pos_update;
			}
			break;
		}
		case(TRAJ_JERK_S5):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_jerk)
			{
				temp_data = (double)(local_para_info[axis_id].constrain_jerk
						 *triangle_vel[axis_id]
						 *TIME_FACTOR);

				update_info[axis_id].acc_update = -temp_data;

				update_info[axis_id].vel_update = (double)((update_info[axis_id].acc_update*
						DIVED_TWO)*triangle_vel[axis_id]*TIME_FACTOR+temp_data_vel[axis_id]);//v+(a-jt)积分
				update_info[axis_id].pos_update = (double)((temp_data_vel[axis_id]+
						update_info[axis_id].acc_update*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR*DIVED_THREE)
						*triangle_vel[axis_id]*TIME_FACTOR+temp_data_pos[axis_id]);//1/6JT^3

			}else{
				temp_data_acc[axis_id] = update_info[axis_id].acc_update;
				temp_data_vel[axis_id] = update_info[axis_id].vel_update;
				temp_data_pos[axis_id] = update_info[axis_id].pos_update;
				triangle_vel[axis_id] = 0;
				traj_state[axis_id] = TRAJ_JERK_S7;
			}
			break;
		}
		case(TRAJ_JERK_S7):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_jerk)
			{
				temp_data = (double)(local_para_info[axis_id].constrain_jerk
									 *triangle_vel[axis_id]
									 *TIME_FACTOR);

				update_info[axis_id].acc_update = temp_data;

				update_info[axis_id].vel_update = (double)((temp_data_acc[axis_id]+update_info[axis_id].acc_update*
									DIVED_TWO)*triangle_vel[axis_id]*TIME_FACTOR+temp_data_vel[axis_id]);//v+(a-jt)积分

				update_info[axis_id].pos_update = (double)((temp_data_vel[axis_id] + temp_data_acc[axis_id]*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR
									+update_info[axis_id].acc_update*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR*DIVED_THREE)
									*triangle_vel[axis_id]*TIME_FACTOR+temp_data_pos[axis_id]);//1/6JT^3

			}else{
				update_info[axis_id].pos_update = local_para_info[axis_id].target_pos;

				temp_data_acc[axis_id] = 0;
				temp_data_vel[axis_id] = 0;
				triangle_vel[axis_id] = 0;
				update_info[axis_id].motion_flag = 0;
			}
			break;
		}
	}
	update_cmd_info[axis_id].pos_update = (int64_t)(update_info[axis_id].pos_update*POS_CONV_FACTOR);
	update_cmd_info[axis_id].vel_update = (int64_t)(update_info[axis_id].vel_update*VEL_FACTOR);

	printf("pos_update:%lld\r\n",update_cmd_info[axis_id].pos_update);
	printf("vel_update:%lld\r\n",update_cmd_info[axis_id].vel_update);
}
/*-----------------------------------------------------------------------------
 *function_name:void Servo_PosTraj_Init(void)
 *input:
 *output:
 *description:
 * --------------------------------------------------------------------------*/
static void Servo_PosTraj_TrapozidNoVel(SERVO_AXIS_ENUM axis_id)
{
	static int32_t triangle_vel[AXIS_MAX] = {0};
	static double temp_data_acc[AXIS_MAX] = {0};
	static double temp_data_vel[AXIS_MAX] = {0};
	static double temp_data_pos[AXIS_MAX] = {0};

	double temp_data = 0;

	triangle_vel[axis_id]++;

	switch(traj_state[axis_id])
	{
		case(TRAJ_JERK_S1):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_jerk)
			{
				temp_data = (double)(local_para_info[axis_id].constrain_jerk
						 *triangle_vel[axis_id]
						 *TIME_FACTOR);
				update_info[axis_id].acc_update = temp_data;

				update_info[axis_id].vel_update = (double)(update_info[axis_id].acc_update*
						triangle_vel[axis_id]*TIME_FACTOR*DIVED_TWO);//1/2jt^2

				update_info[axis_id].pos_update = (double)(update_info[axis_id].vel_update*triangle_vel[axis_id]
				                                           *TIME_FACTOR*DIVED_THREE);//1/6JT^3
			}else{
				temp_data_acc[axis_id] = update_info[axis_id].acc_update;
				temp_data_vel[axis_id] = update_info[axis_id].vel_update;
				temp_data_pos[axis_id] = update_info[axis_id].pos_update;
				triangle_vel[axis_id] = 0;
				traj_state[axis_id] = TRAJ_ACC_S2;
			}
			break;
		}
		case(TRAJ_ACC_S2):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_acc)
			{
				update_info[axis_id].vel_update = (double)(temp_data_acc[axis_id]*
									triangle_vel[axis_id]*TIME_FACTOR+temp_data_vel[axis_id]);//a*t

				update_info[axis_id].pos_update = (double)((temp_data_vel[axis_id]+DIVED_TWO*update_info[axis_id].acc_update*triangle_vel[axis_id]*TIME_FACTOR)
														*triangle_vel[axis_id]*TIME_FACTOR+temp_data_pos[axis_id]);//v*t+1/2*a*t^2,result

			}else{
				temp_data_vel[axis_id] = update_info[axis_id].vel_update;
				temp_data_pos[axis_id] = update_info[axis_id].pos_update;
				triangle_vel[axis_id] = 0;
				traj_state[axis_id] = TRAJ_JERK_S3;
			}
			break;
		}
		case(TRAJ_JERK_S3):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_jerk)
			{
				temp_data = (double)(local_para_info[axis_id].constrain_jerk
						 *triangle_vel[axis_id]
						 *TIME_FACTOR);

				update_info[axis_id].acc_update = -temp_data;

				update_info[axis_id].vel_update = (double)((temp_data_acc[axis_id] + update_info[axis_id].acc_update*
						DIVED_TWO)*triangle_vel[axis_id]*TIME_FACTOR+temp_data_vel[axis_id]);//v+(a-jt)积分

				update_info[axis_id].pos_update = (double)((temp_data_vel[axis_id]+temp_data_acc[axis_id]*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR
						+update_info[axis_id].acc_update*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR*DIVED_THREE)
						*triangle_vel[axis_id]*TIME_FACTOR+temp_data_pos[axis_id]);//v+at+1/6*j*t^2

			}else{
				temp_data_acc[axis_id] = 0.;
				temp_data_vel[axis_id] = update_info[axis_id].vel_update;
				temp_data_pos[axis_id] = update_info[axis_id].pos_update;
				triangle_vel[axis_id] = 0;
				traj_state[axis_id] = TRAJ_JERK_S5;
			}
			break;
		}
		case(TRAJ_JERK_S5):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_jerk)
			{
				temp_data = (double)(local_para_info[axis_id].constrain_jerk
						 *triangle_vel[axis_id]
						 *TIME_FACTOR);

				update_info[axis_id].acc_update = -temp_data;

				update_info[axis_id].vel_update = (double)((update_info[axis_id].acc_update*
						DIVED_TWO)*triangle_vel[axis_id]*TIME_FACTOR+temp_data_vel[axis_id]);//v+(a-jt)积分
				update_info[axis_id].pos_update = (double)((temp_data_vel[axis_id]+
						update_info[axis_id].acc_update*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR*DIVED_THREE)
						*triangle_vel[axis_id]*TIME_FACTOR+temp_data_pos[axis_id]);//1/6JT^3

			}else{
				temp_data_acc[axis_id] = update_info[axis_id].acc_update;
				temp_data_vel[axis_id] = update_info[axis_id].vel_update;
				temp_data_pos[axis_id] = update_info[axis_id].pos_update;
				triangle_vel[axis_id] = 0;
				traj_state[axis_id] = TRAJ_ACC_S6;
			}
			break;
		}
		case(TRAJ_ACC_S6):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_acc)
			{
				update_info[axis_id].vel_update = (double)(temp_data_acc[axis_id]*triangle_vel[axis_id]*TIME_FACTOR+temp_data_vel[axis_id]);//v+(a-jt)积分
				update_info[axis_id].pos_update = (double)((temp_data_vel[axis_id] +
									update_info[axis_id].acc_update*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR)
									*triangle_vel[axis_id]*TIME_FACTOR+temp_data_pos[axis_id]);//v*t+1/2*a*t^2
			}else{
				temp_data_vel[axis_id] = update_info[axis_id].vel_update;
				temp_data_pos[axis_id] = update_info[axis_id].pos_update;
				triangle_vel[axis_id] = 0;
				traj_state[axis_id] = TRAJ_JERK_S7;
			}
			break;
		}
		case(TRAJ_JERK_S7):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_jerk)
			{
				temp_data = (double)(local_para_info[axis_id].constrain_jerk
									 *triangle_vel[axis_id]
									 *TIME_FACTOR);

				update_info[axis_id].acc_update = temp_data;

				update_info[axis_id].vel_update = (double)((temp_data_acc[axis_id]+update_info[axis_id].acc_update*
									DIVED_TWO)*triangle_vel[axis_id]*TIME_FACTOR+temp_data_vel[axis_id]);//v+(a-jt)积分

				update_info[axis_id].pos_update = (double)((temp_data_vel[axis_id] + temp_data_acc[axis_id]*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR
									+update_info[axis_id].acc_update*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR*DIVED_THREE)
									*triangle_vel[axis_id]*TIME_FACTOR+temp_data_pos[axis_id]);//1/6JT^3

			}else{
				update_info[axis_id].pos_update = local_para_info[axis_id].target_pos;

				temp_data_acc[axis_id] = 0;
				temp_data_vel[axis_id] = 0;
				triangle_vel[axis_id] = 0;
				update_info[axis_id].motion_flag = 0;
			}
			break;
		}
	}
	update_cmd_info[axis_id].pos_update = (int64_t)(update_info[axis_id].pos_update*POS_CONV_FACTOR);
	update_cmd_info[axis_id].vel_update = (int64_t)(update_info[axis_id].vel_update*VEL_FACTOR);

	printf("pos_update:%lld\r\n",update_cmd_info[axis_id].pos_update);
	printf("vel_update:%lld\r\n",update_cmd_info[axis_id].vel_update);
}
/*-----------------------------------------------------------------------------
 *function_name:void Servo_PosTraj_Init(void)
 *input:
 *output:
 *description:
 * --------------------------------------------------------------------------*/
static void Servo_PosTraj_TrapozidVel(SERVO_AXIS_ENUM axis_id)
{
	static int32_t triangle_vel[AXIS_MAX] = {0};
	static double temp_data_acc[AXIS_MAX] = {0};
	static double temp_data_vel[AXIS_MAX] = {0};
	static double temp_data_pos[AXIS_MAX] = {0};

	double temp_data = 0;

	triangle_vel[axis_id]++;

	switch(traj_state[axis_id])
	{
		case(TRAJ_JERK_S1):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_jerk)
			{
				temp_data = (double)(local_para_info[axis_id].constrain_jerk
						 *triangle_vel[axis_id]
						 *TIME_FACTOR);
				update_info[axis_id].acc_update = temp_data;

				update_info[axis_id].vel_update = (double)(update_info[axis_id].acc_update*
						triangle_vel[axis_id]*TIME_FACTOR*DIVED_TWO);//1/2jt^2

				update_info[axis_id].pos_update = (double)(update_info[axis_id].vel_update*triangle_vel[axis_id]
				                                           *TIME_FACTOR*DIVED_THREE);//1/6JT^3
			}else{
				temp_data_acc[axis_id] = update_info[axis_id].acc_update;
				temp_data_vel[axis_id] = update_info[axis_id].vel_update;
				temp_data_pos[axis_id] = update_info[axis_id].pos_update;
				triangle_vel[axis_id] = 0;
				traj_state[axis_id] = TRAJ_ACC_S2;
				printf("pos_update:%f\r\n",update_info[axis_id].pos_update);
				printf("vel_update:%f\r\n",update_info[axis_id].vel_update);
			}
			break;
		}
		case(TRAJ_ACC_S2):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_acc)
			{
				update_info[axis_id].vel_update = (double)(temp_data_acc[axis_id]*
									triangle_vel[axis_id]*TIME_FACTOR+temp_data_vel[axis_id]);//a*t

				update_info[axis_id].pos_update = (double)((temp_data_vel[axis_id]+DIVED_TWO*update_info[axis_id].acc_update*triangle_vel[axis_id]*TIME_FACTOR)
														*triangle_vel[axis_id]*TIME_FACTOR+temp_data_pos[axis_id]);//v*t+1/2*a*t^2,result

			}else{
				temp_data_vel[axis_id] = update_info[axis_id].vel_update;
				temp_data_pos[axis_id] = update_info[axis_id].pos_update;
				triangle_vel[axis_id] = 0;
				traj_state[axis_id] = TRAJ_JERK_S3;
				printf("pos_update:%f\r\n",update_info[axis_id].pos_update);
				printf("vel_update:%f\r\n",update_info[axis_id].vel_update);
			}
			break;
		}
		case(TRAJ_JERK_S3):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_jerk)
			{
				temp_data = (double)(local_para_info[axis_id].constrain_jerk
						 *triangle_vel[axis_id]
						 *TIME_FACTOR);

				update_info[axis_id].acc_update = -temp_data;

				update_info[axis_id].vel_update = (double)((temp_data_acc[axis_id] + update_info[axis_id].acc_update*
						DIVED_TWO)*triangle_vel[axis_id]*TIME_FACTOR+temp_data_vel[axis_id]);//v+(a-jt)积分

				update_info[axis_id].pos_update = (double)((temp_data_vel[axis_id]+temp_data_acc[axis_id]*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR
						+update_info[axis_id].acc_update*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR*DIVED_THREE)
						*triangle_vel[axis_id]*TIME_FACTOR+temp_data_pos[axis_id]);//v+at+1/6*j*t^2

			}else{
				temp_data_acc[axis_id] = 0.;
				temp_data_vel[axis_id] = update_info[axis_id].vel_update;
				temp_data_pos[axis_id] = update_info[axis_id].pos_update;
				triangle_vel[axis_id] = 0;
				traj_state[axis_id] = TRAJ_CONS_VEL_S4;
				printf("pos_update:%f\r\n",update_info[axis_id].pos_update);
				printf("vel_update:%f\r\n",update_info[axis_id].vel_update);
			}
			break;
		}
		case(TRAJ_CONS_VEL_S4):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_vel)
			{

				update_info[axis_id].pos_update =
						(double)(temp_data_vel[axis_id]*triangle_vel[axis_id]*TIME_FACTOR+temp_data_pos[axis_id]);//1/6JT^3
			}else{
				triangle_vel[axis_id] = 0;
				traj_state[axis_id] = TRAJ_JERK_S5;
				temp_data_pos[axis_id] = update_info[axis_id].pos_update;
				printf("pos_update:%f\r\n",update_info[axis_id].pos_update);
				printf("vel_update:%f\r\n",update_info[axis_id].vel_update);
			}
			break;
		}
		case(TRAJ_JERK_S5):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_jerk)
			{
				temp_data = (double)(local_para_info[axis_id].constrain_jerk
						 *triangle_vel[axis_id]
						 *TIME_FACTOR);

				update_info[axis_id].acc_update = -temp_data;

				update_info[axis_id].vel_update = (double)((update_info[axis_id].acc_update*
						DIVED_TWO)*triangle_vel[axis_id]*TIME_FACTOR+temp_data_vel[axis_id]);//v+(a-jt)积分
				update_info[axis_id].pos_update = (double)((temp_data_vel[axis_id]+
						update_info[axis_id].acc_update*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR*DIVED_THREE)
						*triangle_vel[axis_id]*TIME_FACTOR+temp_data_pos[axis_id]);//1/6JT^3

			}else{
				temp_data_acc[axis_id] = update_info[axis_id].acc_update;
				temp_data_vel[axis_id] = update_info[axis_id].vel_update;
				temp_data_pos[axis_id] = update_info[axis_id].pos_update;
				triangle_vel[axis_id] = 0;
				traj_state[axis_id] = TRAJ_ACC_S6;
				printf("pos_update:%f\r\n",update_info[axis_id].pos_update);
				printf("vel_update:%f\r\n",update_info[axis_id].vel_update);
			}
			break;
		}
		case(TRAJ_ACC_S6):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_acc)
			{
				update_info[axis_id].vel_update = (double)(temp_data_acc[axis_id]*triangle_vel[axis_id]*TIME_FACTOR+temp_data_vel[axis_id]);//v+(a-jt)积分
				update_info[axis_id].pos_update = (double)((temp_data_vel[axis_id] +
									update_info[axis_id].acc_update*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR)
									*triangle_vel[axis_id]*TIME_FACTOR+temp_data_pos[axis_id]);//v*t+1/2*a*t^2
			}else{
				temp_data_vel[axis_id] = update_info[axis_id].vel_update;
				temp_data_pos[axis_id] = update_info[axis_id].pos_update;
				triangle_vel[axis_id] = 0;
				traj_state[axis_id] = TRAJ_JERK_S7;
				printf("pos_update:%f\r\n",update_info[axis_id].pos_update);
				printf("vel_update:%f\r\n",update_info[axis_id].vel_update);
			}
			break;
		}
		case(TRAJ_JERK_S7):
		{
			if(triangle_vel[axis_id]<=time_conv_info[axis_id].t_jerk)
			{
				temp_data = (double)(local_para_info[axis_id].constrain_jerk
									 *triangle_vel[axis_id]
									 *TIME_FACTOR);

				update_info[axis_id].acc_update = temp_data;

				update_info[axis_id].vel_update = (double)((temp_data_acc[axis_id]+update_info[axis_id].acc_update*
									DIVED_TWO)*triangle_vel[axis_id]*TIME_FACTOR+temp_data_vel[axis_id]);//v+(a-jt)积分

				update_info[axis_id].pos_update = (double)((temp_data_vel[axis_id] + temp_data_acc[axis_id]*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR
									+update_info[axis_id].acc_update*DIVED_TWO*triangle_vel[axis_id]*TIME_FACTOR*DIVED_THREE)
									*triangle_vel[axis_id]*TIME_FACTOR+temp_data_pos[axis_id]);//1/6JT^3

			}else{
				update_info[axis_id].pos_update = (local_para_info[axis_id].target_pos);

				temp_data_acc[axis_id] = 0;
				temp_data_vel[axis_id] = 0;
				triangle_vel[axis_id] = 0;
				update_info[axis_id].motion_flag = 0;
				printf("pos_update:%f\r\n",update_info[axis_id].pos_update);
				printf("vel_update:%f\r\n",update_info[axis_id].vel_update);
			}
			break;
		}
	}
	update_cmd_info[axis_id].pos_update = (int64_t)(update_info[axis_id].pos_update*POS_CONV_FACTOR);
	update_cmd_info[axis_id].vel_update = (int64_t)(update_info[axis_id].vel_update*VEL_FACTOR);
}
/*-----------------------------------------------------------------------------
 *function_name:void Servo_PosTraj_UpateTraj(SERVO_AXIS_ENUM axis_id)
 *input:
 *output:
 *description:
 * --------------------------------------------------------------------------*/
 void virtual_servo_device::Servo_PosTraj_UpateTraj(SERVO_AXIS_ENUM axis_id)
{
	if(update_info[axis_id].motion_flag)
	{
		switch(acc_line_style[axis_id])
		{
			case(TRIANGLE_NO_VEL):
			{
				Servo_PosTraj_TriangleNoVel(axis_id);
				break;
			}
			case(TRIANGLE_VEL):
			{
				Servo_PosTraj_TriangleVel(axis_id);
				break;
			}
			case(TRAPOZID_NO_VEL):
			{
				Servo_PosTraj_TrapozidNoVel(axis_id);
				break;
			}
			case(TRAPOZID_VEL):
			{
				Servo_PosTraj_TrapozidVel(axis_id);
				break;
			}
			default:break;
		}
	}
}
/*-----------------------------------------------------------------------------
 *function_name:void Servo_PosTraj_UpateTraj(SERVO_AXIS_ENUM axis_id)
 *input:
 *output:
 *description:
 * --------------------------------------------------------------------------*/
 void virtual_servo_device::Servo_PosTraj_Get_TrajUpdate_Ptr(SERVO_AXIS_ENUM axis_id,
		SERVO_POS_TRAJ_CONV_UPDATE_INFO** get_update_ptr)
{
		*get_update_ptr = &update_cmd_info[axis_id];
}
/*-----------------------------------------------------------------------------
 *function_name:void Servo_PosTraj_UpateTraj(SERVO_AXIS_ENUM axis_id)
 *input:
 *output:
 *description:
 * --------------------------------------------------------------------------*/
 void virtual_servo_device::Servo_PosTraj_ResetVar(SERVO_AXIS_ENUM axis_id)
{
	time_sou_info[axis_id].t_jerk = 0;//临界到速度时间
	time_sou_info[axis_id].t_vel = 0;
	time_sou_info[axis_id].t_acc = 0;

	time_conv_info[axis_id].t_jerk = 0;
	time_conv_info[axis_id].t_vel = 0;
	time_conv_info[axis_id].t_acc = 0;

	update_cmd_info[axis_id].vel_update = 0;
	update_cmd_info[axis_id].acc_update = 0;
	update_info[axis_id].motion_flag = 0;
}

