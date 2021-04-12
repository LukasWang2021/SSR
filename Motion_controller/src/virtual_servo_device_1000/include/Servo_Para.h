/*
 * Servo_Para.h
 *
 *  Created on: 2019年7月15日
 *      Author: qichao.wang
 */
#ifndef SERVO_PARA_H_
#define SERVO_PARA_H_

#include "Servo_General.h"
#include "common/core_comm_datatype.h"

namespace virtual_servo_device{

typedef struct
{
	SERVO_PARA_COMMO_INFO err_ctrl_word;
	SERVO_PARA_COMMO_INFO peak_current_window;
	SERVO_PARA_COMMO_INFO power_moudle_over_temp;
	SERVO_PARA_COMMO_INFO dc_over_temp;
	SERVO_PARA_COMMO_INFO motor_over_temp;
	SERVO_PARA_COMMO_INFO vol_bus_up_limit;
	SERVO_PARA_COMMO_INFO vol_bus_down_limit;
	SERVO_PARA_COMMO_INFO pos_follow_err_limit;
	SERVO_PARA_COMMO_INFO pos_follow_time_limit;
	SERVO_PARA_COMMO_INFO speed_follow_err_limit;
	SERVO_PARA_COMMO_INFO speed_follow_time_limit;
	SERVO_PARA_COMMO_INFO power_overload_current_1;
	SERVO_PARA_COMMO_INFO power_overload_time_1;
	SERVO_PARA_COMMO_INFO power_overload_current_2;
	SERVO_PARA_COMMO_INFO power_overload_time_2;
	SERVO_PARA_COMMO_INFO motor_overload_current_1;
	SERVO_PARA_COMMO_INFO motor_overload_time_1;
	SERVO_PARA_COMMO_INFO motor_overload_current_2;
	SERVO_PARA_COMMO_INFO motor_overload_time_2;
	SERVO_PARA_COMMO_INFO resetable_error_flag;
	SERVO_PARA_COMMO_INFO reboot_error_flag;
	SERVO_PARA_COMMO_INFO current_error_code;
	SERVO_PARA_COMMO_INFO history_error_code1;
	SERVO_PARA_COMMO_INFO history_error_code2;
	SERVO_PARA_COMMO_INFO history_error_code3;
	SERVO_PARA_COMMO_INFO history_error_code4;
	SERVO_PARA_COMMO_INFO history_error_code5;
	SERVO_PARA_COMMO_INFO history_error_code6;
	SERVO_PARA_COMMO_INFO history_error_code7;
	SERVO_PARA_COMMO_INFO history_error_code8;

	SERVO_PARA_COMMO_INFO filter[18];

}PARA_ERR_CHECK_READ_INFO;

typedef struct
{
	SERVO_PARA_COMMO_INFO kp_factor;
	SERVO_PARA_COMMO_INFO time_res_factor;//equal ki_factor;
	SERVO_PARA_COMMO_INFO inertia_factor;
	SERVO_PARA_COMMO_INFO vel_feed_factor;
	SERVO_PARA_COMMO_INFO vel_input_sature;
	SERVO_PARA_COMMO_INFO iq_output_sature;
	SERVO_PARA_COMMO_INFO arriving_speed_limit;
	SERVO_PARA_COMMO_INFO arriving_time_limit;
	SERVO_PARA_COMMO_INFO period_speed_loop;
	SERVO_PARA_COMMO_INFO zero_speed_limit;
	SERVO_PARA_COMMO_INFO zero_speed_time_limit;
	SERVO_PARA_COMMO_INFO filter[5];

}PARA_SPEED_LOOP_READ_INFO;


typedef struct
{
	SERVO_PARA_COMMO_INFO kp_factor;
	SERVO_PARA_COMMO_INFO period_pos_loop;
	SERVO_PARA_COMMO_INFO arriving_pos_limit;
	SERVO_PARA_COMMO_INFO arriving_time_limit;
	SERVO_PARA_COMMO_INFO filter[12];

}PARA_POSITION_LOOP_READ_INFO;

typedef struct
{
	SERVO_PARA_COMMO_INFO kp_factor;
	SERVO_PARA_COMMO_INFO ki_factor;
	SERVO_PARA_COMMO_INFO pwm_frequency_selection;
	SERVO_PARA_COMMO_INFO period_current_loop;
	SERVO_PARA_COMMO_INFO torque_factor;
	SERVO_PARA_COMMO_INFO filter[11];

}PARA_CURRENT_LOOP_READ_INFO;

typedef struct
{
	SERVO_PARA_COMMO_INFO brake_ctrl_word;
	SERVO_PARA_COMMO_INFO braker_release_time;
	SERVO_PARA_COMMO_INFO braker_action_time;
	SERVO_PARA_COMMO_INFO braker_holding_voltage;
	SERVO_PARA_COMMO_INFO braker_pulse_time;
	SERVO_PARA_COMMO_INFO filter[11];

}PARA_BRAKER_READ_INFO;

typedef struct
{
	SERVO_PARA_COMMO_INFO motor_model_id;
	SERVO_PARA_COMMO_INFO motor_direction;
	SERVO_PARA_COMMO_INFO motor_rated_current;
	SERVO_PARA_COMMO_INFO motor_max_current_in_percent;
	SERVO_PARA_COMMO_INFO motor_rated_torque;
	SERVO_PARA_COMMO_INFO motor_max_torque;
	SERVO_PARA_COMMO_INFO motor_rated_rpm;
	SERVO_PARA_COMMO_INFO motor_max_rpm;
	SERVO_PARA_COMMO_INFO motor_polo_angle;
	SERVO_PARA_COMMO_INFO motor_pole_pairs;
	SERVO_PARA_COMMO_INFO motor_kt;
	SERVO_PARA_COMMO_INFO motor_ke;
	SERVO_PARA_COMMO_INFO motor_ld;
	SERVO_PARA_COMMO_INFO motor_lq;
	SERVO_PARA_COMMO_INFO motor_winding_resistance;
	SERVO_PARA_COMMO_INFO motor_rotor_inertia;
	SERVO_PARA_COMMO_INFO thermal_time_constant;
	SERVO_PARA_COMMO_INFO sample_resistence;
	SERVO_PARA_COMMO_INFO filter[13];


}PARA_MOTOR_CFG_READ_INFO;

typedef struct
{
	SERVO_PARA_COMMO_INFO target_torque;
	SERVO_PARA_COMMO_INFO max_torque;
	SERVO_PARA_COMMO_INFO torque_slope;
	SERVO_PARA_COMMO_INFO torque_profile_type;
	SERVO_PARA_COMMO_INFO positive_torque_imit;
	SERVO_PARA_COMMO_INFO negative_torque_limit;
	SERVO_PARA_COMMO_INFO torque_control_gain;
	SERVO_PARA_COMMO_INFO torque_control_time;
	SERVO_PARA_COMMO_INFO filter[8];


}PARA_TORQUE_TRAJ_READ_INFO;

typedef struct
{
	SERVO_PARA_COMMO_INFO target_velocity;
	SERVO_PARA_COMMO_INFO max_profile_velocity;
	SERVO_PARA_COMMO_INFO max_acceleration;
	SERVO_PARA_COMMO_INFO max_deceleration;
	SERVO_PARA_COMMO_INFO filter[12];

}PARA_SPEED_TRAJ_READ_INFO;

typedef struct
{
	SERVO_PARA_COMMO_INFO target_point;
	SERVO_PARA_COMMO_INFO profile_velocity;
	SERVO_PARA_COMMO_INFO max_velocity;
	SERVO_PARA_COMMO_INFO profile_accleration;
	SERVO_PARA_COMMO_INFO profile_deceleration;
	SERVO_PARA_COMMO_INFO qstop_deceration;
	SERVO_PARA_COMMO_INFO accleration_type;
	SERVO_PARA_COMMO_INFO profile_jerk;
	SERVO_PARA_COMMO_INFO postion_control_max_vel;
	SERVO_PARA_COMMO_INFO filter[7];


}PARA_POSITION_TRAJ_READ_INFO;

typedef struct
{
	SERVO_PARA_COMMO_INFO encode_id;
	SERVO_PARA_COMMO_INFO encode_class;//编码器类型，旋转或者直线
	SERVO_PARA_COMMO_INFO encode_type;//绝对式或者增量式
	SERVO_PARA_COMMO_INFO encode_direction;//安装增减方向
	SERVO_PARA_COMMO_INFO encode_sig_bit;//单圈位数
	SERVO_PARA_COMMO_INFO encode_mul_bit;//多圈位数
	SERVO_PARA_COMMO_INFO encode_resolution;//编码器位数
	SERVO_PARA_COMMO_INFO encode_range;//直线编码器需要
	SERVO_PARA_COMMO_INFO encoe_offset_ls;
	SERVO_PARA_COMMO_INFO encode_offset_hs;
	SERVO_PARA_COMMO_INFO filter[6];

}PARA_ENCODE_CFG_READ_INFO;

typedef struct
{
	SERVO_PARA_COMMO_INFO power_model_id;
	SERVO_PARA_COMMO_INFO power_rated_input_voltage;
	SERVO_PARA_COMMO_INFO power_rated_output_current;
	SERVO_PARA_COMMO_INFO power_peak_output_current;
	SERVO_PARA_COMMO_INFO filter[12];


}PARA_DRIVER_CFG_READ_INFO;

typedef struct
{
	SERVO_PARA_COMMO_INFO notch_freq;
	SERVO_PARA_COMMO_INFO k1_factor;
	SERVO_PARA_COMMO_INFO k2_factor;
	SERVO_PARA_COMMO_INFO valid_flag;

}PARA_NOTCH_COMMON_READ_INFO;

typedef struct
{
	SERVO_PARA_COMMO_INFO lowpass_frequency;

}PARA_FLITER_COMMON_READ_INFO;

typedef struct
{
	PARA_POSITION_LOOP_READ_INFO  pos_alg_loop;
	PARA_SPEED_LOOP_READ_INFO     sp_alg_loop;
	PARA_CURRENT_LOOP_READ_INFO   current_alg_loop;

}PARA_ALG_READ_INFO;

typedef struct
{
	PARA_NOTCH_COMMON_READ_INFO pos_notch;
	PARA_NOTCH_COMMON_READ_INFO sp1_notch;
	PARA_NOTCH_COMMON_READ_INFO sp2_notch;

	PARA_FLITER_COMMON_READ_INFO vel_given_lowpass_fre;
	PARA_FLITER_COMMON_READ_INFO vel_feed_lowpass_fre;
	PARA_FLITER_COMMON_READ_INFO vel_back_lowpass_fre;
	PARA_FLITER_COMMON_READ_INFO iq_given_lowpass_fre;
	PARA_FLITER_COMMON_READ_INFO iq_feed_lowpass_fre;
	PARA_FLITER_COMMON_READ_INFO iq_back_lowpass_fre;

}PARA_SIGNAL_PROCESS_READ_INFO;


typedef struct
{
	SERVO_PARA_COMMO_INFO servo_ctrl;
	SERVO_PARA_COMMO_INFO expect_mode;
	SERVO_PARA_COMMO_INFO pos_lsb;
	SERVO_PARA_COMMO_INFO pos_hsb;
	SERVO_PARA_COMMO_INFO velocity_cmd;
	SERVO_PARA_COMMO_INFO torque_cmd;
	SERVO_PARA_COMMO_INFO velocity_ff_cmd;
	SERVO_PARA_COMMO_INFO acc_ff_cmd;

}PARA_CTRL_CMD_READ_INFO;

typedef struct
{
	SERVO_PARA_COMMO_INFO servo_state_word;
	SERVO_PARA_COMMO_INFO operation_mode_act;
	SERVO_PARA_COMMO_INFO position_fdb_lsb;
	SERVO_PARA_COMMO_INFO position_fdb_msb;
	SERVO_PARA_COMMO_INFO velocity_fdb;
	SERVO_PARA_COMMO_INFO current_torque;
	SERVO_PARA_COMMO_INFO current_power;
	SERVO_PARA_COMMO_INFO velocity_loop_input;
	SERVO_PARA_COMMO_INFO torque_loop_input;
	SERVO_PARA_COMMO_INFO position_error;
	SERVO_PARA_COMMO_INFO velocity_error;
	SERVO_PARA_COMMO_INFO current_error;
	SERVO_PARA_COMMO_INFO ia_sampling;
	SERVO_PARA_COMMO_INFO ib_sampling;
	SERVO_PARA_COMMO_INFO ic_sampling;
	SERVO_PARA_COMMO_INFO dc_voltage_sampling;
	SERVO_PARA_COMMO_INFO power_temp_sampling;
	SERVO_PARA_COMMO_INFO dc_temp_sampling;
	SERVO_PARA_COMMO_INFO motor_temp_sampling;

}PARA_VAR_MONITOR_READ_INFO;

typedef struct
{
	SERVO_PARA_COMMO_INFO         reservd;//1
	PARA_POSITION_TRAJ_READ_INFO  position_tarj_info;//16
	PARA_SPEED_TRAJ_READ_INFO     speed_traj_info;//16
	PARA_TORQUE_TRAJ_READ_INFO    torque_traj_info;//16
	PARA_DRIVER_CFG_READ_INFO     driver_info;//16
	PARA_ENCODE_CFG_READ_INFO 	  encode_info;//16
	PARA_MOTOR_CFG_READ_INFO  	  motor_info;//31
	PARA_BRAKER_READ_INFO         braker_info;//16
	PARA_ERR_CHECK_READ_INFO  	  heck_err_info;//48
	PARA_ALG_READ_INFO   		  axis_alg_info;//48
	PARA_SIGNAL_PROCESS_READ_INFO axis_signal_process;//18
	SERVO_PARA_COMMO_INFO         reserved1[208];
	PARA_CTRL_CMD_READ_INFO       axis_ctrl_info;//19
	PARA_VAR_MONITOR_READ_INFO    axis_monitor_info;//18
	SERVO_PARA_COMMO_INFO         reserved2[23];
	SERVO_PARA_COMMO_INFO         sync_reg;
	SERVO_PARA_COMMO_INFO         reserved3[11];

}PARA_AXIS_READ_INFO;

typedef struct
{
	int32_t kp_factor;
	int32_t time_res_factor;//equal ki_factor;
	int32_t inertia_factor;
	int32_t vel_feed_factor;
	int32_t vel_input_sature;
	int32_t iq_output_sature;
	int32_t arriving_speed_limit;
	int32_t arriving_time_limit;
	int32_t period_speed_loop;
	int32_t zero_speed_limit;
	int32_t zero_speed_time_limit;
	int32_t filter[5];

}PARA_SPEED_LOOP_LOCAL_INFO;

typedef struct
{
	int32_t kp_factor;
	int32_t period_pos_loop;
	int32_t arriving_pos_limit;
	int32_t arriving_time_limit;
	int32_t filter[12];

}PARA_POSITION_LOOP_LOCAL_INFO;

typedef struct
{
	int32_t kp_factor;
	int32_t ki_factor;
	int32_t pwm_frequency_selection;
	int32_t period_current_loop;
	int32_t torque_factor;
	int32_t filter[11];

}PARA_CURRENT_LOOP_LOCAL_INFO;

typedef struct
{
	int32_t brake_ctrl_word;
	int32_t braker_release_time;
	int32_t braker_action_time;
	int32_t braker_holding_voltage;
	int32_t braker_pulse_time;
	int32_t filter[11];

}PARA_BRAKER_LOCAL_INFO;

typedef struct
{
	int32_t target_point;
	int32_t profile_velocity;
	int32_t max_velocity;
	int32_t profile_accleration;
	int32_t profile_deceleration;
	int32_t qstop_deceration;
	int32_t accleration_type;
	int32_t profile_jerk;
	int32_t postion_control_max_vel;
	int32_t filter[7];

}PARA_POSITION_TRAJ_LOCAL_INFO;

typedef struct
{
	int32_t target_velocity;
	int32_t max_profile_velocity;
	int32_t max_acceleration;
	int32_t max_deceleration;
	int32_t filter[12];

}PARA_SPEED_TRAJ_LOCAL_INFO;

typedef struct
{
	int32_t target_torque;
	int32_t max_torque;
	int32_t torque_slope;
	int32_t torque_profile_type;
	int32_t positive_torque_imit;
	int32_t negative_torque_limit;
	int32_t torque_control_gain;
	int32_t torque_control_time;
	int32_t filter[8];

}PARA_TORQUE_TRAJ_LOCAL_INFO;

typedef struct
{
	int32_t motor_model_id;
	int32_t motor_direction;
	int32_t motor_rated_current;
	int32_t motor_max_current_in_percent;
	int32_t motor_rated_torque;
	int32_t motor_max_torque;
	int32_t motor_rated_rpm;
	int32_t motor_max_rpm;
	int32_t motor_polo_angle;
	int32_t motor_pole_pairs;
	int32_t motor_kt;
	int32_t motor_ke;
	int32_t motor_ld;
	int32_t motor_lq;
	int32_t motor_winding_resistance;
	int32_t motor_rotor_inertia;
	int32_t thermal_time_constant;
	int32_t sample_resistence;
	int32_t filter[13];

}PARA_MOTOR_CFG_LOCAL_INFO;

typedef struct
{
	int32_t power_model_id;
	int32_t power_rated_input_voltage;
	int32_t power_rated_output_current;
	int32_t power_peak_output_current;
	int32_t power_input_type;
	int32_t filter[11];

}PARA_DRIVER_CFG_LOCAL_INFO;

typedef struct
{
	int32_t encode_id;
	int32_t encode_class;//编码器类型，旋转或者直线
	int32_t encode_type;//绝对式或者增量式
	int32_t encode_direction;//安装增减方向
	int32_t encode_sig_bit;//单圈位数
	int32_t encode_mul_bit;//多圈位数
	int32_t encode_resolution;//编码器位数
	int32_t encode_range;//直线编码器需要
	int32_t encode_offset_ls;
	int32_t encode_offset_hs;
	int32_t filter[6];

}PARA_ENCODE_CFG_LOCAL_INFO;


typedef struct
{
	int32_t err_ctrl_word;
	int32_t peak_current_window;
	int32_t power_moudle_over_temp;
	int32_t dc_over_temp;
	int32_t motor_over_temp;
	int32_t vol_bus_up_limit;
	int32_t vol_bus_down_limit;
	int32_t pos_follow_err_limit;
	int32_t pos_follow_time_limit;
	int32_t speed_follow_err_limit;
	int32_t speed_follow_time_limit;
	int32_t power_overload_current_1;
	int32_t power_overload_time_1;
	int32_t power_overload_current_2;
	int32_t power_overload_time_2;
	int32_t motor_overload_current_1;
	int32_t motor_overload_time_1;
	int32_t motor_overload_current_2;
	int32_t motor_overload_time_2;
	int32_t resetable_error_flag;
	int32_t reboot_error_flag;
	int32_t current_error_code;
	int32_t history_error_code1;
	int32_t history_error_code2;
	int32_t history_error_code3;
	int32_t history_error_code4;
	int32_t history_error_code5;
	int32_t history_error_code6;
	int32_t history_error_code7;
	int32_t history_error_code8;

	int32_t filter[18];

}PARA_ERR_CHECK_LOCAL_INFO;

typedef struct
{
	int32_t lowpass_frequency;

}PARA_FLITER_COMMON_LOCAL_INFO;


typedef struct
{
	int32_t notch_freq;
	int32_t k1_factor;
	int32_t k2_factor;
	int32_t valid_flag;

}PARA_NOTCH_COMMON_LOCAL_INFO;

typedef struct
{
	PARA_POSITION_LOOP_LOCAL_INFO  pos_alg_loop;
	PARA_SPEED_LOOP_LOCAL_INFO     sp_alg_loop;
	PARA_CURRENT_LOOP_LOCAL_INFO   current_alg_loop;

}PARA_ALG_LOCAL_INFO;

typedef struct
{
	PARA_NOTCH_COMMON_LOCAL_INFO pos_notch;
	PARA_NOTCH_COMMON_LOCAL_INFO sp1_notch;
	PARA_NOTCH_COMMON_LOCAL_INFO sp2_notch;

	PARA_FLITER_COMMON_LOCAL_INFO vel_given_lowpass_fre;
	PARA_FLITER_COMMON_LOCAL_INFO vel_feed_lowpass_fre;
	PARA_FLITER_COMMON_LOCAL_INFO vel_back_lowpass_fre;
	PARA_FLITER_COMMON_LOCAL_INFO iq_given_lowpass_fre;
	PARA_FLITER_COMMON_LOCAL_INFO iq_feed_lowpass_fre;
	PARA_FLITER_COMMON_LOCAL_INFO iq_back_lowpass_fre;

}PARA_SIGNAL_PROCESS_INFO;

typedef struct
{
	int32_t servo_ctrl;
	int32_t expect_mode;
	int32_t pos_lsb;
	int32_t pos_hsb;
	int32_t velocity_cmd;
	int32_t torque_cmd;
	int32_t velocity_ff_cmd;
	int32_t acc_ff_cmd;

}PARA_CTRL_CMD_LOCAL_INFO;

typedef struct
{
	int32_t servo_state_word;
	int32_t operation_mode_act;
	int32_t position_fdb_lsb;
	int32_t position_fdb_msb;
	int32_t velocity_fdb;
	int32_t current_torque;
	int32_t current_power;
	int32_t velocity_loop_input;
	int32_t torque_loop_input;
	int32_t position_error;
	int32_t velocity_error;
	int32_t current_error;
	int32_t ia_sampling;
	int32_t ib_sampling;
	int32_t ic_sampling;
	int32_t dc_voltage_sampling;
	int32_t power_temp_sampling;
	int32_t dc_temp_sampling;
	int32_t motor_temp_sampling;

}PARA_VAR_MONITOR_LOCAL_INFO;

typedef struct
{
	int32_t reservd;//1
	PARA_POSITION_TRAJ_LOCAL_INFO  position_tarj_info;//16
	PARA_SPEED_TRAJ_LOCAL_INFO     speed_traj_info;//16
	PARA_TORQUE_TRAJ_LOCAL_INFO    torque_traj_info;//16
	PARA_DRIVER_CFG_LOCAL_INFO     driver_info;//16
	PARA_ENCODE_CFG_LOCAL_INFO 	   encode_info;//16
	PARA_MOTOR_CFG_LOCAL_INFO  	   motor_info;//31
	PARA_BRAKER_LOCAL_INFO         braker_info;//16
	PARA_ERR_CHECK_LOCAL_INFO  	   heck_err_info;//48
	PARA_ALG_LOCAL_INFO   		   axis_alg_info;//48
	PARA_SIGNAL_PROCESS_INFO       axis_signal_process;//18
	int32_t         			   reserver1[208];//208
	PARA_CTRL_CMD_LOCAL_INFO       ref_cmd_info;//8
	PARA_VAR_MONITOR_LOCAL_INFO    monitor_info;//19
	int32_t         			   reserver2[23];//23
	int32_t						   sync_reg;//1
	int32_t                        reserver3[11];//11

}PARA_AXIS_LOCAL_INFO;


void Servo_Para_Init(void);
/*获取框架参数数据，在轴框架中进行数据分发*/
bool Servo_Para_Get_ReadPtr(SERVO_AXIS_ENUM axis_id,
		PARA_AXIS_READ_INFO** get_read_ptr);

bool Servo_Para_Get_LocalPtr(SERVO_AXIS_ENUM axis_id,
		PARA_AXIS_LOCAL_INFO** get_local_ptr);

bool Servo_Para_Manager_WritePara(SERVO_AXIS_ENUM axis_id
		,int32_t para_value,uint32_t index_offset);

bool Servo_Para_Manager_ReadPara(SERVO_AXIS_ENUM axis_id
		,int32_t* get_para,uint32_t index_offset);

bool Servo_Para_Manager_UploadPara(SERVO_AXIS_ENUM axis_id,
							CommBlockData_t* upload_para_ptr,
                                  uint32_t copy_length);

bool Servo_Para_Manager_DownPara(SERVO_AXIS_ENUM axis_id
						,const CommBlockData_t* axis_para_ptr);

bool Servo_p_Test(void);

bool Servo_Para_SetPara(SERVO_AXIS_ENUM axis_id,
                        uint32_t index_offset,uint32_t para_length);

bool Servo_b_Get_ServoMode(SERVO_AXIS_ENUM axis_id
		,int32_t** servo_mode);
}

#endif /* SERVO_PARA_H_ */

