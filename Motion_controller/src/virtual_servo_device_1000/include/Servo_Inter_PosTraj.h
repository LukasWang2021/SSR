/*
 *Servo_Position.h
 *
 *Created on: 2019年7月23日
 *Author: qichao.wang
 */

 #ifndef SERVO_INTER_POSTRAJH_
 #define SERVO_INTER_POSTRAJH_

 #include "Servo_Para.h"
 #include "Servo_General.h"

 namespace virtual_servo_device{

 typedef enum
 {
    TRIANGLE_NO_VEL,
    TRIANGLE_VEL,
    TRAPOZID_NO_VEL,
    TRAPOZID_VEL,

 }SERVO_POS_TRAJ_ACC_STYLE_ENUM;

 typedef enum
 {
    TRAJ_IDLE,
    TRAJ_JERK_S1,
    TRAJ_ACC_S2,
    TRAJ_JERK_S3,
    TRAJ_CONS_VEL_S4,
    TRAJ_JERK_S5,
    TRAJ_ACC_S6,
    TRAJ_JERK_S7

 }SERVO_POS_TRAJ_STATE_ENUM;

 typedef struct
 {
    int32_t target_pos;
    int32_t constrain_vel;
    int32_t constrain_acc;
    int32_t constrain_dec;
    int32_t constrain_jerk;

 }SERVO_POS_TRAJ_PARA_INFO;

 typedef struct
 {
    double target_pos;
    double constrain_vel;
    double constrain_acc;
    double constrain_dec;
    double constrain_jerk;

 }SERVO_POS_TRAJ_DOUBLE_PARA_INFO;

 typedef struct
 {
    double pos_update;
    double vel_update;
    double acc_update;
    int32_t motion_flag;

 }SERVO_POS_TRAJ_UPDATE_INFO;

 typedef struct
 {
    int64_t pos_update;
    int64_t vel_update;
    int64_t acc_update;
    int64_t pos_offset;

 }SERVO_POS_TRAJ_CONV_UPDATE_INFO;

 typedef struct
 {
    int32_t t_jerk;
    int32_t t_acc;
    int32_t t_vel;

 }SERVO_POS_TRAJ_CONV_TIME_INFO;

 typedef struct
 {
    double t_jerk;
    double t_acc;
    double t_vel;

 }SERVO_POS_TRAJ_SOU_TIME_INFO;

 typedef struct
 {
    double t_jerk_max;
    double t_acc_max;
    double t_vel_max;

 }SERVO_POS_TARJ_VAR_TIME_INFO;

 void Servo_PosTraj_Init(void);

 void Servo_PosTraj_SetPara(SERVO_AXIS_ENUM axis_id,
                                    const int32_t* set_para);

 void Servo_PosTraj_UpateTraj(SERVO_AXIS_ENUM axis_id);

 void Servo_PosTraj_Get_TrajUpdate_Ptr(SERVO_AXIS_ENUM axis_id,
                                            SERVO_POS_TRAJ_CONV_UPDATE_INFO** get_update_ptr);

 void Servo_PosTraj_ResetVar(SERVO_AXIS_ENUM axis_id);
 }

 #endif /* SERVO_INTER_POSTRAJH_ */

