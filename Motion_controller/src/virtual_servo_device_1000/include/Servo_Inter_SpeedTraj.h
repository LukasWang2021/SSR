/*
 *Servo_Inter_SpeedTraj.h
 *
 *Created on: 2019年7月24日
 *Author: qichao.wang
 */

 #ifndef SERVO_INTER_SPEEDTRAJ_H_
 #define SERVO_INTER_SPEEDTRAJ_H_

 #include "Servo_General.h"

 namespace virtual_servo_device{

 typedef enum
 {
    VELOCITY_IDLE,
    VELOCITY_JEARK_S1,
    VELOCITY_ACC,
    VELOCITY_JEARK_S2,
    VELOCITY_CONSTANT,

 }SERVO_SPEED_MODE_STATE_ENUM;

 typedef enum
 {
    J_LINE,
    JA_LINE

 }SERVO_SPEED_MODE_ACC_SHAPE_ENUM;

 typedef struct
 {
    int32_t target_velocity;
    int32_t pos_acc;
    int32_t dec_acc;
    int32_t vel_jeark;
    int32_t direction;

 }SERVO_SPEED_MODE_GIVEN_INFO;

 typedef struct
 {
    uint32_t jeark_time;
    uint32_t acc_time;

 }SERVO_SPEED_TIME_INFO;

 typedef struct
 {
    int32_t velocity_update;
    int32_t update_flag;

 }SERVO_SPEED_MODE_UPDATE_INFO;

 void Servo_SpTraj_Init(void);

 void Servo_SpTraj_SetPara(SERVO_AXIS_ENUM axis_id,
                                const int32_t *spe_ptr);

 void Servo_SpTraj_Get_SpeedPtr(SERVO_SPEED_MODE_UPDATE_INFO **update_vel
                                        ,SERVO_AXIS_ENUM axis_id);

 void Servo_SpTraj_p_Update(SERVO_AXIS_ENUM axis_id);

 void Servo_SpTraj_RestVar(SERVO_AXIS_ENUM axis_id);
 }

 #endif /* SERVO_INTER_SPEEDTRAJ_H_ */

