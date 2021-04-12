/*
 *Servo_Motor.h
 *
 *Created on: 20¹´7æ15æ¥
 *Author: qichao.wang
 */
 
 #ifndef SERVO_MOTOR_H_
 #define SERVO_MOTOR_H_
 
 #include "Servo_Para.h"
 #include "Servo_General.h"
 
 #define CAL_CURRENT_FACTOR     115873   //(x/32767)*(320/r)*(q24/ia*sqrt(2))
 
 #define CURRENT_NUM_SHIFT 4
 #define GLOBAL_ENABLE 0x01

namespace virtual_servo_device{

 typedef enum
 {
    MOTOR_DISABLE,
    MOTOR_ENABLE,

 }SERVO_MOTOR_CMD_ENUM;
 
 typedef struct
 {
    int64_t conv_factor;

 }SERVO_MOTOR_TORQUE2CURRENT_FACTOR_STRUCT;

 void Servo_Motor_Init(void);
 
 void Servo_Motor_SetPara(SERVO_AXIS_ENUM axis_id,
                                const PARA_MOTOR_CFG_READ_INFO *para_ptr);
 
 int32_t Servo_Motor_GetPara(SERVO_AXIS_ENUM axis_id,
                                    PARA_MOTOR_CFG_READ_INFO *para_ptr
                                    ,int32_t copy_num);
 
 void Servo_Motor_TorqueFeed(SERVO_AXIS_ENUM axis_id,
                                    int64_t torque_feed,
                                    int64_t* output);
 
 void Servo_Motor_ConMotor(SERVO_AXIS_ENUM axis_id
                                ,int enable);
} 
 #endif /* SERVO_MOTOR_H_ */

