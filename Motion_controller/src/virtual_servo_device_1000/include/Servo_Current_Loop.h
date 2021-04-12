/*
 *Servo_Current_Loop.h
 *
 *Created on: 2019年7月15日
 * Author: qichao.wang
 */

 #ifndef SERVO_CURRENT_LOOP_H_
 #define SERVO_CURRENT_LOOP_H_

 #include "Servo_Para.h"
 #include "Servo_General.h"
namespace virtual_servo_device{

 typedef struct
 {
    int64_t filter_out;
    int64_t filter_var;

 }SERVO_CURRENT_FILTER_ALG;

 void Servo_CurLoop_Init(void);

 void Servo_CurLoop_SetPara(SERVO_AXIS_ENUM axis_id,
                                const PARA_CURRENT_LOOP_READ_INFO *para_ptr);

 int32_t Servo_CurLoop_GetPara(SERVO_AXIS_ENUM axis_id,
                                    PARA_CURRENT_LOOP_READ_INFO *para_ptr,
                                    int32_t copy_num);
 /*前馈滤波后，加的整个iq值上*/
 void Servo_CurLoop_s_TorqueFeed(SERVO_AXIS_ENUM axis_id,int64_t *current_out,
                                        int64_t set_torque);

 void Servo_CurLoop_p_QCurrentDcoup(SERVO_AXIS_ENUM axis_id);
}

 #endif /* SERVO_CURRENT_LOOP_H_ */

