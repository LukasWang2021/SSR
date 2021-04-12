/*
 *Servo_Notch.h
 *
 *Created on: 2019年7月15日
 *Author: qichao.wang
 */

 #ifndef SERVO_NOTCH_H_
 #define SERVO_NOTCH_H_

 #include "./servo_include/Servo_General.h"
 #include "./servo_include/Servo_Para.h"

namespace virtual_servo_device{
 typedef enum
 {
    POS_NOTCH,
    SP1_NOTCH,
    SP2_NOTCH,
    MAX_NOTCH = SP2_NOTCH + 1,

 }SERVO_NOTCH_ENUM;

 typedef struct
 {
    double temp_var_h;
    double temp_var_i;
    double temp_var_j;
    double temp_var_k;
    double temp_var_l;

 }SERVO_NOTCH_FILTER_VAR_STRUCT;

 typedef struct
 {
    int32_t notch_freq;
    int32_t k1_factor;
    int32_t k2_factor;
    int32_t valid_flag;

 }SERVO_NOTCH_COMMON_STRUCT;

 typedef struct
 {
    double a_factor[3];
    double b_factor[3];
    int32_t a_conv_factor[3];
    int32_t b_conv_factor[3];

 }SERVO_NOTCH_FILTER_ALG_STRUCT;

 void Servo_Notch_Init(void);

 void Servo_Notch_SetPara(SERVO_AXIS_ENUM axis_id,
                                    SERVO_NOTCH_ENUM notch_id,
                                    const PARA_NOTCH_COMMON_LOCAL_INFO *notch_ptr);

 int32_t Servo_Notch_GetPara(SERVO_AXIS_ENUM axis_id,
                                    SERVO_NOTCH_ENUM notch_id,
                                    PARA_NOTCH_COMMON_READ_INFO *notch_ptr,
                                    int32_t copy_num);

 void Servo_Notch_s_Cal(SERVO_AXIS_ENUM axis_id
                                            ,SERVO_NOTCH_ENUM notch_id
                                            ,int64_t notch_in
                                            ,int64_t *notch_out
                                            );

 void Servo_Notch_s_PreCal(SERVO_AXIS_ENUM axis_id
                                            ,SERVO_NOTCH_ENUM notch_id
                                            ,int64_t notch_in
                                            ,int64_t notch_out
                                            );
 }

 #endif /* SERVO_NOTCH_H_ */

