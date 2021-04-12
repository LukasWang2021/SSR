/*
 *Servo_Dignose.h
 *
 *Created on: 2019年7月15日
 *Author: qichao.wang
 */

 #ifndef SERVO_DIGNOSE_H_
 #define SERVO_DIGNOSE_H_

 #include "Servo_Para.h"
 #include "Servo_CoreProcess.h"
 #include "Servo_General.h"
 #define VAR_PTR_MAX 16

namespace virtual_servo_device{

 typedef enum
 {
    POS_DEM_VAL,
    POS_ACT_VAL,
    POS_FOLL_ERR,
    POS_KP_OUT,
    VEL_FEED_VAL,
    VEL_DEM_VAL,
    VEL_ACT_VAL,
    VEL_FOLL_ERR,
    VEL_KP_OUT,
    VEL_KI_OUT,
    TOR_FEED_VAL,
    IQ_GIVEN_VAL,
    IQ_BACK_VAL,
    PHA_CURRENT_U,
    PHA_CURRENT_V,
    ENCODE_SOURCE_VAL,

    VAR_MAX = ENCODE_SOURCE_VAL + 1,


 }SERVO_DIGNOSE_VAR_ENUM;

 typedef enum
 {
    CHANNEL_0,
    CHANNEL_1,
    CHANNEL_2,
    CHANNEL_3,
    CHANNEL_4,
    CHANNEL_5,
    CHANNEL_6,
    CHANNEL_7,
    CHANNEL_8,
    CHANNEL_9,
    CHANNEL_10,
    CHANNEL_11,
    CHANNEL_12,
    CHANNEL_13,
    CHANNEL_14,
    CHANNEL_15,
    CHANNEL_MAX = CHANNEL_15+1,

 }SERVO_CHANNEL_ENUM;

 typedef struct
 {
    int32_t var_cha1;
    int32_t var_cha2;
    int32_t var_cha3;
    int32_t var_cha4;
    int32_t var_cha5;
    int32_t var_cha6;
    int32_t var_cha7;
    int32_t var_cha8;
    int32_t var_cha9;
    int32_t var_cha10;
    int32_t var_cha11;
    int32_t var_cha12;
    int32_t var_cha13;
    int32_t var_cha14;
    int32_t var_cha15;
    int32_t var_cha16;

 }SERVO_DIGNOSE_VAR_STRUCT;

 typedef struct
 {
    SERVO_DIGNOSE_VAR_ENUM var_id;
    int32_t *var_ptr;

 }SERVO_DIGNOSE_VAR_PROCESS_STRUCT;

 typedef struct
 {
    SERVO_DIGNOSE_VAR_PROCESS_STRUCT cha1_ptr;
    SERVO_DIGNOSE_VAR_PROCESS_STRUCT cha2_ptr;
    SERVO_DIGNOSE_VAR_PROCESS_STRUCT cha3_ptr;
    SERVO_DIGNOSE_VAR_PROCESS_STRUCT cha4_ptr;
    SERVO_DIGNOSE_VAR_PROCESS_STRUCT cha5_ptr;
    SERVO_DIGNOSE_VAR_PROCESS_STRUCT cha6_ptr;
    SERVO_DIGNOSE_VAR_PROCESS_STRUCT cha7_ptr;
    SERVO_DIGNOSE_VAR_PROCESS_STRUCT cha8_ptr;
    SERVO_DIGNOSE_VAR_PROCESS_STRUCT cha9_ptr;
    SERVO_DIGNOSE_VAR_PROCESS_STRUCT cha10_ptr;
    SERVO_DIGNOSE_VAR_PROCESS_STRUCT cha11_ptr;
    SERVO_DIGNOSE_VAR_PROCESS_STRUCT cha12_ptr;
    SERVO_DIGNOSE_VAR_PROCESS_STRUCT cha13_ptr;
    SERVO_DIGNOSE_VAR_PROCESS_STRUCT cha14_ptr;
    SERVO_DIGNOSE_VAR_PROCESS_STRUCT cha15_ptr;
    SERVO_DIGNOSE_VAR_PROCESS_STRUCT cha16_ptr;

 }SERVO_DIGNOSE_PTR_INFO_STRUCT;

 typedef struct
 {
    uint32_t sample_count;
    uint32_t sample_flag;

 }SERVO_DIGNOSE_CON_SAMPLE_STRUCT;

 void Servo_Dignose_Init(void);

 void Servo_Dignose_SetPara(SERVO_AXIS_ENUM axis_id,
                                        SERVO_DIGNOSE_VAR_ENUM var_id,
                                        SERVO_CHANNEL_ENUM channel_id);

 int32_t Servo_Dignose_Register_Var(SERVO_AXIS_ENUM axis_id
                                        ,int32_t* var_ptr
                                        ,SERVO_DIGNOSE_VAR_ENUM var_id);

 void Servo_Dignose_p_GetVar(void);

 void Servo_Dignose_b_AnaInfo(void);

}
 #endif /* SERVO_DIGNOSE_H_ */

