/*
 Servo_Encode.h
 *
 *Created on: 2019年7月15日
 *Author: qichao.wang
 */

 #ifndef SERVO_ENCODE_H_
 #define SERVO_ENCODE_H_

 #include "Servo_Para.h"

namespace virtual_servo_device{

 typedef struct
 {
    int64_t position_back;
    int32_t encode_source;

 }SERVO_ENCODE_OUTPUT_STRUCT;

 void Servo_Encode_Init(void);

 void Servo_Encode_SetPara(SERVO_AXIS_ENUM axis_id,
                                        const PARA_ENCODE_CFG_READ_INFO *para_ptr);

 void Servo_Encode_GetPara(SERVO_AXIS_ENUM axis_id,
                                        PARA_ENCODE_CFG_READ_INFO *para_ptr,
                                        int32_t copy_num);

 void Servo_Encode_Get_EncodePtr(SERVO_ENCODE_OUTPUT_STRUCT **get_ptr,SERVO_AXIS_ENUM axis_id);

 void Servo_Encode_p_CalPos(SERVO_AXIS_ENUM axis_id);

}

 #endif /* SERVO_ENCODE_H_ */

