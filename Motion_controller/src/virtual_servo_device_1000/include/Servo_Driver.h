/*
 *Servo_Driver.h
 *
 *Created on: 2019年7月24日
 *Author: qichao.wang
 */

 #ifndef SERVO_DRIVER_H_
 #define SERVO_DRIVER_H_

 #include "Servo_Para.h"

namespace virtual_servo_device{

 void Servo_Driver_Init(void);

 void Servo_Driver_SetPara(SERVO_AXIS_ENUM axis_id,
                                const PARA_DRIVER_CFG_READ_INFO *para_ptr);

 int32_t Servo_Driver_GetPara(SERVO_AXIS_ENUM axis_id,
                                    PARA_DRIVER_CFG_READ_INFO *para_ptr,
                                    int32_t copy_num);
}

 #endif /* SERVO_DRIVER_H_ */

