/*
 *error_process.h
 *
 *Created on: 2019年10月8日
 *Author: qichao.wang
 */

 #ifndef ERROR_PROCESS_H_
 #define ERROR_PROCESS_H_

 #include "Servo_Para.h"
 #include "Servo_General.h"

namespace virtual_servo_device{


 void error_process_init(void);

 void error_process_report(SERVO_AXIS_ENUM axis_id,uint32_t error_code);

 void error_process_print(void);

 void error_process_reset_errcode(SERVO_AXIS_ENUM axis_id);
}
 #endif /* ERROR_PROCESS_H_ */

