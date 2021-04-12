/*
 *Servo_Inter_TorTraj.h
 *
 *Created on: 2019年7月24日
 *Author: qichao.wang
 */

 #ifndef SERVO_INTER_TORTRAJ_H_
 #define SERVO_INTER_TORTRAJ_H_

 #include "Servo_Para.h"

namespace virtual_servo_device{

 typedef struct
 {



 }SERVO_TORQUE_TRAJ_VAR_STRUCT;

 typedef struct
 {
    int64_t torque_traj;

 }SERVO_TORQUE_TRAJ_UPDATE_STRUCT;

 void Servo_TorTraj_Init(void);

 void Servo_TorTrj_SetPara(SERVO_AXIS_ENUM axis_id,const PARA_TORQUE_TRAJ_READ_INFO *para_ptr);

 int32_t Servo_TorTrj_GetPara(PARA_TORQUE_TRAJ_READ_INFO *para_ptr
                                    ,SERVO_AXIS_ENUM axis_id,
                                    int32_t copy_num);

 int32_t Servo_TorTrj_Get_TrjPtr(SERVO_TORQUE_TRAJ_UPDATE_STRUCT **update_torque,SERVO_AXIS_ENUM axis_id);

 void Servo_TorTrj_UpdateTraj(SERVO_AXIS_ENUM axis_id);

}
 #endif /* SERVO_INTER_TORTRAJ_H_ */

