/*
 *Servo_Axis.h
 *
 *Created on: 2019年7月15日
 *Author: qichao.wang
 */

 #ifndef SERVO_AXIS_H_
 #define SERVO_AXIS_H_

 #include "Servo_General.h"
 #include "Servo_CoreProcess.h"
 #include "Servo_Para.h"
 #include "Servo_Speed_Loop.h"
 #include "Servo_Speed.h"
 #include "Servo_Traj.h"
 #include "Servo_QuickStop.h"
 #include "Servo_Inter_SpeedTraj.h"
 #include "Servo_Inter_PosTraj.h"
 #include "Servo_Inter_TorTraj.h"
 #include "Servo_Encode.h"
 #include "Servo_Speed_Loop.h"
 #include "Servo_Position_Loop.h"
namespace virtual_servo_device{

    void Servo_Axis_Init(void);

    void Servo_Axis_p_Cal_PosErr(SERVO_AXIS_ENUM axis_id);

    void Servo_Axis_p_Cal_PosLoop(SERVO_AXIS_ENUM axis_id);

    void Servo_Axis_p_Cal_SpErr(SERVO_AXIS_ENUM axis_id);

    void Servo_Axis_p_Cal_SpeedLoop(SERVO_AXIS_ENUM axis_id);

    void servo_config_CoreComm(void);

    void servo_axis_p_machine_state(SERVO_AXIS_ENUM axis_id);

    void servo_axis_p_comm_state(SERVO_AXIS_ENUM axis_id);

    void Servo_Axis_b_DownPara(SERVO_AXIS_ENUM axis_id);

    void Servo_Axis_b_UploadPara(SERVO_AXIS_ENUM axis_id);

    void servo_axis_test_dignose(void);

    void print_axis_state(SERVO_AXIS_ENUM axis_id);

    void Servo_Axis_Set_AxisState(SERVO_AXIS_ENUM axis_id,ServoSm_e servo_state);

    void Servo_Axis_Set_AxisErr(SERVO_AXIS_ENUM axis_id);

    void Servo_Axis_CheckErr(SERVO_AXIS_ENUM axis_id);

    void Servo_Axis_DriverErr(void);

    void Servo_Axis_Set_TargetPoint(SERVO_AXIS_ENUM axis_id);

    void Servo_Axis_UpdateContrcmd(SERVO_AXIS_ENUM axis_id);

    void Servo_Axis_UpdateMonitor(SERVO_AXIS_ENUM axis_id);

    void Servo_Axis_Check_ArrivePos(SERVO_AXIS_ENUM axis_id);

    void Servo_Axis_ResetCfg(void);
}
 #endif /* SERVO_AXIS_H_ */

