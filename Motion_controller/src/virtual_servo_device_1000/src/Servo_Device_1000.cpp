#include <iostream>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include "thread_help.h"
#include "system/core_comm_system.h"
#include "common/servo_interface.h"
#include "common/servo_cpu_interface.h"

#include "Servo_Fpga.h"
#include "Servo_Dignose.h"
#include "Servo_Axis.h"
#include "Servo_Traj.h"
#include "Servo_Speed.h"
#include "Servo_Para.h"
#include "Servo_Motor.h"
#include "Servo_CheckErr.h"
#include "error_process.h"
#include "Servo_Safety.h"
#include "Servo_Inter_PosTraj.h"
// servo cpu supports 8 servo units
// axis0~3: is configured to SCARA1
// axis4~7: is configured to SCARA2


using namespace std;
using namespace base_space;
using namespace core_comm_space;
using namespace virtual_servo_device;


typedef struct ALT_TEST_TIMER_CONTEXT_s
{
    volatile int count;
    volatile int flag;
    volatile unsigned int cnt;
    volatile unsigned int cnt1;
    volatile uint16_t temp;
    volatile unsigned int soft_count;

    uint32_t  axis_0;
    uint32_t  axis_1;
    uint32_t  axis_2;
    uint32_t  axis_3;
    uint32_t  axis_4;
    uint32_t  axis_5;
    uint32_t  axis_6;
    uint32_t  axis_7;

    uint32_t isr_count;

}ALT_TEST_TIMER_CONTEXT_t;


static ALT_TEST_TIMER_CONTEXT_t  data;

// Note: in real bare core, the function calling sequence should not like the virtual one coding here!!!
void* servoIsrFunc(void* arg)
{
    std::cout<<"servo cpu main isr running---------"<<std::endl;

    while(1)
    {
	//Servo_Axis_DriverErr();
    	Servo_Safety_p_RwSafety();

    	/*状态机切换相关*/
        servo_axis_p_machine_state(AXIS_0);
        servo_axis_p_machine_state(AXIS_1);
        servo_axis_p_machine_state(AXIS_2);
        servo_axis_p_machine_state(AXIS_3);
        servo_axis_p_machine_state(AXIS_4);
        servo_axis_p_machine_state(AXIS_5);
        servo_axis_p_machine_state(AXIS_6);
        servo_axis_p_machine_state(AXIS_7);
        /*通行模式相关切换*/
        servo_axis_p_comm_state(AXIS_0);
        servo_axis_p_comm_state(AXIS_1);
        servo_axis_p_comm_state(AXIS_2);
        servo_axis_p_comm_state(AXIS_3);
        servo_axis_p_comm_state(AXIS_4);
        servo_axis_p_comm_state(AXIS_5);
        servo_axis_p_comm_state(AXIS_6);
        servo_axis_p_comm_state(AXIS_7);

    	Servo_Read_p_FPGA(AXIS_0);
    	Servo_Read_p_FPGA(AXIS_1);
    	Servo_Read_p_FPGA(AXIS_2);
    	Servo_Read_p_FPGA(AXIS_3);
    	Servo_Read_p_FPGA(AXIS_4);
    	Servo_Read_p_FPGA(AXIS_5);
    	Servo_Read_p_FPGA(AXIS_6);
    	Servo_Read_p_FPGA(AXIS_7);
    	/*位置点位信息更新相关*/
    	Servo_Traj_p_UpdatePoint(AXIS_0);
    	Servo_Traj_p_UpdatePoint(AXIS_1);
    	Servo_Traj_p_UpdatePoint(AXIS_2);
    	Servo_Traj_p_UpdatePoint(AXIS_3);
    	Servo_Traj_p_UpdatePoint(AXIS_4);
    	Servo_Traj_p_UpdatePoint(AXIS_5);
    	Servo_Traj_p_UpdatePoint(AXIS_6);
    	Servo_Traj_p_UpdatePoint(AXIS_7);

    	Servo_Encode_p_CalPos(AXIS_0);
    	Servo_Encode_p_CalPos(AXIS_1);
    	Servo_Encode_p_CalPos(AXIS_2);
    	Servo_Encode_p_CalPos(AXIS_3);
    	Servo_Encode_p_CalPos(AXIS_4);
    	Servo_Encode_p_CalPos(AXIS_5);
    	Servo_Encode_p_CalPos(AXIS_6);
    	Servo_Encode_p_CalPos(AXIS_7);

    	if(data.soft_count <1)
    	{
    		/*位置差值计算*/
    		Servo_Axis_p_Cal_PosErr(AXIS_0);
    		Servo_Axis_p_Cal_PosErr(AXIS_1);
    		Servo_Axis_p_Cal_PosErr(AXIS_2);
    		Servo_Axis_p_Cal_PosErr(AXIS_3);
    		Servo_Axis_p_Cal_PosErr(AXIS_4);
    		Servo_Axis_p_Cal_PosErr(AXIS_5);
    		Servo_Axis_p_Cal_PosErr(AXIS_6);
    		Servo_Axis_p_Cal_PosErr(AXIS_7);
    		/*位置环路计算*/
    		Servo_Axis_p_Cal_PosLoop(AXIS_0);
    		Servo_Axis_p_Cal_PosLoop(AXIS_1);
    		Servo_Axis_p_Cal_PosLoop(AXIS_2);
    		Servo_Axis_p_Cal_PosLoop(AXIS_3);
    		Servo_Axis_p_Cal_PosLoop(AXIS_4);
    		Servo_Axis_p_Cal_PosLoop(AXIS_5);
    		Servo_Axis_p_Cal_PosLoop(AXIS_6);
    		Servo_Axis_p_Cal_PosLoop(AXIS_7);

    		data.soft_count++;

    	}else{
    		data.soft_count = 0;
    	}
    	/*速度计算*/
    	Servo_Speed_p_CalSpeed(AXIS_0);
    	Servo_Speed_p_CalSpeed(AXIS_1);
    	Servo_Speed_p_CalSpeed(AXIS_2);
    	Servo_Speed_p_CalSpeed(AXIS_3);
    	Servo_Speed_p_CalSpeed(AXIS_4);
    	Servo_Speed_p_CalSpeed(AXIS_5);
    	Servo_Speed_p_CalSpeed(AXIS_6);
    	Servo_Speed_p_CalSpeed(AXIS_7);
    	/*速度差值计算*/
    	Servo_Axis_p_Cal_SpErr(AXIS_0);
    	Servo_Axis_p_Cal_SpErr(AXIS_1);
    	Servo_Axis_p_Cal_SpErr(AXIS_2);
    	Servo_Axis_p_Cal_SpErr(AXIS_3);
    	Servo_Axis_p_Cal_SpErr(AXIS_4);
    	Servo_Axis_p_Cal_SpErr(AXIS_5);
    	Servo_Axis_p_Cal_SpErr(AXIS_6);
    	Servo_Axis_p_Cal_SpErr(AXIS_7);
    	/*速度环计算*/
    	Servo_Axis_p_Cal_SpeedLoop(AXIS_0);
    	Servo_Axis_p_Cal_SpeedLoop(AXIS_1);
    	Servo_Axis_p_Cal_SpeedLoop(AXIS_2);
    	Servo_Axis_p_Cal_SpeedLoop(AXIS_3);
    	Servo_Axis_p_Cal_SpeedLoop(AXIS_4);
    	Servo_Axis_p_Cal_SpeedLoop(AXIS_5);
    	Servo_Axis_p_Cal_SpeedLoop(AXIS_6);
    	Servo_Axis_p_Cal_SpeedLoop(AXIS_7);

 //   	Servo_Axis_CheckErr(AXIS_0);
//    	Servo_Axis_CheckErr(AXIS_1);
//    	Servo_Axis_CheckErr(AXIS_2);
//    	Servo_Axis_CheckErr(AXIS_3);
//    	Servo_Axis_CheckErr(AXIS_4);
//    	Servo_Axis_CheckErr(AXIS_5);
//    	Servo_Axis_CheckErr(AXIS_6);
//    	Servo_Axis_CheckErr(AXIS_7);

    	Servo_Axis_UpdateMonitor(AXIS_0);
    	Servo_Axis_UpdateMonitor(AXIS_1);
    	Servo_Axis_UpdateMonitor(AXIS_2);
    	Servo_Axis_UpdateMonitor(AXIS_3);
    	Servo_Axis_UpdateMonitor(AXIS_4);
    	Servo_Axis_UpdateMonitor(AXIS_5);
    	Servo_Axis_UpdateMonitor(AXIS_6);
    	Servo_Axis_UpdateMonitor(AXIS_7);

    	Servo_Axis_UpdateContrcmd(AXIS_0);
    	Servo_Axis_UpdateContrcmd(AXIS_1);
    	Servo_Axis_UpdateContrcmd(AXIS_2);
    	Servo_Axis_UpdateContrcmd(AXIS_3);
    	Servo_Axis_UpdateContrcmd(AXIS_4);
    	Servo_Axis_UpdateContrcmd(AXIS_5);
    	Servo_Axis_UpdateContrcmd(AXIS_6);
    	Servo_Axis_UpdateContrcmd(AXIS_7);

    	Servo_Axis_Check_ArrivePos(AXIS_0);
    	Servo_Axis_Check_ArrivePos(AXIS_1);
    	Servo_Axis_Check_ArrivePos(AXIS_2);
    	Servo_Axis_Check_ArrivePos(AXIS_3);
    	Servo_Axis_Check_ArrivePos(AXIS_4);
    	Servo_Axis_Check_ArrivePos(AXIS_5);
    	Servo_Axis_Check_ArrivePos(AXIS_6);
    	Servo_Axis_Check_ArrivePos(AXIS_7);

     	Servo_Dignose_p_GetVar();
     	servo_axis_test_dignose();

        usleep(2000);
    } 
    return NULL;
}


int main()
{
    std::cout<<"servo cpu start to run"<<std::endl;

    Servo_Para_Init();
    Servo_Fpga_Init();
    servo_cmd_init();
    Servo_CheckErr_Init();

    Servo_Motor_Init();
    Servo_PosTraj_Init();

    Servo_Encode_Init();
    Servo_Speed_Init();
    Servo_Axis_Init();
    servo_config_CoreComm();
    Servo_Dignose_Init();
    Servo_Safety_Init();
    Servo_SpLoop_Init();

    // main isr is set and running
    ThreadHelp isr_thread;
    isr_thread.run(&servoIsrFunc, NULL, 50);    

    usleep(100000);
    // routine task running
    while(1)
    {
        for(int32_t i=0; i<AXIS_MAX; i++)
        {
            Servo_Axis_b_DownPara(static_cast<SERVO_AXIS_ENUM>(i));
            Servo_Axis_b_UploadPara(static_cast<SERVO_AXIS_ENUM>(i));   
        }
        Servo_Dignose_b_AnaInfo();

        servo_config_CoreComm();

        usleep(100000);
    }

    std::cout<<"servo cpu shut down"<<std::endl;

    return 0;
}

