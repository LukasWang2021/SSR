/*************************************************************************
	> File Name: manual_test.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月08日 星期三 17时27分48秒
 ************************************************************************/

#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <motion_control_arm_group.h>
#include "thread_help.h"


using namespace std;
using namespace fst_mc;
using namespace fst_log;
using namespace fst_base;


static void rtTask(void *group)
{
    ((BaseGroup*)group)->realtimeTask();
}

int main(int argc, char **argv)
{
    //size_t loop = 0;
    Logger log;
    ArmGroup arm(&log);
    ErrorMonitor error_monitor;
    ThreadHelp rt_thread;
    cout << "begin" << endl;

    arm.initGroup(&error_monitor);
    arm.activeRealtimeTask();
    rt_thread.run(&rtTask, &arm, 80);
    sleep(1);

    MotionTarget target;
    target.type = MOTION_JOINT;
    target.vel = 0.5;
    target.cnt = 0;
    target.joint_target.j1 = 0.1;
    target.joint_target.j2 = 0.1;
    target.joint_target.j3 = 0.1;
    target.joint_target.j4 = 0.1;
    target.joint_target.j5 = 0.1;
    target.joint_target.j6 = 0.1;

    arm.autoMove(10, target);

/*
    cout << "reset group" << endl;
    arm.resetGroup();
    sleep(1);
    arm.setManualFrame(JOINT);

    ManualDirection dir[9] = {STANDING};
    dir[1] = DECREASE;
    arm.setManualMode(CONTINUOUS);
    //arm.setManualMode(STEP);
    arm.manualMove(dir);

    while (loop++ < 20)
    {
        arm.sendPoint();
        usleep(5 * 1000);
    }

    dir[0] = DECREASE;
    arm.manualMove(dir);
    while (loop++ < 30)
    {
        arm.sendPoint();
        usleep(5 * 1000);
    }

    dir[0] = STANDING;
    dir[1] = STANDING;
    arm.manualMove(dir);
    while (loop++ < 200)
    {
        arm.sendPoint();
        usleep(5 * 1000);
    }

    sleep(1);
    */

    /*
    arm.setManualMode(APOINT);
    Joint target;
    target.j1 = 0;
    target.j2 = 0;
    target.j3 = 0;
    target.j4 = 0;
    target.j5 = 0;
    target.j6 = 0;
    arm.manualMove(target);
    while (loop++ < 2000)
    {
        arm.sendPoint();
        usleep(5 * 1000);
    }

    sleep(1);
    arm.stopGroup();
    */


    return 0;
}

