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
#include <trajectory_alg.h>
#include <time.h>


using namespace std;
using namespace fst_mc;
using namespace fst_log;
using namespace fst_base;


static void rtTask(void *group)
{
    ((BaseGroup*)group)->realtimeTask();
}

#define LOOP 1

void test1(void)
{
    JointPoint start_state[LOOP];
    JointPoint target_state[LOOP];
    Joint alpha_upper, alpha_lower;
    Joint jerk;
    TrajSegment segment[LOOP][9];

    for (size_t i = 0; i < LOOP; i++)
    {
        start_state[i].angle[0] = 0.09;
        start_state[i].angle[1] = 0.09;
        start_state[i].angle[2] = 0.09;
        start_state[i].angle[3] = 0.09;
        start_state[i].angle[4] = 0.09;
        start_state[i].angle[5] = 0.09;
        start_state[i].omega[0] = 0;
        start_state[i].omega[1] = 0;
        start_state[i].omega[2] = 0;
        start_state[i].omega[3] = 0;
        start_state[i].omega[4] = 0;
        start_state[i].omega[5] = 0;
        start_state[i].alpha[0] = 0;
        start_state[i].alpha[1] = 0;
        start_state[i].alpha[2] = 0;
        start_state[i].alpha[3] = 0;
        start_state[i].alpha[4] = 0;
        start_state[i].alpha[5] = 0;

        target_state[i].angle[0] = 0.1;
        target_state[i].angle[1] = 0.1;
        target_state[i].angle[2] = 0.1;
        target_state[i].angle[3] = 0.1;
        target_state[i].angle[4] = 0.1;
        target_state[i].angle[5] = 0.1;
        target_state[i].omega[0] = 0;
        target_state[i].omega[1] = 0;
        target_state[i].omega[2] = 0;
        target_state[i].omega[3] = 0;
        target_state[i].omega[4] = 0;
        target_state[i].omega[5] = 0;
        target_state[i].alpha[0] = 0;
        target_state[i].alpha[1] = 0;
        target_state[i].alpha[2] = 0;
        target_state[i].alpha[3] = 0;
        target_state[i].alpha[4] = 0;
        target_state[i].alpha[5] = 0;
    }

    alpha_upper[0] = 124.238852;
    alpha_upper[1] = 27.405693;
    alpha_upper[2] = 16.749410;
    alpha_upper[3] = 115.950513;
    alpha_upper[4] = 341.373255;
    alpha_upper[5] = 678.835059;
    alpha_lower[0] = -123.615654;
    alpha_lower[1] = -22.630127;
    alpha_lower[2] = -50.360374;
    alpha_lower[3] = -140.650880;
    alpha_lower[4] = -340.782976;
    alpha_lower[5] = -678.798393;

    jerk[0] = 1557692.307692;
    jerk[1] = 3030000.000000;
    jerk[2] = 2430000.000000;
    jerk[3] = 2210000.000000;
    jerk[4] = 1666666.750000;
    jerk[5] = 1116071.500000;

    clock_t start, end;

    start = clock();
    for (size_t i = 0; i < LOOP; i++)
        backwardCycle(start_state[i].angle, target_state[i], 0.001, alpha_upper, alpha_lower, jerk, segment[i]);
    end = clock();
    double seconds  =(double)(end - start)/CLOCKS_PER_SEC;
    printf("backCycle %d times, using time: %.6f ms\n", LOOP, seconds * 1000);

    start = clock();
    for (size_t i = 0; i < LOOP; i++)
        forwardCycle(start_state[i], target_state[i].angle, 0.001, alpha_upper, alpha_lower, jerk, segment[i]);
    end = clock();
    seconds  =(double)(end - start)/CLOCKS_PER_SEC;
    printf("foreCycle %d times, using time: %.6f ms\n", LOOP, seconds * 1000);
}

void test2(void)
{
    Logger log;
    ArmGroup arm(&log);
    ErrorMonitor error_monitor;
    ThreadHelp rt_thread;
    cout << "begin" << endl;

    arm.initGroup(&error_monitor);
    arm.activeRealtimeTask();
    rt_thread.run(&rtTask, &arm, 80);
    sleep(1);
    arm.resetGroup();

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

    sleep(10);
}

int main(int argc, char **argv)
{
    //test1();
    test2();

    return 0;
}

