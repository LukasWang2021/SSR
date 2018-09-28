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
    arm.setGlobalVelRatio(1);
    arm.resetGroup();

    MotionTarget target;
    target.type = MOTION_JOINT;
    target.vel = 0.5;
    target.cnt = 0;
    target.joint_target.j1 = 0.0;
    target.joint_target.j2 = 0.0;
    target.joint_target.j3 = 0.0;
    target.joint_target.j4 = 0.0;
    target.joint_target.j5 = -1.5708;
    target.joint_target.j6 = 0.0;
    arm.autoMove(10, target);

    while (!arm.nextMovePermitted())
    {
        cout << "not permitted" << endl;
        usleep(10000);
    }

    cout << "get permitted" << endl;

    target.type = MOTION_JOINT;
    target.vel = 0.5;
    target.cnt = 0;
    target.joint_target.j1 = 0.8;
    target.joint_target.j2 = 0.5;
    target.joint_target.j3 = -0.4;
    target.joint_target.j4 = 1.1;
    target.joint_target.j5 = -0.8;
    target.joint_target.j6 = -2.0;
    arm.autoMove(10, target);

    while (!arm.nextMovePermitted())
    {
        cout << "not permitted" << endl;
        usleep(10000);
    }

    /*
    target.type = MOTION_LINE;
    target.vel = 600;
    target.cnt = 0;
    target.pose_target.position.x = 500;
    target.pose_target.position.y = 0;
    target.pose_target.position.z = 500;
    target.pose_target.orientation.a = 0;
    target.pose_target.orientation.b = 0;
    target.pose_target.orientation.c = 3.1416;
    arm.autoMove(10, target);
     */

    sleep(10);
}

void test3(void)
{
    int i = 0;

    //double J0[6] = { 0.09,0.09,0.09,0.09,0.09,0.09 };
    //double J1[6] = { 0.1,0.1,0.1,0.1,0.1,0.1 };

    //double dq_avg[6];
    //double ddq_max[6] = {
    //	124.238852,
    //	27.405693,
    //	16.749410,
    //	115.950513,
    //	341.373255,
    //	678.835059 };

    //for (int i = 0; i < 6; i++)
    //{
    //	dq_avg[i] = (J1[i] - J0[i]) / 0.055059;
    //}

    double dq0[6] = { 0.18162186509309483,0.18162186509309483,0.18162186509309483,0.18162186509309483,0.112891,0.18162186509309483 };
    double ddq0[6] = {0,0,0,0,0,0 };
    double Ti[6] = { -1,-1,-1,-1,-1,-1 };
    double Ta[6][4] = { 0 };
    double coeff[6][4][4] = { 0 };
    //double maxT = 0;
    //double tqf[6] = { 0 };
    //double dqf[6] = { 0 };
    //double ddqf[6] = { 0 };
    //double jerk[6] = { 5.0*0.5 / 1.3 * pow(10, 4) * 81,
    //3.3*0.4 / 0.44 * pow(10, 4) * 101,
    //3.3*0.4 / 0.44 * pow(10, 4) * 81,
    //1.7*0.39 / 0.18 * pow(10, 4) * 60,
    //1.7*0.25 / 0.17 * pow(10, 4) * 66.66667,
    //1.7*0.25 / 0.17 * pow(10, 4) * 44.64286 };

    //int ret = calculateParam(J0, dq0, ddq0, J1, dq_avg, ddq_max, jerk, Ti, 2, Ta, coeff, &maxT, tqf, dqf, ddqf);

    //printf("maxT=%f\n", maxT);

    //for (int i = 0; i < 6; i++)
    //	Ti[i] = maxT;
    //ret = calculateParam(J0, dq0, ddq0, J1, dq_avg, ddq_max, jerk, Ti, 2, Ta, coeff, &maxT, tqf, dqf, ddqf);

    //for (int i = 0; i < 6; i++)
    //{
    //printf("joint:%d Ta=%f %f %f %f\n", i, Ta[i][0], Ta[i][1], Ta[i][2], Ta[i][3]);
    //printf("coeff:%f %f %f %f -- %f %f %f %f -- %f %f %f %f -- %f %f %f %f\n", coeff[i][0][0], coeff[i][0][1], coeff[i][0][2], coeff[i][0][3], coeff[i][1][0], coeff[i][1][1], coeff[i][1][2], coeff[i][1][3], coeff[i][2][0], coeff[i][2][1], coeff[i][2][2], coeff[i][2][3], coeff[i][3][0], coeff[i][3][1], coeff[i][3][2], coeff[i][3][3]);
    //printf("each joint time:%f\n", maxT);
    //printf("dest status(q,dq,ddq):%f %f %f\n", tqf[i], dqf[i], ddqf[i]);

    //}

    //double J0[6] = { 0.01,0.01,0.01,0.01,0.01,0.01};
    //double J1[6] = { 0,0,0,0,0,0};
    double J0[6] = { 0.02,0.02,0.02,0.02,0.02,0.02};
    double J1[6] = { 0.01,0.01,0.01,0.01,0.01,0.01 };
    /*double J1[6] = { 0.09,0.09,0.09,0.09,0.09,0.09};
    double J0[6] = { 0.08,0.08,0.08,0.08,0.08,0.08};*/
    double ddq_max[6] = {
            124.238852,
            27.405693,
            16.749410,
            115.950513,
            341.373255,
            678.835059};
    double ddq_min[6] = {
            -123.615654,
            -22.630127,
            -50.360374,
            -140.650880,
            -340.782976,
            -678.798393
    };
    double jmax[6] = {
            1557692.30769231,
            3030000,
            2430000,
            2210000,
            1666666.75,
            1116071.5
    };
    Joint start;
    JointPoint target;
    Joint au, al,jk;
    TrajSegment segment[9];

    au.j1 = ddq_max[0]; au.j2 = ddq_max[1]; au.j3 = ddq_max[2]; au.j4 = ddq_max[3]; au.j5 = ddq_max[4]; au.j6 = ddq_max[5];
    al.j1 = ddq_min[0]; al.j2 = ddq_min[1]; al.j3 = ddq_min[2]; al.j4 = ddq_min[3]; al.j5 = ddq_min[4]; al.j6 = ddq_min[5];
    start.j1 = J0[0]; start.j2 = J0[1]; start.j3 = J0[2]; start.j4 = J0[3]; start.j5 = J0[4]; start.j6 = J0[5];
    target.angle.j1 = J1[0]; target.angle.j2 = J1[1]; target.angle.j3 = J1[2]; target.angle.j4 = J1[3]; target.angle.j5 = J1[4]; target.angle.j6 = J1[5];
    target.omega.j1 = dq0[0]; target.omega.j2 = dq0[1]; target.omega.j3 = dq0[2]; target.omega.j4 = dq0[3]; target.omega.j5 = dq0[4]; target.omega.j6 = dq0[5];
    target.alpha.j1 = ddq0[0]; target.alpha.j2 = ddq0[1]; target.alpha.j3 = ddq0[2]; target.alpha.j4 = ddq0[3]; target.alpha.j5 = ddq0[4]; target.alpha.j6 = ddq0[5];
    jk.j1 = jmax[0]; jk.j2 = jmax[1]; jk.j3 = jmax[2]; jk.j4 = jmax[3]; jk.j5 = jmax[4]; jk.j6 = jmax[5];

    forwardCycle(target, start, 0.05505945, au, al, jk, segment);
    //end = GetTickCount();
    //backwardCycle(start, target, 0.0551, au, al, jk, segment);
    for (int i = 0; i < 6; i++)
    {
        printf("start-angle: %f\n", J1[i]);
        printf("ending-angle:%f\n", J0[i]);
        printf("ending-omega:%f\n", dq0[i]);
        printf("ending-alpha:%f\n", ddq0[i]);
        printf("duration = %f %f %f %f\n", segment[i].duration[0], segment[i].duration[1], segment[i].duration[2], segment[i].duration[3]);
        printf("coeff=%f %f %f %f\n", segment[i].coeff[0][0], segment[i].coeff[0][1], segment[i].coeff[0][2], segment[i].coeff[0][3]);
        printf("coeff=%f %f %f %f\n", segment[i].coeff[1][0], segment[i].coeff[1][1], segment[i].coeff[1][2], segment[i].coeff[1][3]);
        printf("coeff=%f %f %f %f\n", segment[i].coeff[2][0], segment[i].coeff[2][1], segment[i].coeff[2][2], segment[i].coeff[2][3]);
        printf("coeff=%f %f %f %f\n", segment[i].coeff[3][0], segment[i].coeff[3][1], segment[i].coeff[3][2], segment[i].coeff[3][3]);
        printf("\n");
    }
    //printf("duration = %f %f %f %f\n", Ta[0][0], Ta[0][1], Ta[0][2], Ta[0][3]);
    //printf("coeff=%f %f %f %f\n", coeff[0][0][0], coeff[0][0][1], coeff[0][0][2], coeff[0][0][3]);
    //printf("coeff=%f %f %f %f\n", coeff[0][1][0], coeff[0][1][1], coeff[0][1][2], coeff[0][1][3]);
    //printf("coeff=%f %f %f %f\n", coeff[0][2][0], coeff[0][2][1], coeff[0][2][2], coeff[0][2][3]);
    //printf("coeff=%f %f %f %f\n", coeff[0][3][0], coeff[0][3][1], coeff[0][3][2], coeff[0][3][3]);
    return 0;
}

void test4(void)
{
    Logger log;
    ArmGroup arm(&log);
    ErrorMonitor error_monitor;
    ThreadHelp rt_thread;
    cout << "begin" << endl;

    arm.initGroup(&error_monitor);

    PoseEuler pose1, pose2;
    Joint joint1, joint2;

    joint1.j1 = 0.00006;
    joint1.j2 = 0.59525;
    joint1.j3 = -0.426034;
    joint1.j4 = -0.000017;
    joint1.j5 = -1.216373;
    joint1.j6 = -0.000254;

    joint2.j1 = 0.000027;
    joint2.j2 = -0.032293;
    joint2.j3 = 0.032703;
    joint2.j4 = 0.0;
    joint2.j5 = -1.047568;
    joint2.j6 = -0.000288;

    arm.getKinematicsPtr()->forwardKinematicsInUser(joint1, pose1);
    arm.getKinematicsPtr()->forwardKinematicsInUser(joint2, pose2);

    printf("Pose1: %.6f %.6f %.6f - %.6f %.6f %.6f\n", pose1.position.x, pose1.position.y, pose1.position.z, pose1.orientation.a, pose1.orientation.b, pose1.orientation.c);
    printf("Pose2: %.6f %.6f %.6f - %.6f %.6f %.6f\n", pose2.position.x, pose2.position.y, pose2.position.z, pose2.orientation.a, pose2.orientation.b, pose2.orientation.c);
}

int main(int argc, char **argv)
{
    //test1();
    test2();
    //test3();
    //test4();

    return 0;
}

