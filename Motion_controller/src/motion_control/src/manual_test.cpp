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
#include <time.h>
#include <motion_control_cache_pool.h>
#include <segment_alg.h>
#include <fstream>


using namespace std;
using namespace fst_mc;
using namespace fst_log;
using namespace fst_base;
using namespace fst_algorithm;



void test0(void)
{
    Joint angle;
    Joint omega;
    Joint alpha_upper, alpha_lower;
    clock_t start, end;
    DynamicsProduct product;
    DynamicsInterface dynamics;
    float alpha[2][6];

    angle[0] = 0.9;
    angle[1] = 0.4;
    angle[2] = -0.2;
    angle[3] = 1.1;
    angle[4] = -1.4;
    angle[5] = 0.3;
    omega[0] = 2.5;
    omega[1] = 1.3;
    omega[2] = -3.4;
    omega[3] = -1.2;
    omega[4] = 0.4;
    omega[5] = -0.9;

    float jnt[6] = {0, 0, 0, 1.1, 0, 0.3};
    float omg[6] = {0, 0, 0, 0, 0, 0};

    start = clock();
    for (size_t i = 0; i < 22; i++)
        dynamics.computeAccMax(jnt , omg, alpha);
    end = clock();
    double seconds  =(double)(end - start)/CLOCKS_PER_SEC;
    printf("dynamics using time: %.6f ms\n", seconds * 1000);

/*
    printf("result = %llx\n", err);
    printf("angle = %.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n", angle[0], angle[1], angle[2], angle[3], angle[4], angle[5]);
    printf("omega = %.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n", omega[0], omega[1], omega[2], omega[3], omega[4], omega[5]);
    printf("alpha-upper = %.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n", alpha_upper[0], alpha_upper[1], alpha_upper[2], alpha_upper[3], alpha_upper[4], alpha_upper[5]);
    printf("alpha-lower = %.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n", alpha_lower[0], alpha_lower[1], alpha_lower[2], alpha_lower[3], alpha_lower[4], alpha_lower[5]);
    printf("produce:\n");
    printf("  m = %.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n", product.m[0][0], product.m[0][1], product.m[0][2], product.m[0][3], product.m[0][4], product.m[0][5]);
    printf("      %.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n", product.m[1][0], product.m[1][1], product.m[1][2], product.m[1][3], product.m[1][4], product.m[1][5]);
    printf("      %.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n", product.m[2][0], product.m[2][1], product.m[2][2], product.m[2][3], product.m[2][4], product.m[2][5]);
    printf("      %.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n", product.m[3][0], product.m[3][1], product.m[3][2], product.m[3][3], product.m[3][4], product.m[3][5]);
    printf("      %.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n", product.m[4][0], product.m[4][1], product.m[4][2], product.m[4][3], product.m[4][4], product.m[4][5]);
    printf("      %.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n", product.m[5][0], product.m[5][1], product.m[5][2], product.m[5][3], product.m[5][4], product.m[5][5]);
    printf("  c = %.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n", product.c[0][0], product.c[0][1], product.c[0][2], product.c[0][3], product.c[0][4], product.c[0][5]);
    printf("      %.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n", product.c[1][0], product.c[1][1], product.c[1][2], product.c[1][3], product.c[1][4], product.c[1][5]);
    printf("      %.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n", product.c[2][0], product.c[2][1], product.c[2][2], product.c[2][3], product.c[2][4], product.c[2][5]);
    printf("      %.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n", product.c[3][0], product.c[3][1], product.c[3][2], product.c[3][3], product.c[3][4], product.c[3][5]);
    printf("      %.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n", product.c[4][0], product.c[4][1], product.c[4][2], product.c[4][3], product.c[4][4], product.c[4][5]);
    printf("      %.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n", product.c[5][0], product.c[5][1], product.c[5][2], product.c[5][3], product.c[5][4], product.c[5][5]);
    printf("  g = %.12f, %.12f, %.12f, %.12f, %.12f, %.12f\n", product.g[0], product.g[1], product.g[2], product.g[3], product.g[4], product.g[5]);
    */
}


void test1(void)
{
    Logger log;
    ArmGroup arm(&log);
    ErrorMonitor error_monitor;
    arm.initGroup(&error_monitor);

    //Joint joint = {PI / 2, PI / 4, PI / 8, PI / 16, PI / 16, PI / 16, 0, 0, 0};
    Joint joint = {0, 0, 0, 0, 0, PI / 2, 0, 0, 0};
    Joint res;
    PoseEuler pose;

    arm.getKinematicsPtr()->forwardKinematicsInBase(joint, pose);
    arm.getKinematicsPtr()->inverseKinematicsInBase(pose, joint, res);

    printf("forward kinematics:\n");
    printf("joint input = %.9f, %.9f, %.9f, %.9f, %.9f, %.9f\n", joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);
    printf("pose output = %.9f, %.9f, %.9f, %.9f, %.9f, %.9f\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    //printf("inverse kinematics:\n");
    //printf("pose input = %.9f, %.9f, %.9f, %.9f, %.9f, %.9f\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    //printf("joint output = %.9f, %.9f, %.9f, %.9f, %.9f, %.9f\n", res[0], res[1], res[2], res[3], res[4], res[5]);
}

static bool g_thread_running = false;

static void rtTask(void *group)
{
    cout << "---------------rt thread running---------------" << endl;
    while (g_thread_running)
    {
        ((BaseGroup*)group)->doPriorityLoop();
        usleep(2 * 1000);
    }
    cout << "---------------rt thread quit---------------" << endl;
}

static void nrtTask(void *group)
{
    cout << "---------------nrt thread running---------------" << endl;
    while (g_thread_running)
    {
        ((BaseGroup*)group)->doCommonLoop();
        usleep(2 * 1000);
    }
    cout << "---------------nrt thread quit---------------" << endl;
}

void test2(void)
{
    Logger log;
    ArmGroup arm(&log);
    ErrorMonitor error_monitor;
    ThreadHelp rt_thread, nrt_thread;
    cout << "begin" << endl;

    arm.initGroup(&error_monitor);
    g_thread_running = true;
    rt_thread.run(&rtTask, &arm, 80);
    nrt_thread.run(&nrtTask, &arm, 78);
    sleep(1);
    arm.setGlobalVelRatio(0.5);
    arm.setGlobalAccRatio(0.8);
    arm.setManualFrame(JOINT);
    arm.setManualStepAxis(0.15);
    arm.resetGroup();
    usleep(100 * 1000);

    //ManualDirection dirs[9] = {STANDING, STANDING, INCREASE, STANDING, STANDING, STANDING, STANDING, STANDING, STANDING};
    //arm.manualMoveStep(dirs);
    //Joint target = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    Joint target = {0.1, 0.3, 1.05, -0.5, -1.0, -0.2, 0, 0, 0};
    arm.manualMoveToPoint(target);
    usleep(100 * 1000);

    while (arm.getGroupState() != STANDBY)
    {
        cout << "Group-state = " << arm.getGroupState() << endl;
        usleep(100 * 1000);
    }

    arm.stopGroup();
    sleep(1);

    g_thread_running = false;
    rt_thread.join();
    nrt_thread.join();
    sleep(1);
}

void test3(void)
{
    Logger log;
    ArmGroup arm(&log);
    ErrorMonitor error_monitor;
    ThreadHelp rt_thread, nrt_thread;
    cout << "begin" << endl;

    arm.initGroup(&error_monitor);
    g_thread_running = true;
    rt_thread.run(&rtTask, &arm, 80);
    nrt_thread.run(&nrtTask, &arm, 78);
    arm.setGlobalVelRatio(0.5);
    arm.setGlobalAccRatio(0.8);
    usleep(500 * 1000);
    arm.resetGroup();
    usleep(100 * 1000);

    MotionTarget target;
    target.type = MOTION_JOINT;
    target.vel = 0.5;
    target.cnt = -1;
    target.joint_target.j1 = 0.0;
    target.joint_target.j2 = 0.0;
    target.joint_target.j3 = 0.0;
    target.joint_target.j4 = 0.0;
    //target.joint_target.j5 = -1.5708;
    target.joint_target.j5 = 0.0;
    target.joint_target.j6 = 0.0;
    arm.autoMove(10, target);

    /*
    while (!arm.nextMovePermitted())
    {
        cout << "not permitted" << endl;
        usleep(100 * 1000);
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
    arm.autoMove(11, target);

    while (!arm.nextMovePermitted())
    {
        cout << "not permitted" << endl;
        usleep(100 * 1000);
    }
    */

    sleep(5);
}

void test4(void)
{
    Logger log;
    ArmGroup arm(&log);
    ErrorMonitor error_monitor;
    arm.initGroup(&error_monitor);
    ofstream  kout("kinematics.csv");

    Pose pose;
    Joint inp, res;

    kout << "input.j1,input.j2,input.j3,input.j4,input.j5,input.j6,pose.x,pose.y,pose.z,pose.ow,pose.ox,pose.oy,pose.oz," 
         << "output.j1,output.j2,output.j3,output.j4,output.j5,output.j6,pose.x,pose.y,pose.z,pose.ow,pose.ox,pose.oy,pose.oz" << endl;

    for (double j1 = -PI + 0.01; j1 < PI - 0.01; j1 += PI / 3)
    {
        inp.j1 = j1;

        for (double j2 = -PI + 0.01; j2 < PI - 0.01; j2 += PI / 3)
        {
            inp.j2 = j2;

            for (double j3 = -PI + 0.01; j3 < PI - 0.01; j3 += PI / 3)
            {
                inp.j3 = j3;

                for (double j4 = -PI + 0.01; j4 < PI - 0.01; j4 += PI / 3)
                {
                    inp.j4 = j4;

                    for (double j5 = -PI + 0.01; j5 < PI - 0.01; j5 += PI / 3)
                    {
                        inp.j5 = j5;

                        for (double j6 = -PI + 0.01; j6 < PI - 0.01; j6 += PI / 3)
                        {
                            inp.j6 = j6;

                            arm.getKinematicsPtr()->forwardKinematicsInUser(inp, pose);
                            arm.getKinematicsPtr()->inverseKinematicsInUser(pose, inp, res);
                            
                            kout << inp.j1 << "," << inp.j2 << "," << inp.j3 << "," << inp.j4 << "," << inp.j5 << "," << inp.j6 << ","
                                 << pose.position.x << "," << pose.position.y << "," << pose.position.z << "," << pose.orientation.w << "," << pose.orientation.x << "," << pose.orientation.y << "," << pose.orientation.z << ",";

                            arm.getKinematicsPtr()->forwardKinematicsInUser(res, pose);
                            kout << res.j1 << "," << res.j2 << "," << res.j3 << "," << res.j4 << "," << res.j5 << "," << res.j6 << ","
                                 << pose.position.x << "," << pose.position.y << "," << pose.position.z << "," << pose.orientation.w << "," << pose.orientation.x << "," << pose.orientation.y << "," << pose.orientation.z << endl;
                        }
                    }
                }
            }
        }
    }


    cout << "kinematics.csv" << endl;
    kout.close();
}

void test6(void)
{
    Pose p;
    Joint ref, res;
    Logger log;
    ArmGroup arm(&log);
    ErrorMonitor error_monitor;
    ThreadHelp rt_thread;

    arm.initGroup(&error_monitor);

    p.position.x = 430.493206;
    p.position.y = 81.351958;
    p.position.z = 564.339605;
    p.orientation.w = -0.001571;
    p.orientation.x = 0.999334;
    p.orientation.y = 0.014975;
    p.orientation.z = 0.024502;

    ref.j1 = 0.185517603768;
    ref.j2 = -0.158492925047;
    ref.j3 = -0.050531790903;
    ref.j4 = -0.005467348039;
    ref.j5 = -1.312221853982;
    ref.j6 = 0.156383300619;

    clock_t start = clock();
    for (size_t i = 0; i < 1000; i++)
        ErrorCode err = arm.getKinematicsPtr()->inverseKinematicsInUser(p, ref, res);
    clock_t end = clock();

    double seconds  =(double)(end - start)/CLOCKS_PER_SEC;
    printf("kinematics using time: %.6f ms\n", seconds * 1000);

    /*
    if (err != SUCCESS)
    {
        log.error("IK err: %x", err);
    }
    log.info("Pose: %.6f,%.6f,%.6f - %.6f,%.6f,%.6f,%.6f", p.position.x, p.position.y, p.position.z, p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    log.info("Ref: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", ref[0], ref[1], ref[2], ref[3], ref[4], ref[5]);
    log.info("Res: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f", res[0], res[1], res[2], res[3], res[4], res[5]);
    */
}


extern ComplexAxisGroupModel model;

void test7(void)
{
    ArmKinematics kinematics;
    DynamicsInterface dynamics;
    double dh_matrix[9][4] = {{0, 0, 365, 0}, {PI/2, 30, 0, PI/2}, {0, 340, 0, 0}, {PI/2, 35, 350, 0}, {-PI/2, 0, 0, 0}, {PI/2, 0, 96.5, 0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}};
    kinematics.initKinematics(dh_matrix);
    initComplexAxisGroupModel();
    initSegmentAlgParam(&kinematics, &dynamics);
    initStack(&model);   

    Joint start_joint = {0.234, 1.234, -2.044, 0.876, -1.2432, 0.003};
    MotionTarget target;
    target.joint_target.j1 = -1.34;
    target.joint_target.j2 = -0.23;
    target.joint_target.j3 = 1.455;
    target.joint_target.j4 = 0.034;
    target.joint_target.j5 = 0.567;
    target.joint_target.j6 = -1.4556;
    target.cnt = 0;
    target.vel = 1.0;
    target.type = MOTION_JOINT;
    PathCache path_cache;
    TrajectoryCache traj_cache;
    JointState start_state;
    start_state.angle = start_joint;
    memset(&start_state.omega, 0, sizeof(start_state.omega));
    memset(&start_state.alpha, 0, sizeof(start_state.alpha));
    clock_t t1 = clock();
    ErrorCode err = planPathJoint(start_joint, target, path_cache);
    clock_t t2 = clock();
    err = planTrajectory(path_cache, start_state, 1, 1, traj_cache);
    clock_t t3 = clock();
    double delta_path  =(double)(t2 - t1)/CLOCKS_PER_SEC;
    double delta_traj  =(double)(t3 - t2)/CLOCKS_PER_SEC;

    std::cout<<"delta_path = "<<delta_path<<" delta_traj = "<<delta_traj<<std::endl;

/*
    printTraj(traj_cache, 0, 0.01);
    printTraj(traj_cache, 1, 0.01);
    printTraj(traj_cache, 2, 0.01);
    printTraj(traj_cache, 3, 0.01);
    printTraj(traj_cache, 4, 0.01);
    printTraj(traj_cache, 5, 0.01);
    */
}

void test8(void)
{
    CachePool<TrajectoryCacheList> traj_cache;
    CachePool<PathCacheList> path_cache;
    traj_cache.initCachePool(4);
    path_cache.initCachePool(4);

    TrajectoryCacheList *p1 = traj_cache.getCachePtr();
    traj_cache.freeCachePtr(p1);
}

int main(int argc, char **argv)
{
    //test0();
    //test1();
    //test2();
    //test3();
    test4();
    //test5();
    //test6();
    //test7();
    //test8();

    return 0;
}

