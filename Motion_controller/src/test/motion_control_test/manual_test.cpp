/*************************************************************************
	> File Name: manual_test.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月08日 星期三 17时27分48秒
 ************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <string.h>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> 

#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include <basic_alg.h>
#include <motion_control_arm_group.h>
#include "thread_help.h"
#include <transformation.h>


using namespace std;
using namespace group_space;
using namespace log_space;
using namespace base_space;
using namespace basic_alg;

#ifndef JOINT_NUM
#define JOINT_NUM 6
#endif
#define TS_POINT_NUM 10
typedef struct 
{
    unsigned int sec;
    unsigned int nsec;
}Time;
typedef struct 
{
    double positions[JOINT_NUM];
    double velocities[JOINT_NUM];
    double accelerations[JOINT_NUM];
    double effort[JOINT_NUM];
    unsigned int valid_level;
    Time time_from_start;
}TrajectoryPoints;

typedef struct
{
    TrajectoryPoints points[TS_POINT_NUM];
    Time stamp;
    unsigned int total_points;
    unsigned int seq;  
    unsigned int last_fragment;
}TrajectorySeg;


void test0(void)
{
    KinematicsRTM kinematics("/root/install/share/runtime/axis_group/");
    ErrorCode err = SUCCESS;
    Joint reference, res_joint;
    Transformation transformation;
    transformation.init(&kinematics);
    PoseQuaternion tcp_in_user, fcp_in_base;
    PoseEuler user_frame_, tool_frame_;
    tcp_in_user.point_.x_ = 250;
    tcp_in_user.point_.y_ = 342;
    tcp_in_user.point_.z_ = 310;
    tcp_in_user.quaternion_.w_ = 0.000046;
    tcp_in_user.quaternion_.x_ = 1;
    tcp_in_user.quaternion_.y_ = 0;
    tcp_in_user.quaternion_.z_ = 0;
    memset(&user_frame_, 0, sizeof(user_frame_));
    memset(&tool_frame_, 0, sizeof(tool_frame_));
    memset(&reference, 0, sizeof(Joint));
    clock_t start_clock = clock();
    Posture posture = {1, 1, 1, 0};

    for (size_t i = 0; i < 1000; i++)
    {
        //transformation.convertPoseFromUserToBase(tcp_in_user, user_frame_, tcp_in_base);
        //transformation.convertTcpToFcp(tcp_in_base, tool_frame_, fcp_in_base);
        err = kinematics.doIK(fcp_in_base, posture, res_joint) ? SUCCESS : MC_COMPUTE_IK_FAIL;
    }
    
    clock_t end_clock = clock();
    double used_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;
    printf("used time: %.6f\n", used_time);
    
    //char buffer[LOG_TEXT_SIZE];
    PoseQuaternion &pose = tcp_in_user;
    PoseEuler tcp_user = Pose2PoseEuler(pose);
    PoseEuler fcp_base = Pose2PoseEuler(fcp_in_base);
    printf("  pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f, %.6f\n", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.quaternion_.w_, pose.quaternion_.x_, pose.quaternion_.y_, pose.quaternion_.z_);
    printf("  user-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f\n", user_frame_.point_.x_, user_frame_.point_.y_, user_frame_.point_.z_, user_frame_.euler_.a_, user_frame_.euler_.b_, user_frame_.euler_.c_);
    printf("  tool-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f\n", tool_frame_.point_.x_, tool_frame_.point_.y_, tool_frame_.point_.z_, tool_frame_.euler_.a_, tool_frame_.euler_.b_, tool_frame_.euler_.c_);
    printf("  tcp-in-user: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f\n", tcp_user.point_.x_, tcp_user.point_.y_, tcp_user.point_.z_, tcp_user.euler_.a_, tcp_user.euler_.b_, tcp_user.euler_.c_);
    printf("  fcp-in-base: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f\n", fcp_base.point_.x_, fcp_base.point_.y_, fcp_base.point_.z_, fcp_base.euler_.a_, fcp_base.euler_.b_, fcp_base.euler_.c_);
    //printf("  reference: %s\n", printDBLine(&reference[0], buffer, LOG_TEXT_SIZE));

}


void test1(void)
{
    ArmGroup arm;
    arm.initGroup(NULL, NULL, NULL,NULL, NULL);

    double data[] = {0, 0, 0, 0, 0, PI / 2, 0, 0, 0};
    //Joint joint = {PI / 2, PI / 4, PI / 8, PI / 16, PI / 16, PI / 16, 0, 0, 0};
    Joint joint;
    memcpy(&joint, data, sizeof(joint));
    Joint res;
    PoseEuler pose;

    arm.getKinematicsPtr()->doFK(joint, pose);
    arm.getKinematicsPtr()->doIK(pose, joint, res);

    printf("forward kinematics:\n");
    printf("joint input = %.9f, %.9f, %.9f, %.9f, %.9f, %.9f\n", joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);
    printf("pose output = %.9f, %.9f, %.9f, %.9f, %.9f, %.9f\n", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
    //printf("inverse kinematics:\n");
    //printf("pose input = %.9f, %.9f, %.9f, %.9f, %.9f, %.9f\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    //printf("joint output = %.9f, %.9f, %.9f, %.9f, %.9f, %.9f\n", res[0], res[1], res[2], res[3], res[4], res[5]);
}

void test3(void)
{
    KinematicsRTM kinematics("/root/install/share/runtime/axis_group/");
    Posture posture;
    posture.arm = 1;
    posture.elbow = 1;
    posture.wrist = 1;
    posture.flip = 0;

    PoseEuler p1;
    p1.point_.x_ = 452.9034;
    p1.point_.y_ = 20.5927;
    p1.point_.z_ = 482.9139;
    p1.euler_.a_ = 1.6152;
    p1.euler_.b_ = 0.0001;
    p1.euler_.c_ = -3.0414;

    PoseEuler p2;
    p2.point_.x_ = 452.9034;
    p2.point_.y_ = 20.5122;
    p2.point_.z_ = 482.9139;
    p2.euler_.a_ = 1.6152;
    p2.euler_.b_ = 0.0001;
    p2.euler_.c_ = -3.0414;

    Joint j1, j2;
    kinematics.doIK(p1, posture, j1);
    kinematics.doIK(p2, posture, j2);

    j1.print("j1:");
    j2.print("j2:");
}

static bool g_thread_running = false;

static void* rtTask(void *group)
{
    cout << "---------------rt thread running---------------" << endl;
    while (g_thread_running)
    {
        ((BaseGroup*)group)->doPriorityLoop();
        usleep(2 * 1000);
    }
    cout << "---------------rt thread quit---------------" << endl;
    return NULL;
}

static void* nrtTask(void *group)
{
    cout << "---------------nrt thread running---------------" << endl;
    while (g_thread_running)
    {
        ((BaseGroup*)group)->doCommonLoop();
        usleep(2 * 1000);
    }
    cout << "---------------nrt thread quit---------------" << endl;
    return NULL;
}

void test2(void)
{
    ArmGroup arm;
    ThreadHelp rt_thread, nrt_thread;
    cout << "begin" << endl;

    arm.initGroup(NULL, NULL,NULL,NULL, NULL);
    g_thread_running = true;
    rt_thread.run(rtTask, &arm, 80);
    nrt_thread.run(nrtTask, &arm, 78);
    sleep(1);
    arm.setGlobalVelRatio(0.5);
    arm.setGlobalAccRatio(0.8);
    arm.setManualFrame(JOINT);
    double steps[9] = {0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0, 0, 0};
    arm.setManualStepAxis(steps);
    //arm.resetGroup();
    usleep(100 * 1000);

    //ManualDirection dirs[9] = {STANDING, STANDING, INCREASE, STANDING, STANDING, STANDING, STANDING, STANDING, STANDING};
    //arm.manualMoveStep(dirs);
    //Joint target = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    Joint target;
    double data[] = {0.1, 0.3, 1.05, -0.5, -1.0, -0.2, 0, 0, 0};
    memcpy(&target, data, sizeof(target));
    //arm.manualMoveToPoint(target);
    usleep(100 * 1000);

    while (arm.getMotionControlState() != STANDBY)
    {
        cout << "Group-state = " << arm.getMotionControlState() << endl;
        usleep(100 * 1000);
    }

    arm.stopGroup();
    sleep(1);

    g_thread_running = false;
    rt_thread.join();
    nrt_thread.join();
    sleep(1);
}
/*
void test3(void)
{
    Logger log;
    ArmGroup arm(&log);
    ErrorMonitor error_monitor;
    ThreadHelp rt_thread, nrt_thread;
    cout << "begin" << endl;

    arm.initGroup(&error_monitor, NULL, NULL);
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
    target.joint_target.j1_ = 0.0;
    target.joint_target.j2_ = 0.0;
    target.joint_target.j3_ = 0.0;
    target.joint_target.j4_ = 0.0;
    //target.joint_target.j5 = -1.5708;
    target.joint_target.j5_ = 0.0;
    target.joint_target.j6_ = 0.0;
    //arm.autoMove(10, target);

    
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
    

    sleep(5);
}
*/

void test4(void)
{
    KinematicsRTM kinematics("/root/install/share/runtime/axis_group/");
    Transformation transformation;
    transformation.init(&kinematics);
    Joint joint_start = {0.2, -0.1, 0.15, 0, -1.47, 0};
    PoseEuler pose_fcp_by_base, pose_by_tool, pose_by_base, tool_frame;
    kinematics.doFK(joint_start, pose_fcp_by_base);
    memset(&pose_by_tool, 0, sizeof(pose_by_tool));
    tool_frame.point_.x_ = 10;
    tool_frame.point_.y_ = 10;
    tool_frame.point_.z_ = 10;
    tool_frame.euler_.a_ = 0;
    tool_frame.euler_.b_ = 0;
    tool_frame.euler_.c_ = 0;

    transformation.convertPoseFromToolToBase(pose_by_tool, pose_fcp_by_base, pose_by_base);
    printf("joint: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", joint_start.j1_, joint_start.j2_, joint_start.j3_, joint_start.j4_, joint_start.j5_, joint_start.j6_);
    printf("pose_fcp_by_base: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", pose_fcp_by_base.point_.x_, pose_fcp_by_base.point_.y_, pose_fcp_by_base.point_.z_, pose_fcp_by_base.euler_.a_, pose_fcp_by_base.euler_.b_, pose_fcp_by_base.euler_.c_);
    printf("pose_by_tool: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", pose_by_tool.point_.x_, pose_by_tool.point_.y_, pose_by_tool.point_.z_, pose_by_tool.euler_.a_, pose_by_tool.euler_.b_, pose_by_tool.euler_.c_);
    printf("tool_frame: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", tool_frame.point_.x_, tool_frame.point_.y_, tool_frame.point_.z_, tool_frame.euler_.a_, tool_frame.euler_.b_, tool_frame.euler_.c_);
    printf("pose_by_base: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", pose_by_base.point_.x_, pose_by_base.point_.y_, pose_by_base.point_.z_, pose_by_base.euler_.a_, pose_by_base.euler_.b_, pose_by_base.euler_.c_);

    PoseQuaternion pose, tcp_in_base, fcp_in_base;
    pose.point_.x_ = 650;
    pose.point_.y_ = 10;
    pose.point_.z_ = 500;
    pose.quaternion_.w_ = 0;
    pose.quaternion_.x_ = 1;
    pose.quaternion_.y_ = 0;
    pose.quaternion_.z_ = 0;
    clock_t start = clock();

    for (size_t i = 0; i < 1000; i++)
    {
        transformation.convertPoseFromUserToBase(pose, tool_frame, tcp_in_base);
        transformation.convertTcpToFcp(tcp_in_base, tool_frame, fcp_in_base);
    }

    clock_t ending = clock();
    double seconds  = (double)(ending - start)/CLOCKS_PER_SEC;
    printf("kinematics using time: %.6f ms\n", seconds * 1000);

    
}

/*
void test4(void)
{
    Logger log;
    ArmGroup arm(&log);
    ErrorMonitor error_monitor;
    arm.initGroup(&error_monitor, NULL, NULL);
    ofstream  kout("kinematics.csv");

    PoseQuaternion pose;
    Joint inp, res;

    kout << "input.j1,input.j2,input.j3,input.j4,input.j5,input.j6,pose.x,pose.y,pose.z,pose.ow,pose.ox,pose.oy,pose.oz," 
         << "output.j1,output.j2,output.j3,output.j4,output.j5,output.j6,pose.x,pose.y,pose.z,pose.ow,pose.ox,pose.oy,pose.oz" << endl;

    for (double j1 = -PI + 0.01; j1 < PI - 0.01; j1 += PI / 3)
    {
        inp.j1_ = j1;

        for (double j2 = -PI + 0.01; j2 < PI - 0.01; j2 += PI / 3)
        {
            inp.j2_ = j2;

            for (double j3 = -PI + 0.01; j3 < PI - 0.01; j3 += PI / 3)
            {
                inp.j3_ = j3;

                for (double j4 = -PI + 0.01; j4 < PI - 0.01; j4 += PI / 3)
                {
                    inp.j4_ = j4;

                    for (double j5 = -PI + 0.01; j5 < PI - 0.01; j5 += PI / 3)
                    {
                        inp.j5_ = j5;

                        for (double j6 = -PI + 0.01; j6 < PI - 0.01; j6 += PI / 3)
                        {
                            inp.j6_ = j6;

                            arm.getKinematicsPtr()->doFK(inp, pose);
                            arm.getKinematicsPtr()->doIK(pose, inp, res);
                            
                            kout << inp.j1_ << "," << inp.j2_ << "," << inp.j3_ << "," << inp.j4_ << "," << inp.j5_ << "," << inp.j6_ << ","
                                 << pose.point_.x_ << "," << pose.point_.y_ << "," << pose.point_.z_ << "," << pose.quaternion_.w_ << "," << pose.quaternion_.x_ << "," << pose.quaternion_.y_ << "," << pose.quaternion_.z_ << ",";

                            arm.getKinematicsPtr()->doFK(res, pose);
                            kout << res.j1_ << "," << res.j2_ << "," << res.j3_ << "," << res.j4_ << "," << res.j5_ << "," << res.j6_ << ","
                                 << pose.point_.x_ << "," << pose.point_.y_ << "," << pose.point_.z_ << "," << pose.quaternion_.w_ << "," << pose.quaternion_.x_ << "," << pose.quaternion_.y_ << "," << pose.quaternion_.z_ << endl;
                        }
                    }
                }
            }
        }
    }


    cout << "kinematics.csv" << endl;
    kout.close();
}
*/


void test6(void)
{
    PoseEuler p;
    Joint ref, res;
    ArmGroup arm;
    ThreadHelp rt_thread;

    arm.initGroup(NULL, NULL, NULL, NULL, NULL);

    p.point_.x_ = -29.04;
    p.point_.y_ = 167.96;
    p.point_.z_ = -465.96;
    p.euler_.a_ = -0.001571;
    p.euler_.b_ = 0.999334;
    p.euler_.c_ = 0.014975;

    ref.j1_ = -0.535158;
    ref.j2_ = -0.203063;
    ref.j3_ = -0.730588;
    ref.j4_ = -0.112341;
    ref.j5_ = 1.777998;
    ref.j6_ = 1.040318;


    arm.getKinematicsPtr()->doFK(ref, p);
    printf("%.4f, %.4f, %.4f - %.4f, %.4f, %.4f\n", p.point_.x_, p.point_.y_, p.point_.z_, p.euler_.a_, p.euler_.b_, p.euler_.c_);



    clock_t start = clock();
    for (size_t i = 0; i < 1000; i++)
        arm.getKinematicsPtr()->doIK(p, ref, res);
    clock_t end = clock();

    double seconds  = (double)(end - start)/CLOCKS_PER_SEC;
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





using namespace basic_alg;
using namespace std;

void test9(void)
{
    PoseEuler pose = {600, 100, 500, 0, 0, 3.14159};
    //PoseEuler pose_res;
    Joint ref = {0, 0, 0, 0, -1, 0, 0, 0, 0};
    //Joint res;
    static KinematicsRTM kinematics("/root/install/share/runtime/axis_group/");
    
    /*
    double dh_matrix[NUM_OF_JOINT][4] = {{0.0, 0.0, 365.0, 0.0}, 
                                        {1.5707963267948966192313216916398, 30, 0, 1.5707963267948966192313216916398},
                                        {0.0, 340.0, 0.0, 0.0},
                                        {1.5707963267948966192313216916398, 35.0, 350.0, 0.0},
                                        {-1.5707963267948966192313216916398, 0.0, 0.0, 0.0}
                                        {1.5707963267948966192313216916398, 0.0, 96.5, 0.0}};
    
    ArmKinematics kinematics2;
    kinematics2.initKinematics(dh_matrix);
    */
    /*
    kinematics.doFK(ref, pose_res);
    cout << "kinematics FK:" << pose_res.point_.x_ << "," << pose_res.point_.y_ << "," << pose_res.point_.z_ << "," << pose_res.euler_.a_ << "," << pose_res.euler_.b_ << "," << pose_res.euler_.c_ << endl;
    kinematics.doIK(pose, ref, res);
    cout << "kinematics IK:" << res[0] << "," << res[1] << "," << res[2] << "," << res[3] << "," << res[4] << "," << res[5] << endl;
    kinematics.doFK(res, pose_res);
    cout << "kinematics FK:" << pose_res.point_.x_ << "," << pose_res.point_.y_ << "," << pose_res.point_.z_ << "," << pose_res.euler_.a_ << "," << pose_res.euler_.b_ << "," << pose_res.euler_.c_ << endl;
    */

    Joint joint = {0.2345235, -0.62435254, 0.3254626, -1.3473245, -1.4542366, 2.542656737};
    Posture posture;
    clock_t start_clock = clock();
    for (size_t i = 0; i < 10; i++)
    {
        //transformation.convertPoseFromUserToBase(tcp_in_user, user_frame_, tcp_in_base);
        //transformation.convertTcpToFcp(tcp_in_base, tool_frame_, fcp_in_base);
        posture = kinematics.getPostureByJoint(joint);
    }
    clock_t end_clock = clock();
    double used_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;
    //printf("getPostureByJoint used time: %.6f\n", used_time);

    start_clock = clock();
    for (size_t i = 0; i < 10; i++)
    {
        kinematics.doIK(pose, ref, joint);
    }
    end_clock = clock();
    used_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;
    //printf("doIK used time: %.6f\n", used_time);
    //printf("doIK result: %.6f %.6f %.6f %.6f %.6f %.6f\n", joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);

    start_clock = clock();
    for (size_t i = 0; i < 10; i++)
    {
        kinematics.doIK(pose, posture, joint);
    }
    end_clock = clock();
    used_time = (double)(end_clock - start_clock) / CLOCKS_PER_SEC * 1000;
    //printf("doIK used time: %.6f\n", used_time);
    //printf("doIK result: %.6f %.6f %.6f %.6f %.6f %.6f\n", joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);
}

void test10(void)
{
    static KinematicsRTM kinematics("/root/install/share/runtime/axis_group/");
    Transformation transformation;
    transformation.init(&kinematics);

    PoseEuler fcp_in_base, tcp_in_base, pose, tool_frame, user_frame;
    Posture posture = {1, 1, -1, 0};
    Joint joint;
    
    pose.point_.x_ = 550;
    pose.point_.y_ = -150;
    pose.point_.z_ = 610;
    pose.euler_.a_ = 0;
    pose.euler_.b_ = 0;
    pose.euler_.c_ = 3.141593;

    tool_frame.point_.x_ = 0;
    tool_frame.point_.y_ = 0;
    tool_frame.point_.z_ = 0;
    tool_frame.euler_.a_ = 0;
    tool_frame.euler_.b_ = 0;
    tool_frame.euler_.c_ = 0;

    
    user_frame.point_.x_ = 0;
    user_frame.point_.y_ = 0;
    user_frame.point_.z_ = 0;
    user_frame.euler_.a_ = 0;
    user_frame.euler_.b_ = 0;
    user_frame.euler_.c_ = 0;

    transformation.convertPoseFromUserToBase(pose, user_frame, tcp_in_base);
    transformation.convertTcpToFcp(tcp_in_base, tool_frame, fcp_in_base);
    
    if (!kinematics.doIK(fcp_in_base, posture, joint))
    {
        printf("error!!\n");
        printf("tcp_in_base: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", tcp_in_base.point_.x_, tcp_in_base.point_.y_, tcp_in_base.point_.z_, tcp_in_base.euler_.a_, tcp_in_base.euler_.b_, tcp_in_base.euler_.c_);
        printf("fcp_in_base: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", fcp_in_base.point_.x_, fcp_in_base.point_.y_, fcp_in_base.point_.z_, fcp_in_base.euler_.a_, fcp_in_base.euler_.b_, fcp_in_base.euler_.c_);
        printf("tool_frame: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", tool_frame.point_.x_, tool_frame.point_.y_, tool_frame.point_.z_, tool_frame.euler_.a_, tool_frame.euler_.b_, tool_frame.euler_.c_);
    
    }
    
    //printf("tcp_in_base: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", tcp_in_base.point_.x_, tcp_in_base.point_.y_, tcp_in_base.point_.z_, tcp_in_base.euler_.a_, tcp_in_base.euler_.b_, tcp_in_base.euler_.c_);
    //printf("fcp_in_base: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", fcp_in_base.point_.x_, fcp_in_base.point_.y_, fcp_in_base.point_.z_, fcp_in_base.euler_.a_, fcp_in_base.euler_.b_, fcp_in_base.euler_.c_);
    //printf("tool_frame: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", tool_frame.point_.x_, tool_frame.point_.y_, tool_frame.point_.z_, tool_frame.euler_.a_, tool_frame.euler_.b_, tool_frame.euler_.c_);
    //printf("joint: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", joint.j1_, joint.j2_, joint.j3_, joint.j4_, joint.j5_, joint.j6_);
    
    
}


#define MAIN_PRIORITY 80
#define MAX_SAFE_STACK (1 * 1024 * 1024)    /* The maximum stack size which is
                                                guaranteed safe to access without
                                                faulting */

bool run_flag = true;


/*
static void* runTask1(void *null)
{
    printf("run task1\n");

    while (run_flag)
    {
        test9();
        usleep(2000);
    }

    printf("task1 quit\n");
    return NULL;
}

static void* runTask2(void *null)
{
    printf("run task2\n");

    while (run_flag)
    {
        test10();
        usleep(1000);
    }

    printf("task2 quit\n");
    return NULL;
}
*/

void test11(void)
{
    PoseEuler pose;
    //PoseEuler pose_res;
    //Joint ref = {0, 0, 0, 0, -1, 0, 0, 0, 0};
    //Joint res;
    KinematicsRTM kinematics("/root/install/share/runtime/axis_group/");
    
    //Joint joint = {-0.142136, -0.699177, -0.591108, 1.89109, 1.53765, -0.990637};
    Joint joint = {-0.142138, -0.689292, -0.590827, -1.632113, -1.561919, 1.878400};
    Posture posture = kinematics.getPostureByJoint(joint);
    printf("joint: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", joint.j1_, joint.j2_, joint.j3_, joint.j4_, joint.j5_, joint.j6_);
    //posture.wrist = -1;
    printf("posture: %d,%d,%d,%d\n", posture.arm, posture.elbow, posture.wrist, posture.flip);

    kinematics.doFK(joint, pose);
    printf("pose: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
    kinematics.doIK(pose, posture, joint);
    printf("IK result: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", joint.j1_, joint.j2_, joint.j3_, joint.j4_, joint.j5_, joint.j6_);
}

void test12(void)
{
    PoseEuler pose;
    pose.point_.x_ = 281.262;
    pose.point_.y_ = 167.878;
    pose.point_.z_ = 643.5;
    pose.euler_.a_ = 0;
    pose.euler_.b_ = 0;
    pose.euler_.c_ = -3.14159;
    Posture posture = {1, 1, -1, 0};
    //PoseEuler pose_res;
    //Joint ref = {0, 0, 0, 0, -1, 0, 0, 0, 0};
    Joint res;
    KinematicsRTM kinematics("/root/install/share/runtime/axis_group/");
    printf("pose: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
    printf("posture: %d,%d,%d,%d\n", posture.arm, posture.elbow, posture.wrist, posture.flip);
    printf("IK: %d\n", kinematics.doIK(pose, posture, res));
    printf("IK result: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", res.j1_, res.j2_, res.j3_, res.j4_, res.j5_, res.j6_);
}

void test13(void)
{
    Posture posture;
    PoseEuler pose;
    Joint joint_input, joint_reference, joint_output;
    KinematicsRTM kinematics("/root/install/share/runtime/axis_group/");

    while (true)
    {
        /*
        joint_reference.j1_ = 0.001 * ((double)rand() / RAND_MAX);
        joint_reference.j2_ = 0.001 * ((double)rand() / RAND_MAX);
        joint_reference.j3_ = 0.001 * ((double)rand() / RAND_MAX);
        joint_reference.j4_ = 0.001 * ((double)rand() / RAND_MAX);
        joint_reference.j5_ = 0.001 * ((double)rand() / RAND_MAX);
        joint_reference.j6_ = 0.001 * ((double)rand() / RAND_MAX);
        joint_reference.j7_ = 0;
        joint_reference.j8_ = 0;
        joint_reference.j9_ = 0;
        */
        joint_reference.j1_ = 0;
        joint_reference.j2_ = 0;
        joint_reference.j3_ = 0;
        joint_reference.j4_ = 0;
        joint_reference.j5_ = 0;
        joint_reference.j6_ = 0;
        joint_reference.j7_ = 0;
        joint_reference.j8_ = 0;
        joint_reference.j9_ = 0;

        joint_input.j1_ = -2.9 + 5.80 * ((double)rand() / RAND_MAX);
        joint_input.j2_ = -2.35 + 4.09 * ((double)rand() / RAND_MAX);
        joint_input.j3_ = -1.22 + 4.36 * ((double)rand() / RAND_MAX);
        joint_input.j4_ = -3.14 + 6.28 * ((double)rand() / RAND_MAX);
        joint_input.j5_ = -2.0 + 4.0 * ((double)rand() / RAND_MAX);
        joint_input.j6_ = -3.14 + 6.28 * ((double)rand() / RAND_MAX);
        joint_input.j7_ = 0;
        joint_input.j8_ = 0;
        joint_input.j9_ = 0;

        joint_reference += joint_input;

        kinematics.doFK(joint_input, pose);
        posture = kinematics.getPostureByJoint(joint_input);

        /*
        if (!kinematics.doIK(pose, posture, joint_output))
        {
            if (fabs(joint_input.j5_) < 0.001)
            {
                continue;
            }
            
            printf("ERROR: IK failed.\n");
            printf("  Joint input: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", joint_input.j1_, joint_input.j2_, joint_input.j3_, joint_input.j4_, joint_input.j5_, joint_input.j6_);
            printf("  Pose: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            printf("  Posture: %d,%d,%d,%d\n", posture.arm, posture.elbow, posture.wrist, posture.flip);
            continue;
        }

        joint_output.j7_ = 0;
        joint_output.j8_ = 0;
        joint_output.j9_ = 0;

        if (!joint_input.isEqual(joint_output))
        {
            printf("ERROR: output joint different with input\n");
            printf("  Pose: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            printf("  Posture: %d,%d,%d,%d\n", posture.arm, posture.elbow, posture.wrist, posture.flip);
            printf("  Joint input:  %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", joint_input.j1_, joint_input.j2_, joint_input.j3_, joint_input.j4_, joint_input.j5_, joint_input.j6_);
            printf("  Joint output: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", joint_output.j1_, joint_output.j2_, joint_output.j3_, joint_output.j4_, joint_output.j5_, joint_output.j6_);
            continue;
        }
        */
    

        if (!kinematics.doIK(pose, joint_reference, joint_output))
        {
            printf("ERROR: IK failed.\n");
            printf("  Joint input: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", joint_input.j1_, joint_input.j2_, joint_input.j3_, joint_input.j4_, joint_input.j5_, joint_input.j6_);
            printf("  Joint reference: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", joint_reference.j1_, joint_reference.j2_, joint_reference.j3_, joint_reference.j4_, joint_reference.j5_, joint_reference.j6_);
            printf("  Pose: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            continue;
        }

        joint_output.j7_ = 0;
        joint_output.j8_ = 0;
        joint_output.j9_ = 0;

        if (!joint_input.isEqual(joint_output))
        {
            printf("ERROR: output joint different with input\n");
            printf("  Pose: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            printf("  Joint reference: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", joint_reference.j1_, joint_reference.j2_, joint_reference.j3_, joint_reference.j4_, joint_reference.j5_, joint_reference.j6_);
            printf("  Joint input:  %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", joint_input.j1_, joint_input.j2_, joint_input.j3_, joint_input.j4_, joint_input.j5_, joint_input.j6_);
            printf("  Joint output: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", joint_output.j1_, joint_output.j2_, joint_output.j3_, joint_output.j4_, joint_output.j5_, joint_output.j6_);
            kinematics.doFK(joint_input, pose);
            printf("  Pose input: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            kinematics.doFK(joint_output, pose);
            printf("  Pose output: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
        }
    }
}

void test14(void)
{
    ThreeJerkDSCurvePlanner ds_planner_;
    double jerk[] = {20, 10, 5};
    ds_planner_.planDSCurve(0, 1, 0.5, 2, jerk, 1.0);
    ds_planner_.outputDSCurve(0.001, "ads.csv");
}

void test15(void)
{
    CirclePlanner planner;
    PoseQuaternion a,b,c;
    a.point_.x_ = -100;
    a.point_.y_ = 25;
    a.point_.z_ = 5;
    b.point_.x_ = 15;
    b.point_.y_ = 100;
    b.point_.z_ = 30;
    c.point_.x_ = 100;
    c.point_.y_ = 45;
    c.point_.z_ = 50;
    planner.planTrajectory(a,b,c,1,1,1,1);
}

struct TestItem
{
	double stamp;
	double q[6];
} ;

struct TestBuffer
{
	uint32_t index;
	TestItem items[10000];
} ;


int main(int argc, char **argv)
{
    /*
    fst_base::ThreadHelp thread1, thread2;

    if (thread1.run(runTask1, NULL, 80))
    
    thread1.join();
    thread2.join();
*/

    //test0();
    //test1();
    //test8();
    //test9();
    //test10();
    //test15();
    /*
    int fd = open("/devmem", O_RDWR);
    static TestBuffer* test_buffer = (TestBuffer *) mmap(NULL, 1000 * 1024, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x3B000000);
    if (test_buffer == MAP_FAILED) 
    {
        close(fd);
        printf("\nError in openMem(): failed on mapping core sharedmem\n");
        return -1;
    }
    
    ofstream jtac("jtac.csv");

    for (uint32_t i = 0; i < test_buffer->index; i++)
    {
        jtac << i << "," << test_buffer->items[i].stamp << "," << test_buffer->items[i].q[0] << "," << test_buffer->items[i].q[1] << "," << test_buffer->items[i].q[2] << "," << test_buffer->items[i].q[3] << "," << test_buffer->items[i].q[4] << "," << test_buffer->items[i].q[5] << endl;
    }
    
    jtac.close();
    */
    TrajectorySeg seg;
    cout << "size of TrajectorySeg: " << sizeof(TrajectorySeg) << endl;
    cout << "size of TrajectoryPoints: " << sizeof(TrajectoryPoints) << endl;
    cout << "offset of points[0]: " << (uint32_t)(&seg.points[0]) - (uint32_t)(&seg) << endl;
    cout << "offset of points[1]: " << (uint32_t)(&seg.points[1]) - (uint32_t)(&seg) << endl;
    cout << "offset of points[2]: " << (uint32_t)(&seg.points[2]) - (uint32_t)(&seg) << endl;
    cout << "offset of points[3]: " << (uint32_t)(&seg.points[3]) - (uint32_t)(&seg) << endl;
    cout << "offset of points[4]: " << (uint32_t)(&seg.points[4]) - (uint32_t)(&seg) << endl;
    cout << "offset of points[5]: " << (uint32_t)(&seg.points[5]) - (uint32_t)(&seg) << endl;
    cout << "offset of points[6]: " << (uint32_t)(&seg.points[6]) - (uint32_t)(&seg) << endl;
    cout << "offset of points[7]: " << (uint32_t)(&seg.points[7]) - (uint32_t)(&seg) << endl;
    cout << "offset of points[8]: " << (uint32_t)(&seg.points[8]) - (uint32_t)(&seg) << endl;
    cout << "offset of points[9]: " << (uint32_t)(&seg.points[9]) - (uint32_t)(&seg) << endl;
    cout << "offset of total_points: " << (uint32_t)(&seg.total_points) - (uint32_t)(&seg) << endl;
    cout << "offset of seq: " << (uint32_t)(&seg.seq) - (uint32_t)(&seg) << endl;
    cout << "offset of last_fragment: " << (uint32_t)(&seg.last_fragment) - (uint32_t)(&seg) << endl;

    ofstream  shm_out("/root/share_memory.dump");
    int fd = open("/dev/mem", O_RDWR);
    void *ptr = mmap(NULL, 524288, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x38100000);
    uint32_t *pdata = (uint32_t*)ptr;
    char buffer[1024];

    for (uint32_t i = 0; i < 524288; i += 16 * 4)
    {
        sprintf(buffer, "0x%08x: 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x 0x%08x", i,
            pdata[0], pdata[1], pdata[2], pdata[3], pdata[4], pdata[5], pdata[6], pdata[7], 
            pdata[8], pdata[9], pdata[10], pdata[11], pdata[12], pdata[13], pdata[14], pdata[15]);
        pdata += 16;
        shm_out << buffer << endl;
    }

    shm_out.flush();
    shm_out.close();

    return 0;
}




