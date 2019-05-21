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
#include <parameter_manager/parameter_manager_param_group.h>
#include <transformation.h>


using namespace std;
using namespace fst_mc;
using namespace fst_log;
using namespace fst_base;
using namespace fst_parameter;
using namespace fst_algorithm;
using namespace basic_alg;


void test0(void)
{
    KinematicsRTM kinematics("/root/install/share/runtime/axis_group/");
    ErrorCode err = SUCCESS;
    Joint reference, res_joint;
    Transformation transformation;
    transformation.init(&kinematics);
    PoseQuaternion tcp_in_user, tcp_in_base, fcp_in_base;
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
    
    char buffer[LOG_TEXT_SIZE];
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
    Logger log;
    ArmGroup arm(&log);
    ErrorMonitor error_monitor;
    arm.initGroup(&error_monitor, NULL, NULL);

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

    arm.initGroup(&error_monitor, NULL, NULL);
    g_thread_running = true;
    rt_thread.run(&rtTask, &arm, 80);
    nrt_thread.run(&nrtTask, &arm, 78);
    sleep(1);
    arm.setGlobalVelRatio(0.5);
    arm.setGlobalAccRatio(0.8);
    arm.setManualFrame(JOINT);
    double steps[9] = {0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0, 0, 0};
    arm.setManualStepAxis(steps);
    arm.resetGroup();
    usleep(100 * 1000);

    //ManualDirection dirs[9] = {STANDING, STANDING, INCREASE, STANDING, STANDING, STANDING, STANDING, STANDING, STANDING};
    //arm.manualMoveStep(dirs);
    //Joint target = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    Joint target;
    double data[] = {0.1, 0.3, 1.05, -0.5, -1.0, -0.2, 0, 0, 0};
    memcpy(&target, data, sizeof(target));
    //arm.manualMoveToPoint(target);
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

void test5(void)
{
    PathCache path_cache;
    MotionInfo via, target;
    PoseEuler start_pose;

    start_pose.point_.x_ = 261.7164;
    start_pose.point_.y_ = -133.4895;
    start_pose.point_.z_ = 533.6744;
    start_pose.euler_.a_ = -0.6651;
    start_pose.euler_.b_ = -0.1726;
    start_pose.euler_.c_ = 2.7489;

    target.type = MOTION_LINE;
    target.cnt = 0.25;
    target.vel = 1200;
    target.target.pose.pose.point_.x_ = 565.3530;
    target.target.pose.pose.point_.y_ = -84.3829;
    target.target.pose.pose.point_.z_ = 489.7908;
    target.target.pose.pose.euler_.a_ = -0.0007;
    target.target.pose.pose.euler_.b_ = 0.0053;
    target.target.pose.pose.euler_.c_ = 3.1414;

    via.type = MOTION_LINE;
    via.cnt = 0.6;
    via.vel = 1000;
    via.target.pose.pose.point_.x_ = 204.4835;
    via.target.pose.pose.point_.y_ = -290.8357;
    via.target.pose.pose.point_.z_ = 467.5943;
    via.target.pose.pose.euler_.a_ = -0.9580;
    via.target.pose.pose.euler_.b_ = -0.0338;
    via.target.pose.pose.euler_.c_ = 3.1416;

    ErrorCode err = planPathSmoothLine(start_pose, via, target, path_cache);
    printf("planPathSmoothLine return %llx", err);

    for (size_t i = 0; i < path_cache.cache_length; i++)
    {
        PoseQuaternion &pose = path_cache.cache[i].pose;
        printf("%.6f,%.6f,%.6f,  %.6f,%.6f,%.6f,%.6f\n", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.quaternion_.w_, pose.quaternion_.x_, pose.quaternion_.y_, pose.quaternion_.z_);
    }
}

void test6(void)
{
    PoseEuler p;
    Joint ref, res;
    Logger log;
    ArmGroup arm(&log);
    ErrorMonitor error_monitor;
    ThreadHelp rt_thread;

    arm.initGroup(&error_monitor, NULL, NULL);

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
        ErrorCode err = arm.getKinematicsPtr()->doIK(p, ref, res) ? SUCCESS : MC_COMPUTE_IK_FAIL;
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


extern ComplexAxisGroupModel model;

void test7(void)
{
    /*
    KinematicsRTM kinematics;
    DynamicsInterface dynamics;
    double dh_matrix[9][4] = {{0, 0, 365, 0}, {PI/2, 30, 0, PI/2}, {0, 340, 0, 0}, {PI/2, 35, 350, 0}, {-PI/2, 0, 0, 0}, {PI/2, 0, 96.5, 0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}};
    kinematics.initKinematics(dh_matrix);
    SegmentAlgParam seg_param;
    ParamGroup param;

    if (param.loadParamFile("/root/install/share/runtime/component_param/segment_alg.yaml"))
    {
        if (param.getParam("accuracy_cartesian_factor", seg_param.accuracy_cartesian_factor) &&
            param.getParam("accuracy_joint_factor", seg_param.accuracy_joint_factor) &&
            param.getParam("max_traj_points_num", seg_param.max_traj_points_num) &&
            param.getParam("path_interval", seg_param.path_interval) &&
            param.getParam("joint_interval", seg_param.joint_interval) &&
            param.getParam("angle_interval", seg_param.angle_interval) &&
            param.getParam("angle_valve", seg_param.angle_valve) &&
            param.getParam("conservative_acc", seg_param.conservative_acc) &&
            param.getParam("time_factor_first", seg_param.time_factor_first) &&
            param.getParam("time_factor_last", seg_param.time_factor_last) &&
            param.getParam("is_fake_dynamics", seg_param.is_fake_dynamics))
        {}
        else
        {
            printf("Fail to load segment algorithm config, code = 0x%llx\n", param.getLastError());
            return;
        }
    }
    else
    {
        printf("Fail to load segment algorithm config, code = 0x%llx\n", param.getLastError());
        return;
    }

    initComplexAxisGroupModel();
    initSegmentAlgParam(&seg_param);

    Joint start_joint;
    double data[] = {0.234, 1.234, -2.044, 0.876, -1.2432, 0.003, 0, 0, 0};
    memcpy(&start_joint, data, sizeof(start_joint));
    MotionTarget target;
    target.joint_target.j1_ = -1.34;
    target.joint_target.j2_ = -0.23;
    target.joint_target.j3_ = 1.455;
    target.joint_target.j4_ = 0.034;
    target.joint_target.j5_ = 0.567;
    target.joint_target.j6_ = -1.4556;
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
    */

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

using namespace basic_alg;
using namespace std;

void test9(void)
{
    PoseEuler pose = {600, 100, 500, 0, 0, 3.14159};
    PoseEuler pose_res;
    Joint ref = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    Joint res;
    KinematicsRTM kinematics("/root/install/share/runtime/axis_group/");
    
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

    kinematics.doFK(ref, pose_res);
    cout << "kinematics FK:" << pose_res.point_.x_ << "," << pose_res.point_.y_ << "," << pose_res.point_.z_ << "," << pose_res.euler_.a_ << "," << pose_res.euler_.b_ << "," << pose_res.euler_.c_ << endl;
    kinematics.doIK(pose, ref, res);
    cout << "kinematics IK:" << res[0] << "," << res[1] << "," << res[2] << "," << res[3] << "," << res[4] << "," << res[5] << endl;
    kinematics.doFK(res, pose_res);
    cout << "kinematics FK:" << pose_res.point_.x_ << "," << pose_res.point_.y_ << "," << pose_res.point_.z_ << "," << pose_res.euler_.a_ << "," << pose_res.euler_.b_ << "," << pose_res.euler_.c_ << endl;
    
}

void test10(void)
{
    KinematicsRTM kinematics("/root/install/share/runtime/axis_group/");
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
    
    printf("tcp_in_base: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", tcp_in_base.point_.x_, tcp_in_base.point_.y_, tcp_in_base.point_.z_, tcp_in_base.euler_.a_, tcp_in_base.euler_.b_, tcp_in_base.euler_.c_);
    printf("fcp_in_base: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", fcp_in_base.point_.x_, fcp_in_base.point_.y_, fcp_in_base.point_.z_, fcp_in_base.euler_.a_, fcp_in_base.euler_.b_, fcp_in_base.euler_.c_);
    printf("tool_frame: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", tool_frame.point_.x_, tool_frame.point_.y_, tool_frame.point_.z_, tool_frame.euler_.a_, tool_frame.euler_.b_, tool_frame.euler_.c_);
    printf("joint: %.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", joint.j1_, joint.j2_, joint.j3_, joint.j4_, joint.j5_, joint.j6_);
    
    
}

int main(int argc, char **argv)
{
    //test0();
    //test1();
    //test2();
    //test3();
    //test4();
    //test5();
    //test6();
    //test7();
    //test8();
    //test9();
    test10();

    return 0;
}

