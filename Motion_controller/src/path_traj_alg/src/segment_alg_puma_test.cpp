#include "segment_alg.h"
#include "kinematics.h"
#include "kinematics_rtm.h"
#include "kinematics_toll.h"
//#include "dynamics_interface.h"
#include "dynamic_alg_rtm.h"
#include "basic_alg_datatype.h"
#include "common_file_path.h"
#include <iostream>
#include <math.h>
#include <time.h>
#include <ctime>

using namespace fst_mc;
using namespace basic_alg;
//using namespace fst_algorithm;

extern double stack[20000];
extern ComplexAxisGroupModel model;
extern SegmentAlgParam segment_alg_param;

//extern DynamicAlgRTM dynamics;


int main(void)
{
    DynamicAlgRTM dynamics;
    dynamics.initDynamicAlg("/root/install/share/runtime/axis_group/");

    DH base_dh;

    DH arm_dh[6];

    base_dh.a = 0;
    base_dh.d = 365;
    base_dh.alpha = 0;
    base_dh.offset = 0;

    arm_dh[0].d = 0;    arm_dh[0].a = 30;   arm_dh[0].alpha = M_PI / 2;  arm_dh[0].offset = 0;
    arm_dh[1].d = 0;    arm_dh[1].a = 340;  arm_dh[1].alpha = 0;         arm_dh[1].offset = M_PI / 2;
    arm_dh[2].d = 0;    arm_dh[2].a = 35;   arm_dh[2].alpha = M_PI / 2;  arm_dh[2].offset = 0;
    arm_dh[3].d = 350;  arm_dh[3].a = 0;    arm_dh[3].alpha = -M_PI / 2; arm_dh[3].offset = 0;
    arm_dh[4].d = 0;    arm_dh[4].a = 0;    arm_dh[4].alpha = M_PI / 2;  arm_dh[4].offset = 0;
    arm_dh[5].d = 96.5; arm_dh[5].a = 0;    arm_dh[5].alpha = 0;         arm_dh[5].offset = 0;    
    
    basic_alg::Kinematics* kinematics_ptr = new basic_alg::KinematicsRTM(base_dh, arm_dh);
    if(!kinematics_ptr->isValid())
    {
        std::cout<<"kinematics init failed"<<std::endl;
    }

    segment_alg_param.accuracy_cartesian_factor = 3;
    segment_alg_param.accuracy_joint_factor = 6;
    segment_alg_param.max_traj_points_num = 20;
    segment_alg_param.path_interval = 1.0;
    segment_alg_param.joint_interval = 0.01745;//PI * 1 / 180;
    segment_alg_param.angle_interval = 0.01745;//PI * 1 / 180;
    segment_alg_param.angle_valve = 0.8727;//PI * 5 / 180;
    segment_alg_param.conservative_acc = 1000;
    segment_alg_param.time_factor_first = 2.5;
    segment_alg_param.time_factor_last = 3;
    segment_alg_param.is_fake_dynamics = true;
    segment_alg_param.max_cartesian_acc = 8000;
    segment_alg_param.kinematics_ptr = kinematics_ptr;
    segment_alg_param.dynamics_ptr = &dynamics;

    double vel_ratio = 0.5;
    double acc_ratio = 0.5;

    double joint_vel_max[9] = {5.8119, 5.66, 5.8119, 5.8540, 5.0686, 5.5592, 0, 0, 0};
    AxisType axis_type[9] = {ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS};
    initSegmentAlgParam(&segment_alg_param, 6, axis_type, joint_vel_max);

    Joint org;
    org.j1_ = 0;
    org.j2_ = 0;
    org.j3_ = 0;
    org.j4_ = 0;
    org.j5_ = -1.5708;
    org.j6_ = 0;
    Posture posture = kinematics_ptr->getPostureByJoint(org);

    PoseEuler p1, p2, p3, p4, p5;
    p1.point_.x_ = 300; p1.point_.y_ = 300; p1.point_.z_ = 400; p1.euler_.a_ = 0; p1.euler_.b_ = 0; p1.euler_.c_ = M_PI;
    p2.point_.x_ = 450; p2.point_.y_ = 150; p2.point_.z_ = 400; p2.euler_.a_ = 0; p2.euler_.b_ = 0; p2.euler_.c_ = M_PI;
    p3.point_.x_ = 300; p3.point_.y_ = 0;   p3.point_.z_ = 400; p3.euler_.a_ = 0; p3.euler_.b_ = 0; p3.euler_.c_ = M_PI;
    p4.point_.x_ = 450; p4.point_.y_ = -150;p4.point_.z_ = 400; p4.euler_.a_ = 0; p4.euler_.b_ = 0; p4.euler_.c_ = M_PI;
    p5.point_.x_ = 300; p5.point_.y_ = -300;p5.point_.z_ = 400; p5.euler_.a_ = 0; p5.euler_.b_ = 0; p5.euler_.c_ = M_PI;

/************** test movec and movec smooth *********/
#if 0
    #if 1
        #if 1
        MotionInfo c1_target;
        c1_target.via.pose.pose = p2;
        c1_target.target.pose.pose = p3;
        c1_target.type = MOTION_CIRCLE;
        c1_target.vel = 500;
        c1_target.cnt = 1;

        PathCache c1_path_cache;
        c1_path_cache.target.vel = c1_target.vel;
        c1_path_cache.target.type = c1_target.type;
        c1_path_cache.target.cnt = c1_target.cnt;
        planPathCircle(p1, c1_target, c1_path_cache);
        printf("c1_path_cache.smooth_out_index = %d\n", c1_path_cache.smooth_out_index);
        printf("c1_path_cache.cache_length = %d\n", c1_path_cache.cache_length);

        #endif
        #if 1
        //printPathResult(c1_path_cache);
        for(size_t i = 0; i < c1_path_cache.cache_length; ++i)
        {
            kinematics_ptr->doIK(c1_path_cache.cache[i].pose, posture, c1_path_cache.cache[i].joint);
            if (i == 133)
            {
                c1_path_cache.cache[i].joint.print("j_out:");
            }
        }

        c1_path_cache.target = c1_target;
        Joint j1;
        kinematics_ptr->doIK(p1, posture, j1);
        JointState p1_state;
        p1_state.angle = j1;
        p1_state.omega.j1_ = 0; p1_state.omega.j2_ = 0; p1_state.omega.j3_ = 0; 
        p1_state.omega.j4_ = 0; p1_state.omega.j5_ = 0; p1_state.omega.j6_ = 0; 
        p1_state.alpha.j1_ = 0; p1_state.alpha.j2_ = 0; p1_state.alpha.j3_ = 0; 
        p1_state.alpha.j4_ = 0; p1_state.alpha.j5_ = 0; p1_state.alpha.j6_ = 0;
        TrajectoryCache c1_traj_cache;
        planTrajectory(c1_path_cache, p1_state, vel_ratio, acc_ratio, c1_traj_cache);

        /*std::cout<<"c1_traj_cache.smooth_out_index = "<<c1_traj_cache.smooth_out_index<<std::endl
                 <<"c1_traj_cache.cache_length = "<<c1_traj_cache.cache_length<<std::endl;*/
        // printTraj(c1_traj_cache, 5, 0.001, c1_traj_cache.smooth_out_index + 1);

        //fkToTraj(c1_traj_cache, c1_target);
        #endif
    #endif
    /************ l2c ************/
    #if 0
        #if 1
        MotionInfo c1_target;
        c1_target.target.pose.pose = p3;
        c1_target.type = MOTION_LINE;
        c1_target.vel = 500;
        c1_target.cnt = 1;

        PathCache c1_path_cache;
        c1_path_cache.target.vel = c1_target.vel;
        c1_path_cache.target.type = c1_target.type;
        c1_path_cache.target.cnt = c1_target.cnt;
        planPathLine(p2, c1_target, c1_path_cache);
        printf("c1_path_cache.smooth_out_index = %d\n", c1_path_cache.smooth_out_index);
        printf("c1_path_cache.cache_length = %d\n", c1_path_cache.cache_length);

        #endif
        #if 1
        for(size_t i = 0; i < c1_path_cache.cache_length; ++i)
        {
            kinematics_ptr->doIK(c1_path_cache.cache[i].pose, posture, c1_path_cache.cache[i].joint);
        }

        c1_path_cache.target = c1_target;
        Joint j1;
        kinematics_ptr->doIK(p1, posture, j1);
        JointState p1_state;
        p1_state.angle = j1;
        p1_state.omega.j1_ = 0; p1_state.omega.j2_ = 0; p1_state.omega.j3_ = 0; 
        p1_state.omega.j4_ = 0; p1_state.omega.j5_ = 0; p1_state.omega.j6_ = 0; 
        p1_state.alpha.j1_ = 0; p1_state.alpha.j2_ = 0; p1_state.alpha.j3_ = 0; 
        p1_state.alpha.j4_ = 0; p1_state.alpha.j5_ = 0; p1_state.alpha.j6_ = 0;
        TrajectoryCache c1_traj_cache;
        planTrajectory(c1_path_cache, p1_state, vel_ratio, acc_ratio, c1_traj_cache);

        /*std::cout<<"c1_traj_cache.smooth_out_index = "<<c1_traj_cache.smooth_out_index<<std::endl
                 <<"c1_traj_cache.cache_length = "<<c1_traj_cache.cache_length<<std::endl;*/
        // printTraj(c1_traj_cache, 5, 0.001, c1_traj_cache.smooth_out_index + 1);

        //fkToTraj(c1_traj_cache, c1_target);
        #endif
    #endif

    /************ j2c ************/
    #if 0
        #if 1
        Joint j_start;
        kinematics_ptr->doIK(p1, posture, j_start);

        MotionInfo c1_target;
        c1_target.target.pose.pose = p3;
        kinematics_ptr->doIK(p3, posture, c1_target.target.joint);
        c1_target.type = MOTION_JOINT;
        c1_target.vel = 1;
        c1_target.cnt = 1;

        PathCache c1_path_cache;
        c1_path_cache.target.vel = c1_target.vel;
        c1_path_cache.target.type = c1_target.type;
        c1_path_cache.target.cnt = c1_target.cnt;

        planPathJoint(j_start, c1_target, c1_path_cache);
        printf("c1_path_cache.smooth_out_index = %d\n", c1_path_cache.smooth_out_index);
        printf("c1_path_cache.cache_length = %d\n", c1_path_cache.cache_length);

        #endif
        #if 1
        for(size_t i = 0; i < c1_path_cache.cache_length; ++i)
        {
            kinematics_ptr->doFK(c1_path_cache.cache[i].joint, c1_path_cache.cache[i].pose);
        }

        c1_path_cache.target = c1_target;
        // Joint j1;
        // kinematics_ptr->doIK(p1, posture, j1);
        JointState p1_state;
        p1_state.angle = j_start;
        p1_state.omega.j1_ = 0; p1_state.omega.j2_ = 0; p1_state.omega.j3_ = 0; 
        p1_state.omega.j4_ = 0; p1_state.omega.j5_ = 0; p1_state.omega.j6_ = 0; 
        p1_state.alpha.j1_ = 0; p1_state.alpha.j2_ = 0; p1_state.alpha.j3_ = 0; 
        p1_state.alpha.j4_ = 0; p1_state.alpha.j5_ = 0; p1_state.alpha.j6_ = 0;
        TrajectoryCache c1_traj_cache;
        planTrajectory(c1_path_cache, p1_state, vel_ratio, acc_ratio, c1_traj_cache);

        /*std::cout<<"c1_traj_cache.smooth_out_index = "<<c1_traj_cache.smooth_out_index<<std::endl
                 <<"c1_traj_cache.cache_length = "<<c1_traj_cache.cache_length<<std::endl;*/
        // printTraj(c1_traj_cache, 5, 0.001, c1_traj_cache.smooth_out_index + 1);

        //fkToTraj(c1_traj_cache);
        #endif
    #endif
    #if 1

    MotionInfo c2_target;
    c2_target.via.pose.pose = p4;
    c2_target.target.pose.pose = p5;
    c2_target.type = MOTION_CIRCLE;
    c2_target.vel = 200;
    c2_target.cnt = -1;

    PoseEuler p_out;
    c1_path_cache.cache[c1_path_cache.smooth_out_index].pose.convertToPoseEuler(p_out);
    PathCache c2_path_cache;
    planPathSmoothCircle(p_out, c1_target, c2_target, c2_path_cache);

    for(size_t i = 0; i < c2_path_cache.cache_length; ++i)
    {
        PoseEuler p_ik;;
        c2_path_cache.cache[i].pose.convertToPoseEuler(p_ik);
        kinematics_ptr->doIK(p_ik, posture, c2_path_cache.cache[i].joint);
    }

    c2_path_cache.target = c2_target;
    Joint j_out;
    kinematics_ptr->doIK(p_out, posture, j_out);
    j_out.print("j_out:");
    JointState out_state;
    out_state.angle = j_out;
    out_state.omega.j1_ = 0; out_state.omega.j2_ = 0; out_state.omega.j3_ = 0; 
    out_state.omega.j4_ = 0; out_state.omega.j5_ = 0; out_state.omega.j6_ = 0; 
    out_state.alpha.j1_ = 0; out_state.alpha.j2_ = 0; out_state.alpha.j3_ = 0; 
    out_state.alpha.j4_ = 0; out_state.alpha.j5_ = 0; out_state.alpha.j6_ = 0;    

    TrajectoryCache c2_traj_cache;
    planTrajectorySmooth(c2_path_cache, out_state, c1_target, vel_ratio, acc_ratio, c2_traj_cache);
    //printTraj(c2_traj_cache, 0, 0.001, c2_traj_cache.cache_length);
    fkToTraj(c2_traj_cache);
    //segment_alg_param.kinematics_ptr->doFK(start_state.angle, start);
    #endif
    #if 0
    for(int i = 0; i < c2_path_cache.cache_length; ++i)
    {
        std::cout<< " " <<c2_path_cache.cache[i].pose.point_.x_
            << " " <<c2_path_cache.cache[i].pose.point_.y_
            << " " <<c2_path_cache.cache[i].pose.point_.z_<<std::endl;
    }
    #endif
#endif


/************** test movel and movex2l smooth *********/
#if 0
    /************ l2l ************/
    #if 1
        #if 1
        PoseEuler start_test;
        start_test.point_.x_ = 250;
        start_test.point_.y_ = 150;
        start_test.point_.z_ = 310;
        start_test.euler_.a_ = 0;
        start_test.euler_.b_ = 0;
        start_test.euler_.c_ = M_PI;

        PoseEuler end_test;
        end_test.point_.x_ = 550;
        end_test.point_.y_ = -150;
        end_test.point_.z_ = 610;
        end_test.euler_.a_ = 0;
        end_test.euler_.b_ = 0;
        end_test.euler_.c_ = M_PI;

        MotionInfo l1_target;
        l1_target.target.pose.pose = end_test;
        l1_target.type = MOTION_LINE;
        l1_target.vel = 1600;
        l1_target.cnt = -1;

        PathCache l1_path_cache;
        l1_path_cache.target.vel = l1_target.vel;
        l1_path_cache.target.type = l1_target.type;
        l1_path_cache.target.cnt = l1_target.cnt;
        planPathLine(start_test, l1_target, l1_path_cache);
        printf("l1_path_cache.smooth_out_index = %d\n", l1_path_cache.smooth_out_index);
        printf("l1_path_cache.cache_length = %d\n", l1_path_cache.cache_length);

        #endif
        #if 1
        for(size_t i = 0; i < l1_path_cache.cache_length; ++i)
        {
            kinematics_ptr->doIK(l1_path_cache.cache[i].pose, posture, l1_path_cache.cache[i].joint);
        }

        l1_path_cache.target = l1_target;
        Joint j1;
        kinematics_ptr->doIK(start_test, posture, j1);
        JointState p1_state;
        p1_state.angle = j1;
        p1_state.omega.j1_ = 0; p1_state.omega.j2_ = 0; p1_state.omega.j3_ = 0; 
        p1_state.omega.j4_ = 0; p1_state.omega.j5_ = 0; p1_state.omega.j6_ = 0; 
        p1_state.alpha.j1_ = 0; p1_state.alpha.j2_ = 0; p1_state.alpha.j3_ = 0; 
        p1_state.alpha.j4_ = 0; p1_state.alpha.j5_ = 0; p1_state.alpha.j6_ = 0;
        TrajectoryCache l1_traj_cache;
        planTrajectory(l1_path_cache, p1_state, vel_ratio, acc_ratio, l1_traj_cache);

        /*std::cout<<"c1_traj_cache.smooth_out_index = "<<c1_traj_cache.smooth_out_index<<std::endl
                 <<"c1_traj_cache.cache_length = "<<c1_traj_cache.cache_length<<std::endl;*/
        printTraj(l1_traj_cache, 5, 0.001, l1_traj_cache.cache_length);

        //fkToTraj(l1_traj_cache);
        #endif
    #endif

    /************ j2l ************/
    #if 0
        #if 1
        Joint j_start;
        kinematics_ptr->doIK(p1, posture, j_start);
        MotionInfo l1_target;
        l1_target.target.pose.pose = p2;
        kinematics_ptr->doIK(l1_target.target.pose.pose, posture, l1_target.target.joint);
        l1_target.type = MOTION_JOINT;
        l1_target.vel = 1;
        l1_target.cnt = 0.5;

        PathCache l1_path_cache;
        l1_path_cache.target.vel = l1_target.vel;
        l1_path_cache.target.type = l1_target.type;
        l1_path_cache.target.cnt = l1_target.cnt;
        planPathJoint(j_start, l1_target, l1_path_cache);
        printf("l1_path_cache.smooth_out_index = %d\n", l1_path_cache.smooth_out_index);
        printf("l1_path_cache.cache_length = %d\n", l1_path_cache.cache_length);

        #endif
        #if 1
        for(size_t i = 0; i < l1_path_cache.cache_length; ++i)
        {
            kinematics_ptr->doFK(l1_path_cache.cache[i].joint, l1_path_cache.cache[i].pose);
        }

        l1_path_cache.target = l1_target;
        Joint j1;
        kinematics_ptr->doIK(p1, posture, j1);
        JointState p1_state;
        p1_state.angle = j1;
        p1_state.omega.j1_ = 0; p1_state.omega.j2_ = 0; p1_state.omega.j3_ = 0; 
        p1_state.omega.j4_ = 0; p1_state.omega.j5_ = 0; p1_state.omega.j6_ = 0; 
        p1_state.alpha.j1_ = 0; p1_state.alpha.j2_ = 0; p1_state.alpha.j3_ = 0; 
        p1_state.alpha.j4_ = 0; p1_state.alpha.j5_ = 0; p1_state.alpha.j6_ = 0;
        TrajectoryCache l1_traj_cache;
        planTrajectory(l1_path_cache, p1_state, vel_ratio, acc_ratio, l1_traj_cache);

        /*std::cout<<"c1_traj_cache.smooth_out_index = "<<c1_traj_cache.smooth_out_index<<std::endl
                 <<"c1_traj_cache.cache_length = "<<c1_traj_cache.cache_length<<std::endl;*/
        // printTraj(c1_traj_cache, 5, 0.001, c1_traj_cache.smooth_out_index + 1);

        //fkToTraj(c1_traj_cache, c1_target);
        #endif
    #endif

    /************ C2l ************/
    #if 0
        #if 1
        MotionInfo l1_target;
        l1_target.via.pose.pose = p2;
        l1_target.target.pose.pose = p3;
        l1_target.type = MOTION_CIRCLE;
        l1_target.vel = 500;
        l1_target.cnt = 1;

        PathCache l1_path_cache;
        l1_path_cache.target.vel = l1_target.vel;
        l1_path_cache.target.type = l1_target.type;
        l1_path_cache.target.cnt = l1_target.cnt;
        planPathCircle(p1, l1_target, l1_path_cache);
        printf("l1_path_cache.smooth_out_index = %d\n", l1_path_cache.smooth_out_index);
        printf("l1_path_cache.cache_length = %d\n", l1_path_cache.cache_length);

        #endif
        #if 1
        for(size_t i = 0; i < l1_path_cache.cache_length; ++i)
        {
            kinematics_ptr->doIK(l1_path_cache.cache[i].pose, posture, l1_path_cache.cache[i].joint);
        }
        // for(size_t i = 0; i < l1_path_cache.cache_length; ++i)
        // {
            // kinematics_ptr->doFK(l1_path_cache.cache[i].joint, l1_path_cache.cache[i].pose);
        // }

        l1_path_cache.target = l1_target;
        Joint j1;
        kinematics_ptr->doIK(p1, posture, j1);
        JointState p1_state;
        p1_state.angle = j1;
        p1_state.omega.j1_ = 0; p1_state.omega.j2_ = 0; p1_state.omega.j3_ = 0; 
        p1_state.omega.j4_ = 0; p1_state.omega.j5_ = 0; p1_state.omega.j6_ = 0; 
        p1_state.alpha.j1_ = 0; p1_state.alpha.j2_ = 0; p1_state.alpha.j3_ = 0; 
        p1_state.alpha.j4_ = 0; p1_state.alpha.j5_ = 0; p1_state.alpha.j6_ = 0;
        TrajectoryCache l1_traj_cache;
        planTrajectory(l1_path_cache, p1_state, vel_ratio, acc_ratio, l1_traj_cache);

        /*std::cout<<"c1_traj_cache.smooth_out_index = "<<c1_traj_cache.smooth_out_index<<std::endl
                 <<"c1_traj_cache.cache_length = "<<c1_traj_cache.cache_length<<std::endl;*/
        // printTraj(c1_traj_cache, 5, 0.001, c1_traj_cache.smooth_out_index + 1);

       // fkToTraj(l1_traj_cache, l1_target);
        #endif
    #endif

    #if 0
    MotionInfo l2_target;
    l2_target.target.pose.pose = p4;
    l2_target.type = MOTION_LINE;
    l2_target.vel = 500;
    l2_target.cnt = -1;

    PoseEuler p_out;
    l1_path_cache.cache[l1_path_cache.smooth_out_index].pose.convertToPoseEuler(p_out);
    PathCache l2_path_cache;
    planPathSmoothLine(p_out, l1_target, l2_target, l2_path_cache);

    for(size_t i = 0; i < l2_path_cache.cache_length; ++i)
    {
        kinematics_ptr->doIK(l2_path_cache.cache[i].pose, posture, l2_path_cache.cache[i].joint);
    }

    l2_path_cache.target = l2_target;
    Joint j_out;
    kinematics_ptr->doIK(p_out, posture, j_out);
    j_out.print("j_out:");
    JointState out_state;
    out_state.angle = j_out;
    out_state.omega.j1_ = 0; out_state.omega.j2_ = 0; out_state.omega.j3_ = 0; 
    out_state.omega.j4_ = 0; out_state.omega.j5_ = 0; out_state.omega.j6_ = 0; 
    out_state.alpha.j1_ = 0; out_state.alpha.j2_ = 0; out_state.alpha.j3_ = 0; 
    out_state.alpha.j4_ = 0; out_state.alpha.j5_ = 0; out_state.alpha.j6_ = 0;    
    TrajectoryCache l2_traj_cache;
    planTrajectorySmooth(l2_path_cache, out_state, l1_target, vel_ratio, acc_ratio, l2_traj_cache);
    //printTraj(c2_traj_cache, 0, 0.001, c2_traj_cache.cache_length);
    fkToTraj(l2_traj_cache);
    #endif
    #if 0
    for(int i = 0; i < l2_path_cache.cache_length; ++i)
    {
        PoseEuler l2_path_pose;
        segment_alg_param.kinematics_ptr->doFK(l2_path_cache.cache[i].joint, l2_path_pose);
        std::cout<< " " <<l2_path_pose.point_.x_
            << " " << l2_path_pose.point_.y_
            << " " << l2_path_pose.point_.z_<<std::endl;
    }
    #endif
#endif


/************** test movej and movex2j smooth *********/
#if 0
    /************ l2j ************/
    #if 0
        #if 1
        MotionTarget l1_target;
        l1_target.pose_target = p2;
        l1_target.tool_frame_id = 0;
        l1_target.user_frame_id = 0;
        l1_target.type = MOTION_LINE;
        l1_target.vel = 500;
        l1_target.cnt = 1;

        PathCache l1_path_cache;
        l1_path_cache.target.vel = l1_target.vel;
        l1_path_cache.target.type = l1_target.type;
        l1_path_cache.target.cnt = l1_target.cnt;
        planPathLine(p1, l1_target, l1_path_cache);
        printf("l1_path_cache.smooth_out_index = %d\n", l1_path_cache.smooth_out_index);
        printf("l1_path_cache.cache_length = %d\n", l1_path_cache.cache_length);

        #endif
        #if 1
        for(size_t i = 0; i < l1_path_cache.cache_length; ++i)
        {
            kinematics_ptr->doIK(l1_path_cache.cache[i].pose, posture, l1_path_cache.cache[i].joint);
            if (i == 133)
            {
                l1_path_cache.cache[i].joint.print("j_out:");
            }
        }

        l1_path_cache.target = l1_target;
        Joint j1;
        kinematics_ptr->doIK(p1, posture, j1);
        JointState p1_state;
        p1_state.angle = j1;
        p1_state.omega.j1_ = 0; p1_state.omega.j2_ = 0; p1_state.omega.j3_ = 0; 
        p1_state.omega.j4_ = 0; p1_state.omega.j5_ = 0; p1_state.omega.j6_ = 0; 
        p1_state.alpha.j1_ = 0; p1_state.alpha.j2_ = 0; p1_state.alpha.j3_ = 0; 
        p1_state.alpha.j4_ = 0; p1_state.alpha.j5_ = 0; p1_state.alpha.j6_ = 0;
        TrajectoryCache l1_traj_cache;
        planTrajectory(l1_path_cache, p1_state, vel_ratio, acc_ratio, l1_traj_cache);

        /*std::cout<<"c1_traj_cache.smooth_out_index = "<<c1_traj_cache.smooth_out_index<<std::endl
                 <<"c1_traj_cache.cache_length = "<<c1_traj_cache.cache_length<<std::endl;*/
        // printTraj(c1_traj_cache, 5, 0.001, c1_traj_cache.smooth_out_index + 1);

        //fkToTraj(c1_traj_cache, c1_target);
        #endif
    #endif

    /************ j2j ************/
    #if 0
        #if 1
        MotionTarget l1_target;
        kinematics_ptr->doIK(p2, posture, l1_target.joint_target);
        l1_target.tool_frame_id = 0;
        l1_target.user_frame_id = 0;
        l1_target.type = MOTION_JOINT;
        l1_target.vel = 1;
        l1_target.cnt = 1;

        PathCache l1_path_cache;
        l1_path_cache.target.vel = l1_target.vel;
        l1_path_cache.target.type = l1_target.type;
        l1_path_cache.target.cnt = l1_target.cnt;
        planPathLine(p1, l1_target, l1_path_cache);
        printf("l1_path_cache.smooth_out_index = %d\n", l1_path_cache.smooth_out_index);
        printf("l1_path_cache.cache_length = %d\n", l1_path_cache.cache_length);

        #endif
        #if 1
        for(size_t i = 0; i < l1_path_cache.cache_length; ++i)
        {
            kinematics_ptr->doFK(l1_path_cache.cache[i].joint, l1_path_cache.cache[i].pose);
        }

        l1_path_cache.target = l1_target;
        Joint j1;
        kinematics_ptr->doIK(p1, posture, j1);
        JointState p1_state;
        p1_state.angle = j1;
        p1_state.omega.j1_ = 0; p1_state.omega.j2_ = 0; p1_state.omega.j3_ = 0; 
        p1_state.omega.j4_ = 0; p1_state.omega.j5_ = 0; p1_state.omega.j6_ = 0; 
        p1_state.alpha.j1_ = 0; p1_state.alpha.j2_ = 0; p1_state.alpha.j3_ = 0; 
        p1_state.alpha.j4_ = 0; p1_state.alpha.j5_ = 0; p1_state.alpha.j6_ = 0;
        TrajectoryCache l1_traj_cache;
        planTrajectory(l1_path_cache, p1_state, vel_ratio, acc_ratio, l1_traj_cache);

        /*std::cout<<"c1_traj_cache.smooth_out_index = "<<c1_traj_cache.smooth_out_index<<std::endl
                 <<"c1_traj_cache.cache_length = "<<c1_traj_cache.cache_length<<std::endl;*/
        // printTraj(c1_traj_cache, 5, 0.001, c1_traj_cache.smooth_out_index + 1);

        //fkToTraj(c1_traj_cache, c1_target);
        #endif
    #endif

    /************ C2j ************/
    #if 1
        #if 1
        MotionInfo l1_target;
        l1_target.via.pose.pose = p2;
        l1_target.target.pose.pose = p3;
        l1_target.type = MOTION_CIRCLE;
        l1_target.vel = 600;
        l1_target.cnt = 1;

        PathCache l1_path_cache;
        l1_path_cache.target.vel = l1_target.vel;
        l1_path_cache.target.type = l1_target.type;
        l1_path_cache.target.cnt = l1_target.cnt;
        planPathCircle(p1, l1_target, l1_path_cache);
        printf("l1_path_cache.smooth_out_index = %d\n", l1_path_cache.smooth_out_index);
        printf("l1_path_cache.smooth_in_index = %d\n", l1_path_cache.smooth_in_index);

        #endif
        #if 1
        for(size_t i = 0; i < l1_path_cache.cache_length; ++i)
        {
            kinematics_ptr->doIK(l1_path_cache.cache[i].pose, posture, l1_path_cache.cache[i].joint);
        }

        l1_path_cache.target = l1_target;
        Joint j1;
        kinematics_ptr->doIK(p1, posture, j1);
        JointState p1_state;
        p1_state.angle = j1;
        p1_state.omega.j1_ = 0; p1_state.omega.j2_ = 0; p1_state.omega.j3_ = 0; 
        p1_state.omega.j4_ = 0; p1_state.omega.j5_ = 0; p1_state.omega.j6_ = 0; 
        p1_state.alpha.j1_ = 0; p1_state.alpha.j2_ = 0; p1_state.alpha.j3_ = 0; 
        p1_state.alpha.j4_ = 0; p1_state.alpha.j5_ = 0; p1_state.alpha.j6_ = 0;
        TrajectoryCache l1_traj_cache;
        planTrajectory(l1_path_cache, p1_state, vel_ratio, acc_ratio, l1_traj_cache);

        /*std::cout<<"c1_traj_cache.smooth_out_index = "<<c1_traj_cache.smooth_out_index<<std::endl
                 <<"c1_traj_cache.cache_length = "<<c1_traj_cache.cache_length<<std::endl;*/
        // printTraj(c1_traj_cache, 5, 0.001, c1_traj_cache.smooth_out_index + 1);

       // fkToTraj(l1_traj_cache, l1_target);
        #endif
    #endif

    #if 1
    MotionInfo l2_target;
    kinematics_ptr->doIK(p4, posture, l2_target.target.joint);
    //l2_target.pose_target = p4;
    l2_target.type = MOTION_JOINT;
    l2_target.vel = 2;
    l2_target.cnt = -1;

    // PoseEuler p_out;
    // l1_path_cache.cache[l1_path_cache.smooth_out_index].pose.convertToPoseEuler(p_out);
    Joint j_out = l1_path_cache.cache[l1_path_cache.smooth_out_index].joint;
    j_out.print("j_out:");
    PathCache l2_path_cache;
    planPathSmoothJoint(j_out, l1_target, l2_target, l2_path_cache);

    for(size_t i = 0; i < l2_path_cache.cache_length; ++i)
    {
        kinematics_ptr->doFK(l2_path_cache.cache[i].joint, l2_path_cache.cache[i].pose);
    }

    l2_path_cache.target = l2_target;
    // Joint j_out;
    // kinematics_ptr->doIK(p_out, posture, j_out);
    printf("l2_path_cache.cache[l2_path_cache.smooth_out_index].pose: x = %lf; y = %lf; z = %lf\n",
        l2_path_cache.cache[l2_path_cache.smooth_out_index].pose.point_.x_,
        l2_path_cache.cache[l2_path_cache.smooth_out_index].pose.point_.y_,
        l2_path_cache.cache[l2_path_cache.smooth_out_index].pose.point_.z_);

    JointState out_state;
    out_state.angle = j_out;
    out_state.omega.j1_ = 0; out_state.omega.j2_ = 0; out_state.omega.j3_ = 0; 
    out_state.omega.j4_ = 0; out_state.omega.j5_ = 0; out_state.omega.j6_ = 0; 
    out_state.alpha.j1_ = 0; out_state.alpha.j2_ = 0; out_state.alpha.j3_ = 0; 
    out_state.alpha.j4_ = 0; out_state.alpha.j5_ = 0; out_state.alpha.j6_ = 0;
    TrajectoryCache l2_traj_cache;
    planTrajectorySmooth(l2_path_cache, out_state, l1_target, vel_ratio, acc_ratio, l2_traj_cache);
    std::cout<<"l2_traj_cache.smooth_out_index = "<<l2_traj_cache.smooth_out_index<<std::endl
                <<"l2_traj_cache.cache_length = "<<l2_traj_cache.cache_length<<std::endl;
    //printTraj(c2_traj_cache, 0, 0.001, c2_traj_cache.cache_length);
    //fkToTraj(l2_traj_cache);
    #endif
    #if 1
    for(int i = 0; i < l2_path_cache.cache_length; ++i)
    {
        PoseEuler l2_path_pose;
        segment_alg_param.kinematics_ptr->doFK(l2_path_cache.cache[i].joint, l2_path_pose);
        std::cout<< " " <<l2_path_pose.point_.x_
            << " " << l2_path_pose.point_.y_
            << " " << l2_path_pose.point_.z_<<std::endl;
    }
    #endif
#endif

    return 0;
}
