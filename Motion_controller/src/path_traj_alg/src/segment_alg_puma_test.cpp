
#include "segment_alg.h"
#include "kinematics.h"
#include "kinematics_rtm.h"
#include "kinematics_toll.h"
#include <dynamic_alg_rtm.h>
#include "basic_alg_datatype.h"
#include "common_file_path.h"
#include <iostream>
#include <math.h>
#include <time.h>
#include <ctime>

using namespace fst_mc;
using namespace basic_alg;

extern double stack[50000];
extern ComplexAxisGroupModel model;
extern SegmentAlgParam segment_alg_param;

extern void getMoveEulerToQuatern(const basic_alg::Euler& euler, double* quatern);


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

    segment_alg_param.accuracy_cartesian_factor = 32;
    segment_alg_param.accuracy_joint_factor = 64;
    segment_alg_param.max_traj_points_num = 50;
    segment_alg_param.path_interval = 1.0;
    segment_alg_param.joint_interval = 0.01745 / 2;//PI * 1 / 180;
    segment_alg_param.angle_interval = 0.01745;//PI * 1 / 180;
    segment_alg_param.angle_valve = 0.8727;//PI * 5 / 180;
    segment_alg_param.conservative_acc = 1000;
    segment_alg_param.time_factor_first = 2.5;
    segment_alg_param.time_factor_last = 3;
    segment_alg_param.is_fake_dynamics = false;
    segment_alg_param.max_cartesian_acc = 8000;
    segment_alg_param.kinematics_ptr = kinematics_ptr;
    segment_alg_param.dynamics_ptr = &dynamics;
    segment_alg_param.time_rescale_flag = 0;
    //segment_alg_param.select_algorithm = 1;
    //segment_alg_param.max_cartesian_circle_acc = 500;
    //segment_alg_param.accuracy_cartesian_circle_factor = 32;
    //segment_alg_param.is_constraint_dynamic = false;
    //segment_alg_param.band_matrix_solution_method = 1;

    double vel_ratio = 0.5;
    double acc_ratio = 0.5;

    double joint_vel_max[9] = {5.8119, 4.6600, 5.8119, 7.8540, 7.06866, 10.5592, 0, 0, 0};
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
    // p1.point_.x_ = 300; p1.point_.y_ = 300; p1.point_.z_ = 400; p1.euler_.a_ = 0; p1.euler_.b_ = 0; p1.euler_.c_ = M_PI;
    // p2.point_.x_ = 450; p2.point_.y_ = 150; p2.point_.z_ = 400; p2.euler_.a_ = 0; p2.euler_.b_ = 0; p2.euler_.c_ = M_PI;
    // p3.point_.x_ = 300; p3.point_.y_ = 0;   p3.point_.z_ = 400; p3.euler_.a_ = 0; p3.euler_.b_ = 0; p3.euler_.c_ = M_PI;
    // p4.point_.x_ = 450; p4.point_.y_ = -150;p4.point_.z_ = 400; p4.euler_.a_ = 0; p4.euler_.b_ = 0; p4.euler_.c_ = M_PI;
    // p5.point_.x_ = 300; p5.point_.y_ = -300;p5.point_.z_ = 400; p5.euler_.a_ = 0; p5.euler_.b_ = 0; p5.euler_.c_ = M_PI;

    p1.point_.x_ = 212.924; p1.point_.y_ = -0.01; p1.point_.z_ =  571.233; p1.euler_.a_ = -1.082; p1.euler_.b_ = -0.154; p1.euler_.c_ =  3.141;
    //p2.point_.x_ = 450; p2.point_.y_ = 150; p2.point_.z_ = 400; p2.euler_.a_ = 0; p2.euler_.b_ = 0; p2.euler_.c_ = M_PI;

    p3.point_.x_ = 428.925; p3.point_.y_ = 220.79;   p3.point_.z_ = 489.633; p3.euler_.a_ = -1.082; p3.euler_.b_ = -0.154; p3.euler_.c_ = 3.141;
    p4.point_.x_ = 77.196;  p4.point_.y_ = -333.366; p4.point_.z_ = 517.744; p4.euler_.a_ = -1.082; p4.euler_.b_ = -0.154; p4.euler_.c_ = 3.141;
    p5.point_.x_ = 281.196; p5.point_.y_ = -248.366; p5.point_.z_ = 517.744; p5.euler_.a_ = -1.082; p5.euler_.b_ = -0.154; p5.euler_.c_ = 3.141;
    Joint j2_via;
    j2_via.j1_ = -0.397; j2_via.j2_ = -0.156; j2_via.j3_ = -0.018; j2_via.j4_ = 0.149;  j2_via.j5_ = -1.08; j2_via.j6_ = -0.447;
    segment_alg_param.kinematics_ptr->doFK(j2_via, p2);

    PoseQuaternion q1,q2,q3,q4,q5; 
    double quatern_temp[4];
    p1.convertToPoseQuaternion(q1);
    p2.convertToPoseQuaternion(q2);
    p3.convertToPoseQuaternion(q3);
    p4.convertToPoseQuaternion(q4);
    p5.convertToPoseQuaternion(q5);

    printf("%lf, %lf, %lf, %lf,  %lf, %lf, %lf\n",
         p1.point_.x_, p1.point_.y_, p1.point_.z_, q1.quaternion_.x_, q1.quaternion_.y_, q1.quaternion_.z_,q1.quaternion_.w_);
    printf("%lf, %lf, %lf, %lf,  %lf, %lf\n",
         p1.point_.x_, p1.point_.y_, p1.point_.z_, p1.euler_.a_, p1.euler_.b_, p1.euler_.c_);
 
    printf("%lf, %lf, %lf, %lf,  %lf, %lf, %lf\n",
         p2.point_.x_, p2.point_.y_, p2.point_.z_, q2.quaternion_.x_, q2.quaternion_.y_, q2.quaternion_.z_,q2.quaternion_.w_);
    printf("%lf, %lf, %lf, %lf,  %lf, %lf\n",
         p2.point_.x_, p2.point_.y_, p2.point_.z_, p2.euler_.a_, p2.euler_.b_, p2.euler_.c_);
    printf("%lf, %lf, %lf, %lf,  %lf, %lf, %lf\n",
         p3.point_.x_, p3.point_.y_, p3.point_.z_, q3.quaternion_.x_, q3.quaternion_.y_, q3.quaternion_.z_,q3.quaternion_.w_);
     printf("%lf, %lf, %lf, %lf,  %lf, %lf\n",
         p3.point_.x_, p3.point_.y_, p3.point_.z_, p3.euler_.a_, p3.euler_.b_, p3.euler_.c_);
    // printf("%lf, %lf, %lf, %lf,  %lf, %lf, %lf\n",
        //  p4.point_.x_, p4.point_.y_, p4.point_.z_, q4.quaternion_.x_, q4.quaternion_.y_, q4.quaternion_.z_,q4.quaternion_.w_);
    // printf("%lf, %lf, %lf, %lf,  %lf, %lf\n",
        //  p4.point_.x_, p4.point_.y_, p4.point_.z_, p4.euler_.a_, p4.euler_.b_, p4.euler_.c_);
//  
    // printf("%lf, %lf, %lf, %lf,  %lf, %lf, %lf\n",
        //  p5.point_.x_, p5.point_.y_, p5.point_.z_, q5.quaternion_.x_, q5.quaternion_.y_, q5.quaternion_.z_,q5.quaternion_.w_);
    // printf("%lf, %lf, %lf, %lf,  %lf, %lf\n",
        //  p5.point_.x_, p5.point_.y_, p5.point_.z_, p5.euler_.a_, p5.euler_.b_, p5.euler_.c_);
    // p1 cart: {"x": 212.924, "y": -0.01, "z": 571.233, "a": 0, "b": -0.345994, "c": 3.140999}, 
    // p2 joint: {"j1": -0.397, "j2": -0.156, "j3": -0.018, "j4": 0.149, "j5": -1.08, "j6": -0.447}
    // p3 cart: {"x": 428.925, "y": 220.79, "z": 489.633, "a": 0, "b": -0.346, "c": 3.141
    // p4 cart: {"x": 77.196, "y": -333.366, "z": 517.744, "a": -1.082, "b": -0.154, "c": 3.141}, 
    // p5 cart: {"x": 281.196, "y": -248.366, "z": 517.744, "a": -1.082, "b": -0.154, "c": 3.141}, 
/************** test movec and movec smooth *********/
#if 0
    #if 0
        #if 1
        MotionInfo c1_target;
        c1_target.via.pose.pose = p2;
        c1_target.target.pose.pose = p3;
        c1_target.type = MOTION_CIRCLE;
        c1_target.vel = 2000;
        c1_target.cnt = -1;

        PathCache c1_path_cache;
        c1_path_cache.target.vel = c1_target.vel;
        c1_path_cache.target.type = c1_target.type;
        c1_path_cache.target.cnt = c1_target.cnt;
        planPathCircle(p1, c1_target, c1_path_cache);
        printf("c1_path_cache.smooth_out_index = %d\n", c1_path_cache.smooth_out_index);
        printf("c1_path_cache.cache_length = %d\n", c1_path_cache.cache_length);

        #if 0
        for(int i = 0; i < c1_path_cache.cache_length; ++i)
        {
            std::cout<< " " <<c1_path_cache.cache[i].pose.point_.x_
                << " " <<c1_path_cache.cache[i].pose.point_.y_
                << " " <<c1_path_cache.cache[i].pose.point_.z_
                << " " <<c1_path_cache.cache[i].pose.quaternion_.x_
                << " " <<c1_path_cache.cache[i].pose.quaternion_.y_
                << " " <<c1_path_cache.cache[i].pose.quaternion_.z_
                << " " <<c1_path_cache.cache[i].pose.quaternion_.w_<<std::endl;
        }
        #endif

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

        timeval start_time_val;
        gettimeofday(&start_time_val, NULL);

        planTrajectory(c1_path_cache, p1_state, vel_ratio, acc_ratio, c1_traj_cache);

        timeval end_time_val;
        gettimeofday(&end_time_val, NULL);

        long delta_time = (long)(end_time_val.tv_usec - start_time_val.tv_usec);
        printf("delta_time = %ld\n", delta_time);

        /*std::cout<<"c1_traj_cache.smooth_out_index = "<<c1_traj_cache.smooth_out_index<<std::endl
                 <<"c1_traj_cache.cache_length = "<<c1_traj_cache.cache_length<<std::endl;*/
        // printTraj(c1_traj_cache, 0, 0.01, c1_traj_cache.cache_length);
        // getTrajJoint(c1_traj_cache);
        // getTrajCart(c1_traj_cache);
        fkToTraj(c1_traj_cache); 
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
    #if 0

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
        start_test.point_.x_ = 300;
        start_test.point_.y_ = 300;
        start_test.point_.z_ = 400;
        start_test.euler_.a_ = 0;
        start_test.euler_.b_ = 0;
        start_test.euler_.c_ = M_PI;

        PoseEuler end_test;
        end_test.point_.x_ = 450;
        end_test.point_.y_ = 150;
        end_test.point_.z_ = 400;
        end_test.euler_.a_ = 0;
        end_test.euler_.b_ = 0;
        end_test.euler_.c_ = M_PI;

        MotionInfo l1_target;
        l1_target.target.pose.pose = end_test;
        l1_target.type = MOTION_LINE;
        l1_target.smooth_type = SMOOTH_DISTANCE;
        l1_target.vel = 1000;
        l1_target.cnt = 500;

        PathCache l1_path_cache;
        l1_path_cache.target.vel = l1_target.vel;
        l1_path_cache.target.type = l1_target.type;
        l1_path_cache.target.cnt = l1_target.cnt;
        planPathLine(start_test, l1_target, l1_path_cache);
        // printf("l1_path_cache.smooth_out_index = %d\n", l1_path_cache.smooth_out_index);
        // printf("l1_path_cache.cache_length = %d\n", l1_path_cache.cache_length);

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
        // printTraj(l1_traj_cache, 5, 0.001, l1_traj_cache.cache_length);

        printf("=========== start traj cache 1 ============== \n");
        //fkToTraj(l1_traj_cache);
        //getTrajCart(l1_traj_cache);
        
        printf("=========== end traj cache 1 ============== \n");
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

    #if 1
    PoseEuler smooth_end_test;
    smooth_end_test.point_.x_ = 300;
    smooth_end_test.point_.y_ = 0;
    smooth_end_test.point_.z_ = 400;
    smooth_end_test.euler_.a_ = 0;
    smooth_end_test.euler_.b_ = 0;
    smooth_end_test.euler_.c_ = M_PI;

    MotionInfo l2_target;
    l2_target.target.pose.pose = smooth_end_test;
    l2_target.type = MOTION_LINE;
    l2_target.smooth_type = SMOOTH_NONE;
    l2_target.vel = 1000;
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
    // /j_out.print("j_out:");
    JointState out_state;
    getOutPVA(l1_traj_cache, out_state.angle, out_state.omega, out_state.alpha);
    // out_state.angle = j_out;
    // out_state.omega.j1_ = 0; out_state.omega.j2_ = 0; out_state.omega.j3_ = 0; 
    // out_state.omega.j4_ = 0; out_state.omega.j5_ = 0; out_state.omega.j6_ = 0; 
    // out_state.alpha.j1_ = 0; out_state.alpha.j2_ = 0; out_state.alpha.j3_ = 0; 
    // out_state.alpha.j4_ = 0; out_state.alpha.j5_ = 0; out_state.alpha.j6_ = 0;    
    TrajectoryCache l2_traj_cache;
    planTrajectorySmooth(l2_path_cache, out_state, l1_target, vel_ratio, acc_ratio, l2_traj_cache);
    //printTraj(c2_traj_cache, 0, 0.001, c2_traj_cache.cache_length);
    printf("=========== start traj cache 2 ============== \n");
    getTrajCart(l2_traj_cache);
    fkToTraj(l2_traj_cache);
    printf("=========== start traj cache 2============== \n");
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
#if 1
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
    #if 1
        #if 1
        MotionInfo l1_target;
        l1_target.target.pose.pose = p2;
        kinematics_ptr->doIK(p2, posture, l1_target.target.joint);
        l1_target.type = MOTION_JOINT;
        l1_target.smooth_type = SMOOTH_NONE;
        l1_target.vel = 0.5;
        l1_target.cnt = -1;

        PathCache l1_path_cache;
        l1_path_cache.target.vel = l1_target.vel;
        l1_path_cache.target.type = l1_target.type;
        l1_path_cache.target.cnt = l1_target.cnt;

        Joint j1;
        kinematics_ptr->doIK(p1, posture, j1);

        planPathJoint(j1, l1_target, l1_path_cache);
        printf("l1_path_cache.smooth_out_index = %d\n", l1_path_cache.smooth_out_index);
        printf("l1_path_cache.cache_length = %d\n", l1_path_cache.cache_length);

        #endif
        #if 1
        for(size_t i = 0; i < l1_path_cache.cache_length; ++i)
        {
            kinematics_ptr->doFK(l1_path_cache.cache[i].joint, l1_path_cache.cache[i].pose);
        }

        l1_path_cache.target = l1_target;
        // Joint j1;
        // kinematics_ptr->doIK(p1, posture, j1);
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
        //printTraj(l1_traj_cache, 1, 0.001, l1_traj_cache.cache_length);

         fkToTraj(l1_traj_cache);
        // getTrajCart(l1_traj_cache);
        #endif
    #endif

    /************ C2j ************/
    #if 0
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

    #if 0
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

    return 0;
}