#include "segment_alg.h"
#include "base_kinematics.h"
#include "arm_kinematics.h"
#include "dynamics_interface.h"
#include "basic_alg.h"
#include <iostream>
#include <math.h>
#include <time.h>
#include <ctime>

using namespace fst_mc;
using namespace fst_algorithm;
using namespace basic_alg;

extern double stack[];
extern ComplexAxisGroupModel model;
extern SegmentAlgParam segment_alg_param;


DynamicsInterface dynamics;


void doIK(BaseKinematics* kinematics_ptr, PathCache& path_cache, Joint& start_joint)
{
    Joint result_joint;
    Joint ref_joint = start_joint;
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        kinematics_ptr->inverseKinematicsInUser(path_cache.cache[i].pose, ref_joint, path_cache.cache[i].joint);
        ref_joint = path_cache.cache[i].joint;       
    }
}

int main(void)
{
    initComplexAxisGroupModel();
    BaseKinematics* kinematics_ptr = new ArmKinematics();
    double dh_matrix[9][4] = {{0, 0, 365, 0}, {PI/2, 30, 0, PI/2}, {0, 340, 0, 0}, {PI/2, 35, 350, 0}, {-PI/2, 0, 0, 0}, {PI/2, 0, 96.5, 0}, {0,0,0,0}, {0,0,0,0}, {0,0,0,0}};
    kinematics_ptr->initKinematics(dh_matrix);    
    initSegmentAlgParam(kinematics_ptr, &dynamics);
    initStack(&model);       
    

    fst_mc::PoseEuler start;
    fst_mc::MotionTarget via;
    fst_mc::MotionTarget target;
    fst_mc::Joint start_joint;
    fst_mc::JointState start_state;
    fst_mc::PathCache path_cache;
    fst_mc::TrajectoryCache traj_cache;    
    double acc_ratio = 1;
    double vel_ratio = 1;

#if 0
    start_joint.j1 = 0;
    start_joint.j2 = 0;
    start_joint.j3 = 0;
    start_joint.j4 = 0;
    start_joint.j5 = 0;
    start_joint.j6 = 0;

    target.joint_target.j1 = PI / 2;
    target.joint_target.j2 = PI / 4;
    target.joint_target.j3 = 0;
    target.joint_target.j4 = 0;
    target.joint_target.j5 = 0;
    target.joint_target.j6 = 0;
    target.cnt = 0;
    target.vel = 0.5;

    planPathJoint(start_joint, target, path_cache);

    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        std::cout<<" ["<<i<<"] "
                 <<" J1 = "<<path_cache.cache[i].joint.j1
                 <<" J2 = "<<path_cache.cache[i].joint.j2
                 <<" J3 = "<<path_cache.cache[i].joint.j3
                 <<" J4 = "<<path_cache.cache[i].joint.j4
                 <<" J5 = "<<path_cache.cache[i].joint.j5
                 <<" J6 = "<<path_cache.cache[i].joint.j6
                 <<" PointType = "<<path_cache.cache[i].point_type
                 <<" MotionType = "<<path_cache.cache[i].motion_type<<std::endl;

    }
#endif    
#if 0
    start.position.x = 0;
    start.position.y = 200;
    start.position.z = 300;
    start.orientation.a = 0;
    start.orientation.b = 0;
    start.orientation.c = 0;

    target.pose_target.position.x = 100;
    target.pose_target.position.y = 200;
    target.pose_target.position.z = 300;
    target.pose_target.orientation.a = PI/2;
    target.pose_target.orientation.b = 0;
    target.pose_target.orientation.c = 0;
    target.cnt = 0.5;
    target.vel = 1000;
    
    planPathLine(start, target, path_cache);

    std::cout<<" CacheLength = "<<path_cache.cache_length
             <<" Smooth_in_Index = "<<path_cache.smooth_in_index
             <<" Smooth_out_Index = "<<path_cache.smooth_out_index<<std::endl;
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        std::cout<<" ["<<i<<"] "
                 <<" X = "<<path_cache.cache[i].pose.position.x
                 <<" Y = "<<path_cache.cache[i].pose.position.y
                 <<" Z = "<<path_cache.cache[i].pose.position.z
                 <<" x = "<<path_cache.cache[i].pose.orientation.x
                 <<" y = "<<path_cache.cache[i].pose.orientation.y
                 <<" z = "<<path_cache.cache[i].pose.orientation.z
                 <<" W = "<<path_cache.cache[i].pose.orientation.w
                 <<" PointType = "<<path_cache.cache[i].point_type
                 <<" MotionType = "<<path_cache.cache[i].motion_type<<std::endl;
    }
#endif
#if 0
    start.position.x = 300;
    start.position.y = 0;
    start.position.z = 0;
    start.orientation.a = 0;
    start.orientation.b = 0;
    start.orientation.c = 0;

    via.pose_target.position.x = 300;
    via.pose_target.position.y = 300;
    via.pose_target.position.z = 0;
    via.pose_target.orientation.a = 0;
    via.pose_target.orientation.b = 0;
    via.pose_target.orientation.c = 0;
    via.cnt = 0.5;
    via.vel = 100;

    target.pose_target.position.x = -300;
    target.pose_target.position.y = 300;
    target.pose_target.position.z = 0;
    target.pose_target.orientation.a = PI / 2;
    target.pose_target.orientation.b = 0;
    target.pose_target.orientation.c = 0;
    target.cnt = 0.2;
    target.vel = 1000;

    clock_t start_time = clock();
    planPathSmoothLine(start, via, target, path_cache);
    clock_t end_time = clock();
    long long delta = end_time - start_time;

    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        std::cout<<" ["<<i<<"] "
                 <<" X = "<<path_cache.cache[i].pose.position.x
                 <<" Y = "<<path_cache.cache[i].pose.position.y
                 <<" Z = "<<path_cache.cache[i].pose.position.z
                 <<" x = "<<path_cache.cache[i].pose.orientation.x
                 <<" y = "<<path_cache.cache[i].pose.orientation.y
                 <<" z = "<<path_cache.cache[i].pose.orientation.z
                 <<" W = "<<path_cache.cache[i].pose.orientation.w
                 <<" PointType = "<<path_cache.cache[i].point_type
                 <<" MotionType = "<<path_cache.cache[i].motion_type<<std::endl;
    }

    std::cout<<"delta_time = "<<delta<<std::endl;
#endif


#if 0
    path_cache.cache_length = 91;
    path_cache.smooth_in_index = -1;
    path_cache.smooth_out_index = -1;

    double joint0_angle = 0;
    double joint1_angle = 0;
    double joint0_angle_delta = PI * 1 / 180.0;
    double joint1_angle_delta = PI * 0.5 / 180.0;
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        path_cache.cache[i].joint[0] = joint0_angle;
        path_cache.cache[i].joint[1] = joint1_angle;
        path_cache.cache[i].joint[2] = 0;
        path_cache.cache[i].joint[3] = 0;
        path_cache.cache[i].joint[4] = 0;
        path_cache.cache[i].joint[5] = 0;
        path_cache.cache[i].joint[6] = 0;
        path_cache.cache[i].joint[7] = 0;
        path_cache.cache[i].joint[8] = 0;
        path_cache.cache[i].motion_type = MOTION_JOINT;
        path_cache.cache[i].point_type = PATH_POINT;
        joint0_angle += joint0_angle_delta;
        joint1_angle += joint1_angle_delta;
    }

    for(int i = 0; i < 9; ++i)
    {
        start_state.alpha[i] = 0;
        start_state.omega[i] = 0;
        start_state.angle[i] = 0;
    }
    
    planTrajectory(path_cache, start_state, vel_ratio, acc_ratio, traj_cache);
#endif    

#if 0
    start_joint.j1 = 0;
    start_joint.j2 = 0;
    start_joint.j3 = 0;
    start_joint.j4 = 0;
    start_joint.j5 = 0;
    start_joint.j6 = 0;

    target.joint_target.j1 = PI / 4;
    target.joint_target.j2 = 0;
    target.joint_target.j3 = 0;
    target.joint_target.j4 = 0;
    target.joint_target.j5 = 0;
    target.joint_target.j6 = 0;
    target.cnt = 0;
    target.vel = 1;
    target.type = MOTION_JOINT;
//clock_t t1 = clock();
    planPathJoint(start_joint, target, path_cache);
//clock_t t2 = clock();    
    planTrajectory(path_cache, start_state, target.vel, acc_ratio, traj_cache);
//clock_t t3 = clock();
//long delta_path = t2 - t1;
//long delta_traj = t3 - t2;
//std::cout<<"delta_path = "<<delta_path<<" delta_traj = "<<delta_traj<<std::endl;
    printTraj(traj_cache, 0, 0.001);
#endif

#if 0
    start_joint[0] = 0;
    start_joint[1] = 0;
    start_joint[2] = 0;
    start_joint[3] = 0;
    start_joint[4] = -PI/2;
    start_joint[5] = 0;
    /*Pose start_pose;
    PoseEuler start_euler;
    kinematics_ptr->forwardKinematicsInUser(start_joint, start_euler);
    std::cout<<"start_pose: "<<start_euler.position.x<<" "
                                  <<start_euler.position.y<<" "
                                  <<start_euler.position.z<<" "
                                  <<start_euler.orientation.a<<" "
                                  <<start_euler.orientation.b<<" "
                                  <<start_euler.orientation.c<<std::endl;*/

    start_state.angle = start_joint;
    start_state.omega[0] = 0; start_state.omega[1] = 0; start_state.omega[2] = 0;
    start_state.omega[3] = 0; start_state.omega[4] = 0; start_state.omega[5] = 0;
    start_state.alpha[0] = 0; start_state.alpha[1] = 0; start_state.alpha[2] = 0;
    start_state.alpha[3] = 0; start_state.alpha[4] = 0; start_state.alpha[5] = 0;

    start.position.x = 380;
    start.position.y = 0;
    start.position.z = 643.5;
    start.orientation.a = 0;
    start.orientation.b = 0;
    start.orientation.c = PI;

    target.pose_target.position.x = 250;
    target.pose_target.position.y = 150;
    target.pose_target.position.z = 310;
    target.pose_target.orientation.a = 0;
    target.pose_target.orientation.b = 0;
    target.pose_target.orientation.c = PI;
    target.cnt = -1;
    target.vel = 1000;
    target.type = MOTION_LINE;

    path_cache.target.vel = 1000;
    
    planPathLine(start, target, path_cache);
    /*std::cout<<" CacheLength = "<<path_cache.cache_length
             <<" Smooth_in_Index = "<<path_cache.smooth_in_index
             <<" Smooth_out_Index = "<<path_cache.smooth_out_index<<std::endl;
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        std::cout<<" ["<<i<<"] "
                 <<" X = "<<path_cache.cache[i].pose.position.x
                 <<" Y = "<<path_cache.cache[i].pose.position.y
                 <<" Z = "<<path_cache.cache[i].pose.position.z
                 <<" x = "<<path_cache.cache[i].pose.orientation.x
                 <<" y = "<<path_cache.cache[i].pose.orientation.y
                 <<" z = "<<path_cache.cache[i].pose.orientation.z
                 <<" W = "<<path_cache.cache[i].pose.orientation.w
                 <<" PointType = "<<path_cache.cache[i].point_type
                 <<" MotionType = "<<path_cache.cache[i].motion_type<<std::endl;
    }*/

    Joint result_joint;
    Joint ref_joint = start_joint;
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        kinematics_ptr->inverseKinematicsInUser(path_cache.cache[i].pose, ref_joint, path_cache.cache[i].joint);
        ref_joint = path_cache.cache[i].joint;       
    }

    planTrajectory(path_cache, start_state, vel_ratio, acc_ratio, traj_cache);

    //std::cout<<"traj_cache.smooth_out_index = "<<traj_cache.smooth_out_index<<std::endl;
    //std::cout<<"traj_cache.cache_length = "<<traj_cache.cache_length<<std::endl;
    printAllTraj(traj_cache, 0.001);
#endif

#if 0
    start_joint[0] = 0.289008;
    start_joint[1] = -0.623643;
    start_joint[2] = -0.061923;
    start_joint[3] = 0.000009;
    start_joint[4] = -0.885228;
    start_joint[5] = 0.289;

    start_state.angle = start_joint;
    start_state.omega[0] = 0.747278; start_state.omega[1] = -0.281871; start_state.omega[2] = 0.472685;
    start_state.omega[3] = -0.000003; start_state.omega[4] = -0.190809; start_state.omega[5] = 0.74728;
    start_state.alpha[0] = -152.107803; start_state.alpha[1] = 57.374650; start_state.alpha[2] = -96.214629;
    start_state.alpha[3] = 0.000687; start_state.alpha[4] = 38.838923; start_state.alpha[5] = -152.1085;

    start.position.x = 500;
    start.position.y = 148.6686;
    start.position.z = 350;
    start.orientation.a = 0;
    start.orientation.b = 0;
    start.orientation.c = PI;

    via.pose_target.position.x = 500;
    via.pose_target.position.y = 150;
    via.pose_target.position.z = 350;
    via.pose_target.orientation.a = 0;
    via.pose_target.orientation.b = 0;
    via.pose_target.orientation.c = PI;
    via.cnt = 0.25;
    via.vel = 1600;
    via.type = MOTION_LINE;

    target.pose_target.position.x = 200;
    target.pose_target.position.y = 150;
    target.pose_target.position.z = 350;
    target.pose_target.orientation.a = 0;
    target.pose_target.orientation.b = 0;
    target.pose_target.orientation.c = PI;
    target.cnt = -1;
    target.vel = 1600;
    target.type = MOTION_LINE;

#if 0
    PathCache path_cache_1;
    TrajectoryCache traj_cache_1;
    path_cache_1.target.vel = via.vel;
    path_cache_1.target.cnt = via.cnt;
    planPathLine(start, via, path_cache_1);
    /*std::cout<<" CacheLength = "<<path_cache_1.cache_length
             <<" Smooth_in_Index = "<<path_cache_1.smooth_in_index
             <<" Smooth_out_Index = "<<path_cache_1.smooth_out_index<<std::endl;
    for(int i = 0; i < path_cache_1.cache_length; ++i)
    {
        std::cout<<" ["<<i<<"] "
                 <<" X = "<<path_cache_1.cache[i].pose.position.x
                 <<" Y = "<<path_cache_1.cache[i].pose.position.y
                 <<" Z = "<<path_cache_1.cache[i].pose.position.z
                 <<" x = "<<path_cache_1.cache[i].pose.orientation.x
                 <<" y = "<<path_cache_1.cache[i].pose.orientation.y
                 <<" z = "<<path_cache_1.cache[i].pose.orientation.z
                 <<" W = "<<path_cache_1.cache[i].pose.orientation.w
                 <<" PointType = "<<path_cache_1.cache[i].point_type
                 <<" MotionType = "<<path_cache_1.cache[i].motion_type<<std::endl;
    }*/
 
    doIK(kinematics_ptr, path_cache_1, start_joint);
    /*for(int i=0; i<path_cache_1.cache_length; ++i)
    {
        std::cout<<i<<" "<<path_cache_1.cache[i].joint[0]<<" "
                 <<path_cache_1.cache[i].joint[1]<<" "
                 <<path_cache_1.cache[i].joint[2]<<" "
                 <<path_cache_1.cache[i].joint[3]<<" "
                 <<path_cache_1.cache[i].joint[4]<<" "
                 <<path_cache_1.cache[i].joint[5]<<std::endl;
    }*/
    planTrajectory(path_cache_1, start_state, vel_ratio, acc_ratio, traj_cache_1);

/*for(int i=0; i< traj_cache_1.cache_length; ++i)
{
    std::cout<<i<<" path_index = "<<traj_cache_1.cache[i].index_in_path_cache
                <<" duration = "<<traj_cache_1.cache[i].duration
                <<" A3 = "<<traj_cache_1.cache[i].axis[1].data[3]
                <<" A2 = "<<traj_cache_1.cache[i].axis[1].data[2]
                <<" A1 = "<<traj_cache_1.cache[i].axis[1].data[1]
                <<" A0 = "<<traj_cache_1.cache[i].axis[1].data[0]<<std::endl;
                
}*/

    JointState out_state;
    int p_address = S_TrajP0 + traj_cache_1.smooth_out_index + 1;
    int v_address = p_address + 50;
    int a_address = p_address + 100;
    for(int i=0; i<6; ++i)
    {
        out_state.angle[i] = stack[p_address];
        out_state.omega[i] = stack[v_address] / stack[S_TrajRescaleFactor];
        out_state.alpha[i] = stack[a_address] / (stack[S_TrajRescaleFactor] * stack[S_TrajRescaleFactor]);
        p_address += 150;
        v_address += 150;
        a_address += 150;
    }

    Pose pose_out;
    PoseEuler euler_out;
    double out_quatern[4];
    double out_euler[3];
    pose_out = path_cache_1.cache[traj_cache_1.cache[traj_cache_1.smooth_out_index].index_in_path_cache].pose;
    out_quatern[0] = pose_out.orientation.x;
    out_quatern[1] = pose_out.orientation.y;
    out_quatern[2] = pose_out.orientation.z;
    out_quatern[3] = pose_out.orientation.w;
    getQuaternToEuler(out_quatern, out_euler);
    euler_out.position = pose_out.position;
    euler_out.orientation.a = out_euler[0];
    euler_out.orientation.b = out_euler[1];
    euler_out.orientation.c = out_euler[2];                         
#endif

    PathCache path_cache_2;
    TrajectoryCache traj_cache_2;
    path_cache_2.target.vel = target.vel; 
    path_cache_2.target.cnt = target.cnt;
  
    //planPathSmoothLine(euler_out, via, target, path_cache_2);
    planPathSmoothLine(start, via, target, path_cache_2);
//std::cout<<"path_cache_2.smooth_out_index = "<<path_cache_2.smooth_out_index<<std::endl;
//std::cout<<"path_cache_2.cache_length = "<<path_cache_2.cache_length<<std::endl;     
    /*std::cout<<" CacheLength = "<<path_cache_2.cache_length
             <<" Smooth_in_Index = "<<path_cache_2.smooth_in_index
             <<" Smooth_out_Index = "<<path_cache_2.smooth_out_index<<std::endl; */  
    /*for(int i = 0; i < path_cache_2.cache_length; ++i)
    {
        std::cout<<" ["<<i<<"] "
                 <<" X = "<<path_cache_2.cache[i].pose.position.x
                 <<" Y = "<<path_cache_2.cache[i].pose.position.y
                 <<" Z = "<<path_cache_2.cache[i].pose.position.z
                 <<" x = "<<path_cache_2.cache[i].pose.orientation.x
                 <<" y = "<<path_cache_2.cache[i].pose.orientation.y
                 <<" z = "<<path_cache_2.cache[i].pose.orientation.z
                 <<" W = "<<path_cache_2.cache[i].pose.orientation.w
                 <<" PointType = "<<path_cache_2.cache[i].point_type
                 <<" MotionType = "<<path_cache_2.cache[i].motion_type<<std::endl;
    }*/

    doIK(kinematics_ptr, path_cache_2, start_joint);

    /*for(int i=0; i<path_cache_2.cache_length; ++i)
    {
        std::cout<<i<<" "<<path_cache_2.cache[i].joint[0]<<" "
                 <<path_cache_2.cache[i].joint[1]<<" "
                 <<path_cache_2.cache[i].joint[2]<<" "
                 <<path_cache_2.cache[i].joint[3]<<" "
                 <<path_cache_2.cache[i].joint[4]<<" "
                 <<path_cache_2.cache[i].joint[5]<<std::endl;
    }*/
 
    //planTrajectorySmooth(path_cache_2, out_state, via, vel_ratio, acc_ratio, traj_cache_2);
    planTrajectorySmooth(path_cache_2, start_state, via, vel_ratio, acc_ratio, traj_cache_2);
//std::cout<<"traj_cache_2.smooth_out_index = "<<traj_cache_2.smooth_out_index<<std::endl;
//std::cout<<"traj_cache_2.cache_length = "<<traj_cache_2.cache_length<<std::endl;  

    //printAllTraj(traj_cache_2, 0.001);
    printTraj(traj_cache_2, 1, 0.001);
#endif

#if 1
        start_joint[0] = 0.643499;
        start_joint[1] = 0.056281;
        start_joint[2] = -0.979224;
        start_joint[3] = 0.00001;
        start_joint[4] = -0.647849;
        start_joint[5] = 0.64349;
    
        start_state.angle = start_joint;
        start_state.omega[0] = 0; start_state.omega[1] = 0; start_state.omega[2] = 0;
        start_state.omega[3] = 0; start_state.omega[4] = 0; start_state.omega[5] = 0;
        start_state.alpha[0] = 0; start_state.alpha[1] = 0; start_state.alpha[2] = 0;
        start_state.alpha[3] = 0; start_state.alpha[4] = 0; start_state.alpha[5] = 0;
    
        start.position.x = 200;
        start.position.y = 150;
        start.position.z = 350;
        start.orientation.a = 0;
        start.orientation.b = 0;
        start.orientation.c = PI;
    
        via.pose_target.position.x = 200;
        via.pose_target.position.y = -150;
        via.pose_target.position.z = 350;
        via.pose_target.orientation.a = 0;
        via.pose_target.orientation.b = 0;
        via.pose_target.orientation.c = PI;
        via.cnt = 0.5;
        via.vel = 1600;
        via.type = MOTION_LINE;
    
        target.pose_target.position.x = 500;
        target.pose_target.position.y = -150;
        target.pose_target.position.z = 350;
        target.pose_target.orientation.a = 0;
        target.pose_target.orientation.b = 0;
        target.pose_target.orientation.c = PI;
        target.cnt = 1;
        target.vel = 1600;
        target.type = MOTION_LINE;
    
        PathCache path_cache_1;
        TrajectoryCache traj_cache_1;
        path_cache_1.target.vel = via.vel;
        path_cache_1.target.cnt = via.cnt;
        planPathLine(start, via, path_cache_1);
        /*std::cout<<" CacheLength = "<<path_cache_1.cache_length
                 <<" Smooth_in_Index = "<<path_cache_1.smooth_in_index
                 <<" Smooth_out_Index = "<<path_cache_1.smooth_out_index<<std::endl;
        for(int i = 0; i < path_cache_1.cache_length; ++i)
        {
            std::cout<<" ["<<i<<"] "
                     <<" X = "<<path_cache_1.cache[i].pose.position.x
                     <<" Y = "<<path_cache_1.cache[i].pose.position.y
                     <<" Z = "<<path_cache_1.cache[i].pose.position.z
                     <<" x = "<<path_cache_1.cache[i].pose.orientation.x
                     <<" y = "<<path_cache_1.cache[i].pose.orientation.y
                     <<" z = "<<path_cache_1.cache[i].pose.orientation.z
                     <<" W = "<<path_cache_1.cache[i].pose.orientation.w
                     <<" PointType = "<<path_cache_1.cache[i].point_type
                     <<" MotionType = "<<path_cache_1.cache[i].motion_type<<std::endl;
        }*/
     
        doIK(kinematics_ptr, path_cache_1, start_joint);
        /*for(int i=0; i<path_cache_1.cache_length; ++i)
        {
            std::cout<<i<<" "<<path_cache_1.cache[i].joint[0]<<" "
                     <<path_cache_1.cache[i].joint[1]<<" "
                     <<path_cache_1.cache[i].joint[2]<<" "
                     <<path_cache_1.cache[i].joint[3]<<" "
                     <<path_cache_1.cache[i].joint[4]<<" "
                     <<path_cache_1.cache[i].joint[5]<<std::endl;
        }*/
        planTrajectory(path_cache_1, start_state, vel_ratio, acc_ratio, traj_cache_1);
    
    /*for(int i=0; i< traj_cache_1.cache_length; ++i)
    {
        std::cout<<i<<" path_index = "<<traj_cache_1.cache[i].index_in_path_cache
                    <<" duration = "<<traj_cache_1.cache[i].duration
                    <<" A3 = "<<traj_cache_1.cache[i].axis[1].data[3]
                    <<" A2 = "<<traj_cache_1.cache[i].axis[1].data[2]
                    <<" A1 = "<<traj_cache_1.cache[i].axis[1].data[1]
                    <<" A0 = "<<traj_cache_1.cache[i].axis[1].data[0]<<std::endl;
                    
    }*/
    
        JointState out_state;
        int p_address = S_TrajP0 + traj_cache_1.smooth_out_index + 1;
        int v_address = p_address + 50;
        int a_address = p_address + 100;
        for(int i=0; i<6; ++i)
        {
            out_state.angle[i] = stack[p_address];
            out_state.omega[i] = stack[v_address] / stack[S_TrajRescaleFactor];
            out_state.alpha[i] = stack[a_address] / (stack[S_TrajRescaleFactor] * stack[S_TrajRescaleFactor]);
            p_address += 150;
            v_address += 150;
            a_address += 150;
        }
 //std::cout<<"out_state: "<<out_state.angle[1]<<" "<<out_state.omega[1]<<" "<<out_state.alpha[1]<<std::endl;
        Pose pose_out;
        PoseEuler euler_out;
        double out_quatern[4];
        double out_euler[3];
        pose_out = path_cache_1.cache[traj_cache_1.cache[traj_cache_1.smooth_out_index].index_in_path_cache].pose;
        out_quatern[0] = pose_out.orientation.x;
        out_quatern[1] = pose_out.orientation.y;
        out_quatern[2] = pose_out.orientation.z;
        out_quatern[3] = pose_out.orientation.w;
        getQuaternToEuler(out_quatern, out_euler);
        euler_out.position = pose_out.position;
        euler_out.orientation.a = out_euler[0];
        euler_out.orientation.b = out_euler[1];
        euler_out.orientation.c = out_euler[2];                         
    
        PathCache path_cache_2;
        TrajectoryCache traj_cache_2;
        path_cache_2.target.vel = target.vel; 
        path_cache_2.target.cnt = target.cnt;
      
        planPathSmoothLine(euler_out, via, target, path_cache_2);
    //std::cout<<"path_cache_2.smooth_out_index = "<<path_cache_2.smooth_out_index<<std::endl;
    //std::cout<<"path_cache_2.cache_length = "<<path_cache_2.cache_length<<std::endl;     
        /*std::cout<<" CacheLength = "<<path_cache_2.cache_length
                 <<" Smooth_in_Index = "<<path_cache_2.smooth_in_index
                 <<" Smooth_out_Index = "<<path_cache_2.smooth_out_index<<std::endl; */  
        /*for(int i = 0; i < path_cache_2.cache_length; ++i)
        {
            std::cout<<" ["<<i<<"] "
                     <<" X = "<<path_cache_2.cache[i].pose.position.x
                     <<" Y = "<<path_cache_2.cache[i].pose.position.y
                     <<" Z = "<<path_cache_2.cache[i].pose.position.z
                     <<" x = "<<path_cache_2.cache[i].pose.orientation.x
                     <<" y = "<<path_cache_2.cache[i].pose.orientation.y
                     <<" z = "<<path_cache_2.cache[i].pose.orientation.z
                     <<" W = "<<path_cache_2.cache[i].pose.orientation.w
                     <<" PointType = "<<path_cache_2.cache[i].point_type
                     <<" MotionType = "<<path_cache_2.cache[i].motion_type<<std::endl;
        }*/
    
        doIK(kinematics_ptr, path_cache_2, out_state.angle);
    
        /*for(int i=0; i<path_cache_2.cache_length; ++i)
        {
            std::cout<<i<<" "<<path_cache_2.cache[i].joint[0]<<" "
                     <<path_cache_2.cache[i].joint[1]<<" "
                     <<path_cache_2.cache[i].joint[2]<<" "
                     <<path_cache_2.cache[i].joint[3]<<" "
                     <<path_cache_2.cache[i].joint[4]<<" "
                     <<path_cache_2.cache[i].joint[5]<<std::endl;
        }*/
     
        planTrajectorySmooth(path_cache_2, out_state, via, vel_ratio, acc_ratio, traj_cache_2);
    //std::cout<<"traj_cache_2.smooth_out_index = "<<traj_cache_2.smooth_out_index<<std::endl;
    //std::cout<<"traj_cache_2.cache_length = "<<traj_cache_2.cache_length<<std::endl;  
    
        //printAllTraj(traj_cache_2, 0.001);
        printf("traj-cache1: smooth = %d\n", traj_cache_1.smooth_out_index);
        for (size_t i = 0; i < traj_cache_1.cache_length; i++)
        {
            TrajectoryBlock &block = traj_cache_1.cache[i];
            printf("block[%d]: duration = %.4f, c0 = %.4f, c1 = %.4f, c2 = %.4f, c3 = %.4f\n", i,
                    block.duration, block.axis[1].data[0], block.axis[1].data[1], block.axis[1].data[2], block.axis[1].data[3]);
        }
        printf("----------------\n");
        for (size_t i = 0; i < traj_cache_2.cache_length; i++)
        {
            TrajectoryBlock &block = traj_cache_2.cache[i];
            printf("block[%d]: duration = %.4f, c0 = %.4f, c1 = %.4f, c2 = %.4f, c3 = %.4f\n", i,
                    block.duration, block.axis[1].data[0], block.axis[1].data[1], block.axis[1].data[2], block.axis[1].data[3]);
        }
        //printTraj(traj_cache_2, 1, 0.001);
#endif


	return 0;
}


