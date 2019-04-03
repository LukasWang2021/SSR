#include "segment_alg.h"
#include "kinematics.h"
#include "kinematics_rtm.h"
#include "kinematics_toll.h"
#include "dynamics_interface.h"
#include "basic_alg_datatype.h"
#include "common_file_path.h"
#include <iostream>
#include <math.h>
#include <time.h>
#include <ctime>

using namespace fst_mc;
using namespace basic_alg;
using namespace fst_algorithm;

extern double stack[20000];
extern ComplexAxisGroupModel model;
extern SegmentAlgParam segment_alg_param;

DynamicsInterface dynamics;


void doIK(Kinematics* kinematics_ptr, PathCache& path_cache, Joint& start_joint)
{
    Joint result_joint;
    Joint ref_joint = start_joint;
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        kinematics_ptr->doIK(path_cache.cache[i].pose, ref_joint, path_cache.cache[i].joint);
        ref_joint = path_cache.cache[i].joint;       
    }
}

int main_tool(void)
{
    std::string file_path = AXIS_GROUP_DIR;
    Kinematics* kinematics_ptr = new KinematicsRTM(file_path);
    //Kinematics* kinematics_ptr = new KinematicsToll(file_path);//for toll
    if(!kinematics_ptr->isValid())
    {
        std::cout<<"kinematics init failed"<<std::endl;
    }
    fst_ctrl::ToolManager* tool_manager_ptr = new fst_ctrl::ToolManager();
    tool_manager_ptr->init();
    fst_ctrl::CoordinateManager* coordinate_manager_ptr = new fst_ctrl::CoordinateManager();
    coordinate_manager_ptr->init();


    segment_alg_param.accuracy_cartesian_factor = 1;
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
    segment_alg_param.max_cartesian_acc = 16000;
    segment_alg_param.coordinate_manager_ptr = coordinate_manager_ptr;
    segment_alg_param.tool_manager_ptr = tool_manager_ptr;
    segment_alg_param.kinematics_ptr = kinematics_ptr;
    segment_alg_param.dynamics_ptr = &dynamics;

    double joint_vel_max[6] = {1200, 4.67, 5.81, 7.85, 5.81, 7.85};
    AxisType axis_type[9] = {ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS};
    initSegmentAlgParam(&segment_alg_param, 6, axis_type, joint_vel_max);

    //for toll
    //double joint_vel_max[4] = {1200, 4.67, 5.81, 7.85};
    //AxisType axis_type[9] = {LINEAR_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS};
    //initSegmentAlgParam(&segment_alg_param, 4, axis_type, joint_vel_max);
           
    basic_alg::PoseEuler start;
    fst_mc::MotionTarget via;
    fst_mc::MotionTarget target;
    basic_alg::Joint start_joint;
    fst_mc::JointState start_state;
    fst_mc::PathCache path_cache;
    fst_mc::TrajectoryCache traj_cache;    
    double acc_ratio = 0.5;
    double vel_ratio = 0.5;

//planPathSmoothCircle
#if 0 
    printf("testing planPathSmoothCircle\n");
    Joint ref_joint;
    ref_joint.j1_ = 0;
    ref_joint.j2_ = 0;
    ref_joint.j3_ = 0;
    ref_joint.j4_ = 0;
    ref_joint.j5_ = PI/2;
    ref_joint.j6_ = 0;
    ref_joint.j7_ = 0;
    ref_joint.j8_ = 0;
    ref_joint.j9_ = 0;


    start.point_.x_ = 50;//0
    start.point_.y_ = 150;//150
    start.point_.z_ = 350;
    start.euler_.a_ = 0;
    start.euler_.b_ = 0;
    start.euler_.c_ = PI;
    //used by L2C
    via.pose_target.point_.x_ = 200;
    via.pose_target.point_.y_ = 150;
    via.pose_target.point_.z_ = 350;
    via.pose_target.euler_.a_ = 0;
    via.pose_target.euler_.b_ = 0;
    via.pose_target.euler_.c_ = PI;
    //used byC2C
    //via.circle_target.pose2.point_.x_ = 200;
    //via.circle_target.pose2.point_.y_ = 150;
    //via.circle_target.pose2.point_.z_ = 350;
    //via.circle_target.pose2.euler_.a_ = 0;
    //via.circle_target.pose2.euler_.b_ = 0;
    //via.circle_target.pose2.euler_.c_ = PI;    
    //used byJ2C
    Joint via_joint;
    segment_alg_param.kinematics_ptr->doIK(via.pose_target, ref_joint, via_joint);
    via.joint_target.j1_ = via_joint.j1_;
    via.joint_target.j2_ = via_joint.j2_;
    via.joint_target.j3_ = via_joint.j3_;
    via.joint_target.j4_ = via_joint.j4_;
    via.joint_target.j5_ = via_joint.j5_;
    via.joint_target.j6_ = via_joint.j6_;
    via.joint_target.j7_ = via_joint.j7_;
    via.joint_target.j8_ = via_joint.j8_;
    via.joint_target.j9_ = via_joint.j9_;

    via.cnt = 0.3;
    via.vel = 1000;
    via.type = MOTION_JOINT;//MOTION_LINE;//MOTION_CIRCLE

    target.circle_target.pose1.point_.x_ = 200;
    target.circle_target.pose1.point_.y_ = -150;
    target.circle_target.pose1.point_.z_ = 350;
    target.circle_target.pose1.euler_.a_ = 0;
    target.circle_target.pose1.euler_.b_ = 0;
    target.circle_target.pose1.euler_.c_ = PI;

    target.circle_target.pose2.point_.x_ = 500;
    target.circle_target.pose2.point_.y_ = -150;
    target.circle_target.pose2.point_.z_ = 350;
    target.circle_target.pose2.euler_.a_ = 0;
    target.circle_target.pose2.euler_.b_ = 0;
    target.circle_target.pose2.euler_.c_ = PI;

    target.cnt = 0.4;
    target.vel = 1000;

    target.type = MOTION_CIRCLE;

    path_cache.target.vel = 1000;
    path_cache.target.type = MOTION_CIRCLE;

    planPathSmoothCircle(start, via, target, path_cache);
// 
/*
    Joint result_joint;
    Joint ref_joint = start_joint;
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        kinematics_ptr->doIK(path_cache.cache[i].pose, ref_joint, path_cache.cache[i].joint);
        //kinematics_ptr->inverseKinematicsInUser(path_cache.cache[i].pose, ref_joint, path_cache.cache[i].joint);
        ref_joint = path_cache.cache[i].joint;
    }

    //planTrajectorySmooth(path_cache_2, start_state, via, vel_ratio, acc_ratio, traj_cache_2);
    planTrajectorySmooth(path_cache, start_state, via, vel_ratio, acc_ratio, traj_cache);
    // printTraj(traj_cache, 0, 0.001, traj_cache.cache_length);
    //fkToTraj(traj_cache);
    */

    std::cout<<" CacheLength = "<<path_cache.cache_length
                 <<", Smooth_in_Index = "<<path_cache.smooth_in_index
                 <<", Smooth_out_Index = "<<path_cache.smooth_out_index<<std::endl;
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
            std::cout<<path_cache.cache[i].pose.point_.x_
                     <<"  "<<path_cache.cache[i].pose.point_.y_
                     <<"  "<<path_cache.cache[i].pose.point_.z_<<std::endl;
    }
/*
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
            std::cout<<" ["<<i<<"] "
                     <<" X = "<<path_cache.cache[i].pose.point_.x_
                     <<" Y = "<<path_cache.cache[i].pose.point_.y_
                     <<" Z = "<<path_cache.cache[i].pose.point_.z_
                     <<" x = "<<path_cache.cache[i].pose.quaternion_.x_
                     <<" y = "<<path_cache.cache[i].pose.quaternion_.y_
                     <<" z = "<<path_cache.cache[i].pose.quaternion_.z_
                     <<" W = "<<path_cache.cache[i].pose.quaternion_.w_
                     <<" PointType = "<<path_cache.cache[i].point_type
                     <<" MotionType = "<<path_cache.cache[i].motion_type<<std::endl;
    }
*/
    printf("end of planPathSmoothCircle\n");

#endif

#if 0
    start_joint.j1_ = 0;
    start_joint.j2_ = 0;
    start_joint.j3_ = 0;
    start_joint.j4_ = 0;

    target.joint_target.j1_ = 182;
    target.joint_target.j2_ = -0.22;
    target.joint_target.j3_ = 1.28;
    target.joint_target.j4_ = 1.33;

    target.cnt = -1;
    target.vel = 1;

    planPathJoint(start_joint, target, path_cache);

    /*for(int i = 0; i < path_cache.cache_length; ++i)
    {
        std::cout<<" ["<<i<<"] "
                 <<" J1 = "<<path_cache.cache[i].joint.j1_
                 <<" J2 = "<<path_cache.cache[i].joint.j2_
                 <<" J3 = "<<path_cache.cache[i].joint.j3_
                 <<" J4 = "<<path_cache.cache[i].joint.j4_<<std::endl;

    }*/
    memset(&start_state, 0, sizeof(JointState));    
    path_cache.target.type = MOTION_JOINT;
    path_cache.target.vel = target.vel;
    path_cache.target.cnt = target.cnt;
    planTrajectory(path_cache, start_state, vel_ratio, acc_ratio, traj_cache);
//std::cout<<"traj_cache.length = "<<traj_cache.cache_length<<std::endl;


    printTraj(traj_cache, 0, 0.001, traj_cache.cache_length);
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
//planPathSmoothLine
#if 0
    printf("start testing planPathSmoothLine\n");
    Joint ref_joint;
    ref_joint.j1_ = 0;
    ref_joint.j2_ = 0;
    ref_joint.j3_ = 0;
    ref_joint.j4_ = 0;
    ref_joint.j5_ = -PI/2;
    ref_joint.j6_ = 0;
    ref_joint.j7_ = 0;
    ref_joint.j8_ = 0;
    ref_joint.j9_ = 0;

    start.point_.x_ = 300;
    start.point_.y_ = 0;
    start.point_.z_ = 500;
    start.euler_.a_ = 0;
    start.euler_.b_ = 0;
    start.euler_.c_ = 0;

    PoseEuler temp_pose;
    temp_pose.point_.x_ = 300;
    temp_pose.point_.y_ = 300;
    temp_pose.point_.z_ = 500;
    temp_pose.euler_.a_ = 0;
    temp_pose.euler_.b_ = 0;
    temp_pose.euler_.c_ = 0;
    //L2L
    //via.pose_target.point_.x_ = 300;
    //via.pose_target.point_.y_ = 300;
    //via.pose_target.point_.z_ = 500;
    //via.pose_target.euler_.a_ = 0;
    //via.pose_target.euler_.b_ = 0;
    //via.pose_target.euler_.c_ = 0;
    //used byC2L
    //via.circle_target.pose2.point_.x_ = 300;
    //via.circle_target.pose2.point_.y_ = 300;
    //via.circle_target.pose2.point_.z_ = 500;
    //via.circle_target.pose2.euler_.a_ = 0;
    //via.circle_target.pose2.euler_.b_ = 0;
    //via.circle_target.pose2.euler_.c_ = 0;
    //used byJ2L
    Joint via_joint;
    segment_alg_param.kinematics_ptr->doIK(temp_pose, ref_joint, via_joint);
    via.joint_target.j1_ = via_joint.j1_;
    via.joint_target.j2_ = via_joint.j2_;
    via.joint_target.j3_ = via_joint.j3_;
    via.joint_target.j4_ = via_joint.j4_;
    via.joint_target.j5_ = via_joint.j5_;
    via.joint_target.j6_ = via_joint.j6_;
    via.joint_target.j7_ = via_joint.j7_;
    via.joint_target.j8_ = via_joint.j8_;
    via.joint_target.j9_ = via_joint.j9_;

    via.cnt = 0.5;
    via.vel = 100;
    via.type = MOTION_JOINT; //MOTION_LINE;//MOTION_CIRCLE

    target.pose_target.point_.x_ = -300;
    target.pose_target.point_.y_ = 300;
    target.pose_target.point_.z_ = 500;
    target.pose_target.euler_.a_ = 0;
    target.pose_target.euler_.b_ = 0;
    target.pose_target.euler_.c_ = 0;
    target.cnt = 0.2;
    target.vel = 1000;
    target.type = MOTION_LINE;//MOTION_CIRCLE

    clock_t start_time = clock();
    ErrorCode err = planPathSmoothLine(start, via, target, path_cache);
    if(err != SUCCESS)
    {
        printf("planPathSmoothLine failed, err = 0x%llx\n", err);
        return err;
    }
    clock_t end_time = clock();
    long long delta = end_time - start_time;

    for(int i = 0; i < path_cache.cache_length; ++i)
    {
            std::cout<<path_cache.cache[i].pose.point_.x_
                     <<"  "<<path_cache.cache[i].pose.point_.y_
                     <<"  "<<path_cache.cache[i].pose.point_.z_<<std::endl;
    }
    printf("end testing planPathSmoothLine\n");

/*
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        std::cout<<" ["<<i<<"] "
                 <<" X = "<<path_cache.cache[i].pose.point_.x_
                 <<" Y = "<<path_cache.cache[i].pose.point_.y_
                 <<" Z = "<<path_cache.cache[i].pose.point_.z_
                 <<" x = "<<path_cache.cache[i].pose.quaternion_.x_
                 <<" y = "<<path_cache.cache[i].pose.quaternion_.y_
                 <<" z = "<<path_cache.cache[i].pose.quaternion_.z_
                 <<" W = "<<path_cache.cache[i].pose.quaternion_.w_
                 <<" PointType = "<<path_cache.cache[i].point_type
                 <<" MotionType = "<<path_cache.cache[i].motion_type<<std::endl;
    }
*/
    std::cout<<"-delta_time = "<<delta<<std::endl;
#endif
//planPathSmoothJoint
#if 1
    basic_alg::Joint js, jv, je;
    js[0] = 0; jv[0] = M_PI/2; je[0] = M_PI;
    js[1] = 0; jv[1] = 0;    je[1] = M_PI;
    js[2] = 0; jv[2] = 0;    je[2] = 0;
    js[3] = 0; jv[3] = 0;    je[3] = 0;
    js[4] = M_PI/4; jv[4] = M_PI/4;    je[4] = M_PI/4;
    js[5] = 0; jv[5] = 0;    je[5] = 0;

    start_joint = js;

    //L2J
    PoseEuler temp_pose;
    segment_alg_param.kinematics_ptr->doFK(jv, temp_pose);
    via.pose_target = temp_pose;
    via.cnt = 0.5;
    via.vel = 500;
    via.type = MOTION_LINE;


    //J2J
    //via.joint_target = jv;
    //via.cnt = 0.5;
    //via.vel = 1;
    //via.type = MOTION_JOINT;

    target.joint_target = je;
    target.cnt = -1;
    target.vel = 1;
    target.type = MOTION_JOINT;

    ErrorCode err = planPathSmoothJoint(start_joint, via, target, path_cache);
    if (err != SUCCESS)
    {
        printf("faled planPathSmoothJoint\n");
    }
    std::cout<<"path_cache.smooth_out_index = "<<path_cache.smooth_out_index<<std::endl;
    std::cout<<"path_cache.smooth_in_index = "<<path_cache.smooth_in_index<<std::endl;
    std::cout<<"path_cache.cache_length = "<<path_cache.cache_length<<std::endl;
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        std::cout<<" "<<path_cache.cache[i].joint[0]
                 <<" "<<path_cache.cache[i].joint[1]<<std::endl;
    }

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

    target.joint_target.j1 = PI ;
    target.joint_target.j2 = 0;
    target.joint_target.j3 = 0;
    target.joint_target.j4 = 0;
    target.joint_target.j5 = 0;
    target.joint_target.j6 = 0;
    target.cnt = 0;
    target.vel = 1;
    target.type = MOTION_JOINT;

    path_cache.target.vel = 1;
    path_cache.target.type = MOTION_JOINT;
//clock_t t1 = clock();
    planPathJoint(start_joint, target, path_cache);
//clock_t t2 = clock();    
    planTrajectory(path_cache, start_state, target.vel, acc_ratio, traj_cache);
//clock_t t3 = clock();
//long delta_path = t2 - t1;
//long delta_traj = t3 - t2;
//std::cout<<"delta_path = "<<delta_path<<" delta_traj = "<<delta_traj<<std::endl;
    printTraj(traj_cache, 0, 0.001, traj_cache.cache_length);
#endif

#if 0
    start_joint[0] = 0;
    start_joint[1] = 0;
    start_joint[2] = 0;
    start_joint[3] = 0;
    start_joint[4] = -1.5708;
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

    start.position.x = 250;
    start.position.y = 150;
    start.position.z = 310;
    start.orientation.a = 0;
    start.orientation.b = 0;
    start.orientation.c = -PI;

    //Joint r_joint;
    //kinematics_ptr->inverseKinematicsInUser(start, start_joint, r_joint);
//std::cout<<r_joint[0]<<" "<<r_joint[1]<<" "<<r_joint[2]<<" "<<r_joint[3]<<" "<<r_joint[4]<<" "<<r_joint[5]<<std::endl;

    target.pose_target.position.x = 550;
    target.pose_target.position.y = -150;
    target.pose_target.position.z = 610;
    target.pose_target.orientation.a = 0;
    target.pose_target.orientation.b = 0;
    target.pose_target.orientation.c = -PI;
    target.cnt = -1;
    target.vel = 1600;
    target.type = MOTION_LINE;

    path_cache.target.vel = 1600;
    path_cache.target.type = MOTION_LINE;
    
    planPathLine(start, target, path_cache);
    //std::cout<<"path_cache_length = "<<path_cache.cache_length<<std::endl;
    //std::cout<<"smooth_out_index = "<<path_cache.smooth_out_index<<std::endl;

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

    printTraj(traj_cache, 1, 0.001, traj_cache.cache_length);
    double total_time = 0;
    for(int i=0; i<traj_cache.cache_length; i++)
    {
        total_time += traj_cache.cache[i].duration;
    }
    std::cout<<"total_time = "<<total_time<<std::endl;
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
//smooth L test
#if 0 
        start_joint[0] = 0.358665;
        start_joint[1] = -0.3994186;
        start_joint[2] = -0.4181983;
        start_joint[3] = 0.0;
        start_joint[4] = -0.75304;
        start_joint[5] = 0.3582637;
    
        start_state.angle = start_joint;
        start_state.omega[0] = 0; start_state.omega[1] = 0; start_state.omega[2] = 0;
        start_state.omega[3] = 0; start_state.omega[4] = 0; start_state.omega[5] = 0;
        start_state.alpha[0] = 0; start_state.alpha[1] = 0; start_state.alpha[2] = 0;
        start_state.alpha[3] = 0; start_state.alpha[4] = 0; start_state.alpha[5] = 0;
    
        start.position.x = 400;
        start.position.y = 150;
        start.position.z = 350;
        start.orientation.a = 0;
        start.orientation.b = 0;
        start.orientation.c = PI;
    
        via.pose_target.position.x = 400;
        via.pose_target.position.y = 250;
        via.pose_target.position.z = 350;
        via.pose_target.orientation.a = 0;
        via.pose_target.orientation.b = 0;
        via.pose_target.orientation.c = PI;
        via.cnt = 0.1;
        via.vel = 1600;
        via.type = MOTION_LINE;
    
        target.pose_target.position.x = 300;
        target.pose_target.position.y = 250;
        target.pose_target.position.z = 350;
        target.pose_target.orientation.a = 0;
        target.pose_target.orientation.b = 0;
        target.pose_target.orientation.c = PI;
        target.cnt = 0.1;
        target.vel = 1600;
        target.type = MOTION_LINE;
    
        PathCache path_cache_1;
        TrajectoryCache traj_cache_1;
        path_cache_1.target.type = MOTION_LINE;
        path_cache_1.target.vel = via.vel;
        path_cache_1.target.cnt = via.cnt;
        planPathLine(start, via, path_cache_1);
        /*std::cout<<" CacheLength = "<<path_cache_1.cache_length
                 <<" Smooth_in_Index = "<<path_cache_1.smooth_in_index
                 <<" Smooth_out_Index = "<<path_cache_1.smooth_out_index<<std::endl;*/
        /*for(int i = 0; i < path_cache_1.cache_length; ++i)
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

        printTraj(traj_cache_1, 2, 0.001, traj_cache_1.smooth_out_index + 1);
        
    /*for(int i=0; i< traj_cache_1.cache_length; ++i)
    {
        std::cout<<i<<" path_index = "<<traj_cache_1.cache[i].index_in_path_cache
                    <<" duration = "<<traj_cache_1.cache[i].duration
                    <<" A3 = "<<traj_cache_1.cache[i].axis[1].data[3]
                    <<" A2 = "<<traj_cache_1.cache[i].axis[1].data[2]
                    <<" A1 = "<<traj_cache_1.cache[i].axis[1].data[1]
                    <<" A0 = "<<traj_cache_1.cache[i].axis[1].data[0]<<std::endl;
                    
    }*/
    //std::cout<<"traj_cache_1.smooth_out_index = "<<traj_cache_1.smooth_out_index<<std::endl;
    
        JointState out_state;
        int p_address = S_TrajP0 + traj_cache_1.smooth_out_index + 1;
        int v_address = p_address + 25;
        int a_address = p_address + 50;
        for(int i=0; i<6; ++i)
        {
            out_state.angle[i] = stack[p_address];
            out_state.omega[i] = stack[v_address];
            out_state.alpha[i] = stack[a_address];
            p_address += 75;
            v_address += 75;
            a_address += 75;
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
        path_cache_2.target.type = MOTION_LINE;
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
    
        printTraj(traj_cache_2, 2, 0.001, traj_cache_2.cache_length);
#endif


#if 0

    basic_alg::Joint js, jv, je;
    js[0] = 0; jv[0] = M_PI; je[0] = M_PI;
    js[1] = 0; jv[1] = 0;    je[1] = M_PI;
    js[2] = 0; jv[2] = 0;    je[2] = 0;
    js[3] = 0; jv[3] = 0;    je[3] = 0;
    js[4] = 0; jv[4] = 0;    je[4] = 0;
    js[5] = 0; jv[5] = 0;    je[5] = 0;    
    start_state.angle = js;
    start_state.omega[0] = 0; start_state.omega[1] = 0; start_state.omega[2] = 0;
    start_state.omega[3] = 0; start_state.omega[4] = 0; start_state.omega[5] = 0;
    start_state.alpha[0] = 0; start_state.alpha[1] = 0; start_state.alpha[2] = 0;
    start_state.alpha[3] = 0; start_state.alpha[4] = 0; start_state.alpha[5] = 0;

    
    via.joint_target = jv;
    via.cnt = 0.5;
    via.vel = 1;
    via.type = MOTION_JOINT;

    target.joint_target = je;
    target.cnt = -1;
    target.vel = 1;
    target.type = MOTION_JOINT;

    PathCache path_cache_1;
    TrajectoryCache traj_cache_1;
    path_cache_1.target.type = MOTION_JOINT;
    path_cache_1.target.vel = via.vel;
    path_cache_1.target.cnt = via.cnt;
    planPathJoint(js, via, path_cache_1);

    /*for(int i=0; i<path_cache_1.cache_length; ++i)
    {
        std::cout<<i<<" "<<path_cache_1.cache[i].joint[0]<<" "
                 <<path_cache_1.cache[i].joint[1]<<" "
                 <<path_cache_1.cache[i].joint[2]<<" "
                 <<path_cache_1.cache[i].joint[3]<<" "
                 <<path_cache_1.cache[i].joint[4]<<" "
                 <<path_cache_1.cache[i].joint[5]<<std::endl;
    }*/
    //std::cout<<"path_cache_1.smooth_out_index = "<<path_cache_1.smooth_out_index<<std::endl;
    
    planTrajectory(path_cache_1, start_state, vel_ratio, acc_ratio, traj_cache_1);
    
    //std::cout<<"traj_cache_1.smooth_out_index = "<<traj_cache_1.smooth_out_index<<std::endl;
    //std::cout<<"traj_cache_1.cache_length = "<<traj_cache_1.cache_length<<std::endl;

//    printTraj(traj_cache_1, 1, 0.001, traj_cache_1.smooth_out_index + 1);
        
    /*for(int i=0; i< traj_cache_1.cache_length; ++i)
    {
        std::cout<<i<<" path_index = "<<traj_cache_1.cache[i].index_in_path_cache
                    <<" duration = "<<traj_cache_1.cache[i].duration
                    <<" A3 = "<<traj_cache_1.cache[i].axis[1].data[3]
                    <<" A2 = "<<traj_cache_1.cache[i].axis[1].data[2]
                    <<" A1 = "<<traj_cache_1.cache[i].axis[1].data[1]
                    <<" A0 = "<<traj_cache_1.cache[i].axis[1].data[0]<<std::endl;
                    
    }*/
    //std::cout<<"traj_cache_1.smooth_out_index = "<<traj_cache_1.smooth_out_index<<std::endl;

    

    
    JointState out_state;
    int p_address = S_TrajP0 + traj_cache_1.smooth_out_index + 1;
    int v_address = p_address + 25;
    int a_address = p_address + 50;
    for(int i=0; i<6; ++i)
    {
        out_state.angle[i] = stack[p_address];
        out_state.omega[i] = stack[v_address];
        out_state.alpha[i] = stack[a_address];
        p_address += 75;
        v_address += 75;
        a_address += 75;
    }
                            
    PathCache path_cache_2;
    TrajectoryCache traj_cache_2;
    path_cache_2.target.type = MOTION_JOINT;
    path_cache_2.target.vel = target.vel; 
    path_cache_2.target.cnt = target.cnt;
  
    planPathSmoothJoint(out_state.angle, via, target, path_cache_2);
    std::cout<<"path_cache_2.smooth_out_index = "<<path_cache_2.smooth_out_index<<std::endl;
    std::cout<<"path_cache_2.smooth_in_index = "<<path_cache_2.smooth_in_index<<std::endl;
    std::cout<<"path_cache_2.cache_length = "<<path_cache_2.cache_length<<std::endl;
    for(int i = 0; i < path_cache_2.cache_length; ++i)
    {
        std::cout<<" "<<path_cache_2.cache[i].joint[0]
                 <<" "<<path_cache_2.cache[i].joint[1]<<std::endl;
    }
    /*     
    for(int i = 0; i < path_cache_2.cache_length; ++i)
    {
        std::cout<<" ["<<i<<"] "
                 <<" J0 = "<<path_cache_2.cache[i].joint[0]
                 <<" J1 = "<<path_cache_2.cache[i].joint[1]<<std::endl;
    }*/
    std::cout<<"out_state: "<<out_state.angle[0]<<" "<<out_state.omega[0]<<" "<<out_state.alpha[0]<<std::endl;  

    planTrajectorySmooth(path_cache_2, out_state, via, vel_ratio, acc_ratio, traj_cache_2);
    //std::cout<<"traj_cache_2.smooth_out_index = "<<traj_cache_2.smooth_out_index<<std::endl;
    //std::cout<<"traj_cache_2.cache_length = "<<traj_cache_2.cache_length<<std::endl;  
    
 //   printTraj(traj_cache_2, 1, 0.001, traj_cache_2.cache_length);

#endif
#if 0
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

    start.point_.x_ = 200;
    start.point_.y_ = 150;
    start.point_.z_ = 350;
    start.euler_.a_ = 0;
    start.euler_.b_ = 0;
    start.euler_.c_ = PI;

    target.circle_target.pose1.point_.x_ = 200;
    target.circle_target.pose1.point_.y_ = -150;
    target.circle_target.pose1.point_.z_ = 350;
    target.circle_target.pose1.euler_.a_ = 0;
    target.circle_target.pose1.euler_.b_ = 0;
    target.circle_target.pose1.euler_.c_ = PI;

    target.circle_target.pose2.point_.x_ = 500;
    target.circle_target.pose2.point_.y_ = -150;
    target.circle_target.pose2.point_.z_ = 350;
    target.circle_target.pose2.euler_.a_ = 0;
    target.circle_target.pose2.euler_.b_ = 0;
    target.circle_target.pose2.euler_.c_ = PI;

    target.cnt = 0;
    target.vel = 1600;

    target.type = MOTION_CIRCLE;

    path_cache.target.vel = 1600;
    path_cache.target.type = MOTION_CIRCLE;

    planPathCircle(start, target, path_cache);
#endif
#if 0
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        std::cout<< " " <<path_cache.cache[i].pose.point_.x_
            << " " <<path_cache.cache[i].pose.point_.y_
            << " " <<path_cache.cache[i].pose.point_.z_
            << " " <<path_cache.cache[i].pose.quaternion_.x_
            << " " <<path_cache.cache[i].pose.quaternion_.y_
            << " " <<path_cache.cache[i].pose.quaternion_.z_
            << " " <<path_cache.cache[i].pose.quaternion_.w_<<std::endl;
            // << " x=" <<path_cache.cache[i].point_type
            // << " x=" <<path_cache.cache[i].motion_type<<std::endl;
    }

#endif
#if 0
    Joint result_joint;
    Joint ref_joint = start_joint;
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        kinematics_ptr->doIK(path_cache.cache[i].pose, ref_joint, path_cache.cache[i].joint);
        //kinematics_ptr->inverseKinematicsInUser(path_cache.cache[i].pose, ref_joint, path_cache.cache[i].joint);
        ref_joint = path_cache.cache[i].joint;
    }

    planTrajectory(path_cache, start_state, vel_ratio, acc_ratio, traj_cache);

    printTraj(traj_cache, 1, 0.001, traj_cache.cache_length);
    double total_time = 0;
    for(int i=0; i<traj_cache.cache_length; i++)
    {
        total_time += traj_cache.cache[i].duration;
    }
    //std::cout<<"total_time = "<<total_time<<std::endl;
#endif

    return 0;
}


