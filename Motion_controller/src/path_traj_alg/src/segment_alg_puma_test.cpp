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

extern DynamicsInterface dynamics;


int main(void)
{
    std::string file_path = AXIS_GROUP_DIR;
    Kinematics* kinematics_ptr = new KinematicsRTM(file_path);

    if(!kinematics_ptr->isValid())
    {
        std::cout<<"kinematics init failed"<<std::endl;
    }
    fst_ctrl::ToolManager* tool_manager_ptr = new fst_ctrl::ToolManager();
    tool_manager_ptr->init();
    fst_ctrl::CoordinateManager* coordinate_manager_ptr = new fst_ctrl::CoordinateManager();
    coordinate_manager_ptr->init();

    segment_alg_param.accuracy_cartesian_factor = 3;
    segment_alg_param.accuracy_joint_factor = 8;
    segment_alg_param.max_traj_points_num = 20;
    segment_alg_param.path_interval = 1;
    segment_alg_param.joint_interval = 0.01745;//PI * 1 / 180;
    segment_alg_param.angle_interval = 0.01745;//PI * 1 / 180;
    segment_alg_param.angle_valve = 0.8727;//PI * 5 / 180;
    segment_alg_param.conservative_acc = 1000;
    segment_alg_param.time_factor_first = 2.5;
    segment_alg_param.time_factor_last = 3;
    segment_alg_param.is_fake_dynamics = true;
    segment_alg_param.max_cartesian_acc = 8000;
    segment_alg_param.coordinate_manager_ptr = coordinate_manager_ptr;
    segment_alg_param.tool_manager_ptr = tool_manager_ptr;
    segment_alg_param.kinematics_ptr = kinematics_ptr;
    segment_alg_param.dynamics_ptr = &dynamics;

    double joint_vel_max[6] = {1.33, 1.67, 1.81, 1.85, 1.81, 1.85};
    AxisType axis_type[9] = {ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS, ROTARY_AXIS};
    initSegmentAlgParam(&segment_alg_param, 6, axis_type, joint_vel_max);

    basic_alg::PoseEuler start;
    fst_mc::MotionTarget via;
    fst_mc::MotionTarget target;
    basic_alg::Joint start_joint;
    fst_mc::JointState start_state;
    fst_mc::PathCache path_cache;
    fst_mc::TrajectoryCache traj_cache;
    fst_mc::TrajectoryCache traj_cache_pause;
    fst_mc::TrajectoryCache traj_cache_movec_smooth;
    double acc_ratio = 0.5;
    double vel_ratio = 0.5;

/************* test movel fine  ****************/
#if 0
    start_joint[0] = 0;
    start_joint[1] = 0;
    start_joint[2] = 0;
    start_joint[3] = 0;
    start_joint[4] = -1.5708;
    start_joint[5] = 0;

    start_state.angle = start_joint;
    start_state.omega[0] = 0; start_state.omega[1] = 0; start_state.omega[2] = 0;
    start_state.omega[3] = 0; start_state.omega[4] = 0; start_state.omega[5] = 0;
    start_state.alpha[0] = 0; start_state.alpha[1] = 0; start_state.alpha[2] = 0;
    start_state.alpha[3] = 0; start_state.alpha[4] = 0; start_state.alpha[5] = 0;

    start.point_.x_ = 250;
    start.point_.y_ = 150;
    start.point_.z_ = 310;
    start.euler_.a_ = 0;
    start.euler_.b_ = 0;
    start.euler_.c_ = -PI;

    target.pose_target.point_.x_ = 550;
    target.pose_target.point_.y_ = -150;
    target.pose_target.point_.z_ = 610;
    target.pose_target.euler_.a_ = 0;
    target.pose_target.euler_.b_ = 0;
    target.pose_target.euler_.c_ = -PI;
    target.cnt = -1;
    target.vel = 1600;
    target.type = MOTION_LINE;

    path_cache.target.vel = 1600;
    path_cache.target.type = MOTION_LINE;

    planPathLine(start, target, path_cache);

    /************* test movel fine plan ****************/
    #if 0
    std::cout<<"path_cache_length = "<<path_cache.cache_length<<std::endl;
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        std::cout<< " " <<path_cache.cache[i].pose.point_.x_
            << " " <<path_cache.cache[i].pose.point_.y_
            << " " <<path_cache.cache[i].pose.point_.z_<<std::endl;
    }
    std::cout<<"smooth_out_index = "<<path_cache.smooth_out_index<<std::endl;
    #endif

    /************ test movel fine traj ******************/
    #if 0
    Joint result_joint;
    Joint ref_joint = start_joint;
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        kinematics_ptr->doIK(path_cache.cache[i].pose, ref_joint, path_cache.cache[i].joint);
        ref_joint = path_cache.cache[i].joint;
    }

    planTrajectory(path_cache, start_state, vel_ratio, acc_ratio, traj_cache);

    // std::cout<<"traj_cache.smooth_out_index = "<<traj_cache.smooth_out_index<<std::endl;
    // std::cout<<"traj_cache.cache_length = "<<traj_cache.cache_length<<std::endl;

    //fkToTraj(traj_cache);
    //printTraj(traj_cache, 1, 0.001, traj_cache.cache_length);
    #endif

    /************ test movel fine pause traj ******************/
    #if 0
    start_state.angle = start_joint;
    start_state.omega[0] = 0.001; start_state.omega[1] = 0.01; start_state.omega[2] = 0.001;
    start_state.omega[3] = 0.000; start_state.omega[4] = 0.01; start_state.omega[5] = 0.001;
    start_state.alpha[0] = 0.5; start_state.alpha[1] = 0.5; start_state.alpha[2] = 0.5;
    start_state.alpha[3] = 0.5; start_state.alpha[4] = 0.5; start_state.alpha[5] = 0.5;
    path_cache.target.vel = 1600;
    path_cache.target.type = MOTION_LINE;
    path_cache.smooth_in_index = -1;
    int path_stop_index = ceil(path_cache.cache_length / 3);

    for (int i= 0; i != model.link_num; ++i)
    {
        start_state.angle[i] = path_cache.cache[path_stop_index].joint[i];
    }

    printf("path_stop_index = %d, path_cache.cache_length = %d\n", path_stop_index, path_cache.cache_length);

    ErrorCode error_code = planPauseTrajectory(path_cache, start_state, acc_ratio, traj_cache_pause, path_stop_index);
    if (error_code != SUCCESS) cout << "ailed to pause:" << std::hex << error_code << endl;

    //fkToTraj(traj_cache_pause);

    for (int i = 0; i != model.link_num; ++i)
    {
        printf("link: %d\n", i);
        printTraj(traj_cache, i, 0.001, traj_cache.cache_length);
    }

    #endif
#endif

/************* test movej fine  ****************/
#if 1
    // start_joint[0] = 0;
    // start_joint[1] = 0;
    // start_joint[2] = 0;
    // start_joint[3] = 0;
    // start_joint[4] = -1.5708;
    // start_joint[5] = 0;

    start_joint[0] = -0.001810;
    start_joint[1] = 0.000037;
    start_joint[2] = 0.001021;
    start_joint[3] = 0.000020;
    start_joint[4] = 0.016061;
    start_joint[5] = -0.00101;

    start_state.angle[0] = -0.102207; start_state.angle[1] = 0.002102; start_state.angle[2] = 0.057643;
    start_state.angle[3] = 0.001148; start_state.angle[4] = 0.906681; start_state.angle[5] = -0.05738;
    start_state.omega[0] = 0; start_state.omega[1] = 0; start_state.omega[2] = 0;
    start_state.omega[3] = 0; start_state.omega[4] = 0; start_state.omega[5] = 0;
    start_state.alpha[0] = 0; start_state.alpha[1] = 0; start_state.alpha[2] = 0;
    start_state.alpha[3] = 0; start_state.alpha[4] = 0; start_state.alpha[5] = 0;

    // target.joint_target.j1_ = 1.82;
    // target.joint_target.j2_ = -0.22;
    // target.joint_target.j3_ = 1.28;
    // target.joint_target.j4_ = 1.33;
    // target.joint_target.j5_ = 0;
    // target.joint_target.j6_ = 1.5;

    //[ INFO][951890715. 34620]  target = -1.561110 -0.894680 0.775001 0.912329 0.228286 -0.96254

    target.joint_target.j1_ = -1.561110;
    target.joint_target.j2_ = -0.894680;
    target.joint_target.j3_ = 0.775001;
    target.joint_target.j4_ = 0.912329;
    target.joint_target.j5_ = 0.228286;
    target.joint_target.j6_ = -0.96254;
    target.cnt = -1;
    target.vel = 0.5;
    target.type = MOTION_JOINT;

    path_cache.target.vel = 0.5;
    path_cache.target.type = MOTION_JOINT;

    planPathJoint(start_state.angle, target, path_cache);

    /************* test fine path ****************/
    #if 0
    std::cout<<"path_cache_length = "<<path_cache.cache_length<<std::endl;
    printMoveJPoint(path_cache);
    std::cout << "smooth_out_index = " << path_cache.smooth_out_index << std::endl;
    #endif

    /************ test fine traj ******************/
    #if 0
    planTrajectory(path_cache, start_state, target.vel, acc_ratio, traj_cache);
    //fkToTraj(traj_cache);
    #endif

    /************ test fine pause traj ******************/
    #if 1
    // start_state.omega[0] = 0.001; start_state.omega[1] = 0.01; start_state.omega[2] = 0.001;
    // start_state.omega[3] = 0.000; start_state.omega[4] = 0.01; start_state.omega[5] = 0.001;
    // start_state.alpha[0] = 0.5; start_state.alpha[1] = 0.5; start_state.alpha[2] = 0.5;
    // start_state.alpha[3] = 0.5; start_state.alpha[4] = 0.5; start_state.alpha[5] = 0.5;
    start_state.angle[0] = -0.588508; start_state.angle[1] = -0.296826; start_state.angle[2] = 0.296763;
    start_state.angle[3] = 0.304875; start_state.angle[4] = 0.680549; start_state.angle[5] = -0.35910;
    start_state.omega[0] = -0.171243; start_state.omega[1] = -0.105263; start_state.omega[2] = 0.084202;
    start_state.omega[3] = 0.106953; start_state.omega[4] = -0.079629; start_state.omega[5] = -0.10624;
    start_state.alpha[0] = -0.023257; start_state.alpha[1] = -0.014296; start_state.alpha[2] = 0.011435;
    start_state.alpha[3] = 0.014525; start_state.alpha[4] = -0.010814; start_state.alpha[5] = -0.01442;


    //[ INFO][951890715.939284]angle: -0.588508 -0.296826 0.296763 0.304875 0.680549 -0.35910
    //[ INFO][951890715.939344]omega: -0.171243 -0.105263 0.084202 0.106953 -0.079629 -0.10624
    //[ INFO][951890715.940009]alpha: -0.023257 -0.014296 0.011435 0.014525 -0.010814 -0.01442

    path_cache.target.vel = 0.5;
    path_cache.target.type = MOTION_JOINT;
    path_cache.smooth_in_index = -1;
    int path_stop_index = 28;//ceil(path_cache.cache_length / 3);

    // for (int i= 0; i != model.link_num; ++i)
    // {
        // start_state.angle[i] = path_cache.cache[path_stop_index].joint[i];
    // }

    printf("path_stop_index = %d, path_cache.cache_length = %d\n", path_stop_index, path_cache.cache_length);

    ErrorCode error_code = planPauseTrajectory(path_cache, start_state, acc_ratio, traj_cache_pause, path_stop_index);
    if (error_code != SUCCESS) cout << "failed to pause:" << std::hex << error_code << endl;

    //fkToTraj(traj_cache_pause);
    printTraj(traj_cache_pause, 0, 0.001, traj_cache_pause.cache_length);
    // for (int i = 0; i != model.link_num; ++i)
    // {
        // printf("link: %d\n", i);
        // printTraj(traj_cache_pause, i, 0.001, traj_cache_pause.cache_length);
    //}

    #endif
#endif

/************* test movec fine ****************/
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
    path_cache.smooth_in_index = -1;

    planPathCircle(start, target, path_cache);

    /************* test movel fine plan ****************/
    #if 0
    std::cout<<"path_cache_length = "<<path_cache.cache_length<<std::endl;
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        std::cout<< " " <<path_cache.cache[i].pose.point_.x_
            << " " <<path_cache.cache[i].pose.point_.y_
            << " " <<path_cache.cache[i].pose.point_.z_<<std::endl;
    }
    std::cout<<"smooth_out_index = "<<path_cache.smooth_out_index<<std::endl;
    #endif

    /************ test movel fine traj ******************/
    #if 1
    Joint result_joint;
    Joint ref_joint = start_joint;
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        kinematics_ptr->doIK(path_cache.cache[i].pose, ref_joint, path_cache.cache[i].joint);
        ref_joint = path_cache.cache[i].joint;
    }

    planTrajectory(path_cache, start_state, vel_ratio, acc_ratio, traj_cache);

    // std::cout<<"traj_cache.smooth_out_index = "<<traj_cache.smooth_out_index<<std::endl;
    // std::cout<<"traj_cache.cache_length = "<<traj_cache.cache_length<<std::endl;

    fkToTraj(traj_cache);
    #endif

    /************ test movel fine pause traj ******************/
    #if 0
    start_state.omega[0] = 0.001; start_state.omega[1] = 0.01; start_state.omega[2] = 0.001;
    start_state.omega[3] = 0.000; start_state.omega[4] = 0.01; start_state.omega[5] = 0.001;
    start_state.alpha[0] = 0.5; start_state.alpha[1] = 0.5; start_state.alpha[2] = 0.5;
    start_state.alpha[3] = 0.5; start_state.alpha[4] = 0.5; start_state.alpha[5] = 0.5;
    path_cache.target.vel = 1600;
    path_cache.target.type = MOTION_CIRCLE;
    path_cache.smooth_in_index = -1;
    int path_stop_index = ceil(path_cache.cache_length / 3);

    for (int i= 0; i != model.link_num; ++i)
    {
        start_state.angle[i] = path_cache.cache[path_stop_index].joint[i];
    }

    printf("path_stop_index = %d, path_cache.cache_length = %d\n", path_stop_index, path_cache.cache_length);

    ErrorCode error_code = planPauseTrajectory(path_cache, start_state, acc_ratio, traj_cache_pause, path_stop_index);
    if (error_code != SUCCESS) cout << "ailed to pause:" << std::hex << error_code << endl;

    fkToTraj(traj_cache_pause);
    #endif

    #if 0
    for (int i = 0; i != model.link_num; ++i)
    {
        printf("link: %d\n", i);
        printTraj(traj_cache, i, 0.001, traj_cache.cache_length);
    }
    #endif
    
#endif


/************* test movel smooth ****************/
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

    start.point_.x_ = 300;
    start.point_.y_ = 0;
    start.point_.z_ = 500;
    start.euler_.a_ = 0;
    start.euler_.b_ = 0;
    start.euler_.c_ = 0;

    /**** for l2l via ***********/
    #if 0
    via.pose_target.point_.x_ = 300;
    via.pose_target.point_.y_ = 300;
    via.pose_target.point_.z_ = 500;
    via.pose_target.euler_.a_ = 0;
    via.pose_target.euler_.b_ = 0;
    via.pose_target.euler_.c_ = 0;
    via.type = MOTION_LINE;
    via.cnt = 0.1;
    via.vel = 100;
    #endif

    /**** for j2l via ***********/
    #if 0
    PoseEuler temp_pose;
    temp_pose.point_.x_ = 300;
    temp_pose.point_.y_ = 300;
    temp_pose.point_.z_ = 500;
    temp_pose.euler_.a_ = 0;
    temp_pose.euler_.b_ = 0;
    temp_pose.euler_.c_ = 0;

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

    segment_alg_param.kinematics_ptr->doIK(temp_pose, ref_joint, via.joint_target);
    via.type = MOTION_JOINT;
    via.cnt = 0.1;
    via.vel = 100;
    #endif

    /**** for c2l via ***********/
    #if 1
    via.circle_target.pose2.point_.x_ = 300;
    via.circle_target.pose2.point_.y_ = 300;
    via.circle_target.pose2.point_.z_ = 500;
    via.circle_target.pose2.euler_.a_ = 0;
    via.circle_target.pose2.euler_.b_ = 0;
    via.circle_target.pose2.euler_.c_ = 0;
    via.type = MOTION_CIRCLE;
    via.cnt = 0.1;
    via.vel = 100;
    #endif

    target.pose_target.point_.x_ = -300;
    target.pose_target.point_.y_ = 300;
    target.pose_target.point_.z_ = 500;
    target.pose_target.euler_.a_ = 0;
    target.pose_target.euler_.b_ = 0;
    target.pose_target.euler_.c_ = 0;
    target.cnt = 0.3;
    target.vel = 1000;
    target.type = MOTION_LINE;
    ErrorCode err = planPathSmoothLine(start, via, target, path_cache);
    if(err != SUCCESS)
    {
        printf("planPathSmoothLine failed, err = 0x%llx\n", err);
        return err;
    }

    /************* test path ****************/
    #if 0
    std::cout<<"path_cache_length = "<<path_cache.cache_length<<std::endl;
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        std::cout<< " " <<path_cache.cache[i].pose.point_.x_
            << " " <<path_cache.cache[i].pose.point_.y_
            << " " <<path_cache.cache[i].pose.point_.z_<<std::endl;
    }
    std::cout<<"smooth_out_index = "<<path_cache.smooth_out_index<<std::endl;
    #endif

    /************ test traj ******************/
    #if 1
    Joint result_joint;
    Joint reference_joint = start_joint;

    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        kinematics_ptr->doIK(path_cache.cache[i].pose, reference_joint, path_cache.cache[i].joint);
        reference_joint = path_cache.cache[i].joint;
    }
    path_cache.target.cnt = target.cnt;
    path_cache.target.vel = 1000;
    path_cache.target.type = MOTION_LINE;

    ErrorCode error_code = planTrajectorySmooth(path_cache, start_state, via, vel_ratio, acc_ratio, traj_cache);

    if (error_code != SUCCESS)
    {
        printf("failed to plan traj : 0x%llx\n", error_code);
        return -1;
    }

    //fkToTraj(traj_cache);
    #endif

    /************ test pause traj ******************/
    #if 1
    start_state.omega[0] = 0.1; start_state.omega[1] = 0.1; start_state.omega[2] = 0.1;
    start_state.omega[3] = 0.000; start_state.omega[4] = 0.01; start_state.omega[5] = 0.1;
    start_state.alpha[0] = 0.5; start_state.alpha[1] = 0.5; start_state.alpha[2] = 0.5;
    start_state.alpha[3] = 0.5; start_state.alpha[4] = 0.5; start_state.alpha[5] = 0.5;

    int path_stop_index = ceil(path_cache.cache_length / 3);

    for (int i= 0; i != model.link_num; ++i)
    {
        start_state.angle[i] = path_cache.cache[path_stop_index].joint[i];
        printf("pause angle[%d] = %lf\n", i, start_state.angle[i]);
    }

    printf("path_stop_index = %d, path_in_index = %d, path_out_index = %d, path_cache.cache_length = %d\n", 
        path_stop_index, path_cache.smooth_in_index, path_cache.smooth_out_index, path_cache.cache_length);

    error_code = planPauseTrajectory(path_cache, start_state, acc_ratio, traj_cache_pause, path_stop_index);
    if (error_code != SUCCESS)
    {
        cout << "ailed to pause:" << std::hex << error_code << endl;
        return -1;
    }

    //fkToTraj(traj_cache_pause);
    #endif
    #if 1
    for (int i = 0; i != model.link_num; ++i)
    {
        printf("linklink: %d\n", i);
        printTraj(traj_cache_pause, i, 0.01, traj_cache_pause.cache_length);
    }
    #endif

#endif
/************* test movej smooth ****************/
#if 0
    basic_alg::Joint js, jv, je;
    js[0] = 0; jv[0] = M_PI/2; je[0] = M_PI;
    js[1] = 0; jv[1] = 0;    je[1] = 0;
    js[2] = 0; jv[2] = 0;    je[2] = 0;
    js[3] = 0; jv[3] = 0;    je[3] = 0;
    js[4] = M_PI/4; jv[4] = M_PI/4;    je[4] = M_PI/4;
    js[5] = 0; jv[5] = 0;    je[5] = 0;

    start_joint = js;
    start_state.angle = js;
    start_state.omega[0] = 0; start_state.omega[1] = 0; start_state.omega[2] = 0;
    start_state.omega[3] = 0; start_state.omega[4] = 0; start_state.omega[5] = 0;
    start_state.alpha[0] = 0; start_state.alpha[1] = 0; start_state.alpha[2] = 0;
    start_state.alpha[3] = 0; start_state.alpha[4] = 0; start_state.alpha[5] = 0;

    /**** for l2j via ***********/
    #if 0
    kinematics_ptr->doFK(jv, via.pose_target);
    via.type = MOTION_LINE;
    via.cnt = 0.5;
    via.vel = 200;
    #endif

    /**** for j2j via ***********/
    #if 0
    via.joint_target = jv;
    via.type = MOTION_JOINT;
    via.cnt = 0.3;
    #endif

    /**** for c2j via ***********/
    #if 1
    kinematics_ptr->doFK(jv, via.circle_target.pose2);
    via.type = MOTION_CIRCLE;
    via.vel = 200;
    via.cnt = 0.1;
    #endif

    target.joint_target = je;
    target.cnt = 0.3;
    target.vel = 0.1;
    target.type = MOTION_JOINT;

    ErrorCode err = planPathSmoothJoint(start_joint, via, target, path_cache);
    if(err != SUCCESS)
    {
        printf("planPathSmoothLine failed, err = 0x%llx\n", err);
        return err;
    }

    /************* test path ****************/
    #if 0
    std::cout<<"path_cache_length = "<<path_cache.cache_length<<std::endl;
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        kinematics_ptr->doFK(path_cache.cache[i].joint, path_cache.cache[i].pose);
    }

    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        std::cout<< " " <<path_cache.cache[i].pose.point_.x_
            << " " <<path_cache.cache[i].pose.point_.y_
            << " " <<path_cache.cache[i].pose.point_.z_<<std::endl;
    }
    std::cout<<"smooth_out_index = "<<path_cache.smooth_out_index<<std::endl;
    #endif

    /************ test traj ******************/
    #if 1
    path_cache.target.cnt = target.cnt;
    path_cache.target.vel = target.vel;
    path_cache.target.type = MOTION_JOINT;

    ErrorCode error_code = planTrajectorySmooth(path_cache, start_state, via, vel_ratio, acc_ratio, traj_cache);

    if (error_code != SUCCESS)
    {
        printf("failed to plan traj : 0x%llx\n", error_code);
        return -1;
    }

    fkToTraj(traj_cache);
    #endif

    /************ test pause traj ******************/
    #if 0
    start_state.omega[0] = 0.1; start_state.omega[1] = 0.1; start_state.omega[2] = 0.1;
    start_state.omega[3] = 0.000; start_state.omega[4] = 0.01; start_state.omega[5] = 0.1;
    start_state.alpha[0] = 0.5; start_state.alpha[1] = 0.5; start_state.alpha[2] = 0.5;
    start_state.alpha[3] = 0.5; start_state.alpha[4] = 0.5; start_state.alpha[5] = 0.5;

    int path_stop_index = ceil(path_cache.cache_length / 3);

    for (int i= 0; i != model.link_num; ++i)
    {
        start_state.angle[i] = path_cache.cache[path_stop_index].joint[i];
        printf("pause angle[%d] = %lf\n", i, start_state.angle[i]);
    }

    printf("path_stop_index = %d, path_in_index = %d, path_out_index = %d, path_cache.cache_length = %d\n", 
        path_stop_index, path_cache.smooth_in_index, path_cache.smooth_out_index, path_cache.cache_length);

    error_code = planPauseTrajectory(path_cache, start_state, acc_ratio, traj_cache_pause, path_stop_index);
    if (error_code != SUCCESS)
    {
        cout << "ailed to pause:" << std::hex << error_code << endl;
        return -1;
    }

    fkToTraj(traj_cache_pause);
    #endif
    #if 0
    for (int i = 0; i != model.link_num; ++i)
    {
        printf("linklink: %d\n", i);
        printTraj(traj_cache_pause, i, 0.01, traj_cache_pause.cache_length);
    }
    #endif

#endif

/************* test movec smooth ****************/
#if 0
    start.point_.x_ = 300;
    start.point_.y_ = -300;
    start.point_.z_ = 500;
    start.euler_.a_ = 0;
    start.euler_.b_ = 0;
    start.euler_.c_ = 0;

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

    segment_alg_param.kinematics_ptr->doIK(start, ref_joint, start_joint);

    // start_joint[0] = 0.643499;
    // start_joint[1] = 0.056281;
    // start_joint[2] = -0.979224;
    // start_joint[3] = 0.00001;
    // start_joint[4] = -0.647849;
    // start_joint[5] = 0.64349;

    //kinematics_ptr->doFK(start_joint, start);
    start_state.angle = start_joint;
    start_state.omega[0] = 0; start_state.omega[1] = 0; start_state.omega[2] = 0;
    start_state.omega[3] = 0; start_state.omega[4] = 0; start_state.omega[5] = 0;
    start_state.alpha[0] = 0; start_state.alpha[1] = 0; start_state.alpha[2] = 0;
    start_state.alpha[3] = 0; start_state.alpha[4] = 0; start_state.alpha[5] = 0;

    // start.point_.x_ = 300;
    // start.point_.y_ = 0;
    // start.point_.z_ = 500;
    // start.euler_.a_ = 0;
    // start.euler_.b_ = 0;
    // start.euler_.c_ = 0;

    /**** for l2l via ***********/
    #if 0
    via.pose_target.point_.x_ = 300;
    via.pose_target.point_.y_ = 300;
    via.pose_target.point_.z_ = 500;
    via.pose_target.euler_.a_ = 0;
    via.pose_target.euler_.b_ = 0;
    via.pose_target.euler_.c_ = 0;
    via.type = MOTION_LINE;
    via.cnt = 0.1;
    via.vel = 100;
    #endif

    /**** for j2l via ***********/
    #if 0
    PoseEuler temp_pose;
    temp_pose.point_.x_ = 300;
    temp_pose.point_.y_ = 300;
    temp_pose.point_.z_ = 500;
    temp_pose.euler_.a_ = 0;
    temp_pose.euler_.b_ = 0;
    temp_pose.euler_.c_ = 0;

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

    segment_alg_param.kinematics_ptr->doIK(temp_pose, ref_joint, via.joint_target);
    via.type = MOTION_JOINT;
    via.cnt = 0.1;
    via.vel = 100;
    #endif

    /**** for c2l via ***********/
    #if 1
    via.circle_target.pose2.point_.x_ = 300;
    via.circle_target.pose2.point_.y_ = 300;
    via.circle_target.pose2.point_.z_ = 500;
    via.circle_target.pose2.euler_.a_ = 0;
    via.circle_target.pose2.euler_.b_ = 0;
    via.circle_target.pose2.euler_.c_ = 0;
    via.type = MOTION_CIRCLE;
    via.cnt = 0.2;
    via.vel = 1000;
    #endif

    target.circle_target.pose1.point_.x_ = -300;
    target.circle_target.pose1.point_.y_ = 300;
    target.circle_target.pose1.point_.z_ = 500;
    target.circle_target.pose1.euler_.a_ = 0;
    target.circle_target.pose1.euler_.b_ = 0;
    target.circle_target.pose1.euler_.c_ = 0;

    target.circle_target.pose2.point_.x_ = -300;
    target.circle_target.pose2.point_.y_ = -300;
    target.circle_target.pose2.point_.z_ = 500;
    target.circle_target.pose2.euler_.a_ = 0;
    target.circle_target.pose2.euler_.b_ = 0;
    target.circle_target.pose2.euler_.c_ = 0;
    target.cnt = 0.2;
    target.vel = 500;
    target.type = MOTION_CIRCLE;
    ErrorCode err = planPathSmoothCircle(start, via, target, path_cache);
    if(err != SUCCESS)
    {
        printf("planPathSmoothLine failed, err = 0x%llx\n", err);
        return err;
    }

    /************* test path ****************/
    #if 0
    std::cout<<"path_cache_length = "<<path_cache.cache_length<<std::endl;
    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        std::cout<< " " <<path_cache.cache[i].pose.point_.x_
            << " " <<path_cache.cache[i].pose.point_.y_
            << " " <<path_cache.cache[i].pose.point_.z_<<std::endl;
    }
    std::cout<<"smooth_out_index = "<<path_cache.smooth_out_index<<std::endl;
    #endif

    /************ test traj ******************/
    #if 1
    Joint result_joint;
    Joint reference_joint = start_joint;

    for(int i = 0; i < path_cache.cache_length; ++i)
    {
        kinematics_ptr->doIK(path_cache.cache[i].pose, reference_joint, path_cache.cache[i].joint);
        reference_joint = path_cache.cache[i].joint;
    }
    path_cache.target.cnt = target.cnt;
    path_cache.target.vel = 500;
    path_cache.target.type = MOTION_CIRCLE;

    ErrorCode error_code = planTrajectorySmooth(path_cache, start_state, via, vel_ratio, acc_ratio, traj_cache_movec_smooth);

    if (error_code != SUCCESS)
    {
        printf("failed to plan traj : 0x%llx\n", error_code);
        return -1;
    }

    fkToTraj(traj_cache_movec_smooth);
    #endif

    /************ test pause traj ******************/
    #if 0
    start_state.omega[0] = 0.1; start_state.omega[1] = 0.1; start_state.omega[2] = 0.1;
    start_state.omega[3] = 0.000; start_state.omega[4] = 0.01; start_state.omega[5] = 0.1;
    start_state.alpha[0] = 0.5; start_state.alpha[1] = 0.5; start_state.alpha[2] = 0.5;
    start_state.alpha[3] = 0.5; start_state.alpha[4] = 0.5; start_state.alpha[5] = 0.5;

    int path_stop_index = ceil(path_cache.cache_length / 3);

    for (int i= 0; i != model.link_num; ++i)
    {
        start_state.angle[i] = path_cache.cache[path_stop_index].joint[i];
        printf("pause angle[%d] = %lf\n", i, start_state.angle[i]);
    }

    printf("path_stop_index = %d, path_in_index = %d, path_out_index = %d, path_cache.cache_length = %d\n", 
        path_stop_index, path_cache.smooth_in_index, path_cache.smooth_out_index, path_cache.cache_length);

    error_code = planPauseTrajectory(path_cache, start_state, acc_ratio, traj_cache_pause, path_stop_index);
    if (error_code != SUCCESS)
    {
        cout << "ailed to pause:" << std::hex << error_code << endl;
        return -1;
    }

    //fkToTraj(traj_cache_pause);
    #endif
    #if 0
    for (int i = 0; i != model.link_num; ++i)
    {
        printf("linklink: %d\n", i);
        printTraj(traj_cache_pause, i, 0.01, traj_cache_pause.cache_length);
    }
    #endif

#endif
    return 0;
}