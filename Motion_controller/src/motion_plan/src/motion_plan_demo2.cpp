/*************************************************************************
	> File Name: motion_plan_demo1.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年01月09日 星期二 10时15分23秒
 ************************************************************************/

#include <iostream>
#include <time.h>
#include <string.h>
#include <fstream>

#include <parameter_manager/parameter_manager_param_group.h>

#include <motion_plan_arm_group.h>
#include <motion_plan_matrix.h>
#include <motion_plan_kinematics.h>
#include <motion_plan_variable.h>
#include <motion_plan_motion_command.h>
#include <motion_plan_traj_plan.h>
#include <motion_plan_reuse.h>


using namespace std;
using namespace fst_controller;
using namespace fst_algorithm;

int main(int argc, char **argv)
{
    g_cycle_time = 0.001;                   // s
    g_global_speed_ratio = 1.0;
    g_global_acc_ratio = 1.0;
    g_cart_vel_default = 1000.0;            // mm/s
    g_cart_acc_default = 16000.0;           // mm/s2
    g_cart_vel_max = 4000.0;                // mm/s
    g_cart_acc_max = 48000.0;               // mm/s2
    g_cart_acc_reference = 4000.0;          // mm/s2
    g_orientation_omega_reference = 1.0;    // rad
    g_cycle_distance = 1.0;                 // mm
    
    g_ort_linear_polation_threshold_ = 0.01;    // rad

    g_user_frame.identityMatrix();
    g_tool_frame.identityMatrix();
    g_user_frame_inverse.identityMatrix();
    g_tool_frame_inverse.identityMatrix();

    fst_parameter::ParamGroup param;

    g_dh_mat[0][0] = 0;
    g_dh_mat[0][1] = 0;
    g_dh_mat[0][2] = 330;
    g_dh_mat[0][3] = 0;
    g_dh_mat[1][0] = PI/2;
    g_dh_mat[1][1] = 50;
    g_dh_mat[1][2] = 0;
    g_dh_mat[1][3] = PI/2;
    g_dh_mat[2][0] = 0;
    g_dh_mat[2][1] = 330;
    g_dh_mat[2][2] = 0;
    g_dh_mat[2][3] = 0;
    g_dh_mat[3][0] = PI/2;
    g_dh_mat[3][1] = 35;
    g_dh_mat[3][2] = 330;
    g_dh_mat[3][3] = 0;
    g_dh_mat[4][0] = -PI/2;
    g_dh_mat[4][1] = 0;
    g_dh_mat[4][2] = 0;
    g_dh_mat[4][3] = 0;
    g_dh_mat[5][0] = PI/2;
    g_dh_mat[5][1] = 0;
    g_dh_mat[5][2] = 77.5;
    g_dh_mat[5][3] = 0;

    param.loadParamFile("share/configuration/configurable/soft_constraints.yaml");

    param.getParam("soft_constraint.j1.upper", g_soft_constraint.j1.upper);
    param.getParam("soft_constraint.j1.lower", g_soft_constraint.j1.lower);
    param.getParam("soft_constraint.j2.upper", g_soft_constraint.j2.upper);
    param.getParam("soft_constraint.j2.lower", g_soft_constraint.j2.lower);
    param.getParam("soft_constraint.j3.upper", g_soft_constraint.j3.upper);
    param.getParam("soft_constraint.j3.lower", g_soft_constraint.j3.lower);
    param.getParam("soft_constraint.j4.upper", g_soft_constraint.j4.upper);
    param.getParam("soft_constraint.j4.lower", g_soft_constraint.j4.lower);
    param.getParam("soft_constraint.j5.upper", g_soft_constraint.j5.upper);
    param.getParam("soft_constraint.j5.lower", g_soft_constraint.j5.lower);
    param.getParam("soft_constraint.j6.upper", g_soft_constraint.j6.upper);
    param.getParam("soft_constraint.j6.lower", g_soft_constraint.j6.lower);

    param.getParam("soft_constraint.j1.omega_max", g_soft_constraint.j1.max_omega);
    param.getParam("soft_constraint.j1.alpha_max", g_soft_constraint.j1.max_alpha);
    param.getParam("soft_constraint.j2.omega_max", g_soft_constraint.j2.max_omega);
    param.getParam("soft_constraint.j2.alpha_max", g_soft_constraint.j2.max_alpha);
    param.getParam("soft_constraint.j3.omega_max", g_soft_constraint.j3.max_omega);
    param.getParam("soft_constraint.j3.alpha_max", g_soft_constraint.j3.max_alpha);
    param.getParam("soft_constraint.j4.omega_max", g_soft_constraint.j4.max_omega);
    param.getParam("soft_constraint.j4.alpha_max", g_soft_constraint.j4.max_alpha);
    param.getParam("soft_constraint.j5.omega_max", g_soft_constraint.j5.max_omega);
    param.getParam("soft_constraint.j5.alpha_max", g_soft_constraint.j5.max_alpha);
    param.getParam("soft_constraint.j6.omega_max", g_soft_constraint.j6.max_omega);
    param.getParam("soft_constraint.j6.alpha_max", g_soft_constraint.j6.max_alpha);

    Joint       jnt;
    PoseEuler   pose;

    jnt.j1 = 0.1;
    jnt.j2 = -0.2;
    jnt.j3 = 0.25;
    jnt.j4 = 0.17;
    jnt.j5 = -1.5708;
    jnt.j6 = -0.38;
    fst_algorithm::forwardKinematics(jnt, pose);
    
    cout << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << ", "
         << pose.orientation.a << ", " << pose.orientation.b << ", " << pose.orientation.c << endl;

    Joint ref, res;
    ref.j1 = 0.0;
    ref.j2 = 0.0;
    ref.j3 = 0.0;
    ref.j4 = 0.0;
    ref.j5 = 0.0;
    ref.j6 = 0.0;
    cout << "IK res=" << fst_algorithm::inverseKinematics(pose, ref, res) << endl;
    cout << res.j1 << ", " << res.j2 << ", " << res.j3 << ", " << res.j4 << ", " << res.j5 << ", " << res.j6 << endl;

/////////////////////////////////////////////////////////////////////////////////////
    


    Joint start_joint;
    start_joint.j1 = 0.0;
    start_joint.j2 = 0.0;
    start_joint.j3 = 0.0;
    start_joint.j4 = 0.0;
    start_joint.j5 = -1.5708;
    start_joint.j6 = 0.0;

    MotionTarget target;
    target.type = MOTION_LINE;
    target.cnt = 0;
    target.vel = 500.0;
    target.acc = 16000.0;
    target.pose_target.position.x = 480.0;
    target.pose_target.position.y = 160.0;
    target.pose_target.position.z = 450.0;
    target.pose_target.orientation.a = 0.0;
    target.pose_target.orientation.b = 0.0;
    target.pose_target.orientation.c = 3.1416;

    ErrorCode err = SUCCESS;
    MotionCommand cmd;
    err = cmd.setStartJoint(start_joint);
    err = cmd.setTarget(target, 15);
    err = cmd.planPath();

    vector<PathPoint> path;
    
    int len = cmd.getPathLength();
    err = cmd.pickPathPoint(len, path);
    FST_INFO("path_len=%d, pick_len=%d", len, path.size());

    FST_INFO("type=%d, ID=%d, source=%lp", path[0].type, path[0].source->getMotionID(), path[0].source);

    
    for (int i = 0; i < path.size(); ++i) {
        Pose &p = path[i].pose;
        //FST_INFO("%3d: %f,%f,%f %f,%f,%f,%f", path[i].stamp, p.position.x, p.position.y, p.position.z,\
                p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
        
    }
    

    Angle rf[6] = {start_joint.j1, start_joint.j2, start_joint.j3, start_joint.j4, start_joint.j5, start_joint.j6};
    
    vector<ControlPoint> traj;
    traj.resize(path.size());
    
    int i;
    
    for (i = 0; i < traj.size(); i++) {
        err = inverseKinematics(path[i].pose, rf, traj[i].point.joint);
        if (err == SUCCESS) {
            traj[i].path_point = path[i];
            traj[i].time_from_start = -1;
            traj[i].duration = -1;
            traj[i].expect_duration = -1;
            traj[i].brake = false;

            //FST_INFO("%3d: %f,%f,%f,%f,%f,%f", i,\
                        traj[i].point.joint[0], traj[i].point.joint[1],\
                        traj[i].point.joint[2], traj[i].point.joint[3],\
                        traj[i].point.joint[4], traj[i].point.joint[5]);
            memcpy(rf, traj[i].point.joint, 6 * sizeof(double));
        }
        else
            break;
    }

    ControlPoint start_ctl_pnt;
    start_ctl_pnt.path_point.type = MOTION_LINE;
    start_ctl_pnt.path_point.stamp = 0;
    start_ctl_pnt.path_point.source = &cmd;
    start_ctl_pnt.path_point.pose = forwardKinematics(start_joint);
    start_ctl_pnt.time_from_start = 0;
    start_ctl_pnt.duration = 0;
    start_ctl_pnt.expect_duration = 0;
    start_ctl_pnt.brake = false;
    memset(start_ctl_pnt.point.joint, 0, 9 * sizeof(double));
    memset(start_ctl_pnt.point.omega, 0, 9 * sizeof(double));
    memset(start_ctl_pnt.point.alpha, 0, 9 * sizeof(double));
    memcpy(start_ctl_pnt.point.joint, &start_joint, 6 * sizeof(double));
    
    FST_INFO("max_alpha:%f,%f,%f,%f,%f,%f", g_soft_constraint.j1.max_alpha,
                                            g_soft_constraint.j2.max_alpha,
                                            g_soft_constraint.j3.max_alpha,
                                            g_soft_constraint.j4.max_alpha,
                                            g_soft_constraint.j5.max_alpha,
                                            g_soft_constraint.j6.max_alpha);
    createTrajectoryFromPath(start_ctl_pnt, traj[0]);

    ofstream os("test.csv");
    os  << "0,"\
        << traj[0].path_point.pose.position.x << ","\
        << traj[0].path_point.pose.position.y << ","\
        << traj[0].path_point.pose.position.z << ","\
        << traj[0].path_point.pose.orientation.w << ","\
        << traj[0].path_point.pose.orientation.x << ","\
        << traj[0].path_point.pose.orientation.y << ","\
        << traj[0].path_point.pose.orientation.z << ","\
        << traj[0].point.joint[0] << ","\
        << traj[0].point.joint[1] << ","\
        << traj[0].point.joint[2] << ","\
        << traj[0].point.joint[3] << ","\
        << traj[0].point.joint[4] << ","\
        << traj[0].point.joint[5] << ","\
        << traj[0].point.omega[0] << ","\
        << traj[0].point.omega[1] << ","\
        << traj[0].point.omega[2] << ","\
        << traj[0].point.omega[3] << ","\
        << traj[0].point.omega[4] << ","\
        << traj[0].point.omega[5] << ","\
        << traj[0].point.alpha[0] << ","\
        << traj[0].point.alpha[1] << ","\
        << traj[0].point.alpha[2] << ","\
        << traj[0].point.alpha[3] << ","\
        << traj[0].point.alpha[4] << ","\
        << traj[0].point.alpha[5] << endl;

    for (i = 0; i + 1 < traj.size(); i++) {
        if (traj[i + 1].path_point.speed_down == false) {
            createTrajectoryFromPath(traj[i], traj[i + 1]);
                os  << i + 1 << ","\
                    << traj[i+1].path_point.pose.position.x << ","\
                    << traj[i+1].path_point.pose.position.y << ","\
                    << traj[i+1].path_point.pose.position.z << ","\
                    << traj[i+1].path_point.pose.orientation.w << ","\
                    << traj[i+1].path_point.pose.orientation.x << ","\
                    << traj[i+1].path_point.pose.orientation.y << ","\
                    << traj[i+1].path_point.pose.orientation.z << ","\
                    << traj[i+1].point.joint[0] << ","\
                    << traj[i+1].point.joint[1] << ","\
                    << traj[i+1].point.joint[2] << ","\
                    << traj[i+1].point.joint[3] << ","\
                    << traj[i+1].point.joint[4] << ","\
                    << traj[i+1].point.joint[5] << ","\
                    << traj[i+1].point.omega[0] << ","\
                    << traj[i+1].point.omega[1] << ","\
                    << traj[i+1].point.omega[2] << ","\
                    << traj[i+1].point.omega[3] << ","\
                    << traj[i+1].point.omega[4] << ","\
                    << traj[i+1].point.omega[5] << ","\
                    << traj[i+1].point.alpha[0] << ","\
                    << traj[i+1].point.alpha[1] << ","\
                    << traj[i+1].point.alpha[2] << ","\
                    << traj[i+1].point.alpha[3] << ","\
                    << traj[i+1].point.alpha[4] << ","\
                    << traj[i+1].point.alpha[5] << endl;
        }
        else {
            FST_INFO("stamp=%d, speed-down=%d", i + 1, traj[i + 1].path_point.speed_down); 
        }
    }
    os.close();

    return 0;
}

