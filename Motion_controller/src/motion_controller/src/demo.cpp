
#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <std_msgs/String.h>
#include <time.h>

#include <motion_controller/arm_group.h>
#include <parameter_manager/parameter_manager_param_group.h>

#include <vector>

using std::string;
using std::cout;
using std::endl;

void displayTrajectory(const std::vector<fst_controller::JointPoint> &traj, ros::Publisher &publisher)
{
    
    moveit_msgs::DisplayTrajectory disp;
    disp.model_id = "fr701";
    
    disp.trajectory.resize(1);
    disp.trajectory[0].joint_trajectory.header.frame_id = "/base_link";
    disp.trajectory[0].joint_trajectory.joint_names.resize(6);
    disp.trajectory[0].joint_trajectory.joint_names[0] = "j1";
    disp.trajectory[0].joint_trajectory.joint_names[1] = "j2";
    disp.trajectory[0].joint_trajectory.joint_names[2] = "j3";
    disp.trajectory[0].joint_trajectory.joint_names[3] = "j4";
    disp.trajectory[0].joint_trajectory.joint_names[4] = "j5";
    disp.trajectory[0].joint_trajectory.joint_names[5] = "j6";
    int num = traj.size();
    disp.trajectory[0].joint_trajectory.points.resize(num);
    for(int cnt=0;cnt<num;cnt++) {
	disp.trajectory[0].joint_trajectory.points[cnt].positions.resize(6);
	disp.trajectory[0].joint_trajectory.points[cnt].positions[0] = traj[cnt].joints.j1;
	disp.trajectory[0].joint_trajectory.points[cnt].positions[1] = traj[cnt].joints.j2;
	disp.trajectory[0].joint_trajectory.points[cnt].positions[2] = traj[cnt].joints.j3;
	disp.trajectory[0].joint_trajectory.points[cnt].positions[3] = traj[cnt].joints.j4;
	disp.trajectory[0].joint_trajectory.points[cnt].positions[4] = traj[cnt].joints.j5;
	disp.trajectory[0].joint_trajectory.points[cnt].positions[5] = traj[cnt].joints.j6;
    }
    publisher.publish(disp);
    cout << "disp sended" << endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_controller_demo");
    ros::NodeHandle node;
    ros::Publisher publisher = node.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1000);
    ros::Rate loop_rate(1);

    fst_parameter::ParamGroup param("share/motion_controller/config/motion_controller.yaml");
    if (!param.uploadParam()) {
        printf("error code=0x%llx\n", param.getLastError());
        return 0;
    }
    /*
    std::vector<double> data;
    bool ree = fst_parameter::ParamGroup::getRemoteParamImpl("/fst_param/motion_controller/calibrator/normal_offset_threshold", data);
    cout << "ree=" << ree << "data=" << data[0] << ", " << data[1] << ", " << data[2] << ", " << data[3] << ", " << data[4] << ", " << data[5] << ", " << data[6] << ", " << data[7] << endl;
    */

    fst_controller::JointValues jnt1,jnt2,jnt3,jnt4,jnt5;
    fst_controller::Pose pose1, pose2, pose3, pose4, pose5;
    fst_controller::PoseEuler posee1,posee2,posee3;
    fst_controller::Transformation transformation;
    transformation.position.x = 0;
    transformation.position.y = 0;
    transformation.position.z = 0;
    transformation.orientation.a = 0;
    transformation.orientation.b = 0;
    transformation.orientation.c = 0;
    ErrorCode err;

    jnt1.j1 = 1.4;
    jnt1.j2 = 0.7;
    jnt1.j3 = -0.5;
    jnt1.j4 = 0.4;
    jnt1.j5 = -1.0;
    jnt1.j6 = 0.8;

    unsigned int cali_res;
    //fst_controller::ArmGroup *arm1 = new fst_controller::ArmGroup(jnt1,err);
    fst_controller::ArmGroup group;
    group.initArmGroup(err);
    if(err != SUCCESS) {
        ROS_ERROR("Error while init algerithm, code=0x%llx", err);
	    return              0;
    }

    unsigned int result;
    if (group.checkZeroOffset(result, err)) {
        cout <<"check zero offset: " << result << endl;
    }
    

    group.setCycleTime(0.01);
    group.setToolFrame(transformation);
    cout << "-----------------------------Test 1-------------------------------------------" << endl;
    posee1.position.x = 330;
    posee1.position.y = 0;
    posee1.position.z = 567.5;
    posee1.orientation.a = 0;
    posee1.orientation.b = 0;
    posee1.orientation.c = 3.14159;
    
    posee2.position.x = 430.0;
    posee2.position.y = 0;
    posee2.position.z = 567.5;
    posee2.orientation.a = 0;
    posee2.orientation.b = 0;
    posee2.orientation.c = 3.14159;
    
    posee3.position.x = 380;
    posee3.position.y = 0;
    posee3.position.z = 617.5;
    posee3.orientation.a = 0.0;
    posee3.orientation.b = 0.0;
    posee3.orientation.c = 3.14159;

    jnt2.j1 = 0;
    jnt2.j2 = 0;
    jnt2.j3 = 0;
    jnt2.j4 = 0;
    jnt2.j5 = -1.5708;
    jnt2.j6 = 0;
    
    jnt3.j1 = -0.8;
    jnt3.j2 = -0.3;
    jnt3.j3 = 0.3;
    jnt3.j4 = -0.6;
    jnt3.j5 = -0.5;
    jnt3.j6 = 0;

    jnt4.j1 = 0;
    jnt4.j2 = 0;
    jnt4.j3 = 0;
    jnt4.j4 = 0;
    jnt4.j5 = -1.5708;
    jnt4.j6 = 0;


    bool res;
    res = group.MoveJ(jnt3, 2000,16000,30,jnt2,2000,16000,0, 1061,err);
    res = group.MoveJ(jnt2,4000,16000,1062,err);
    //res = group.MoveJ(jnt2, 2000,32000,10,posee1,500,32000,0, 1063,err);
    //res = group.MoveJ(jnt2, 2000,16000, 1062,err);
    res = group.MoveC(posee1,posee2, 100,6000, 1064,err);
    //res = group.MoveL(posee1,200,16000,50,posee2,100,16000,0,1064,err);
    //res = group.MoveL(posee2,600,16000,30,posee3,500,16000,40,1067,err);
    //res = group.MoveL(posee2,100,16000, 1067,err);
    //res = group.MoveL(posee3,500,32000,40,jnt3,2000,32000,0,1065,err);
    //res = group.MoveL(posee2,600,32000,30,posee3,500,32000,0,1065,err);
    //res = group.MoveL(posee3,500,16000,1066,err);
    
    
    //res = group.MoveJ(jnt1,2000,32000,0, group.transformPoseEuler2Pose(posee1),500,32000,50, 1062,err);
    //res = group.MoveL(posee1,500,32000,50,posee2,600,32000,0,1063,err);
    //res = group.MoveL(posee2,600,32000,1064,err);

    cout << "planned-path:" << group.getPlannedPathFIFOLength() << ", joint_traj:" << group.getJointTrajectoryFIFOLength() << endl;
    group.convertPathToTrajectory(190,err);
    cout << "planned-path:" << group.getPlannedPathFIFOLength() << ", joint_traj:" << group.getJointTrajectoryFIFOLength() << endl;
    std::vector<fst_controller::JointPoint> traj;
    cout << "get points from joint FIFO:" << group.getPointsFromJointTrajectoryFIFO(traj,190,err) << endl;;
    cout << "planned-path:" << group.getPlannedPathFIFOLength() << ", joint_traj:" << group.getJointTrajectoryFIFOLength() << endl;
    group.convertPathToTrajectory(350,err);
    cout << "planned-path:" << group.getPlannedPathFIFOLength() << ", joint_traj:" << group.getJointTrajectoryFIFOLength() << endl;
    cout << "get points from joint FIFO:" << group.getPointsFromJointTrajectoryFIFO(traj,190,err) << endl;;
    group.convertPathToTrajectory(100,err);
    cout << "planned-path:" << group.getPlannedPathFIFOLength() << ", joint_traj:" << group.getJointTrajectoryFIFOLength() << endl;
    //group.suspendArmMotion();
    cout << "get points from joint FIFO:" << group.getPointsFromJointTrajectoryFIFO(traj,190,err) << endl;
    //res = group.MoveJ(jnt1,2000,32000,1066,err);
    //group.convertPathToTrajectory(10000,err);
    cout << "planned-path:" << group.getPlannedPathFIFOLength() << ", joint_traj:" << group.getJointTrajectoryFIFOLength() << endl;
    
    //displayTrajectory(traj,publisher);
    struct timeval tv;
    gettimeofday(&tv, NULL);
    cout << tv.tv_sec << "-" << tv.tv_usec << endl;
    gettimeofday(&tv, NULL);
    cout << tv.tv_sec << "-" << tv.tv_usec << endl;
    
    while(ros::ok()) {
//	displayTrajectory(traj,publisher);
	loop_rate.sleep();
    }


    return 0;
}
