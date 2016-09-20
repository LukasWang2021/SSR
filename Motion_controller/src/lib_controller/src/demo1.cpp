
#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>


#include <lib_controller/lib_controller.h>
#include <std_msgs/String.h>


struct Test {
    int a;
    int b;
    string c;
};

Test inst1,inst2;

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
    ROS_INFO("disp sended");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_demo");
    ros::NodeHandle node;
    ros::Publisher publisher = node.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1000);
    ros::Publisher chatter = node.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(1);

    fst_controller::JointValues jnt1,jnt2,jnt3,jnt4,jnt5;
    fst_controller::Pose pose1, pose2, pose3, pose4, pose5;
    fst_controller::PoseEuler pose_e;
    ErrorCode err;

    jnt1.j1 = 0;
    jnt1.j2 = 0;
    jnt1.j3 = 0;
    jnt1.j4 = 0;
    jnt1.j5 = -1.5708;
    jnt1.j6 = 0;

    //fst_controller::ArmGroup *arm1 = new fst_controller::ArmGroup(jnt1,err);
    fst_controller::ArmGroup group(jnt1,err);
    
    if(err!=Error::Success)
	ROS_ERROR("Error while init algerithm, code=%d", err);
    /*
    fst_controller::JointConstraints constraints;
    constraints.j1.upper = 2.9671;
    constraints.j2.upper = 1.3090;
    constraints.j3.upper = 3.2289;
    constraints.j4.upper = 3.3161;
    constraints.j5.upper = 1.8500;
    constraints.j6.upper = 6.2832;
    constraints.j1.lower = -2.9671;
    constraints.j2.lower = -2.2689;
    constraints.j3.lower = -1.1694;
    constraints.j4.lower = -3.3161;
    constraints.j5.lower = -1.8500;
    constraints.j6.lower = -6.2832;
    constraints.j1.max_omega = 5.1;
    constraints.j2.max_omega = 4.6;
    constraints.j3.max_omega = 5.8;
    constraints.j4.max_omega = 8.7;
    constraints.j5.max_omega = 6.2;
    constraints.j6.max_omega = 11.0;
    group.setJointConstraints(constraints);*/

    //group.setCycleTime(0.01);
    jnt1= group.getCurrentJointValues();
    ROS_INFO("current joints:%f,%f,%f,%f,%f,%f", jnt1.j1,jnt1.j2,jnt1.j3,jnt1.j4,jnt1.j5,jnt1.j6);
    pose1 = group.getCurrentPose();
    ROS_INFO("current pose:%f,%f,%f,%f,%f,%f,%f",   pose1.position.x,
						    pose1.position.y,
						    pose1.position.z,
						    pose1.orientation.w,
						    pose1.orientation.x,
						    pose1.orientation.y,
						    pose1.orientation.z);
    pose_e = group.transformPose2PoseEuler(pose1);
    ROS_INFO("current pose_e:%f,%f,%f,%f,%f,%f",   pose_e.position.x,
						    pose_e.position.y,
						    pose_e.position.z,
						    pose_e.orientation.a,
						    pose_e.orientation.b,
						    pose_e.orientation.c);
    
    ROS_INFO("-----------------------------Test 1-------------------------------------------");
    pose1.position.x = 342.949213;
    pose1.position.y = 0.0;
    pose1.position.z = 617.5;
    pose1.orientation.w = 0;
    pose1.orientation.x = 1;
    pose1.orientation.y = 0;
    pose1.orientation.z = 0;
    ROS_INFO("pose1: %f,%f,%f,%f,%f,%f,%f",	    pose1.position.x,
						    pose1.position.y,
						    pose1.position.z,
						    pose1.orientation.w,
						    pose1.orientation.x,
						    pose1.orientation.y,
						    pose1.orientation.z);
    group.computeIK(pose1,jnt1,err);
    ROS_INFO("pose1->joint1 %d: %f,%f,%f,%f,%f,%f", err, jnt1.j1,jnt1.j2,jnt1.j3,jnt1.j4,jnt1.j5,jnt1.j6);
    group.computeFK(jnt1,pose1,err);
    ROS_INFO("joint1->pose1: %f,%f,%f,%f,%f,%f,%f", pose1.position.x,
						    pose1.position.y,
						    pose1.position.z,
						    pose1.orientation.w,
						    pose1.orientation.x,
						    pose1.orientation.y,
						    pose1.orientation.z);
    
    ROS_INFO("-----------------------------Test 2-------------------------------------------");
    pose1.position.x = 337.906564;
    pose1.position.y = 0.0;
    pose1.position.z = 617.5;
    pose1.orientation.w = 0;
    pose1.orientation.x = 1;
    pose1.orientation.y = 0;
    pose1.orientation.z = 0;
    ROS_INFO("pose1: %f,%f,%f,%f,%f,%f,%f",	    pose1.position.x,
						    pose1.position.y,
						    pose1.position.z,
						    pose1.orientation.w,
						    pose1.orientation.x,
						    pose1.orientation.y,
						    pose1.orientation.z);
    group.computeIK(pose1,jnt1,err);
    ROS_INFO("pose1->joint1 %d: %f,%f,%f,%f,%f,%f", err, jnt1.j1,jnt1.j2,jnt1.j3,jnt1.j4,jnt1.j5,jnt1.j6);
    group.computeFK(jnt1,pose1,err);
    ROS_INFO("joint1->pose1: %f,%f,%f,%f,%f,%f,%f", pose1.position.x,
						    pose1.position.y,
						    pose1.position.z,
						    pose1.orientation.w,
						    pose1.orientation.x,
						    pose1.orientation.y,
						    pose1.orientation.z);

    ROS_INFO("-----------------------------Test 3-------------------------------------------");
    fst_controller::PoseEuler posee1,posee2,posee3,posee4,posee5,posee6;
    posee1.position.x = 120;
    posee1.position.y = 0;
    posee1.position.z = 617.5;
    posee1.orientation.a = 0;
    posee1.orientation.b = -0.7;
    posee1.orientation.c = 2.7;
    
    posee2.position.x = 380.0;
    posee2.position.y = 0;
    posee2.position.z = 450;
    posee2.orientation.a = 0;
    posee2.orientation.b = 0.7;
    posee2.orientation.c = 3.54159;
    
    posee3.position.x = 380;
    posee3.position.y = 0;
    posee3.position.z = 617.5;
    posee3.orientation.a = 0.0;
    posee3.orientation.b = 0.0;
    posee3.orientation.c = 3.14159;
    
    posee4.position.x = 450;
    posee4.position.y = -100;
    posee4.position.z = 524.5;
    posee4.orientation.a = 0.7;
    posee4.orientation.b = 0.0;
    posee4.orientation.c = 3.1416;
    
    posee5.position.x = 500;
    posee5.position.y = 100;
    posee5.position.z = 524.5;
    posee5.orientation.a = 0.9;
    posee5.orientation.b = -0.4;
    posee5.orientation.c = 3.1416;
    
    posee6.position.x = 550;
    posee6.position.y = -100;
    posee6.position.z = 524.5;
    posee6.orientation.a = 1.2;
    posee6.orientation.b = -0.9;
    posee6.orientation.c = 3.1416;
    /*
    group.MoveL(posee1,500.0,15000.0,1064,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    group.MoveL(posee2,500.0,15000.0,1064,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    group.MoveL(posee3,500.0,15000.0,1064,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    group.MoveL(posee4,500.0,15000.0,1064,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    group.MoveL(posee5,500.0,15000.0,1064,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    group.MoveL(posee6,500.0,15000.0,1064,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
*/
    /*group.MoveL(posee1,500,15000,50,posee2,500,15000,30,1064,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    group.MoveL(posee2,500,15000,30,posee3,500,15000,60,1064,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    group.MoveL(posee3,500,15000,60,posee4,500,15000,50,1064,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    group.MoveL(posee4,500,15000,50,posee5,500,15000,20,1064,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    group.MoveL(posee5,500,15000,20,posee6,500,15000,-1,1064,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    group.MoveL(posee6,500.0,15000.0,1064,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    */

    group.MoveL(posee1,1000,32000,50,posee2,600,32000,30,1064,err);
    group.MoveL(posee2,600,32000,30,posee3,1200,32000,0,1064,err);
    group.MoveL(posee3,1200,32000,1065,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());

    group.convertPathToTrajectory(10000,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    
    std::vector<fst_controller::JointPoint> traj;
    ROS_INFO("get points from joint FIFO:%d", group.getPointsFromJointTrajectoryFIFO(traj,10000,err));
    //displayTrajectory(traj,publisher);
    
    while(ros::ok()) {
//	std_msgs::String msg;
//	chatter.publish(msg);
//	moveit_msgs::DisplayTrajectory disp;
//	publisher.publish(disp);
	displayTrajectory(traj,publisher);
	loop_rate.sleep();
    }

    /*
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    group.convertPathToTrajectory(100,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    ROS_INFO("get points from joint FIFO:%d", group.getPointsFromJointTrajectoryFIFO(traj,100,err));
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    group.convertPathToTrajectory(100,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    ROS_INFO("get points from joint FIFO:%d", group.getPointsFromJointTrajectoryFIFO(traj,100,err));
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    group.convertPathToTrajectory(100,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    ROS_INFO("get points from joint FIFO:%d", group.getPointsFromJointTrajectoryFIFO(traj,100,err));
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    group.convertPathToTrajectory(100,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    ROS_INFO("get points from joint FIFO:%d", group.getPointsFromJointTrajectoryFIFO(traj,100,err));
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    group.convertPathToTrajectory(100,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    ROS_INFO("get points from joint FIFO:%d", group.getPointsFromJointTrajectoryFIFO(traj,100,err));
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    */
    if(group.getJointTrajectoryFIFOIsLocked())
	ROS_INFO("joint FIFO is locked");
    else
	ROS_INFO("joint FIFO is NOT locked");

    return 0;
}
