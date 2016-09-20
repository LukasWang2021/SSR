
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
    ros::Rate loop_rate(1);

    fst_controller::JointValues jnt1,jnt2,jnt3,jnt4,jnt5;
    fst_controller::Pose pose1, pose2, pose3, pose4, pose5;
    fst_controller::PoseEuler posee1,posee2,posee3;
    ErrorCode err;

    jnt1.j1 = 1.4;
    jnt1.j2 = 0.7;
    jnt1.j3 = -0.5;
    jnt1.j4 = 0.4;
    jnt1.j5 = -1.0;
    jnt1.j6 = 0.8;


    //fst_controller::ArmGroup *arm1 = new fst_controller::ArmGroup(jnt1,err);
    fst_controller::ArmGroup group(jnt1,err);
    ROS_INFO("vel=%f,acc=%f",group.getMaxVelocity(),group.getMaxAcceleration());
    
    if(err!=Error::Success)
	ROS_ERROR("Error while init algerithm, code=%d", err);

    group.setCycleTime(0.01);
    ROS_INFO("-----------------------------Test 1-------------------------------------------");
    posee1.position.x = 120;
    posee1.position.y = 0;
    posee1.position.z = 617.5;
    posee1.orientation.a = 0;
    posee1.orientation.b = -0.4;
    posee1.orientation.c = 2.95;
    
    posee2.position.x = 380.0;
    posee2.position.y = 0;
    posee2.position.z = 450;
    posee2.orientation.a = 0;
    posee2.orientation.b = 0.4;
    posee2.orientation.c = 3.44;
    
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
    
    //group.MoveL(posee1,1000,32000,50,jnt1,3200,45000,30,1064,err);
    //group.MoveJ(jnt1,3200,45000,30,posee2,600,45000,30,1065,err)
    //group.MoveL(posee2,600,32000,30,posee3,1200,32000,0,1064,err);
    //group.MoveL(posee3,1200,32000,1065,err);
    
    bool res;
    res = group.MoveJ(jnt3, 2000,32000,30,jnt2,2000,32000,10, 1061,err);
    ROS_INFO("vel=%f,acc=%f",group.getMaxVelocity(),group.getMaxAcceleration());
    //res = group.MoveJ(jnt2,2000,32000,0,jnt2,2000,32000,0, 1062,err);
    res = group.MoveJ(jnt2, 2000,32000,10,posee1,500,32000,0, 1063,err);
    ROS_INFO("vel=%f,acc=%f",group.getMaxVelocity(),group.getMaxAcceleration());
    res = group.MoveL(posee1,500,32000,30,posee2,600,32000,30,1064,err);
    ROS_INFO("vel=%f,acc=%f",group.getMaxVelocity(),group.getMaxAcceleration());
    res = group.MoveL(posee2,600,32000,30,posee3,500,32000,40,1067,err);
    ROS_INFO("vel=%f,acc=%f",group.getMaxVelocity(),group.getMaxAcceleration());
    res = group.MoveL(posee3,500,32000,40,jnt3,2000,32000,0,1065,err);
    ROS_INFO("vel=%f,acc=%f",group.getMaxVelocity(),group.getMaxAcceleration());
    //res = group.MoveL(posee2,600,32000,30,posee3,500,32000,0,1065,err);
    //res = group.MoveL(posee3,500,32000,1066,err);
    
    
    //res = group.MoveJ(jnt1,2000,32000,0, group.transformPoseEuler2Pose(posee1),500,32000,50, 1062,err);
    //res = group.MoveL(posee1,500,32000,50,posee2,600,32000,0,1063,err);
    //res = group.MoveL(posee2,600,32000,1064,err);

    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    group.convertPathToTrajectory(10000,err);
    res = group.MoveJ(jnt1,2000,32000,1066,err);
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


    return 0;
}
