
#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>


#include <motion_controller/arm_group.h>
#include <std_msgs/String.h>

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
    ros::init(argc, argv, "motion_controller_demo");
    ros::NodeHandle node;
    ros::Publisher publisher = node.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1000);
    ros::Rate loop_rate(1);

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

    //fst_controller::ArmGroup *arm1 = new fst_controller::ArmGroup(jnt1,err);
    fst_controller::ArmGroup group(jnt1,err);
    ROS_INFO("vel=%f,acc=%f",group.getMaxVelocity(),group.getMaxAcceleration());
    
    if(err!=Error::Success)
	ROS_ERROR("Error while init algerithm, code=%d", err);

    group.setCycleTime(0.01);
    group.setToolFrame(transformation);
    ROS_INFO("-----------------------------Test 1-------------------------------------------");
    posee1.position.x = 150;
    posee1.position.y = 0;
    posee1.position.z = 617.5;
    posee1.orientation.a = 0;
    posee1.orientation.b = 0;
    posee1.orientation.c = 3.14159;
    
    posee2.position.x = 380.0;
    posee2.position.y = 0;
    posee2.position.z = 450;
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


    double joint[3][6];
    fst_controller::JointValues v[3];
    v[0] = jnt1;
    ROS_INFO("v[0]=%f,%f,%f,%f,%f,%f",v[0].j1,v[0].j2,v[0].j3,v[0].j4,v[0].j5,v[0].j6);
    ROS_INFO("joint[0]=%f,%f,%f,%f,%f,%f",joint[0][0],joint[0][1],joint[0][2],joint[0][3],joint[0][4],joint[0][5]);
    memcpy(joint[0], &v[0], sizeof(v[0]));
    ROS_INFO("joint[0]=%f,%f,%f,%f,%f,%f",joint[0][0],joint[0][1],joint[0][2],joint[0][3],joint[0][4],joint[0][5]);
    memcpy(&v[1], joint[0], sizeof(joint[0]));
    ROS_INFO("v[1]=%f,%f,%f,%f,%f,%f",v[1].j1,v[1].j2,v[1].j3,v[1].j4,v[1].j5,v[1].j6);
    memcpy(&v[0], joint[0], sizeof(joint[0]));
    memcpy(&v[2], joint[0], sizeof(joint[0]));
    memcpy(joint, v, sizeof(joint));
    ROS_INFO("joint[0]=%f,%f,%f,%f,%f,%f",joint[0][0],joint[0][1],joint[0][2],joint[0][3],joint[0][4],joint[0][5]);
    ROS_INFO("joint[1]=%f,%f,%f,%f,%f,%f",joint[1][0],joint[1][1],joint[1][2],joint[1][3],joint[1][4],joint[1][5]);
    ROS_INFO("joint[2]=%f,%f,%f,%f,%f,%f",joint[2][0],joint[2][1],joint[2][2],joint[2][3],joint[2][4],joint[2][5]);


    ROS_INFO("-----------------------------Test 2-------------------------------------------");
    fst_controller::JointPoint jp;
    jp.joints = jnt2;
    jp.omegas.j1 = 0.5;
    jp.omegas.j3 = 0.5;
    jp.omegas.j5 = 0.5;
    memset(&jp.omegas, 0, sizeof(jp.omegas));
    ROS_INFO("joints=%f,%f,%f,%f,%f,%f", jp.joints.j1, jp.joints.j2, jp.joints.j3, jp.joints.j4, jp.joints.j5, jp.joints.j6);
    ROS_INFO("omegas=%f,%f,%f,%f,%f,%f", jp.omegas.j1, jp.omegas.j2, jp.omegas.j3, jp.omegas.j4, jp.omegas.j5, jp.omegas.j6);
    double cyc;
    //node.getParam("/fst_controller/arm_group/cycle_time", cyc);
    ros::param::get("/fst_controller/arm_group/cycle_time", cyc);
    ROS_INFO("we get param from server, cyc=%f", cyc);
    node.getParam("/fst_controller/kinematics_limit/j1/omega_max", cyc);
    ROS_INFO("we get param from server, max_omega=%f", cyc);
    ROS_INFO("j5:%d", node.hasParam("/fst_controller/kinematics_limit/j5"));
    ROS_INFO("j7:%d", node.hasParam("/fst_controller/kinematics_limit/j7"));
    
    std::map<std::string, double> map;

    if (ros::param::get("/fst_controller/arm_group", map))
        ROS_INFO("map size = %zu", map.size());
    else
        ROS_ERROR("fail");
    
    std::map<std::string, double>::iterator itr = map.find("cycle_time");
    if (itr != map.end()) {
        ROS_INFO("cycle=%f", itr->second);
    }
    else {
        ROS_INFO("cycle not found");
    }
    std::vector<double> v2;

    if (ros::param::get("/fst_controller/arm_group/test_trrt", v2)) {
        ROS_INFO("v2.size()=%zu",v2.size());
        ROS_INFO("v2=[%f %f %f]", v2[0], v2[1], v2[2]);
    }
    else {
        ROS_INFO("cannot get parameters");
    }
    //for (int i=0; i < 3; ++i)
    //   ROS_INFO("%f,%f,%f,%f,%f,%f",joint[0][i],joint[1][i],joint[2][i],joint[3][i],joint[4][i],joint[5][i]);
    //memcpy(joint, &v, sizeof(joint));
    //for (int i=0; i < 3; ++i)
    //    ROS_INFO("%f,%f,%f,%f,%f,%f",joint[0][i],joint[1][i],joint[2][i],joint[3][i],joint[4][i],joint[5][i]);
    
    //group.MoveL(posee1,1000,32000,50,jnt1,3200,45000,30,1064,err);
    //group.MoveJ(jnt1,3200,45000,30,posee2,600,45000,30,1065,err)
    //group.MoveL(posee2,600,32000,30,posee3,1200,32000,0,1064,err);
    //group.MoveL(posee3,1200,32000,1065,err);
    
    ROS_INFO("-----------------------------Test 3-------------------------------------------");
    bool res;
    res = group.MoveJ(jnt3, 2000,16000,30,jnt2,2000,16000,0, 1061,err);
    res = group.MoveJ(jnt2,4000,16000,1062,err);
    //res = group.MoveJ(jnt2, 2000,32000,10,posee1,500,32000,0, 1063,err);
    //res = group.MoveJ(jnt2, 2000,16000, 1062,err);
    res = group.MoveL(posee1,200,16000, 1064,err);
    //res = group.MoveL(posee1,200,16000,50,posee2,100,16000,0,1064,err);
    //res = group.MoveL(posee2,600,16000,30,posee3,500,16000,40,1067,err);
    res = group.MoveL(posee2,100,16000, 1067,err);
    //res = group.MoveL(posee3,500,32000,40,jnt3,2000,32000,0,1065,err);
    //res = group.MoveL(posee2,600,32000,30,posee3,500,32000,0,1065,err);
    //res = group.MoveL(posee3,500,16000,1066,err);
    
    
    //res = group.MoveJ(jnt1,2000,32000,0, group.transformPoseEuler2Pose(posee1),500,32000,50, 1062,err);
    //res = group.MoveL(posee1,500,32000,50,posee2,600,32000,0,1063,err);
    //res = group.MoveL(posee2,600,32000,1064,err);

    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    group.convertPathToTrajectory(90,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    std::vector<fst_controller::JointPoint> traj;
    ROS_INFO("get points from joint FIFO:%d", group.getPointsFromJointTrajectoryFIFO(traj,90,err));
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    group.convertPathToTrajectory(50,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    ROS_INFO("get points from joint FIFO:%d", group.getPointsFromJointTrajectoryFIFO(traj,50,err));
    group.convertPathToTrajectory(100,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    group.suspendArmMotion();
    ROS_INFO("get points from joint FIFO:%d", group.getPointsFromJointTrajectoryFIFO(traj,157,err));
    //res = group.MoveJ(jnt1,2000,32000,1066,err);
    //group.convertPathToTrajectory(10000,err);
    ROS_INFO("planned-path:%d, joint_traj:%d", group.getPlannedPathFIFOLength(),group.getJointTrajectoryFIFOLength());
    
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
