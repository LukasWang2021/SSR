#include <iostream>
#include "ros_basic.h"
#include <sys/time.h>


RosBasic::RosBasic(ros::NodeHandle *n)
{	
	//build this topic to publish current joint state, "/fst_feedback_joint_states" topic will subscribe this topic
	joint_state_pub_ = n->advertise<sensor_msgs::JointState>("fst_feedback_joint_states", 1);
	//ros::Rate loop_rate(50);

	std::vector<std::string> JointName;
	JointName.resize(6);
	JointName[0] = "j1";
	JointName[1] = "j2";
	JointName[2] = "j3";
	JointName[3] = "j4";
	JointName[4] = "j5";
	JointName[5] = "j6";

	
	js_.name = JointName;
	js_.header.seq = 0;
	js_.position.resize(6);

}

RosBasic::~RosBasic()
{
}

void RosBasic::rosRobotInit()
{
}

void RosBasic::pubJointState(Joint joint_values)
{
	js_.position[0] = joint_values.j1;
	js_.position[1] = joint_values.j2;
	js_.position[2] = joint_values.j3;
	js_.position[3] = joint_values.j4;
	js_.position[4] = joint_values.j5;
	js_.position[5] = joint_values.j6;
	
	js_.header.stamp = ros::Time::now();
	//start to set time stamp
	/*ros::Time cur_time = ros::Time::now();*/
	//js_.header.stamp.sec = cur_time.sec - start_time_stamp_.sec;
	//js_.header.stamp.nsec = cur_time.nsec - start_time_stamp_.nsec;

	ros::spinOnce();  //nessasery
	joint_state_pub_.publish(js_);
}

void RosBasic::setStartTimeStamp()
{
	start_time_stamp_ = ros::Time::now();
}
