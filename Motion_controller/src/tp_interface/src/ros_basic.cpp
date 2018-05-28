#include <iostream>
#include "ros_basic.h"
#include <sys/time.h>


RosBasic::RosBasic(ros::NodeHandle *n)
{	
	//build this topic to publish current joint state, "/fst_feedback_joint_states" topic will subscribe this topic
        joint_state_pub_ = n->advertise<moveit_msgs::DisplayRobotState>("fst_feedback_joint_states", 1);
	//ros::Rate loop_rate(50);

	std::vector<std::string> JointName;
	JointName.resize(6);
	JointName[0] = "joint_1";
	JointName[1] = "joint_2";
	JointName[2] = "joint_3";
	JointName[3] = "joint_4";
	JointName[4] = "joint_5";
	JointName[5] = "joint_6";

        js_.state.joint_state.name = JointName;
        js_.state.joint_state.header.seq = 0;
        js_.state.joint_state.position.resize(6);
}

RosBasic::~RosBasic()
{
}

void RosBasic::rosRobotInit()
{
}

void RosBasic::pubJointState(Joint joint_values)
{
	js_.state.joint_state.position[0] = joint_values.j1;
	js_.state.joint_state.position[1] = joint_values.j2;
	js_.state.joint_state.position[2] = joint_values.j3;
	js_.state.joint_state.position[3] = joint_values.j4;
	js_.state.joint_state.position[4] = joint_values.j5;
	js_.state.joint_state.position[5] = joint_values.j6;
	
	js_.state.joint_state.header.stamp = ros::Time::now();
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
