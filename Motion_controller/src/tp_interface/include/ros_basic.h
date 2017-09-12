#ifndef TP_INTERFACE_ROS_BASIC_H
#define TP_INTERFACE_ROS_BASIC_H
/**
 * @file ros_basic.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-11-02
 */
#include "trajplan/fst_datatype.h"
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>


using namespace fst_controller;

class RosBasic
{
  public:
	RosBasic(ros::NodeHandle *n);
	~RosBasic();

	void rosRobotInit();
	void pubJointState(Joint joint_values);
	void setStartTimeStamp();

  private:
	ros::Time start_time_stamp_;
	sensor_msgs::JointState js_;
	ros::Publisher joint_state_pub_;
};

#endif //TP_INTERFACE_ROS_BASIC_H
