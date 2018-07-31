#ifndef TP_INTERFACE_ROS_BASIC_H
#define TP_INTERFACE_ROS_BASIC_H
/**
 * @file ros_basic.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-11-02
 */
#include "fst_datatype.h"
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <ros/ros.h>


using namespace fst_controller;

class RosBasic
{
  public:
	RosBasic(ros::NodeHandle *n);
	~RosBasic();

    /**
     * @brief: init 
     */
	void rosRobotInit();

    /**
     * @brief: publish current joints to rviz
     *
     * @param joint_values: curent joints
     */
	void pubJointState(Joint joint_values);

    /**
     * @brief: set start time stamp 
     */
	void setStartTimeStamp();

  private:
	ros::Time start_time_stamp_;    //start time stamp
        moveit_msgs::DisplayRobotState js_; //joint state
	ros::Publisher joint_state_pub_;    //Publisher
};

#endif //TP_INTERFACE_ROS_BASIC_H
