/*************************************************************************
	> File Name: motion_control_ros_basic.h
	> Author: 
	> Mail: 
	> Created Time: 2018年10月29日 星期一 18时47分20秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_ROS_BASIC_H
#define _MOTION_CONTROL_ROS_BASIC_H

#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <base_datatype.h>

namespace fst_mc
{

    class RosBasic
    {
    public:
        void initRosBasic(void);
        void pubJointState(const Joint &joint);
        void setStartTimeStamp();

    private:
        ros::Time start_time_stamp_;
        sensor_msgs::JointState js_;
        ros::Publisher joint_state_pub_;
        ros::NodeHandle *ros_node_ptr_;
    };

}

#endif
