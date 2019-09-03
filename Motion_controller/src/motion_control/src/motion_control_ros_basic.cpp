/*************************************************************************
	> File Name: motion_control_ros_basic.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年10月29日 星期一 18时46分26秒
 ************************************************************************/

#include <vector>
#include <motion_control_ros_basic.h>

using namespace basic_alg;

namespace fst_mc
{

void RosBasic::initRosBasic(void)
{
    printf(">>>>>>>>>>>>>>>> initRosBasic <<<<<<<<<<<<<<<<<<<\n");
    int argc = 1;
    char buf[16] = {"MotionControl"};
    char *bufs[] = {buf};
    ros::init(argc, bufs, "fst_mc");
    ros_node_ptr_ = new ros::NodeHandle;
    //build this topic to publish current joint state, "/fst_feedback_joint_states" topic will subscribe this topic
    //joint_state_pub_ = ros_node_ptr_->advertise<sensor_msgs::JointState>("fst_feedback_joint_states", 1);
    joint_state_pub_ = ros_node_ptr_->advertise<sensor_msgs::JointState>("joint_states", 1);

    std::vector<std::string> joint_name;
    
    joint_name.resize(6);
    joint_name[0] = "joint1";
    joint_name[1] = "joint2";
    joint_name[2] = "joint3";
    joint_name[3] = "joint4";
    joint_name[4] = "joint5";
    joint_name[5] = "joint6";

    js_.name = joint_name;
    js_.header.seq = 0;
    js_.position.resize(6);
}


void RosBasic::pubJointState(const Joint &joint)
{
    js_.position[0] = joint.j1_;
    js_.position[1] = joint.j2_;
    js_.position[2] = joint.j3_;
    js_.position[3] = joint.j4_;
    js_.position[4] = joint.j5_;
    js_.position[5] = joint.j6_;

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

}




