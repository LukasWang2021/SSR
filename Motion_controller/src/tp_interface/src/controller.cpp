/**
 * @file controller.cpp
 * @brief
 * @author WangWei
 * @version 1.0.0
 * @date 2016-08-9
 */
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <signal.h>
#include <boost/thread.hpp>
#include "proto_parse.h"

#include <iostream>
#include <fstream>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <sensor_msgs/JointState.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <std_msgs/String.h>

using namespace boost;

Proto_Parse proto_parse;


void sigroutine(int dunno)
{
	ROS_INFO("stop!\n");
	exit(0);
}



/**
 * @brief: new socket server thread for command
 */
void Sock_Command_Server()
{
	while(1)
	{
		proto_parse.nn_socket->NN_Socket_Recieve();
		proto_parse.Parse_Buffer(proto_parse.nn_socket->req_buffer, proto_parse.nn_socket->recv_req_data_size);
		//usleep(100000);
	}
}

/**
 * @brief Execute Instructions in instruction queue
 *
 * @param proto_parse
 */
void Execute_Instructions()
{
	uint8_t buffer[1024];
	boost::mutex mu;
   /*FILE *fp = fopen("data.bin", "rb");*/
	//size_t count = fread(buffer, 1, sizeof(buffer), fp);
	//proto_parse.robot_motion.mode = AUTO_RUN_M;
	//for(int i = 0; i < 3; i++)
		/*proto_parse.Parse_Buffer(buffer, count);*/

	/*int cnt = 0;*/
	//FeedbackJointState fbjs;
	//while(1)
	//{
		//cnt ++;
		//if(cnt == 100)
				//cnt = 0;
		//proto_parse.robot_motion.share_mem.shm_jnt_cmd.joint_cmd.total_points = 10;
		//for(int index = 0; index < proto_parse.robot_motion.share_mem.shm_jnt_cmd.joint_cmd.total_points; index++)
		//{
			//for(int i = 0; i < 6;i++)
			//{
				//proto_parse.robot_motion.share_mem.shm_jnt_cmd.joint_cmd.points[index].positions[i] = (index+1)*i*1.0+cnt;
				//proto_parse.robot_motion.share_mem.shm_jnt_cmd.joint_cmd.points[index].point_position = MID_POINT;
			//}
		////	proto_parse.robot_motion.share_mem.shm_jnt_cmd.joint_cmd.points[0].point_position = START_POINT;
		////	proto_parse.robot_motion.share_mem.shm_jnt_cmd.joint_cmd.points[9].point_position = START_POINT;
		//}
		//if(cnt == 1)
			//proto_parse.robot_motion.share_mem.shm_jnt_cmd.joint_cmd.points[0].point_position = START_POINT;
		//if(cnt == 30)
			//proto_parse.robot_motion.share_mem.shm_jnt_cmd.joint_cmd.points[9].point_position = END_POINT;
		//bool result = proto_parse.robot_motion.share_mem.Set_Joint_Positions(proto_parse.robot_motion.share_mem.shm_jnt_cmd.joint_cmd);

		//if(proto_parse.robot_motion.share_mem.Get_Feedback_Joint_State(fbjs)==true){
		//for(int i=0;i<JOINT_NUM; i++){printf("%f ", fbjs.position[i]);}
		//printf("\n");
		//}
		//usleep(100*1000);
	/*}*/


	while(1)
	{
		proto_parse.Check_Manual_Wdt();
		mu.lock();
		proto_parse.robot_motion.Queue_Process();
		mu.unlock();

		usleep(1000);
	}
}

bool Compare_Joints(JointValues src_joints, JointValues dst_joints)
{
	if(src_joints.j1 != dst_joints.j1) return false;
	if(src_joints.j2 != dst_joints.j2) return false;
	if(src_joints.j3 != dst_joints.j3) return false;
	if(src_joints.j4 != dst_joints.j4) return false;
	if(src_joints.j5 != dst_joints.j5) return false;
	if(src_joints.j6 != dst_joints.j6) return false;

	return true;
}


/**
 * @brief Get current value of joints and publish them
 *
 * @param proto_parse
 */
void Shm_Recv_State()
{
#ifdef SIMMULATION
	ros::NodeHandle n;
	//build this topic to publish current joint state, "/fst_feedback_joint_states" topic will subscribe this topic
	ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("fst_feedback_joint_states", 1);
	//ros::Rate loop_rate(50);

	vector<string> JointName;
	JointName.resize(6);
	JointName[0] = "j1";
	JointName[1] = "j2";
	JointName[2] = "j3";
	JointName[3] = "j4";
	JointName[4] = "j5";
	JointName[5] = "j6";

	sensor_msgs::JointState js;
	js.name = JointName;
	js.header.seq = 0;
	js.position.resize(6);
#endif
	JointValues former_joints;
	int cnt = 0;
	int total_cnt = 0;
	while(1)
	{
		JointValues joint_values = proto_parse.robot_motion.Get_Cur_Joints_Value();
		if(Compare_Joints(former_joints, joint_values) == false)
		{
			cnt++;
			former_joints = joint_values;
		//	printf("cnt:%d, joint:%f,%f,%f,%f,%f,%f\n", cnt,\
				joint_values.j1,joint_values.j2,joint_values.j3,joint_values.j4,joint_values.j5,joint_values.j6);
#ifdef SIMMULATION
			js.position[0] = joint_values.j1;
			js.position[1] = joint_values.j2;
			js.position[2] = joint_values.j3;
			js.position[3] = joint_values.j4;
			js.position[4] = joint_values.j5;
			js.position[5] = joint_values.j6;
			ros::spinOnce();  //nessasery
			joint_state_pub.publish(js);

	//	loop_rate.sleep();
#endif
		}
		else
		{
			if(cnt > 0)
				printf("update count:%d\n",cnt);
			cnt = 0;
		}

		int joints_len = proto_parse.robot_motion.arm_group->getJointTrajectoryFIFOLength();
		if(joints_len)
		{
			total_cnt++;
			//printf("total_cnt:%d\n", total_cnt++);
		}
		else
		{
			//if(total_cnt >0)
			//	printf("total_cnt:%d\n", total_cnt);
			total_cnt = 0;
		}

		usleep(40*1000);
	}
}

void Robot_Publisher()
{
	static int state_cnt = 0;
	static int mode_cnt = 0;
	static int jnt_cnt = 0;
	static int coord_cnt = 0;

	while(1)
	{
		if(proto_parse.state_publish_tm)
		{
			state_cnt++;
			if(state_cnt == proto_parse.state_publish_tm)
			{
				proto_parse.Pub_Cur_State();
				state_cnt = 0;
			}
		}
		if(proto_parse.mode_publish_tm)
		{
			mode_cnt++;
			if(mode_cnt == proto_parse.mode_publish_tm)
			{
				proto_parse.Pub_Cur_Mode();
				mode_cnt = 0;
			}
		}
		if(proto_parse.joint_publish_tm)
		{
			jnt_cnt++;
			if(jnt_cnt == proto_parse.joint_publish_tm)
			{
				proto_parse.Pub_Cur_Joint();
				jnt_cnt = 0;
			}
		}
		if(proto_parse.coord_publish_tm)
		{
			coord_cnt++;
			if(coord_cnt == proto_parse.coord_publish_tm)
			{
				proto_parse.Pub_Cur_Coord();
				coord_cnt = 0;
			}
		}

		usleep(MIN_ROBOT_PUBLISH_TIME*1000);
	}
}




int main(int argc, char **argv)
{
	signal(SIGINT, sigroutine);

	ROS_DEBUG("ssldfslfklskf");
#ifdef SIMMULATION
	ros::init(argc, argv, "controller");
#endif

//	Execute_Instructions();

	//thread to receive and reply messages from TP
	boost::thread thrd_sock_cmmand(&Sock_Command_Server);
	boost::thread thrd_exc_instruction(&Execute_Instructions);

	boost::thread thrd_recv_state(&Shm_Recv_State);
	boost::thread thrd_robot_publisher(&Robot_Publisher);
#ifdef SIMMULATION
	ros::spin();
#else
	while(1){
		sleep(1);
	}
#endif

}
