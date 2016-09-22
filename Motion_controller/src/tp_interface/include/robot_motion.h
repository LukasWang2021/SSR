/**
 * @file robot_motion.h
 * @brief 
 * @author Wang Wei
 * @version 1.0.0
 * @date 2016-08-21
 */
#ifndef _ROBOT_MOTION_H_
#define	_ROBOT_MOTION_H_

#include "lib_controller/fst_datatype.h"
#include "lib_controller/lib_controller.h"
#include "common.h"
#include "motionSL.pb.h"
#include "threadsafe_queue.h"
#include "proto_define.h"
#include "share_mem.h"

using namespace Error;

using namespace std;
using namespace fst_controller;

#define MAX_CNT_VAL		(100)

#define MAX_PLANNED_POINTS_NUM		(20)
#define NUM_OF_POINTS_TO_SHARE_MEM	(10)

typedef struct _Command_Instruction
{
	bool		has_smooth;
	uint32_t	id;	
    motion_spec_MOTIONTYPE commandtype;	
	string		command_arguments; //pointer of command
}Command_Instruction;

typedef struct _MoveL_Param
{
	double vel_max;
	double acc_max;
	double smooth;
}MoveL_Param;

typedef struct _MoveJ_Param
{
	double vel_max;
	double acc_max;
	double smooth;
}MoveJ_Param;


typedef struct _Last_Position
{
	JointPoint	joint_point;
	int			position;
}Last_Position;


class Robot_Motion
{
public:
	ArmGroup		*arm_group;
	Share_Mem		share_mem;
	JointValues		joint_values;
	PoseEuler		pose;
	FST_ROBOT_MODE	mode;
	FST_ROBOT_STATE	state;
	FST_ROBOT_MODE	prev_mode;

	vector<JointPoint>	joint_traj;
	Last_Position		last_pos;
	Command_Instruction cur_instruction;
	Command_Instruction next_move_instruction;

	bool id_changed_flag;
	bool instruction_flag;
	int	previous_command_id;
	int current_command_id;
	

	int next_move_id;
	

	Threadsafe_Queue<motion_spec_MotionCommand> motion_queue;
	Threadsafe_Queue<motion_spec_MotionCommand> non_move_queue;
	Threadsafe_Queue<Command_Instruction> manual_instruction_queue;

	Robot_Motion()
	{
		id_changed_flag = false;
		instruction_flag = false;
		previous_command_id = -1;
		current_command_id = -1;
		next_move_id  = -1; //unknown next move id
								
		prev_mode = INIT_M;
		mode = INIT_M;

		pose.position.x = 2.1;
		pose.position.y = 33;

		//joint_values = Get_Cur_Joints_Value(); //get current joint value to arm_group
		joint_values.j1 = 0;
		joint_values.j2 = 0;
		joint_values.j3 = 0;
		joint_values.j4 = 0;
		joint_values.j5 = -1.5708;
		joint_values.j6 = 0;

		ErrorCode err;
		
		arm_group = new ArmGroup(joint_values,err);
		if(err != Success)
		{
			printf("construct arm group failed\n");
			exit(-1);
		}
		JointConstraints jnt_constraint = arm_group->getJointConstraints();
		jnt_constraint.j1.max_omega = 20;
		jnt_constraint.j2.max_omega = 20;
		jnt_constraint.j3.max_omega = 20;
		jnt_constraint.j4.max_omega = 20;
		jnt_constraint.j5.max_omega = 20;
		jnt_constraint.j6.max_omega = 20;

		arm_group->setJointConstraints(jnt_constraint);

		prev_mode = PAUSE_M;
		mode = PAUSE_M;
	}

	~Robot_Motion()
	{
		delete arm_group;
	}

	bool Get_Logic_Mode(FST_ROBOT_MODE &mode);
	bool Get_Logic_State(FST_ROBOT_STATE &state);
	JointValues Get_Cur_Joints_Value();
	ErrorCode Get_Cur_Position(PoseEuler &pose);
	bool Set_Logic_Mode(FST_ROBOT_MODE_CMD mode);
	bool Set_Logic_State(FST_ROBOT_STATE_CMD state);
	void Manual_Pause();
	ErrorCode Auto_Motion();
	void Unit_Convert(motion_spec_MoveL *src_moveL, PoseEuler& dst_pose, MoveL_Param &dst_movel_param);
	void Unit_Convert(motion_spec_MoveJ *src_moveJ, JointValues &dst_joints, MoveJ_Param &dst_moveJ_param);
	ErrorCode Move_Line(motion_spec_MoveL* current_moveL, int id, motion_spec_MoveL* next_moveL = NULL);
	ErrorCode Move_Line(PoseEuler* cur_pose, PoseEuler* next_pose = NULL);
	ErrorCode Move_Joints(JointValues *cur_joints, JointValues *next_joints = NULL);
	void Clear_Motion_Queue();
	void Add_Manual_Queue(motion_spec_MOTIONTYPE command_type, string arguments);
	void Add_Motion_Queue(motion_spec_MotionCommand &motion_command);
	bool Pick_Manual_Instruction();
	bool Pick_Motion_Instruction();
	bool Find_Next_Move_Command(motion_spec_MotionCommand &nex_motion_command);
	
//private:
	ErrorCode Parse_Motion_Command(motion_spec_MotionCommand motion_command, Command_Instruction &cmd_instruction);
	void Queue_Process();
	bool Check_Command_ID(int joints_len);

};


#endif
