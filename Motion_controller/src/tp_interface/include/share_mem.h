/**
 * @file share_mem.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-08-28
 */
#include "struct_to_mem/struct_joint_command.h" 
#include "struct_to_mem/struct_feedback_joint_states.h"
#include "middleware_to_mem/middleware_to_sharedmem.h"
#include "lib_controller/fst_datatype.h"

typedef struct _Shm_Joints_Cmd
{
	JointCommand joint_cmd;
	bool is_written;
}Shm_joints_Cmd;


class Share_Mem
{
public:
//	char* mem_ptr;
	int mem_handle;
	FeedbackJointState servo_state;
	Shm_joints_Cmd shm_jnt_cmd;

	Share_Mem()
	{
		shm_jnt_cmd.is_written = false;  //has not writen any joint command
	//	mem_ptr = getPtrOfMem();
		mem_handle = openMem(MEM_PROCESS);
	}
	
	bool Get_Feedback_Joint_State(FeedbackJointState &fbjs);
	bool Set_Joint_Positions(JointCommand jc_w);
	bool Is_Joint_Command_Written();
};
