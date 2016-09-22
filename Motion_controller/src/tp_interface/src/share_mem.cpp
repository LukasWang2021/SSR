
#include "share_mem.h"

bool Share_Mem::Get_Feedback_Joint_State(FeedbackJointState &fbjs)
{
	int i;
	int ret;
	
	ret = readWriteSharedMem(mem_handle, &fbjs, "FeedbackJointState", MEM_READ);
	if(ret)
	{
		/*for(i=0;i<JOINT_NUM; i++){printf("fbjs.position[%d] = %f \n", i, fbjs.position[i]);}*/
		//for(i=0;i<JOINT_NUM; i++){printf("fbjs.velocity[%d] = %f \n", i, fbjs.velocity[i]);}
		//for(i=0;i<JOINT_NUM; i++){printf("fbjs.effort[%d] = %f \n", i, fbjs.effort[i]);}
		/*printf(" fbjs.state = %d\n\n", fbjs.state);*/
		servo_state = fbjs;

		return true;
	}

	return false;

}


bool Share_Mem::Set_Joint_Positions(JointCommand jc_w)
{

	int ret = readWriteSharedMem(mem_handle, &jc_w, "JointCommand", MEM_WRITE);
	if(ret)
	{
		shm_jnt_cmd.is_written = true;
		return true;
	}
	else 
	{
		shm_jnt_cmd.is_written = false;
		return false;
	}
}

bool Share_Mem::Is_Joint_Command_Written()
{
	return shm_jnt_cmd.is_written;
}

