// #include "stdafx.h"

#ifdef WIN32
#pragma warning(disable : 4786)
#endif
#include "stdio.h"
#include "string.h"
#include "setjmp.h"
#include "time.h"
#include "math.h"
#include "ctype.h"
#include "stdlib.h" 
#include "forsight_innercmd.h"

#ifndef WIN32
#include "error_code.h"
#include <execinfo.h>
#endif

#ifdef USE_FORSIGHT_REGISTERS_MANAGER
#include "reg_manager/forsight_registers_manager.h"
#else
#include "reg-shmi/forsight_registers.h"
#endif

#define FILE_PATH_LEN       1024
#define MAX_STOPWATCH_NUM   128

#define   MOVJ_COMMAND_PARAM_MIN     8
#define   MOVL_COMMAND_PARAM_MIN     8
#define   MOVC_COMMAND_PARAM_MIN     14

#define   SMOOTH_TYPE_CNT      "cnt"
#define   SMOOTH_TYPE_SD       "sd"
#define   SMOOTH_TYPE_SV       "sv"

typedef struct _StopWatch
{
    time_t start_time;
    int    diff_time;
}StopWatch;

StopWatch g_structStopWatch[MAX_STOPWATCH_NUM];

int call_MoveL(int iLineNum, struct thread_control_block* objThreadCntrolBlock);
int call_MoveJ(int iLineNum, struct thread_control_block* objThreadCntrolBlock);
int call_MoveC(int iLineNum, struct thread_control_block* objThreadCntrolBlock);
int call_MoveXPos(int iLineNum, struct thread_control_block* objThreadCntrolBlock);

int call_Timer(int iLineNum, struct thread_control_block* objThreadCntrolBlock);
int call_UserAlarm(int iLineNum, struct thread_control_block* objThreadCntrolBlock);
int call_Wait(int iLineNum, struct thread_control_block* objThreadCntrolBlock);
int call_Pause(int iLineNum, struct thread_control_block* objThreadCntrolBlock);
int call_Abort(int iLineNum, struct thread_control_block* objThreadCntrolBlock);
int call_BLDC_CTRL(int iLineNum, struct thread_control_block* objThreadCntrolBlock);

// This structure links a library function name   
// with a pointer to that function.   
struct intern_cmd_type {   
    char *f_name; // function name
    int can_exec_sub_thread ;
    int (*p)(int , struct thread_control_block* );   // pointer to the function   
} intern_cmd[] = {   
    // move on the top
    (char *)"movel",          0, call_MoveL,
    (char *)"movej",          0, call_MoveJ,
    (char *)"movec",          0, call_MoveC,
    (char *)"movex",          0, call_MoveXPos,
    // left
    (char *)FORSIGHT_TIMER,   1, call_Timer,
    (char *)"useralarm",      1, call_UserAlarm,
    (char *)"wait",           1, call_Wait,
    (char *)"pause",          1, call_Pause,
    (char *)"abort",          1, call_Abort,
    (char *)"bldc_ctrl",  1, call_BLDC_CTRL,
    (char *)"", 0  // null terminate the list   
};

/************************************************* 
	Function:		isdelim
	Description:	Return true if c is a delimiter.
	Input:			c           
	Return: 		1 - true ; 0 - false
*************************************************/ 
static int isdelim(char c)
{
  if(strchr(" ;,+-<>/*%^=()[]", c) || c==9 || c=='\r' || c=='\n' || c==0) 
    return 1;  
  return 0;
}

/************************************************* 
	Function:		iswhite
	Description:	Return 1 if c is space or tab.
	Input:			c           
	Return: 		1 - true ; 0 - false
*************************************************/ 
static int iswhite(char c)
{
  if(c==' ' || c=='\t') return 1;
  else return 0;
}

/************************************************* 
	Function:		find_copy_eol
	Description:	Find the start of the next line and copy left part into temp.
	Input:			prog         -    program pointer
	Input:			temp         -    current line buffer
	Return: 		1 - true ; 0 - false
*************************************************/ 
static char * find_copy_eol(char * prog, char* temp)
{
  while(*prog!='\n'  && *prog!='\0') 
	  *temp++=*prog++;
  if(*prog) prog++;
  return prog;
}

/************************************************* 
	Function:		get_cmd_param
	Description:	get command parameter.
	Input:			prog         -    program pointer
	Input:			temp         -    current line buffer
	Return: 		1 - true ; 0 - false
*************************************************/ 
static char * get_cmd_param(char * prog, char* temp)
{
  while(iswhite(*prog)) ++prog;  /* skip over white space */
    while(!isdelim(*prog)) 
		*temp++=*prog++;
  if(*prog) prog++;
  return prog;
}

time_t get_timer_start_time(int iIdx)
{
	if(iIdx >= MAX_STOPWATCH_NUM)
		return time(0);
	else
		return g_structStopWatch[iIdx].start_time;
}
/************************************************* 
	Function:		find_internal_cmd
	Description:	find internal cmd in the intern_cmd.
	Input:			s         -    cmd name
	Return: 		-1 - not found ; i - index of intern_cmd
*************************************************/ 
int find_internal_cmd(char *s)   
{   
    int i;
    for(i=0; intern_cmd[i].f_name[0]; i++) {
        if(!strcasecmp(intern_cmd[i].f_name, s))  return i;   
    }   
    return -1;   
}   

/************************************************* 
	Function:		call_internal_cmd_exec_sub_thread
	Description:	Whether it is moving command and 
	                can be executed in the multipled thread.
	Input:			s         -    cmd name
	Return: 		-1 - not found ; i - index of intern_cmd
*************************************************/ 
int call_internal_cmd_exec_sub_thread(int index)   
{   
    // int i; 
    if(index >= 0)
       return intern_cmd[index].can_exec_sub_thread;
	else 
	   return -1 ;
}

/************************************************* 
	Function:		call_internal_cmd
	Description:	Call the function in intern_cmd by index.
	Input:			index                  -    index of intern_cmd
	Input:			iLineNum               -    Line Number
	Input:			thread_control_block   - interpreter info
	Return: 		-1        -    index out of range ;
	                Other     -    function ret.
*************************************************/ 
int call_internal_cmd(int index, int iLineNum, 
					 struct thread_control_block* objThreadCntrolBlock)   
{   
    // int i; 
    if(index >= 0)
       return (*intern_cmd[index].p)(iLineNum, objThreadCntrolBlock);
	else 
	   return -1 ;
}

/************************************************* 
	Function:		getAditionalInfomation
	Description:	get aditional parameter infomation
	Input:			thread_control_block   - interpreter info
	Input:			instrSetPtr            - AdditionalInfomation buffer 
	                                         in the objThreadCntrolBlock->instrSet
	Return: 		0        -    Failed ;
	                Other    -    Count of aditional parameter infomation.
*************************************************/ 
int getAditionalInfomation(struct thread_control_block* objThreadCntrolBlock, 
						   char * instrSetPtr)
{
	int iCount = 0 ;
	eval_value value;
	int boolValue;
	
	char var[80];
	eval_value result;
	FrameOffset * objFrameOffsetPtr = 0 ;

	AdditionalInfomation additionalInfomation ;

	while(*(objThreadCntrolBlock->token) != '\r')
	{
		get_token(objThreadCntrolBlock);
		// 1.	ACC加速度倍率指令
		if(strcasecmp(objThreadCntrolBlock->token, "acc") == 0)
		{
			additionalInfomation.type = ACC ;
			get_exp(objThreadCntrolBlock, &value, &boolValue);
			int speed = (int)value.getDoubleValue();
			additionalInfomation.acc_speed = speed ;
			iCount++ ;
			objThreadCntrolBlock->instrSet->target.acc = (float)speed / 100 ;
		}
		// 2.	 Condition <case>（低优先级）
		// 3.	批量位置补偿指令（低优先级）
		// 4.	单句位置补偿指令
		// 5.	Tool_Offset工具坐标补偿指令
		else if((strcasecmp(objThreadCntrolBlock->token, "offset") == 0)
		      ||(strcasecmp(objThreadCntrolBlock->token, "tool_offset") == 0))
		{
			if(strcasecmp(objThreadCntrolBlock->token, "offset") == 0)
			{
				additionalInfomation.type = OFFSET ;
				objFrameOffsetPtr = &(objThreadCntrolBlock->instrSet->target.user_frame_offset) ;
			}
			else if(strcasecmp(objThreadCntrolBlock->token, "tool_offset") == 0)
			{
				additionalInfomation.type = TOOL_OFFSET ;
				objFrameOffsetPtr = &(objThreadCntrolBlock->instrSet->target.tool_frame_offset) ;
			}
 			objFrameOffsetPtr->valid = true ;

			get_token(objThreadCntrolBlock);
			if(strncasecmp(objThreadCntrolBlock->token, "pr", strlen("pr")) == 0)
			{
				if(strcasecmp(objThreadCntrolBlock->token, "pr") == 0)
					additionalInfomation.offset.type = PR;
				else if(strcasecmp(objThreadCntrolBlock->token, "pr_uf") == 0)
					additionalInfomation.offset.type = PR_UF;
				else if(strcasecmp(objThreadCntrolBlock->token, "pr_tf") == 0)
					additionalInfomation.offset.type = PR_TF;
				
				get_token(objThreadCntrolBlock);
				if(strcasecmp(objThreadCntrolBlock->token, "pr") == 0)
					additionalInfomation.offset.pr_reg.type = POSE_REG;
				else if(strcasecmp(objThreadCntrolBlock->token, "uf") == 0)
					additionalInfomation.offset.pr_reg.type = NUM_REG;
				
				get_token(objThreadCntrolBlock);
				if(*(objThreadCntrolBlock->token) != '[') {
					return 0 ;
				}
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				additionalInfomation.offset.pr_reg.index = (int)value.getDoubleValue();
				// 
				memset(var, 0x00, 80);
				// NOTICE: lower case
				sprintf(var, "pr[%d]", (int)value.getDoubleValue());
				result = find_var(objThreadCntrolBlock, var);
				if(result.hasType(TYPE_JOINT) == TYPE_JOINT)
				{
					objFrameOffsetPtr->coord_type = COORDINATE_JOINT;
					objFrameOffsetPtr->offset_joint = result.getJointValue();
#ifndef WIN32
	       			FST_INFO("setJointValue pos_type = %d at HOINT:(%f, %f, %f, %f, %f, %f)",
						result.getIntType(), 
						result.getJointValue().j1_, result.getJointValue().j2_, 
						result.getJointValue().j3_, result.getJointValue().j4_, 
						result.getJointValue().j5_, result.getJointValue().j6_);
#else
					FST_INFO("setJointValue pos_type = %d at HOINT:(%f, %f, %f, %f, %f, %f)",
						result.getIntType(), 
						result.getJointValue().j1, result.getJointValue().j2, 
						result.getJointValue().j3, result.getJointValue().j4, 
						result.getJointValue().j5, result.getJointValue().j6);		
#endif	
				}
				else if(result.hasType(TYPE_POSE) == TYPE_POSE)
				{
					objFrameOffsetPtr->coord_type = COORDINATE_CARTESIAN;
					objFrameOffsetPtr->offset_pose = result.getPoseValue();
#ifndef WIN32
	       			FST_INFO("setPoseValue pos_type = %d at CART:(%f, %f, %f, %f, %f, %f)", 
						result.getIntType(), 
						result.getPoseValue().point_.x_, result.getPoseValue().point_.y_, 
						result.getPoseValue().point_.z_, result.getPoseValue().euler_.a_, 
						result.getPoseValue().euler_.b_, result.getPoseValue().euler_.c_);
#else
					FST_INFO("setPoseValue pos_type = %d at CART:(%f, %f, %f, %f, %f, %f)", 
						result.getIntType(), 
						result.getPoseValue().position.x, result.getPoseValue().position.y, 
						result.getPoseValue().position.z, result.getPoseValue().orientation.a, 
						result.getPoseValue().orientation.b, result.getPoseValue().orientation.c);
#endif	
				}
				// 
				get_token(objThreadCntrolBlock);
				if(*(objThreadCntrolBlock->token) != ']') {
					return 0 ;
				}
				
				if(additionalInfomation.offset.type == PR_UF)
				{
	  				get_token(objThreadCntrolBlock);
					if(strcasecmp(objThreadCntrolBlock->token, "pr") == 0)
						additionalInfomation.offset.uf_reg.type = POSE_REG;
					else if(strcasecmp(objThreadCntrolBlock->token, "uf") == 0)
						additionalInfomation.offset.uf_reg.type = NUM_REG;
					
					get_token(objThreadCntrolBlock);
					if(*(objThreadCntrolBlock->token) != '[') {
						return 0 ;
					}
					get_exp(objThreadCntrolBlock, &value, &boolValue);
					additionalInfomation.offset.uf_reg.index = (int)value.getDoubleValue();
					//
					objFrameOffsetPtr->offset_frame_id = (int)value.getDoubleValue();
					//
					get_token(objThreadCntrolBlock);
					if(*(objThreadCntrolBlock->token) != ']') {
						return 0 ;
					}
				}
				else if(additionalInfomation.offset.type == PR_TF)
				{
					get_token(objThreadCntrolBlock);
					if(strcasecmp(objThreadCntrolBlock->token, "pr") == 0)
						additionalInfomation.offset.uf_reg.type = POSE_REG;
					else if(strcasecmp(objThreadCntrolBlock->token, "tf") == 0)
						additionalInfomation.offset.uf_reg.type = NUM_REG;
					
					get_token(objThreadCntrolBlock);
					if(*(objThreadCntrolBlock->token) != '[') {
						return 0 ;
					}
					get_exp(objThreadCntrolBlock, &value, &boolValue);
					additionalInfomation.offset.uf_reg.index = (int)value.getDoubleValue();
					//
					objFrameOffsetPtr->offset_frame_id = (int)value.getDoubleValue();
					//
					get_token(objThreadCntrolBlock);
					if(*(objThreadCntrolBlock->token) != ']') {
						return 0 ;
					}
				}
				else 
				{
					// Current offset_frame_id
					objFrameOffsetPtr->offset_frame_id = -1;
				}
			}
			else if(strncasecmp(objThreadCntrolBlock->token, "c_vec", strlen("c_vec")) == 0)
			{
				if(strcasecmp(objThreadCntrolBlock->token, "c_vec") == 0)
					additionalInfomation.offset.type = C_VEC;
				else if(strcasecmp(objThreadCntrolBlock->token, "c_vec_uf") == 0)
					additionalInfomation.offset.type = C_VEC_UF;
				else if(strcasecmp(objThreadCntrolBlock->token, "c_vec_tf") == 0)
					additionalInfomation.offset.type = C_VEC_TF;
				
				// additionalInfomation.offset.pose_target ;
				get_token(objThreadCntrolBlock);
				if(*(objThreadCntrolBlock->token) != '(')
					return 0 ;
				
				objFrameOffsetPtr->coord_type = COORDINATE_CARTESIAN;
				
			    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
				additionalInfomation.offset.pose_target.point_.x_ = value.getDoubleValue();
				objFrameOffsetPtr->offset_pose.point_.x_ = value.getDoubleValue();
#else
				additionalInfomation.offset.pose_target.position.x = value.getDoubleValue();
				objFrameOffsetPtr->offset_pose.position.x = value.getDoubleValue();
#endif
				get_token(objThreadCntrolBlock);
				if(*(objThreadCntrolBlock->token) != ',')
					return 0 ;
				
			    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
				additionalInfomation.offset.pose_target.point_.y_ = value.getDoubleValue();
				objFrameOffsetPtr->offset_pose.point_.y_ = value.getDoubleValue();
#else
				additionalInfomation.offset.pose_target.position.y = value.getDoubleValue();
				objFrameOffsetPtr->offset_pose.position.y = value.getDoubleValue();
#endif
				get_token(objThreadCntrolBlock);
				if(*(objThreadCntrolBlock->token) != ',')
					return 0 ;
				
			    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
				additionalInfomation.offset.pose_target.point_.z_ = value.getDoubleValue();
				objFrameOffsetPtr->offset_pose.point_.z_ = value.getDoubleValue();
#else
				additionalInfomation.offset.pose_target.position.z = value.getDoubleValue();
				objFrameOffsetPtr->offset_pose.position.z = value.getDoubleValue();
#endif
				get_token(objThreadCntrolBlock);
				if(*(objThreadCntrolBlock->token) != ',')
					return 0 ;
				
			    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
				additionalInfomation.offset.pose_target.euler_.a_ = value.getDoubleValue();
				objFrameOffsetPtr->offset_pose.euler_.a_ = value.getDoubleValue();
#else
				additionalInfomation.offset.pose_target.orientation.a = value.getDoubleValue();
				objFrameOffsetPtr->offset_pose.orientation.a = value.getDoubleValue();
#endif
				get_token(objThreadCntrolBlock);
				if(*(objThreadCntrolBlock->token) != ',')
					return 0 ;
				
			    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
				additionalInfomation.offset.pose_target.euler_.b_ = value.getDoubleValue();
				objFrameOffsetPtr->offset_pose.euler_.b_ = value.getDoubleValue();
#else
				additionalInfomation.offset.pose_target.orientation.b = value.getDoubleValue();
				objFrameOffsetPtr->offset_pose.orientation.b = value.getDoubleValue();
#endif
				get_token(objThreadCntrolBlock);
				if(*(objThreadCntrolBlock->token) != ',')
					return 0 ;
				
			    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
				additionalInfomation.offset.pose_target.euler_.c_ = value.getDoubleValue();
				objFrameOffsetPtr->offset_pose.euler_.c_ = value.getDoubleValue();
#else
				additionalInfomation.offset.pose_target.orientation.c = value.getDoubleValue();
				objFrameOffsetPtr->offset_pose.orientation.c = value.getDoubleValue();
#endif
				get_token(objThreadCntrolBlock);
				if(*(objThreadCntrolBlock->token) != ')')
					return 0 ;
				
				if(additionalInfomation.offset.type == C_VEC_UF)
				{
	  				get_token(objThreadCntrolBlock);
					if(strcasecmp(objThreadCntrolBlock->token, "pr") == 0)
						additionalInfomation.offset.uf_reg.type = POSE_REG;
					else if(strcasecmp(objThreadCntrolBlock->token, "uf") == 0)
						additionalInfomation.offset.uf_reg.type = NUM_REG;
					
					get_token(objThreadCntrolBlock);
					if(*(objThreadCntrolBlock->token) != '[') {
						return 0 ;
					}
					get_exp(objThreadCntrolBlock, &value, &boolValue);
					additionalInfomation.offset.uf_reg.index = (int)value.getDoubleValue();
					//
					objFrameOffsetPtr->offset_frame_id = (int)value.getDoubleValue();
					//
					get_token(objThreadCntrolBlock);
					if(*(objThreadCntrolBlock->token) != ']') {
						return 0 ;
					}
				}	
				else if(additionalInfomation.offset.type == C_VEC_TF)
				{
					get_token(objThreadCntrolBlock);
					if(strcasecmp(objThreadCntrolBlock->token, "pr") == 0)
						additionalInfomation.offset.uf_reg.type = POSE_REG;
					else if(strcasecmp(objThreadCntrolBlock->token, "tf") == 0)
						additionalInfomation.offset.uf_reg.type = NUM_REG;
					
					get_token(objThreadCntrolBlock);
					if(*(objThreadCntrolBlock->token) != '[') {
						return 0 ;
					}
					get_exp(objThreadCntrolBlock, &value, &boolValue);
					additionalInfomation.offset.uf_reg.index = (int)value.getDoubleValue();
					//
					objFrameOffsetPtr->offset_frame_id = (int)value.getDoubleValue();
					//
					get_token(objThreadCntrolBlock);
					if(*(objThreadCntrolBlock->token) != ']') {
						return 0 ;
					}
				}	
				else 
				{
					// Current offset_frame_id
					objFrameOffsetPtr->offset_frame_id = -1;
				}
			}
			else if(strncasecmp(objThreadCntrolBlock->token, "j_vec", strlen("j_vec")) == 0)
			{
				if(strcasecmp(objThreadCntrolBlock->token, "j_vec") == 0)
					additionalInfomation.offset.type = J_VEC;
				else if(strcasecmp(objThreadCntrolBlock->token, "j_vec_uf") == 0)
					additionalInfomation.offset.type = J_VEC_UF;
				else if(strcasecmp(objThreadCntrolBlock->token, "j_vec_tf") == 0)
					additionalInfomation.offset.type = J_VEC_TF;
				
				// additionalInfomation.offset.joint_target ;
				get_token(objThreadCntrolBlock);
				if(*(objThreadCntrolBlock->token) != '(')
					return 0 ;
				
				//
				objFrameOffsetPtr->coord_type = COORDINATE_JOINT ;
				//
			    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
				additionalInfomation.offset.joint_target.j1_ = value.getDoubleValue();
				objFrameOffsetPtr->offset_joint.j1_ = value.getDoubleValue();
#else
				additionalInfomation.offset.joint_target.j1 = value.getDoubleValue();
				objFrameOffsetPtr->offset_joint.j1 = value.getDoubleValue();
#endif
				get_token(objThreadCntrolBlock);
				if(*(objThreadCntrolBlock->token) != ',')
					return 0 ;
				
			    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
				additionalInfomation.offset.joint_target.j2_ = value.getDoubleValue();
				objFrameOffsetPtr->offset_joint.j2_ = value.getDoubleValue();
#else
				additionalInfomation.offset.joint_target.j2 = value.getDoubleValue();
				objFrameOffsetPtr->offset_joint.j2 = value.getDoubleValue();
#endif
				get_token(objThreadCntrolBlock);
				if(*(objThreadCntrolBlock->token) != ',')
					return 0 ;
				
			    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
				additionalInfomation.offset.joint_target.j3_ = value.getDoubleValue();
				objFrameOffsetPtr->offset_joint.j3_ = value.getDoubleValue();
#else
				additionalInfomation.offset.joint_target.j3 = value.getDoubleValue();
				objFrameOffsetPtr->offset_joint.j3 = value.getDoubleValue();
#endif
				get_token(objThreadCntrolBlock);
				if(*(objThreadCntrolBlock->token) != ',')
					return 0 ;
				
			    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
				additionalInfomation.offset.joint_target.j4_ = value.getDoubleValue();
				objFrameOffsetPtr->offset_joint.j4_ = value.getDoubleValue();
#else
				additionalInfomation.offset.joint_target.j4 = value.getDoubleValue();
				objFrameOffsetPtr->offset_joint.j4 = value.getDoubleValue();
#endif
				get_token(objThreadCntrolBlock);
				if(*(objThreadCntrolBlock->token) != ',')
					return 0 ;
				
			    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
				additionalInfomation.offset.joint_target.j5_ = value.getDoubleValue();
				objFrameOffsetPtr->offset_joint.j5_ = value.getDoubleValue();
#else
				additionalInfomation.offset.joint_target.j5 = value.getDoubleValue();
				objFrameOffsetPtr->offset_joint.j5 = value.getDoubleValue();
#endif
				get_token(objThreadCntrolBlock);
				if(*(objThreadCntrolBlock->token) != ',')
					return 0 ;
				
			    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
				additionalInfomation.offset.joint_target.j6_ = value.getDoubleValue();
				objFrameOffsetPtr->offset_joint.j6_ = value.getDoubleValue();
#else
				additionalInfomation.offset.joint_target.j6 = value.getDoubleValue();
				objFrameOffsetPtr->offset_joint.j6 = value.getDoubleValue();
#endif
				get_token(objThreadCntrolBlock);
				if(*(objThreadCntrolBlock->token) != ')')
					return 0 ;
				
				if(additionalInfomation.offset.type == J_VEC_UF)
				{
	  				get_token(objThreadCntrolBlock);
					if(strcasecmp(objThreadCntrolBlock->token, "pr") == 0)
						additionalInfomation.offset.uf_reg.type = POSE_REG;
					else if(strcasecmp(objThreadCntrolBlock->token, "uf") == 0)
						additionalInfomation.offset.uf_reg.type = NUM_REG;
					
					get_token(objThreadCntrolBlock);
					if(*(objThreadCntrolBlock->token) != '[') {
						return 0 ;
					}
					get_exp(objThreadCntrolBlock, &value, &boolValue);
					additionalInfomation.offset.uf_reg.index = (int)value.getDoubleValue();
					//
					objFrameOffsetPtr->offset_frame_id = (int)value.getDoubleValue();
					//
					get_token(objThreadCntrolBlock);
					if(*(objThreadCntrolBlock->token) != ']') {
						return 0 ;
					}
				}
				else if(additionalInfomation.offset.type == J_VEC_TF)
				{
					get_token(objThreadCntrolBlock);
					if(strcasecmp(objThreadCntrolBlock->token, "pr") == 0)
						additionalInfomation.offset.uf_reg.type = POSE_REG;
					else if(strcasecmp(objThreadCntrolBlock->token, "tf") == 0)
						additionalInfomation.offset.uf_reg.type = NUM_REG;
					
					get_token(objThreadCntrolBlock);
					if(*(objThreadCntrolBlock->token) != '[') {
						return 0 ;
					}
					get_exp(objThreadCntrolBlock, &value, &boolValue);
					additionalInfomation.offset.uf_reg.index = (int)value.getDoubleValue();
					//
					objFrameOffsetPtr->offset_frame_id = (int)value.getDoubleValue();
					//
					get_token(objThreadCntrolBlock);
					if(*(objThreadCntrolBlock->token) != ']') {
						return 0 ;
					}
				}
				else 
				{
					// Current offset_frame_id
					objFrameOffsetPtr->offset_frame_id = -1;
				}
			}
			iCount++ ;
		}
		// 6.	EV同步附加轴速度指令（低优先级）
		else if(strncasecmp(objThreadCntrolBlock->token, "ev", 2) == 0)
		{
			additionalInfomation.type = EV ;
			get_exp(objThreadCntrolBlock, &value, &boolValue);
		//	int speed = (int)value.getDoubleValue();
			iCount++ ;
		}
		// 7.	Ind_EV非同步附加轴速度指令（低优先级）
		else if(strncasecmp(objThreadCntrolBlock->token, "ind_ev", 6) == 0)
		{
			additionalInfomation.type = IND_EV ;
			get_exp(objThreadCntrolBlock, &value, &boolValue);
		//	int speed = (int)value.getDoubleValue();
			iCount++ ;
		}
		// 8.	TB先执行指令
		else if((strcasecmp(objThreadCntrolBlock->token, "tb") == 0)
			  ||(strcasecmp(objThreadCntrolBlock->token, "ta") == 0)
			  ||(strcasecmp(objThreadCntrolBlock->token, "db") == 0))
		{
			if(strcasecmp(objThreadCntrolBlock->token, "tb") == 0)
				additionalInfomation.type = TB ;
			else if(strcasecmp(objThreadCntrolBlock->token, "ta") == 0)
				additionalInfomation.type = TA ;
			else if(strcasecmp(objThreadCntrolBlock->token, "db") == 0)
				additionalInfomation.type = DB ;
			
			get_exp(objThreadCntrolBlock, &value, &boolValue);
			additionalInfomation.execute.range = value.getDoubleValue() ;
			
			get_token(objThreadCntrolBlock);
			if(strcasecmp(objThreadCntrolBlock->token, "assign") == 0)
			{
  				get_token(objThreadCntrolBlock);
				if(strcasecmp(objThreadCntrolBlock->token, "pr") == 0)
					additionalInfomation.execute.assignment.type = POSE_REG;
				else if(strcasecmp(objThreadCntrolBlock->token, "r") == 0)
					additionalInfomation.execute.assignment.type = NUM_REG;
				
				get_token(objThreadCntrolBlock);
				if(*(objThreadCntrolBlock->token) != '[') {
					return 0 ;
				}
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				additionalInfomation.execute.assignment.index = (int)value.getDoubleValue();
				get_token(objThreadCntrolBlock);
				if(*(objThreadCntrolBlock->token) != ']') {
					return 0 ;
				}
				
				get_token(objThreadCntrolBlock);
				if(*(objThreadCntrolBlock->token)!=EQ) {
					return 0 ;
				}
                get_exp(objThreadCntrolBlock, &value, &boolValue);
				sprintf(additionalInfomation.execute.assignment.value, "%f", value.getDoubleValue());
			}
			else if(strcasecmp(objThreadCntrolBlock->token, "call") == 0)
			{
				// prog_demo_dec::prog_1 (1, 2)
				memset(additionalInfomation.execute.fname, 0x00, 128);
				get_token(objThreadCntrolBlock);
				sprintf(additionalInfomation.execute.fname, 
					"%s%s", additionalInfomation.execute.fname, 
						objThreadCntrolBlock->token);
				while (strcasecmp(objThreadCntrolBlock->token, ")") != 0)
				{
					get_token(objThreadCntrolBlock);
					sprintf(additionalInfomation.execute.fname, 
						"%s%s", additionalInfomation.execute.fname, 
						objThreadCntrolBlock->token);
				}
			}
			iCount++ ;
		}
		else if (*(objThreadCntrolBlock->token) == '\r')
		{
			break;
		}
	    get_token(objThreadCntrolBlock);
		memcpy(instrSetPtr, &additionalInfomation, sizeof(AdditionalInfomation));
		instrSetPtr += sizeof(AdditionalInfomation);
	}
	return iCount;
}


/************************************************* 
	Function:		set_global_TF
	Description:	set global TF
	Input:			iLineNum               -    Line Number
	Input:			iTFNum                 -    index of TF
	Input:			thread_control_block   - interpreter info
	Return: 		1        -    Success ;
*************************************************/ 
int set_global_TF(int iLineNum, int iTFNum, struct thread_control_block* objThreadCntrolBlock)
{
    Instruction instr;
	
	memset((char *)&instr, 0x00, sizeof(Instruction));
	instr.type = SET_TF ;
#ifdef USE_XPATH
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
		sprintf(instr.line, "%s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	else
		sprintf(instr.line, "OutRange with %d", iLineNum);
#else
	FST_INFO("call_MoveJ XPATH at %d", iLineNum);
	instr.line = iLineNum;
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
	{
		FST_INFO("set_global_UF XPATH: %s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	}
	else
		FST_INFO("OutRange with:  %d", iLineNum);
#endif
	instr.current_tf = iTFNum ;
	memcpy(objThreadCntrolBlock->instrSet, &instr, sizeof(Instruction));
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
	{
		FST_INFO("set_global_TF XPATH: %s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	}
	else
		FST_INFO("OutRange with:  %d", iLineNum);
	bool bRet = setInstruction(objThreadCntrolBlock, objThreadCntrolBlock->instrSet);
	while(bRet == false)
	{
		bRet = setInstruction(objThreadCntrolBlock, objThreadCntrolBlock->instrSet);
	}
    return 1;
}

/************************************************* 
	Function:		set_global_UF
	Description:	set global UF
	Input:			iLineNum               -    Line Number
	Input:			iUFNum                 -    index of UF
	Input:			thread_control_block   - interpreter info
	Return: 		1        -    Success ;
*************************************************/ 
int set_global_UF(int iLineNum, int iUFNum, struct thread_control_block* objThreadCntrolBlock)
{
    Instruction instr;
	
	memset((char *)&instr, 0x00, sizeof(Instruction));
	instr.type = SET_UF ;
#ifdef USE_XPATH
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
		sprintf(instr.line, "%s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	else
		sprintf(instr.line, "OutRange with %d", iLineNum);
#else
	FST_INFO("call_MoveJ XPATH at %d", iLineNum);
	instr.line = iLineNum;
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
	{
		FST_INFO("set_global_UF XPATH: %s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	}
	else
		FST_INFO("OutRange with:  %d", iLineNum);
#endif
	instr.current_uf = iUFNum ;
	memcpy(objThreadCntrolBlock->instrSet, &instr, sizeof(Instruction));
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
	{
		FST_INFO("set_global_UF XPATH: %s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	}
	else
		FST_INFO("OutRange with:  %d", iLineNum);
	bool bRet = setInstruction(objThreadCntrolBlock, objThreadCntrolBlock->instrSet);
	while(bRet == false)
	{
		bRet = setInstruction(objThreadCntrolBlock, objThreadCntrolBlock->instrSet);
	}
    return 1;
}

/************************************************* 
	Function:		set_OVC
	Description:	set OVC(Overall Velocity Coefficient).
	Input:			iLineNum               - Line Number
	Input:			dOVCNum                - value of OVC
	Input:			thread_control_block   - interpreter info
	Return: 		1        -    Success ;
*************************************************/ 
int set_OVC(int iLineNum, double dOVCNum, struct thread_control_block* objThreadCntrolBlock)
{
    Instruction instr;
	
	memset((char *)&instr, 0x00, sizeof(Instruction));
	instr.type = SET_OVC ;
#ifdef USE_XPATH
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
		sprintf(instr.line, "%s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	else
		sprintf(instr.line, "OutRange with %d", iLineNum);
#else
	FST_INFO("call_MoveJ XPATH at %d", iLineNum);
	instr.line = iLineNum;
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
	{
		FST_INFO("set_global_UF XPATH: %s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	}
	else
		FST_INFO("OutRange with:  %d", iLineNum);
#endif
	instr.current_ovc = dOVCNum / 100 ;
	memcpy(objThreadCntrolBlock->instrSet, &instr, sizeof(Instruction));
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
	{
		FST_INFO("set_global_UF XPATH: %s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	}
	else
		FST_INFO("OutRange with:  %d", iLineNum);
	bool bRet = setInstruction(objThreadCntrolBlock, objThreadCntrolBlock->instrSet);
	while(bRet == false)
	{
		bRet = setInstruction(objThreadCntrolBlock, objThreadCntrolBlock->instrSet);
	}
    return 1;
}

/************************************************* 
	Function:		set_OAC
	Description:	set OAC(Overall Acceleration Coefficient).
	Input:			iLineNum               - Line Number
	Input:			dOACNum                - value of OAC
	Input:			thread_control_block   - interpreter info
	Return: 		1        -    Success ;
*************************************************/ 
int set_OAC(int iLineNum, double dOACNum, struct thread_control_block* objThreadCntrolBlock)
{
    Instruction instr;
	
	memset((char *)&instr, 0x00, sizeof(Instruction));
	instr.type = SET_OAC ;
#ifdef USE_XPATH
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
		sprintf(instr.line, "%s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	else
		sprintf(instr.line, "OutRange with %d", iLineNum);
#else
	FST_INFO("call_MoveJ XPATH at %d", iLineNum);
	instr.line = iLineNum;
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
	{
		FST_INFO("set_global_UF XPATH: %s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	}
	else
		FST_INFO("OutRange with:  %d", iLineNum);
#endif
	instr.current_oac = dOACNum / 100 ;
	memcpy(objThreadCntrolBlock->instrSet, &instr, sizeof(Instruction));
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
	{
		FST_INFO("set_global_UF XPATH: %s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	}
	else
		FST_INFO("OutRange with:  %d", iLineNum);
	bool bRet = setInstruction(objThreadCntrolBlock, objThreadCntrolBlock->instrSet);
	while(bRet == false)
	{
		bRet = setInstruction(objThreadCntrolBlock, objThreadCntrolBlock->instrSet);
	}
    return 1;
}


/************************************************* 
	Function:		call_MoveJ
	Description:	Execute MOVEJ
	                FORMAT: MOVEJ P[1], 250 CNT -1  +���Ӳ���
	Input:			iLineNum               - Line Number
	Input:			thread_control_block   - interpreter info
	Return: 		1        -    Success ;
*************************************************/ 
int call_MoveJ(int iLineNum, struct thread_control_block* objThreadCntrolBlock)
{  
    MoveCommandDestination movCmdDst ;
	eval_value value;
	int boolValue;
    
	// char commandParam[1024];
    Instruction instr;
    // char * commandParamPtr = commandParam;
	
	memset((char *)&instr, 0x00, sizeof(Instruction));
	instr.type = MOTION ;
	instr.target.type = MOTION_JOINT;
#ifdef USE_XPATH
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
		sprintf(instr.line, "%s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	else
		sprintf(instr.line, "OutRange with %d", iLineNum);
#else
	FST_INFO("call_MoveJ XPATH at %d", iLineNum);
	instr.line = iLineNum;
#endif
	// Save start position
	// memset(&movCmdDst ,0x00, sizeof(MoveCommandDestination));
	getMoveCommandDestination(movCmdDst);
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
	{
	    FST_INFO("call_MoveJ Run XPATH: %d: %s", iLineNum, objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	    // FST_INFO("call_MoveJ Run movCmdDst: %08X with(%08X, %08X, %08X, %08X)", 
		//  	movCmdDst.type, MOTION_NONE, MOTION_JOINT, MOTION_LINE, MOTION_CIRCLE);
		if(movCmdDst.type != MOTION_NONE)
		{
			if(objThreadCntrolBlock->start_mov_position.find(iLineNum)
				==objThreadCntrolBlock->start_mov_position.end())
			{
			//    FST_INFO("Forward move to JOINT and Insert:(%f, %f, %f, %f, %f, %f) in MovJ", 
			//		movCmdDst.joint_target.j1, movCmdDst.joint_target.j2, 
			//		movCmdDst.joint_target.j3, movCmdDst.joint_target.j4, 
			//		movCmdDst.joint_target.j5, movCmdDst.joint_target.j6);
				try
				{
					objThreadCntrolBlock->start_mov_position.insert(
						map<int, MoveCommandDestination>::value_type(iLineNum, 
										movCmdDst));
				}
				catch (std::exception& e)
				{
				    std::cerr << "Exception catched : " << e.what() << std::endl;
				}
			}
			else
			{
	            FST_INFO("call_MoveJ Run XPATH: %s exists ", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
			}
		}
		else
		{
			FST_INFO("call_MoveJ XPATH without movCmdDst");
		}
	}
	else
	{
		; // FST_INFO("call_MoveJ XPATH out of range at %d", iLineNum);
	}
	// FST_INFO("call_MoveJ XPATH: %s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());

    get_exp(objThreadCntrolBlock, &value, &boolValue);
	FST_INFO("call_MoveJ value.getType() = %d", value.getIntType());
//	if(value.hasType(TYPE_NONE) == TYPE_NONE)
//	{
//	    FST_INFO("value.getIntType() == TYPE_INT in MovJ");
//		find_eol(objThreadCntrolBlock);
//		return 0;
//	}
//	else 
	if((value.hasType(TYPE_INT) == TYPE_INT) || (value.hasType(TYPE_DOUBLE) == TYPE_DOUBLE))
	{
		instr.target.target.type      = COORDINATE_JOINT ;
#ifndef WIN32
		instr.target.target.joint.j1_ = value.getDoubleValue();
#else
		instr.target.target.joint.j1 = value.getDoubleValue();
#endif
		get_token(objThreadCntrolBlock);
		
	    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.target.joint.j2_ = value.getDoubleValue();
#else
		instr.target.target.joint.j2 = value.getDoubleValue();
#endif
		get_token(objThreadCntrolBlock);
		
	    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.target.joint.j3_ = value.getDoubleValue();
#else
		instr.target.target.joint.j3 = value.getDoubleValue();
#endif
		get_token(objThreadCntrolBlock);
		
	    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.target.joint.j4_ = value.getDoubleValue();
#else
		instr.target.target.joint.j4 = value.getDoubleValue();
#endif
		get_token(objThreadCntrolBlock);
		
	    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.target.joint.j5_ = value.getDoubleValue();
#else
		instr.target.target.joint.j5 = value.getDoubleValue();
#endif
		get_token(objThreadCntrolBlock);
		
	    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.target.joint.j6_ = value.getDoubleValue();
#else
		instr.target.target.joint.j6 = value.getDoubleValue();
#endif
		instr.target.user_frame_id = instr.target.tool_frame_id = -1 ;
	    FST_INFO("value.getIntType() == TYPE_DOUBLE in MovJ");
	}
	else if(value.hasType(TYPE_POSE) == TYPE_POSE)
	{
		instr.target.target.type      = COORDINATE_CARTESIAN ;
		instr.target.target.pose.pose = value.getPoseValue();
		instr.target.user_frame_id = value.getUFIndex();
		instr.target.tool_frame_id = value.getTFIndex();
		instr.target.target.pose.posture = value.getPosture();
		instr.target.target.pose.turn = value.getTurn();
		
		// instr.target.pose_target = value.getPoseValue();
	    FST_INFO("value.getIntType() == TYPE_POSE in MovJ");
		
#ifndef WIN32
	    FST_INFO("Forward movej to POSE:(%f, %f, %f, %f, %f, %f) in MovL", 
			instr.target.target.pose.pose.point_.x_, instr.target.target.pose.pose.point_.y_, 
			instr.target.target.pose.pose.point_.z_, instr.target.target.pose.pose.euler_.a_, 
			instr.target.target.pose.pose.euler_.b_, instr.target.target.pose.pose.euler_.c_);
#else
	    FST_INFO("Forward movej to POSE:(%f, %f, %f, %f, %f, %f) in MovL", 
			instr.target.target.pose.pose.position.x, instr.target.target.pose.pose.position.y, 
			instr.target.target.pose.pose.position.z, instr.target.target.pose.pose.orientation.a, 
			instr.target.target.pose.pose.orientation.b, instr.target.target.pose.pose.orientation.c);
#endif
	//	serror(objThreadCntrolBlock, 16);
	//	find_eol(objThreadCntrolBlock);
    //	return 0;
	}
	else if(value.hasType(TYPE_JOINT) == TYPE_JOINT)
	{
		instr.target.target.type      = COORDINATE_JOINT ;
		instr.target.target.joint = value.getJointValue();
		instr.target.user_frame_id = value.getUFIndex();
		instr.target.tool_frame_id = value.getTFIndex();
		instr.target.target.pose.posture = value.getPosture();
		instr.target.target.pose.turn = value.getTurn();
		
	    FST_INFO("Forward movej to JOINT:(%f, %f, %f, %f, %f, %f) in MovJ", 
#ifndef WIN32
			instr.target.target.joint.j1_, instr.target.target.joint.j2_, 
			instr.target.target.joint.j3_, instr.target.target.joint.j4_, 
			instr.target.target.joint.j5_, instr.target.target.joint.j6_);
#else
			instr.target.target.joint.j1, instr.target.target.joint.j2, 
			instr.target.target.joint.j3, instr.target.target.joint.j4, 
			instr.target.target.joint.j5, instr.target.target.joint.j6);
#endif		
	}
	
	// Use start point in revert mode  
	if(objThreadCntrolBlock->execute_direction == EXECUTE_BACKWARD)
	{
	     FST_INFO("Use start point in revert mode.");
		 
		if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
		{
		    FST_INFO("call_MoveJ XPATH: %s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
			if(objThreadCntrolBlock->start_mov_position.find(iLineNum)
				!=objThreadCntrolBlock->start_mov_position.end())
			{
			    instr.target.target.joint
				 	= objThreadCntrolBlock->start_mov_position[iLineNum].joint_target;
			}
			else
			{
				FST_INFO("call_MoveL XPATH without StartJoint");
			}
		}
		else
		{
			FST_INFO("call_MoveJ XPATH out of range at %d", iLineNum);
		}
		 
#ifndef WIN32
	    FST_INFO("Backward move to JOINT:(%f, %f, %f, %f, %f, %f) in MovJ", 
			instr.target.target.joint.j1_, instr.target.target.joint.j2_, 
			instr.target.target.joint.j3_, instr.target.target.joint.j4_, 
			instr.target.target.joint.j5_, instr.target.target.joint.j6_);
#else
	    FST_INFO("Backward move to JOINT:(%f, %f, %f, %f, %f, %f) in MovJ", 
			instr.target.target.joint.j1, instr.target.target.joint.j2, 
			instr.target.target.joint.j3, instr.target.target.joint.j4, 
			instr.target.target.joint.j5, instr.target.target.joint.j6);
#endif		
	}
	get_token(objThreadCntrolBlock);

    get_exp(objThreadCntrolBlock, &value, &boolValue);
	// Divide 100 as percent.
	FST_INFO("get_token =  '%f'", value.getDoubleValue());
    instr.target.vel        = value.getDoubleValue() / 100;
	
	get_token(objThreadCntrolBlock);
	if(strcasecmp(objThreadCntrolBlock->token, SMOOTH_TYPE_CNT) == 0)
    {
		// 平滑参数的范围
    	get_exp(objThreadCntrolBlock, &value, &boolValue);
    	if(objThreadCntrolBlock->prog_mode == STEP_MODE)
    	{
			instr.target.smooth_type = SMOOTH_NONE;
        	instr.target.cnt = -1;
    	}
	    else
	    {
			// // 如果是FINE语句CNT应为-1
	        if(value.getDoubleValue() < 0) // == -1
	    	{
				instr.target.smooth_type = SMOOTH_NONE;
	        	instr.target.cnt = -1;
	     		FST_INFO("instr.target.cnt = %f in the FINE.", instr.target.cnt);
	    	}
	        else 
			{
				// 速度平滑[0.0, 1.0]，
				instr.target.smooth_type = SMOOTH_VELOCITY;
				instr.target.cnt = value.getDoubleValue() / 100;
			}
	    }
    }
    else
    {
		if(strcasecmp(objThreadCntrolBlock->token, SMOOTH_TYPE_SD) == 0)
		{
			// 距离平滑[0.0, +∞]，
			get_exp(objThreadCntrolBlock, &value, &boolValue);
			if(value.getDoubleValue() < 0) // == -1
			{
				instr.target.smooth_type = SMOOTH_NONE;
				instr.target.cnt = -1;
				FST_INFO("instr.target.cnt = %f in the FINE.", instr.target.cnt);
			}
			else 
			{
				instr.target.smooth_type = SMOOTH_DISTANCE;
        		instr.target.cnt = value.getDoubleValue() ; 
			}
		}
		else if(strcasecmp(objThreadCntrolBlock->token, SMOOTH_TYPE_SV) == 0)
		{
			// 速度平滑[0.0, 1.0]，
    		get_exp(objThreadCntrolBlock, &value, &boolValue);
			if(value.getDoubleValue() < 0) // == -1
			{
				instr.target.smooth_type = SMOOTH_NONE;
				instr.target.cnt = -1;
				FST_INFO("instr.target.cnt = %f in the FINE.", instr.target.cnt);
			}
			else 
			{
				instr.target.smooth_type = SMOOTH_VELOCITY;
				instr.target.cnt = value.getDoubleValue() / 100;
			}
		}
		else 
		{
			instr.target.smooth_type = SMOOTH_NONE;
			instr.target.cnt = -1.0000;
		}
    }
	// instr.target.acc = -1 ;
	// Set to instrSet
	memcpy(objThreadCntrolBlock->instrSet, &instr, sizeof(Instruction));
	objThreadCntrolBlock->instrSet->target.acc = 1.0 ;
	objThreadCntrolBlock->instrSet->target.user_frame_offset.valid = false;
	objThreadCntrolBlock->instrSet->target.tool_frame_offset.valid = false;
	
	get_token(objThreadCntrolBlock);
	// result.size() > MOVJ_COMMAND_PARAM_MIN
	if(objThreadCntrolBlock->token_type == DELIMITER)
	{
		char * instrSetPtr = 
				(char *)objThreadCntrolBlock->instrSet
				+ sizeof(Instruction) - sizeof(char) ;
		objThreadCntrolBlock->instrSet->is_additional = true ;
		if(*(objThreadCntrolBlock->token) == ';')
		{
			objThreadCntrolBlock->instrSet->add_num    =  
				getAditionalInfomation(objThreadCntrolBlock, instrSetPtr);
		}
		else
		{
			AdditionalInfomation additionalInfomation ;
			additionalInfomation.type = ACC ;
			additionalInfomation.acc_speed = 100 ;
			memcpy(instrSetPtr, &additionalInfomation, sizeof(AdditionalInfomation));
			
			objThreadCntrolBlock->instrSet->add_num    = 1 ;
		}
	}
	
	if(objThreadCntrolBlock->instrSet->target.user_frame_offset.valid)
	{
		if(objThreadCntrolBlock->instrSet->target.user_frame_offset.coord_type == COORDINATE_JOINT)
		{
#ifndef WIN32
	    FST_INFO("user_frame_offset to JOINT:(%f, %f, %f, %f, %f, %f) in MovJ with %d", 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j1_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j2_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j3_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j4_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j5_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j6_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_frame_id);
#else
	    FST_INFO("user_frame_offset to JOINT:(%f, %f, %f, %f, %f, %f) in MovJ with %d", 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j1, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j2, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j3, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j4, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j5, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j6, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_frame_id);
#endif	
		}
		else if(objThreadCntrolBlock->instrSet->target.user_frame_offset.coord_type == COORDINATE_CARTESIAN)
		{
#ifndef WIN32
	    FST_INFO("user_frame_offset to CART:(%f, %f, %f, %f, %f, %f) in MovJ with %d", 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.point_.x_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.point_.y_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.point_.z_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.euler_.a_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.euler_.b_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.euler_.c_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_frame_id);
#else
	    FST_INFO("user_frame_offset to CART:(%f, %f, %f, %f, %f, %f) in MovJ with %d", 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.position.x, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.position.y, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.position.z, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.orientation.a, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.orientation.b, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.orientation.c, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_frame_id);
#endif	
		}
	}
	if(objThreadCntrolBlock->instrSet->target.tool_frame_offset.valid)
	{
		if(objThreadCntrolBlock->instrSet->target.tool_frame_offset.coord_type == COORDINATE_JOINT)
		{
#ifndef WIN32
	    FST_INFO("tool_frame_offset to JOINT:(%f, %f, %f, %f, %f, %f) in MovJ with %d", 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j1_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j2_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j3_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j4_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j5_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j6_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_frame_id);
#else
	    FST_INFO("tool_frame_offset to JOINT:(%f, %f, %f, %f, %f, %f) in MovJ with %d", 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j1, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j2, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j3, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j4, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j5, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j6, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_frame_id);
#endif	
		}
		else if(objThreadCntrolBlock->instrSet->target.tool_frame_offset.coord_type == COORDINATE_CARTESIAN)
		{
#ifndef WIN32
	    FST_INFO("tool_frame_offset to CART:(%f, %f, %f, %f, %f, %f) in MovJ with %d", 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.point_.x_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.point_.y_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.point_.z_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.euler_.a_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.euler_.b_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.euler_.c_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_frame_id);
#else
	    FST_INFO("tool_frame_offset to CART:(%f, %f, %f, %f, %f, %f) in MovJ with %d", 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.position.x, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.position.y, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.position.z, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.orientation.a, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.orientation.b, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.orientation.c, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_frame_id);
#endif	
		}
	}
	
	// objThreadCntrolBlock->currentMotionTarget = instr.target ;
// 	#ifdef USE_XPATH
// 		FST_INFO("setInstruction MOTION_JOINT at %s", instr.line);
// 	#else
// 		FST_INFO("setInstruction MOTION_JOINT at %d", instr.line);
// 	#endif
	FST_INFO("call_MoveJ instr.target.cnt = %f setInstruction with %f.", 
				objThreadCntrolBlock->instrSet->target.cnt, 
				objThreadCntrolBlock->instrSet->target.acc);
	bool bRet = setInstruction(objThreadCntrolBlock, objThreadCntrolBlock->instrSet);
	while(bRet == false)
	{
		bRet = setInstruction(objThreadCntrolBlock, objThreadCntrolBlock->instrSet);
#ifdef WIN32
		Sleep(1);
#else
        usleep(1000);
#endif
	}
    return 1;   
}

/************************************************* 
	Function:		call_MoveL
	Description:	Execute MOVEL
	                FORMAT: MOVEL P[1], 250 CNT -1  +���Ӳ���
	Input:			iLineNum               - Line Number
	Input:			thread_control_block   - interpreter info
	Return: 		1        -    Success ;
*************************************************/ 
int call_MoveL(int iLineNum, struct thread_control_block* objThreadCntrolBlock)
{  
    MoveCommandDestination movCmdDst ;
	eval_value value;
	int boolValue;
	
	// char commandParam[1024];
    Instruction instr;
    // char * commandParamptr = commandParam;
	
	memset((char *)&instr, 0x00, sizeof(Instruction));
	instr.type = MOTION ;
	instr.target.type = MOTION_LINE;
#ifdef USE_XPATH
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
		sprintf(instr.line, "%s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	else
		sprintf(instr.line, "OutRange with %d", iLineNum);
#else
	FST_INFO("call_MoveJ XPATH at %d", iLineNum);
	instr.line = iLineNum;
#endif
	// Save start position
	// memset(&movCmdDst ,0x00, sizeof(MoveCommandDestination));
	getMoveCommandDestination(movCmdDst);
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
	{
	    FST_INFO("call_MoveL Run XPATH: %d: %s", iLineNum, objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	    // FST_INFO("call_MoveL Run movCmdDst: %08X with(%08X, %08X, %08X, %08X)", 
		//  	movCmdDst.type, MOTION_NONE, MOTION_JOINT, MOTION_LINE, MOTION_CIRCLE);
		if(movCmdDst.type != MOTION_NONE)
		{
			if(objThreadCntrolBlock->start_mov_position.find(iLineNum)
				==objThreadCntrolBlock->start_mov_position.end())
			{
			//    FST_INFO("move from POSE and insert:(%f, %f, %f, %f, %f, %f) in MovL", 
			//		movCmdDst.pose_target.position.x,    movCmdDst.pose_target.position.y, 
			//		movCmdDst.pose_target.position.z,    movCmdDst.pose_target.orientation.a, 
			//		movCmdDst.pose_target.orientation.b, movCmdDst.pose_target.orientation.c);
				try
				{
					objThreadCntrolBlock->start_mov_position.insert(
						map<int, MoveCommandDestination>::value_type(iLineNum, 
											movCmdDst));
				}
				catch (std::exception& e)
				{
				    std::cerr << "Exception catched : " << e.what() << std::endl;
				}
			}
			else
			{
	            FST_INFO("call_MoveL Run XPATH: %s exists ", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
			}
		}
		else
		{
			; // FST_INFO("call_MoveL XPATH without movCmdDst");
		}
	}
	else
	{
		FST_INFO("call_MoveL XPATH out of range at %d", iLineNum);
	}
	// FST_INFO("call_MoveL Run XPATH: %s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	
    // result.size() == MOVJ_COMMAND_PARAM_MIN
    get_exp(objThreadCntrolBlock, &value, &boolValue);
	FST_INFO("call_MoveL value.getIntType() = %d", value.getIntType());
//	if(value.hasType(TYPE_NONE) == TYPE_NONE)
//	{
//		find_eol(objThreadCntrolBlock);
//		return 0;
//	}
//	else 
	if((value.hasType(TYPE_INT) == TYPE_INT) || (value.hasType(TYPE_DOUBLE) == TYPE_DOUBLE))
	{
		instr.target.target.type      = COORDINATE_CARTESIAN ;
#ifndef WIN32
		instr.target.target.pose.pose.point_.x_    = value.getDoubleValue();
#else
		instr.target.target.pose.pose.position.x    = value.getDoubleValue();
#endif
		get_token(objThreadCntrolBlock);
		
		get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.target.pose.pose.point_.y_    = value.getDoubleValue();
#else
		instr.target.target.pose.pose.position.y    = value.getDoubleValue();
#endif
		get_token(objThreadCntrolBlock);
		
		get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.target.pose.pose.point_.z_    = value.getDoubleValue();
#else
		instr.target.target.pose.pose.position.z    = value.getDoubleValue();
#endif
		get_token(objThreadCntrolBlock);
		
		get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.target.pose.pose.euler_.a_ = value.getDoubleValue();
#else
		instr.target.target.pose.pose.orientation.a = value.getDoubleValue();
#endif
		get_token(objThreadCntrolBlock);
		
		get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.target.pose.pose.euler_.b_ = value.getDoubleValue();
#else
		instr.target.target.pose.pose.orientation.b = value.getDoubleValue();
#endif
		get_token(objThreadCntrolBlock);
		
		get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.target.pose.pose.euler_.c_ = value.getDoubleValue();
#else
		instr.target.target.pose.pose.orientation.c = value.getDoubleValue();
#endif

		instr.target.user_frame_id = instr.target.tool_frame_id = -1 ;
	}
	else if(value.hasType(TYPE_POSE) == TYPE_POSE)
	{
		instr.target.target.type      = COORDINATE_CARTESIAN ;
		instr.target.target.pose.pose = value.getPoseValue();
		instr.target.user_frame_id = value.getUFIndex();
		instr.target.tool_frame_id = value.getTFIndex();
		instr.target.target.pose.posture = value.getPosture();
		instr.target.target.pose.turn    = value.getTurn();
		
#ifndef WIN32
	    FST_INFO("Forward movel to POSE:(%f, %f, %f, %f, %f, %f) in MovL", 
			instr.target.target.pose.pose.point_.x_, instr.target.target.pose.pose.point_.y_, 
			instr.target.target.pose.pose.point_.z_, instr.target.target.pose.pose.euler_.a_, 
			instr.target.target.pose.pose.euler_.b_, instr.target.target.pose.pose.euler_.c_);
#else
	    FST_INFO("Forward movel to POSE:(%f, %f, %f, %f, %f, %f) in MovL", 
			instr.target.target.pose.pose.position.x, instr.target.target.pose.pose.position.y, 
			instr.target.target.pose.pose.position.z, instr.target.target.pose.pose.orientation.a, 
			instr.target.target.pose.pose.orientation.b, instr.target.target.pose.pose.orientation.c);
#endif
	}
	else if(value.hasType(TYPE_JOINT) == TYPE_JOINT)
	{
		instr.target.target.type      = COORDINATE_JOINT ;
		instr.target.target.joint     = value.getJointValue();
		instr.target.user_frame_id = value.getUFIndex();
		instr.target.tool_frame_id = value.getTFIndex();
		
		instr.target.target.pose.posture = value.getPosture();
		instr.target.target.pose.turn    = value.getTurn();
		
		// instr.target.joint_target = value.getJointValue();
	    FST_INFO("value.getIntType() == TYPE_JOINT in MovL");
	//	serror(objThreadCntrolBlock, 15);
	//	find_eol(objThreadCntrolBlock);
    //	return 0;
	}
	
	// Use start point in revert mode  
	if(objThreadCntrolBlock->execute_direction == EXECUTE_BACKWARD)
	{
	     FST_INFO("Use start point in revert mode.");
		 
		if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
		{
		    FST_INFO("call_MoveL XPATH: %s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
			if(objThreadCntrolBlock->start_mov_position.find(iLineNum)
				!=objThreadCntrolBlock->start_mov_position.end())
			{
			    instr.target.target.pose.pose
				 	= objThreadCntrolBlock->start_mov_position[iLineNum].pose_target;
			}
			else
			{
				FST_INFO("call_MoveL XPATH without StartPoint");
			}
		}
		else
		{
			FST_INFO("call_MoveL XPATH out of range at %d", iLineNum);
		}
		 
#ifndef WIN32
	    FST_INFO("Backward move to POSE:(%f, %f, %f, %f, %f, %f) in MovL", 
			instr.target.target.pose.pose.point_.x_, instr.target.target.pose.pose.point_.y_, 
			instr.target.target.pose.pose.point_.z_, instr.target.target.pose.pose.euler_.a_, 
			instr.target.target.pose.pose.euler_.b_, instr.target.target.pose.pose.euler_.c_);
#else
	    FST_INFO("Backward move to POSE:(%f, %f, %f, %f, %f, %f) in MovL", 
			instr.target.target.pose.pose.position.x, instr.target.target.pose.pose.position.y, 
			instr.target.target.pose.pose.position.z, instr.target.target.pose.pose.orientation.a, 
			instr.target.target.pose.pose.orientation.b, instr.target.target.pose.pose.orientation.c);
#endif	
	}
	
	get_token(objThreadCntrolBlock);
    get_exp(objThreadCntrolBlock, &value, &boolValue);
    instr.target.vel                  = value.getDoubleValue();

	get_token(objThreadCntrolBlock);
	if(strcasecmp(objThreadCntrolBlock->token, SMOOTH_TYPE_CNT) == 0)
    {
		// 平滑参数的范围
    	get_exp(objThreadCntrolBlock, &value, &boolValue);
	    FST_INFO("instr.target.cnt = %f setInstruction.", value.getDoubleValue());
    	if(objThreadCntrolBlock->prog_mode == STEP_MODE)
    	{
			instr.target.smooth_type = SMOOTH_NONE;
        	instr.target.cnt = -1;
    	}
	    else
	    {
			// 如果是FINE语句CNT应为-1
	        if(value.getDoubleValue() < 0) // == -1
	    	{
				instr.target.smooth_type = SMOOTH_NONE;
	        	instr.target.cnt = -1.0000;
	     		FST_INFO("instr.target.cnt = %f in the FINE.", instr.target.cnt);
	    	}
	        else 
			{
				// 速度平滑[0.0, 1.0]
				instr.target.smooth_type = SMOOTH_VELOCITY;
				instr.target.cnt = value.getDoubleValue() / 100;
			}
	    }
    }
    else
    {
		if(strcasecmp(objThreadCntrolBlock->token, SMOOTH_TYPE_SD) == 0)
		{
			instr.target.smooth_type = SMOOTH_DISTANCE;
			// 距离平滑[0.0, +∞]，
			get_exp(objThreadCntrolBlock, &value, &boolValue);
			if(value.getDoubleValue() < 0) // == -1
			{
				instr.target.smooth_type = SMOOTH_NONE;
				instr.target.cnt = -1;
				FST_INFO("instr.target.cnt = %f in the FINE.", instr.target.cnt);
			}
			else 
			{
				instr.target.smooth_type = SMOOTH_DISTANCE;
        		instr.target.cnt = value.getDoubleValue() ; 
			}
		}
		else if(strcasecmp(objThreadCntrolBlock->token, SMOOTH_TYPE_SV) == 0)
		{
			instr.target.smooth_type = SMOOTH_VELOCITY;
			// 速度平滑[0.0, 1.0]，
    		get_exp(objThreadCntrolBlock, &value, &boolValue);
			if(value.getDoubleValue() < 0) // == -1
			{
				instr.target.smooth_type = SMOOTH_NONE;
				instr.target.cnt = -1;
				FST_INFO("instr.target.cnt = %f in the FINE.", instr.target.cnt);
			}
			else 
			{
				instr.target.smooth_type = SMOOTH_VELOCITY;
				instr.target.cnt = value.getDoubleValue() / 100;
			}
		}
		else 
		{
			instr.target.smooth_type = SMOOTH_NONE;
			instr.target.cnt = -1.0000;
		}
    }
	// instr.target.acc = -1 ;
	// Set to instrSet
	memcpy(objThreadCntrolBlock->instrSet, &instr, sizeof(Instruction));
	objThreadCntrolBlock->instrSet->target.acc = 1.0 ;
	
	get_token(objThreadCntrolBlock);
	// result.size() > MOVJ_COMMAND_PARAM_MIN
	if(objThreadCntrolBlock->token_type == DELIMITER)
	{
		char * instrSetPtr = 
			(char *)objThreadCntrolBlock->instrSet
			+ sizeof(Instruction) - sizeof(char) ;
		objThreadCntrolBlock->instrSet->is_additional = true ;
		if(*(objThreadCntrolBlock->token) == ';')
		{
			objThreadCntrolBlock->instrSet->add_num    =  
				getAditionalInfomation(objThreadCntrolBlock, instrSetPtr);
		}
		else
		{
			AdditionalInfomation additionalInfomation ;
			additionalInfomation.type = ACC ;
			additionalInfomation.acc_speed = 100 ;
			memcpy(instrSetPtr, &additionalInfomation, sizeof(AdditionalInfomation));
			
			objThreadCntrolBlock->instrSet->add_num    = 1 ;
		}
	}
	// FST_INFO("MOVL: instr.target.accleration = %f .", instr.target.acc);
	
	if(objThreadCntrolBlock->instrSet->target.user_frame_offset.valid)
	{
		if(objThreadCntrolBlock->instrSet->target.user_frame_offset.coord_type == COORDINATE_JOINT)
		{
#ifndef WIN32
	    FST_INFO("user_frame_offset to JOINT:(%f, %f, %f, %f, %f, %f) in MovL with %d", 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j1_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j2_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j3_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j4_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j5_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j6_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_frame_id);
#else
	    FST_INFO("user_frame_offset to JOINT:(%f, %f, %f, %f, %f, %f) in MovL with %d", 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j1, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j2, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j3, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j4, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j5, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j6, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_frame_id);
#endif	
		}
		else if(objThreadCntrolBlock->instrSet->target.user_frame_offset.coord_type == COORDINATE_CARTESIAN)
		{
#ifndef WIN32
	    FST_INFO("user_frame_offset to CART:(%f, %f, %f, %f, %f, %f) in MovL with %d", 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.point_.x_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.point_.y_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.point_.z_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.euler_.a_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.euler_.b_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.euler_.c_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_frame_id);
#else
	    FST_INFO("user_frame_offset to CART:(%f, %f, %f, %f, %f, %f) in MovL with %d", 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.position.x, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.position.y, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.position.z, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.orientation.a, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.orientation.b, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.orientation.c, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_frame_id);
#endif	
		}
	}
	if(objThreadCntrolBlock->instrSet->target.tool_frame_offset.valid)
	{
		if(objThreadCntrolBlock->instrSet->target.tool_frame_offset.coord_type == COORDINATE_JOINT)
		{
#ifndef WIN32
	    FST_INFO("tool_frame_offset to JOINT:(%f, %f, %f, %f, %f, %f) in MovL with %d", 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j1_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j2_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j3_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j4_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j5_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j6_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_frame_id);
#else
	    FST_INFO("tool_frame_offset to JOINT:(%f, %f, %f, %f, %f, %f) in MovL with %d", 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j1, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j2, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j3, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j4, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j5, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j6, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_frame_id);
#endif	
		}
		else if(objThreadCntrolBlock->instrSet->target.tool_frame_offset.coord_type == COORDINATE_CARTESIAN)
		{
#ifndef WIN32
	    FST_INFO("tool_frame_offset to CART:(%f, %f, %f, %f, %f, %f) in MovL with %d", 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.point_.x_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.point_.y_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.point_.z_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.euler_.a_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.euler_.b_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.euler_.c_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_frame_id);
#else
	    FST_INFO("tool_frame_offset to CART:(%f, %f, %f, %f, %f, %f) in MovL with %d", 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.position.x, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.position.y, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.position.z, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.orientation.a, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.orientation.b, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.orientation.c, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_frame_id);
#endif	
		}
	}
	
// 	#ifdef USE_XPATH
// 		FST_INFO("setInstruction MOTION_LINE at %s", instr.line);
// 	#else
// 		FST_INFO("setInstruction MOTION_LINE at %d", instr.line);
// 	#endif
	
	FST_INFO("call_MoveL instr.target.cnt = %f setInstruction with %f.", 
				objThreadCntrolBlock->instrSet->target.cnt, 
				objThreadCntrolBlock->instrSet->target.acc);
	bool bRet = setInstruction(objThreadCntrolBlock, objThreadCntrolBlock->instrSet);
	while(bRet == false)
	{
		bRet = setInstruction(objThreadCntrolBlock, objThreadCntrolBlock->instrSet);
#ifdef WIN32
		Sleep(1);
#else
        usleep(1000);
#endif
	}

//    FST_INFO("setInstruction return true");
    return 1;   
}

/************************************************* 
	Function:		call_MoveC
	Description:	Execute MOVEC
	                FORMAT: MOVEC P[1] P[1]  250 CNT -1 +���Ӳ���
	Input:			iLineNum               - Line Number
	Input:			thread_control_block   - interpreter info
	Return: 		1        -    Success ;
*************************************************/ 
int call_MoveC(int iLineNum, struct thread_control_block* objThreadCntrolBlock)
{  
    MoveCommandDestination movCmdDst ;
	eval_value value;
	int boolValue;
	
	// char commandParam[1024];
    Instruction instr;
    // char * commandParamptr = commandParam;
	
	memset((char *)&instr, 0x00, sizeof(Instruction));
	instr.type = MOTION ;
	instr.target.type = MOTION_CIRCLE;
#ifdef USE_XPATH
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
		sprintf(instr.line, "%s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	else
		sprintf(instr.line, "OutRange with %d", iLineNum);
#else
	FST_INFO("call_MoveJ XPATH at %d", iLineNum);
	instr.line = iLineNum;
#endif
	// Save start position
	// memset(&movCmdDst ,0x00, sizeof(MoveCommandDestination));
	getMoveCommandDestination(movCmdDst);
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
	{
	    FST_INFO("call_MoveC Run XPATH: %d: %s", iLineNum, objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
		if(movCmdDst.type != MOTION_NONE)
		{
			if(objThreadCntrolBlock->start_mov_position.find(iLineNum)
				==objThreadCntrolBlock->start_mov_position.end())
			{
				try
				{
					objThreadCntrolBlock->start_mov_position.insert(
						map<int, MoveCommandDestination>::value_type(iLineNum, 
											movCmdDst));
				}
				catch (std::exception& e)
				{
				    std::cerr << "Exception catched : " << e.what() << std::endl;
				}
			}
			else
			{
	            FST_INFO("call_MoveC Run XPATH: %s exists ", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
			}
		}
		else
		{
			FST_INFO("call_MoveC XPATH without movCmdDst");
		}
	}
	else
	{
		FST_INFO("call_MoveC XPATH out of range at %d", iLineNum);
	}

    // result.size() == MOVJ_COMMAND_PARAM_MIN
    get_exp(objThreadCntrolBlock, &value, &boolValue);
	
//	if(value.hasType(TYPE_NONE) == TYPE_NONE)
//	{
//		find_eol(objThreadCntrolBlock);
//		return 0;
//	}
//	else 
	if((value.hasType(TYPE_INT) == TYPE_INT) || (value.hasType(TYPE_DOUBLE) == TYPE_DOUBLE))
	{
		instr.target.via.type      = COORDINATE_CARTESIAN ;
#ifndef WIN32
		instr.target.via.pose.pose.point_.x_    = value.getDoubleValue();
#else
		instr.target.via.pose.pose.position.x    = value.getDoubleValue();
#endif	
		get_token(objThreadCntrolBlock);

	    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.via.pose.pose.point_.y_    = value.getDoubleValue();
#else
		instr.target.via.pose.pose.position.y    = value.getDoubleValue();
#endif	
		get_token(objThreadCntrolBlock);
		
	    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.via.pose.pose.point_.z_    = value.getDoubleValue();
#else
		instr.target.via.pose.pose.position.z    = value.getDoubleValue();
#endif	
		get_token(objThreadCntrolBlock);
		
	    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.via.pose.pose.euler_.a_ = value.getDoubleValue();
#else
		instr.target.via.pose.pose.orientation.a = value.getDoubleValue();
#endif	
		get_token(objThreadCntrolBlock);
		
	    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.via.pose.pose.euler_.b_ = value.getDoubleValue();
#else
		instr.target.via.pose.pose.orientation.b = value.getDoubleValue();
#endif	
		get_token(objThreadCntrolBlock);
		
	    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.via.pose.pose.euler_.c_ = value.getDoubleValue();
#else
		instr.target.via.pose.pose.orientation.c = value.getDoubleValue();
#endif	
		instr.target.user_frame_id = instr.target.tool_frame_id = -1 ;
	}
	else if(value.hasType(TYPE_POSE) == TYPE_POSE)
	{
		instr.target.via.type      = COORDINATE_CARTESIAN ;
		instr.target.via.pose.pose = value.getPoseValue();
		
		instr.target.via.pose.posture = value.getPosture();
		instr.target.via.pose.turn    = value.getTurn();
	}
	else if(value.hasType(TYPE_JOINT) == TYPE_JOINT)
	{
		instr.target.via.type      = COORDINATE_JOINT ;
		instr.target.via.joint     = value.getJointValue();
		
		instr.target.via.pose.posture = value.getPosture();
		instr.target.via.pose.turn    = value.getTurn();
	//  instr.target.joint_target = value.getJointValue();
	//	find_eol(objThreadCntrolBlock);
    //	return 0;
	}
	
	get_token(objThreadCntrolBlock);

    get_exp(objThreadCntrolBlock, &value, &boolValue);
//	if(value.hasType(TYPE_NONE) == TYPE_NONE)
//	{
//		find_eol(objThreadCntrolBlock);
//    	return 0;
//	}
//	else 
	if((value.hasType(TYPE_INT) == TYPE_INT) || (value.hasType(TYPE_DOUBLE) == TYPE_DOUBLE))
	{
		instr.target.target.type      = COORDINATE_CARTESIAN ;
#ifndef WIN32
		instr.target.target.pose.pose.point_.x_    = value.getDoubleValue();
#else
		instr.target.target.pose.pose.position.x    = value.getDoubleValue();
#endif	
		get_token(objThreadCntrolBlock);

	    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.target.pose.pose.point_.y_    = value.getDoubleValue();
#else
		instr.target.target.pose.pose.position.y    = value.getDoubleValue();
#endif	
		get_token(objThreadCntrolBlock);
		
	    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.target.pose.pose.point_.z_    = value.getDoubleValue();
#else
		instr.target.target.pose.pose.position.z    = value.getDoubleValue();
#endif	
		get_token(objThreadCntrolBlock);
		
	    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.target.pose.pose.euler_.a_ = value.getDoubleValue();
#else
		instr.target.target.pose.pose.orientation.a = value.getDoubleValue();
#endif	
		get_token(objThreadCntrolBlock);
		
	    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.target.pose.pose.euler_.b_ = value.getDoubleValue();
#else
		instr.target.target.pose.pose.orientation.b = value.getDoubleValue();
#endif	
		get_token(objThreadCntrolBlock);
		
	    get_exp(objThreadCntrolBlock, &value, &boolValue);
#ifndef WIN32
		instr.target.target.pose.pose.euler_.c_ = value.getDoubleValue();
#else
		instr.target.target.pose.pose.orientation.c = value.getDoubleValue();
#endif	
	}
	else if(value.hasType(TYPE_POSE) == TYPE_POSE)
	{
		instr.target.target.type      = COORDINATE_CARTESIAN ;
		instr.target.target.pose.pose = value.getPoseValue();
		instr.target.target.pose.posture = value.getPosture();
		instr.target.target.pose.turn    = value.getTurn();
		
		instr.target.user_frame_id = value.getUFIndex();
		instr.target.tool_frame_id = value.getTFIndex();
		
#ifndef WIN32
	    FST_INFO("move to POSE:(%f, %f, %f, %f, %f, %f) in MovC", 
			instr.target.target.pose.pose.point_.x_, instr.target.target.pose.pose.point_.y_, 
			instr.target.target.pose.pose.point_.z_, instr.target.target.pose.pose.euler_.a_, 
			instr.target.target.pose.pose.euler_.b_, instr.target.target.pose.pose.euler_.c_);
#else
	    FST_INFO("move to POSE:(%f, %f, %f, %f, %f, %f) in MovC", 
			instr.target.target.pose.pose.position.x, instr.target.target.pose.pose.position.y, 
			instr.target.target.pose.pose.position.z, instr.target.target.pose.pose.orientation.a, 
			instr.target.target.pose.pose.orientation.b, instr.target.target.pose.pose.orientation.c);
#endif	
	}
	else if(value.hasType(TYPE_JOINT) == TYPE_JOINT)
	{
		instr.target.target.type      = COORDINATE_JOINT ;
		instr.target.target.joint     = value.getJointValue();
		
		instr.target.target.pose.posture = value.getPosture();
		instr.target.target.pose.turn    = value.getTurn();
		// instr.target.joint_target = value.getJointValue();
	    FST_INFO("value.getIntType() == TYPE_JOINT in MovC");
	//	find_eol(objThreadCntrolBlock);
    //	return 0;
	}
	
	// Use start point in revert mode  
	if(objThreadCntrolBlock->execute_direction == EXECUTE_BACKWARD)
	{
	     FST_INFO("Use start point in revert mode.");
		 // Wait for revert
	     // instr.target.circle_target.pose2 
	     //    = objThreadCntrolBlock->start_mov_position[objThreadCntrolBlock->vector_XPath[iLineNum]];
	}
	get_token(objThreadCntrolBlock);
	
    get_exp(objThreadCntrolBlock, &value, &boolValue);
    instr.target.vel                  = value.getDoubleValue();

	get_token(objThreadCntrolBlock);
	if(strcasecmp(objThreadCntrolBlock->token, SMOOTH_TYPE_CNT) == 0)
    {
		// 平滑参数的范围
    	get_exp(objThreadCntrolBlock, &value, &boolValue);
    	if(objThreadCntrolBlock->prog_mode == STEP_MODE)
    	{
			instr.target.smooth_type = SMOOTH_NONE;
        	instr.target.cnt = -1;
    	}
	    else
		{
			// 如果是FINE语句CNT应为-1
	        if(value.getDoubleValue() < 0) // == -1
	    	{
				instr.target.smooth_type = SMOOTH_NONE;
	        	instr.target.cnt = -1.0000;
	     		FST_INFO("instr.target.cnt = %f in the FINE.", instr.target.cnt);
	    	}
	        else 
			{
				// 速度平滑[0.0, 1.0]，
				instr.target.smooth_type = SMOOTH_VELOCITY;
				instr.target.cnt = value.getDoubleValue() / 100;
			}
	    }
    }
    else
    {
		if(strcasecmp(objThreadCntrolBlock->token, SMOOTH_TYPE_SD) == 0)
		{
			instr.target.smooth_type = SMOOTH_DISTANCE;
			// 距离平滑[0.0, +∞]，
			get_exp(objThreadCntrolBlock, &value, &boolValue);
			if(value.getDoubleValue() < 0) // == -1
			{
				instr.target.smooth_type = SMOOTH_NONE;
				instr.target.cnt = -1;
				FST_INFO("instr.target.cnt = %f in the FINE.", instr.target.cnt);
			}
			else 
			{
				instr.target.smooth_type = SMOOTH_DISTANCE;
        		instr.target.cnt = value.getDoubleValue() ; 
			}
		}
		else if(strcasecmp(objThreadCntrolBlock->token, SMOOTH_TYPE_SV) == 0)
		{
			instr.target.smooth_type = SMOOTH_VELOCITY;
			// 速度平滑[0.0, 1.0]，
    		get_exp(objThreadCntrolBlock, &value, &boolValue);
			if(value.getDoubleValue() < 0) // == -1
			{
				instr.target.smooth_type = SMOOTH_NONE;
				instr.target.cnt = -1;
				FST_INFO("instr.target.cnt = %f in the FINE.", instr.target.cnt);
			}
			else 
			{
				instr.target.smooth_type = SMOOTH_VELOCITY;
				instr.target.cnt = value.getDoubleValue() / 100;
			}
		}
		else 
		{
			instr.target.smooth_type = SMOOTH_NONE;
			instr.target.cnt = -1.0000;
		}
    }
	// instr.target.acc = -1 ;
	// Set to instrSet
	memcpy(objThreadCntrolBlock->instrSet, &instr, sizeof(Instruction));
	objThreadCntrolBlock->instrSet->target.acc = 1.0 ;
	
	get_token(objThreadCntrolBlock);
	// result.size() > MOVJ_COMMAND_PARAM_MIN
	if(objThreadCntrolBlock->token_type == DELIMITER)
	{
		char * instrSetPtr = 
			(char *)objThreadCntrolBlock->instrSet
			+ sizeof(Instruction) - sizeof(char) ;
		objThreadCntrolBlock->instrSet->is_additional = true ;
		if(*(objThreadCntrolBlock->token) == ';')
		{
			objThreadCntrolBlock->instrSet->add_num    =  
				getAditionalInfomation(objThreadCntrolBlock, instrSetPtr);
		}
		else
		{
			AdditionalInfomation additionalInfomation ;
			additionalInfomation.type = ACC ;
			additionalInfomation.acc_speed = 100 ;
			memcpy(instrSetPtr, &additionalInfomation, sizeof(AdditionalInfomation));
			
			objThreadCntrolBlock->instrSet->add_num    = 1 ;
		}
	}
	
	if(objThreadCntrolBlock->instrSet->target.user_frame_offset.valid)
	{
		if(objThreadCntrolBlock->instrSet->target.user_frame_offset.coord_type == COORDINATE_JOINT)
		{
#ifndef WIN32
	    FST_INFO("Backward move to JOINT:(%f, %f, %f, %f, %f, %f) in MovJ with %d", 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j1_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j2_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j3_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j4_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j5_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j6_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_frame_id);
#else
	    FST_INFO("Backward move to JOINT:(%f, %f, %f, %f, %f, %f) in MovJ with %d", 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j1, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j2, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j3, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j4, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j5, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j6, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_frame_id);
#endif	
		}
		else if(objThreadCntrolBlock->instrSet->target.user_frame_offset.coord_type == COORDINATE_CARTESIAN)
		{
#ifndef WIN32
	    FST_INFO("Backward move to CART:(%f, %f, %f, %f, %f, %f) in MovJ with %d", 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.point_.x_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.point_.y_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.point_.z_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.euler_.a_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.euler_.b_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.euler_.c_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_frame_id);
#else
	    FST_INFO("Backward move to CART:(%f, %f, %f, %f, %f, %f) in MovJ with %d", 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.position.x, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.position.y, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.position.z, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.orientation.a, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.orientation.b, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.orientation.c, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_frame_id);
#endif	
		}
	}
	if(objThreadCntrolBlock->instrSet->target.tool_frame_offset.valid)
	{
		if(objThreadCntrolBlock->instrSet->target.tool_frame_offset.coord_type == COORDINATE_JOINT)
		{
#ifndef WIN32
	    FST_INFO("Backward move to JOINT:(%f, %f, %f, %f, %f, %f) in MovJ with %d", 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j1_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j2_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j3_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j4_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j5_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j6_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_frame_id);
#else
	    FST_INFO("Backward move to JOINT:(%f, %f, %f, %f, %f, %f) in MovJ with %d", 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j1, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j2, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j3, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j4, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j5, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j6, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_frame_id);
#endif	
		}
		else if(objThreadCntrolBlock->instrSet->target.tool_frame_offset.coord_type == COORDINATE_CARTESIAN)
		{
#ifndef WIN32
	    FST_INFO("Backward move to CART:(%f, %f, %f, %f, %f, %f) in MovJ with %d", 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.point_.x_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.point_.y_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.point_.z_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.euler_.a_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.euler_.b_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.euler_.c_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_frame_id);
#else
	    FST_INFO("Backward move to CART:(%f, %f, %f, %f, %f, %f) in MovJ with %d", 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.position.x, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.position.y, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.position.z, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.orientation.a, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.orientation.b, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.orientation.c, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_frame_id);
#endif	
		}
	}
	
	if(objThreadCntrolBlock->instrSet->target.user_frame_offset.valid)
	{
		if(objThreadCntrolBlock->instrSet->target.user_frame_offset.coord_type == COORDINATE_JOINT)
		{
#ifndef WIN32
	    FST_INFO("user_frame_offset to JOINT:(%f, %f, %f, %f, %f, %f) in MovC with %d", 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j1_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j2_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j3_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j4_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j5_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j6_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_frame_id);
#else
	    FST_INFO("user_frame_offset to JOINT:(%f, %f, %f, %f, %f, %f) in MovC with %d", 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j1, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j2, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j3, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j4, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j5, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_joint.j6, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_frame_id);
#endif	
		}
		else if(objThreadCntrolBlock->instrSet->target.user_frame_offset.coord_type == COORDINATE_JOINT)
		{
#ifndef WIN32
	    FST_INFO("user_frame_offset to JOINT:(%f, %f, %f, %f, %f, %f) in MovC with %d", 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.point_.x_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.point_.y_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.point_.z_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.euler_.a_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.euler_.b_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.euler_.c_, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_frame_id);
#else
	    FST_INFO("user_frame_offset to JOINT:(%f, %f, %f, %f, %f, %f) in MovC with %d", 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.position.x, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.position.y, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.position.z, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.orientation.a, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.orientation.b, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_pose.orientation.c, 
			objThreadCntrolBlock->instrSet->target.user_frame_offset.offset_frame_id);
#endif	
		}
	}
	if(objThreadCntrolBlock->instrSet->target.tool_frame_offset.valid)
	{
		if(objThreadCntrolBlock->instrSet->target.tool_frame_offset.coord_type == COORDINATE_JOINT)
		{
#ifndef WIN32
	    FST_INFO("tool_frame_offset to JOINT:(%f, %f, %f, %f, %f, %f) in MovC with %d", 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j1_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j2_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j3_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j4_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j5_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j6_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_frame_id);
#else
	    FST_INFO("tool_frame_offset to JOINT:(%f, %f, %f, %f, %f, %f) in MovC with %d", 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j1, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j2, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j3, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j4, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j5, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_joint.j6, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_frame_id);
#endif	
		}
		else if(objThreadCntrolBlock->instrSet->target.tool_frame_offset.coord_type == COORDINATE_JOINT)
		{
#ifndef WIN32
	    FST_INFO("tool_frame_offset JOINT:(%f, %f, %f, %f, %f, %f) in MovC with %d", 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.point_.x_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.point_.y_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.point_.z_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.euler_.a_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.euler_.b_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.euler_.c_, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_frame_id);
#else
	    FST_INFO("tool_frame_offset JOINT:(%f, %f, %f, %f, %f, %f) in MovC with %d", 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.position.x, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.position.y, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.position.z, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.orientation.a, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.orientation.b, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_pose.orientation.c, 
			objThreadCntrolBlock->instrSet->target.tool_frame_offset.offset_frame_id);
#endif	
		}
	}
	
// 	#ifdef USE_XPATH
// 		FST_INFO("setInstruction MOTION_CURVE at %s", instr.line);
// 	#else
// 		FST_INFO("setInstruction MOTION_CURVE at %d", instr.line);
// 	#endif
	FST_INFO("call_MoveC instr.target.cnt = %f setInstruction with %f.", 
				objThreadCntrolBlock->instrSet->target.cnt, 
				objThreadCntrolBlock->instrSet->target.acc);
	bool bRet = setInstruction(objThreadCntrolBlock, objThreadCntrolBlock->instrSet);
	while(bRet == false)
	{
		bRet = setInstruction(objThreadCntrolBlock, objThreadCntrolBlock->instrSet);
#ifdef WIN32
		Sleep(1);
#else
        usleep(1000);
#endif
	}
    return 1;     
}


/************************************************* 
	Function:		call_MoveXPos
	Description:	Execute MOVEL
	                FORMAT: MOVEL P[1], 250 CNT -1  +���Ӳ���
	Input:			iLineNum               - Line Number
	Input:			thread_control_block   - interpreter info
	Return: 		1        -    Success ;
*************************************************/ 
int call_MoveXPos(int iLineNum, struct thread_control_block* objThreadCntrolBlock)
{  
    MoveCommandDestination movCmdDst ;
	eval_value value;
	int boolValue;
	
	// char commandParam[1024];
    Instruction instr;
    // char * commandParamptr = commandParam;
	
	memset((char *)&instr, 0x00, sizeof(Instruction));
	instr.type = MOTION ;
	instr.target.type = MOTION_XPOS;
#ifdef USE_XPATH
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
		sprintf(instr.line, "%s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	else
		sprintf(instr.line, "OutRange with %d", iLineNum);
#else
	FST_INFO("call_MoveJ XPATH at %d", iLineNum);
	instr.line = iLineNum;
#endif
	// Save start position
	// memset(&movCmdDst ,0x00, sizeof(MoveCommandDestination));
	getMoveCommandDestination(movCmdDst);
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
	{
	    FST_INFO("call_MoveXPos Run XPATH: %d: %s", iLineNum, objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	    // FST_INFO("call_MoveL Run movCmdDst: %08X with(%08X, %08X, %08X, %08X)", 
		//  	movCmdDst.type, MOTION_NONE, MOTION_JOINT, MOTION_LINE, MOTION_CIRCLE);
		if(movCmdDst.type != MOTION_NONE)
		{
			if(objThreadCntrolBlock->start_mov_position.find(iLineNum)
				==objThreadCntrolBlock->start_mov_position.end())
			{
			//    FST_INFO("move from POSE and insert:(%f, %f, %f, %f, %f, %f) in MovL", 
			//		movCmdDst.pose_target.position.x,    movCmdDst.pose_target.position.y, 
			//		movCmdDst.pose_target.position.z,    movCmdDst.pose_target.orientation.a, 
			//		movCmdDst.pose_target.orientation.b, movCmdDst.pose_target.orientation.c);
				try
				{
					objThreadCntrolBlock->start_mov_position.insert(
						map<int, MoveCommandDestination>::value_type(iLineNum, 
											movCmdDst));
				}
				catch (std::exception& e)
				{
				    std::cerr << "Exception catched : " << e.what() << std::endl;
				}
			}
			else
			{
	            FST_INFO("call_MoveXPos Run XPATH: %s exists ", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
			}
		}
		else
		{
			; // FST_INFO("call_MoveL XPATH without movCmdDst");
		}
	}
	else
	{
		FST_INFO("call_MoveXPos XPATH out of range at %d", iLineNum);
	}
	// FST_INFO("call_MoveL Run XPATH: %s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	memset(&instr.target.prPos, 0x00, PR_POS_LEN * sizeof(int));
	get_token(objThreadCntrolBlock);
	int prPosIdx = 0 ;
	while(strcasecmp(objThreadCntrolBlock->token, "pr") == 0)
	{
		get_token(objThreadCntrolBlock);
		if(objThreadCntrolBlock->token[0] == '['){
			get_exp(objThreadCntrolBlock, &value, &boolValue);
			instr.target.prPos[prPosIdx] = (int)value.getDoubleValue();
			prPosIdx++;
			get_token(objThreadCntrolBlock);
			if(objThreadCntrolBlock->token[0] != ']'){
				serror(objThreadCntrolBlock, 0);
				break;
			}
			// Jump ","
			get_token(objThreadCntrolBlock);
			// Get next PR
			get_token(objThreadCntrolBlock);
		}
		else{
			serror(objThreadCntrolBlock, 0);
			break;
		}
	}
	
	putback(objThreadCntrolBlock);
	// Not set PR and use default 21 - 30
	if(prPosIdx == 0)
	{
		for (prPosIdx = 0; prPosIdx < 10; prPosIdx++)
		{
			instr.target.prPos[prPosIdx] = 21 + prPosIdx;
		}
	}
	instr.target.user_frame_id = instr.target.tool_frame_id = -1 ;
	// We had jump ","
	// get_token(objThreadCntrolBlock);
    get_exp(objThreadCntrolBlock, &value, &boolValue);
    instr.target.vel                  = value.getDoubleValue() / 100;
	
	get_token(objThreadCntrolBlock);
	if(strcasecmp(objThreadCntrolBlock->token, SMOOTH_TYPE_CNT) == 0)
    {
		// 平滑参数的范围
		get_exp(objThreadCntrolBlock, &value, &boolValue);
		FST_INFO("instr.target.cnt = %f setInstruction.", value.getDoubleValue());
		if(objThreadCntrolBlock->prog_mode == STEP_MODE)
		{
			instr.target.smooth_type = SMOOTH_NONE;
			instr.target.cnt = -1;
		}
		else
		{
			// 如果是FINE语句CNT应为-1
			if(value.getDoubleValue() < 0) // == -1
			{
				instr.target.smooth_type = SMOOTH_NONE;
				instr.target.cnt = -1.0000;
				FST_INFO("instr.target.cnt = %f in the FINE.", instr.target.cnt);
			}
			else 
			{
				// 速度平滑[0.0, 1.0]，
				instr.target.smooth_type = SMOOTH_VELOCITY;
				instr.target.cnt = value.getDoubleValue() / 100;
			}
		}
    }
    else
    {
		if(strcasecmp(objThreadCntrolBlock->token, SMOOTH_TYPE_SD) == 0)
		{
			instr.target.smooth_type = SMOOTH_DISTANCE;
			// 距离平滑[0.0, +∞]，
			get_exp(objThreadCntrolBlock, &value, &boolValue);
			if(value.getDoubleValue() < 0) // == -1
			{
				instr.target.smooth_type = SMOOTH_NONE;
				instr.target.cnt = -1;
				FST_INFO("instr.target.cnt = %f in the FINE.", instr.target.cnt);
			}
			else 
			{
				instr.target.smooth_type = SMOOTH_DISTANCE;
        		instr.target.cnt = value.getDoubleValue() ; 
			}
		}
		else if(strcasecmp(objThreadCntrolBlock->token, SMOOTH_TYPE_SV) == 0)
		{
			instr.target.smooth_type = SMOOTH_VELOCITY;
			// 速度平滑[0.0, 1.0]，
    		get_exp(objThreadCntrolBlock, &value, &boolValue);
			if(value.getDoubleValue() < 0) // == -1
			{
				instr.target.smooth_type = SMOOTH_NONE;
				instr.target.cnt = -1;
				FST_INFO("instr.target.cnt = %f in the FINE.", instr.target.cnt);
			}
			else 
			{
				instr.target.smooth_type = SMOOTH_VELOCITY;
				instr.target.cnt = value.getDoubleValue() / 100;
			}
		}
		else 
		{
			instr.target.smooth_type = SMOOTH_NONE;
			instr.target.cnt = -1.0000;
		}
    }
	// instr.target.acc = -1 ;
	// Set to instrSet
	memcpy(objThreadCntrolBlock->instrSet, &instr, sizeof(Instruction));
	objThreadCntrolBlock->instrSet->target.acc = 1.0 ;
	
	get_token(objThreadCntrolBlock);
	// result.size() > MOVJ_COMMAND_PARAM_MIN
	if(objThreadCntrolBlock->token_type == DELIMITER)
	{
		char * instrSetPtr = 
			(char *)objThreadCntrolBlock->instrSet
			+ sizeof(Instruction) - sizeof(char) ;
		objThreadCntrolBlock->instrSet->is_additional = true ;
		if(*(objThreadCntrolBlock->token) == ';')
		{
			objThreadCntrolBlock->instrSet->add_num    =  
				getAditionalInfomation(objThreadCntrolBlock, instrSetPtr);
		}
		else
		{
			AdditionalInfomation additionalInfomation ;
			additionalInfomation.type = ACC ;
			additionalInfomation.acc_speed = 100 ;
			memcpy(instrSetPtr, &additionalInfomation, sizeof(AdditionalInfomation));
			
			objThreadCntrolBlock->instrSet->add_num    = 1 ;
		}
	}
	FST_INFO("prPos = {%d, %d, %d, %d }.", 
		objThreadCntrolBlock->instrSet->target.prPos[0], 
		objThreadCntrolBlock->instrSet->target.prPos[1], 
		objThreadCntrolBlock->instrSet->target.prPos[2], 
		objThreadCntrolBlock->instrSet->target.prPos[3]);
	
// 	#ifdef USE_XPATH
// 		FST_INFO("setInstruction MOTION_CURVE at %s", instr.line);
// 	#else
// 		FST_INFO("setInstruction MOTION_CURVE at %d", instr.line);
// 	#endif
	FST_INFO("call_MoveXPos instr.target.cnt = %f setInstruction with %f.", 
				objThreadCntrolBlock->instrSet->target.cnt, 
				objThreadCntrolBlock->instrSet->target.acc);
	bool bRet = setInstruction(objThreadCntrolBlock, objThreadCntrolBlock->instrSet);
	while(bRet == false)
	{
		bRet = setInstruction(objThreadCntrolBlock, objThreadCntrolBlock->instrSet);
#ifdef WIN32
		Sleep(1);
#else
        usleep(1000);
#endif
	}
    return 1;     
}

/************************************************* 
	Function:		call_UserAlarm
	Description:	Execute UserAlarm
	                FORMAT: UserAlarm 1
	Input:			iLineNum               - Line Number
	Input:			thread_control_block   - interpreter info
	Return: 		1        -    Success ;
*************************************************/ 
int call_UserAlarm(int iLineNum, struct thread_control_block* objThreadCntrolBlock) 
{  
	eval_value value;
	int boolValue;
    int alarmNumber;
	
    get_exp(objThreadCntrolBlock, &value, &boolValue);
	alarmNumber = (int)value.getDoubleValue() ;
	setMessage(alarmNumber);
    find_eol(objThreadCntrolBlock);
    return 1;
}

static int get_char_token(char * src, char * dst)
{
	char * tmp = src ;
	if(isalpha(*src)) { /* var or command */
		while(!isdelim(*(src))) 
			*dst++=*(src)++;
	}
	return src - tmp ;
}

static int get_num_token(char * src, char * dst)
{
	char * tmp = src ;
	if(isdigit(*src)) { /* var or command */
		while(!isdelim(*(src))) 
			*dst++=*(src)++;
	}
	return src - tmp ;
}

int execute_Timer(struct thread_control_block* objThreadCntrolBlock, char *vname, eval_value& value)
{
	// bool bRet = false ;
	char timer_name[16] ;
	char timer_idx[16] ;
	// char io_key_buffer[16] ;

	int  iTimerIdx = 0 ;
	char * namePtr = vname ;
	char *temp = NULL ;
	
	memset(timer_name, 0x00, 16);
	memset(timer_idx, 0x00, 16);
	
	temp = timer_name ;
	get_char_token(namePtr, temp);
	
	namePtr += strlen(timer_name) ;
	if(namePtr[0] != '['){
		return -1 ;
	}
	namePtr++ ;
	
	memset(timer_idx, 0x00, 16);
	temp = timer_idx ;
	get_num_token(namePtr, temp);
	iTimerIdx = atoi(timer_idx);
	// namePtr += strlen(reg_idx) ;
	
	namePtr += strlen(timer_idx) ;
	if(namePtr[0] != ']'){
		return -1 ;
	}
	namePtr++ ;

	if(value.getDoubleValue() == TIMER_START_VALUE)
	{
		FST_INFO("%d: call_Timer  start", __LINE__);
		g_structStopWatch[iTimerIdx].start_time = time(0);
		// value.setDoubleValue(0); // 0.0; 
		erase_var(objThreadCntrolBlock, vname);
		assign_global_var(objThreadCntrolBlock, vname, value); // 0.0);
	}
	else if(value.getDoubleValue() == TIMER_RESET_VALUE)
	{
		FST_INFO("%d: call_Timer restart", __LINE__);
		g_structStopWatch[iTimerIdx].start_time = time(0);
		// value.setDoubleValue(0); // 0.0; 
		erase_var(objThreadCntrolBlock, vname);
		assign_global_var(objThreadCntrolBlock, vname, value); // 0.0);
	}
	else if(value.getDoubleValue() == TIMER_STOP_VALUE)
	{
		FST_INFO("%d: call_Timer  stop", __LINE__);
		g_structStopWatch[iTimerIdx].diff_time = time(0) - 
			g_structStopWatch[iTimerIdx].start_time ;
		
		value.setDoubleValue(g_structStopWatch[iTimerIdx].diff_time)  ;
	//	assign_var(objThreadCntrolBlock, vname, value);
	
		assign_global_var(objThreadCntrolBlock, vname, value);
		FST_INFO("Time elapse : %d .", g_structStopWatch[iTimerIdx].diff_time);
	}
	return 1 ;
}

/************************************************* 
	Function:		call_Timer
	Description:	Execute Timer
	                FORMAT: TIMER[1]  = START
	                    OR: TIMER 1 start
	Input:			iLineNum               - Line Number
	Input:			thread_control_block   - interpreter info
	Return: 		1        -    Success ;
*************************************************/ 
int call_Timer(int iLineNum, struct thread_control_block* objThreadCntrolBlock) 
{  
	eval_value value;
	int boolValue;
    int timerNumber;
    char var[80];
	
	// TIMER[1]  = START
	if(objThreadCntrolBlock->prog[0] == '['){
		putback(objThreadCntrolBlock);
		assignment(objThreadCntrolBlock);
	}
	// TIMER 1 start 
    else {
		get_exp(objThreadCntrolBlock, &value, &boolValue);
		timerNumber = (int)value.getDoubleValue() ;
        FST_INFO("%d: call_Timer  enter %d ", __LINE__, timerNumber);
		if(timerNumber >= MAX_STOPWATCH_NUM)
			return 0;
		
		sprintf(var, "%s[%d]", FORSIGHT_TIMER, timerNumber);
		get_token(objThreadCntrolBlock);
        FST_INFO("%d: call_Timer  enter %s ", __LINE__, objThreadCntrolBlock->token);
		if(strcasecmp(objThreadCntrolBlock->token, "start") == 0)
		{
			value.setDoubleValue(TIMER_START_VALUE); 
			execute_Timer(objThreadCntrolBlock, var, value); 
		}
		else if(strcasecmp(objThreadCntrolBlock->token, "reset") == 0)
		{
			value.setDoubleValue(TIMER_RESET_VALUE);
			execute_Timer(objThreadCntrolBlock, var, value);
		}
		else if(strcasecmp(objThreadCntrolBlock->token, "stop") == 0)
		{
			value.setDoubleValue(TIMER_STOP_VALUE); 
			execute_Timer(objThreadCntrolBlock, var, value);
			FST_INFO("Time elapse : %d .", g_structStopWatch[timerNumber].diff_time);
		}
		else 
		{
			find_eol(objThreadCntrolBlock);
			return 0;
		}
		find_eol(objThreadCntrolBlock);
	}
    return 1;   
}

/************************************************* 
	Function:		call_Wait
	Description:	Execute Wait
	                FORMAT: WAIT 3 ;   WAIT R[1]
			                WAIT DI[3]=ON
	                        WAIT DI[2]=ON, TimeOut=60 Skip
	Input:			iLineNum               - Line Number
	Input:			thread_control_block   - interpreter info
	Return: 		1        -    Success ;
*************************************************/ 
int call_Wait(int iLineNum, struct thread_control_block* objThreadCntrolBlock) 
{  
	eval_value value;
	int boolValue;
	time_t timeStart, now ;
	int timeWaitSeconds ;
	int timeWaitMicroSeconds ;   // us: 1s = 1,000,000 us
    int cond, outTime;
    struct select_and_cycle_stack wait_stack;
    
    Instruction instr;
	memset((char *)&instr, 0x00, sizeof(Instruction));
	
	instr.target.type = MOTION_JOINT;
#ifdef USE_XPATH
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
		sprintf(instr.line, "%s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	else
		sprintf(instr.line, "OutRange with %d", iLineNum);
#else
	FST_INFO("call_MoveJ XPATH at %d", iLineNum);
	instr.line = iLineNum;
	if(iLineNum < (int)objThreadCntrolBlock->vector_XPath.size())
	{
		FST_INFO("call_Wait XPATH: %s", objThreadCntrolBlock->vector_XPath[iLineNum].c_str());
	}
	else
		FST_INFO("OutRange with:  %d", iLineNum);
#endif
	
	get_token(objThreadCntrolBlock);
	if(strcasecmp(objThreadCntrolBlock->token, "cond") != 0)
    {
		putback(objThreadCntrolBlock);
    	get_exp(objThreadCntrolBlock, &value, &boolValue);
		// Need not get_token again . let call_interpreter filter it .
		timeWaitSeconds = (int)value.getDoubleValue() ;
	    FST_INFO("call_Wait timeWaitSeconds = %d at %lld.", timeWaitSeconds, time(NULL));
		now = timeStart  = time(0);
		while(now - timeStart < timeWaitSeconds)
		{
#ifdef WIN32
			Sleep(1000);
#else
			usleep(200000);
#endif
			now = time(0);
			if(objThreadCntrolBlock->is_abort == true ||
			   objThreadCntrolBlock->is_paused == true)
			{
				break ;
			}
		}
	    FST_INFO("call_Wait timeWaitSeconds = %d at %lld.", timeWaitSeconds, time(NULL));
		timeWaitMicroSeconds = (int)(value.getDoubleValue() * 1000000 - timeWaitSeconds * 1000000);
#ifdef WIN32
		Sleep(timeWaitMicroSeconds/1000);
#else
		usleep(timeWaitMicroSeconds);
#endif
		find_eol(objThreadCntrolBlock);
    }
    else  // Deal wait with condition
    {
        // wait_stack.itokentype = WAIT ;
        wait_stack.while_loc = objThreadCntrolBlock->prog;
		cond = calc_conditions(objThreadCntrolBlock);
		
		get_token(objThreadCntrolBlock);
		// Without timeout dealing which includes Timeout and skip/warning/call XXX
		if(objThreadCntrolBlock->token[0] == '\r')
		{
			putback(objThreadCntrolBlock);
			outTime = forgesight_get_wait_time_out_config();
			now = timeStart  = time(0);
			while(!cond)
			{
#ifdef WIN32
				Sleep(100);
				break;
#else
                // IO_Manager use 10ms, So I use 5ms
			    // usleep(1000);
			    usleep(5000);
#endif
			    now = time(0);

				objThreadCntrolBlock->prog = wait_stack.while_loc;
				cond = calc_conditions(objThreadCntrolBlock);
				if(now - timeStart > outTime)
				{
					FST_WARN("wait condition default time(%d) out forced quit current line", outTime);
					setWarning(WARN_INTERPRETER_DEFAULT_TIMEOUT);
            		setPrgmState(objThreadCntrolBlock, INTERPRETER_PAUSED);
					break ;
				}
					
				if(objThreadCntrolBlock->is_abort == true ||
				   objThreadCntrolBlock->is_paused == true)//add to fix can not JUMP in wait
				{
					break ;
				}
			}
		}
		// with dealing method when timeout
		else if(objThreadCntrolBlock->token_type == NUMBER) 
		{
			putback(objThreadCntrolBlock);
			get_exp(objThreadCntrolBlock, &value, &boolValue);
			outTime = (int)value.getDoubleValue() ;
			if(outTime <= 0)
			{
				outTime = forgesight_get_wait_time_out_config();
			}
			wait_stack.loc = objThreadCntrolBlock->prog;
			now = timeStart  = time(0);
			while(cond == 0)
			{
#ifdef WIN32
				Sleep(100);
				break;
#else
                // IO_Manager use 10ms, So I use 5ms
			    // usleep(1000);
			    usleep(5000);
#endif
			    now = time(0);
				objThreadCntrolBlock->prog = wait_stack.while_loc;
				cond = calc_conditions(objThreadCntrolBlock);
				if(now - timeStart > outTime)
				{
	    			FST_INFO("cond Timeout.");
					break ;
				}
				if(objThreadCntrolBlock->is_abort == true)
				{
					break ;
				}
			}
			
			objThreadCntrolBlock->prog = wait_stack.loc;
	    	FST_INFO("cond = %d .", cond);
			// Execute Timeout operation
			if(cond == 0)
			{
				get_token(objThreadCntrolBlock);
				if(strcasecmp(objThreadCntrolBlock->token, "skip") == 0)
				{
					; // Do nothing  
				}
				else if(strcasecmp(objThreadCntrolBlock->token, "warning") == 0)
				{
					// exec_call(objThreadCntrolBlock)
	    			FST_INFO("setWarning(INFO_INTERPRETER_WAIT_TIMEOUT).");
					setWarning(INFO_INTERPRETER_WAIT_TIMEOUT) ; 
				}
				else if(strcasecmp(objThreadCntrolBlock->token, "call") == 0)
				{
					// exec_call(objThreadCntrolBlock) ;
					int iRet = exec_call(objThreadCntrolBlock);
					if(iRet == END_COMMND_RET)
						return END_COMMND_RET;
				}
				else 
				{
					putback(objThreadCntrolBlock);
				}
			}
		//	else
		//		find_eol(objThreadCntrolBlock);
			
		}
		else 
		{
			putback(objThreadCntrolBlock);
		}
		find_eol(objThreadCntrolBlock);
    }
    return 1;   
}

/************************************************* 
	Function:		call_Pause
	Description:	Execute Pause
	                FORMAT: PAUSE
	Input:			iLineNum               - Line Number
	Input:			thread_control_block   - interpreter info
	Return: 		1        -    Success ;
*************************************************/ 
int call_Pause(int iLineNum, struct thread_control_block* objThreadCntrolBlock) 
{  
	// InterpreterState interpreterState  = INTERPRETER_PAUSED ;
	setPrgmState(objThreadCntrolBlock, INTERPRETER_PAUSED);
/*
	while(interpreterState == PAUSED_R)
	{
#ifdef WIN32
		Sleep(100);
		interpreterState = EXECUTE_R ;
#else
		sleep(1);
		interpreterState = getPrgmState();
#endif
	}
*/
//    objThreadCntrolBlock->is_paused = true;
    FST_INFO("call_Pause: Enter waitInterpreterStateleftPaused %d ", iLineNum);
	waitInterpreterStateleftPaused(objThreadCntrolBlock);
    FST_INFO("call_Pause: Left  waitInterpreterStateleftPaused %d ", iLineNum);
//    objThreadCntrolBlock->is_paused = false;
    find_eol(objThreadCntrolBlock);
    return 1;   
}

/************************************************* 
	Function:		call_Abort
	Description:	Execute Abort
	                FORMAT: ABORT
	Input:			iLineNum               - Line Number
	Input:			thread_control_block   - interpreter info
	Return: 		1        -    Success ;
*************************************************/ 
int call_Abort(int iLineNum, struct thread_control_block* objThreadCntrolBlock) 
{  
	setPrgmState(objThreadCntrolBlock, INTERPRETER_IDLE);
  // clear line path and ProgramName
	resetProgramNameAndLineNum(objThreadCntrolBlock);
    objThreadCntrolBlock->is_abort = true;
#ifdef WIN32
	Sleep(100);
    return 0; 
#else
	sleep(1);
    return END_COMMND_RET;   
#endif
}

/************************************************* 
	Function:		call_BLDC_CTRL
	Description:	Execute Abort
	                FORMAT: ABORT
	Input:			iLineNum               - Line Number
	Input:			thread_control_block   - interpreter info
	Return: 		1        -    Success ;
*************************************************/ 
int call_BLDC_CTRL(int iLineNum, struct thread_control_block* objThreadCntrolBlock) 
{  
	// bool bRet = false ;
	eval_value value;
	int boolValue;
	// uint8_t iDir = 0, iVel = 0 ;
#if 1
	get_exp(objThreadCntrolBlock, &value, &boolValue);
	value.getDoubleValue() ;
#else
	get_token(objThreadCntrolBlock);
	if(strcasecmp(objThreadCntrolBlock->token, "dir") == 0)
	{
		/* get the equals sign */
		get_token(objThreadCntrolBlock);
		if(*(objThreadCntrolBlock->token)!=EQ) {
			serror(objThreadCntrolBlock, 3);
			return false;
		}
		get_exp(objThreadCntrolBlock, &value, &boolValue);
		iDir = (int)value.getDoubleValue() ;
	}

	get_token(objThreadCntrolBlock);
	if(strcasecmp(objThreadCntrolBlock->token, "vel") == 0)
	{
		/* get the equals sign */
		get_token(objThreadCntrolBlock);
		if(*(objThreadCntrolBlock->token)!=EQ) {
			serror(objThreadCntrolBlock, 3);
			return false;
		}
		get_exp(objThreadCntrolBlock, &value, &boolValue);
		iVel = (int)value.getDoubleValue() ;
	}
	iVel = iDir * 128 + iVel ;  
#endif

#ifdef WIN32
    return 0; 
#else
//	if(g_objRegManagerInterface)
//	{
//  //		bRet = g_objRegManagerInterface->setBLDC(iVel);
//		if(bRet)
//		{
//			return bRet ;
//		}
//	}
//	else
//	{
//		FST_ERROR("g_objRegManagerInterface is NULL");
//	}
    return 1;   
#endif
}

/************************************************* 
	Function:		getXPathLineNum
	Description:	Get line count
	Input:			file_name               - file name
	Return: 		iLineCount              - line count
*************************************************/ 
int getXPathLineNum(char * file_name)
{
	int iLineCount = 0 ;
	char contentLine[FILE_PATH_LEN];
	FILE *xpath_file ;
	if((xpath_file = fopen(file_name, "r"))==NULL){
		perror("open file_name file failed\n");  
		return -1;
	}
	
	memset(contentLine,    0x00, FILE_PATH_LEN);
	while(fgets(contentLine, sizeof(contentLine), xpath_file)!=NULL)  
	{  
		iLineCount++ ;
	}
	fclose(xpath_file);
	return iLineCount ;
}

/************************************************* 
	Function:		mergeImportXPathToProjectXPath
	Description:	merge XPath content of import file into the XPath content of major file
	Input:			thread_control_block   - interpreter info
	Input:			fname                  - import file name
	Return: 		NULL
*************************************************/ 
void mergeImportXPathToProjectXPath(
		struct thread_control_block* objThreadCntrolBlock, char * fname)
{
	int iImportLineNum = 0 , iMainLineCount = 0 ;
	char contentImportLine[FILE_PATH_LEN];
	char * contentImportSepPtr; 
	char contentImportLineNum[LAB_LEN];
	char contentImportXPath[FILE_PATH_LEN];
	
	char xpath_import_file_name[FILE_PATH_LEN];
	char xpath_main_file_name[FILE_PATH_LEN];

	FILE *xpath_import_file ;
	FILE *xpath_main_file ;
	
	memset(xpath_import_file_name, 0x00, FILE_PATH_LEN);
	memset(xpath_main_file_name, 0x00, FILE_PATH_LEN);
#ifdef WIN32
	sprintf(xpath_import_file_name, "%s_xpath.txt", fname);
	sprintf(xpath_main_file_name, "%s_xpath.txt", objThreadCntrolBlock->project_name);
#else
	sprintf(xpath_import_file_name, "%s\/programs\/%s_xpath.txt", 
			forgesight_get_programs_path(), fname);
	sprintf(xpath_main_file_name, "%s\/programs\/%s_xpath.txt", 
			forgesight_get_programs_path(), objThreadCntrolBlock->project_name);
#endif
	iMainLineCount = getXPathLineNum(xpath_main_file_name);
	
	if((xpath_import_file = fopen(xpath_import_file_name, "r"))==NULL){
		perror("open xpath_import_file_name file failed\n");  
		return ;
	}
	if((xpath_main_file = fopen(xpath_main_file_name, "a"))==NULL){
		perror("open xpath_import_file_name file failed\n");  
		return ;
	}
	
	memset(contentImportLine,    0x00, FILE_PATH_LEN);
	while(fgets(contentImportLine, 
		sizeof(contentImportLine), xpath_import_file)!=NULL)  
	{  
		memset(contentImportLineNum, 0x00, LAB_LEN);
		memset(contentImportXPath,   0x00, FILE_PATH_LEN);
		contentImportSepPtr = strchr(contentImportLine, ':');
		if(contentImportSepPtr)
		{
			memcpy(contentImportLineNum, contentImportLine, 
				contentImportSepPtr - contentImportLine);
			strcpy(contentImportXPath,   contentImportSepPtr + 1);
			iImportLineNum = atoi(contentImportLineNum);
		//	FST_INFO("%04d:%s", iImportLineNum + iMainLineCount, 
		//		contentImportSepPtr + 1);
			fprintf(xpath_main_file, "%04d:%s", iImportLineNum + iMainLineCount, 
				contentImportSepPtr + 1);
		}
	}
	fclose(xpath_import_file);
	fclose(xpath_main_file);
}

/************************************************* 
	Function:		generateXPathVector
	Description:	generate a Vector which recode all of XPath
	Input:			thread_control_block   - interpreter info
	Input:			fname                  - import file name
	Return: 		NULL
*************************************************/ 
#define   XPATH_VECTOR_BLOCK_SIZE    1024
void generateXPathVector(
		struct thread_control_block* objThreadCntrolBlock, char * fname)
{
	int iXPathVectorSize = XPATH_VECTOR_BLOCK_SIZE;
	char xpath_file_name[FILE_PATH_LEN];
	int iLineNum = 0 ;
	char contentLine[FILE_PATH_LEN];
	char * contentSepPtr; 
	char contentLineNum[LAB_LEN];
	char contentXPath[FILE_PATH_LEN];

	FILE *xpath_file ;

	memset(xpath_file_name, 0x00, FILE_PATH_LEN);
#ifdef WIN32
	sprintf(xpath_file_name, "%s_xpath.txt", fname);
#else
	sprintf(xpath_file_name, "%s\/programs\/%s_xpath.txt", 
			forgesight_get_programs_path(), fname);
#endif

	if((xpath_file = fopen(xpath_file_name, "r"))==NULL){
		perror("open file failed\n");  
		return ;
	}
	
	// Pre-arrange
	objThreadCntrolBlock->vector_XPath.resize(iXPathVectorSize);

	memset(contentLine,    0x00, FILE_PATH_LEN);
	while(fgets(contentLine,sizeof(contentLine),xpath_file)!=NULL)  
	{  
		memset(contentLineNum, 0x00, LAB_LEN);
		memset(contentXPath,   0x00, FILE_PATH_LEN);
		contentSepPtr = strchr(contentLine, ':');
		if(contentSepPtr)
		{
			memcpy(contentLineNum, contentLine, 
				contentSepPtr - contentLine);
			char * strEnter = strchr(contentSepPtr + 1, '\n');
			if(strEnter)
			{
			    memcpy(contentXPath, contentSepPtr + 1, strlen(contentSepPtr + 1) - 1);
			}
			else
			{
				strcpy(contentXPath,   contentSepPtr + 1);
			}
			iLineNum = atoi(contentLineNum);
			if(iLineNum >= iXPathVectorSize)
			{
				iXPathVectorSize += XPATH_VECTOR_BLOCK_SIZE ;
				objThreadCntrolBlock->vector_XPath.resize(iXPathVectorSize);
			}
			objThreadCntrolBlock->vector_XPath[iLineNum] = string(contentXPath) ;
		}
	}
	objThreadCntrolBlock->vector_XPath.resize(iLineNum + 2);
	fclose(xpath_file);
//	for(int i =1 ; i < objThreadCntrolBlock->vector_XPath.size() ; i++)
//	{
//		FST_INFO("%d - %s" , i, objThreadCntrolBlock->vector_XPath[i].c_str());
//	}
}

/************************************************* 
	Function:		getLineNumFromXPathVector
	Description:	get line number by XPath 
	Input:			xPath                  - XPath (ProgramName:XPath)
	Return: 		i                      - line number
*************************************************/ 
int getLineNumFromXPathVector(
		struct thread_control_block* objThreadCntrolBlock, char * xPath)
{
    for(unsigned int i = 0; i < (int)objThreadCntrolBlock->vector_XPath.size(); ++i)  
    {  
	//    FST_INFO("getLineNumFromXPathVector Try:: %s ", 
	//		objThreadCntrolBlock->vector_XPath[i].c_str());
        if(objThreadCntrolBlock->vector_XPath[i] == string(xPath))
        	return i ;
    }
	FST_INFO("getLineNumFromXPathVector Failed:: %s ", xPath);
	return -1 ;
}

/************************************************* 
	Function:		getMaxLineNum
	Description:	get line count
	Input:			NULL
	Return: 		line count
*************************************************/ 
int getMaxLineNum(struct thread_control_block* objThreadCntrolBlock)
{
	return (int)objThreadCntrolBlock->vector_XPath.size() ;
}

