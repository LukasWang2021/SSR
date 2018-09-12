#ifdef WIN32
#pragma warning(disable : 4786)
#endif
#include "forsight_inter_control.h"
#include "forsight_innercmd.h"
#include "forsight_program_property.h"
#include "forsight_io_mapping.h"
#include "forsight_io_controller.h"
#ifndef WIN32
// #include "io_interface.h"
#include "error_code.h"
#endif

#ifdef USE_FORSIGHT_REGISTERS_MANAGER
#ifndef WIN32
#include "reg_manager/reg_manager_interface_wrapper.h"
using namespace fst_ctrl ;
#endif
#include "reg_manager/forsight_registers_manager.h"

#else
#include "reg-shmi/forsight_registers.h"
#include "reg-shmi/forsight_op_regs_shmi.h"
#endif

#include "forsight_launch_code_startup.h"
#include "forsight_macro_instr_startup.h"

#define VELOCITY    (500)
using namespace std;

static std::vector<Instruction> g_script;
// static int block_start;
static int block_end;
bool terminated = false;

bool is_backward= false;
InterpreterState prgm_state = INTERPRETER_IDLE;
//bool send_flag = false;
// int target_line;
// static IntprtStatus intprt_status;
// static Instruction instruction;
static CtrlStatus ctrl_status;
// static InterpreterControl intprt_ctrl;

fst_base::InterpreterServerCmd g_lastcmd;

static InterpreterState g_privateInterpreterState;

extern jmp_buf e_buf; /* hold environment for longjmp() */
extern struct thread_control_block g_thread_control_block[NUM_THREAD];

#ifdef WIN32
extern HANDLE    g_basic_interpreter_handle[NUM_THREAD];
#else
extern pthread_t g_basic_interpreter_handle[NUM_THREAD];
#endif
int  g_iCurrentThreadSeq = -1 ;  // minus one add one equals to zero

AutoMode g_current_auto_mode = AUTOMODE_NONE_U;

std::string g_files_manager_data_path = "";

InterpreterPublish  g_interpreter_publish; 

vector<string> split(string str,string pattern)
{
	string::size_type pos;
	vector<string> result;
	str+=pattern;
	int size=str.size();

	for(int i=0; i<size; i++)
	{
		pos=str.find(pattern,i);
		if((int)pos<size)
		{
			string s=str.substr(i,pos-i);
			result.push_back(s);
			i=pos+pattern.size()-1;
		}
	}
	return result;
}

bool parseScript(const char* fname)
{
    Instruction instr;
    char command[256];
    string path = "/home/fst/Program/";
    path = path + fname;
    ifstream fin(path.c_str());
    int count = 0;
    while(fin.getline(command, sizeof(command)))
    {
        printf("command:%s\n",command);
#ifdef USE_XPATH
	    // itoa(++count, instr.line, 10);
        sprintf(instr.line, "%d", ++count);
#else
        instr.line = ++count;
#endif
        vector<string> result=split(command, " "); //use space to split
        if (result[0] == "LOOP")
        {
            instr.type = LOGIC_TOK; // instr.type = LOOP;
            if (result.size() == 1)
                instr.loop_cnt = -1;
            else 
                instr.loop_cnt = atoi(result[1].c_str());
            printf("loop_cnt:%d\n", instr.loop_cnt);
        }
        else if (result[0] == "END")
        {
            instr.type = END_PROG; // END;
        }
        else
        {
            instr.type = MOTION;
            if (result[0] == "moveJ")
            {
                if(result.size() < 7)
                {
                    printf("Error:moveJ j1 j2 j3 j4 j5 j6 [smooth]\n");
                    return false;
                }
                instr.target.type = MOTION_JOINT;
                instr.target.joint_target = *(_Joint*)&result[1];
                if (result.size() == 7)
                    instr.target.cnt = -1;
                else
                    instr.target.cnt = atoi(result[7].c_str());
            }
            if (result[0] == "moveL")
            {
                if(result.size() < 7)
                {
                    printf("Error:moveL x y z a b c [smooth]\n");
                    return false;
                }
                instr.target.type = MOTION_LINE;
                instr.target.pose_target = *(_PoseEuler*)&result[1];
                if (result.size() == 7)
                    instr.target.cnt = -1;
                else
                    instr.target.cnt = atoi(result[7].c_str());

            }
            if (result[0] == "moveC")
            {
                if(result.size() < 13)
                {
                    perror("Error:moveC x1 y1 z1 a1 b1 c1 x2 y2 z2 a2 b2 c2 [smooth]\n");
                    return false;
                }
                instr.target.type = MOTION_CIRCLE;
                instr.target.circle_target = *(CircleTarget*)&result[1];
                if (result.size() == 7)
                    instr.target.cnt = -1;
                else
                    instr.target.cnt = atoi(result[7].c_str());
            }            
        }
        g_script.push_back(instr);
        memset((void*)&instr, 0, sizeof(instr));
        memset(command, 0, sizeof(command));
    }
    fin.close();

    return true;
}

void findLoopEnd(int index)
{
    for(int i = index; i < (int)g_script.size(); i++)
    {
        if (g_script[i].type == END_PROG) // END)
        {
            block_end = i -1;
            return;
        }
    }
    block_end = g_script.size() - 1;
}

void setMoveCommandDestination(MoveCommandDestination movCmdDst)
{ 
    writeShm(SHM_INTPRT_DST, 0, (void*)&movCmdDst, sizeof(movCmdDst));
}

void getMoveCommandDestination(MoveCommandDestination& movCmdDst)
{
    readShm(SHM_INTPRT_DST, 0, (void*)&movCmdDst, sizeof(movCmdDst));
}

void copyMoveCommandDestination(MoveCommandDestination& movCmdDst)
{
    getMoveCommandDestination(movCmdDst);
    setMoveCommandDestination(movCmdDst);
}

void setIntprtDataFlag(bool flag)
{
#ifdef WIN32
	CtrlStatus temp,  * tempPtr = &temp;
    int offset = (int)&(tempPtr->is_data_ready) - (int)tempPtr ;
#else
    int offset = (int)&((CtrlStatus*)0)->is_data_ready;
#endif  
    writeShm(SHM_CTRL_STATUS, offset, (void*)&flag, sizeof(flag));
}

bool getIntprtDataFlag()
{
    bool is_data_ready;
#ifdef WIN32
	CtrlStatus temp,  * tempPtr = &temp;
    int offset = (int)&(tempPtr->is_data_ready) - (int)tempPtr ;
#else
    int offset = (int)&((CtrlStatus*)0)->is_data_ready;  
#endif     
    readShm(SHM_CTRL_STATUS, offset, (void*)&is_data_ready, sizeof(is_data_ready));
    return is_data_ready;
}

void returnRegInfo(RegMap info)
{
    printf("returnRegInfo to %d\n", (int)info.type);
	
	printf("reg.value = (");
	for(int iRet = 0 ; iRet < 100 ; iRet++)
	{
	    printf("%08X, ", info.value[iRet]);
	}
	printf(") \n");
				
    writeShm(SHM_REG_IO_INFO, 0, (void*)&info, sizeof(RegMap));
	setIntprtDataFlag(true);
}

void returnDIOInfo(IOPathInfo& info)
{
    printf("returnDIOInfo to %s:%d\n", info.dio_path, (int)info.value);
    writeShm(SHM_REG_IO_INFO, 0, (char*)&info.value, sizeof(char));
	setIntprtDataFlag(true);
}

void returnIODeviceInfo(char * info, int iNum)
{
    writeShm(SHM_REG_IO_INFO, 0, (char*)info, sizeof(IODeviceInfoShm) * iNum);
	setIntprtDataFlag(true);
}

InterpreterState getPrgmState()
{
	return g_privateInterpreterState ;
}

void setPrgmState(InterpreterState state)
{
 	g_privateInterpreterState = state ;
// #ifdef WIN32
//     IntprtStatus temp,  * tempPtr = &temp;
//     int offset = (int)&(tempPtr->state) - (int)tempPtr ;
// #else
//     int offset = (int)&((IntprtStatus*)0)->state;
// #endif
//    printf("setPrgmState to %d\n", (int)state);
//    writeShm(SHM_INTPRT_STATUS, offset, (void*)&state, sizeof(state));
	
	g_interpreter_publish.status = state ;
}

void setCurLine(char * line)
{
// #ifdef WIN32
//  	Instruction temp,  * tempPtr = &temp;
//      int offset = (int)&(tempPtr->line) - (int)tempPtr ;
// #else
//   // int offset = (int)&intprt_status.line - (int)&intprt_status;
//      int offset = (int)&((Instruction*)0)->line;   
// #endif  
//     printf("setCurLine %s(%d) at %d\n", line, strlen(line), offset);
//  //    writeShm(SHM_INTPRT_STATUS, offset, (void*)&line, sizeof(line));
//  	writeShm(SHM_INTPRT_STATUS, offset, (void*)line, strlen(line) + 1); 
	strcpy(g_interpreter_publish.current_line_path, line); 
}

#ifdef WIN32
void setWarning(__int64 warn)
#else
void setWarning(long long int warn)
#endif  
{
// #ifdef WIN32
//  	IntprtStatus temp,  * tempPtr = &temp;
//      int offset = (int)&(tempPtr->warn) - (int)tempPtr ;
// #else
//      int offset = (int)&((IntprtStatus*)0)->warn;
// #endif  
//      writeShm(SHM_INTPRT_STATUS, offset, (void*)&warn, sizeof(warn));
	g_objInterpreterServer->sendEvent(INTERPRETER_EVENT_TYPE_ERROR, &warn);
}

void setSendPermission(bool flag)
{
#ifdef WIN32
	CtrlStatus temp,  * tempPtr = &temp;
    int offset = (int)&(tempPtr->is_permitted) - (int)tempPtr ;
#else
    int offset = (int)&((CtrlStatus*)0)->is_permitted;
#endif  
    writeShm(SHM_CTRL_STATUS, offset, (void*)&flag, sizeof(flag));
}

void getSendPermission()
{
#ifdef WIN32
	CtrlStatus temp,  * tempPtr = &temp;
    int offset = (int)&(tempPtr->is_permitted) - (int)tempPtr ;
#else
    int offset = (int)&((CtrlStatus*)0)->is_permitted;
#endif  
    readShm(SHM_CTRL_STATUS, offset, (void*)&ctrl_status.is_permitted, sizeof(ctrl_status.is_permitted));
}

UserOpMode getUserOpMode()
{
#ifdef WIN32
	CtrlStatus temp,  * tempPtr = &temp;
    int offset = (int)&(tempPtr->user_op_mode) - (int)tempPtr ;
#else
    int offset = (int)&((CtrlStatus*)0)->user_op_mode;
#endif  
    readShm(SHM_CTRL_STATUS, offset, (void*)&ctrl_status.user_op_mode, sizeof(ctrl_status.user_op_mode));

    return ctrl_status.user_op_mode;
}

/*SysCtrlMode getMotionMode()*/
//{
    //int offset = &((CtrlStatus*)0)->sys_ctrl_mode;
    //readShm(SHM_CTRL_STATUS, offset, (void*)&ctrl_status.sys_ctrl_mode, sizeof(ctrl_status.sys_ctrl_mode));

    //return ctrl_status.sys_ctrl_mode;
//}

bool setInstruction(struct thread_control_block * objThdCtrlBlockPtr, Instruction * instruction)
{
    bool ret;
//    int iLineNum = 0;
	// We had eaten MOV* as token. 
    if (objThdCtrlBlockPtr->is_abort)
    {
        // target_line++;
        return false;
    }
    ret = g_objRegManagerInterface->isNextInstructionNeeded();
    if (ret == false)
    {
        printf("not permitted\n");
        return false;
    }
	// setSendPermission(false);
	
//    int count = 0;
    //printf("cur state:%d\n", prgm_state);
    if ((objThdCtrlBlockPtr->prog_mode == STEP_MODE) 
		&& (prgm_state == INTERPRETER_EXECUTE_TO_PAUSE))
    {
        if (isInstructionEmpty(SHM_INTPRT_CMD))
        {
            printf("check if step is done in setInstruction\n");
            setPrgmState(INTERPRETER_PAUSED);
        }
		// printf("cur state:%d in STEP_MODE \n", prgm_state);
        return false;
    }

    // if (prgm_state != INTERPRETER_EXECUTE)
    //    return false;
    do
    {
		if (instruction->is_additional == false)
		{
	     	// printf("instr.target.cnt = %f .\n", instruction->target.cnt);
			// ret = tryWrite(SHM_INTPRT_CMD, 0, 
			//		(void*)instruction, sizeof(Instruction));
			
			ret = g_objRegManagerInterface->setInstruction(instruction);
		}
		else
		{
	     	// printf("instr.target.cnt = %f .\n", instruction->target.cnt);
			// ret = tryWrite(SHM_INTPRT_CMD, 0, 
			//  	(void*)instruction, 
			//  	sizeof(Instruction) 
			//  		+ sizeof(AdditionalInfomation) * instruction->add_num);
			
			ret = g_objRegManagerInterface->setInstruction(instruction);
		}
#ifndef WIN32
	    if (ret)
#endif
	    {
	        if (is_backward)
	        {
	            is_backward = false;
		        //    iLineNum--;
		        //    setCurLine(iLineNum);
	        }
	        //else
	        //{
	        //    iLineNum++;
	        //    setCurLine(iLineNum);
	        //}   
			//	iLineNum = calc_line_from_prog(objThdCtrlBlockPtr);
			//  printf("set line in setInstruction\n");
	        //    setLinenum(objThdCtrlBlockPtr, iLineNum);

	        if (objThdCtrlBlockPtr->prog_mode == STEP_MODE)
	        {
			    printf("In STEP_MODE, it seems that it does not need to wait\n");
	            // setPrgmState(INTERPRETER_EXECUTE_TO_PAUSE);   //wait until this Instruction end
            }
	    }

#ifdef WIN32
		Sleep(1);
		break ;
#else
        usleep(1000);
#endif
     //   if (count++ > 500)
     //       return false;
    }while(!ret);

    // Wait until finish 
    ret = g_objRegManagerInterface->isNextInstructionNeeded();
    printf("wait ctrl_status.is_permitted == false\n");
    while (ret == false)
    {
#ifdef WIN32
		Sleep(1);
		break ;
#else
        usleep(1000);
#endif
    	ret = g_objRegManagerInterface->isNextInstructionNeeded();
    }
    printf("ctrl_status.is_permitted == true\n");

    return true;
}

bool getIntprtCtrl(InterpreterControl& intprt_ctrl)
{
    bool iRet = tryRead(SHM_CTRL_CMD, 0, (void*)&intprt_ctrl, sizeof(intprt_ctrl));
	if(g_lastcmd != intprt_ctrl.cmd)
    {
       printf("getIntprtCtrl = %d\n", intprt_ctrl.cmd);
	   g_lastcmd = (fst_base::InterpreterServerCmd)intprt_ctrl.cmd ;
	}
	return iRet ;
}

void startFile(struct thread_control_block * objThdCtrlBlockPtr, 
	char * proj_name, int idx)
{
	strcpy(objThdCtrlBlockPtr->project_name, proj_name); // "prog_demo_dec"); // "BAS-EX1.BAS") ; // 
	objThdCtrlBlockPtr->is_main_thread = MAIN_THREAD ;
	objThdCtrlBlockPtr->is_in_macro    = false ;
	objThdCtrlBlockPtr->iThreadIdx = idx ;
	append_program_prop_mapper(objThdCtrlBlockPtr, proj_name);
	// Refresh InterpreterPublish project_name
	strcpy(g_interpreter_publish.program_name, proj_name); 
	// Start thread
	basic_thread_create(idx, objThdCtrlBlockPtr);
	// intprt_ctrl.cmd = LOAD ;
}

bool deal_auto_mode(AutoMode autoMode)
{
	if(g_current_auto_mode == autoMode)
	{
    	return true;
	}	
	switch(g_current_auto_mode) 	
	{
	// NoThread -> Thread
	case AUTOMODE_NONE_U:
	case LOCAL_TRIGGER_U:
		if(autoMode == LOCAL_TRIGGER_U)
		{
    		return true;
		}
		else if(autoMode == LAUNCH_CODE_U)
		{
			launch_code_thread_create(NULL);
    		return true;
		}
		else if(autoMode == MACRO_TRIGGER_U)
		{
			macro_instr_thread_create(NULL);
    		return true;
		}
		break;
	// Thread -> NoThread/OtherThread
	case LAUNCH_CODE_U:
		// Thread -> NoThread
		if(autoMode == LOCAL_TRIGGER_U)
		{
			launch_code_thread_destroy();
    		return true;
		}
		else if(autoMode == MACRO_TRIGGER_U)
		{
			launch_code_thread_destroy();
			macro_instr_thread_create(NULL);
    		return true;
		}
		break;
	// Thread -> NoThread/OtherThread
	case MACRO_TRIGGER_U:
		// Thread -> NoThread
		if(autoMode == LOCAL_TRIGGER_U)
		{
			launch_code_thread_destroy();
    		return true;
		}
		else if(autoMode == LAUNCH_CODE_U)
		{
			macro_instr_thread_destroy();
			launch_code_thread_create(NULL);
    		return true;
		}
		break;
	default:
		break;
	}
    return true;
}

/*
void waitInterpreterStateleftWaiting(
	struct thread_control_block * objThdCtrlBlockPtr)
{
	InterpreterState interpreterState  = getPrgmState();
	while(interpreterState == WAITING_R)
	{
#ifdef WIN32
		Sleep(100);
		interpreterState = EXECUTE_R ;
#else
		sleep(1);
		interpreterState = getPrgmState();
		if(objThdCtrlBlockPtr->is_abort == true)
		{
			// setPrgmState(INTERPRETER_PAUSE_TO_IDLE) ;
			break;
		}
#endif
	}
}

void waitInterpreterStateToWaiting(
	struct thread_control_block * objThdCtrlBlockPtr)
{
	InterpreterState interpreterState  = getPrgmState();
	while(interpreterState != WAITING_R)
	{
#ifdef WIN32
		Sleep(100);
		// interpreterState = EXECUTE_R ;
#else
		sleep(1);
		interpreterState = getPrgmState();
		if(objThdCtrlBlockPtr->is_abort == true)
		{
			// setPrgmState(INTERPRETER_PAUSE_TO_IDLE) ;
			break;
		}
#endif
	}
}	
*/

void waitInterpreterStateleftPaused(
	struct thread_control_block * objThdCtrlBlockPtr)
{
	InterpreterState interpreterState  = getPrgmState();
	while(interpreterState == INTERPRETER_PAUSED)
	{
#ifdef WIN32
		Sleep(100);
		interpreterState = INTERPRETER_EXECUTE ;
#else
		sleep(1);
		interpreterState = getPrgmState();
		if(objThdCtrlBlockPtr->is_abort == true)
		{
			// setPrgmState(INTERPRETER_PAUSE_TO_IDLE) ;
			break;
		}
#endif
	}
}

void waitInterpreterStateToPaused(
	struct thread_control_block * objThdCtrlBlockPtr)
{
	InterpreterState interpreterState  = getPrgmState();
	while(interpreterState != INTERPRETER_PAUSED)
	{
#ifdef WIN32
		Sleep(100);
		// interpreterState = INTERPRETER_EXECUTE ;
#else
		sleep(1);
		interpreterState = getPrgmState();
		if(objThdCtrlBlockPtr->is_abort == true)
		{
			// setPrgmState(INTERPRETER_PAUSE_TO_IDLE) ;
   			printf("waitInterpreterStateToPaused abort\n");
			break;
		}
		if(interpreterState == INTERPRETER_IDLE)
		{
   			printf("waitInterpreterStateToPaused = IDLE_R\n");
			break;
		}
#endif
	}
}	

void parseCtrlComand(InterpreterControl intprt_ctrl) // (struct thread_control_block * objThdCtrlBlockPtr)
{
//	InterpreterState interpreterState  = IDLE_R;
#ifdef WIN32
	__int64 result = 0 ;
#endif

//	RegMap reg ;
	IOPathInfo  dioPathInfo ;
	
	int iLineNum = 0 ;
	static fst_base::InterpreterServerCmd lastCmd ;
	UserOpMode userOpMode ;
	AutoMode   autoMode ;
    thread_control_block * objThdCtrlBlockPtr = NULL;

#ifndef WIN32
//    int iIONum = 0 ;
//	fst_io_manager::IODeviceInfo * objIODeviceInfoPtr ;
//	char * objCharPtr ;
//	IODeviceInfoShm * objIODeviceInfoShmPtr ; 
	
//	char tempDebug[1024];
//	int iSeq = 0 ;
//	char * testDebug = 0 ;
//	RegChgList  * regChgList ;
//	ChgFrameSimple * chgFrameSimple ;
//	RegManagerInterface objRegManagerInterface("share/configuration/machine");
//	std::vector<BaseRegData> vecRet ; 
//    char * strChgRegLst ;
	// if(intprt_ctrl.cmd != UPDATE_IO_DEV_ERROR)
	if(intprt_ctrl.cmd != fst_base::INTERPRETER_SERVER_CMD_LOAD)
        printf("parseCtrlComand: %d\n", intprt_ctrl.cmd);
#endif
    switch (intprt_ctrl.cmd)
    {
        case fst_base::INTERPRETER_SERVER_CMD_LOAD:
            // printf("load file_name\n");
            break;
        case fst_base::INTERPRETER_SERVER_CMD_DEBUG:
            printf("debug...\n");
			g_iCurrentThreadSeq++ ;
			if(g_iCurrentThreadSeq < 0) break ;
		    objThdCtrlBlockPtr = &g_thread_control_block[g_iCurrentThreadSeq];
			
            objThdCtrlBlockPtr->prog_mode = STEP_MODE;
			objThdCtrlBlockPtr->execute_direction = EXECUTE_FORWARD ;
            setPrgmState(INTERPRETER_PAUSED);
			if(strlen(intprt_ctrl.start_ctrl) == 0)
			{
			   strcpy(intprt_ctrl.start_ctrl, "while_test");
			}
            startFile(objThdCtrlBlockPtr, 
				intprt_ctrl.start_ctrl, g_iCurrentThreadSeq);
	        // g_iCurrentThreadSeq++ ;
            break;
        case fst_base::INTERPRETER_SERVER_CMD_START:
            printf("start run...\n");
            printf("start run %s ...\n", intprt_ctrl.start_ctrl);
			g_iCurrentThreadSeq++ ;
			if(g_iCurrentThreadSeq < 0) break ;
		    objThdCtrlBlockPtr = &g_thread_control_block[g_iCurrentThreadSeq];
			
            objThdCtrlBlockPtr->prog_mode = FULL_MODE;
			objThdCtrlBlockPtr->execute_direction = EXECUTE_FORWARD ;
            setPrgmState(INTERPRETER_EXECUTE);
			if(strlen(intprt_ctrl.start_ctrl) == 0)
			{
			   strcpy(intprt_ctrl.start_ctrl, "reconstruction_pr_test");
			}
			startFile(objThdCtrlBlockPtr, 
				intprt_ctrl.start_ctrl, g_iCurrentThreadSeq);
	        // g_iCurrentThreadSeq++ ;
            break;
        case fst_base::INTERPRETER_SERVER_CMD_JUMP:
			if(g_iCurrentThreadSeq < 0) break ;
			if(g_basic_interpreter_handle[g_iCurrentThreadSeq] == 0)
			{
            	printf("Thread exits at %d \n", getPrgmState());
				break;
			}
			objThdCtrlBlockPtr = &g_thread_control_block[g_iCurrentThreadSeq];
			if(objThdCtrlBlockPtr->is_in_macro == true)
			{
				printf("Can not JUMP macro \n");
				break;
			}
			if(objThdCtrlBlockPtr->is_paused == true)
			{
            	printf("Can not FORWARD in calling Pause \n");
           		break;
			}
			if(getPrgmState() == INTERPRETER_EXECUTE)
			{
            	printf("Can not FORWARD in EXECUTE_R \n");
           		break;
			}
            printf("jump to line:%d\n", intprt_ctrl.jump_line);
			iLineNum = intprt_ctrl.jump_line;
            setLinenum(objThdCtrlBlockPtr, iLineNum);
			// Jump prog
			jump_prog_from_line(objThdCtrlBlockPtr, iLineNum);
			// Just Move to line and do not execute
            // setPrgmState(INTERPRETER_EXECUTE);
			break;
        case fst_base::INTERPRETER_SERVER_CMD_FORWARD:
            printf("step forward at %d \n", g_iCurrentThreadSeq);
			if(g_iCurrentThreadSeq < 0) break ;
			if(g_basic_interpreter_handle[g_iCurrentThreadSeq] == 0)
			{
            	printf("Thread exits at %d \n", getPrgmState());
				break;
			}
			objThdCtrlBlockPtr = &g_thread_control_block[g_iCurrentThreadSeq];
			if(objThdCtrlBlockPtr->is_in_macro == true)
			{
				printf("Can not FORWARD macro \n");
				break;
			}
			if(objThdCtrlBlockPtr->is_paused == true)
			{
            	printf("Can not FORWARD in calling Pause \n");
           		break;
			}
			if(getPrgmState() == INTERPRETER_EXECUTE)
			{
            	printf("Can not FORWARD in EXECUTE_R \n");
           		break;
			}
			
            objThdCtrlBlockPtr->prog_mode = STEP_MODE ;
			objThdCtrlBlockPtr->execute_direction = EXECUTE_FORWARD ;
            // target_line++;
            iLineNum = getLinenum(objThdCtrlBlockPtr);
            printf("step forward to %d \n", iLineNum);
            setPrgmState(INTERPRETER_EXECUTE);
			
            printf("Enter waitInterpreterStateToPaused %d \n", iLineNum);
			waitInterpreterStateToPaused(objThdCtrlBlockPtr);
			// target_line++ in setInstruction
            printf("Left  waitInterpreterStateToPaused %d \n", iLineNum);

			// Use the program pointer to get the current line number.
			// to support logic
			iLineNum = calc_line_from_prog(objThdCtrlBlockPtr);
            setLinenum(objThdCtrlBlockPtr, iLineNum);
            break;
        case fst_base::INTERPRETER_SERVER_CMD_BACKWARD:
            printf("backward at %d \n", g_iCurrentThreadSeq);
			if(g_iCurrentThreadSeq < 0) break ;
			if(g_basic_interpreter_handle[g_iCurrentThreadSeq] == 0)
			{
            	printf("Thread exits at %d \n", getPrgmState());
				break;
			}
			objThdCtrlBlockPtr = &g_thread_control_block[g_iCurrentThreadSeq];
			if(objThdCtrlBlockPtr->is_in_macro == true)
			{
				printf("Can not BACKWARD macro \n");
				break;
			}
			if(objThdCtrlBlockPtr->is_paused == true)
			{
            	printf("Can not BACKWARD in calling Pause \n");
           		break;
			}
			if(getPrgmState() == INTERPRETER_EXECUTE)
			{
            	printf("Can not FORWARD in EXECUTE_R \n");
           		break;
			}

			if(lastCmd == fst_base::INTERPRETER_SERVER_CMD_FORWARD)
			{
			    // In this circumstance, 
			    // call calc_line_from_prog to get the next FORWARD line.
			    iLineNum = calc_line_from_prog(objThdCtrlBlockPtr);
				if((objThdCtrlBlockPtr->prog_jmp_line[iLineNum - 1].type == LOGIC_TOK)
				 ||(objThdCtrlBlockPtr->prog_jmp_line[iLineNum - 1].type == END_TOK))
				{
            		printf("Can not BACKWARD to %d(%d).\n",
						iLineNum, objThdCtrlBlockPtr->prog_jmp_line[iLineNum].type);
					break ;
				}
				iLineNum-- ;
		    	setLinenum(objThdCtrlBlockPtr, iLineNum);
            	printf("JMP to %d(%d) in the FORWARD -> BACKWARD .\n", 
					iLineNum,    objThdCtrlBlockPtr->prog_jmp_line[iLineNum].type);
				break;
			}
            // setPrgmState(INTERPRETER_EXECUTE);  
			// In this circumstance, 
			// We had jmp to the right line, we should use the iLineNum.
            iLineNum = getLinenum(objThdCtrlBlockPtr);
            if (iLineNum < PROGRAM_START_LINE_NUM)
            {
            	printf("Can not BACKWARD out of program \n");
  				setWarning(INFO_INTERPRETER_BACK_TO_BEGIN) ; 
                break;
            }
            // if (objThdCtrlBlockPtr->prog_jmp_line[iLineNum].type == MOTION)
            is_backward = true;
            // else {  perror("can't back\n");  break;      }
            objThdCtrlBlockPtr->prog_mode = STEP_MODE ;
			objThdCtrlBlockPtr->execute_direction = EXECUTE_BACKWARD ;
            printf("step BACKWARD to %d \n", iLineNum);
			// set_prog_from_line(objThdCtrlBlockPtr, iLineNum);
            setPrgmState(INTERPRETER_EXECUTE);

            printf("Enter waitInterpreterStateToPaused %d \n", iLineNum);
			waitInterpreterStateToPaused(objThdCtrlBlockPtr);
			// target_line-- in setInstruction
            printf("Left  waitInterpreterStateToPaused %d \n", iLineNum);
			
			iLineNum-- ;
		    setLinenum(objThdCtrlBlockPtr, iLineNum);
		    break;
		case fst_base::INTERPRETER_SERVER_CMD_RESUME:
			if(g_iCurrentThreadSeq < 0) break ;
		    objThdCtrlBlockPtr = &g_thread_control_block[g_iCurrentThreadSeq];
			
			if(getPrgmState() == INTERPRETER_PAUSED)
	        {
	            printf("continue move..\n");
				// Not Change program mode  
				// objThdCtrlBlockPtr->prog_mode = FULL_MODE;
	            setPrgmState(INTERPRETER_EXECUTE);
			}
			else
			    setWarning(FAIL_INTERPRETER_NOT_IN_PAUSE);
            break;
        case fst_base::INTERPRETER_SERVER_CMD_PAUSE:
			if(g_iCurrentThreadSeq < 0) break ;
			objThdCtrlBlockPtr = &g_thread_control_block[g_iCurrentThreadSeq];
			if(objThdCtrlBlockPtr->is_in_macro == true)
			{
				printf("Can not PAUSE macro \n");
				break;
			}
			
            userOpMode = getUserOpMode();
            if ((userOpMode == SLOWLY_MANUAL_MODE_U)
            || (userOpMode == UNLIMITED_MANUAL_MODE_U))
            {
                objThdCtrlBlockPtr->prog_mode = STEP_MODE ;
            }
            setPrgmState(INTERPRETER_PAUSED); 
            break;
        case fst_base::INTERPRETER_SERVER_CMD_ABORT:
            printf("abort motion\n");
			if(g_iCurrentThreadSeq < 0) break ;
		    objThdCtrlBlockPtr = &g_thread_control_block[g_iCurrentThreadSeq];
			
	        objThdCtrlBlockPtr->is_abort = true;
            // target_line = getMaxLineNum();
            // target_line = 0;
            // Restore program pointer
            objThdCtrlBlockPtr->prog = objThdCtrlBlockPtr->p_buf ;
			
		    setPrgmState(INTERPRETER_PAUSE_TO_IDLE);
#ifdef WIN32
			Sleep(1);
#else
	        usleep(1000);
#endif
  			printf("setPrgmState(IDLE_R).\n");
		    setPrgmState(INTERPRETER_IDLE);
            break;
        case fst_base::INTERPRETER_SERVER_CMD_SET_AUTO_START_MODE:
			// intprt_ctrl.RegMap.
			autoMode = intprt_ctrl.autoMode ;
			deal_auto_mode(autoMode);
			// forgesight_simulate_launch_config_values();
			// launch_code_thread_create(NULL);
			// macro_instr_thread_create(NULL);
			g_current_auto_mode = autoMode ;
			// Do nothing after it.
			// intprt_ctrl.cmd = LOAD ;
            break;
        default:
            break;

    }
	lastCmd = intprt_ctrl.cmd;
  	//		printf("left parseCtrlComand.\n");
}

void forgesight_load_programs_path()
{
	g_files_manager_data_path = "";
#ifdef WIN32
    g_files_manager_data_path = std::string(DATA_PATH);
#else
    fst_parameter::ParamGroup param_;
    param_.loadParamFile("/root/install/share/configuration/machine/programs_path.yaml");
    param_.getParam("file_manager/data", g_files_manager_data_path);
	printf("forgesight_load_programs_path: %s .\n", g_files_manager_data_path.c_str());
#endif
	
}

char * forgesight_get_programs_path()
{
	return (char *)g_files_manager_data_path.c_str();
}

void initShm()
{
//    openShm(SHM_INTPRT_CMD, 1024);
//    openShm(SHM_INTPRT_STATUS, 1024);
	
	// Lujiaming add at 0323
//    openShm(SHM_REG_IO_INFO, 1024);
	// Lujiaming add at 0323 end
	
	// Lujiaming add at 0514
//    openShm(SHM_CHG_REG_LIST_INFO, 1024);
	// Lujiaming add at 0514 end
	
//    openShm(SHM_CTRL_CMD, 1024);
//    openShm(SHM_CTRL_STATUS, 1024);
//    openShm(SHM_INTPRT_DST, 1024);
    // intprt_ctrl.cmd = LOAD;
    // intprt_ctrl.cmd = START;
	g_privateInterpreterState = INTERPRETER_IDLE ;
	
//	setPrgmState(INTERPRETER_IDLE);
#ifdef WIN32
//	generateFakeData();
#else
	
#ifndef USE_FORSIGHT_REGISTERS_MANAGER
	initRegShmi();
#endif
//	initShmi(1024);
#endif
	forgesight_load_programs_path();
}


void updateIOError()
{
#ifndef WIN32
//	U64 result = SUCCESS ;
//	result = IOInterface::instance()->updateIOError();
//    if (result != SUCCESS)
//    {
//		setWarning(result) ; 
//    }
#endif
}

