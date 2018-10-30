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
extern struct thread_control_block g_thread_control_block[NUM_THREAD + 1];

#ifdef WIN32
extern HANDLE    g_basic_interpreter_handle[NUM_THREAD + 1];
#else
extern pthread_t g_basic_interpreter_handle[NUM_THREAD + 1];
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
        FST_INFO("command:%s",command);
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
            FST_INFO("loop_cnt:%d", instr.loop_cnt);
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
                    FST_ERROR("Error:moveJ j1 j2 j3 j4 j5 j6 [smooth]");
                    return false;
                }
                instr.target.type = MOTION_JOINT;
                instr.target.joint_target = *(Joint*)&result[1];
                if (result.size() == 7)
                    instr.target.cnt = -1;
                else
                    instr.target.cnt = atoi(result[7].c_str());
            }
            if (result[0] == "moveL")
            {
                if(result.size() < 7)
                {
                    FST_ERROR("Error:moveL x y z a b c [smooth]");
                    return false;
                }
                instr.target.type = MOTION_LINE;
                instr.target.pose_target = *(PoseEuler*)&result[1];
                if (result.size() == 7)
                    instr.target.cnt = -1;
                else
                    instr.target.cnt = atoi(result[7].c_str());

            }
            if (result[0] == "moveC")
            {
                if(result.size() < 13)
                {
                    perror("Error:moveC x1 y1 z1 a1 b1 c1 x2 y2 z2 a2 b2 c2 [smooth]");
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
    FST_INFO("returnRegInfo to %d", (int)info.type);
	
	FST_INFO("reg.value = (");
	for(int iRet = 0 ; iRet < 100 ; iRet++)
	{
	    FST_INFO("%08X, ", info.value[iRet]);
	}
	FST_INFO(") ");
				
    writeShm(SHM_REG_IO_INFO, 0, (void*)&info, sizeof(RegMap));
	setIntprtDataFlag(true);
}

void returnDIOInfo(IOPathInfo& info)
{
    FST_INFO("returnDIOInfo to %s:%d", info.dio_path, (int)info.value);
    writeShm(SHM_REG_IO_INFO, 0, (char*)&info.value, sizeof(char));
	setIntprtDataFlag(true);
}

void returnIODeviceInfo(char * info, int iNum)
{
    writeShm(SHM_REG_IO_INFO, 0, (char*)info, sizeof(IODeviceInfoShm) * iNum);
	setIntprtDataFlag(true);
}

void resetProgramNameAndLineNum()
{
	setCurLine((char *)"", 0);
	setProgramName((char *)""); 
}

void setProgramName(char * program_name)
{
    FST_INFO("setProgramName to %s", program_name);
	strcpy(g_interpreter_publish.program_name, program_name); 
}

struct thread_control_block *  getThreadControlBlock()
{
	if(getCurrentThreadSeq() < 0)
    {
        FST_ERROR("getThreadControlBlock failed from %d", getCurrentThreadSeq());
		return NULL;
	}
	else
    {
    	FST_INFO("getThreadControlBlock at %d", getCurrentThreadSeq());
		return &g_thread_control_block[getCurrentThreadSeq()] ;
	}
}

int getCurrentThreadSeq()
{
	return g_iCurrentThreadSeq ;
}

void incCurrentThreadSeq()
{
	if(g_iCurrentThreadSeq < NUM_THREAD)
		g_iCurrentThreadSeq++ ;
	else
		g_iCurrentThreadSeq = 0 ;
}

void decCurrentThreadSeq()
{
	if(g_iCurrentThreadSeq == 0)
    	FST_ERROR("g_iCurrentThreadSeq == 0");
	g_iCurrentThreadSeq-- ;
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
//    FST_INFO("setPrgmState to %d", (int)state);
//    writeShm(SHM_INTPRT_STATUS, offset, (void*)&state, sizeof(state));
	
    FST_INFO("setPrgmState to %d", (int)state);
	g_interpreter_publish.status = state ;
}

void setCurLine(char * line, int lineNum)
{
// #ifdef WIN32
//  	Instruction temp,  * tempPtr = &temp;
//      int offset = (int)&(tempPtr->line) - (int)tempPtr ;
// #else
//   // int offset = (int)&intprt_status.line - (int)&intprt_status;
//      int offset = (int)&((Instruction*)0)->line;   
// #endif  
//     FST_INFO("setCurLine %s(%d) at %d", line, strlen(line), offset);
//  //    writeShm(SHM_INTPRT_STATUS, offset, (void*)&line, sizeof(line));
//  	writeShm(SHM_INTPRT_STATUS, offset, (void*)line, strlen(line) + 1); 
	strcpy(g_interpreter_publish.current_line_path, line); 
	g_interpreter_publish.current_line_num = lineNum; 
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
    bool ret = true;
//    int iLineNum = 0;
	// We had eaten MOV* as token. 
    if (objThdCtrlBlockPtr->is_abort)
    {
        // target_line++;
        return false;
    }
	// Speed up at 0930
//    ret = g_objRegManagerInterface->isNextInstructionNeeded();
//    if (ret == false)
 //   {
 //       FST_INFO("not permitted");
 //       return false;
 //   }
//	// setSendPermission(false);
	
//    int count = 0;
    //FST_INFO("cur state:%d", prgm_state);
    if ((objThdCtrlBlockPtr->prog_mode == STEP_MODE) 
		&& (prgm_state == INTERPRETER_EXECUTE_TO_PAUSE))
    {
//        if (isInstructionEmpty(SHM_INTPRT_CMD))
//        {
//            FST_INFO("check if step is done in setInstruction");
//            setPrgmState(INTERPRETER_PAUSED);
//        }
		// FST_INFO("cur state:%d in STEP_MODE ", prgm_state);
        return false;
    }

    // if (prgm_state != INTERPRETER_EXECUTE)
    //    return false;
    do
    {
		if (instruction->is_additional == false)
		{
	     	FST_INFO("setInstruction:: instr.target.cnt = %f .", instruction->target.cnt);
			// ret = tryWrite(SHM_INTPRT_CMD, 0, 
			//		(void*)instruction, sizeof(Instruction));
			
			ret = g_objRegManagerInterface->setInstruction(instruction);
		}
		else
		{
	     	FST_INFO("setInstruction:: instr.target.cnt = %f .", instruction->target.cnt);
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
			//  FST_INFO("set line in setInstruction");
	        //    setLinenum(objThdCtrlBlockPtr, iLineNum);

	        if (objThdCtrlBlockPtr->prog_mode == STEP_MODE)
	        {
			    FST_INFO("In STEP_MODE, it seems that it does not need to wait");
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
    FST_INFO("wait ctrl_status.is_permitted == false");
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
    FST_INFO("ctrl_status.is_permitted == true");

    return true;
}

bool getIntprtCtrl(InterpreterControl& intprt_ctrl)
{
    bool iRet = tryRead(SHM_CTRL_CMD, 0, (void*)&intprt_ctrl, sizeof(intprt_ctrl));
	if(g_lastcmd != intprt_ctrl.cmd)
    {
       FST_INFO("getIntprtCtrl = %d", intprt_ctrl.cmd);
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
	setProgramName(proj_name); 
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
			launch_code_thread_create(NULL);
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
			macro_instr_thread_create(NULL);
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
   			FST_INFO("waitInterpreterStateToPaused abort");
			break;
		}
		if(interpreterState == INTERPRETER_IDLE)
		{
   			FST_INFO("waitInterpreterStateToPaused = IDLE_R");
			break;
		}
#endif
	}
}	

void parseCtrlComand(InterpreterControl intprt_ctrl, void * requestDataPtr) 
	// (struct thread_control_block * objThdCtrlBlockPtr)
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
        FST_INFO("parseCtrlComand: %d", intprt_ctrl.cmd);
#endif
    switch (intprt_ctrl.cmd)
    {
        case fst_base::INTERPRETER_SERVER_CMD_LOAD:
            // FST_INFO("load file_name");
            break;
        case fst_base::INTERPRETER_SERVER_CMD_DEBUG:
			memcpy(intprt_ctrl.start_ctrl, requestDataPtr, 256);
            FST_INFO("start debug %s ...", intprt_ctrl.start_ctrl);
			incCurrentThreadSeq();
		    // objThdCtrlBlockPtr = &g_thread_control_block[getCurrentThreadSeq()];
		    objThdCtrlBlockPtr = getThreadControlBlock();
			if(objThdCtrlBlockPtr == NULL) break ;
			
            objThdCtrlBlockPtr->prog_mode = STEP_MODE;
			objThdCtrlBlockPtr->execute_direction = EXECUTE_FORWARD ;
            setPrgmState(INTERPRETER_PAUSED);
			if(strlen(intprt_ctrl.start_ctrl) == 0)
			{
			   strcpy(intprt_ctrl.start_ctrl, "while_test");
			}
            startFile(objThdCtrlBlockPtr, 
				intprt_ctrl.start_ctrl, getCurrentThreadSeq());
	        // g_iCurrentThreadSeq++ ;
            break;
        case fst_base::INTERPRETER_SERVER_CMD_START:
			memcpy(intprt_ctrl.start_ctrl, requestDataPtr, 256);
            FST_INFO("start run %s ...", intprt_ctrl.start_ctrl);
			incCurrentThreadSeq();
		    // objThdCtrlBlockPtr = &g_thread_control_block[getCurrentThreadSeq()];
		    objThdCtrlBlockPtr = getThreadControlBlock();
			if(objThdCtrlBlockPtr == NULL) break ;
			
            objThdCtrlBlockPtr->prog_mode = FULL_MODE;
			objThdCtrlBlockPtr->execute_direction = EXECUTE_FORWARD ;
            setPrgmState(INTERPRETER_EXECUTE);
			if(strlen(intprt_ctrl.start_ctrl) == 0)
			{
			   strcpy(intprt_ctrl.start_ctrl, "reconstruction_pr_test");
			}
			startFile(objThdCtrlBlockPtr, 
				intprt_ctrl.start_ctrl, getCurrentThreadSeq());
	        // g_iCurrentThreadSeq++ ;
            break;
        case fst_base::INTERPRETER_SERVER_CMD_JUMP:
			memcpy(intprt_ctrl.jump_line, requestDataPtr, 256);
			if(getCurrentThreadSeq() < 0) break ;
			if(g_basic_interpreter_handle[getCurrentThreadSeq()] == 0)
			{
            	FST_ERROR("Thread exits at %d ", getPrgmState());
				break;
			}
			// objThdCtrlBlockPtr = &g_thread_control_block[getCurrentThreadSeq()];
		    objThdCtrlBlockPtr = getThreadControlBlock();
			if(objThdCtrlBlockPtr == NULL) break ;
			if(objThdCtrlBlockPtr->is_in_macro == true)
			{
				FST_ERROR("Can not JUMP macro ");
				break;
			}
			if(objThdCtrlBlockPtr->is_paused == true)
			{
            	FST_ERROR("Can not JUMP in calling Pause ");
           		break;
			}
			if(getPrgmState() == INTERPRETER_EXECUTE)
			{
            	FST_ERROR("Can not JUMP in EXECUTE_R ");
           		break;
			}
            FST_INFO("jump to line:%s", intprt_ctrl.jump_line);
			iLineNum = getLineNumFromXPathVector(intprt_ctrl.jump_line);
			if(iLineNum > 0)
            {
            	setLinenum(objThdCtrlBlockPtr, iLineNum);
				// Jump prog
				jump_prog_from_line(objThdCtrlBlockPtr, iLineNum);
				// Just Move to line and do not execute
            	// setPrgmState(INTERPRETER_EXECUTE);
			}
			else
            {
            	FST_ERROR("Failed to jump to line:%d", iLineNum);
			}
			break;
        case fst_base::INTERPRETER_SERVER_CMD_SWITCH_STEP:
			memcpy(&intprt_ctrl.step_mode, requestDataPtr, sizeof(int));
            FST_INFO("switch Step at %d with %d", 
				getCurrentThreadSeq(), intprt_ctrl.step_mode);
			if(getCurrentThreadSeq() < 0) break ;
			if(g_basic_interpreter_handle[getCurrentThreadSeq()] == 0)
			{
            	FST_ERROR("Thread exits at %d ", getPrgmState());
				break;
			}
			// objThdCtrlBlockPtr = &g_thread_control_block[getCurrentThreadSeq()];
		    objThdCtrlBlockPtr = getThreadControlBlock();
			if(objThdCtrlBlockPtr == NULL) break ;
            FST_INFO("SWITCH_STEP with %d", intprt_ctrl.step_mode);
            objThdCtrlBlockPtr->prog_mode = intprt_ctrl.step_mode;
            break;
        case fst_base::INTERPRETER_SERVER_CMD_FORWARD:
            FST_INFO("step forward at %d ", getCurrentThreadSeq());
			if(getCurrentThreadSeq() < 0) break ;
			if(g_basic_interpreter_handle[getCurrentThreadSeq()] == 0)
			{
            	FST_ERROR("Thread exits at %d ", getPrgmState());
				break;
			}
			// objThdCtrlBlockPtr = &g_thread_control_block[getCurrentThreadSeq()];
		    objThdCtrlBlockPtr = getThreadControlBlock();
			if(objThdCtrlBlockPtr == NULL) break ;
			if(objThdCtrlBlockPtr->is_in_macro == true)
			{
				FST_ERROR("Can not FORWARD macro ");
				break;
			}
			if(objThdCtrlBlockPtr->is_paused == true)
			{
            	FST_ERROR("Can not FORWARD in calling Pause ");
           		break;
			}
			if(getPrgmState() == INTERPRETER_EXECUTE)
			{
            	FST_ERROR("Can not FORWARD in EXECUTE_R ");
           		break;
			}
			
            objThdCtrlBlockPtr->prog_mode = STEP_MODE ;
			objThdCtrlBlockPtr->execute_direction = EXECUTE_FORWARD ;
            // target_line++;
            iLineNum = getLinenum(objThdCtrlBlockPtr);
            FST_INFO("step forward to %d ", iLineNum);
            setPrgmState(INTERPRETER_EXECUTE);

			// Controller use the PrgmState and LineNum to check to execute 
//            FST_INFO("Enter waitInterpreterStateToPaused %d ", iLineNum);
//            waitInterpreterStateToPaused(objThdCtrlBlockPtr);
// 			// target_line++ in setInstruction
//            FST_INFO("Left  waitInterpreterStateToPaused %d ", iLineNum);

			// Use the program pointer to get the current line number.
			// to support logic
			iLineNum = calc_line_from_prog(objThdCtrlBlockPtr);
            setLinenum(objThdCtrlBlockPtr, iLineNum);
            break;
        case fst_base::INTERPRETER_SERVER_CMD_BACKWARD:
            FST_INFO("backward at %d ", getCurrentThreadSeq());
			if(getCurrentThreadSeq() < 0) break ;
			if(g_basic_interpreter_handle[getCurrentThreadSeq()] == 0)
			{
            	FST_ERROR("Thread exits at %d ", getPrgmState());
				break;
			}
			// objThdCtrlBlockPtr = &g_thread_control_block[getCurrentThreadSeq()];
		    objThdCtrlBlockPtr = getThreadControlBlock();
			if(objThdCtrlBlockPtr == NULL) break ;
			if(objThdCtrlBlockPtr->is_in_macro == true)
			{
				FST_ERROR("Can not BACKWARD macro ");
				break;
			}
			if(objThdCtrlBlockPtr->is_paused == true)
			{
            	FST_ERROR("Can not BACKWARD in calling Pause ");
           		break;
			}
			if(getPrgmState() == INTERPRETER_EXECUTE)
			{
            	FST_ERROR("Can not FORWARD in EXECUTE_R ");
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
            		FST_ERROR("Can not BACKWARD to %d(%d).",
						iLineNum, objThdCtrlBlockPtr->prog_jmp_line[iLineNum].type);
					break ;
				}
				iLineNum-- ;
		    	setLinenum(objThdCtrlBlockPtr, iLineNum);
            	FST_INFO("JMP to %d(%d) in the FORWARD -> BACKWARD .", 
					iLineNum,    objThdCtrlBlockPtr->prog_jmp_line[iLineNum].type);
				break;
			}
            // setPrgmState(INTERPRETER_EXECUTE);  
			// In this circumstance, 
			// We had jmp to the right line, we should use the iLineNum.
            iLineNum = getLinenum(objThdCtrlBlockPtr);
            if (iLineNum < PROGRAM_START_LINE_NUM)
            {
            	FST_ERROR("Can not BACKWARD out of program ");
  				setWarning(INFO_INTERPRETER_BACK_TO_BEGIN) ; 
                break;
            }
            // if (objThdCtrlBlockPtr->prog_jmp_line[iLineNum].type == MOTION)
            is_backward = true;
            // else {  perror("can't back");  break;      }
            objThdCtrlBlockPtr->prog_mode = STEP_MODE ;
			objThdCtrlBlockPtr->execute_direction = EXECUTE_BACKWARD ;
            FST_INFO("step BACKWARD to %d ", iLineNum);
			// set_prog_from_line(objThdCtrlBlockPtr, iLineNum);
            setPrgmState(INTERPRETER_EXECUTE);

			// Controller use the PrgmState and LineNum to check to execute 
//            FST_INFO("Enter waitInterpreterStateToPaused %d ", iLineNum);
//			waitInterpreterStateToPaused(objThdCtrlBlockPtr);
//			// target_line-- in setInstruction
//            FST_INFO("Left  waitInterpreterStateToPaused %d ", iLineNum);
			
			iLineNum-- ;
		    setLinenum(objThdCtrlBlockPtr, iLineNum);
		    break;
		case fst_base::INTERPRETER_SERVER_CMD_RESUME:
			if(getCurrentThreadSeq() < 0) break ;
		    // objThdCtrlBlockPtr = &g_thread_control_block[getCurrentThreadSeq()];
		    objThdCtrlBlockPtr = getThreadControlBlock();
			if(objThdCtrlBlockPtr == NULL) break ;
			
			if(getPrgmState() == INTERPRETER_PAUSED)
	        {
	            FST_INFO("continue move..");
				// Not Change program mode  
				// objThdCtrlBlockPtr->prog_mode = FULL_MODE;
	            setPrgmState(INTERPRETER_EXECUTE);
			}
			else
			    setWarning(FAIL_INTERPRETER_NOT_IN_PAUSE);
            break;
        case fst_base::INTERPRETER_SERVER_CMD_PAUSE:
			if(getCurrentThreadSeq() < 0) break ;
			// objThdCtrlBlockPtr = &g_thread_control_block[getCurrentThreadSeq()];
		    objThdCtrlBlockPtr = getThreadControlBlock();
			if(objThdCtrlBlockPtr == NULL) break ;
			if(objThdCtrlBlockPtr->is_in_macro == true)
			{
				FST_ERROR("Can not PAUSE macro ");
				break;
			}
			if(getPrgmState() == INTERPRETER_IDLE)
			{
            	FST_ERROR("Can not PAUSE in INTERPRETER_IDLE ");
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
            FST_INFO("abort motion");
			if(getCurrentThreadSeq() < 0) break ;
		    // objThdCtrlBlockPtr = &g_thread_control_block[getCurrentThreadSeq()];
		    objThdCtrlBlockPtr = getThreadControlBlock();
			if(objThdCtrlBlockPtr == NULL) break ;
			
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
  			FST_INFO("setPrgmState(IDLE_R).");
		    setPrgmState(INTERPRETER_IDLE);
		    // clear line path and ProgramName
		    resetProgramNameAndLineNum();
            break;
        case fst_base::INTERPRETER_SERVER_CMD_SET_AUTO_START_MODE:
			memcpy(&intprt_ctrl.autoMode, requestDataPtr, sizeof(AutoMode));
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
  	//		FST_INFO("left parseCtrlComand.");
}

void forgesight_load_programs_path()
{
	std::string data_path = "";
	g_files_manager_data_path = "";
#ifdef WIN32
    g_files_manager_data_path = std::string(DATA_PATH);
#else
	g_files_manager_data_path = string(getenv("ROBOT_DATA_PREFIX"));
	if(g_files_manager_data_path.length() == 0)
	{
	    fst_parameter::ParamGroup param_;
	    param_.loadParamFile("/root/install/share/configuration/machine/programs_path.yaml");
	    param_.getParam("file_manager/programs_path", g_files_manager_data_path);
	}
	else
	{
	    fst_parameter::ParamGroup param_;
	    param_.loadParamFile("/root/install/share/configuration/machine/programs_path.yaml");
	    param_.getParam("file_manager/data_path", data_path);
		g_files_manager_data_path.append(data_path);
	}
	FST_INFO("forgesight_load_programs_path: %s .", g_files_manager_data_path.c_str());
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

