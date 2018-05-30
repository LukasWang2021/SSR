#include "share_mem.h"
#include "error_code.h"
#include "error_monitor.h"
#include "sub_functions.h"
#include "shm.h"

#define MAX_ACCURATE_VALUE      (0.1)
#define MIN_ACCURATE_VALUE		(0.00001)

ShareMem* ShareMem::shm_instance_ = NULL;
ShareMem::ShareMem(RosBasic *ros_basic):ros_basic_(ros_basic)
{
    jnt_updated_ = true;
	shm_jnt_cmd_.empty = true;  //has not writen any joint command
    initial();

    createShm(SHM_INTPRT_CMD,        SHM_INTPRTCMD_SIZE);
    createShm(SHM_INTPRT_STATUS,     SHM_INTPRTSTATUS_SIZE);
    createShm(SHM_REG_IO_INFO,       SHM_INTPRTSTATUS_SIZE);
    createShm(SHM_CHG_REG_LIST_INFO, SHM_INTPRTSTATUS_SIZE);
    createShm(SHM_CTRL_CMD,          SHM_CTRLCMD_SIZE);
    createShm(SHM_CTRL_STATUS,       SHM_CTRLSTATUS_SIZE);

    shm_instance_ = this;
}

ShareMem::~ShareMem()
{
}

ShareMem* ShareMem::instance()
{
    return shm_instance_;
}

void ShareMem::initial()
{
    U64 result = TPI_SUCCESS;
    //FST_INFO("init share memory...");

#ifdef CROSS_PLATFORM
	if ((result = core_interface_.init()) != TPI_SUCCESS)
    {
        rcs::Error::instance()->add(result);
    }  
#endif
}

bool ShareMem::getLatestJoint(Joint &servo_joint)
{
    U64 result;
     //////////////////////////////////////////////////////
    // read the latest joint value
    // ==================================================
    int count = 0;
    do
    {
        FeedbackJointState fbjs;
#ifdef CROSS_PLATFORM
	    result = core_interface_.recvBareCore(fbjs);
#else
	    result = core_interface_.recvBareCoreFake(fbjs);
#endif
        if (result == TPI_SUCCESS)
        {
            servo_state_ = fbjs.state; 
            servo_joint = *(fst_controller::Joint*)fbjs.position;
            break;
        }
        usleep(1000);   
        if (count >= 500)       //500ms timeout
        {
            rcs::Error::instance()->add(READ_SHARE_MEMORY_TIMEOUT);
            return false;
        }
    }while(result);

    return true;
}

void ShareMem::setEmptyFlag(bool flag)
{
    shm_jnt_cmd_.empty = flag;
}


/**
 * @brief: set current share memory JointCommand
 *
 */
void ShareMem::setCurrentJointCmd(JointCommand joint_cmd)
{
	shm_jnt_cmd_.joint_cmd = joint_cmd; 
    shm_jnt_cmd_.empty = false;
}

/**
 * @brief: get FeedbackJointState from share memory
 *
 * @param fbjs: output==> the FeedbackJointState
 *
 * @return: true if successfullly get the joint state 
 */
U64 ShareMem::getFeedbackJoint(Joint &servo_joint)
{    
    static uint32_t read_cnt = 0;    
    read_cnt++;

    static FeedbackJointState fbjs;
	memset(&fbjs, 0, sizeof(FeedbackJointState));
#ifdef CROSS_PLATFORM
	U64 result = core_interface_.recvBareCore(fbjs);
#else
	U64 result = core_interface_.recvBareCoreFake(fbjs);
#endif
    if ((result == TPI_SUCCESS)/* || (fbjs.state != STATE_INIT)*/)
    {        
        read_cnt = 0;
        if (servo_state_ != fbjs.state)
        {
            FST_INFO("servo_state_:%d", fbjs.state);
            pre_servo_state_ = servo_state_.load();
            servo_state_ = fbjs.state;
        }
        Joint *cur_jnt = (Joint*)fbjs.position;
        if (!isJointChanged(&servo_joint, cur_jnt))
            return PARAMETER_NOT_UPDATED;
        if (isOutMax(&servo_joint, cur_jnt))
        {
            //FST_ERROR("interval too much between two joints from encoder");
            printDbLine("prev_joints:", (double*)&servo_joint, 6);
            printDbLine("cur_joints:", (double*)&fbjs.position, 6);
        }
        
        jnt_updated_ = true;
        servo_joint = *(fst_controller::Joint*)fbjs.position;
#ifdef SIMMULATION
        // ===update joints and publish to rviz===================
        ros_basic_->pubJointState(servo_joint);
#endif
        return TPI_SUCCESS;
    }		
    else if (read_cnt >= READ_COUNT_LIMIT)
    {
        FST_ERROR("read share memory timeout");        
        read_cnt = 0;
        return READ_SHARE_MEMORY_TIMEOUT;
    }
    else
    {
       //FST_ERROR("read memory failed:%d times, error:0x%x",read_cnt,result);
       return READ_SHARE_MEMORY_FAILED;
    }

}

/**
 * @brief: set JointCommand to share memory
 *
 * @param jc_w: input==> the JointCommand
 *
 * @return: true if successfullly set JointCommand 
 */
U64 ShareMem::setJointPositions()
{
    if (shm_jnt_cmd_.empty)
        return TPI_SUCCESS;
    static uint32_t write_cnt = 0;
    write_cnt++;
#ifdef CROSS_PLATFORM
	U64 result = core_interface_.sendBareCore(shm_jnt_cmd_.joint_cmd);
#else
	U64 result = core_interface_.sendBareCoreFake(shm_jnt_cmd_.joint_cmd);
#endif
	if (result == TPI_SUCCESS)
	{
		shm_jnt_cmd_.empty = true;
        /*if (servo_state_ == STATE_READY)//in case can't detect running state*/
        //{            
            //servo_state_ = STATE_RUNNING;
            //FST_INFO("current servo is running");
        /*}*/
        write_cnt = 0;
       // FST_INFO("Write memory successfully");
		return TPI_SUCCESS;
	}
	else 
	{
        if (write_cnt >= WRITE_COUNT_LIMIT)
        {
            write_cnt = 0;
            //FST_ERROR("write share memory timeout");
            return WRITE_SHARE_MEMORY_TIMEOUT;
        }
        else 
        {
           // FST_INFO("Write memory failed");
            return WRITE_SHARE_MEMORY_FAILED;
        }
	}
    
}




int ShareMem::getServoState()
{
    return servo_state_;
}

bool ShareMem::getInstruction(Instruction &inst)
{
    return lockRead(SHM_INTPRT_CMD, 0, (void*)&inst, sizeof(Instruction));
}

void ShareMem::sendingPermitted()
{
    //FST_INFO("unlock read===");
    unlockRead(SHM_INTPRT_CMD);
}

bool ShareMem::intprtControl(InterpreterControl ctrl)
{
    return tryWrite(SHM_CTRL_CMD, 0, (void*)&ctrl, sizeof(ctrl));
}

void ShareMem::setMoveCommandDestination(MoveCommandDestination& movCmdDst)
{ 
    writeShm(SHM_INTPRT_DST, 0, (void*)&movCmdDst, sizeof(movCmdDst));
}

void ShareMem::getMoveCommandDestination(MoveCommandDestination& movCmdDst)
{
    readShm(SHM_INTPRT_DST, 0, (void*)&movCmdDst, sizeof(movCmdDst));
}

bool ShareMem::getIntprtSendFlag()
{
    bool is_permitted = false ;
    int offset = &((CtrlStatus*)0)->is_permitted;
	readShm(SHM_CTRL_STATUS, offset, (void*)&is_permitted, sizeof(is_permitted));
	return is_permitted ;
}


void ShareMem::setIntprtSendFlag(bool flag)
{
    int offset = &((CtrlStatus*)0)->is_permitted;
    writeShm(SHM_CTRL_STATUS, offset, (void*)&flag, sizeof(flag));
}

void ShareMem::setIntprtDataFlag(bool flag)
{
    int offset = &((CtrlStatus*)0)->is_data_ready;
    writeShm(SHM_CTRL_STATUS, offset, (void*)&flag, sizeof(flag));
}

bool ShareMem::getIntprtDataFlag()
{
    bool is_data_ready;
    int offset = &((CtrlStatus*)0)->is_data_ready;     
    readShm(SHM_CTRL_STATUS, offset, (void*)&is_data_ready, sizeof(is_data_ready));
    return is_data_ready;
}

void ShareMem::setUserOpMode(UserOpMode mode)
{
    int offset = &((CtrlStatus*)0)->user_op_mode;
    writeShm(SHM_CTRL_STATUS, offset, (void*)&mode, sizeof(mode));
}

/*void ShareMem::setSysCtrlMode(SysCtrlMode mode)*/
//{
    //int offset = &((CtrlStatus*)0)->sys_ctrl_mode;
    //writeShm(SHM_CTRL_STATUS, offset, (void*)&mode, sizeof(mode));
/*}*/

void ShareMem::getCurLine(char * line)
{
    int offset = &((IntprtStatus*)0)->line;     
    readShm(SHM_INTPRT_STATUS, offset, (void*)line, TP_XPATH_LEN);
    return line;
}
InterpreterState ShareMem::getIntprtState()
{
    InterpreterState state;
    int offset = &((IntprtStatus*)0)->state;
    readShm(SHM_INTPRT_STATUS, offset, (void*)&state, sizeof(state));
    return state;
}

bool ShareMem::getRegInfo(RegMap * info)
{
    readShm(SHM_REG_IO_INFO, 0, (void*)info, sizeof(RegMap));
    return true;
}

bool ShareMem::getDIOInfo(char * info)
{
    readShm(SHM_REG_IO_INFO, 0, (void*)info, sizeof(char));
    return true;
}

std::vector<ChgFrameSimple> ShareMem::getChangeRegList()
{
    char strChgRegLst[1024];
	std::vector<ChgFrameSimple> vecRet ; 
	char tempDebug[1024];
	int iSeq = 0 ;
	RegChgList  * regChgList ;
	ChgFrameSimple * chgFrameSimple ;
    readShm(SHM_CHG_REG_LIST_INFO, 0, (void*)strChgRegLst, 1024);
	regChgList  = (RegChgList  *)strChgRegLst ;
	chgFrameSimple = (ChgFrameSimple *)((char *)regChgList + sizeof(RegChgList)) ;

	for (int i = 0 ; i < sizeof(RegChgList) + 1 * sizeof(ChgFrameSimple) ; i++)
		printf("GET:: data: %d\n", strChgRegLst[i]);
			
	printf("tempDebug: %d  (%d) .\n", regChgList->command, regChgList->count);
	for(int i = 0 ; i < regChgList->count ;i++)
	{
		printf("%d : %s \n", 
			chgFrameSimple[i].id, chgFrameSimple[i].comment);
		vecRet.push_back(chgFrameSimple[i]);
	}

    return vecRet;
}

bool ShareMem::isServoDone()
{
    if ((pre_servo_state_ == STATE_RUNNING) || (pre_servo_state_ == STATE_WAIT_SERVOREADY))
    {
        if (servo_state_ == STATE_READY) 
        {
            pre_servo_state_ = STATE_READY;
            return true;
        }
    }
    if ((pre_servo_state_ == STATE_RUNNING) || (pre_servo_state_ == STATE_WAIT_SERVODOWN))
    {
        if (servo_state_ == STATE_ERROR)
        {
            pre_servo_state_ = STATE_ERROR;
            return true;
        }
    }
    return false;
}

bool ShareMem::isJointUpdated()
{
    if (jnt_updated_)
    {
        jnt_updated_ = false;
        return true;
    }
    return false;
}


bool ShareMem::isOutMax(Joint *src_joints, Joint *dst_joints)
{
	if (fabs(src_joints->j1 - dst_joints->j1) > MAX_ACCURATE_VALUE) return true;
	if (fabs(src_joints->j2 - dst_joints->j2) > MAX_ACCURATE_VALUE) return true;
	if (fabs(src_joints->j3 - dst_joints->j3) > MAX_ACCURATE_VALUE) return true;
	if (fabs(src_joints->j4 - dst_joints->j4) > MAX_ACCURATE_VALUE) return true;
	if (fabs(src_joints->j5 - dst_joints->j5) > MAX_ACCURATE_VALUE) return true;
	if (fabs(src_joints->j6 - dst_joints->j6) > MAX_ACCURATE_VALUE) return true;

	return false;
}	

bool ShareMem::isJointChanged(Joint *src_joints, Joint *dst_joints)
{
	if (fabs(src_joints->j1 - dst_joints->j1) > MIN_ACCURATE_VALUE) return true;
	if (fabs(src_joints->j2 - dst_joints->j2) > MIN_ACCURATE_VALUE) return true;
	if (fabs(src_joints->j3 - dst_joints->j3) > MIN_ACCURATE_VALUE) return true;
	if (fabs(src_joints->j4 - dst_joints->j4) > MIN_ACCURATE_VALUE) return true;
	if (fabs(src_joints->j5 - dst_joints->j5) > MIN_ACCURATE_VALUE) return true;
	if (fabs(src_joints->j6 - dst_joints->j6) > MIN_ACCURATE_VALUE) return true;

	return false;
}	
