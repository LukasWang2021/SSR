#include "auto_motion.h"
#include "share_mem.h"
#include "error_monitor.h"

#define INSTRUCTION_RECV_INTERVAL   (1)

AutoMotion::AutoMotion(ArmGroup *arm_group):arm_group_(arm_group)
{
    mot_target_.cnt = -1;
   // prgm_state_ = IDLE_R;
}

AutoMotion::~AutoMotion()
{
        
}


/*ProgramState AutoMotion::getPrgmState()*/
//{
    //return prgm_state_.load();
//}

//void AutoMotion::setPrgmState(ProgramState state)
//{
    //prgm_state_ = state;
/*}*/

bool AutoMotion::getDoneFlag()
{
    return is_done_;
}

void AutoMotion::setDoneFlag(bool flag)
{
    is_done_ = flag;
}

void AutoMotion::motionClear()
{
    arm_group_->clearArmGroup();
}

int AutoMotion::getSmooth()
{
    return mot_target_.cnt;
}

void AutoMotion::pause()
{
    /*//===first change ProgramState=======*/
    //ProgramState prgm_state = prgm_state_;
    
    //if ((prgm_state == IDLE_R)
    //|| (prgm_state == PAUSED_R)
    //|| (prgm_state == EXECUTE_TO_PAUSE_T))
    //{
        //return;
    //}
    //else if (prgm_state == EXECUTE_R)
    //{                
        //prgm_state_ = EXECUTE_TO_PAUSE_T;
    //}
    //else if (prgm_state == IDLE_TO_EXECUTE_T)
    //{                
        //prgm_state_ = IDLE_R;
    //}
    //else if (prgm_state == PAUSE_TO_EXECUTE_T)
    //{
        //prgm_state_ = PAUSED_R;                
    /*}*/

    U64 result = arm_group_->suspendMotion();
    if (TPI_SUCCESS != result)
    {
        rcs::Error::instance()->add(result);            
    }
}

bool AutoMotion::resume()
{
   // if (prgm_state_ != PAUSED_R)
     //   return false;
    U64 result = arm_group_->resumeMotion();
    //=====clear all the fifos=======
    if (TPI_SUCCESS != result)
    {
        rcs::Error::instance()->add(result);
        //prgm_state_ = PAUSED_R;
        return false;
    }
    
    //prgm_state_ = PAUSE_TO_EXECUTE_T;        

    return true;
}


bool AutoMotion::step()
{
    /*InterpreterControl ctrl;*/
    //ctrl.cmd = FORWARD;
    /*ShareMem::instance()->intprtControl(ctrl);*/
}
bool AutoMotion::jump(int id)
{
    /*InterpreterControl ctrl;*/
    //ctrl.cmd = JUMP;
    //ctrl.line = id;
    /*ShareMem::instance()->intprtControl(ctrl);*/

}
bool AutoMotion::backwords()
{

}

bool AutoMotion::abort()
{
    //prgm_state_ = PAUSE_TO_IDLE_T;
    /*InterpreterControl ctrl;*/
    //ctrl.cmd = ABORT;
    /*ShareMem::instance()->intprtControl(ctrl);*/
    motionClear();
    usleep(20*1000);
    Instruction inst;
    ShareMem::instance()->getInstruction(inst); //read out all instructions
}

ErrorCode AutoMotion::moveTarget(MotionTarget target)
{
    ErrorCode err = SUCCESS;
    err = arm_group_->autoMove(target, 0);
    mot_target_ = target;
    is_done_ = false;
	return err;
}
