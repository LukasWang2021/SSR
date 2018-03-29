/**
 * @file controller.cpp
 * @brief
 * @author WangWei
 * @version 1.0.0
 * @date 2017-07-14
 */
#include <stdio.h>
#include "ctrl_func.h"
#include "io_interface.h"
#include "error_monitor.h"
#include "error_code.h"
#include "sub_functions.h"				
#include "service_heartbeat.h"

using std::vector;


Controller* Controller::instance_ = NULL;
Controller::Controller()
{
    U64 result;
    ctrl_state_ = ESTOP_S;
    debug_ready_ = false;

    memset(&servo_joints_, 0.000, sizeof(servo_joints_));
    
    arm_group_ = new fst_controller::ArmGroup();
    if ((result = arm_group_->initArmGroup()) != FST_SUCCESS)
    {
        FST_ERROR("init arm_group_ failed");
        rcs::Error::instance()->add(result);
    }
    robot_ = new Robot(arm_group_);
    manu_motion_ = new ManualMotion(robot_, arm_group_);
    auto_motion_ = new AutoMotion(arm_group_);
    inst_parser_ = new InstructionParser(arm_group_);

    Transformation tool_frame;
	memset(&tool_frame, 0, sizeof(tool_frame));
	arm_group_->setToolFrame(tool_frame);  

    tp_interface_ = new TPInterface();

    ctrl_task_ = new rcs::Task(STATE_MACHINE_INTERVAL);
    ctrl_task_->function(std::bind(&Controller::stateMachine, this, (void*)NULL));
    ctrl_task_->run();
    
    rt_traj_task_ = new rcs::Task(TRAJ_FLOW_INTERVAL, 80, true); 
    rt_traj_task_->function(std::bind(&Controller::rtTrajFlow, this, (void*)NULL));
    rt_traj_task_->run();

    heartbeat_task_ = new rcs::Task(HEART_BEAT_INTERVAL);
    heartbeat_task_->function(std::bind(&Controller::heartBeat, this, (void*)NULL));
    heartbeat_task_->run();

    instance_ = this;
}


Controller::~Controller()
{
    heartbeat_task_->stop();
    if (heartbeat_task_ != NULL)
        delete heartbeat_task_;
    rt_traj_task_->stop();
    if (rt_traj_task_ != NULL)
        delete rt_traj_task_;
    tp_interface_->destroy();
    ctrl_task_->stop();
    if (ctrl_task_ != NULL)
        delete ctrl_task_;    
    if (tp_interface_ != NULL)
        delete tp_interface_;
    if (inst_parser_ != NULL)
        delete inst_parser_;
    if (manu_motion_)
        delete manu_motion_;
    if (robot_ != NULL)
        delete robot_;
    if (arm_group_ != NULL)
        delete arm_group_;
}

void Controller::setError(void* params, int len)
{    
}
void Controller::getError(void* params)
{
    tp_interface_->setReply(BaseTypes_StatusCode_FAILED);    
}

void Controller::updateDefault(int id)
{
    //no actions in this function
}

void Controller::getWarnings(void* params)
{
    string err_str = rcs::Error::instance()->getErrorBytes();
    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData(err_str.c_str(), err_str.size());
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = err_str.size();
        //FST_INFO("error size:%d", param->size);
        memcpy(param->bytes, err_str.c_str(), param->size);
        
    }
}
void Controller::updateWarnings(int id)
{
    if (!rcs::Error::instance()->updated())
        return; 
    int warning_level = rcs::Error::instance()->getWarningLevel();
    
    if (warning_level > 4)
    {
        FST_INFO("in estop process...");
        safetyStop(warning_level);
        servoEStop();
        pauseMotion(); //set mode to pause
        setLogicStateCmd(EMERGENCY_STOP_E);
    }
    else if (warning_level > 2)
    {
        FST_INFO("in pause process...");
        pauseMotion();
        setLogicStateCmd(EMERGENCY_STOP_E);
    }
    setUpdateFlagByID(id, true);
}

void Controller::getCurveMode(void* params)
{
    //FST_INFO("controller:===this is getting curve mode===");
    //!!!qj!!! ArmGroup not support any more.
    int curve_mode = -1;//(int)arm_group_->getCurveMode();
    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)&curve_mode, sizeof(curve_mode));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(curve_mode);
        memcpy(param->bytes, (char*)&curve_mode, param->size);
    }
}
void Controller::updateCurveMode(int id)
{
    //static pre_curv_mode;
}

//20180313: qianjin add begin
void Controller:: getRunningMode(void* params)
{
    int rep_run_mode = (int)run_mode_;

    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)&rep_run_mode, sizeof(rep_run_mode));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(rep_run_mode);
        memcpy(param->bytes, (char*)&rep_run_mode, param->size);
    }

}
void Controller:: getServoState(void* params)
{
    int rep_servo_state = (int)ShareMem::instance()->getServoState();

    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)&rep_servo_state, sizeof(rep_servo_state));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(rep_servo_state);
        memcpy(param->bytes, (char*)&rep_servo_state, param->size);
    }
}

// call back for running mode
void  Controller:: updateRunningMode(int id)
{
}
// call back for servo state
void  Controller:: updateServoState(int id)
{
}

//20180313: qianjin add end


void Controller::getWorkStatus(void* params)
{
    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)&work_status_, sizeof(work_status_));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(work_status_);
        memcpy(param->bytes, (char*)&work_status_, param->size);
    }
}
void Controller::updateWorkStatus(int id)
{
    static WorkStatus prev_work_status;
    static int idle2exe_cnt = 0;
    static int pause_cnt = 0;
    
    WorkStatus status = work_status_;
    switch (status)
    {
        case IDLE_W:
        {
            // judge state of program interpreter 
            // if the state changed to exe??????
            // ====================================
            InterpreterState state = ShareMem::instance()->getIntprtState();
            if (EXECUTE_R == state)
            {
                U64 result = arm_group_->setStartState(servo_joints_);
                if (result != TPI_SUCCESS)
                {
                    rcs::Error::instance()->add(result);
                }
                work_status_ = IDLE_TO_RUNNING_T;
            }
            else if (PAUSED_R == state || WAITING_R == state  )
            {
                static const int max_count = MAX_TIME_IN_PUASE / STATE_MACHINE_INTERVAL;            
                if (ctrl_state_ == ENGAGED_S)
                {
                    pause_cnt++;
                    if (pause_cnt >= max_count) //wait
                    {
                        pause_cnt = 0;
                        ctrl_state_ = ESTOP_S;
                        // then we need to return this info
                    }
                }
                else
                {
                    pause_cnt = 0;
                }
            }
            break;
        }
        case IDLE_TO_TEACHING_T:
            pause_cnt = 0;
            break;
        case IDLE_TO_RUNNING_T:  
            pause_cnt = 0;
            if (idle2exe_cnt++ >= 2)    //run twice (delay 20ms)
            {
                idle2exe_cnt = 0;
                FST_INFO("IDLE TO RUNNING...");
                work_status_ = RUNNING_W;
            }
            if (rcs::Error::instance()->getWarningLevel() >= 4) 
            {
                abortMotion(); //abort the planned motion 
                work_status_ = IDLE_W;
            }
            break;
        case RUNNING_TO_IDLE_T:
            if ((intprt_state_ != IDLE_R)
            && (intprt_state_ != WAITING_R))
                break;
        case TEACHING_TO_IDLE_T:        
            //FST_INFO("servo state:%d", getServoState());
            if ((ShareMem::instance()->getServoState() == STATE_READY) 
            || (ShareMem::instance()->getServoState() == STATE_ERROR))
            {
                work_status_ = IDLE_W;
            }            
            break;
        case RUNNING_W:
        {
            int i;
            InterpreterState state = ShareMem::instance()->getIntprtState();
            i = state;
            printf("check intprtState=%d\n",i);
            if (state == IDLE_R)
            {
                FST_INFO("EXECUTE_TO_idle");
                //work_status_ = IDLE_W;
                work_status_ = RUNNING_TO_IDLE_T;
            }
            else if (state == PAUSED_R ||  state == WAITING_R)
            {
                FST_INFO("RUNNING_TO_IDLE_T");
                work_status_ = RUNNING_TO_IDLE_T;
            }
            break;
        }
        default:
            break;
    }//end switch (prgm_state)

 
    if (prev_work_status != work_status_)
    {
        prev_work_status = work_status_;
        setUpdateFlagByID(id, true);
    }
} 


void Controller::getInterpreterState(void* params)
{
    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)&intprt_state_, sizeof(intprt_state_));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(intprt_state_);
        memcpy(param->bytes, (char*)&intprt_state_, param->size);
    }
}
void Controller::updateInterpreterState(int id)
{
    static InterpreterState prev_state;    
    intprt_state_ = ShareMem::instance()->getIntprtState();
    if (prev_state == intprt_state_)
        return;
    prev_state = intprt_state_;
    setUpdateFlagByID(id, true);
}
void Controller::getCtrlState(void* params)
{
    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)&ctrl_state_, sizeof(RobotState));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(RobotState);
        memcpy(param->bytes, (char*)&ctrl_state_, param->size);
    }
}
void Controller::updateCtrlState(int id)
{
    static RobotState pre_state;
    RobotState state = ctrl_state_;
    int i,j;
    
    i= ctrl_state_;
    switch (state)
    {
        case TO_ESTOP_T:
        {
           // FST_INFO("cur ProgramState:%d", ps);
            if (recordJoints())
            {
                ctrl_state_ = ESTOP_S;
            }
            break;
        }
        case RESET_ESTOP_T:
        {
            //===wait RESET_ERROR_DELAY ms=========================
            static int count = RESET_ERROR_TIMEOUT / STATE_MACHINE_INTERVAL;
            //FST_INFO("the count is :%d", count);
            if (count-- > 0) //hasn't arrived timeout, during wait
            {
                //===check if error exist again====
                //if error accurs again, return ESTOP_S==
                if (rcs::Error::instance()->updated() 
                || rcs::Error::instance()->isInitError())
                {   
                    int i;
                    if(rcs::Error::instance()->isInitError()) {
                        i=1;
                    }else{
                        i=0;
                    }
                    //!!!qj!!!
                    FST_ERROR("updateCtrlState: estop to off: New error found while reset!");  
                    FST_ERROR("updateCtrlState: isInitError:%d",i );
                    count = 0;
                }
                else
                {
                    //=====servo hasn't ready========
                    //===need keep on wait============
                    if (ShareMem::instance()->getServoState() != STATE_READY)     
                        break;                
                    if (safety_interface_.getDIAlarm())
                        break;
                }
            }// if (count-- > 0)

            if (count <= 0)
            {
                FST_ERROR("updateCtrlState: estop to off: error: reset timeout");            
                serv_jtac_.stopBareMetal();
                ctrl_state_ = ESTOP_S;
            }            
            else
            {
                ctrl_state_ = ENGAGED_S;
            }
            //====set the count to default===
            count = RESET_ERROR_TIMEOUT / STATE_MACHINE_INTERVAL;
            break;
        }
        case TERMINATING_T:
        {
            if (recordJoints())
            {
                ctrl_state_ = TERMINATED_S;
            }
            break;
        }
        default:
            break;
    }

    if (pre_state != state)
    {
        pre_state = state;
        setUpdateFlagByID(id, true);
    }
    
    j= ctrl_state_;

    if(i!=j) {
        FST_INFO("Controller::updateCtrlState: updateCtrlState udpated:old->new is:%d->%d",i,j);            
    }
 
}

void Controller::setUserOpMode(void* params, int len)
{
     user_op_mode_  = *(UserOpMode*)params;
     ShareMem::instance()->setUserOpMode(user_op_mode_);
}

//qianjin change from setUserOpMode 
void Controller::getUserOpMode(void* params)
{


    int opmode = safety_interface_.getDITPUserMode();

    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)&opmode, sizeof(opmode));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(opmode);
        memcpy(param->bytes, (char*)&opmode, param->size);
    }



}
/*void Controller::setMotionModeCmd(void* params, int len)*/
//{
    //MotionModeCmd mode_cmd = *(MotionModeCmd*)params;
    //ProgramState state = auto_motion_->getPrgmState();
    //if ((state != IDLE_R) && (state != PAUSED_R))
    //{
        //FST_ERROR("cant change mode!!");
        //rcs::Error::instance()->add(SET_MODE_FAILED);
        //return;
    //}

    //FST_INFO("mode_cmd:%d", mode_cmd);
	//switch (mode_cmd)
	//{
		//case GOTO_PROGRAM_EXE_E:
            //motion_mode_ = PROGRAM_EXE_M;
			//break;
        //case GOTO_TEACH_E:

            //motion_mode_ = TEACH_M;
		//default:
            //rcs::Error::instance()->add(SET_MODE_FAILED);
			//break;
	//}//end switch (mode_cmd)

	//FST_INFO("current mode:%d", motion_mode_.load());

/*}*/

void Controller::setStateCmd(void* params, int len)
{
    RobotStateCmd state_cmd = *(RobotStateCmd*)params;
    if (state_cmd == EMERGENCY_STOP_E)
    {
        servoEStop();
        pauseMotion(); //set mode to pause 
    }
    setLogicStateCmd(state_cmd);
}
void Controller::getCurJoints(void* params)
{
    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)&servo_joints_, MAX_JOINTS*sizeof(double));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = MAX_JOINTS*sizeof(double);
        memcpy(param->bytes, (char*)&servo_joints_, param->size);
    }
}
void Controller::updateCurJoints(int id)
{
    U64 result = ShareMem::instance()->getFeedbackJoint(servo_joints_);
    if (result != TPI_SUCCESS)
    {
        rcs::Error::instance()->add(result);
        return;
    }

    setUpdateFlagByID(id, true);
}
void Controller::getTCPPose(void* params)
{
    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)robot_->getTCPPosePtr(), sizeof(PoseEuler));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(PoseEuler);
        memcpy(param->bytes, (char*)robot_->getTCPPosePtr(), param->size);
    }
}
void Controller::updateTCPPose(int id)
{
    if (ShareMem::instance()->isJointUpdated())
    {
        //FST_INFO("updated");
        robot_->updatePose(servo_joints_);
        setUpdateFlagByID(id, true);
    }
}
void Controller::getFlangePose(void* params)
{
    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)robot_->getFlangePosePtr(servo_joints_), sizeof(PoseEuler));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(PoseEuler);
        memcpy(param->bytes, (char*)robot_->getFlangePosePtr(servo_joints_), param->size);
    }
}

void Controller::updateFlangePose(int id)
{
    /*if (ShareMem::instance()->isJointUpdated())*/
    //{
        //robot_->updatePose(servo_joints_);
        //setUpdateFlagByID(id, true);
    /*}*/
}

/*void Controller::setJointTraj(void* params, int len)*/
//{
    
//}
//void Controller::setToolCoord(void* params, int len)
//{
/*}*/
void Controller::setToolFrame(void* params, int len)
{
    motion_spec_toolFrame *tool_frame = (motion_spec_toolFrame*)params;
    robot_->setToolFrame(tool_frame);
}
void Controller::setUserFrame(void* params, int len)
{
    motion_spec_userFrame *user_frame = (motion_spec_userFrame*)params;
    robot_->setUserFrame(user_frame);
}
void Controller::getToolFrame(void* params)
{
    motion_spec_toolFrame tool_frame = robot_->getToolFrame();
    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)&tool_frame, sizeof(tool_frame));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(tool_frame);
        memcpy(param->bytes, (char*)&tool_frame, param->size);
    }
}
void Controller::getUserFrame(void* params)
{
    motion_spec_userFrame user_frame = robot_->getUserFrame();
    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)&user_frame, sizeof(user_frame));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(user_frame);
        memcpy(param->bytes, (char*)&user_frame, param->size);
    }
}
void Controller::getLineID(void* params)
{
    int line = ShareMem::instance()->getCurLine();
    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)&line, sizeof(line));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(line);
        memcpy(param->bytes, (char*)&line, param->size);
    }

}

void Controller::updateLineID(int id)
{

}

void Controller::setUserRegs(void* params, int len)
{

}
void Controller::getUserRegs(void* params)
{

}

void Controller::updateUserRegs(int id)
{
    
}

void Controller::startRun(void* params, int len)
{
    if ((ctrl_state_ != ENGAGED_S) || (work_status_ != IDLE_W))
    {
        FST_ERROR("cant start run!!");
        rcs::Error::instance()->add(INVALID_ACTION_IN_CURRENT_STATE);
        return;
    }
    StartCtrl* start = (StartCtrl*)params;

    
    InterpreterControl ctrl;
    ctrl.cmd = START;
    ctrl.start_ctrl = *start;
    //start_mode_ = start->mode;
    ShareMem::instance()->intprtControl(ctrl);
    //permit sending command
    ShareMem::instance()->setIntprtSendFlag(true);
}

void Controller::startDebug(void* params, int len)
{
    int t1,t2;

    if ((ctrl_state_ != ENGAGED_S) || (work_status_ != IDLE_W))
    {
        FST_ERROR("cant debug run!!");
        t1 = ctrl_state_;
        t2 = work_status_;
        printf("INVALID_ACTION_IN_CURRENT_STATE: ctrl_state is: %d; work_status_ is: %d", t1, t2);
        rcs::Error::instance()->add(INVALID_ACTION_IN_CURRENT_STATE);
        return;
    }
    if (user_op_mode_ == AUTO_MODE_U)
    {
        FST_ERROR("cant debug run!!");
        t1 = user_op_mode_;
        printf("user_op_mode_ is: %d", t1);
        rcs::Error::instance()->add(INVALID_ACTION_IN_CURRENT_STATE);
        return;
    }

    auto_motion_->setDoneFlag(true);
    StartCtrl* start = (StartCtrl*)params;

    InterpreterControl ctrl;
    ctrl.cmd = DEBUG;
    ctrl.start_ctrl = *start;
    //start_mode_ = start->mode;
    ShareMem::instance()->intprtControl(ctrl);
    //permit sending command
    ShareMem::instance()->setIntprtSendFlag(true);
}


void Controller::jumpLine(void* params, int len)
{
    if ((ctrl_state_ != ENGAGED_S) || (work_status_ != IDLE_W) /*|| (!debug_ready_)*/)
    {
        int sta_c, sta_w;
        sta_c = ctrl_state_;
        sta_w = work_status_;

        
        FST_ERROR("jumpLine: cant jump line when ctrl_state = %d && work_status = %d!!", sta_c,sta_w);
        rcs::Error::instance()->add(INVALID_ACTION_IN_CURRENT_STATE);
        return;
    }
    if (user_op_mode_ == AUTO_MODE_U)
    {
        FST_ERROR("cant jumpline in auto mode!!");
        rcs::Error::instance()->add(INVALID_ACTION_IN_CURRENT_STATE);
        return;
    }
    //first abort motion
    //if (state == PAUSED_R)
    abortMotion();
    //----------------
    int line = *(int*)params;
    InterpreterControl ctrl;
    ctrl.cmd = JUMP;
    ctrl.id = line;
    ShareMem::instance()->intprtControl(ctrl);
}
void Controller::step(void* params, int len)
{
    if ((ctrl_state_ != ENGAGED_S)/* || (work_status_ != IDLE_W) || (!debug_ready_)*/)
    {
        FST_ERROR("cant step!!");
        rcs::Error::instance()->add(INVALID_ACTION_IN_CURRENT_STATE);
        return;
    }
    if (user_op_mode_ == AUTO_MODE_U)
    {
        FST_ERROR("cant step in auto mode!!");
        rcs::Error::instance()->add(INVALID_ACTION_IN_CURRENT_STATE);
        return;
    }
    
    if (resumeMotion())
    {
        work_status_ = RUNNING_TO_IDLE_T;
        return;
    }
    //   debug_ready_ = true;    //set debug flag

    InterpreterControl ctrl;
    ctrl.cmd = FORWARD;
    ShareMem::instance()->intprtControl(ctrl);
}
void Controller::backward(void* params, int len)
{
   // ProgramState state = auto_motion_->getPrgmState();
    if ((ctrl_state_ != ENGAGED_S)/* || (work_status_ != IDLE_W) || (!debug_ready_)*/)
    {
        FST_ERROR("cant backward!!");
        rcs::Error::instance()->add(INVALID_ACTION_IN_CURRENT_STATE);
        return;
    }
    if (user_op_mode_ == AUTO_MODE_U)
    {
        FST_ERROR("cant backward in auto mode!!");
        rcs::Error::instance()->add(INVALID_ACTION_IN_CURRENT_STATE);
        return;
    }
    
    if (resumeMotion())
    {
        work_status_ = RUNNING_TO_IDLE_T;
        return;
    }
   
    // debug_ready_ = true;    //set debug flag

    InterpreterControl ctrl;
    ctrl.cmd = BACKWARD;
    ShareMem::instance()->intprtControl(ctrl);
}



void Controller::getSafetyTPManual(void* params)
{
    char di = safety_interface_.getDITPManual();
    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)&di, sizeof(di));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(di);
        memcpy(param->bytes, (char*)&di, param->size);
    }
}
void Controller::getSafetyTPAuto(void* params)
{
    char di = safety_interface_.getDITPAuto();
    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)&di, sizeof(di));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(di);
        memcpy(param->bytes, (char*)&di, param->size);
    }
}
void Controller::getIOInfo(void* params)
{
    motion_spec_DeviceList dev_list;
    IOInterface::instance()->getIODevices(dev_list);
    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        FST_INFO("dev size:%d", sizeof(dev_list));
        rep->fillData((char*)&dev_list, sizeof(dev_list));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(dev_list);
        memcpy(param->bytes, (char*)&dev_list, param->size);
    }
}
void Controller::getFK(void* params){}
void Controller::getIK(void* params){}
void Controller::getLocalTime(void* params)
{
    long time = getCurTimeSecond();
    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)&time, sizeof(time));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(time);
        memcpy(param->bytes, (char*)&time, param->size);
    }
}
void Controller::getSoftLimit(void* params)
{
    motion_spec_JointConstraint slmt =  robot_->getSoftConstraint();
    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)&slmt, sizeof(slmt));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(slmt);
        memcpy(param->bytes, (char*)&slmt, param->size);
    }
}
void Controller::setJointConstraint(void* params, int len)
{
    motion_spec_JointConstraint slmt = *(motion_spec_JointConstraint*)params;
}
void Controller::getDH(void* params)
{
    motion_spec_DHGroup dh = robot_->getDHGroup();
    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)&dh, sizeof(dh));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(dh);
        memcpy(param->bytes, (char*)&dh, param->size);
    }
}
void Controller::getHardLimit(void* params)
{
    motion_spec_JointConstraint jc = robot_->getHardConstraint();
    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)&jc, sizeof(jc));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(jc);
        memcpy(param->bytes, (char*)&jc, param->size);
    }
}
void Controller::getGlobalVel(void* params)
{    
    double factor = arm_group_->getGlobalVelRatio();

    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)&factor, sizeof(factor));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(factor);
        memcpy(param->bytes, (char*)&factor, param->size);
    }
}
void Controller::setGlobalVel(void* params, int len)
{
    double factor = *(double*)params;
    if (arm_group_->setGlobalVelRatio(factor) == false)
    {
        rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
    }
}

void Controller::getSafetyInFrame(void* params)
{    
    U32 frm_data = safety_interface_.getDIFrm2();

    TPIParamBuf *param_ptr = (TPIParamBuf*)params;
    if (param_ptr->type == REPLY)
    {
        TPIFRepData* rep = (TPIFRepData*)param_ptr->params;
        rep->fillData((char*)&frm_data, sizeof(frm_data));
    }
    else
    {
        motion_spec_Signal_param_t *param = (motion_spec_Signal_param_t*)param_ptr->params;
        param->size = sizeof(frm_data);
        memcpy(param->bytes, (char*)&frm_data, param->size);
    }
}

void Controller::updateSafetyFrame(int id)
{    
    if (safety_interface_.isDIFrmChanged())
    {
        setUpdateFlagByID(id, true);
    }
}


void Controller::setCtrlCmd(void* params, int len)
{
    RobotCtrlCmd cmd = *(RobotCtrlCmd*)params;
   // ProgramState prgm_state = auto_motion_->getPrgmState();
    switch (cmd)
    {
        case SHUTDOWN_CMD:
            break;
        case PAUSE_CMD:         
            if (work_status_ == IDLE_W)             
            {
                rcs::Error::instance()->add(INVALID_ACTION_IN_CURRENT_STATE);
                break;
            }                        
            pauseMotion();
            break;
        case CONTINUE_CMD:
            if (ctrl_state_ != ENGAGED_S)
            {
                rcs::Error::instance()->add(INVALID_ACTION_IN_CURRENT_STATE);
                break;
            }
            InterpreterControl ctrl;
            ctrl.cmd = CONTINUE;
            ShareMem::instance()->intprtControl(ctrl);

            work_status_ = IDLE_TO_RUNNING_T;
            //if (auto_motion_->getDoneFlag() == false)
            {
                resumeMotion();
            }
            break;
        case ABORT_CMD:
        {
            InterpreterControl ctrl;
            ctrl.cmd = ABORT;
            ShareMem::instance()->intprtControl(ctrl);
            abortMotion();
            break;
        }
        case CALIBRATE_CMD:
            if (ctrl_state_ != ESTOP_S)
            {
                rcs::Error::instance()->add(INVALID_ACTION_IN_CURRENT_STATE);
            }
            else
            {
                calibrate();
            }
            break;
        case SETTMPZERO_CMD:
            if (ctrl_state_ != ESTOP_S)
            {
                rcs::Error::instance()->add(INVALID_ACTION_IN_CURRENT_STATE);
            }
            else
            {
                setTempZero();
            }
            break;
        default:
            rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
            break;
    }//end switch (cmd)

    return;
 
}

void Controller::systemShutdown(void* params){}
void Controller::setLocalTime(void* params, int len)
{
    int time = *(int*)params;
    setTimeSecond(time);
}

void Controller::setManualCmd(void* params, int len)
{
     //command must be set in idle or pause state_cmd
     //Qianjin
    if ((ctrl_state_ != ENGAGED_S) ||( (work_status_ != IDLE_W)&&(work_status_ != TEACHING_W)     ))
    {
        FST_ERROR("cant manual run!!");
        rcs::Error::instance()->add(INVALID_ACTION_IN_CURRENT_STATE);
        return;
    }
    work_status_ = TEACHING_W;
    U64 result = arm_group_->setStartState(servo_joints_);
    if (result != TPI_SUCCESS)
    {
        rcs::Error::instance()->add(result);
    }
    motion_spec_ManualCommand command = *(motion_spec_ManualCommand*)params;
    manu_motion_->setManuCommand(command);
}
void Controller::setTeachTarget(void* params, int len)
{
    if ((ctrl_state_ != ENGAGED_S)
    || (work_status_ != TEACHING_W))
    {
        FST_ERROR("cant manual !!");
        rcs::Error::instance()->add(INVALID_ACTION_IN_CURRENT_STATE);
        return;
    }
    motion_spec_TeachTarget target = *(motion_spec_TeachTarget*)params;
    manu_motion_->setTeachTarget(target);
}

bool Controller::isTerminated()
{
   if (ctrl_state_ == TERMINATED_S)
       return true;

   return false;
}

void Controller::exit()
{
    Controller* ctrl = instance_;
    if (ctrl)
    {
        ctrl->setLogicStateCmd(GOTO_TERMINATED);//ready to terminate process
    }
}


void Controller::setLogicStateCmd(RobotStateCmd state_cmd)
{
    FST_INFO("setLogicStateCmd:aim_state:%d",state_cmd);
    RobotState state = ctrl_state_;
	switch (state_cmd)
	{
        case EMERGENCY_STOP_E:
            if (state == ESTOP_S)
            {
                break;
            }
            ctrl_state_ = TO_ESTOP_T;                        
            break; 
        case ACKNOWLEDGE_ERROR:
            if (state == ENGAGED_S)
            {
                break;
            }
            else if (state == ESTOP_S)
            {
                bool  res_t;
                ctrl_state_ = RESET_ESTOP_T;
                //first clear error list 
                rcs::Error::instance()->clear();
                //====reset core1==========
                res_t=serv_jtac_.resetBareMetal();
                if(res_t == false) {
                        FST_ERROR("Send serv_jtac_.resetBareMetal command failed!");   

                }
            
                //=======reset safety board==================
                safety_interface_.reset();  //reset safety board*/
                //==========reset ArmGroup=============
                //!!!!qj!!!!!No need any more. 
                //U64 result = 0;// arm_group_->resetArmGroup();
                //if (result != TPI_SUCCESS)
                //{
                //    if (result == CURRENT_JOINT_OUT_OF_CONSTRAINT)
                //    {
                //        run_mode_ = SOFTLIMITED_R;
                //    }
                //    else
                //    {
                //        FST_ERROR("reset ArmGroup failed:%llx", result);   
                //        rcs::Error::instance()->add(result);
                //        break;
                //    }
                //}                  
            }//end else if (state == ESTOP_S)
            else 
            {
                FST_ERROR("invalid action in state:%d", state);
                rcs::Error::instance()->add(SET_STATE_FAILED);
            }
            break;
        case GOTO_TERMINATED:
            servoEStop();   //disable servo
            pauseMotion(); //set mode to pause
            ctrl_state_ = TERMINATING_T;
        default:
            break;
    }//end switch (state_cmd)
    FST_INFO("current state:%d", ctrl_state_.load());

}

void Controller::stateMachine(void* params)
{
    Instruction inst;
    int fifo_len =  arm_group_->getFIFOLength();
    /*if (auto_motion_->getPrgmState() == EXECUTE_R)*/
    //{ 
        //FST_INFO("fifo_len:%d, smooth:%d, servo_state:%d", fifo_len, auto_motion_->getSmooth(), ShareMem::instance()->getServoState());
    /*}*/

    if (auto_motion_->getSmooth() >= 0) //must full speed mode
    {
        if (fifo_len < 50)
        {
            ShareMem::instance()->sendingPermitted();
            auto_motion_->setDoneFlag(true);
        }
    }
    else
    {
        //StartMode mode = auto_motion_->getStartMode();
       {
            if ((ShareMem::instance()->getServoState() == STATE_READY)
            && (fifo_len == 0))
            {
                if ((work_status_ == RUNNING_W) || (RUNNING_TO_IDLE_T))
                {
                    ShareMem::instance()->sendingPermitted();
                    auto_motion_->setDoneFlag(true);
                    if (debug_ready_ == false)
                        debug_ready_ = true;
                }
            }
        }
    }
    if ((work_status_ == IDLE_W)
    || (work_status_ == IDLE_TO_RUNNING_T)
    || (work_status_ == RUNNING_W))
    {
        if (ShareMem::instance()->getInstruction(inst))
        {
            printf("get instruction, line:%d\n", inst.line);
            auto_motion_->moveTarget(inst.target);
        }
    }
    static int count = 0;
    if (++count >= SM_INTERVAL_COUNT)
    {
        requestProc();
        updateProc();
        count = 0;
    }

    return;// NULL;
}

void Controller::rtTrajFlow(void* params)
{
    if (work_status_ == IDLE_W)
        return;// NULL;

    U64 result = ShareMem::instance()->setJointPositions();
    if (result != TPI_SUCCESS)
    {
        rcs::Error::instance()->add(result);
        return;// NULL;
    }//end if (share_mem_.isJointCommandWritten() == false)

    int joints_len = arm_group_->getFIFOLength();  
    if (joints_len <= 0)
        return;// NULL;

    //FST_INFO("joints fifo length:%d, traj_len:%d", joints_len, arm_group_->getPlannedPathFIFOLength());

    //!!!!!qj!!!!!!!update arguments!!!!!!
    //vector<fst_controller::JointPoint> joint_traj;
    vector<fst_controller::JointOutput> joint_traj;
    result = arm_group_->getPointFromFIFO(10,joint_traj);
    
    if (result != TPI_SUCCESS)
    {
       // FST_ERROR("=======pick point failed ==========");
        rcs::Error::instance()->add(result);
        return;// NULL;
    }

    int joints_in = joint_traj.size();
    //dbcount+=joints_in;
    //printDbLine("joints:", (double*)&joint_traj[joints_in-1].joint, 6);
    //joints_len = arm_group_->getTrajectoryFIFOLength();
   // FST_INFO("joints fifo length:%d", joints_in);
    JointCommand joint_command;
    joint_command.total_points = joints_in;
    printf("Qianjin: Send joint cmd%d!\n", joints_in);
    for (int i = 0; i < joints_in; i++)
    {
        joint_command.points[i].positions[0] = joint_traj[i].joint.j1;
        joint_command.points[i].positions[1] = joint_traj[i].joint.j2;
        joint_command.points[i].positions[2] = joint_traj[i].joint.j3;
        joint_command.points[i].positions[3] = joint_traj[i].joint.j4;
        joint_command.points[i].positions[4] = joint_traj[i].joint.j5;
        joint_command.points[i].positions[5] = joint_traj[i].joint.j6;

        //fillInFIFO2(joint_command.points[i].positions);
        
        joint_command.points[i].point_position = joint_traj[i].level; //point position: start\middle\ending
       //FST_INFO("level:%d", joint_traj[i].level); 
       // printDbLine("joints:", joint_command.points[i].positions, 6);
      //  if (ctrl_mode_ == AUTO_RUN_M)
        {
            
            /*int cur_id = inst_parser_->getCurrentCmdID();*/
            //int traj_id = joint_traj[i].id;
            //inst_parser_->updateID(traj_id);
            

            //int level = joint_command.points[i].point_position;
            //if (level == POINT_LAST)
            //{
                ////printDbLine("the last joints:", joint_command.points[i].positions, 6);                   
            //}
            //else if (level == POINT_ENDING)  
            //{
                ////if (prgm_state_ == EXECUTE_R) //in case for pausing
                //if (arm_group_->isTrajectoryTotallyFinished())
                //{                    
                    //inst_parser_->rmMoveInstruction();
                //}
                ////printDbLine("the end joints:", joint_command.points[i].positions, 6);
                         
               //// FST_INFO("traj_id is :%d", cur_id);
            //} 
            //else if (level == POINT_START)
            //{
                ////printDbLine("the start joints:", joint_command.points[i].positions, 6);
                ////FST_INFO("traj_id is :%d, cur_id:%d", traj_id, cur_id);
            //}
            //else if (level == POINT_FIRST)
            //{
                //if (traj_id != cur_id) //in case in middle of path
                //{
                    //inst_parser_->rmMoveInstruction();
                //}
                ////printDbLine("the first joints:", joint_command.points[i].positions, 6);
                ////FST_INFO("traj_id is :%d, cur_id:%d", traj_id, cur_id);
            /*}*/
        }
    }// end for (int i = 0; i < joints_in; i++)


    ShareMem::instance()->setCurrentJointCmd(joint_command); //store this command in case it can't write Success

        //return NULL;
}


void Controller::heartBeat(void* params)
{
    IOInterface::instance()->updateIOError(); 
    //ServiceHeartbeat::instance()->sendRequest();
    U64 result = safety_interface_.setSafetyHeartBeat();
    if (result != TPI_SUCCESS)
    {
        rcs::Error::instance()->add(result);
    }
}


void Controller::requestProc()
{
    uint32_t id;
    if (!tp_interface_->getReqDataPtr()->isFilled())
        return;
    switch (tp_interface_->getReqDataPtr()->getType())
    {
        case SET:
        {
            string str_path = (const char*)tp_interface_->getReqDataPtr()->getPathBufPtr();
            if (str_path.substr(0, 7) == "root/IO")
            {
                //int buf_len;             
                //U64 result = IOInterface::instance()->checkIO(str_path.c_str(), buf_len, id);
                IOPortInfo info;
                U64 result = IOInterface::instance()->checkIO(str_path.c_str(), &info);
                if (result != TPI_SUCCESS)
                {
                    rcs::Error::instance()->add(result);
                    tp_interface_->setReply(BaseTypes_StatusCode_FAILED);
                    break;
                }
                char val = *tp_interface_->getReqDataPtr()->getParamBufPtr();
                IOInterface::instance()->setDO(&info, val);
            }
            else
            {
                id = tp_interface_->getReqDataPtr()->getID();
                //FST_INFO("request id:%d", id);
                void* pointer = tp_interface_->getReqDataPtr()->getParamBufPtr();
                int len = tp_interface_->getReqDataPtr()->getParamLen();
                (this->*g_ctrl_funcs_mp[id].setValue)(pointer, len);
            }
            tp_interface_->setReply(BaseTypes_StatusCode_OK);
            break;
        }
        case GET:
        {
            string str_path = (const char*)tp_interface_->getReqDataPtr()->getPathBufPtr();
            if (str_path.substr(0, 7) == "root/IO")
            {
                IOPortInfo info;
                U64 result = IOInterface::instance()->checkIO(str_path.c_str(), &info);

                if (result != TPI_SUCCESS)
                {
                    rcs::Error::instance()->add(result);
                    tp_interface_->setReply(BaseTypes_StatusCode_FAILED);
                    break;
                }

                IOInterface::instance()->getDIO(&info, 
                        tp_interface_->getRepDataPtr()->getParamBufPtr(),
                        tp_interface_->getRepDataPtr()->getParamBufLen());
                tp_interface_->getRepDataPtr()->setParamLen(info.bytes_len);
            }
            else
            {
                id = tp_interface_->getReqDataPtr()->getID();           
                //FST_INFO("request id:%d", id);
                TPIParamBuf param_buf;
                param_buf.type = REPLY;
                param_buf.params = tp_interface_->getRepDataPtr();
                (this->*g_ctrl_funcs_mp[id].getValue)(&param_buf);
            }
            tp_interface_->setReply(PARAM, id);
            break;
        }
        case CMD:
            parseCmdMsg();
            break;
        default:
            break;
    }

    tp_interface_->getReqDataPtr()->setFilledFlag(false);
}

void Controller::updateProc()
{
    std::map<int, PublishUpdate>::iterator itr;
    std::map<int, CtrlFunctions>::iterator it = g_ctrl_funcs_mp.begin();
    for (; it != g_ctrl_funcs_mp.end(); ++it)
    {
        (this->*it->second.updateValue)(it->first);
    }

    if (tp_interface_->getPubDataPtr()->isFilled())        
        return;
    motion_spec_SignalGroup *sig_gp = tp_interface_->getPubDataPtr()->getParamBufPtr();
    sig_gp->sig_param_count = 0;

    int base_property_count = 0;
    itr = id_pub_map_.begin();
    for (; itr != id_pub_map_.end(); ++itr)
    {
        int id = itr->first;        
        PublishUpdate *pub_update = &itr->second;
        motion_spec_Signal *sig = &sig_gp->sig_param[sig_gp->sig_param_count];
        if (id >= IO_BASE_ADDRESS)
        {               
            sig_gp->sig_param_count++;
            sig->id = id;
            IOInterface::instance()->getDIO(id, sig->param.bytes, sizeof(sig->param.bytes), pub_update->buf_len); 
        }
        else
        {
            if (pub_update->count++ >= pub_update->base_interval)
            {
                if ((pub_update->count >= pub_update->max_interval)
                || (pub_update->update_flag == true))
                {
                    base_property_count++;
                    pub_update->count = 0;
                    sig_gp->sig_param_count++;
                    sig->id = id;
                    //FST_INFO("pub id:%d", id);
                    TPIParamBuf param_buf;
                    param_buf.type = PUBLISH;
                    param_buf.params = (void*)&sig->param;
                    (this->*g_ctrl_funcs_mp[itr->first].getValue)(&param_buf);
                    //FST_INFO("param len:%d", sig->param.size);
                    pub_update->update_flag = false;
                }
            }
        }
    }
    if (sig_gp->sig_param_count > 0)
    {
        //FST_INFO("signal count :%d", sig_gp->sig_param_count);
        if (base_property_count > 0)
        {
            for (itr = id_pub_map_.begin(); itr != id_pub_map_.end(); ++itr)
            {
                itr->second.count = 0;
            }
        }
        tp_interface_->getPubDataPtr()->setFilledFlag(true);
    }
}

void Controller::parseCmdMsg()
{
    CmdParams cmd_param;
    memcpy((void*)&cmd_param, tp_interface_->getReqDataPtr()->getParamBufPtr(), sizeof(cmd_param));
    switch (cmd_param.cmd)
	{
		case BaseTypes_CommandType_LIST:
		{
			FST_INFO("list...\n");
            tp_interface_->setReply(LIST);
			break;
		}			
        case BaseTypes_CommandType_ADD:
        {
            
            string str_path = (const char*)tp_interface_->getReqDataPtr()->getPathBufPtr();            
            PublishUpdate *pub_update = &cmd_param.pub_update;
            FST_INFO("add path:%s, freq:%d,%d", str_path.c_str(), pub_update->base_interval, pub_update->max_interval);
            addPubParameter(str_path, pub_update);
            //BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;
            tp_interface_->setReply(BaseTypes_StatusCode_OK, tp_interface_->getReqDataPtr()->getID());
            break;
        }
		case BaseTypes_CommandType_REMOVE:
		{
            FST_INFO("remove...");
            string str_path = (const char*)tp_interface_->getReqDataPtr()->getPathBufPtr();
			removePubParameter(str_path);
            //BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;
            tp_interface_->setReply(BaseTypes_StatusCode_OK);
            
            break;
		} 
        case BaseTypes_CommandType_REMOVE_ALL:
        {
            FST_INFO("remove all...");
            removeAllPubParams();
            //BaseTypes_StatusCode status_code = BaseTypes_StatusCode_OK;
            tp_interface_->setReply(BaseTypes_StatusCode_OK);
            break;
        } 
        default:
        {
            FST_ERROR("ERROR command:%d", cmd_param.cmd);
            rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
            //BaseTypes_StatusCode status_code = BaseTypes_StatusCode_FAILED;
            tp_interface_->setReply(BaseTypes_StatusCode_FAILED);
            break;
        }
	}//end switch (cmd_param.cmd)
}

void Controller::addPubParameter(string str_path, PublishUpdate *pub_update)
{
    uint32_t id;
    std::map<int, PublishUpdate>::iterator it = id_pub_map_.find(id);
    if (it != id_pub_map_.end())
    {
        it->second.base_interval = pub_update->base_interval / ctrl_task_->period();
        it->second.max_interval = pub_update->max_interval / ctrl_task_->period();
        it->second.count = pub_update->max_interval;                
        it->second.update_flag = true;
        return;
    }

    if (str_path.substr(0, 7) == "root/IO")
    {
        //int buf_len;
        IOPortInfo info;
        U64 result = IOInterface::instance()->checkIO(str_path.c_str(), &info);
        if (result != TPI_SUCCESS)
        {
            rcs::Error::instance()->add(result);
            return;
        }
        id = info.msg_id;
    }
    else
    {
        pub_update->base_interval = pub_update->base_interval / ctrl_task_->period();
        pub_update->max_interval = pub_update->max_interval / ctrl_task_->period();
        pub_update->count = pub_update->max_interval;
        id = tp_interface_->getReqDataPtr()->getID();
    }
    id_pub_map_.insert(std::map<int, PublishUpdate>::value_type(id, *pub_update));
}

void Controller::removePubParameter(string str_path)
{
    uint32_t id;
    //FST_INFO("move path:%s", str_path.c_str());
    if (str_path.size() <= 8)
        return;
    if (str_path.substr(0, 7) == "root/IO")
    {
        IOPortInfo info;        
        U64 result = IOInterface::instance()->checkIO(str_path.c_str(), &info);
        if (result != TPI_SUCCESS)
        {
            rcs::Error::instance()->add(result);
            return;
        }
        id = info.msg_id;
    }
    else
    {
        id = tp_interface_->getReqDataPtr()->getID();        
    }
    std::map<int, PublishUpdate>::iterator it = id_pub_map_.find(id);
    id_pub_map_.erase(it);

}

void Controller::removeAllPubParams()
{
    id_pub_map_.erase(id_pub_map_.begin(), id_pub_map_.end());
}


void Controller::setUpdateFlagByID(int id, bool flag)
{
    std::map<int, PublishUpdate>::iterator it = id_pub_map_.find(id);
    if (it != id_pub_map_.end())
    {
        it->second.update_flag = flag;
    }    
}

#if 0
bool Controller::planFifo()
{
    /*U64 result = arm_group_->convertPathToTrajectory(MAX_PLANNED_POINTS_NUM);*/
    //if (result != TPI_SUCCESS)
    //{
        //rcs::Error::instance()->add(result);
    /*} */   
    if (ctrl_mode_ == MANUAL_MODE_M) 
    {
        manu_motion_->stepCounter();        
    }
    else if (ctrl_mode_ == AUTO_RUN_M)
    {
        //inst_parser_->runNMInstructions();
        /*U64 result = inst_parser_->pickInstructions();*/
        //if (result != TPI_SUCCESS)
        //{
            //rcs::Error::instance()->add(result);
            //return false;
        /*}*/
    }
    return true;
}


bool Controller::hasMoveCommand()
{
    if (ctrl_mode_ == MANUAL_MODE_M) 
    {
       return manu_motion_->isReady();
    }
    else if (ctrl_mode_ == AUTO_RUN_M)
    {
        // judge state of program interpreter 
        // if the state changed to exe??????
        // ====================================
        if (EXECUTE_R == ShareMem::instance()->getIntprtState())
        {
            U64 result = arm_group_->setStartState(servo_joints_);
            if (result != TPI_SUCCESS)
            {
                rcs::Error::instance()->add(result);
            }
            return true;
        }
        //return ret;
    }    

    return false;
}

bool Controller::hasTransformedToIdle()
{

    if (ctrl_mode_ == MANUAL_MODE_M) 
    {
        if ((ShareMem::instance()->getServoState() == STATE_READY)
        && isFifoEmpty()
        /*&& arm_group_->isTrajectoryTotallyFinished()*/)
        {
            return true;
        }
    }
    else if (ctrl_mode_ == AUTO_RUN_M)
    {
        if (ShareMem::instance()->getIntprtState() == IDLE_R)
            return true;
/*        if (ShareMem::instance()->isServoDone())*/
        //{
            ////==set servo wait ready flag to false========
            //return true; 
        //}
        //if ((ShareMem::instance()->getServoState() == STATE_READY)
        //&& isFifoEmpty()[> && 
        //(!inst_parser_->isInstructionExist())*/)
        //{
            //return true;
        /*}  */      
    }

    return false;
}
#endif
bool Controller::isFifoEmpty()
{
    //FST_INFO("the fifo size:%d, joints:%d", traj_len, joints_len);
    if (arm_group_->getFIFOLength() == 0)
    {
        return true;
    }

    return false;
}


void Controller::setRunningMode(RunningMode rm)
{
    if (rm == NORMAL_R)
    {
        return;
    }
    else
    {
        run_mode_ = (RunningMode)(run_mode_ | rm);
    }
}

void Controller::clearRunningMode(RunningMode rm)
{
    if (rm == NORMAL_R)
    {
        return;
    }
    else
    {
        run_mode_ = (RunningMode)(run_mode_ & (~rm));
    }
}

bool Controller::recordJoints()
{
    if (work_status_ != IDLE_W)
        return false;

    //==record current joints for calibrate==
    return calib_.recordCurrentJoint();
}

void Controller::abortMotion()
{

    FST_INFO("abortMotion...");
    U64 result = arm_group_->clearArmGroup();
    //=====clear all the fifos=======
    if (TPI_SUCCESS != result)
    {
        FST_ERROR("reset ArmGroup failed");
        rcs::Error::instance()->add(result);
    }
    ShareMem::instance()->setEmptyFlag(true); //don't write share_mem any more

    auto_motion_->abort();
}

void Controller::pauseMotion()
{    
    if (work_status_ == TEACHING_W)
        work_status_ = TEACHING_TO_IDLE_T;
    else if (work_status_ == RUNNING_W)
        work_status_ = RUNNING_TO_IDLE_T;
    else
    {
        work_status_ = IDLE_W;
        return;
    }
    InterpreterControl ctrl;
    ctrl.cmd = PAUSE;
    ShareMem::instance()->intprtControl(ctrl);

    auto_motion_->pause(); 
}

bool Controller::resumeMotion()
{
    if ( ( (intprt_state_ == PAUSED_R) ||  (intprt_state_ == WAITING_R)  )  && (auto_motion_->getDoneFlag() == false))
    {
        auto_motion_->resume();
        FST_INFO("fsssssssssssssssssss\n");
        return true;
    }

    FST_INFO("DONE FLAG:%d", auto_motion_->getDoneFlag());

    return false;
}

void Controller::calibrate()
{
    unsigned int caliborate_val;
    if (calib_.calibrateZeroOffset(caliborate_val))
    {
        clearRunningMode(CALIBRATE_R);        
    }
}

void Controller::setTempZero()
{
    if (false == calib_.setTempZeroOffset())
    {
        rcs::Error::instance()->add(FAILED_TO_SET_TEMP_ZERO);
        return;
    }

    setRunningMode(CALIBRATE_R);

    auto_motion_->abort();
    //set prgm_state_ to IDLE_R
    if (intprt_state_ == PAUSED_R)
    {
        InterpreterControl ctrl;
        ctrl.cmd = ABORT;
        ShareMem::instance()->intprtControl(ctrl);

    }

}

void Controller::safetyStop(int level)
{
    FST_INFO("safetyStop ...");
    switch (level)
    {
        case 5:
        case 6:
            safety_interface_.setDOType1Stop(1);
            break;
        case 7:
            safety_interface_.setDOType0Stop(1);
            break;
        case 8:
        case 9:
            safety_interface_.setDOType2Stop(1);
            break;
        case 10:
            safety_interface_.setDOType0Stop(1);
            break;
        case 11:
            safety_interface_.setDOType0Stop(1);
            break;
        default:
            break;
    }
}

void Controller::servoEStop()
{    
    //stop bare metal, can't write fifo since
    if (serv_jtac_.stopBareMetal())
    {
        //if there is no point, then do not call declareEstop
        if (work_status_ == IDLE_W)
            //!!!!!!!!qj!update function argument!!!!!!!!!
            arm_group_->declareESTOP( servo_joints_);
    }
    ShareMem::instance()->setEmptyFlag(true);
    usleep(500*1000);   //wait for brake on, need to ask ??
}

void Controller::errorAction(int warning_level)
{
    FST_INFO("warning_level is:%d", warning_level);
    if (warning_level > 4)
    {
        FST_INFO("in estop process...");
        safetyStop(warning_level);
        servoEStop();
        pauseMotion(); //set mode to pause
        setLogicStateCmd(EMERGENCY_STOP_E);
    }
    else if (warning_level > 2)
    {
        FST_INFO("in pause process...");
        pauseMotion();
        setLogicStateCmd(EMERGENCY_STOP_E);
    }
}

void Controller::shutdown()
{

}

