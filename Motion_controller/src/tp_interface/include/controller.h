/**
 * @file controller.h
 * @brief 
 * @author wangwei
 * @version 1.0.0
 * @date 2017-07-14
 */
#ifndef RCS_CONTROLLER_H_
#define RCS_CONTROLLER_H_

#include <map>
#include <iostream>
#include <string.h>
#include <cstring>
#include <fstream>
#include <vector>
#include "motion_plan_arm_group.h"
#include "motion_plan_frame_manager.h"
#include "motion_plan_reuse.h"
#include "motion_plan_variable.h"
#include "tp_interface.h"
#include "proto_parse.h"
#include "share_mem.h"
#include "base.h"
#include "manual_motion.h"
#include "instruction_parser.h"
#include "auto_motion.h"
#include "robot.h"
#include "service_jtac.h"
#include "safety_interface.h"
#include "offset_calibrator.h"
#include "launch_code_mgr.h"

#define MAX_PLANNED_POINTS_NUM		(20)
#define IDLE2EXE_DELAY              (50)  //wait from idle to execute(ms)
#define MAX_TIME_IN_PUASE           (20*1000)  //(ms)
#define RESET_ERROR_TIMEOUT	        (5000)	//wait until to judge if errors are reset

using namespace std;
using namespace fst_controller;
using namespace fst_algorithm;

extern Matrix   fst_algorithm::g_user_frame;
extern Matrix   fst_algorithm::g_tool_frame;
extern Matrix   fst_algorithm::g_user_frame_inverse;
extern Matrix   fst_algorithm::g_tool_frame_inverse;

typedef enum _BufType
{
    REPLY,
    PUBLISH,
}BufType;

typedef struct _TPIParamBuf
{
    BufType type;
    void*   params;
}TPIParamBuf;

class Controller
{
  public:
    Controller();
    ~Controller();

    /**
     * @brief :callback for default set function
     *
     * @param params
     * @param len
     */
    void setError(void* params, int len);

    /**
     * @brief: callback for default get function 
     *
     * @param params
     */
    void getError(void* params);
    
    /**
     * @brief: callback for default update function
     *
     * @param id
     */
    void updateDefault(int id); 

    /**
     * @brief: callback function for get warning 
     *
     * @param params
     */
    void getWarnings(void* params);

    /**
     * @brief: callback function for update warning 
     *
     * @param id
     */
    void updateWarnings(int id);

    /**
     * @brief: callback function for getting curve mode 
     *
     * @param params
     */
    void getCurveMode(void* params);

    /**
     * @brief: callback function for updating curve mode 
     *
     * @param id
     */
    void updateCurveMode(int id);

    /**
     * @brief: callback function for getting program state 
     *
     * @param params
     */
    void getWorkStatus(void* params);

    /**
     * @brief: callback function for updating program state 
     *
     * @param id
     */
    void updateWorkStatus(int id);

    /**
     * @brief:callback function for getting teach status
     *
     * @param params
     */
//    void getTeachStatus(void* params);

    /**
     * @brief: callback function for updating teach status
     *
     * @param id
     */
  //  void updateTeachStatus(int id);

    /**
     * @brief: callback function for getting motion mode 
     *
     * @param params
     */
    void getInterpreterState(void* params);

    /**
     * @brief: callback function for updating motion mode 
     *
     * @param id
     */
    void updateInterpreterState(int id);

    /**
     * @brief: callback function for getting robot state 
     *
     * @param params
     */
    void getCtrlState(void* params);

//     * qj: 20180312 add function begin
    /**
     * @brief: callback function for updating running mode
     * @param id
     */
    void getRunningMode(void* params);

    /**
     * @brief: callback function for updating ServoState
     * @param id
     */
    void getServoState(void* params);


	// call back for running mode
	void  updateRunningMode(int id);
	// call back for servo state
	void  updateServoState(int id);


	//	   * qj: 20180312 add function end


    /**
     * @brief: callback function for updating robot state 
     *
     * @param id
     */

	
    void updateCtrlState(int id);

    /**
     * @brief: callback function for setting user operation mode 
     *
     * @param params
     * @param len
     */
    void setUserOpMode(void* params, int len);
    //qianjin change from setUserOpMode
    void getUserOpMode(void* params);

    /**
     * @brief: callback function for setting MotionModeCmd 
     *
     * @param params
     * @param len
     */
  //  void setMotionModeCmd(void* params, int len);

    /**
     * @brief: callback for setting RobotStateCmd 
     *
     * @param params
     * @param len
     */
    void setStateCmd(void* params, int len);

    /**
     * @brief: callback for getting current servo joints 
     *
     * @param params
     */
    void getCurJoints(void* params);

    /**
     * @brief: callback for updating current servo joints 
     *
     * @param id
     */
    void updateCurJoints(int id);

    /**
     * @brief:callback for getting current TCP pose 
     *
     * @param params
     */
    void getTCPPose(void* params);

    /**
     * @brief: callback for updating current TCP pose 
     *
     * @param id
     */
    void updateTCPPose(int id);

    /**
     * @brief: callback for getting current flange pose 
     *
     * @param params
     */
    void getFlangePose(void* params);

    /**
     * @brief: callback for updating current flange pose 
     *
     * @param id
     */
    void updateFlangePose(int id);
   
    /**
     * @brief: callback for getting program line id 
     *
     * @param params
     */
    void getLineID(void* params);

    /**
     * @brief: callback for updating program line id 
     *
     * @param id
     */
    void updateLineID(int id);

    /**
     * @brief: callback for setting user registers 
     *
     * @param params
     * @param len
     */
    void setUserRegs(void* params, int len);

    /**
     * @brief: callback for getting user registers 
     *
     * @param params
     */
    void getUserRegs(void* params);

    /**
     * @brief: callback for updating user registers 
     *
     * @param id
     */
    void updateUserRegs(int id);

    /**
     * @brief: callback for starting running 
     *
     * @param params
     * @param len
     */
    void startRun(void* params, int len);

    /**
     * @brief: callback for starting debugging 
     *
     * @param params
     * @param len
     */
    void startDebug(void* params, int len);

    /**
     * @brief: callback for jump line 
     *
     * @param params
     * @param len
     */
    void jumpLine(void* params, int len);

    /**
     * @brief: callback for stepping forward 
     *
     * @param params
     * @param len
     */
    void step(void* params, int len);

    /**
     * @brief: callback for stepping backward 
     *
     * @param params
     * @param len
     */
    void backward(void* params, int len);

    /**
     * @brief: callback for getting TP panel(will not use)
     *
     * @param params
     */
    void getSafetyTPManual(void* params);
    /**
     * @brief: callback for getting TP panel 
     *
     * @param params
     */
    void getSafetyTPAuto(void* params);

    /**
     * @brief: callback for getting IO infomation 
     *
     * @param params
     */
    void getIOInfo(void* params);

    /**
     * @brief: callback for getting forward kinematics 
     *
     * @param params
     */
    void getFK(void* params);
   
    /**
     * @brief: callback for getting inverse kinematics
     *
     * @param params
     */
    void getIK(void* params);

    /**
     * @brief: callback for getting local time 
     *
     * @param params
     */
    void getLocalTime(void* params);

    /**
     * @brief: callback for getting soft limit
     *
     * @param params
     */
    void getSoftLimit(void* params);

    /**
     * @brief: callback for getting soft Constraint limit
     *
     * @param params
     */
    void getSoftConstraintLimit(void* params);

    /**
     * @brief: callback for setting soft limit 
     *
     * @param params
     * @param len
     */
    void setJointConstraint(void* params, int len);

    /**
     * @brief: callback for getting DH parameters
     *
     * @param params
     */
    void getDH(void* params);

    /**
     * @brief: callback for getting hard limit 
     *
     * @param params
     */
    void getHardLimit(void* params);
    
    /**
     * @brief: callback for getting global velocity 
     *
     * @param params
     */
    void getGlobalVel(void* params);

    /**
     * @brief: callback for setting global velocity 
     *
     * @param params
     * @param len
     */
    void setGlobalVel(void* params, int len);
    
    /**
     * @brief: callback for getting safety frame data 
     *
     * @param params
     */
    void getSafetyInFrame(void* params);

    /**
     * @brief: callback for updating safety frame data 
     *
     * @param id
     */
    void updateSafetyFrame(int id);

    /**
     * @brief: callback for setting Controller command 
     *
     * @param params
     * @param len
     */
    void setCtrlCmd(void* params, int len);

    /**
     * @brief: callback for shutting down 
     *
     * @param params
     */
    void systemShutdown(void* params);

    /**
     * @brief: callback for setting local time 
     *
     * @param params
     * @param len
     */
    void setLocalTime(void* params, int len); 

    /**
     * @brief: callback for setting manual motion command 
     *
     * @param params
     * @param len
     */
    void setManualCmd(void* params, int len);

    /**
     * @brief: callback for setting teaching target 
     *
     * @param params
     * @param len
     */
    void setTeachTarget(void* params, int len);
   
    /**
     * @brief: judge if the controller has terminated 
     *
     * @return: true if the controller has terminated 
     */
    bool isTerminated();


    /**
     * @brief: system exit 
     */
    static void exit();
  
    /**
     * @brief: callback for setting user registers 
     *
     * @param params
     * @param len
     */
    void setIOStatus(char* params, char value);

    /**
     * @brief: try to get IO
     *
     * @param params
     * @param len
     */
    void sendGetIORequest(char* params, int len);

    /**
     * @brief: try to get dio
     *
     * @param params
     * @param len
     */
    // void sendGetIORequest(void* params, int len);	

    /**
     * @brief: callback for getting user registers 
     *
     * @param params
     */
    int getIOReply(void* params);
	
    /**
     * @brief: send IO Simulate Status Request  
     *
     * @param params
     * @param len
     */
    void sendIOSimulateStatusRequest(char* params, int len);
	
    /**
     * @brief: set IO Simulate Status
     *
     * @param params
     * @param len
     */
    void setIOSimulateStatus(char* params, char value);
	
    /**
     * @brief: set IO Simulate Value
     *
     * @param params
     * @param len
     */
    void setIOSimulateValue(char* params, char value);
    /**
     * @brief: callback for getting version data 
     *
     * @param params
     */
    void getVersion(void* params);

    /************* The Follow functions for tool frame **************/
    /**
     * @brief: callback for setting a ordirary tool frame
     *
     * @param params
     * @param len
     */
    void setToolFrame(void* params, int len);
    
    /**
     * @brief: callback for getting a ordirary tool frame 
     *
     * @param params
     */
    void getToolFrame(void* params);

    /**
     * @brief: callback for setting current tool frame
     *
     * @param params
     * @param len
     */
    void setActivateToolFrame(void* params, int len);

    /**
     * @brief: callback for getting current tool frame 
     *
     * @param params
     */
    void getActivateToolFrame(void* params);

    /************* The Follow functions for user frame **************/
    /**
     * @brief: callback for setting a ordirary user frame 
     *
     * @param params
     * @param len
     */
    void setUserFrame(void* params, int len);

    /**
     * @brief: callback for getting a ordirary user frame 
     *
     * @param params
     */
    void getUserFrame(void* params);

    /**
     * @brief: callback for setting current user frame 
     *
     * @param params
     * @param len
     */
    void setActivateUserFrame(void* params, int len);

    /**
     * @brief: callback for getting current user frame 
     *
     * @param params
     */
    void getActivateUserFrame(void* params);

    /************* The Follow function for string data ************/
    /*
     * @brief: callback for getting string data 
     *
     * @param params
     */
    void getString(void* params);

    /**
     * @brief: update string data 
     *
     * @param id
     */
    void updateString(int id);


    /************* The Follow function for register **************/
    /*
     * @brief: callback for setting register
     *
     * @param params
     * @param len
     */
    void setRegister(void* params, int len);

    /**
     * @brief: try to get Register
     *
     * @param params
     * @param len
     */
    void sendGetRegisterRequest(void* params, int len);

    /**
     * @brief: callback for getting register 
     *
     * @param params
     */
    int getRegisterReply(void* params);

    /**
     * @brief: callback for getting register 
     *
     * @param params
     */
    void getRegister(void* params);

    /**
     * @brief: Judge whether there is a index error for setting
     */
    bool isRegisterIndexError(int &send_index, int &reg_total);

    /**
     * @brief: callback for setting pose register : PR
     *
     * @param params
     * @param len
     */
    void setPoseRegister(void* params, int len);

    /**
     * @brief: callback for getting pose register : PR
     *
     * @param params
     */
    void getPoseRegister(void* params);

   /**
     * @brief: callback for setting number register: R
     *
     * @param params
     * @param len
     */
    void setNumberRegister(void* params, int len);

    /**
     * @brief: callback for getting number register: R
     *
     * @param params
     */
    void getNumberRegister(void* params);

    /**
     * @brief: callback for setting global Acc
     *
     * @param params
     * @param len
     */
    void setGlobalAcc(void* params, int len);

    /**
     * @brief: callback for getting global Acc
     *
     * @param params
     */
    void getGlobalAcc(void* params);
    
    /**
     * @brief: try to get Register
     *
     * @param params
     * @param len
     */
    void sendGetChangeRegListRequest(InterpreterCommand cmd, void* params, int len);

    /**
     * @brief: callback for getting register 
     *
     * @param params
     */
    int getChangeRegListReply(void* params);

    /**
     * @brief: callback for getting register 
     *
     * @param params
     */
    void getChangeRegList(InterpreterCommand cmd, void* params);


    /**
     * @brief: callback for getting user valid frame id list
     *
     * @param params
     */
    void getUserValidFrameIDList(void* params);

    /**
     * @brief: callback for getting tool valid frame id list
     *
     * @param params
     */
    void getToolValidFrameIDList(void* params);
  private:
    static Controller           *instance_;     //this class 
    fst_controller::ArmGroup    *arm_group_;    //pointer of ArmGroup class
    InstructionParser           *inst_parser_;  //no use any more
    Robot                       *robot_;        //pointer of Robot class
    ManualMotion                *manu_motion_;  //pointer of ManualMotion class
    AutoMotion                  *auto_motion_;  //pointer of AutoMotion class
    Calibrator  calib_;             //instance of calibrate
    std::mutex  mutex_;             // no use any more 
    TPInterface *tp_interface_;     //pointer of TPInterface
    rcs::Task   *ctrl_task_;        //thread for state machine
    rcs::Task   *rt_traj_task_;     //thread for trajectory flow
    rcs::Task   *heartbeat_task_;   //thread for heartBeat
        
    Joint       servo_joints_;      //current servo joints

    int         inst_num_;          //no use any more 
    bool        debug_ready_;       //if it is ready to step or jump line
    std::atomic<UserOpMode>     user_op_mode_;  //TP panel  
    //std::atomic<MotionMode>     motion_mode_;   //teach and program execute
    std::atomic<InterpreterState>   intprt_state_;
    std::atomic<WorkStatus>     work_status_;
    std::atomic<RobotState>     ctrl_state_;    //fault and running
    std::atomic<RunningMode>    run_mode_;      //normal and limited running
    

    SafetyInterface             safety_interface_;  //instance of SafetyInterface
    ServiceJTAC                 serv_jtac_;     //instance of ServiceJTAC


    std::map<int, PublishUpdate>	    id_pub_map_; //the map from parameter id to there publish time

    LaunchCodeMgr               launch_code_mgr_;  //instance of SafetyInterface

    FrameManager *user_frame_manager_;
    FrameManager *tool_frame_manager_;

	/**
	 * @brief: set current mode
	 *
	 * @param: mode_cmd mode command
	 */
	void setLogicModeCmd(MotionModeCmd mode_cmd);
	/**
	 * @brief: set current logic state
	 *
	 * @param state_cmd: input==> state command
	 */
	void setLogicStateCmd(RobotStateCmd state_cmd);

    /**
     * @brief: thread of state machine 
     *
     * @param params
     */
    void stateMachine(void *params);

    /**
     * @brief: thread of trajectory flow 
     *
     * @param params
     */
    void rtTrajFlow(void* params);

    /**
     * @brief: thread of heartbeat
     *
     * @param params
     */
    void heartBeat(void* params);

    /**
     * @brief: process for TP request data 
     */
    void requestProc();

    /**
     * @brief: process for updating data to TP 
     */
    void updateProc();

    /**
     * @brief: parse TP CommandMsg 
     */
    void parseCmdMsg();

    /**
     * @brief: add publish parameter to path 
     *
     * @param str_path: path to add
     * @param pub_update: publish parameters
     */
    void addPubParameter(string str_path, PublishUpdate *pub_update);

    /**
     * @brief: remove publish parameter from the map 
     *
     * @param str_path: path to remove
     */
    void removePubParameter(string str_path);

    /**
     * @brief: remove all publish parameters from the map 
     */
    void removeAllPubParams();

    /**
     * @brief: set if publish this parameter or not 
     *
     * @param id: parameter id
     * @param flag: true if needed to publish this parameter
     */
    void setUpdateFlagByID(int id, bool flag);
#if 0
    /**
     * @brief: no use any more 
     *
     * @return 
     */
    bool planFifo();

    bool hasMoveCommand();
    bool hasTransformedToIdle();
#endif

    /**
     * @brief: check if FIFO is empty 
     *
     * @return: true if empty 
     */
    bool isFifoEmpty();

    /**
     * @brief: set RunningMode 
     *
     * @param rm: RunningMode to set
     */
    void setRunningMode(RunningMode rm);

    /**
     * @brief: clear calibrator limit or soft limit 
     *
     * @param rm: RunningMode to clear
     */
    void clearRunningMode(RunningMode rm);

    /**
     * @brief:  record current servo joints
     *
     * @return: true if Success 
     */
    bool recordJoints();

    /**
     * @brief: abort motion 
     */
    void abortMotion();

    /**
     * @brief: pause motion 
     */
    void pauseMotion();

    /**
     * @brief: continue running 
     */
     bool resumeMotion();

    /**
     * @brief: calibrate 
     */
    void calibrate();

    /**
     * @brief: set tmperary zero offset 
     */
    void setTempZero();

    /**
     * @brief: safety stop according to diffrent level 
     *
     * @param level: level to according
     */
    void safetyStop(int level);

    /**
     * @brief: servo estop 
     */
    void servoEStop();

    /**
     * @brief: action according to diffrent level 
     *
     * @param warning_level
     */
    void errorAction(int warning_level);

    /**
     * @brief: no use 
     */
    void shutdown();
};





#endif //RCS_CONTROLLER_H_


