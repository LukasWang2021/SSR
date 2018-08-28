#ifndef CONTROLLER_PUBLISH_H
#define CONTROLLER_PUBLISH_H

#include "controller_param.h"
#include "common_log.h"
#include "virtual_core1.h"
#include "tp_comm.h"
#include "controller_sm.h"
#include "motion_control.h"
#include "base_datatype.h"
#include <vector>
#include <list>

namespace fst_ctrl
{
class ControllerPublish
{
public:
    ControllerPublish();
    ~ControllerPublish();

    void init(fst_log::Logger* log_ptr, ControllerParam* param_ptr, VirtualCore1* virtual_core1_ptr, fst_comm::TpComm* tp_comm_ptr,
                    ControllerSm* state_machine_ptr, fst_mc::MotionControl* motion_control_ptr);

    typedef void* (ControllerPublish::*HandlePublishFuncPtr)(void);
    typedef void (ControllerPublish::*HandleUpdateFuncPtr)(void);
    HandlePublishFuncPtr getPublishHandlerByHash(unsigned int hash);
    HandleUpdateFuncPtr getUpdateHandlerByHash(unsigned int hash);
    void addTaskToUpdateList(HandleUpdateFuncPtr func_ptr);
    void processPublish();
    
private:
    fst_log::Logger* log_ptr_;
    ControllerParam* param_ptr_;
    VirtualCore1* virtual_core1_ptr_;
    fst_comm::TpComm* tp_comm_ptr_;
    ControllerSm* state_machine_ptr_;
    fst_mc::MotionControl* motion_control_ptr_;

    enum {HASH_BYTE_SIZE = 4,};
    enum {QUICK_SEARCH_TABLE_SIZE = 128,};

    typedef struct
    {
        std::string path;
        unsigned int hash;
        HandlePublishFuncPtr publish_func_ptr;
        HandleUpdateFuncPtr update_func_ptr;
    }PublishService;
    std::vector<PublishService> publish_table_;
    std::vector<PublishService> publish_quick_search_table_[QUICK_SEARCH_TABLE_SIZE]; 

    // publish data, mutex protected    
    MessageType_Int32_DoubleList joint_feedback_;
    MessageType_Int32_DoubleList tcp_world_cartesian_;
    MessageType_Int32_DoubleList tcp_base_cartesian_;
    MessageType_Int32_DoubleList tcp_current_cartesian_;
    MessageType_Int32List current_coordinate_;
    MessageType_Int32List current_tool_;
    MessageType_Double global_vel_ratio_;
    MessageType_Double global_acc_ratio_;
    MessageType_String_Int32 program_status_;

    std::list<HandleUpdateFuncPtr> update_list_;
    
    void initPublishTable();
    void initPublishQuickSearchTable();
    

    // get publish element ptr
    void* getUserOpModePtr();
    void* getRunningStatePtr();
    void* getInterpreterStatePtr();
    void* getRobotStatePtr();
    void* getCtrlStatePtr();
    void* getServoStatePtr();
    void* getSafetyAlarmPtr();
    void* getAxisGroupJointFeedbackPtr();
    void* getAxisGroupTcpWorldCartesianPtr();
    void* getAxisGroupTcpBaseCartesianPtr();
    void* getAxisGroupTcpCurrentCartesianPtr();
    void* getAxisGroupCurrentCoordinatePtr();
    void* getAxisGroupCurrentToolPtr();
    void* getGlobalVelRatioPtr();
    void* getGlobalAccRatioPtr();
    void* getProgramStatusPtr();

    // update publish element
    void updateAxisGroupJointFeedback();
    void updateAxisGroupTcpWorldCartesian();
    void updateAxisGroupTcpBaseCartesian();
    void updateAxisGroupTcpCurrentCartesian();
    void updateAxisGroupCurrentCoordinate();
    void updateAxisGroupCurrentTool();
    void updateGlobalVelRatio();
    void updateGlobalAccRatio();
    void updateProgramStatus();
};

}


#endif

