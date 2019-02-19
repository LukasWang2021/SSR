#ifndef CONTROLLER_PUBLISH_H
#define CONTROLLER_PUBLISH_H

#include "controller_param.h"
#include "common_log.h"
#include "virtual_core1.h"
#include "tp_comm.h"
#include "controller_sm.h"
#include "motion_control.h"
#include "base_datatype.h"
#include "reg_manager.h"
#include "process_comm.h"
#include "io_mapping.h" //feng add for iomapping.
#include "fst_safety_device.h"
#include "base_device.h"//feng add
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
                    ControllerSm* state_machine_ptr, fst_mc::MotionControl* motion_control_ptr, RegManager* reg_manager_ptr,
                    fst_base::ControllerClient* controller_client_ptr,
                    fst_ctrl::IoMapping* io_mapping_ptr, fst_hal::DeviceManager* device_manager_ptr, 
                    fst_hal::IoManager* io_manager_ptr);//feng add mapping

    typedef void* (ControllerPublish::*HandlePublishFuncPtr)(void);
    typedef void (ControllerPublish::*HandleUpdateFuncPtr)(void);
    HandlePublishFuncPtr getPublishHandlerByHash(unsigned int hash);
    HandleUpdateFuncPtr getUpdateHandlerByHash(unsigned int hash);
    void addTaskToUpdateList(HandleUpdateFuncPtr func_ptr);
    void unrefUpdateListElement(HandleUpdateFuncPtr func_ptr);
    void cleanUpdateList();
    void deleteTaskFromUpdateList(std::vector<fst_comm::TpPublishElement>& publish_element_list);

    void* getRegPublishDataPtr(RegType reg_type, int reg_index);
    void* addTaskToRegUpdateList(RegType reg_type, int reg_index);
    void unrefRegUpdateListElement(RegType reg_type, int reg_index);
    void cleanRegUpdateList();
    void deleteTaskFromRegUpdateList(std::vector<fst_comm::TpPublishElement>& publish_element_list);

    void* addTaskToIoUpdateList(uint32_t port_type, uint32_t port_offset);//feng add for io publish
    void deleteTaskFromIoUpdateList(std::vector<fst_comm::TpPublishElement>& publish_element_list);
    void unrefIoUpdateListElement(uint32_t port_type, uint32_t port_offset);
    void cleanIoUpdateList();

    void processPublish();
    
private:
    fst_log::Logger* log_ptr_;
    ControllerParam* param_ptr_;
    VirtualCore1* virtual_core1_ptr_;
    fst_comm::TpComm* tp_comm_ptr_;
    ControllerSm* state_machine_ptr_;
    fst_mc::MotionControl* motion_control_ptr_;
    RegManager* reg_manager_ptr_;
    fst_base::ControllerClient* controller_client_ptr_;
    fst_ctrl::IoMapping* io_mapping_ptr_; //feng add for mapping.
    fst_hal::DeviceManager* device_manager_ptr_;
    fst_hal::FstSafetyDevice* safety_device_ptr_;
    fst_hal::IoManager* io_manager_ptr_;
    fst_hal::ModbusManager* modbus_manager_ptr_;

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
    typedef struct
    {
        HandleUpdateFuncPtr func_ptr;
        int ref_count;
    }PublishUpdate;
    std::list<PublishUpdate> update_list_;
    
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
    MessageType_StringList tp_program_status_;
    MessageType_Uint32 safety_board_status_;
    MessageType_IoBoardStatusList io_board_status_; 
    MessageType_ModbusClientCtrlStatusList modbus_client_ctrl_status_;

    typedef struct
    {
        RegType reg_type;
        int reg_index;
        MessageType_PrValue pr_value;
        MessageType_HrValue hr_value;
        MessageType_MrValue mr_value;
        MessageType_SrValue sr_value;
        MessageType_RValue r_value;
        int ref_count;
    }RegPublishUpdate;
    std::list<RegPublishUpdate> reg_update_list_;

    //feng add for addIoTopic
    typedef struct
    {
        //int device_index;
        //char address;
        uint32_t port_type;
        uint32_t port_offset;
        MessageType_Uint32 value;
        bool is_valid;
        int ref_count;
    }IoPublishUpdate;
    std::list<IoPublishUpdate> io_update_list_;


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
    void* getTpProgramStatusPtr();
    void* getSafetyBoardStatusPtr();
    void* getIoBoardStatusPtr();
    void* getModbusClientCtrlStatusPtr();

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
    void updateTpProgramStatus();
    void updateSafetyBoardStatus();
    void updateIoBoardStatus();
    void updateModbusClientCtrlStatus();
    // update reg publish
    void updateReg();

    // update io publish
    void updateIo();
};

}


#endif

