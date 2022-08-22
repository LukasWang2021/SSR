#ifndef CONTROLLER_H
#define CONTROLLER_H

/**
 * @file controller.h
 * @brief The file is the header file of class "Controller".
 * @author Feng.Wu
 */

#include <mutex>
#include "common_error_code.h"
#include "yaml_help.h"
#include "xml_help.h"
#include "thread_help.h"
#include "system/core_comm_system.h"
#include "system/servo1001_comm.h"
#include "tp_comm.h"
#include "controller_config.h"
#include "controller_rpc.h"
#include "controller_publish.h"
#include "log_manager_producer.h"
#include "system_model_manager.h"
#include "axis.h"
#include "error_queue.h"
#include "axis1000.h"
#include "io_1000.h"
#include "io_safety.h"
#include "group.h"
#include "motion_control.h"
#include "reg_manager.h"
#include "interpreter_control.h"
#include "force_sensor.h"

/**
 * @brief user_space includes the user level implementation.
 */
namespace user_space
{

/**
 * @brief Controller is the application of axes.
 * @details 
 */
class Controller
{
public:
    /**
     * @brief Destructor of the class. 
     */   
    ~Controller();

    /**
     * @brief Singleton.
     * @details
     * @return Singleton.
     */
    static Controller* getInstance();

    /**
     * @brief Initialization.
     * @details
     * @return error_code.
     */
    ErrorCode init();

    /**
     * @brief Check if the proccess is about to exit the cycle.
     * @retval false The proccess is not setted to exit the cycle.
     * @retval true The proccess is setted to exit the cycle.
     */
    bool isExit();

    /**
     * @brief Set the proccess to exit the cycle.
     * @details The action is triggered by keyboard "Ctrl+C".\n
     * @return void
     */
    void setExit();

    /**
     * @brief The routine thread function.
     * @details Mostly handle RPC and Publishing.\n
     * @return void
     */
    void runRoutineThreadFunc();
    void runRpcThreadFunc();

    /**
     * @brief The algorithm thread function.
     * @details Deal with Function Block Queue.\n
     * @return void
     */
    void runPlannerThreadFunc();

    void runOnlineTrajThreadFunc();
    void runPriorityThreadFunc();
    
    /**
     * @brief The realtime thread function.
     * @details Deal with feedback, state machine and Trajectories.\n
     * @return void
     */
    void runRtThreadFunc();
private: 
    /**
     * @brief Constructor of the class.
     */
    Controller();
    ErrorCode bootUp(void);
        
    static Controller* instance_;
    base_space::YamlHelp yaml_help_;
    ControllerConfig* config_ptr_;
    bool is_exit_;

    core_comm_space::CoreCommSystem core_comm_system_;
    base_space::ThreadHelp routine_thread_;
    base_space::ThreadHelp planner_thread_;
    base_space::ThreadHelp priority_thread_;
    base_space::ThreadHelp rt_thread_;
    base_space::ThreadHelp rpc_thread_;
    base_space::ThreadHelp online_traj_thread_;

    servo_comm_space::Servo1001* servo_1001_ptr_;
    servo_comm_space::ServoCpuCommBase* cpu_comm_ptr_;
    servo_comm_space::ServoCommBase* servo_comm_ptr_[AXIS_NUM];

	user_space::TpComm tp_comm_;
	ControllerRpc rpc_;
	ControllerPublish publish_;
	base_space::FileManager file_manager_;
    log_space::LogProducer log_manager_;
    system_model_space::SystemModelManager model_manager_;
    std::vector<hal_space::BaseDevice*> dev_ptr_list;
    hal_space::Io1000* io_digital_dev_ptr_;
    hal_space::IoSafety* io_safety_dev_ptr_;

    system_model_space::AxisModel_t* axis_model_ptr_[AXIS_NUM];
    axis_space::Axis* axis_ptr_[AXIS_NUM];
    std::vector<system_model_space::AxisConfig_t> axes_config_;
    
    //group related
    fst_ctrl::ToolManager tool_manager_;
    fst_ctrl::CoordinateManager coordinate_manager_;
    fst_ctrl::RegManager reg_manager_;
    system_model_space::GroupModel_t* group_model_ptr_[GROUP_NUM];
    group_space::MotionControl* group_ptr_[GROUP_NUM];
    std::vector<system_model_space::GroupConfig_t> group_config_;

    system_model_space::ForceModel_t* force_model_ptr_;
    std::vector<system_model_space::ForceConfig_t> forces_config_;

	sensors_space::ForceSensor force_sensor_;

    uint32_t fdb_current_time_stamp_;

    void uploadErrorCode(void);
    void DisableControllerByErrorCode(ErrorCode err);
    void processDevice(void);
    bool downloadServoParams(int32_t axis_id);
    bool downloadForceControlParams();
};

}

void* controllerRoutineThreadFunc(void* arg);
void* controllerPlannerThreadFunc(void* arg);
void* controllerPriorityThreadFunc(void* arg);
void* controllerRealTimeThreadFunc(void* arg);
void* controllerRpcThreadFunc(void* arg);
void* controllerOnlineTrajThreadFunc(void* arg);


#endif


