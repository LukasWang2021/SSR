#ifndef CONTROLLER_H
#define CONTROLLER_H


#include "controller_param.h"
#include "tp_comm.h"
#include "common_log.h"
#include "thread_help.h"
#include "controller_sm.h"
#include "controller_rpc.h"
#include "controller_publish.h"
#include "tool_manager.h"
#include "coordinate_manager.h"
#include "reg_manager.h"
#include "process_comm.h"
#include "controller_ipc.h"
#include "device_manager.h"
#include "motion_control.h"
#include "serverAlarmApi.h"
#include "preformance_monitor.h"
#include "io_mapping.h" 
#include "program_launching.h"
#include "file_manager.h"
#include "system_manager.h"
#include "param_manager.h"
#include <string>
#include "forsight_inter_control.h"
// for test only
#include "virtual_core1.h"
#include <cstdint>


namespace fst_ctrl
{
class Controller
{
public:
    Controller();
    ~Controller();

    static Controller* getInstance();
    ErrorCode init();
    bool isExit();
    void setExit();
    
    void runRoutineThreadFunc();
    void runHeartbeatThreadFunc();
    
private: 
    static Controller* instance_;
    fst_log::Logger* log_ptr_;
    ControllerParam* param_ptr_;
    ControllerSm state_machine_;
    fst_comm::TpComm tp_comm_;
    ControllerRpc rpc_;
    ControllerPublish publish_;
    ControllerIpc ipc_;
    ToolManager tool_manager_;
    CoordinateManager coordinate_manager_;
    RegManager reg_manager_;
    fst_hal::DeviceManager device_manager_;
    fst_mc::MotionControl motion_control_;
    fst_base::ProcessComm* process_comm_ptr_;
    fst_base::PreformanceMonitor preformance_monitor_;
    VirtualCore1 virtual_core1_; // for test only
    fst_hal::IoManager io_manager_;
    IoMapping io_mapping_;   
    ProgramLaunching program_launching_;
    fst_base::FileManager file_manager_;
    fst_ctrl::SystemManager system_manager_;
    fst_mc::ParamManager param_manager_;

    fst_hal::ModbusManager* modbus_manager_ptr_; 

    // thread related
    bool is_exit_;
    fst_base::ThreadHelp routine_thread_;
    fst_base::ThreadHelp heartbeat_thread_;

    void recordLog(std::string log_str);
    void recordLog(ErrorCode error_code, std::string log_str);
    void recordLog(ErrorCode major_error_code, ErrorCode minor_error_code, std::string log_str);

    // light a LED to hint the controller is ok
    void  isOkLed();
};

}


// thread function
void* controllerRoutineThreadFunc(void* arg);
void* heartbeatThreadFunc(void* arg);


#endif


