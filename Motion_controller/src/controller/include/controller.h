#ifndef CONTROLLER_H
#define CONTROLLER_H


#include "controller_param.h"
#include "tp_comm.h"
#include "common_log.h"
#include "thread_help.h"
#include "controller_sm.h"
#include "controller_rpc.h"
#include "tool_manager.h"
#include "coordinate_manager.h"
// for test only
#include "virtual_core1.h"


namespace fst_ctrl
{
class Controller
{
public:
    Controller();
    ~Controller();

    static Controller* getInstance();
    bool init();
    bool isExit();
    
    void runRoutineThreadFunc();
    
private: 
    static Controller* instance_;
    fst_log::Logger* log_ptr_;
    ControllerParam* param_ptr_;
    ControllerSm state_machine_;
    TpComm tp_comm_;
    ControllerRpc rpc_;
    ToolManager tool_manager_;
    CoordinateManager coordinate_manager_;
    VirtualCore1 virtual_core1_; // for test only
    
    // thread related
    bool is_exit_;
    fst_base::ThreadHelp routine_thread_;
    
};

}


// thread function
void controllerRoutineThreadFunc(void* arg);


#endif


