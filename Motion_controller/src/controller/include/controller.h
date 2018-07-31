#ifndef CONTROLLER_H
#define CONTROLLER_H


#include "controller_param.h"
#include "tp_comm_component.h"
#include "common_log.h"



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
    
private: 
    static Controller* instance_;
    fst_log::Logger log_;
    ControllerParam param_;
    TpCommComponent tp_comm_;
    bool is_exit_;
};

}

#endif


