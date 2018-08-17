#ifndef PROCESS_COMM_PARAM_H
#define PROCESS_COMM_PARAM_H


#include "parameter_manager/parameter_manager_param_group.h"

namespace fst_base
{
class ProcessCommParam
{
public:
    ProcessCommParam();
    ~ProcessCommParam();

    bool loadParam();
    bool saveParam();

    // param to load & save
    int log_level_;

    int controller_server_thread_priority_;
    int controller_server_cycle_time_;    //us
    std::string i2c_req_res_ip_;   // Server(Controller)/Client(Interpreter)
    
    int interpreter_server_thread_priority_;
    int interpreter_server_cycle_time_; //us
    std::string c2i_req_res_ip_;   // Server(Interpreter)/Client(Controller)
    std::string c2i_pub_ip_;        // Server(Interpreter)/Client(Controller)
    std::string c2i_event_ip_;      // Server(Interpreter)/Client(Controller)

    int heartbeat_cycle_time_;      //us
    std::string heartbeat_ip_;      // Server(ServiceManager)/Client(Controller)
    
    int interpreter_server_event_buffer_size_;
    int recv_buffer_size_;
    int send_buffer_size_;

private:
    fst_parameter::ParamGroup yaml_help_;
    std::string file_path_;
};

}


#endif

