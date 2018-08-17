#ifndef PROCESS_COMM_H
#define PROCESS_COMM_H


#include "process_comm_param.h"
#include "common_log.h"
#include "controller_server.h"
#include "controller_client.h"
#include "interpreter_server.h"
#include "interpreter_client.h"
#include "heartbeat_client.h"

namespace fst_base
{
class ProcessComm
{
public:
    ProcessComm();
    ~ProcessComm();

    static ProcessComm* getInstance();

    ControllerServer* getControllerServerPtr();
    ControllerClient* getControllerClientPtr();
    InterpreterServer* getInterpreterServerPtr();
    InterpreterClient* getInterpreterClientPtr();
    HeartbeatClient* getHeartbeatClientPtr();
    
private:
    static ProcessComm* instance_;
    ProcessCommParam* param_ptr_;
    fst_log::Logger* log_ptr_;

    ControllerServer* controller_server_ptr_;
    ControllerClient* controller_client_ptr_;
    InterpreterServer* interpreter_server_ptr_;
    InterpreterClient* interpreter_client_ptr_;
    HeartbeatClient* heartbeat_client_ptr_;
    
};

}

#endif


