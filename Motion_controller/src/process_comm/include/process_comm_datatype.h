#ifndef PROCESS_COMM_DATATYPE_H
#define PROCESS_COMM_DATATYPE_H

#include "interpreter_common.h"

namespace fst_base
{
enum {PROCESS_COMM_CMD_ID_SIZE = 4,};

typedef enum
{
    CONTROLLER_SERVER_CMD_SET_PR_REG = 0,
    CONTROLLER_SERVER_CMD_SET_HR_REG = 1,
    CONTROLLER_SERVER_CMD_SET_MR_REG = 2,
    CONTROLLER_SERVER_CMD_SET_SR_REG = 3,
    CONTROLLER_SERVER_CMD_SET_R_REG = 4,
    CONTROLLER_SERVER_CMD_GET_PR_REG = 5,
    CONTROLLER_SERVER_CMD_GET_HR_REG = 6,
    CONTROLLER_SERVER_CMD_GET_MR_REG = 7,
    CONTROLLER_SERVER_CMD_GET_SR_REG = 8,
    CONTROLLER_SERVER_CMD_GET_R_REG = 9,
}ControllerServerCmd;

typedef enum
{
    INTERPRETER_SERVER_CMD_START = 0,
    INTERPRETER_SERVER_CMD_DEBUG = 1,
    INTERPRETER_SERVER_CMD_FORWARD = 2,
    INTERPRETER_SERVER_CMD_BACKWARD = 3,
    INTERPRETER_SERVER_CMD_JUMP = 4,
    INTERPRETER_SERVER_CMD_PAUSE = 5,
    INTERPRETER_SERVER_CMD_RESUME = 6,
    INTERPRETER_SERVER_CMD_ABORT = 7,
    INTERPRETER_SERVER_CMD_GET_NEXT_INSTRUCTION = 8,
}InterpreterServerCmd;

typedef struct
{
    unsigned int cmd_id;
    void* request_data_ptr;
    void* response_data_ptr;
}ProcessCommRequestResponse;

typedef struct
{
    int interval;   // ms
    InterpreterPublish* data_ptr;
    struct timeval last_publish_time;
}ProcessCommPublish;

typedef struct
{
    int event_type;
    // if multi data type exist, change data to union 
    unsigned long long int data;
}ProcessCommEvent;

}

#endif

