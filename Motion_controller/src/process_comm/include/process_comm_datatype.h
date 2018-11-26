#ifndef PROCESS_COMM_DATATYPE_H
#define PROCESS_COMM_DATATYPE_H

#include "interpreter_common.h"
#include "fst_io_device.h"

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
    CONTROLLER_SERVER_CMD_SET_INSTRUCTION = 10,
    CONTROLLER_SERVER_CMD_IS_NEXT_INSTRUCTION_NEEDED = 11,
    CONTROLLER_SERVER_CMD_CHECK_IO = 12,//todo delete, do not use.
    CONTROLLER_SERVER_CMD_SET_IO = 13,//todo
    CONTROLLER_SERVER_CMD_GET_IO = 14,//todo
    CONTROLLER_SERVER_CMD_SET_INTERPRETER_SERVER_STATUS = 15,
    CONTROLLER_SERVER_CMD_GET_DI = 16,
    CONTROLLER_SERVER_CMD_SET_DI = 17,
    CONTROLLER_SERVER_CMD_GET_DO = 18,
    CONTROLLER_SERVER_CMD_SET_DO = 19,
    CONTROLLER_SERVER_CMD_GET_RI = 20,
    CONTROLLER_SERVER_CMD_SET_RI = 21,
    CONTROLLER_SERVER_CMD_GET_RO = 22,
    CONTROLLER_SERVER_CMD_SET_RO = 23,
}ControllerServerCmd;

typedef enum
{
    // used by interpretor
    INTERPRETER_SERVER_CMD_LOAD = 255,
    // used by controller
    INTERPRETER_SERVER_CMD_START = 0,
    INTERPRETER_SERVER_CMD_DEBUG = 1,
    INTERPRETER_SERVER_CMD_FORWARD = 2,
    INTERPRETER_SERVER_CMD_BACKWARD = 3,
    INTERPRETER_SERVER_CMD_JUMP = 4,
    INTERPRETER_SERVER_CMD_PAUSE = 5,
    INTERPRETER_SERVER_CMD_RESUME = 6,
    INTERPRETER_SERVER_CMD_ABORT = 7,
    INTERPRETER_SERVER_CMD_GET_NEXT_INSTRUCTION = 8,
    INTERPRETER_SERVER_CMD_SET_AUTO_START_MODE = 9,
    INTERPRETER_SERVER_CMD_SWITCH_STEP = 10,
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
    unsigned long long data;
}ProcessCommEvent;

typedef struct//todo delete
{
    IOPortInfo port_info;
    ErrorCode error_code;
}ResponseCheckIo;

typedef struct//todo delete
{
    IOPortInfo port_info;
    char value;
}RequestSetIo;

typedef struct//todo delete
{
    IOPortInfo port_info;
    int buffer_length;
}RequestGetIo;

typedef struct//doto delete
{
    unsigned long long error_code;
    char value;
}ResponseGetIo;

typedef struct
{
    uint32_t port_offset;
}RequestGetDi,RequestGetRi,RequestGetDo,RequestGetRo;

typedef struct
{
    unsigned long long error_code;
    uint32_t value;
}ResponseGetDi,ResponseGetRi,ResponseGetDo,ResponseGetRo;

typedef struct
{
    uint32_t port_offset;
    uint32_t value;
}RequestSetDi,RequestSetRi,RequestSetDo,RequestSetRo;


}

#endif

