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
    CONTROLLER_SERVER_CMD_SET_MI = 5,
    CONTROLLER_SERVER_CMD_SET_MH = 6,
    CONTROLLER_SERVER_CMD_GET_PR_REG = 7,
    CONTROLLER_SERVER_CMD_GET_HR_REG = 8,
    CONTROLLER_SERVER_CMD_GET_MR_REG = 9,
    CONTROLLER_SERVER_CMD_GET_SR_REG = 10,
    CONTROLLER_SERVER_CMD_GET_R_REG = 11,
    CONTROLLER_SERVER_CMD_GET_MI = 12,
    CONTROLLER_SERVER_CMD_GET_MH = 13,
    CONTROLLER_SERVER_CMD_SET_INSTRUCTION = 14,
    CONTROLLER_SERVER_CMD_IS_NEXT_INSTRUCTION_NEEDED = 15,
    CONTROLLER_SERVER_CMD_SET_INTERPRETER_SERVER_STATUS = 16,
    CONTROLLER_SERVER_CMD_GET_DI = 17,
    CONTROLLER_SERVER_CMD_SET_DI = 18,
    CONTROLLER_SERVER_CMD_GET_DO = 19,
    CONTROLLER_SERVER_CMD_SET_DO = 20,
    CONTROLLER_SERVER_CMD_GET_RI = 21,
    CONTROLLER_SERVER_CMD_SET_RI = 22,
    CONTROLLER_SERVER_CMD_GET_RO = 23,
    CONTROLLER_SERVER_CMD_SET_RO = 24,
    CONTROLLER_SERVER_CMD_GET_UI = 25,
    CONTROLLER_SERVER_CMD_SET_UI = 26,
    CONTROLLER_SERVER_CMD_GET_UO = 27,
    CONTROLLER_SERVER_CMD_GET_JOINT = 28,
    CONTROLLER_SERVER_CMD_GET_CART = 29,
    CONTROLLER_SERVER_CMD_CART_TO_JOINT = 30,
    CONTROLLER_SERVER_CMD_JOINT_TO_CART = 31,
    CONTROLLER_SERVER_CMD_OP_MODE = 32,
    CONTROLLER_SERVER_CMD_SET_DO_PULSE = 33,
    CONTROLLER_SERVER_CMD_SET_RO_PULSE = 34,
    CONTROLLER_SERVER_CMD_GET_POSTURE = 35,
    CONTROLLER_SERVER_CMD_GET_TURN = 36,
}ControllerServerCmd;

typedef enum
{
    // used by interpretor
    INTERPRETER_SERVER_CMD_LOAD = 255,
    // used by controller
    INTERPRETER_SERVER_CMD_START                 = 0,
    INTERPRETER_SERVER_CMD_LAUNCH                = 1,
    INTERPRETER_SERVER_CMD_FORWARD               = 2,
    INTERPRETER_SERVER_CMD_BACKWARD              = 3,
    INTERPRETER_SERVER_CMD_JUMP                  = 4,
    INTERPRETER_SERVER_CMD_PAUSE                 = 5,
    INTERPRETER_SERVER_CMD_RESUME                = 6,
    INTERPRETER_SERVER_CMD_ABORT                 = 7,
    INTERPRETER_SERVER_CMD_GET_NEXT_INSTRUCTION  = 8,
    INTERPRETER_SERVER_CMD_CODE_START            = 9,
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


typedef struct
{
    int id;
    int value;
}MiDataIpc, MhDataIpc;


typedef struct
{
    uint32_t port_offset;
}RequestGetDi,RequestGetRi,RequestGetUi,RequestGetDo,RequestGetRo,RequestGetUo;

typedef struct
{
    unsigned long long error_code;
    uint32_t value;
}ResponseGetDi,ResponseGetRi,ResponseGetUi,ResponseGetDo,ResponseGetRo,ResponseGetUo;

typedef struct
{
    uint32_t port_offset;
    uint32_t value;
}RequestSetDi,RequestSetRi,RequestSetUi,RequestSetDo,RequestSetRo;

typedef struct
{
    uint32_t port_offset;
    double time;
}RequestSetPulse;

}

#endif

