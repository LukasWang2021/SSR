#ifndef PROCESS_COMM_DATATYPE_H
#define PROCESS_COMM_DATATYPE_H

namespace fst_base
{

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

typedef struct
{
    unsigned int cmd_id;
    void* request_data_ptr;
    void* response_data_ptr;
}ProcessCommRequestResponse;


}


#endif
