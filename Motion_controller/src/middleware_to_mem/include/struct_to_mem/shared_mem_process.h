#ifndef MIDDLEWARE_TO_MEM_SHARED_MEM_PROCESS_H_
#define MIDDLEWARE_TO_MEM_SHARED_MEM_PROCESS_H_

#include "data_type.h"

#define MEM_PROCESS_ITEM(Item, hands, name) {(unsigned long)(&(((SharedMemProcess *)0)->Item##_item)), (unsigned long)(&(((SharedMemProcess *)0)->Item##_flag)), sizeof(Item), hands, name}
#define DECLARE_ITEM(struct_type) struct_type struct_type##_item; AccessFlag struct_type##_flag


//1.Bellow include the new message structure

#include "struct_to_mem/struct_joint_command.h"
#include "struct_to_mem/struct_feedback_joint_states.h"
#include "struct_to_mem/struct_io_signal.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"


//2.Bellow add the new message structure to the big structure
typedef struct
{
    DECLARE_ITEM(JointCommand);
    DECLARE_ITEM(FeedbackJointState);
    DECLARE_ITEM(IOSignal);
    DECLARE_ITEM(ServiceRequest);
    DECLARE_ITEM(ServiceResponse);

}SharedMemProcess;

//3.Bellow modify MEM_TABLE_LEN and add the info of the new message to the table
#define MEM_TABLE_PROCESS_LEN 5
FunctionTable tableProcess[MEM_TABLE_PROCESS_LEN] = 
{
    MEM_PROCESS_ITEM(JointCommand, MEM_HANDSHAKE, "JointCommand"),        
    MEM_PROCESS_ITEM(FeedbackJointState, MEM_NO_HANDSHAKE, "FeedbackJointState"), 
    MEM_PROCESS_ITEM(IOSignal, MEM_HANDSHAKE, "IOSignal"),
    MEM_PROCESS_ITEM(ServiceRequest, MEM_HANDSHAKE, "ServiceRequest"),
    MEM_PROCESS_ITEM(ServiceResponse, MEM_HANDSHAKE, "ServiceResponse"),
};

#endif // MIDDLEWARE_TO_MEM_SHARED_MEM_PROCESS_H_
