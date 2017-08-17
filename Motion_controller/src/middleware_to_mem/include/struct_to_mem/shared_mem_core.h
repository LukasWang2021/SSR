#ifndef MIDDLEWARE_TO_MEM_SHARED_MEM_CORE_H_
#define MIDDLEWARE_TO_MEM_SHARED_MEM_CORE_H_

#include "data_type.h"

#define MEM_CORE_ITEM(Item, hands, name) {(unsigned long)(&(((SharedMemCore *)0)->Item##_item)), (unsigned long)(&(((SharedMemCore *)0)->Item##_flag)), sizeof(Item), hands, name}
#define DECLARE_ITEM(struct_type) struct_type struct_type##_item; AccessFlag struct_type##_flag


//1.Bellow you add the new message structure.
#include "struct_to_mem/struct_trajectory_segment.h"
#include "struct_to_mem/struct_feedback_joint_states.h"
#include "struct_to_mem/struct_service_request.h"
#include "struct_to_mem/struct_service_response.h"

//2 Bellow add the new message structure to the core structure.
typedef struct
{
    DECLARE_ITEM(TrajectorySegment);
    DECLARE_ITEM(FeedbackJointState);
    DECLARE_ITEM(ServiceRequest);
    DECLARE_ITEM(ServiceResponse);
}SharedMemCore;

//3 Bellow modify MEM_TABLE_LEN and add the info of the new message to the table.
#define MEM_TABLE_CORE_LEN 4
static const FunctionTable tableCore[MEM_TABLE_CORE_LEN] = 
{
    MEM_CORE_ITEM(TrajectorySegment, MEM_HANDSHAKE, "TrajectorySegment"),        //{   0,  2104, "TrajectorySegment"}
    MEM_CORE_ITEM(FeedbackJointState, MEM_NO_HANDSHAKE, "FeedbackJointState"),   //{2120,   152, "FeedbackJointState"}
    MEM_CORE_ITEM(ServiceRequest, MEM_HANDSHAKE, "ServiceRequest"),
    MEM_CORE_ITEM(ServiceResponse, MEM_HANDSHAKE, "ServiceResponse"),
};


#endif  //MIDDLEWARE_TO_MEM_SHARED_MEM_CORE_H_
