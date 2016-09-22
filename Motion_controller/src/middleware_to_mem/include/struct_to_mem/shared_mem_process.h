#ifndef _SHARED_MEM_PROCESS_H_
#define _SHARED_MEM_PROCESS_H_
#include "data_type.h"

#define OFFSET_OF(structure, member) (unsigned long)(&(((structure *)0)->member))
#define MEM_PROCESS_ITEM(Item, hands, name) {(unsigned long)(&(((SharedMemProcess *)0)->Item##_item)), (unsigned long)(&(((SharedMemProcess *)0)->Item##_flag)), sizeof(Item), hands, name}
#define DECLARE_ITEM(struct_type) struct_type struct_type##_item; AccessFlag struct_type##_flag


//1.Bellow include the new message structure

#include "struct_joint_command.h"
#include "struct_feedback_joint_states.h"


//2.Bellow add the new message structure to the big structure
typedef struct
{
    DECLARE_ITEM(JointCommand);
    DECLARE_ITEM(FeedbackJointState);

}SharedMemProcess;

//3.Bellow modify MEM_TABLE_LEN and add the info of the new message to the table
#define MEM_TABLE_PROCESS_LEN 2
static const FunctionTable tableProcess[MEM_TABLE_PROCESS_LEN] = 
{
    MEM_PROCESS_ITEM(JointCommand, MEM_HANDSHAKE, "JointCommand"),        
    MEM_PROCESS_ITEM(FeedbackJointState, MEM_NO_HANDSHAKE, "FeedbackJointState"),   

};


#endif
