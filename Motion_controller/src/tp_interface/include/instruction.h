/**
 * @file instruction.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-10-13
 */
#ifndef INSTRUCTION_H_
#define INSTRUCTION_H_

#include <string>
#include "motionSL.pb.h"


typedef enum _PickStatus
{
    FRESH = 1,
    USING,
    PICKED,
}PickStatus;

/*typedef union _CommandArgs*/
//{
    //motion_spec_MoveJ   movej;
    //motion_spec_MoveL   movel;
    //motion_spec_MoveC   movec;
    //motion_spec_Set     set;
    //motion_spec_Wait    wait;
/*}CommandArgs;*/

typedef struct _CommandInstruction
{
    PickStatus  pick_status;
    bool        is_finished;
    bool        is_pickedout;
    uint32_t	id;
    int         count;
    int         timesout;
    int         path_fifo_len;
    int         remain_count;
	double		smoothDistance;	
    //motion_spec_MOTIONTYPE commandtype;	
    //CommandArgs		command_arguments; //pointer of command
}CommandInstruction;

#endif //#ifndef INSTRUCTION_H_

