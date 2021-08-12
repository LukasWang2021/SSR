/*************************************************************************
	> File Name: motion_control_datatype.h
	> Author: 
	> Mail: 
	> Created Time: 2018年08月08日 星期三 09时26分40秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_DATATYPE_H
#define _MOTION_CONTROL_DATATYPE_H

#include <basic_alg_datatype.h>
#include <basic_constants.h>
#include <common_enum.h>
#include "trajectory_datatype.h"

namespace group_space
{

//interpreter interface
typedef enum _InstType
{
    COMMON = 1,
    LOGIC_TOK,
    END_TOK,
    MOTION,
    SET_UF,
    SET_TF,
    SET_OVC,
    SET_OAC,
    SET_PAYLOAD,
    END_PROG,
}InstType;

typedef void (*PauseCallBack_fp)(void);
typedef void (*SetLineNumCallBack_fp)(int32_t);

struct Instruction
{
    InstType type;
    UserOpMode user_op_mode;
    union
    {
        int uf_id;
        int tf_id;
        int payload_id;
        double ovc;
        double oac;
        MotionTarget target;
    };
    int line_num;
    PauseCallBack_fp interp_pause;
    // SetLineNumCallBack_fp set_line_num;
};

enum AxisIndex
{
    INDEX_JOINT1 = 0,
    INDEX_JOINT2 = 1,
    INDEX_JOINT3 = 2,
    INDEX_JOINT4 = 3,
    INDEX_JOINT5 = 4,
    INDEX_JOINT6 = 5,
};

}

#endif
