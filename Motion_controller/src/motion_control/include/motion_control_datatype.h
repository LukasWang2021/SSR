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
    STE_PAYLOAD,
    END_PROG,
}InstType;

typedef void (*PauseCallBack_fp)(bool);
typedef void (*SetLineNumCallBack_fp)(int32_t);
//typedef struct
struct Instruction
{

    char            line[512];

    int             line_num;

    InstType        type;
    group_space::UserOpMode user_op_mode;

    MotionTarget    target;
    int  loop_cnt;

	int  current_uf ;
	int  current_tf ;
	double  current_ovc ;
	double  current_oac ;
    int payload_id;
    bool is_additional;
    int add_num;
    // motion control can tell interpreter whether the command is failed or success
    PauseCallBack_fp interp_pause;
    SetLineNumCallBack_fp set_line_num;
    char additional[0]; //malloc other memory

    //add some other additional statement
    //.....
    //.....
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
