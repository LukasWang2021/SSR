#ifndef MOTION_PLAN_FRAME_MANAGER_H
#define MOTION_PLAN_FRAME_MANAGER_H

#include "fst_datatype.h"
#include <vector>
#include <string>

namespace fst_controller
{

enum 
{
    MAX_COMMENT_LENGTH = 32,
};
    
enum 
{
    MAX_USER_FRAME_NUM = 11,
    MAX_TOOL_FRAME_NUM = 11,
};

typedef struct
{
    int id;
    bool is_valid;
    char comment[MAX_COMMENT_LENGTH];
    PoseEuler data;
}Frame;

typedef struct
{
    int id;
    char comment[MAX_COMMENT_LENGTH];
}FrameSimple;


}

#endif

