/*************************************************************************
	> File Name: base_kinematics.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年02月01日 星期四 09时25分48秒
 ************************************************************************/

#include <string.h>
#include <base_kinematics.h>

using namespace basic_alg;

namespace fst_mc
{

BaseKinematics::BaseKinematics()
{
    memset(dh_matrix_, 0, sizeof(dh_matrix_));
    user_frame_.eye();
    user_frame_.eye();
    world_frame_.eye();
    inverse_user_frame_.eye();
    inverse_tool_frame_.eye();
    inverse_world_frame_.eye();
}

BaseKinematics::~BaseKinematics()
{}

ErrorCode BaseKinematics::setUserFrame(PoseEuler uf)
{
    user_frame_ = uf;
    inverse_user_frame_ = uf;
    inverse_user_frame_.inverse();
    return SUCCESS;
}

ErrorCode BaseKinematics::setToolFrame(PoseEuler tf)
{
    tool_frame_ = tf;
    inverse_tool_frame_ = tf;
    inverse_tool_frame_.inverse();
    return SUCCESS;
}

}

