/*************************************************************************
	> File Name: base_kinematics.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年02月01日 星期四 09时25分48秒
 ************************************************************************/

#include <string.h>
#include <base_kinematics.h>

namespace fst_mc
{

BaseKinematics::BaseKinematics()
{
    memset(dh_matrix_, 0, sizeof(dh_matrix_));
    user_frame_.eye();
    user_frame_.eye();
    inverse_user_frame_.eye();
    inverse_tool_frame_.eye();
}

BaseKinematics::~BaseKinematics()
{}

}

