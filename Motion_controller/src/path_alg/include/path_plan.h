/*************************************************************************
	> File Name: path_plan.h
	> Author: 
	> Mail: 
	> Created Time: 2018年09月05日 星期三 17时56分28秒
 ************************************************************************/

#ifndef _PATH_PLAN_H
#define _PATH_PLAN_H

#include <base_datatype.h>
#include <error_code.h>


namespace fst_mc
{

ErrorCode planJointPath(const Joint &start, const Joint &target, double &precision, size_t &index, Joint (&path)[MAX_PATH_SIZE], size_t &valid_length);
ErrorCode planLinePath(const PoseEuler &start, const PoseEuler &target, double &precision, Pose (&path)[MAX_PATH_SIZE], size_t &valid_length);

}




#endif
