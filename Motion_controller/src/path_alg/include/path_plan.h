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

/*
class PathPlan
{
  public:
    PathPlan(void);
    ~PathPlan(void);

    ErrorCode initPathPlan(size_t joint_num);
    ErrorCode planJointPath(const Joint &start, const Joint &target, int id, double precision);

  private:
    size_t joint_num_;
};
*/

}




#endif
