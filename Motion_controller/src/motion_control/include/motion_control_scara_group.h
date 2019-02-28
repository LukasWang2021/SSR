/*************************************************************************
	> File Name: motion_control_scara_group.h
	> Author: 
	> Mail: 
	> Created Time: 2019年02月28日 星期四 10时55分53秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_SCARA_GROUP_H
#define _MOTION_CONTROL_SCARA_GROUP_H

#include <common_log.h>
#include <motion_control_base_group.h>


#define JOINT_OF_SCARA    4

namespace fst_mc
{

class ScaraGroup : public BaseGroup
{
  public:
    ScaraGroup(fst_log::Logger* plog) : BaseGroup(plog) {};
    ~ScaraGroup() {};

    ErrorCode initGroup(fst_base::ErrorMonitor *error_monitor_ptr);

    size_t getNumberOfJoint(void);
    size_t getFIFOLength(void);

  private:
    inline char* printDBLine(const int *data, char *buffer, size_t length);
    inline char* printDBLine(const double *data, char *buffer, size_t length);
};


#endif
