/*************************************************************************
	> File Name: motion_control_core_interface.h
	> Author: 
	> Mail: 
	> Created Time: 2018年08月08日 星期三 15时35分55秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_CORE_INTERFACE_H
#define _MOTION_CONTROL_CORE_INTERFACE_H

#include <comm_interface/core_interface.h>
#include <struct_to_mem/struct_service_request.h>
#include <struct_to_mem/struct_service_response.h>
#include <service_actions/response_actions.h>
#include <comm_interface/comm_interface.h>
#include <motion_control_datatype.h>
#include <motion_control_error_code.h>
#include <common_enum.h>

namespace fst_mc
{


enum PointProperty
{
    POINT_POS = 1,
    POINT_POS_VEL = 2,
    POINT_POS_VEL_ACC = 4,
    POINT_POS_VEL_ACC_EFF = 8,
};

enum PointLevel
{
    POINT_MIDDLE = 0,
    POINT_START  = 1,
    POINT_ENDING = 2,
};

struct TrajectoryPoint
{
    Joint   angle;
    Joint   omega;
    Joint   alpha;
    Joint   torque;
    Joint   inertia;
    Joint   gravity;
    PointLevel  level;
};

struct PointCache
{
    bool is_empty;
    JointCommand cache;
    PointProperty property;
};

class BareCoreInterface
{
  public:
    BareCoreInterface();
    ~BareCoreInterface();

    bool initInterface(void);
    
    bool sendPoint(void);
    bool isPointCacheEmpty(void);
    bool fillPointCache(TrajectoryPoint *points, size_t length, PointProperty proerty);

    bool getLatestJoint(Joint &joint, ServoStatus &state);

    bool resetBareCore(void);
    bool stopBareCore(void);

  private:
    PointCache  point_cache_;

    fst_core_interface::CoreInterface   core_interface_;
    fst_comm_interface::CommInterface   command_interface_;
};









}

#endif
