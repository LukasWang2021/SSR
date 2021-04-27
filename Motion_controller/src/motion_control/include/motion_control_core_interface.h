/*************************************************************************
	> File Name: motion_control_core_interface.h
	> Author: 
	> Mail: 
	> Created Time: 2018年08月08日 星期三 15时35分55秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_CORE_INTERFACE_H
#define _MOTION_CONTROL_CORE_INTERFACE_H

#include <vector>
#include <motion_control_datatype.h>
#include <common_error_code.h>
#include <common_enum.h>
#include "group.h"

namespace group_space
{
#define JC_POINT_NUM 10
#define START_POINT 1
#define END_POINT 2
#define MID_POINT 0
typedef struct 
{
    double angle[6];
    double omega[6];
    double alpha[6];
    double inertia[6];
    int point_position;
}Points;
typedef struct 
{
    Points points[JC_POINT_NUM];
    int total_points;
}JointCommand;

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

    bool initInterface(uint32_t joint_num, std::map<int32_t, axis_space::Axis*>* axis_group_ptr, GroupSm* sm_ptr);
    
    bool sendPoint(void);
    bool isPointCacheEmpty(void);
    bool clearPointCache(void);
    bool fillPointCache(TrajectoryPoint *points, size_t length, PointProperty proerty);

    bool getLatestJoint(basic_alg::Joint &joint, uint32_t (&encoder_state)[NUM_OF_JOINT], ServoState &state);

    bool setConfigData(int id, const std::vector<int> &data);
    bool setConfigData(int id, const std::vector<double> &data);
    bool getConfigData(int id, std::vector<double> &data);
    bool getEncoder(std::vector<int> &data);
    bool getEncoderError(std::vector<int> &data);
    bool resetEncoderError(size_t index);
    bool resetEncoderError(void);
    bool getControlPosition(double *data, size_t len);

  private:
    uint32_t joint_num_;
    PointCache  point_cache_;
    std::map<int32_t, axis_space::Axis*>* axis_group_ptr_;              /**< The list of the axes in the group.*/
	  GroupSm* sm_ptr_;                                               /**< The state machine of the group.*/
};









}

#endif
