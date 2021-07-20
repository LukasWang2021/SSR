/*************************************************************************
	> File Name: trajectory_datatype.h
	> Author: 
	> Mail: 
	> Created Time: 2018年08月08日 星期三 09时26分40秒
 ************************************************************************/

#ifndef _TRAJECTORY_DATATYPE_H
#define _TRAJECTORY_DATATYPE_H

#include <assert.h>
#include <basic_alg_datatype.h>
#include <basic_constants.h>
#include <common_enum.h>
#include <kinematics.h>

namespace group_space
{

typedef unsigned int Tick;
typedef double  MotionTime;


struct JointState
{
	basic_alg::Joint angle;
	basic_alg::Joint omega;
	basic_alg::Joint alpha;
    basic_alg::Joint jerk;
    basic_alg::Joint torque;
};

struct PoseAndPosture
{
    basic_alg::PoseEuler    pose;
    basic_alg::Posture      posture;
    basic_alg::Turn         turn;
};

struct TargetPoint
{
    CoordinateType  type;
    
    union
    {
        basic_alg::Joint    joint;
        PoseAndPosture      pose;
    };
};

struct IntactPoint
{
    basic_alg::Joint joint;
    PoseAndPosture   pose;
    basic_alg::PoseEuler user_frame;
    basic_alg::PoseEuler tool_frame;
};

struct FrameOffset
{
    bool valid;                     // false -> 不需要坐标偏移， true -> 需要坐标偏移
    int offset_frame_id;            // 在offset_frame_id指定的基准坐标系内进行偏移,
                                    // 0 -> 基于极坐标系进行偏移，　-1 -> 基于当前坐标系进行偏移
    CoordinateType coord_type;      // 在关节空间或者笛卡尔空间进行偏移

    union
    {
        basic_alg::Joint offset_joint;      // 关节空间偏移量
        basic_alg::PoseEuler offset_pose;   // 笛卡尔空间偏移量
    };
};

struct MotionInfo
{
    MotionType  type;   // 指令的运动类型
    SmoothType  smooth_type;    // 指令的平滑类型
    bool is_swift;

    double  cnt;    // 平滑语句的CNT范围 ： 速度平滑[0.0, 1.0]，距离平滑[0.0, +∞]， FINE语句的CNT < 0
    double  vel;    // 指令速度： 如果是moveJ，指令速度是百分比, 范围: 0.0 - 1.0
                    //            如果是moveL或moveC，指令速度是mm/s, 范围： 0.0 - MAX_VEL
    double  acc;    // 指令加速度： 附加指令中的加速度是百分比，范围: 0.0 - 1.0

    IntactPoint target;
    IntactPoint via;
};

#define     PR_POS_LEN           64
struct MotionTarget     // 用于move指令的数据结构
{
    MotionType  type;           // 指令的运动类型
    SmoothType  smooth_type;    // 指令的平滑类型
    bool is_swift;

    double  cnt;    // 平滑参数的范围 ： 速度平滑[0.0, 1.0]，距离平滑[0.0, +∞]， 如果是FINE语句CNT应为-1
    double  vel;    // 指令速度： 如果是moveJ，指令速度是百分比, 范围: 0.0 - 1.0
                    //          如果是moveL或moveC，指令速度是mm/s, 范围： 0.0 - MAX_VEL
    double  acc;    // 指令加速度： 附加指令中的加速度是百分比，范围: 0.0 - 1.0
    
    int user_frame_id;  // 需要指定目标点所处的用户坐标系标号和所用工具的标号，反解时需要
    int tool_frame_id;  // 如果用户坐标系标号和工具标号与当前的在用标号不符时直接报错，如果是-1则使用当前激活的uf和tf

    FrameOffset user_frame_offset;
    FrameOffset tool_frame_offset;

    int prPos[PR_POS_LEN];
    TargetPoint target;   // moveJ和moveL时使用
    TargetPoint via;      // moveC时用作中间一个辅助点
};

struct TrajectoryPoint  // 差值得到的轨迹点
{
    JointState state;
    PointLevel level;  // 轨迹点位置，起始点、中间点或者结束点
    double time_stamp;
};

struct AxisCoeff        // 一个轴轨迹的表达式
{
    int     reserve[4]; // 保留
    double  data[10];  // 表达式，根据算法实现进行细化
};


class GroupDirection        // 手动示教模式下各轴运动方向
{
  public:
    ManualDirection axis1;
    ManualDirection axis2;
    ManualDirection axis3;
    ManualDirection axis4;
    ManualDirection axis5;
    ManualDirection axis6;
    ManualDirection axis7;
    ManualDirection axis8;
    ManualDirection axis9;

    ManualDirection& operator[](size_t index) {assert(index < NUM_OF_JOINT); return *(&axis1 + index);}
    const ManualDirection& operator[](size_t index) const {assert(index < NUM_OF_JOINT); return *(&axis1 + index);}
};

struct ManualAxisBlock
{
    MotionTime time_stamp[8];
    AxisCoeff coeff[7];
};



}

#endif
