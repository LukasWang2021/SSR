/*************************************************************************
	> File Name: motion_control_datatype.h
	> Author: 
	> Mail: 
	> Created Time: 2018年08月08日 星期三 09时26分40秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_DATATYPE_H
#define _MOTION_CONTROL_DATATYPE_H

#include <base_datatype.h>

namespace fst_mc
{

#define PATH_CACHE_SIZE         1024
#define TRAJECTORY_CACHE_SIZE   128

struct JointConstraint  // 关节限位
{
    Joint upper;    // 上限
    Joint lower;    // 下限
};

struct CircleTarget     // moveC的目标点，由2个点位姿点构成
{
    PoseEuler pose1;
    PoseEuler pose2;
};

struct MotionTarget     // 用于move指令的数据结构
{
    MotionType  type;   // 指令的运动类型
    double      cnt;    // 平滑语句的CNT范围 ： 0.0 - 1.0， FINE语句的CNT    : < 0
    double      vel;    // 指令速度： 如果是moveJ，指令速度是百分比, 范围: 0.0 - 1.0
                        //           如果是moveL或moveC，指令速度是mm/s, 范围： 0.0 - MAX_VEL
    
    int user_frame_id;  // 如果是moveL或者moveC，需要指定目标点所处的用户坐标系标号和所用工具的标号，反解时需要
    int tool_frame_id;  // 如果用户坐标系标号和工具标号与当前的在用标号不符时直接报错

    union               // 根据type指定的运动类型，使用相应的目标数据
    {
        Joint           joint_target;   // 关节目标点，moveJ时使用
        PoseEuler       pose_target;    // 位姿目标点，moveL时使用
        CircleTarget    circle_target;  // 2个位姿目标点，moveC时使用
    };
};

struct PathBlock    // 路径点的数据结构
{
    PointType   point_type;     // 路径点所属的区段, 路径点或者过渡点
    MotionType  motion_type;    // 路径点的类型，关节类型或者笛卡尔类型

    Pose    pose;   // 路径点的笛卡尔空间位姿表示
    Joint   joint;  // 路径点的关节空间表示
};

struct PathCache    // 路径缓存
{
    MotionTarget    target;                 // 运动指令的目标
    int             smooth_in_index;        // 如果上一条指令带平滑，index指示了过度路径切入本条路径的位置，否则index应为-1，在过渡段的最后一个点
    int             smooth_out_index;       // 如果本条指令带平滑，index指示了由本条路径切出到过度路径的位置，否则index应为-1，在路径上的最后一个点
    size_t          cache_length;           // 路径缓存中的有效路径点数
    PathBlock       cache[PATH_CACHE_SIZE]; // 由一系列连续有序的路径点构成的路径
};

struct PathCacheList    // 路径缓存链表
{
    int             id;                     // 运动指令的ID/行号
    PathCacheList*  next_ptr;               // 下一个路径缓存所在的地址，用于构成路径缓存链表
    PathCache       path_cache;
};

struct AxisCoeff        // 一个轴轨迹的表达式
{
    int     reserve[4]; // 保留
    double  data[10];  // 表达式，根据算法实现进行细化
};

struct TrajectoryBlock  // 一段时间内的轨迹
{
    size_t      index_in_path_cache;    // 轨迹段末尾位置对应在路径缓存中的索引
    MotionTime  duration;               // 轨迹段在时间轴上的延续长度，在本段轨迹上的差值时间：0 ～ duration
    MotionTime  time_from_start;        // 从本条指令起始到block起始之间的时间
    AxisCoeff   axis[NUM_OF_JOINT];     // 各个轴的表达式
};

struct TrajectoryCache      // 轨迹缓存
{
    int                 smooth_out_index;               // 如果本条指令带平滑，index指示了由本条路径切出到过度路径的位置，否则index应为-1
    PathCache*          path_cache_ptr;                 // 轨迹对应的路径缓存地址
    size_t              cache_length;                   // 轨迹缓存中的有效轨迹段数
    TrajectoryBlock     cache[TRAJECTORY_CACHE_SIZE];   // 由一系列连续有序的轨迹段构成的轨迹
};

struct TrajectoryCacheList  // 轨迹缓存链表
{
    size_t                  pick_index;
    MotionTime              pick_time_from_block;
    MotionTime              time_from_start;                // 从运动起始到本条指令起始之间的时间
    TrajectoryCacheList*    next_ptr;                       // 下一个轨迹缓存所在的地址，用于构成轨迹缓存链表
    TrajectoryCache         trajectory_cache;
};

struct TrajectorySegment
{
    MotionTime  time_from_start;        // 从运动起始到本段segment起点之间的时间
    MotionTime  time_from_block;        // 从block的起始到本段segment起始之间的时间
    MotionTime  duration;               // 本段segment的持续时间
    AxisCoeff   axis[NUM_OF_JOINT];     // 各个轴的表达式
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

struct ManualCoef           // 手动示教模式下关节或者笛卡尔坐标的梯形轨迹
{
    MotionTime start_time;  // 开始加速的时刻
    MotionTime stable_time; // 加速完毕的时刻
    MotionTime brake_time;  // 开始减速的时刻
    MotionTime stop_time;   // 停止的时刻

    double  start_alpha;    // 匀加速阶段的加速度
    double  brake_alpha;    // 匀减速阶段的加速度
};

struct ManualTrajectory     // 手动示教模式下的运动轨迹
{
    ManualMode      mode;           // 手动示教的模式，步进、连续或者到目标点
    ManualFrame     frame;          // 手动示教的坐标系，关节空间、基座坐标系、用户坐标系、世界坐标系、工具坐标系
    ManualDirection direction[NUM_OF_JOINT];   // 手动示教的各轴方向

    Joint       joint_start;        // 示教运动的开始关节位置
    Joint       joint_ending;       // 示教运动的结束关节位置

    PoseEuler   cart_start;         // 示教运动的开始笛卡尔位姿
    PoseEuler   cart_ending;        // 示教运动结束始笛卡尔位姿
    PoseEuler   tool_coordinate;    // 工具坐标系到基座坐标系的变换，用于工具坐标系下的示教

    MotionTime      duration;       // 示教运动的耗时
    ManualCoef      coeff[NUM_OF_JOINT];        // 各轴系数
};

struct TrajectoryPoint  // 差值得到的轨迹点
{
    Joint   angle;      // 轨迹点的位置
    Joint   omega;      // 轨迹点的速度
    Joint   alpha;      // 轨迹点的加速度
    Joint   ma_cv_g;    // 轨迹点的力矩
    PointLevel  level;  // 轨迹点位置，起始点、中间点或者结束点
};


}

#endif
