#ifndef TRAJECTORY_BLOCK_H
#define TRAJECTORY_BLOCK_H

/**
 * @file trajectory_block.h
 * @brief The file includes the definition of trajectory block.
 * @author zhengyu.shen
 */

#include <stdint.h>
#include "function_block.h"

/**
 * @def MAX_TRAJECTORY_POINTS 
 * The maximum number of the trajectory points of an axis.
 */
#define MAX_TRAJECTORY_POINTS 10000
/**
 * @def AXIS_TRAJECTORY_NUM 
 * The number of joints when the control object is an axis.
 */
#define AXIS_TRAJECTORY_NUM 1
/**
 * @def GROUP_TRAJECTORY_NUM 
 * The number of joints when the control object is a group.
 */
#define GROUP_TRAJECTORY_NUM 4
/**
 * @brief base_space includes all foundational definitions and realizations.
 */
namespace base_space
{
/**
 * @brief The definition of a trajectory point.
 */
typedef struct
{
    double pos; /**< position of joint, unit in mm or rad.*/
    double vel; /**< velocity of joint, unit in mm/s or rad/s.*/
    double acc; /**< acceleration of joint, unit in mm/s^2 or rad/s^2.*/
}TrajPoint_t;
/**
 * @brief The definition of a set point which is going to be sent to servo.
 * @attention It should be defined the same as ctrl-pdo structure. In this application, say CircleBufferAppData4000_t.
 */
typedef struct
{
    int64_t pos;        /**< the position of joint, unit in pulse.*/
    int32_t ff_vel;     /**< Not used*/
    int32_t ff_tor;     /**< Not used*/
}SetPoint_t;
/**
 * @brief Defines a trajectory for an axis.
 */
typedef struct
{
    TrajPoint_t traj_point[MAX_TRAJECTORY_POINTS];   /**< Trajectory points are computed by trajectory planning algorithm. It is must.*/
    SetPoint_t set_point[MAX_TRAJECTORY_POINTS];     /**< Set points are generated by unit conversion of the trajectory points. It is must.*/
    uint32_t next_index;            /**< The index of the next point to be sent to servo. It is must.*/
    uint32_t point_count;           /**< The total amount of valid points that are in traj_point[] and set_point[]. It is must.*/
}AxisTrajectory_t;
/**
 * @brief Defines a trajectory block for an axis.
 */
typedef struct
{
    AxisTrajectory_t axis[AXIS_TRAJECTORY_NUM]; /**< A trajectory for an axis. It is must.*/
    bool is_cnt;    /**< A flag to tell if the trajectory block is the last one in a whole moving action. It is must.*/
}AxisTrajectoryBlock_t;
/**
 * @brief Defines a trajectory block for a group.
 */
typedef struct
{
    AxisTrajectory_t axis[GROUP_TRAJECTORY_NUM]; /**< Trajectorys for an S6A group. It is must.*/
    bool is_cnt;    /**< A flag to tell if the trajectory block is the last one in a whole moving action. It is must.*/
}GroupTrajectoryBlock_t;

}

#endif

