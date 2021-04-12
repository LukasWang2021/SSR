#ifndef FUNCTION_BLOCK_H
#define FUNCTION_BLOCK_H

/**
 * @file function_block.h
 * @brief The file includes the definition of function block.
 * @author zhengyu.shen
 */

#include <stdint.h>
/**
 * @brief base_space includes all foundational definitions and realizations.
 */
namespace base_space
{
/**
 * @brief Defines the type of a function block.
 */
typedef enum
{
    FB_TYPE_INTERNAL = 0,   /**< Function block ask for an internal operation.*/
    FB_TYPE_MOTION = 1,     /**< Function block ask for a motion operation, such as moving an axis.*/
    FB_TYPE_NONMOTION = 2,  /**< Function block ask for a non-motion operation, such as IO operation.*/
}FunctionBlockType_e;       

/**
 * @brief Defines the executor of a function block.
 */
typedef enum
{
    FB_EXECUTOR_AXIS = 0,   /**< Function block is assigned to an axis.*/
    FB_EXECUTOR_GROUP = 1,  /**< Function block is assigned to an group.*/
}FunctionBlockExecutor_e;
/**
 * @brief Defines the command type of an internal function block.
 */
typedef enum
{
    INTERNAL_CMD_DUMMY = 0, /**< Not used.*/
    // more, user defined
}InternalCmd_e;
/**
 * @brief Defines the command type of a motion function block.
 */
typedef enum
{
    MOTION_CMD_DUMMY = 0,   /**< Not used.*/
    // more, user defined
    MOTION_CMD_MOVJ = 1,    /**< Joint moving command.*/
    MOTION_CMD_MOVL = 2,    /**< Line moving command.*/
}MotionCmd_e;
/**
 * @brief Defines the command type of a non-motion function block.
 */
typedef enum
{
    NONMOTION_CMD_DUMMY = 0,    /**< Not used.*/
    // more, user defined
}NonMotionCmd_e;

/**
 * @brief Defines the information of movj command for an axis.
 */
typedef struct
{
    double start;       /**< Start position.*/
    double end;         /**< End position.*/
    double a_percent;   /**< Percentage of the accerleration in terms of the max accerelation of the axis.*/
    double v_percent;   /**< Percentage of the velocity in terms of the max velocity of the axis.*/
    double jerk_time;   /**< Not used.*/
    double valve;       /**< Not used.*/
    /** Flag to tell if the axis should wait until it reaches the end position before execute the next motion function block.\n
     *  Axis will wait be in position if is_cnt set to false, vice versa. 
     */
    bool is_cnt;        
    /** Flag to tell if it is necessary to check the soft limits when doing trajectory plan.\n
     *  Trajectory plan algorithm should check soft limits if is_sf_limit_enable set to true, vice versa. 
     */
    bool is_sf_limit_enable;    
}AxisMotionCmdMovJ_t;
/**
 * @brief Defines the point type.
 */
typedef enum
{
    GROUP_POINT_JOINT = 0,      /**< Point expresses in joint space like {J1,J2,...Jn}*/
    GROUP_POINT_CARTESIAN = 1,  /**< Point expresses in cartesian space like {X,Y,Z,a,b,c}, where a,b,c is euler angle in ZYX rotation sequence.*/
}GroupPointType_e;
/**
 * @brief Struct of a point for S6A group in space.
 */
typedef struct
{
    uint32_t type;  /**< Point type refers to GroupPointType_e.*/
    int32_t turn;   /**< Turns of the J4 of S6A group.*/
    int32_t posture;/**< Posture of the S6A group. -1 is left hand, 1 is right hand.*/
    uint32_t num;   /**< Valid number of data for data[9].*/
    double data[9]; /**< Store position data according to the point type.*/
}GroupPoint_t;
/**
 * @brief Defines the information of movj command for a group.
 */
typedef struct
{
    GroupPoint_t start; /**< Start position.*/
    GroupPoint_t end;   /**< End position.*/
    double a_percent;   /**< Percentage of the accerleration in terms of the max accerelation for each axis.*/
    double v_percent;   /**< Percentage of the velocity in terms of the max velocity for each axis.*/
    double jerk_time;   /**< Not used.*/
    double valve;       /**< Not used.*/
    /** Flag to tell if the group should wait until it reaches the end position before execute the next motion function block.\n
     *  Group will wait be in position if is_cnt set to false, vice versa. 
     */
    bool is_cnt;
    /** Flag to tell if it is necessary to check the soft limits when doing trajectory plan.\n
     *  Trajectory plan algorithm should check soft limits if is_sf_limit_enable set to true, vice versa. 
     */
    bool is_sf_limit_enable;
}GroupMotionCmdMovJ_t;
/**
 * @brief Defines the information of movl command for a group.
 */
typedef struct
{
    GroupPoint_t start; /**< Start position.*/
    GroupPoint_t end;   /**< Start position.*/
    double a_max;       /**< The commanded spatial acceleration of the movement.*/
    double v_max;       /**< The commanded spatial velocity of the movement.*/
    double jerk_time;   /**< Not used.*/
    double valve;       /**< Not used.*/
    /** Flag to tell if the group should wait until it reaches the end position before execute the next motion function block.\n
     *  Group will wait be in position if is_cnt set to false, vice versa. 
     */
    bool is_cnt;
}GroupMotionCmdMovL_t;
/**
 * @brief Defines the function block of the system.
 */
typedef struct
{
    FunctionBlockType_e block_type;         /**< Function block type refers to FunctionBlockType_e.*/
    uint32_t cmd_type;                      /**< Command type refers to InternalCmd_e, MotionCmd_e or NonMotionCmd_e.*/
    FunctionBlockExecutor_e executor_type;  /**< Executor type.*/
    int32_t executor_id;                    /**< Executor index.*/
    union
    {
        // more, user defined
        AxisMotionCmdMovJ_t axis_movj;      /**< Command block for axis movj.*/
        GroupMotionCmdMovJ_t group_movj;    /**< Command block for group movj.*/
        GroupMotionCmdMovL_t group_movl;    /**< Command block for group movl.*/
    };
}FunctionBlock_t;

}

#endif

