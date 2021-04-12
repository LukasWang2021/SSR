#ifndef AXIS_DATATYPE_H
#define AXIS_DATATYPE_H

/**
 * @file axis_datatype.h
 * @brief The file is the header file of the common used data structure.
 * @author Feng.Wu
 */

namespace axis_space {

/**
 * @brief AxisDirection_e is the enum of the motion direction.
 */
typedef enum{
	AXIS_DIRECTION_POSITIVE = 0,  /**< Axis rotates in the positive direction.*/
	AXIS_DIRECTION_NEGATIVE = 1,  /**< Axis rotates in the negative direction.*/
}AxisDirection_e;

/**
 * @brief AxisInfo_b contains the information from the servo.
 */
typedef struct{
	bool simulation: 1;
	bool communication_ready: 1;
	bool ready_power_on: 1;
	bool power_on: 1;
	int  rsvd: 28;
}AxisInfo_b;

/**
 * @brief AxisStatus_e is the enum of the status.
 */
typedef enum{
	AXIS_STATUS_UNKNOWN = 0,            /**< The status stayes in unkown if the axis is in a group.*/
	AXIS_STATUS_ERRORSTOP = 1,          /**< If error happens.*/
	AXIS_STATUS_DISABLED = 2,           /**< If the power stage is off.*/
	AXIS_STATUS_STANDSTILL = 3,         /**< If the power stage is on.*/
	AXIS_STATUS_STOPPING = 4,           /**< If the axis goes stopping.*/
	AXIS_STATUS_HOMING = 5,             /**< If the axis is in homing motion.*/
	AXIS_STATUS_DISCRETE_MOTION = 6,    /**< If the axis is in discreted motion.*/
	AXIS_STATUS_CONTINUOUS_MOTION = 7,  /**< If the axis is in continuous motion.*/
	AXIS_STATUS_SYNCHRONIZED_MOTION = 8 /**< If the axis is in synchronized motion.*/
}AxisStatus_e;

/**
 * @brief Axis type enumeration.
 */
typedef enum{
	AXIS_TYPE_SERVO = 0,  /**< Servo axis.*/
	AXIS_TYPE_STEPPER = 1,  /**< Stepper axis.*/
	AXIS_TYPE_UNKNOWN = 2,  /**< Unknown type.*/
}AxisType_e;


}
#endif
