#ifndef GROUP_DATATYPE_H
#define GROUP_DATATYPE_H

/**
 * @file group_datatype.h
 * @brief The file is the header file of the common used data structure.
 * @author Feng.Wu
 */

namespace group_space {

/**
 * @brief CoordType_e is the enum of the coordinate system.
 */
typedef enum{
	COORD_TYPE_ACS = 0,  /**< Axis Coordinate System.*/
	COORD_TYPE_MCS = 1,  /**< Machine Coordinate System.*/
	COORD_TYPE_PCS = 2,  /**< Product Coordinate System.*/
}CoordType_e;

/**
 * @brief AxisStatus_e is the enum of the status.
 */
typedef enum{
    GROUP_STATUS_UNKNOWN = 0,
    GROUP_STATUS_STANDBY = 1,     /**< If the power stage of all axes are on.*/
    GROUP_STATUS_MOVING = 2,      /**< If the group is in moving motion.*/
    GROUP_STATUS_DISABLED = 3,    /**< If the power stage of any axis is off.*/
    GROUP_STATUS_ERROR_STOP = 4,  /**< If any axis has error.*/
    GROUP_STATUS_STOPPING = 5,    /**< If the group goes stopping.*/
    GROUP_STATUS_HOMING = 6,      /**< If the group is in homing motion.*/
    
}GroupStatus_e;



}
#endif
