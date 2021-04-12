#ifndef AXIS_CONVERSION_H
#define AXIS_CONVERSION_H

/**
 * @file axis_conversion.h
 * @brief The file is the header file of class "AxisConversion".
 * @author Feng.Wu
 */

#include "common_error_code.h"
#include "system_model_manager.h"
#include "axis_application_param_1001.h"
#include "system/servo_comm_base.h"
#include "axis_datatype.h"
#include "log_manager_producer.h"

/**
 * @brief axis_space includes all axis related definitions and implementation.
 */
namespace axis_space {

/**
 * @brief MotorDirection_e is the enum of the motor direction.
 */
typedef enum{
	MOTOR_DIRECTION_POSITIVE = 0,      /**< The motor moves in the positive direction.*/
	MOTOR_DIRECTION_NEGATIVE = 1,      /**< The motor moves in the negative direction.*/
}MotorDirection_e;

/**
 * @brief MotorClass_e is the enum of the axis class.
 */
typedef enum{
	AXIS_CLASS_REVOLUTE = 0,          /**< The motion is revolute.*/
	AXIS_CLASS_PRISMATIC = 1,         /**< The motion is prismatic.*/
	AXIS_CLASS_PRISMATIC_COUPLE = 2,  /**< The motion is prismatic with ratio, coupled with the other axis.*/
}AxisClass_e;

/**
 * @brief AxisConversion converts the unit between the axis and motor.
 * @details
 */
class AxisConversion
{
  public:
    /**
     * @brief Constructor of the class.
     */
    AxisConversion(void);
    /**
     * @brief Destructor of the class. 
     */   
    ~AxisConversion(void);

    /**
     * @brief Initializatin.
     * @details Extract the related parameters from the model data.\n
     * @param [in] id The reference of the axis.
     * @param [in] db_ptr The pointer of the parameters of the axis model.
     * @retval true success.
     * @retval false Failed to initialize.
     */
    bool init(int32_t id, system_model_space::AxisModel_t* db_ptr);

    /**
     * @brief Convert the axis position from the user to the motor side.
     * @details 
     * @param [in] position The axis position of the user side. Unit: pulse or um.
     * @return The position of the motor side. Unit: pulse.
     */
    int64_t convertPosA2M(double position);

    /**
     * @brief Convert the axis velocity from the user to the motor side.
     * @details 
     * @param [in] velocity The axis velocity of the user side. Unit: pulse/s.
     * @return The velocity of the motor side. Unit: revolutions/s.
     */
	int32_t convertVelA2M(double velocity);

    /**
     * @brief Convert the axis acceleration from the user to the motor side.
     * @details 
     * @param [in] acc The axis acceleration of the user side. Unit: pulse/s^2.
     * @return The acceleration of the motor side. Unit: revolutions/s^2.
     */
	int32_t convertAccA2M(double acc);

    /**
     * @brief Convert the axis deceleration from the user to the motor side.
     * @details 
     * @param [in] dec The axis deceleration of the user side. Unit: pulse/s^2.
     * @return The deceleration of the motor side. Unit: revolutions/s^2.
     */
	int32_t convertDecA2M(double dec);

    /**
     * @brief Convert the axis jerk from the user to the motor side.
     * @details 
     * @param [in] jerk The axis jerk of the user side. Unit: pulse/s^3.
     * @return The jerk of the motor side. Unit: revolutions/s^3.
     */
	int32_t convertJerkA2M(double jerk);

    /**
     * @brief Convert the axis position from the motor to the user side.
     * @details 
     * @param [in] position The axis deceleration of the motor side. Unit: pulse. 
     * @return The v of the user side. Unit: pulse or um.
     */
	double convertPosM2A(int64_t position);

    /**
     * @brief Convert the axis velocity from the motor to the user side.
     * @details 
     * @param [in] velocity The axis velocity of the motor side. Unit: revolutions/s. 
     * @return The velocity of the user side. Unit: pulse/s.
     */
	double convertVelM2A(int32_t velocity);

    /**
     * @brief Convert the torque from the motor to the user side.
     * @details 
     * @param [in] torque The torque of the motor side. Unit: 0.01N*m. 
     * @return The torque of the user side. Unit: N*m.
     */
	double convertTorqueM2A(int32_t torque);

    /**
     * @brief Convert the motor direction from the user to the motor side.
     * @details 
     * @param [in] direction The directoin setted by the user.
     * @return The actual of the direction.
     * - AXIS_DIRECTION_POSITIVE = 0,
     * - AXIS_DIRECTION_NEGATIVE = 1, 
     */
    AxisDirection_e getDirection(AxisDirection_e direction);
	
  private:  
  	system_model_space::AxisModel_t* db_ptr_;
    int32_t id_;  

    //axis
    int32_t class_;
	int32_t gear_ratio_numerator_;
	int32_t gear_ratio_denominator_;
    int32_t pitch_;
	int32_t direction_;
	
	//encode
    int32_t pulse_per_unit_;

	//motor
	int32_t rated_torque_;

	//facotr
	double factor_pos_a2m_;
	double factor_vel_a2m_;
	double factor_acc_a2m_;
	double factor_dec_a2m_;
	double factor_jerk_a2m_;
	
	double factor_pos_m2a_;
	double factor_vel_m2a_;
	double factor_torque_m2a_;
	
	bool computerFactor(void);
};

}
#endif
