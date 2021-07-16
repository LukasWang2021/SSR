#include <string.h>  
#include "axis_conversion.h"
#include "math.h"

using namespace axis_space;
using namespace system_model_space;
using namespace log_space;

AxisConversion::AxisConversion(void):
	db_ptr_(NULL),
    factor_pos_a2m_(1),
	factor_vel_a2m_(1),
	factor_acc_a2m_(1),
	factor_dec_a2m_(1),
	factor_jerk_a2m_(1),
	factor_pos_m2a_(1),
	factor_vel_m2a_(1),
	factor_torque_m2a_(1)
{
    
}

AxisConversion::~AxisConversion(void)
{
}

bool AxisConversion::init(int32_t id, system_model_space::AxisModel_t* db_ptr)
{
    id_ = id;
    if (db_ptr == NULL)
		return false;
	
	db_ptr_ = db_ptr;

    //axis
	if (!db_ptr_->application_ptr->get(AxisApplication1001__model_class, &class_))
		return false;
	if (!db_ptr_->application_ptr->get(AxisApplication1001__model_gear_ratio_numerator, &gear_ratio_numerator_))
		return false;
	if (!db_ptr_->application_ptr->get(AxisApplication1001__model_gear_ratio_denominator, &gear_ratio_denominator_))
		return false;
    if (!db_ptr_->application_ptr->get(AxisApplication1001__model_pitch, &pitch_))
		return false;
	if (!db_ptr_->application_ptr->get(AxisApplication1001__model_direction, &direction_))
		return false;

	//encoder
	if (!db_ptr_->application_ptr->get(AxisApplication1001__encoder_pulse_per_revolution, &pulse_per_unit_))
		return false;
	
    //motor
    if (!db_ptr_->application_ptr->get(AxisApplication1001__motor_rated_torque, &rated_torque_))
		return false;

    LogProducer::info("Axis", "Axis[%d] class=%d, numerator=%d, denominator=%d, pitch=%d, resolution=%d, rated_torque=%d",
        id_, class_, gear_ratio_numerator_, gear_ratio_denominator_, pitch_, pulse_per_unit_, rated_torque_);

    //compute factor
    if (!computerFactor()) return false;

	return true;
}


int64_t AxisConversion::convertPosA2M(double position)
{
    return (int64_t)(position * factor_pos_a2m_);
}

int32_t AxisConversion::convertVelA2M(double velocity)
{
    return (int32_t)(velocity * factor_vel_a2m_);
}

int32_t AxisConversion::convertAccA2M(double acc)
{
    return (int32_t)(acc * factor_acc_a2m_);
}

int32_t AxisConversion::convertDecA2M(double dec)
{
    return (int32_t)(dec * factor_dec_a2m_);
}

int32_t AxisConversion::convertJerkA2M(double jerk)
{
    return (int32_t)(jerk * factor_jerk_a2m_);
}

double AxisConversion::convertPosM2A(int64_t position)
{
    return (position * factor_pos_m2a_);
}
double AxisConversion::convertVelM2A(int32_t velocity)
{
    return (velocity * factor_vel_m2a_);
}

double AxisConversion::convertTorqueM2A(int32_t torque)
{
    return (torque * factor_torque_m2a_);
}

AxisDirection_e AxisConversion::getDirection(AxisDirection_e direction)
{
    switch (direction_)
    {
        case MOTOR_DIRECTION_POSITIVE:
            if (direction == AXIS_DIRECTION_POSITIVE)
                return AXIS_DIRECTION_POSITIVE;
            else if (direction_ == AXIS_DIRECTION_NEGATIVE)
                return AXIS_DIRECTION_NEGATIVE;
            break;
        case MOTOR_DIRECTION_NEGATIVE:
            if (direction == AXIS_DIRECTION_POSITIVE)
                return AXIS_DIRECTION_NEGATIVE;
            else if (direction == AXIS_DIRECTION_NEGATIVE)
                return AXIS_DIRECTION_POSITIVE;
            break;
        default:
            break;
    }

    return direction;
}

bool AxisConversion::computerFactor(void)
{
    //compute factor_pos_down_, factor_vel_down_, acc, dec, jerk
    if (class_ == AXIS_CLASS_REVOLUTE)
    {
        // (position/360deg) * ratio* pulsePerCircle
        factor_pos_a2m_ = ((double)gear_ratio_numerator_ * pulse_per_unit_) / (gear_ratio_denominator_ *2*M_PI);

        // velocity * ratio 
        factor_vel_a2m_ = ((double)gear_ratio_numerator_ * pulse_per_unit_)/ (gear_ratio_denominator_*2*M_PI);
        factor_acc_a2m_ = (double)gear_ratio_numerator_ / gear_ratio_denominator_;
        factor_dec_a2m_ = (double)gear_ratio_numerator_ / gear_ratio_denominator_;
        factor_jerk_a2m_ = (double)gear_ratio_numerator_ / gear_ratio_denominator_;
    }   
    else if (class_ == AXIS_CLASS_PRISMATIC)
    {
        // (position/pitch) * pulsePerCircle
		factor_pos_a2m_ = (double)pulse_per_unit_ / pitch_;
	
        // (velocity/pitch)
        factor_vel_a2m_ =  (double)(1.0) / pitch_;
        factor_acc_a2m_ = (double)(1.0) / pitch_;
        factor_dec_a2m_ = (double)(1.0) / pitch_;
        factor_jerk_a2m_ = (double)(1.0) / pitch_;
    }
    else if (class_ == AXIS_CLASS_PRISMATIC_COUPLE)
	{
	    // (position/pitch) * ratio * pulsePerCircle
		factor_pos_a2m_ = ((double)gear_ratio_numerator_ * pulse_per_unit_) / (pitch_ * gear_ratio_denominator_);
	
        // velocity * ratio
        factor_vel_a2m_ = (double)(gear_ratio_numerator_) /  gear_ratio_denominator_;
        factor_acc_a2m_ = (double)(gear_ratio_numerator_) / gear_ratio_denominator_;
        factor_dec_a2m_ = (double)(gear_ratio_numerator_) / gear_ratio_denominator_;
        factor_jerk_a2m_ = (double)(gear_ratio_numerator_) / gear_ratio_denominator_;
    }
    else
    {
        return false;
    }

    //the direction affact the position
    switch (direction_)
    {
        case MOTOR_DIRECTION_POSITIVE:
        break;
        case MOTOR_DIRECTION_NEGATIVE:
            factor_pos_a2m_ = - factor_pos_a2m_;
        break;
        default:
        break;
    }

    //compute pos, vel, torque up
	factor_pos_m2a_ = 1 / factor_pos_a2m_;
	factor_vel_m2a_ = 1 / factor_vel_a2m_;
	factor_torque_m2a_ = 0.001 * rated_torque_ * 0.01;

    return true;
}



