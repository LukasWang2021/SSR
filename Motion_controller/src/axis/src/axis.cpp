#include <unistd.h>
#include <string.h>  
#include "axis.h"

using namespace axis_space;
using namespace system_model_space;
using namespace log_space;
using namespace base_space;
using namespace servo_comm_space;

Axis::Axis(int32_t id, AxisType_e type):
	servo_comm_ptr_(NULL),
    id_(id),
    axis_error_(0),
    is_in_group_(false),
    type_(type)
{
}

Axis::~Axis(void)
{
    for(size_t i = 0; i < support_alg_list_.size(); ++i)
    {
        if (support_alg_list_[i] != NULL)
        {
            delete support_alg_list_[i];
        }
    }
    support_alg_list_.clear();
}

bool Axis::init(servo_comm_space::ServoCpuCommBase* cpu_comm_ptr, servo_comm_space::ServoCommBase* servo_comm_ptr, 
    system_model_space::AxisModel_t* db_ptr, system_model_space::AxisConfig_t* axis_config_ptr)
{
	//init ptr;
	if (cpu_comm_ptr == NULL || servo_comm_ptr == NULL || db_ptr == NULL || axis_config_ptr == NULL)
		return false;
    cpu_comm_ptr_ = cpu_comm_ptr;
    servo_comm_ptr_ = servo_comm_ptr;
	db_ptr_ = db_ptr;
    axis_config_ptr_ = axis_config_ptr;

	//init fdb
	if (!fdb_.init(id_, servo_comm_ptr_))
	{
	    LogProducer::error("Axis","Axis[%d] failed to init axis feedback", id_);
		return false;
	}

	//init conv
	if (!conv_.init(id_, db_ptr_))
	{
	    LogProducer::error("Axis","Axis[%d] failed to init axis conversion", id_);
		return false;
	}

    // init state machine.
    if (!sm_.init(id_, &fdb_, servo_comm_ptr_, db_ptr_))
    {
    	LogProducer::error("Axis","Axis[%d] failed to init axis state machine", id_);
		return false;
    }

    //init subclass application
    if (!initApplication())
    {
        LogProducer::error("Axis","Axis[%d] failed to init application", id_);
		return false;
    }

    //init loading parameters
    if (!reloadSystemModel())
    {
        LogProducer::error("Axis","Axis[%d] failed to load parameters", id_);
		return false;
    }

    LogProducer::info("Axis", "Axis[%d] to init success", id_);
	return true;	
}

bool Axis::reloadAlgorithms(void)
{
    for(size_t i = 0; i < support_alg_list_.size(); ++i)
    {
        if (!support_alg_list_[i]->syncModel(db_ptr_))
        {
            LogProducer::error("Axis","Axis[%d] failed to sync algorithms", id_);
            return false;
        }
    }
    return true;
}

ErrorCode Axis::mcReset(void)
{
    clearBQ();

    // to update application params
    if (!reloadSystemModel())
    {
        LogProducer::error("Axis", "Axis[%d] reload parameters failed", id_);
        return AXIS_ALG_NOT_DEFINED;
    }
        
	bool ret = servo_comm_ptr_->emitServoCmdFaultReset();
	if (!ret)
	{
		LogProducer::warn("Axis", "Axis[%d] mcResetServo called failed when emitServoCmdFaultReset", id_);
		return AXIS_SEND_CORE_RESET_FAILED;
	}

    usleep(100*1000);//high signal last 100ms.

    ret = servo_comm_ptr_->emitServoCmdResetFaultReset();
	if (!ret)
	{
		LogProducer::warn("Axis", "Axis[%d] mcResetServo called failed when emitServoCmdResetFaultReset", id_);
		return AXIS_SEND_CORE_RESET_FAILED;
	}
    
    usleep(10*1000); //wait servo to response.
	clearError();
	LogProducer::info("Axis", "Axis[%d] mcResetServo called", id_);
	return SUCCESS;
}

ErrorCode Axis::mcStop(void)
{ 
    AxisStatus_e axis_status = sm_.getAxisStatus();  
    if (axis_status == AXIS_STATUS_ERRORSTOP || axis_status == AXIS_STATUS_DISABLED)
    {
        LogProducer::warn("Axis", "Axis[%d] mcStopServo called failed when axis_status is %s", id_, sm_.getAxisStateString(axis_status).c_str());
		return AXIS_STATE_TRANSFER_INVALID;
    }

    bool ret = servo_comm_ptr_->emitServoCmdQuickStop();
	if (!ret)
	{
		LogProducer::warn("Axis", "Axis[%d] mcStopServo called failed when emitServoCmdQuickStop", id_);
		return AXIS_SEND_CORE_STOP_FAILED;
	}
	
    clearBQ();
    LogProducer::info("Axis", "Axis[%d] mcStopServo called", id_);
	return SUCCESS;
}

ErrorCode Axis::mcHalt(void)
{
    AxisStatus_e axis_status = sm_.getAxisStatus();  
    if (axis_status == AXIS_STATUS_CONTINUOUS_MOTION || axis_status == AXIS_STATUS_SYNCHRONIZED_MOTION)
    {
        bool ret = servo_comm_ptr_->emitServoCmdQuickStop();//todo, should be deccelerate not estop.
	    if (!ret)
        {   
            LogProducer::warn("Axis", "Axis[%d] mcHaltServo called failed when emitServoCmdQuickStop", id_);
		    return AXIS_SEND_CORE_HALT_FAILED;
        }

	    ret = sm_.transferStateToDiscreteMotion();
	    if (!ret)
	    {
	        LogProducer::warn("Axis", "Axis[%d] mcHaltServo called failed even though axis_status is %s", id_, sm_.getAxisStateString(axis_status).c_str());
	        return AXIS_STATE_TRANSFER_INVALID;
	    }
		return SUCCESS;
	}
    LogProducer::warn("Axis", "Axis[%d] mcHaltServo called failed when axis_status is %s", id_, sm_.getAxisStateString(axis_status).c_str());
	return AXIS_STATE_TRANSFER_INVALID;
}

ErrorCode Axis::mcSetPosition(double position)
{
    AxisStatus_e axis_status = sm_.getAxisStatus();  
    if (axis_status != AXIS_STATUS_DISABLED && axis_status != AXIS_STATUS_ERRORSTOP && axis_status != AXIS_STATUS_UNKNOWN)
    {
        LogProducer::warn("Axis", "Axis[%d] mcSetPosition called failed when axis_status is %s", id_, sm_.getAxisStateString(axis_status).c_str());
        return AXIS_STATE_TRANSFER_INVALID;
    }

    int32_t encoder_val = fdb_.getEncoderValue();
    ErrorCode ret = servo_comm_ptr_->doServoCmdWriteParameter(SERVO_PARAM_ENCODER_OFFSET_LSB, encoder_val);
    if(ret != SUCCESS)
    {
        LogProducer::warn("Axis", "Axis[%d] mcSetPosition write servo param failed", id_);
        return ret;
    }

    if(!db_ptr_->actuator.servo_ptr->set(SERVO_PARAM_ENCODER_OFFSET_LSB, encoder_val))
    {
        LogProducer::warn("Axis", "Axis[%d] mcSetPosition set servo param file failed", id_);
        return AXIS_SET_ZERO_FAILED;
    }  
    if(!db_ptr_->actuator.servo_ptr->save())
    {
        LogProducer::warn("Axis", "Axis[%d] mcSetPosition save servo param file failed", id_);
        return AXIS_SET_ZERO_FAILED;
    }

    ret = servo_comm_ptr_->doServoCmdSetZeroOffset(encoder_val);
    if(ret != SUCCESS)
    {
        LogProducer::warn("Axis", "Axis[%d] mcSetPosition info servo to set zero failed", id_);
        return ret;
    }

    return SUCCESS;
}

ErrorCode Axis::mcReadParamter(int32_t param_index, int32_t &param_value)
{
    return servo_comm_ptr_->doServoCmdReadParameter(param_index, &param_value);
}

ErrorCode Axis::mcWriteParamter(int32_t param_index, int32_t param_value)
{
    return servo_comm_ptr_->doServoCmdWriteParameter(param_index, param_value);
}

ErrorCode Axis::mcMoveAbsolute(double position, double velocity, double acc, double dec, double jerk)
{
    LogProducer::debug("Axis", "Axis[%d] mcMoveAbsolute called:pos=%lf rad, vel=%lf rad/s, acc=%lf rad/s^2, dec=%lf rad/s^2, jerk=%lf rad/s^3", id_, position, velocity, acc, dec, jerk);
    AxisStatus_e axis_status = sm_.getAxisStatus();  
    if (axis_status == AXIS_STATUS_STANDSTILL || axis_status == AXIS_STATUS_CONTINUOUS_MOTION 
		|| axis_status == AXIS_STATUS_SYNCHRONIZED_MOTION || axis_status == AXIS_STATUS_DISCRETE_MOTION)
    {
        int64_t pos = conv_.convertPosA2M(position);
	    int32_t vel = conv_.convertVelA2M(velocity);
	    int32_t servo_acc = conv_.convertAccA2M(acc);
	    int32_t servo_dec = conv_.convertDecA2M(dec);
	    int32_t servo_jerk = conv_.convertJerkA2M(jerk);
        ErrorCode result = servo_comm_ptr_->doServoCmdMoveAbsolute(pos, vel, servo_acc, servo_dec, servo_jerk);
	    if (result != SUCCESS)
	    {
	    	LogProducer::warn("Axis", "Axis[%d] mcMoveAbsolute called failed when doServoCmdMoveAbsolute, result = 0x%llx", id_, result);
		    return result;
	    }

        bool ret = sm_.transferStateToDiscreteMotion();
	    if (!ret)
	    {
	        LogProducer::warn("Axis", "Axis[%d] mcMoveAbsolute called failed even though axis_status is %s", id_, sm_.getAxisStateString(axis_status).c_str());
	        return AXIS_STATE_TRANSFER_INVALID;
	    }
		LogProducer::info("Axis", "mcMoveAbsolute called success:pos=%lld, vel=%d, acc=%d, dec=%d, jerk=%d", pos, vel, servo_acc, servo_dec, servo_jerk);
		return SUCCESS;
	}
    LogProducer::warn("Axis", "Axis[%d] mcMoveAbsolute called failed when axis_status is %s", id_, sm_.getAxisStateString(axis_status).c_str());
	return AXIS_STATE_TRANSFER_INVALID;
}

ErrorCode Axis::mcMoveRelative(double position, double velocity, double acc, double dec, double jerk)
{
    LogProducer::debug("Axis", "Axis[%d] mcMoveRelative called:pos=%lf rad, vel=%lf rad/s, acc=%lf rad/s^2, dec=%lf rad/s^2, jerk=%lf rad/s^3", id_, position, velocity, acc, dec, jerk);
    AxisStatus_e axis_status = sm_.getAxisStatus();  
    if (axis_status == AXIS_STATUS_STANDSTILL || axis_status == AXIS_STATUS_CONTINUOUS_MOTION 
		|| axis_status == AXIS_STATUS_SYNCHRONIZED_MOTION || axis_status == AXIS_STATUS_DISCRETE_MOTION)
    {
        int64_t pos = conv_.convertPosA2M(position);
	    int32_t vel = conv_.convertVelA2M(velocity);
	    int32_t servo_acc = conv_.convertAccA2M(acc);
	    int32_t servo_dec = conv_.convertDecA2M(dec);
	    int32_t servo_jerk = conv_.convertJerkA2M(jerk);
        ErrorCode result = servo_comm_ptr_->doServoCmdMoveRelative(pos, vel, servo_acc, servo_dec, servo_jerk);
	    if (result != SUCCESS)
	    {
	    	LogProducer::warn("Axis", "Axis[%d] mcMoveRelative called failed when doServoCmdMoveAbsolute, result = 0x%llx", id_, result);
		    return result;
	    }

        bool ret = sm_.transferStateToDiscreteMotion();
	    if (!ret)
	    {
	        LogProducer::warn("Axis", "Axis[%d] mcMoveRelative called failed even though axis_status is %s", id_, sm_.getAxisStateString(axis_status).c_str());
	        return AXIS_STATE_TRANSFER_INVALID;
	    }
		LogProducer::info("Axis", "mcMoveRelative called success:pos=%lld, vel=%d, acc=%d, dec=%d, jerk=%d", pos, vel, servo_acc, servo_dec, servo_jerk);
		return SUCCESS;
	}
    LogProducer::warn("Axis", "Axis[%d] mcMoveRelative called failed when axis_status is %s", id_, sm_.getAxisStateString(axis_status).c_str());
	return AXIS_STATE_TRANSFER_INVALID;
}

ErrorCode Axis::mcMoveVelocity(double velocity, double acc, double dec, double jerk, AxisDirection_e direction)
{
    LogProducer::debug("Axis", "Axis[%d] mcMoveVelocity called:vel=%lf rad/s, acc=%lf rad/s^2, dec=%lf rad/s^2, jerk=%lf rad/s^3, dir=%d", id_, velocity, acc,dec, jerk, direction);
    AxisStatus_e axis_status = sm_.getAxisStatus();  
    if (axis_status == AXIS_STATUS_STANDSTILL || axis_status == AXIS_STATUS_CONTINUOUS_MOTION 
		|| axis_status == AXIS_STATUS_SYNCHRONIZED_MOTION || axis_status == AXIS_STATUS_DISCRETE_MOTION)
    {
	    int32_t vel = conv_.convertVelA2M(velocity);
	    int32_t servo_acc = conv_.convertAccA2M(acc);
	    int32_t servo_dec = conv_.convertDecA2M(dec);
	    int32_t servo_jerk = conv_.convertJerkA2M(jerk);
        AxisDirection_e dir = conv_.getDirection(direction);
        ErrorCode result = servo_comm_ptr_->doServoCmdMoveVelocity(vel, servo_acc, servo_dec, servo_jerk, dir);
        if (result != SUCCESS)
        {
        	LogProducer::warn("Axis", "Axis[%d] mcMoveVelocity called failed when doServoCmdMoveVelocity, result = 0x%llx", id_, result);
		    return result;
        }
	
        bool ret = sm_.transferStateToContinuousMotion();
	    if (!ret)
	    {
	        LogProducer::warn("Axis", "Axis[%d] mcMoveVelocity called failed even though axis_status is %s", id_, sm_.getAxisStateString(axis_status).c_str());
	        return AXIS_STATE_TRANSFER_INVALID;
	    }
		LogProducer::info("Axis", "Axis[%d] mcMoveVelocity called success:vel=%d, acc=%d, dec=%d, jerk=%d, dir=%d", id_, vel, servo_acc, servo_dec, servo_jerk, dir);
		return SUCCESS;
	}
    LogProducer::warn("Axis", "Axis[%d] mcMoveVelocity called failed when axis_status is %s", id_, sm_.getAxisStateString(axis_status).c_str());
	return AXIS_STATE_TRANSFER_INVALID;
}

ErrorCode Axis::mcReadActualPosition(double &position)
{
    position = conv_.convertPosM2A(fdb_.getServoPosition());
	return SUCCESS;
}

ErrorCode Axis::mcReadActualVelocity(double &velocity)
{
    velocity = conv_.convertVelM2A(fdb_.getServoVelocity());
	return SUCCESS;
}

ErrorCode Axis::mcReadActualTorque(double &torque)
{
    torque = conv_.convertTorqueM2A(fdb_.getServoTorque());
	return SUCCESS;
}

ErrorCode Axis::mcReadAxisInfo(AxisInfo_b &info)
{
    return SUCCESS;//todo
}

ErrorCode Axis::mcReadStatus(AxisStatus_e &status)
{
    status = sm_.getAxisStatus();
	return SUCCESS;
}

ErrorCode Axis::mcReadAxisError(int32_t &error)
{
    servo_comm_ptr_->doServoCmdReadParameter(SERVO_PARAM_CURRENT_ERROR, &error);
    return SUCCESS;
}

ErrorCode Axis::mcHome(void)
{
    AxisStatus_e axis_status = sm_.getAxisStatus();
    if (axis_status == AXIS_STATUS_STANDSTILL)
    {        
        ErrorCode ret = servo_comm_ptr_->doServoCmdHoming();
	    if (ret != SUCCESS)
	    {
	    	LogProducer::warn("Axis", "Axis[%d] mcHome called failed when emitServoCmdHoming", id_);
		    return ret;
	    }

		LogProducer::info("Axis", "mcHome called success");
		return SUCCESS;
	}
    LogProducer::warn("Axis", "Axis[%d] mcHome called failed when axis_status is %s", id_, sm_.getAxisStateString(axis_status).c_str());
	return AXIS_STATE_TRANSFER_INVALID;
}

ErrorCode Axis::rtmAbortHoming()
{
    AxisStatus_e axis_status = sm_.getAxisStatus();
    if (axis_status == AXIS_STATUS_HOMING)
    {
        ErrorCode ret = servo_comm_ptr_->doServoCmdAbortHoming();
	    if (ret != SUCCESS)
        {   
            LogProducer::warn("Axis", "Axis[%d] rtmAbortHoming called failed when emitServoCmdAbortHoming", id_);
		    return ret;
        }
		return SUCCESS;
	}
    LogProducer::warn("Axis", "Axis[%d] rtmAbortHoming called failed when axis_status is %s", id_, sm_.getAxisStateString(axis_status).c_str());
	return AXIS_STATE_TRANSFER_INVALID;
}

ErrorCode Axis::rtmReadAxisErrorHistory(int32_t *error_ptr, int32_t size)
{
    int32_t param_index = SERVO_PARAM_HISTORY_ERROR;
    for(int32_t i = 0; i < size; ++i)
    {
        servo_comm_ptr_->doServoCmdReadParameter(param_index, error_ptr);
        ++error_ptr;
        ++param_index;
    }
    return SUCCESS;
}

uint8_t* Axis::rtmReadAxisFdbPdoPtr(int32_t* size)
{
    return fdb_.getFdbPdoPtr(size);
}

int32_t Axis::rtmGetEncoderState(void)
{
    return fdb_.getEncoderState();
}

ErrorCode Axis::rtmResetEncoder(void)
{
    AxisStatus_e axis_status = sm_.getAxisStatus();  
    if (axis_status != AXIS_STATUS_ERRORSTOP && axis_status != AXIS_STATUS_DISABLED)
    {
        LogProducer::warn("Axis", "Axis[%d] rtmResetEncoder called failed when axis_status is %s", id_, sm_.getAxisStateString(axis_status).c_str());
		return AXIS_STATE_TRANSFER_INVALID;
    }
    return servo_comm_ptr_->doServoCmdResetEncoder();
}

bool Axis::rtmIsTargetReached(void)
{
    return fdb_.isTargetReached();
}

ServoSm_e Axis::readServoStatus(void)
{
    return fdb_.getServoState();
}

ServoCommBase* Axis::getServoCommPtr(void)
{
    return servo_comm_ptr_;
}

AxisConversion* Axis::getAxisConvPtr(void)
{
    return &conv_;
}

int32_t Axis::getID()
{
    return id_;
}

AxisType_e Axis::getType()
{
    return type_;
}

void Axis::setAxisInGroup(bool is_in_group)
{
    is_in_group_ = is_in_group;
    sm_.setValid(!is_in_group_);
}

bool Axis::isAxisInGroup(void)
{
    return is_in_group_;
}

void Axis::setError(ErrorCode err)
{
    sm_.setError();
    axis_error_ = err;
    ErrorQueue::instance().push(axis_error_);
}

void Axis::clearError(void)
{
    sm_.clearError();
    axis_error_ = SUCCESS;
}

void Axis::lockCtrlPdoMutex(void)
{
    ctrl_pdo_mutex_.lock();
}

void Axis::unlockCtrlPdoMutex(void)
{
    ctrl_pdo_mutex_.unlock();
}

void Axis::lockFdbPdoMutex(void)
{
    fdb_pdo_mutex_.lock();
}

void Axis::unlockFdbPdoMutex(void)
{
    fdb_pdo_mutex_.unlock();
}

void Axis::processStateMachine()
{
    sm_.processStatemachine();
}

void Axis::processFdbPdoCurrent(uint32_t* current_time_stamp_ptr)
{
    fdb_.processFdbPdoCurrent(current_time_stamp_ptr);
}

void Axis::processFdbPdoSync(uint32_t expect_time_stamp)
{
    fdb_.processFdbPdoSync(expect_time_stamp);
}


