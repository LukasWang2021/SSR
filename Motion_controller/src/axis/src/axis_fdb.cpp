#include "axis_fdb.h"

using namespace axis_space;
using namespace log_space;

AxisFdb::AxisFdb(void):
	pdo_ptr_(NULL),
	pdo_size_(128),
	fdb_pdo_id_(0),
	position_(0),
	velocity_(0),
	torque_(0),
	fdb_current_func_ptr_(NULL),
	fdb_sync_func_ptr_(NULL)
{
    state_word_.all = 0;
}

AxisFdb::~AxisFdb(void)
{
    if (pdo_ptr_ != NULL)
    {
        delete[] (uint8_t*)pdo_ptr_;
        pdo_ptr_ = NULL;
    }
}

bool AxisFdb::init(int32_t id, servo_comm_space::ServoCommBase* servo_comm_ptr)
{
    id_ = id;
	servo_comm_ptr_ = servo_comm_ptr;
	initFdbTable();

    //get pdo application id and then bind to HandleFdbFuncPtr
    servo_comm_space::ServoCommInfo_t info;
	servo_comm_ptr_->getServoCommInfo(&info);
	fdb_pdo_id_ = info.fdb_pdo_id;
	LogProducer::info("Axis", "Axis[%d] fdb pdo application id = %d", id_, fdb_pdo_id_);
    for (size_t i = 0; i < fdb_table_.size(); ++i)
	{
	    if (fdb_pdo_id_ == fdb_table_[i].app_id)
		{
		    fdb_current_func_ptr_ = fdb_table_[i].fdb_current_func_ptr;
            fdb_sync_func_ptr_ = fdb_table_[i].fdb_sync_func_ptr;
        }
    }
    if (fdb_current_func_ptr_ == NULL || fdb_sync_func_ptr_ == NULL)
	{
	    return false;
	}
	
	pdo_ptr_ = new uint8_t[pdo_size_]();
	memset(pdo_ptr_, 0, pdo_size_);
	
	return true;
}

void AxisFdb::processFdbPdoCurrent(uint32_t* current_time_stamp_ptr)
{
    (this->*fdb_current_func_ptr_)(current_time_stamp_ptr);
}

void AxisFdb::processFdbPdoSync(uint32_t expect_time_stamp)
{
    (this->*fdb_sync_func_ptr_)(expect_time_stamp);
}


int64_t AxisFdb::getServoPosition(void)
{   	
	return position_;
}

int32_t AxisFdb::getServoVelocity(void)
{
	return velocity_;
}

int32_t AxisFdb::getServoTorque(void)
{
	return torque_;
}

ServoSm_e AxisFdb::getServoState(void)
{
    return servo_state_;
}

ServoOpMode_e AxisFdb::getServoOpMode(void)
{
    return servo_op_mode_;
}

bool AxisFdb::isTargetReached(void)
{
    return (bool)(state_word_.bit.target_reached);
}

bool AxisFdb::isHomingSuccess(void)
{
	//Reference of CanOpen page.78
	if (state_word_.bit.operation_mode_spec == 0x1 
	    && state_word_.bit.target_reached)
    	return true;

	return false;
}

bool AxisFdb::isHoming(void)
{
	//Reference of CanOpen page.78
	if (state_word_.bit.operation_mode_spec == 0 
	    && state_word_.bit.target_reached == 0)
    	return true;

	return false;
}


uint8_t* AxisFdb::getFdbPdoPtr(int32_t* size)
{
    *size = pdo_size_;
    return (uint8_t*)pdo_ptr_;
}


