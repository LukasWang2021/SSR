#include "axis_fdb.h"

using namespace axis_space;

void AxisFdb::initFdbTable()
{
    FdbPdo fdb_pdo;
	fdb_pdo = {3000, &AxisFdb::handleFdbCurrentCircleBuffer3000, &AxisFdb::handleFdbSyncCircleBuffer3000}; fdb_table_.push_back(fdb_pdo);
	fdb_pdo = {3001, &AxisFdb::handleFdbCurrentCircleBuffer3001, &AxisFdb::handleFdbSyncCircleBuffer3001}; fdb_table_.push_back(fdb_pdo);
	fdb_pdo = {3002, &AxisFdb::handleFdbCurrentCircleBuffer3002, &AxisFdb::handleFdbSyncCircleBuffer3002}; fdb_table_.push_back(fdb_pdo);
}


void AxisFdb::handleFdbCurrentCircleBuffer3000(uint32_t* current_time_stamp_ptr)
{
    servo_comm_ptr_->processFdbPdoCurrent((uint8_t*)pdo_ptr_, current_time_stamp_ptr);

    CircleBufferAppData3000_t* fdb_pdo_ptr = static_cast<CircleBufferAppData3000_t*>(pdo_ptr_);
	servo_state_ = servo_comm_ptr_->getServoState(fdb_pdo_ptr->state_word);
    pdo_size_ = sizeof(CircleBufferAppData3000_t);

    position_ = fdb_pdo_ptr->fdb_position;
	velocity_ = fdb_pdo_ptr->fdb_velocity;
	torque_ = fdb_pdo_ptr->fdb_torque;
	state_word_ = fdb_pdo_ptr->state_word;
    servo_op_mode_ = (ServoOpMode_e)(fdb_pdo_ptr->actual_op_mode);

    //for debug
	/*static uint32_t pre_state = 0;
	if ((id_ == 0) && (pre_state != fdb_pdo_ptr->state_word.all))
	{
	    pre_state = fdb_pdo_ptr->state_word.all;
		printf("aixs[0] current servo state change to 0x%x\n", pre_state);
	}*/
}

void AxisFdb::handleFdbSyncCircleBuffer3000(uint32_t expect_time_stamp)
{  
    servo_comm_ptr_->processFdbPdoSync((uint8_t*)pdo_ptr_, expect_time_stamp);

    CircleBufferAppData3000_t* fdb_pdo_ptr = static_cast<CircleBufferAppData3000_t*>(pdo_ptr_);
    servo_state_ = servo_comm_ptr_->getServoState(fdb_pdo_ptr->state_word);
    pdo_size_ = sizeof(CircleBufferAppData3000_t);

    position_ = fdb_pdo_ptr->fdb_position;
    velocity_ = fdb_pdo_ptr->fdb_velocity;
    torque_ = fdb_pdo_ptr->fdb_torque;
    state_word_ = fdb_pdo_ptr->state_word;
    servo_op_mode_ = (ServoOpMode_e)(fdb_pdo_ptr->actual_op_mode);
}

void AxisFdb::handleFdbCurrentCircleBuffer3001(uint32_t* current_time_stamp_ptr)
{
    servo_comm_ptr_->processFdbPdoCurrent((uint8_t*)pdo_ptr_, current_time_stamp_ptr);

    CircleBufferAppData3001_t* fdb_pdo_ptr = static_cast<CircleBufferAppData3001_t*>(pdo_ptr_);
	servo_state_ = servo_comm_ptr_->getServoState(fdb_pdo_ptr->state_word);
    pdo_size_ = sizeof(CircleBufferAppData3001_t);

    position_ = fdb_pdo_ptr->fdb_position;
    cmd_position_ = fdb_pdo_ptr->cmd_position;
	velocity_ = fdb_pdo_ptr->fdb_velocity;
	torque_ = fdb_pdo_ptr->fdb_torque;
    digital_input_ = fdb_pdo_ptr->digital_input;
	state_word_ = fdb_pdo_ptr->state_word;
    servo_op_mode_ = (ServoOpMode_e)(fdb_pdo_ptr->actual_op_mode);
}

void AxisFdb::handleFdbSyncCircleBuffer3001(uint32_t expect_time_stamp)
{  
    servo_comm_ptr_->processFdbPdoSync((uint8_t*)pdo_ptr_, expect_time_stamp);

    CircleBufferAppData3001_t* fdb_pdo_ptr = static_cast<CircleBufferAppData3001_t*>(pdo_ptr_);
    servo_state_ = servo_comm_ptr_->getServoState(fdb_pdo_ptr->state_word);
    pdo_size_ = sizeof(CircleBufferAppData3001_t);

    position_ = fdb_pdo_ptr->fdb_position;
    cmd_position_ = fdb_pdo_ptr->cmd_position;
    velocity_ = fdb_pdo_ptr->fdb_velocity;
    torque_ = fdb_pdo_ptr->fdb_torque;
    digital_input_ = fdb_pdo_ptr->digital_input;
	state_word_ = fdb_pdo_ptr->state_word;
    servo_op_mode_ = (ServoOpMode_e)(fdb_pdo_ptr->actual_op_mode);
}

void AxisFdb::handleFdbCurrentCircleBuffer3002(uint32_t* current_time_stamp_ptr)
{
    servo_comm_ptr_->processFdbPdoCurrent((uint8_t*)pdo_ptr_, current_time_stamp_ptr);

    CircleBufferAppData3002_t* fdb_pdo_ptr = static_cast<CircleBufferAppData3002_t*>(pdo_ptr_);
	servo_state_ = servo_comm_ptr_->getServoState(fdb_pdo_ptr->state_word);
    pdo_size_ = sizeof(CircleBufferAppData3002_t);

    position_ = fdb_pdo_ptr->fdb_position;
    cmd_position_ = fdb_pdo_ptr->cmd_position;
	stepper_currentA_ = fdb_pdo_ptr->fdb_currentA;
	stepper_currentB_ = fdb_pdo_ptr->fdb_currentB;
    digital_input_ = fdb_pdo_ptr->digital_input;
	state_word_ = fdb_pdo_ptr->state_word;
    servo_op_mode_ = (ServoOpMode_e)(fdb_pdo_ptr->actual_op_mode);
}

void AxisFdb::handleFdbSyncCircleBuffer3002(uint32_t expect_time_stamp)
{  
    servo_comm_ptr_->processFdbPdoSync((uint8_t*)pdo_ptr_, expect_time_stamp);

    CircleBufferAppData3002_t* fdb_pdo_ptr = static_cast<CircleBufferAppData3002_t*>(pdo_ptr_);
    servo_state_ = servo_comm_ptr_->getServoState(fdb_pdo_ptr->state_word);
    pdo_size_ = sizeof(CircleBufferAppData3002_t);

    position_ = fdb_pdo_ptr->fdb_position;
    cmd_position_ = fdb_pdo_ptr->cmd_position;
	stepper_currentA_ = fdb_pdo_ptr->fdb_currentA;
	stepper_currentB_ = fdb_pdo_ptr->fdb_currentB;
    digital_input_ = fdb_pdo_ptr->digital_input;
	state_word_ = fdb_pdo_ptr->state_word;
    servo_op_mode_ = (ServoOpMode_e)(fdb_pdo_ptr->actual_op_mode);
}
