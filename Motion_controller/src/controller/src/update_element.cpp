#include "controller_publish.h"
#include "basic_alg_datatype.h"
#include "fio_cmd.h"

using namespace user_space;
using namespace basic_alg;
using namespace axis_space;
using namespace group_space;

extern uint32_t* g_isr_ptr_;

void ControllerPublish::updateAxisFdb()
{
    static AxisStatus_e status[AXIS_NUM] = {AXIS_STATUS_UNKNOWN};
    static double position[AXIS_NUM] = {0}; 
    static double velocity[AXIS_NUM] = {0}; 
    static double torque[AXIS_NUM] = {0};
    
    axis_fdb_.data_count = AXIS_NUM;

    //because of coupling between the fifth and sixth axis, the positions should come from group.
    std::vector<double> pos;
    if (group_ptr_[0]->mcGroupReadActualPosition(COORD_TYPE_ACS, pos) == SUCCESS)
    {
        for(size_t i = 0; i < pos.size(); ++i)
        {
            position[i] = pos[i];
        }
    }

    for (size_t i = 0; i < AXIS_NUM; ++i)
    {
        axis_ptr_[i]->mcReadStatus(status[i]);
        if (!axis_ptr_[i]->isAxisInGroup())
        {
            axis_ptr_[i]->mcReadActualPosition(position[i]);
        }
        axis_ptr_[i]->mcReadActualVelocity(velocity[i]);
        axis_ptr_[i]->mcReadActualTorque(torque[i]);

        axis_fdb_.data[i].data1.data_count = 2;
        axis_fdb_.data[i].data1.data[0] = *g_isr_ptr_;
        axis_fdb_.data[i].data1.data[1] = status[i];
        axis_fdb_.data[i].data2.data_count = 3;
        axis_fdb_.data[i].data2.data[0] = position[i];
        axis_fdb_.data[i].data2.data[1] = velocity[i];
        axis_fdb_.data[i].data2.data[2] = torque[i];
    }
}

void ControllerPublish::updateServo1001ServoFdb()
{
    int32_t size = 0;
    servo1001_servo_fdb_.data_count = AXIS_NUM;
    for (size_t i = 0; i < servo1001_servo_fdb_.data_count; ++i)
    {
        uint8_t *ptr = axis_ptr_[i]->rtmReadAxisFdbPdoPtr(&size);
        servo1001_servo_fdb_.data[i].data_count = (size / sizeof(int32_t));
        for (size_t j = 0 ; j < servo1001_servo_fdb_.data[i].data_count; ++j)
        {
            servo1001_servo_fdb_.data[i].data[j] = *(int32_t *)ptr;
            ptr += sizeof(int32_t);
        }
    }
}

void ControllerPublish::updateServo1001CpuFdb()
{
    servo1001_cpu_fdb_.data_count = AXIS_NUM + 1;
    for (size_t i = 0; i < AXIS_NUM; ++i)
    {
        servo1001_cpu_fdb_.data[i] = cpu_comm_ptr_->getCtrlPdoSync(i);
    }
    servo1001_cpu_fdb_.data[AXIS_NUM] = cpu_comm_ptr_->getSamplingSync();
}

void ControllerPublish::updateIODigitalFdb()
{
    io_digital_fdb_.data_count = IO_DI_MAX_SIZE + IO_DO_MAX_SIZE;
    uint8_t value = 0;

    for (uint32_t i = 0; i < IO_DI_MAX_SIZE; ++i)
    {
        if (SUCCESS == io_dev_ptr_->readDiBit(i + 1, value))
        {
            io_digital_fdb_.data[i] = value;
        }      
    }
    for(uint32_t i = 0; i < IO_DO_MAX_SIZE; ++i)
    {
        if (SUCCESS == io_dev_ptr_->readDoBit(i + 1, value))
        {
            io_digital_fdb_.data[IO_DI_MAX_SIZE + i] = value;
        }
    }
}

void ControllerPublish::updateIOSafetyFdb()
{
    io_safety_fdb_.data_count = 2;
    safety_ptr_->readStatusAll(io_safety_fdb_.data[0], io_safety_fdb_.data[1]);
}

void ControllerPublish::updateTorqueFdb()
{
	torque_fdb_.data_count = 6;
	
	force_sensor_ptr_->updateSourceValue(GROUP_0);
	
	force_sensor_ptr_->calibratedForceSensor(GROUP_0);
	
	force_sensor_ptr_->transCalibrated2Tool(GROUP_0, &torque_fdb_.data[0], torque_fdb_.data_count);
}

void ControllerPublish::updateFioInfoFdb()
{
    hal_space::FioStatus_u tmp_st;
    hal_space::FioTopicVal_t tmp_sub;

	fio_info_fdb_.data_count = 2;
	
	tmp_st = fio_dev_ptr_->getStatus();
    fio_info_fdb_.data[0] = tmp_st.all;
    tmp_sub = fio_dev_ptr_->getTopicVal();
    fio_info_fdb_.data[1] = tmp_sub.grind_speed;
}

