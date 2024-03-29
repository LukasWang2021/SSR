//#include "stdafx.h"
#include "sub_interface.h"
#include "comm_def.h"
#include "sub_basic.h"
#include "common_error_code.h"


COMM_INTERFACE_API uint64_t c_initSub(char* server_ip)
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if (sub_ptr == NULL)
		return HANDLE_SUB_FAILED;
	if (0 != sub_ptr->init(std::string(server_ip)))
	    return HANDLE_SUB_FAILED;
	return 0;
}

COMM_INTERFACE_API uint64_t c_exitSub(void)
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if (sub_ptr == NULL)
		return HANDLE_SUB_FAILED;
	sub_ptr->exit();
	delete sub_ptr;
	sub_ptr = NULL;
	return 0;
}

COMM_INTERFACE_API uint64_t c_getTopicAxisFeedback(uint32_t array_size, uint32_t isr[AXIS_NUM],
	                    uint32_t state[AXIS_NUM],
	                    double position[AXIS_NUM],
	                    double velocity[AXIS_NUM],
	                    double torque[AXIS_NUM])
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if (sub_ptr == NULL)
		return HANDLE_SUB_FAILED;
	TopicData* data_ptr = sub_ptr->getTopicDataPtr();
	sub_ptr->lockTopicData();
	uint32_t size = array_size < data_ptr->axis_feedback.data_count ? array_size : data_ptr->axis_feedback.data_count;
	for (uint32_t i = 0; i < size; ++i)
	{
		isr[i] = data_ptr->axis_feedback.data[i].data1.data[0];
		state[i] = data_ptr->axis_feedback.data[i].data1.data[1];
		position[i] = data_ptr->axis_feedback.data[i].data2.data[0];
		velocity[i] = data_ptr->axis_feedback.data[i].data2.data[1];
		torque[i] = data_ptr->axis_feedback.data[i].data2.data[2];
	}
	sub_ptr->unlockTopicData();
	return 0;
}

COMM_INTERFACE_API uint64_t c_getTopicServoFeedback(uint32_t axis_size, int32_t data[AXIS_NUM][32])
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if (sub_ptr == NULL)
		return HANDLE_SUB_FAILED;
	TopicData* data_ptr = sub_ptr->getTopicDataPtr();
	sub_ptr->lockTopicData();
	uint32_t size = axis_size < data_ptr->servo_feedback.data_count ? axis_size : data_ptr->servo_feedback.data_count;
	for (uint32_t i = 0; i < size; ++i)
	{
		for (uint32_t j = 0; j < data_ptr->servo_feedback.data->data_count; ++j)
		{
			data[i][j] = data_ptr->servo_feedback.data[i].data[j];
		}
	}
	sub_ptr->unlockTopicData();
	return 0;
}

uint64_t c_getTopicCpuFeedback(uint32_t array_size, uint32_t ctrl_pdo_sync[AXIS_NUM], uint32_t* sampling_sync)
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if (sub_ptr == NULL)
		return HANDLE_SUB_FAILED;
	TopicData* data_ptr = sub_ptr->getTopicDataPtr();
	sub_ptr->lockTopicData();
	if (data_ptr->cpu_feedback.data_count < 1)
	{
		sub_ptr->unlockTopicData();
		return 0;
	}
	uint32_t axis_pdo_sync_num = data_ptr->cpu_feedback.data_count - 1;
	uint32_t size = array_size < axis_pdo_sync_num ? array_size : axis_pdo_sync_num;

	for (uint32_t i = 0; i < size; ++i)
	{
		ctrl_pdo_sync[i] = data_ptr->cpu_feedback.data[i];
	}
	*sampling_sync = data_ptr->cpu_feedback.data[axis_pdo_sync_num];

	sub_ptr->unlockTopicData();
	return 0;
}


uint64_t c_getAxisFeedBackByIsrCount(uint32_t array_size, uint32_t isr_count, uint32_t state[AXIS_NUM], double position[AXIS_NUM], double velocity[AXIS_NUM], double torque[AXIS_NUM])
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if (sub_ptr == NULL)
		return HANDLE_SUB_FAILED;
	if (array_size < AXIS_NUM)
		return HANDLE_SUB_FAILED;
	return sub_ptr->getAxisFeedBackByIsrCount(isr_count, state, position, velocity, torque);
}

uint64_t c_getSafetyIOFeedBack(uint32_t* low, uint32_t* high)
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if (sub_ptr == NULL)
		return HANDLE_SUB_FAILED;
	TopicData* data_ptr = sub_ptr->getTopicDataPtr();
	sub_ptr->lockTopicData();

	*low = data_ptr->iosafety_feedback.data[0];
	*high = data_ptr->iosafety_feedback.data[1];

	sub_ptr->unlockTopicData();

	return 0;
}

uint64_t c_getTorqueFeedBack(double* torque, int size)
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if(torque == NULL || size <= 0)
		return HANDLE_SUB_FAILED;

	TopicData* data_ptr = sub_ptr->getTopicDataPtr();
	sub_ptr->lockTopicData();

	//printf("torque: %lf %lf %lf %lf %lf %lf\n", 
	//	data_ptr->torque_feedback.data[0], data_ptr->torque_feedback.data[1],
	//	data_ptr->torque_feedback.data[2], data_ptr->torque_feedback.data[3], 
	//	data_ptr->torque_feedback.data[4], data_ptr->torque_feedback.data[5]);

	memcpy(torque, &data_ptr->torque_feedback.data[0], 6 * sizeof(double));
	sub_ptr->unlockTopicData();
	return 0;
}

uint64_t c_getFioInfoFeedBackDevieState(uint32_t* state)
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if (state == NULL)
		return HANDLE_SUB_FAILED;

	TopicData* data_ptr = sub_ptr->getTopicDataPtr();
	sub_ptr->lockTopicData();
	*state = data_ptr->fio_info_feedback.data[0];
	sub_ptr->unlockTopicData();
	return 0;
}

uint64_t c_getFioInfoFeedBackActualVelocity(uint32_t* vel)
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if (vel == NULL)
		return HANDLE_SUB_FAILED;

	TopicData* data_ptr = sub_ptr->getTopicDataPtr();
	sub_ptr->lockTopicData();
	*vel = data_ptr->fio_info_feedback.data[1];
	sub_ptr->unlockTopicData();
	return 0;
}

uint64_t c_getTopicSystemState(double* data)
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if (data == NULL)
		return HANDLE_SUB_FAILED;

	TopicData* data_ptr = sub_ptr->getTopicDataPtr();
	sub_ptr->lockTopicData();
	memcpy(data, data_ptr->sys_status_feedback.data, data_ptr->sys_status_feedback.data_count);
	sub_ptr->unlockTopicData();
	return 0;
}

uint64_t c_getTopicArmState(uint32_t* data)
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if (data == NULL)
		return HANDLE_SUB_FAILED;

	TopicData* data_ptr = sub_ptr->getTopicDataPtr();
	sub_ptr->lockTopicData();
	*data = (uint32_t)data_ptr->sys_status_feedback.data[0];
	sub_ptr->unlockTopicData();
	return 0;
}

uint64_t c_getTopicMotionControlState(uint32_t* data)
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if (data == NULL)
		return HANDLE_SUB_FAILED;

	TopicData* data_ptr = sub_ptr->getTopicDataPtr();
	sub_ptr->lockTopicData();
	*data = (uint32_t)data_ptr->sys_status_feedback.data[1];
	sub_ptr->unlockTopicData();
	return 0;
}

uint64_t c_getTopicServoState(uint32_t* data)
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if (data == NULL)
		return HANDLE_SUB_FAILED;

	TopicData* data_ptr = sub_ptr->getTopicDataPtr();
	sub_ptr->lockTopicData();
	*data = (uint32_t)data_ptr->sys_status_feedback.data[2];
	sub_ptr->unlockTopicData();
	return 0;
}

uint64_t c_getTopicGloableVel(double* data)
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if (data == NULL)
		return HANDLE_SUB_FAILED;

	TopicData* data_ptr = sub_ptr->getTopicDataPtr();
	sub_ptr->lockTopicData();
	*data = data_ptr->sys_status_feedback.data[3];
	sub_ptr->unlockTopicData();
	return 0;
}

uint64_t c_getTopicGloableAcc(double* data)
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if (data == NULL)
		return HANDLE_SUB_FAILED;

	TopicData* data_ptr = sub_ptr->getTopicDataPtr();
	sub_ptr->lockTopicData();
	*data = data_ptr->sys_status_feedback.data[4];
	sub_ptr->unlockTopicData();
	return 0;
}

uint64_t c_getTopicWorkMode(uint32_t* data)
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if (data == NULL)
		return HANDLE_SUB_FAILED;

	TopicData* data_ptr = sub_ptr->getTopicDataPtr();
	sub_ptr->lockTopicData();
	*data = (uint32_t)data_ptr->sys_status_feedback.data[5];
	sub_ptr->unlockTopicData();
	return 0;
}

uint64_t c_getTopicControlMode(uint32_t* data)
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if (data == NULL)
		return HANDLE_SUB_FAILED;

	TopicData* data_ptr = sub_ptr->getTopicDataPtr();
	sub_ptr->lockTopicData();
	*data = (uint32_t)data_ptr->sys_status_feedback.data[6];
	sub_ptr->unlockTopicData();
	return 0;
}

uint64_t c_getTopicToolFrame(uint32_t* data)
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if (data == NULL)
		return HANDLE_SUB_FAILED;

	TopicData* data_ptr = sub_ptr->getTopicDataPtr();
	sub_ptr->lockTopicData();
	*data = (uint32_t)data_ptr->sys_status_feedback.data[7];
	sub_ptr->unlockTopicData();
	return 0;
}

uint64_t c_getTopicUserFrame(uint32_t* data)
{
	SubBasic* sub_ptr = SubBasic::getInstance();
	if (data == NULL)
		return HANDLE_SUB_FAILED;

	TopicData* data_ptr = sub_ptr->getTopicDataPtr();
	sub_ptr->lockTopicData();
	*data = (uint32_t)data_ptr->sys_status_feedback.data[8];
	sub_ptr->unlockTopicData();
	return 0;
}


