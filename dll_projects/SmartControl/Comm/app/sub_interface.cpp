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

	for (uint32_t i = 0; i < axis_pdo_sync_num; ++i)
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

