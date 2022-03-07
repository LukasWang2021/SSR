//#include "stdafx.h"
#include "event_interface.h"
#include "comm_def.h"
#include "event_basic.h"
#include "common_error_code.h"


COMM_INTERFACE_API uint64_t c_initEvent(char* server_ip)
{
	EventBasic* event_ptr = EventBasic::getInstance();
	if (event_ptr == NULL)
		return -1;
	if (0 != event_ptr->init(std::string(server_ip)))
		return HANDLE_EVENT_FAILED;
	return 0;
}

COMM_INTERFACE_API uint64_t c_exitEvent(void)
{
	EventBasic* event_ptr = EventBasic::getInstance();
	if (event_ptr == NULL)
		return HANDLE_EVENT_FAILED;
	event_ptr->exit();
	delete event_ptr;
	event_ptr = NULL;
	return 0;
}

COMM_INTERFACE_API uint64_t c_getEventErrorList(uint64_t error[8], uint64_t time_stamp[8], int32_t* size)
{
	EventBasic* event_ptr = EventBasic::getInstance();
	if (event_ptr == NULL)
		return HANDLE_EVENT_FAILED;

	EventInfo event[8];
	event_ptr->popAll(event, size);
	for (int32_t i = 0; i < *size; ++i)
	{
		error[i] = event[i].data;
		time_stamp[i] = event[i].time_stamp;
	}
	return 0;
}
