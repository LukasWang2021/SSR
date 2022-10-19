#include "err_proc.h"
#include "touch_test.h"
#include "touch_interface.h"
#include <iostream>

typedef struct {
	int err_index;
	char err_buff_info[ERR_MAX][ERR_INFO_BYTES];
}TouchErr_t;

static TouchErr_t err_info;

void PushErrInfo(int module, const char* err_str)
{
	if (module<0 || module>ERR_MAX)
		return;

	err_info.err_index = module;
	memcpy_s(err_info.err_buff_info[module], ERR_INFO_BYTES, err_str, strlen(err_str)+1);
	TouchCallbackRun(TOUCH_EVENT_ERROR_EXIST, err_info.err_buff_info[module]);
}

int GetErrPtr(char** ptr, int index)
{
	if (ptr==NULL)
		return -1;

	*ptr = err_info.err_buff_info[index];
	return 0;
}

char* GetErrInfoPtr(void)
{
	return err_info.err_buff_info[err_info.err_index];
}

int GetErrInfo(char* ptr, int size)
{
	if (ptr == NULL || size <= 0)
		return -1;

	memcpy_s(ptr, size, err_info.err_buff_info[err_info.err_index], ERR_INFO_BYTES);
	return 0;
}

void ErrInfoReset(void)
{
	memset(&err_info, 0, sizeof(TouchErr_t));
}