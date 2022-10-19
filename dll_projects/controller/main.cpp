#include "rpc_interface.h"
#include "sub_interface.h"
#include "event_interface.h"
#include <stdio.h>
#include "common_error_code.h"
#include <iostream>

int main()
{
	printf("Comm main\n");

	char ip[32] = { 0 };
	memcpy(ip, "192.168.30.108", sizeof("192.168.30.108"));
	uint64_t init_ret = c_initEvent(ip);
	if (init_ret != 0)
	{
		printf("failed to init event client\n");
		return -1;
	}
	printf("waiting command\n");
	getchar();
	printf("receive command\n");

	ErrorCode error_code[8] = { 0 };
	uint64_t time_stamp[8] = { 0 };
	int32_t size = 0;
	ErrorCode ret = c_getEventErrorList(error_code, time_stamp, &size);
	if (ret != 0)
	{
		printf("failed to get event\n");
		return -1;
	}
	
	printf("error size=%d\n", size);
	ErrorCode detail_ret = 0;
	char text[128] = { 0 };
	int text_size = 0;
	for (int32_t i = 0; i < size; ++i)
	{
		detail_ret = c_getErrorCodeDetail(error_code[i], text, &text_size);
		printf("ret = %lld, code=%lld, text = %s\n", detail_ret, error_code[i], text);
	}

	init_ret = c_exitEvent();
	printf("exit event ret = 0x%llX\n", init_ret);

	/*
	ErrorCode error_code = 0x6008;
	char text[128] = { 0 };
	int size = 0;
	uint64_t ret = c_getErrorCodeDetail(error_code, text, &size);
	printf("ret = %lld, text = %s\n", ret, text);
	*/
#ifdef _WIN_PLAT
	Sleep(10000);
#else
	usleep(10000000);
#endif
	return 0;
}