#include "rpc_interface.h"
#include "sub_interface.h"
#include "event_interface.h"
#include <stdio.h>
#include "common_error_code.h"
#include <iostream>

int main()
{
	printf("Comm main\n");
	double force[6];
	char ip[32] = { 0 };
	memcpy(ip, "192.168.110.2", sizeof("192.168.110.2"));
	uint64_t init_ret = c_initRpc(ip);
	if (init_ret != 0)
	{
		printf("c_initRpc failed\n");
		return -1;
	}
	c_deleteTopic();
	init_ret = c_addTopic();
	if (init_ret != 0)
	{
		printf("c_addTopic failed!\n");
		return -1;
	}
	init_ret = c_initSub(ip);
	if (init_ret != 0)
	{
		printf("c_initSub failed!\n");
		return -1;
	}
	while (1)
	{
		if (c_getTorqueFeedBack(force, 6 * sizeof(double)) == 0)
			printf("force value: %lf %lf %lf %lf %lf %lf\n", 
				force[0], force[1], force[2], force[3], force[4], force[5]);

#ifdef _WIN_PLAT
		Sleep(1000);
#else
		usleep(1000000);
#endif
	}
	return 0;
}