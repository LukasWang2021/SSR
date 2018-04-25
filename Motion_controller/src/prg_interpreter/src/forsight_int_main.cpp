#ifndef WIN32
#include <unistd.h>
#endif
#include "forsight_inter_control.h"
#include "forsight_io_mapping.h"
#include "forsight_io_controller.h"

int main(int  argc, char *argv[])
{
	initShm();
	append_io_mapping();
	forgesight_load_io_config();
	while(1)
	{
		bool ret = getIntprtCtrl();
		if (ret)
		{
			parseCtrlComand(); //  &g_thread_control_block[0]);
		}
#ifdef WIN32
		Sleep(100);
#else
		sleep(1);
#endif
	}
	return 1;
}


