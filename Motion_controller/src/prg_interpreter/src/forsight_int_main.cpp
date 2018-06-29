#ifndef WIN32
#include <unistd.h>
#endif
#include "forsight_inter_control.h"
#include "forsight_io_mapping.h"
#include "forsight_io_controller.h"
#include "common.h"

using namespace fst_log;

Logger glog;

int main(int  argc, char *argv[])
{
	initShm();
	append_io_mapping();
	forgesight_load_io_config();
#ifndef WIN32
	load_register_data();
#endif
    glog.initLogger("interpreter");\
    glog.setDisplayLevel(fst_log::MSG_LEVEL_INFO);\
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
		usleep(1000);
#endif
	}
	return 1;
}


