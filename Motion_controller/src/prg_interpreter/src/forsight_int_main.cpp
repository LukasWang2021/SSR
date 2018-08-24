#ifndef WIN32
#include <unistd.h>
#include "common.h"
using namespace fst_log;
#endif
#include "forsight_inter_control.h"
#include "forsight_io_mapping.h"
#include "forsight_io_controller.h"

#include "reg_manager/reg_manager_interface_wrapper.h"

#define IO_ERROR_INTERVAL_COUNT           (5)  //2ms*5=10ms

// #ifndef WIN32
// Logger glog;
// #endif

int main(int  argc, char *argv[])
{
	InterpreterControl intprt_ctrl;
	initShm();
	memset(&intprt_ctrl, 0x00, sizeof(intprt_ctrl));
	intprt_ctrl.cmd = fst_base::INTERPRETER_SERVER_CMD_START ;
	append_io_mapping();
	forgesight_load_io_config();
	load_register_data();
	while(1)
	{
		std::vector<fst_base::ProcessCommRequestResponse>::iterator it;
		std::vector<fst_base::ProcessCommRequestResponse> request_list
			= g_objInterpreterServer->popTaskFromRequestList();
		if(request_list.size() != 0)
		{
			for(it = request_list.begin(); it != request_list.end(); ++it)
			{
				memset(&intprt_ctrl, 0x00, sizeof(intprt_ctrl));
				intprt_ctrl.cmd = it->cmd_id ;
				memcpy(intprt_ctrl.start_ctrl, it->request_data_ptr, 256);
	            printf("parseCtrlComand at %d \n", intprt_ctrl.cmd);
				parseCtrlComand(intprt_ctrl);
				bool * bRsp = it->response_data_ptr;
				*bRsp = true;
				g_objInterpreterServer->pushTaskToResponseList(*it);
			}
#ifdef WIN32
			Sleep(100);
#else
			usleep(1000);
#endif
		    static int count = 0;
		    if (++count >= IO_ERROR_INTERVAL_COUNT)
		    {
				updateIOError();
		        count = 0;
		    }
		}
		else
		{
			intprt_ctrl.cmd = fst_base::INTERPRETER_SERVER_CMD_LOAD ;
		}
	}
	return 1;
}


