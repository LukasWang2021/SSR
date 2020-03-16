#ifndef WIN32
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include "common.h"
using namespace fst_log;
#else
#include "forsight_io_mapping.h"
#endif
#include "forsight_inter_control.h"
#include "forsight_io_controller.h"

#include "reg_manager/reg_manager_interface_wrapper.h"

#define IO_ERROR_INTERVAL_COUNT           (5)  //2ms*5=10ms

// #ifndef WIN32
// Logger glog;
// #endif

#ifndef WIN32
void signalInterrupt(int signo) 
{
	if(log_ptr_ != NULL)
	{
        FST_INFO("Free log_ptr_ in the signalInterrupt");
		
		delete log_ptr_;
		log_ptr_ = NULL ;
	}
    _exit(0);
}
#endif


/************************************************* 
	Function:		main
	Description:	main function.
	Input:			name  - io name
	Input:			NULL
	Output: 		NULL
	Return: 		Never
*************************************************/ 
int main(int  argc, char *argv[])
{}

void prgRoutine(ControllerSm* state_machine_ptr, 
					fst_mc::MotionControl* motion_control_ptr, 
					RegManager* reg_manager_ptr, 
                    IoMapping* io_mapping_ptr, 
                    IoManager* io_manager_ptr, 
                    fst_hal::ModbusManager* modbus_manager_ptr)
{
	InterpreterControl intprt_ctrl; 
	memset(&intprt_ctrl, 0x00, sizeof(intprt_ctrl));
#ifndef WIN32
	signal(SIGINT, signalInterrupt);
	if(log_ptr_ == NULL)
	{
		log_ptr_ = new fst_log::Logger();
    	FST_LOG_INIT("Interpreter");
	}
	
	state_machine_ptr_  = state_machine_ptr;
	motion_control_ptr_ = motion_control_ptr;
	reg_manager_ptr_    = reg_manager_ptr;
	io_mapping_ptr_     = io_mapping_ptr;
	io_manager_ptr_     = io_manager_ptr;
	modbus_manager_ptr_ = modbus_manager_ptr;
#else
	//	append_io_mapping();
	intprt_ctrl.cmd = fst_base::INTERPRETER_SERVER_CMD_LAUNCH ;
#endif
	initInterpreter();

	bool bRet = load_register_data();
}
	
void prgRoutineUnInit(void)
{
	uninitInterpreter();
#ifndef WIN32
	if(log_ptr_ != NULL)
	{
        FST_INFO("Free log_ptr_");
		
		delete log_ptr_;
		log_ptr_ = NULL ;
	}
#endif
	return 1;
}


