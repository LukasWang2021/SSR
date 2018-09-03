#include "controller_ipc.h"
#include "io_interface.h"
#include <unistd.h>
#include <stdlib.h>
#include <cstring>

//using namespace fst_hal;
using namespace fst_ctrl;
using namespace fst_base;

void ControllerIpc::handleIpcCheckIo(void* request_data_ptr, void* response_data_ptr)
{
    char* rq_data_ptr = static_cast<char*>(request_data_ptr);
    ResponseCheckIo* rs_data_ptr = static_cast<ResponseCheckIo*>(response_data_ptr);
	if(io_device_ptr_)
	{
	    rs_data_ptr->error_code = io_device_ptr_->checkIO(
			rq_data_ptr, &rs_data_ptr->port_info);
	}
	else
	{
		printf("NULL::handleIpcCheckIo with %s\n", rq_data_ptr);
	}
}

void ControllerIpc::handleIpcSetIo(void* request_data_ptr, void* response_data_ptr)
{
    RequestSetIo* rq_data_ptr = static_cast<RequestSetIo*>(request_data_ptr);
    unsigned long long* rs_data_ptr = static_cast<unsigned long long*>(response_data_ptr);
	if(io_device_ptr_)
	{
	    *rs_data_ptr = io_device_ptr_->setDO(&(rq_data_ptr->port_info), rq_data_ptr->value);
	}
	else
	{
		printf("NULL::handleIpcSetIo with <%d, %d, %d, %d, %d, %d>\n", 
			rq_data_ptr->port_info.msg_id, rq_data_ptr->port_info.dev_id, 
			rq_data_ptr->port_info.port_type, rq_data_ptr->port_info.port_index, 
			rq_data_ptr->port_info.bytes_len);
	}
}

void ControllerIpc::handleIpcGetIo(void* request_data_ptr, void* response_data_ptr)
{
	char cTmp[8];
    RequestGetIo* rq_data_ptr = static_cast<RequestGetIo*>(request_data_ptr);
    ResponseGetIo* rs_data_ptr = static_cast<ResponseGetIo*>(response_data_ptr);

	if(io_device_ptr_)
	{

	    rs_data_ptr->error_code = io_device_ptr_->getDIO(
			&rq_data_ptr->port_info, cTmp, rq_data_ptr->buffer_length);
		rs_data_ptr->value = cTmp[0];
		
	}
	else
	{
		printf("NULL::handleIpcGetIo with <%d, %d, %d, %d, %d, %d>\n", 
			rq_data_ptr->port_info.msg_id, rq_data_ptr->port_info.dev_id, 
			rq_data_ptr->port_info.port_type, rq_data_ptr->port_info.port_index, 
			rq_data_ptr->port_info.bytes_len);
	}
}

