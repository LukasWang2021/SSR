#include "controller_ipc.h"
#include "io_interface.h"
#include <unistd.h>
#include <stdlib.h>
#include <cstring>

//using namespace fst_hal;
using namespace fst_ctrl;
using namespace fst_base;

//GetDi
void ControllerIpc::handleIpcGetDi(void* request_data_ptr, void* response_data_ptr)
{
    RequestGetDi* rq_data_ptr = static_cast<RequestGetDi*>(request_data_ptr);
    ResponseGetDi* rs_data_ptr = static_cast<ResponseGetDi*>(response_data_ptr);

    uint8_t value = 0;
    rs_data_ptr->error_code = io_mapping_ptr_->getDIByBit(rq_data_ptr->port_offset, value);
    FST_INFO("handleIpcGetDi: user_port=%d, value=%d, err=%llx\n", rq_data_ptr->port_offset, value, rs_data_ptr->error_code);

    if(rs_data_ptr->error_code == SUCCESS)
    {
        rs_data_ptr->value = value;
    }
    else
    {
        printf("NULL::handleIpcGetDi with <port_offset = %d>\n",rq_data_ptr->port_offset);
    }
}

//SetDi
void ControllerIpc::handleIpcSetDi(void* request_data_ptr, void* response_data_ptr)
{
	RequestSetDi* rq_data_ptr = static_cast<RequestSetDi*>(request_data_ptr);
    unsigned long long* rs_data_ptr = static_cast<unsigned long long*>(response_data_ptr);

	*rs_data_ptr = io_mapping_ptr_->setDIByBit(rq_data_ptr->port_offset, rq_data_ptr->value);
    FST_INFO("handleIpcSetDi: user_port=%d, value=%d, ret =%x\n", rq_data_ptr->port_offset, rq_data_ptr->value, *rs_data_ptr);

    if (*rs_data_ptr != SUCCESS)
	    printf("NULL::handleIpcSetDi with <port_offset = %d, value=%d>\n",rq_data_ptr->port_offset, rq_data_ptr->value);
}

//GetDo
void ControllerIpc::handleIpcGetDo(void* request_data_ptr, void* response_data_ptr)
{
	RequestGetDo* rq_data_ptr = static_cast<RequestGetDo*>(request_data_ptr);
    ResponseGetDo* rs_data_ptr = static_cast<ResponseGetDo*>(response_data_ptr);

    uint8_t value = 0;
    rs_data_ptr->error_code = io_mapping_ptr_->getDOByBit(rq_data_ptr->port_offset, value);
    FST_INFO("handleIpcGetDo: user_port=%d, value=%d, err=%llx\n", rq_data_ptr->port_offset, value, rs_data_ptr->error_code);

    if(rs_data_ptr->error_code == SUCCESS)
    {
        rs_data_ptr->value = value;
    }
    else
    {
        printf("NULL::handleIpcGetDo with <port_offset = %d>\n",rq_data_ptr->port_offset);
    }
}

//SetDo
void ControllerIpc::handleIpcSetDo(void* request_data_ptr, void* response_data_ptr)
{
	RequestSetDo* rq_data_ptr = static_cast<RequestSetDo*>(request_data_ptr);
    unsigned long long* rs_data_ptr = static_cast<unsigned long long*>(response_data_ptr);

	*rs_data_ptr = io_mapping_ptr_->setDOByBit(rq_data_ptr->port_offset, rq_data_ptr->value);
    FST_INFO("handleIpcSetDo: user_port=%d, value=%d, ret =%x\n", rq_data_ptr->port_offset, rq_data_ptr->value, *rs_data_ptr);

    if (*rs_data_ptr != SUCCESS)
	    printf("NULL::handleIpcSetDo with <port_offset = %d, value=%d>\n",rq_data_ptr->port_offset, rq_data_ptr->value);

}

//GetRi
void ControllerIpc::handleIpcGetRi(void* request_data_ptr, void* response_data_ptr)
{
	RequestGetRi* rq_data_ptr = static_cast<RequestGetRi*>(request_data_ptr);
    ResponseGetRi* rs_data_ptr = static_cast<ResponseGetRi*>(response_data_ptr);

    uint8_t value = 0;
    rs_data_ptr->error_code = io_mapping_ptr_->getRIByBit(rq_data_ptr->port_offset, value);
    FST_INFO("handleIpcGetRi: user_port=%d, value=%d, err=%llx\n", rq_data_ptr->port_offset, value, rs_data_ptr->error_code);

    if(rs_data_ptr->error_code == SUCCESS)
    {
        rs_data_ptr->value = value;
    }
    else
    {
        printf("NULL::handleIpcGetRi with <port_offset = %d>\n",rq_data_ptr->port_offset);
    }
}

//SetRi
void ControllerIpc::handleIpcSetRi(void* request_data_ptr, void* response_data_ptr)
{
	RequestSetRi* rq_data_ptr = static_cast<RequestSetRi*>(request_data_ptr);
    unsigned long long* rs_data_ptr = static_cast<unsigned long long*>(response_data_ptr);

	*rs_data_ptr = io_mapping_ptr_->setRIByBit(rq_data_ptr->port_offset, rq_data_ptr->value);
    FST_INFO("handleIpcSetRi: user_port=%d, value=%d, ret =%x\n", rq_data_ptr->port_offset, rq_data_ptr->value, *rs_data_ptr);

    if (*rs_data_ptr != SUCCESS)
	    printf("NULL::handleIpcSetRi with <port_offset = %d, value=%d>\n",rq_data_ptr->port_offset, rq_data_ptr->value);
}

//GetRo
void ControllerIpc::handleIpcGetRo(void* request_data_ptr, void* response_data_ptr)
{
	RequestGetRo* rq_data_ptr = static_cast<RequestGetRo*>(request_data_ptr);
    ResponseGetRo* rs_data_ptr = static_cast<ResponseGetRo*>(response_data_ptr);

    uint8_t value = 0;
    rs_data_ptr->error_code = io_mapping_ptr_->getROByBit(rq_data_ptr->port_offset, value);
    FST_INFO("handleIpcGetRo: user_port=%d, value=%d, err=%llx\n", rq_data_ptr->port_offset, value, rs_data_ptr->error_code);

    if(rs_data_ptr->error_code == SUCCESS)
    {
        rs_data_ptr->value = value;
    }
    else
    {
        printf("NULL::handleIpcGetRo with <port_offset = %d>\n",rq_data_ptr->port_offset);
    }
}

//SetRo
void ControllerIpc::handleIpcSetRo(void* request_data_ptr, void* response_data_ptr)
{
	RequestSetRo* rq_data_ptr = static_cast<RequestSetRo*>(request_data_ptr);
    unsigned long long* rs_data_ptr = static_cast<unsigned long long*>(response_data_ptr);

	*rs_data_ptr = io_mapping_ptr_->setROByBit(rq_data_ptr->port_offset, rq_data_ptr->value);
    FST_INFO("handleIpcSetRo: user_port=%d, value=%d, ret =%x\n", rq_data_ptr->port_offset, rq_data_ptr->value, *rs_data_ptr);

    if (*rs_data_ptr != SUCCESS)
	    printf("NULL::handleIpcSetRo with <port_offset = %d, value=%d>\n",rq_data_ptr->port_offset, rq_data_ptr->value);

}


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

