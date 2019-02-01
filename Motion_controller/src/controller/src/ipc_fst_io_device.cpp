#include "controller_ipc.h"
//#include "io_interface.h"
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
    FST_INFO("handleIpcGetDi: user_port=%d, value=%d, err=%llx", rq_data_ptr->port_offset, value, rs_data_ptr->error_code);

    if(rs_data_ptr->error_code == SUCCESS)
    {
        rs_data_ptr->value = value;
    }
    else
    {
        FST_INFO("NULL::handleIpcGetDi with <port_offset = %d>",rq_data_ptr->port_offset);
    }
}

//SetDi
void ControllerIpc::handleIpcSetDi(void* request_data_ptr, void* response_data_ptr)
{
	RequestSetDi* rq_data_ptr = static_cast<RequestSetDi*>(request_data_ptr);
    unsigned long long* rs_data_ptr = static_cast<unsigned long long*>(response_data_ptr);

	*rs_data_ptr = io_mapping_ptr_->setDIByBit(rq_data_ptr->port_offset, rq_data_ptr->value);
    FST_INFO("handleIpcSetDi: user_port=%d, value=%d, ret =%x", rq_data_ptr->port_offset, rq_data_ptr->value, *rs_data_ptr);

    if (*rs_data_ptr != SUCCESS)
	    FST_INFO("NULL::handleIpcSetDi with <port_offset = %d, value=%d>",rq_data_ptr->port_offset, rq_data_ptr->value);
}

//GetDo
void ControllerIpc::handleIpcGetDo(void* request_data_ptr, void* response_data_ptr)
{
	RequestGetDo* rq_data_ptr = static_cast<RequestGetDo*>(request_data_ptr);
    ResponseGetDo* rs_data_ptr = static_cast<ResponseGetDo*>(response_data_ptr);

    uint8_t value = 0;
    rs_data_ptr->error_code = io_mapping_ptr_->getDOByBit(rq_data_ptr->port_offset, value);
    FST_INFO("handleIpcGetDo: user_port=%d, value=%d, err=%llx", rq_data_ptr->port_offset, value, rs_data_ptr->error_code);

    if(rs_data_ptr->error_code == SUCCESS)
    {
        rs_data_ptr->value = value;
    }
    else
    {
        FST_INFO("NULL::handleIpcGetDo with <port_offset = %d>",rq_data_ptr->port_offset);
    }
}

//SetDo
void ControllerIpc::handleIpcSetDo(void* request_data_ptr, void* response_data_ptr)
{
	RequestSetDo* rq_data_ptr = static_cast<RequestSetDo*>(request_data_ptr);
    unsigned long long* rs_data_ptr = static_cast<unsigned long long*>(response_data_ptr);

	*rs_data_ptr = io_mapping_ptr_->setDOByBit(rq_data_ptr->port_offset, rq_data_ptr->value);
    FST_INFO("handleIpcSetDo: user_port=%d, value=%d, ret =%x", rq_data_ptr->port_offset, rq_data_ptr->value, *rs_data_ptr);

    if (*rs_data_ptr != SUCCESS)
	    FST_INFO("NULL::handleIpcSetDo with <port_offset = %d, value=%d>",rq_data_ptr->port_offset, rq_data_ptr->value);

}

//GetRi
void ControllerIpc::handleIpcGetRi(void* request_data_ptr, void* response_data_ptr)
{
	RequestGetRi* rq_data_ptr = static_cast<RequestGetRi*>(request_data_ptr);
    ResponseGetRi* rs_data_ptr = static_cast<ResponseGetRi*>(response_data_ptr);

    uint8_t value = 0;
    rs_data_ptr->error_code = io_mapping_ptr_->getRIByBit(rq_data_ptr->port_offset, value);
    FST_INFO("handleIpcGetRi: user_port=%d, value=%d, err=%llx", rq_data_ptr->port_offset, value, rs_data_ptr->error_code);

    if(rs_data_ptr->error_code == SUCCESS)
    {
        rs_data_ptr->value = value;
    }
    else
    {
        FST_INFO("NULL::handleIpcGetRi with <port_offset = %d>",rq_data_ptr->port_offset);
    }
}

//SetRi
void ControllerIpc::handleIpcSetRi(void* request_data_ptr, void* response_data_ptr)
{
	RequestSetRi* rq_data_ptr = static_cast<RequestSetRi*>(request_data_ptr);
    unsigned long long* rs_data_ptr = static_cast<unsigned long long*>(response_data_ptr);

	*rs_data_ptr = io_mapping_ptr_->setRIByBit(rq_data_ptr->port_offset, rq_data_ptr->value);
    FST_INFO("handleIpcSetRi: user_port=%d, value=%d, ret =%x", rq_data_ptr->port_offset, rq_data_ptr->value, *rs_data_ptr);

    if (*rs_data_ptr != SUCCESS)
	    FST_INFO("NULL::handleIpcSetRi with <port_offset = %d, value=%d>",rq_data_ptr->port_offset, rq_data_ptr->value);
}

//GetRo
void ControllerIpc::handleIpcGetRo(void* request_data_ptr, void* response_data_ptr)
{
	RequestGetRo* rq_data_ptr = static_cast<RequestGetRo*>(request_data_ptr);
    ResponseGetRo* rs_data_ptr = static_cast<ResponseGetRo*>(response_data_ptr);

    uint8_t value = 0;
    rs_data_ptr->error_code = io_mapping_ptr_->getROByBit(rq_data_ptr->port_offset, value);
    FST_INFO("handleIpcGetRo: user_port=%d, value=%d, err=%llx", rq_data_ptr->port_offset, value, rs_data_ptr->error_code);

    if(rs_data_ptr->error_code == SUCCESS)
    {
        rs_data_ptr->value = value;
    }
    else
    {
        FST_INFO("NULL::handleIpcGetRo with <port_offset = %d>",rq_data_ptr->port_offset);
    }
}

//SetRo
void ControllerIpc::handleIpcSetRo(void* request_data_ptr, void* response_data_ptr)
{
	RequestSetRo* rq_data_ptr = static_cast<RequestSetRo*>(request_data_ptr);
    unsigned long long* rs_data_ptr = static_cast<unsigned long long*>(response_data_ptr);

	*rs_data_ptr = io_mapping_ptr_->setROByBit(rq_data_ptr->port_offset, rq_data_ptr->value);
    FST_INFO("handleIpcSetRo: user_port=%d, value=%d, ret =%x", rq_data_ptr->port_offset, rq_data_ptr->value, *rs_data_ptr);

    if (*rs_data_ptr != SUCCESS)
	    FST_INFO("NULL::handleIpcSetRo with <port_offset = %d, value=%d>",rq_data_ptr->port_offset, rq_data_ptr->value);

}



//GetUi
void ControllerIpc::handleIpcGetUi(void* request_data_ptr, void* response_data_ptr)
{
    RequestGetUi* rq_data_ptr = static_cast<RequestGetUi*>(request_data_ptr);
    ResponseGetUi* rs_data_ptr = static_cast<ResponseGetUi*>(response_data_ptr);

    uint8_t value = 0;
    rs_data_ptr->error_code = io_mapping_ptr_->getUIByBit(rq_data_ptr->port_offset, value);
    FST_INFO("handleIpcGetUi: user_port=%d, value=%d, err=%llx", rq_data_ptr->port_offset, value, rs_data_ptr->error_code);

    if(rs_data_ptr->error_code == SUCCESS)
    {
        rs_data_ptr->value = value;
    }
    else
    {
        FST_INFO("NULL::handleIpcGetUi with <port_offset = %d>",rq_data_ptr->port_offset);
    }
}

//SetUi
void ControllerIpc::handleIpcSetUi(void* request_data_ptr, void* response_data_ptr)
{
	RequestSetUi* rq_data_ptr = static_cast<RequestSetUi*>(request_data_ptr);
    unsigned long long* rs_data_ptr = static_cast<unsigned long long*>(response_data_ptr);

	*rs_data_ptr = io_mapping_ptr_->setUIByBit(rq_data_ptr->port_offset, rq_data_ptr->value);
    FST_INFO("handleIpcSetUi: user_port=%d, value=%d, ret =%x", rq_data_ptr->port_offset, rq_data_ptr->value, *rs_data_ptr);

    if (*rs_data_ptr != SUCCESS)
	    FST_INFO("NULL::handleIpcSetUi with <port_offset = %d, value=%d>",rq_data_ptr->port_offset, rq_data_ptr->value);
}

//GetUo
void ControllerIpc::handleIpcGetUo(void* request_data_ptr, void* response_data_ptr)
{
	RequestGetUo* rq_data_ptr = static_cast<RequestGetUo*>(request_data_ptr);
    ResponseGetUo* rs_data_ptr = static_cast<ResponseGetUo*>(response_data_ptr);

    uint8_t value = 0;
    rs_data_ptr->error_code = io_mapping_ptr_->getUOByBit(rq_data_ptr->port_offset, value);
    FST_INFO("handleIpcGetUo: user_port=%d, value=%d, err=%llx", rq_data_ptr->port_offset, value, rs_data_ptr->error_code);

    if(rs_data_ptr->error_code == SUCCESS)
    {
        rs_data_ptr->value = value;
    }
    else
    {
        FST_INFO("NULL::handleIpcGetUo with <port_offset = %d>",rq_data_ptr->port_offset);
    }
}

//SetUo is forbidden
void ControllerIpc::handleIpcSetUo(void* request_data_ptr, void* response_data_ptr)
{
	RequestSetUo* rq_data_ptr = static_cast<RequestSetUo*>(request_data_ptr);
    unsigned long long* rs_data_ptr = static_cast<unsigned long long*>(response_data_ptr);

	*rs_data_ptr = SUCCESS;

	FST_INFO("NULL::handleIpcSetUo with <port_offset = %d, value=%d>",rq_data_ptr->port_offset, rq_data_ptr->value);
}