#include "controller_rpc.h"
#include <sys/time.h>
#include <unistd.h>
#include <list>

using namespace user_space;
using namespace base_space;
using namespace log_space;

//"/rpc/io/readDI"	
void ControllerRpc::handleRpc0x000185A9(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    uint8_t value = 0;
    rs_data_ptr->error_code.data = io_dev_ptr_->readDiBit(rq_data_ptr->data.data, value);
    LogProducer::info("rpc", "/rpc/io/readDI offset=%d, value=%d, ret=%llx", rq_data_ptr->data.data, value, rs_data_ptr->error_code.data);

    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.data = value;
    }
}
//"/rpc/io/readDO"	
void ControllerRpc::handleRpc0x000185AF(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    uint8_t value = 0;
    rs_data_ptr->error_code.data = io_dev_ptr_->readDoBit(rq_data_ptr->data.data, value);
    LogProducer::info("rpc", "/rpc/io/readDO offset=%d, value=%d, ret=%llx", rq_data_ptr->data.data, value, rs_data_ptr->error_code.data);

    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.data = value;
    }
}
//"/rpc/io/writeDO"	
void ControllerRpc::handleRpc0x00000C1F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    
    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
    }

    rs_data_ptr->data.data = io_dev_ptr_->writeDoBit(rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
    LogProducer::info("rpc", "/rpc/io/writeDO offset=%d, value=%d, ret=%llx", rq_data_ptr->data.data[0], rq_data_ptr->data.data[1], rs_data_ptr->data.data);
}
//"/rpc/io/readAI"	
void ControllerRpc::handleRpc0x00018679(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    int16_t value = 0;
    rs_data_ptr->error_code.data = io_analog_ptr_->readAIO(0, rq_data_ptr->data.data, value);
    LogProducer::info("rpc", "/rpc/io/readAI offset=%d, value=%d, ret=%llx", rq_data_ptr->data.data, value, rs_data_ptr->error_code.data);

    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.data = value;
    }
}
//"/rpc/io/readAO"	
void ControllerRpc::handleRpc0x0001867F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    int16_t value = 0;
    rs_data_ptr->error_code.data = io_analog_ptr_->readAIO(0, rq_data_ptr->data.data, value);
    LogProducer::info("rpc", "/rpc/io/readAO offset=%d, value=%d, ret=%llx", rq_data_ptr->data.data, value, rs_data_ptr->error_code.data);

    if(rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.data = value;
    }
}
//"/rpc/io/writeAO"	
void ControllerRpc::handleRpc0x00000C4F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32List* rq_data_ptr = static_cast<RequestMessageType_Int32List*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    
    if(rq_data_ptr->data.data_count == 2)
    {
        rs_data_ptr->data.data = SUCCESS;
    }
    else
    {
        rs_data_ptr->data.data = RPC_PARAM_INVALID;
    }

    rs_data_ptr->data.data = io_analog_ptr_->writeAO(0, rq_data_ptr->data.data[0], rq_data_ptr->data.data[1]);
    LogProducer::info("rpc", "/rpc/io/writeAO offset=%d, value=%d, ret=%llx", rq_data_ptr->data.data[0], rq_data_ptr->data.data[1], rs_data_ptr->data.data);

}

