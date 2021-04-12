#include "tp_comm.h"

using namespace user_space;
using namespace std;
using namespace log_space;


/********rpc/io/readDI, ResponseMessageType_Uint64_Int32**********/	
void TpComm::handleResponse0x000185A9(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x000185A9: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Int32*)task->response_data_ptr;
    }
}
/********rpc/io/readDO, ResponseMessageType_Uint64_Int32**********/	
void TpComm::handleResponse0x000185AF(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x000185AF: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Int32*)task->response_data_ptr;
    }
}
/********rpc/io/writeDO, ResponseMessageType_Uint64**********/	
void TpComm::handleResponse0x00000C1F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x00000C1F: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}
