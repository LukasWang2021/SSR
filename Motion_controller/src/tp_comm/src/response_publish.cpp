#include "tp_comm.h"

using namespace user_space;
using namespace log_space;


//"/rpc/publish/addTopic",
void TpComm::handleResponse0x000050E3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Topic*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/publish/deleteTopic"
void TpComm::handleResponse0x00004403(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Uint32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}


