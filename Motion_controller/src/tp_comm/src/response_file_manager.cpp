#include "tp_comm.h"

using namespace user_space;
using namespace log_space;
using namespace std;

//"/rpc/file_manager/readFile"
void TpComm::handleResponse0x0000A545(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Bytes_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_String*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Bytes*)task->response_data_ptr;
    }
}

//"/rpc/file_manager/writeFile"
void TpComm::handleResponse0x00010D95(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_String_Bytes*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}


