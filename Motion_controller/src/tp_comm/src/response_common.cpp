#include "tp_comm.h"

using namespace fst_comm;

using namespace std;

//get rpc table
void TpComm::handleResponse0x00004FA5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_RpcTable_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse0x00004FA5: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_RpcTable*)task->response_data_ptr;
    }
}

// get publish table
void TpComm::handleResponse0x000147A5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_PublishTable_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse0x000147A5: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_PublishTable*)task->response_data_ptr;
    }
}
