#include "tp_comm.h"

using namespace fst_comm;

using namespace std;

//"/rpc/io_manager/getDIByBit"
void TpComm::handleResponse0x0000BFE4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Int32*)task->response_data_ptr;
    }
}
//"/rpc/io_manager/setDIByBit"
void TpComm::handleResponse0x00018684(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
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
//"/rpc/io_manager/getDOByBit"
void TpComm::handleResponse0x0000B4C4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Int32*)task->response_data_ptr;
    }
}
//"/rpc/io_manager/setDOByBit"
void TpComm::handleResponse0x00017B64(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
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

