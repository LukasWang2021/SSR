#include "tp_comm.h"

using namespace user_space;
using namespace log_space;
using namespace std;


/********rpc/group/mcGroupReset, ResponseMessageType_Uint64**********/  
void TpComm::handleResponse0x00016FF4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x00016FF4: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

/********rpc/group/mcGroupEnable, ResponseMessageType_Uint64**********/ 
void TpComm::handleResponse0x00003615(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x00003615: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

/********rpc/group/mcGroupDisable, ResponseMessageType_Uint64**********/    
void TpComm::handleResponse0x0000D185(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x0000D185: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}


/********rpc/group/mcGroupReadError, ResponseMessageType_Uint64_Uint64**********/   
void TpComm::handleResponse0x00004BE2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x0000E039: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Uint64*)task->response_data_ptr;
    }
}

/********rpc/group/mcGroupReadStatus, ResponseMessageType_Uint64_GroupStatus_Bool**********/  
void TpComm::handleResponse0x00002A83(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_GroupStatus_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x00002A83: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_GroupStatus_Bool*)task->response_data_ptr;
    }
}

/********rpc/group/resetAllEncoder, ResponseMessageType_Uint64**********/	
void TpComm::handleResponse0x000019D2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x000019D2: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}		

