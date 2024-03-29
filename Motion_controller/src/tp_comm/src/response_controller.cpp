#include "tp_comm.h"

using namespace user_space;
using namespace log_space;
using namespace std;

//"/rpc/controller/getVersion"
void TpComm::handleResponse0x000093EE(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Uint32List_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x000093EE: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Uint32List*)task->response_data_ptr;
    }
}


//"/rpc/controller/setSystemTime"
void TpComm::handleResponse0x000167C5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x000167C5: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Uint64*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/controller/getSystemTime"
void TpComm::handleResponse0x000003F5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x000003F5: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/controller/setWorkMode"
void TpComm::handleResponse0x00006825(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{    
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x00006825: failed to encode response package");
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
//"/rpc/controller/getWorkMode"
void TpComm::handleResponse0x00003325(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Uint32_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x00003325: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Uint32*)task->response_data_ptr;
    }
}

/********rpc/controller/setControlMode, ResponseMessageType_Uint64**********/	
void TpComm::handleResponse0x0000B555(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x0000B555: failed to encode response package");
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

/********rpc/controller/getControlMode, ResponseMessageType_Uint64_Uint32**********/	
void TpComm::handleResponse0x0000B695(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Uint32_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x0000B695: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Uint32*)task->response_data_ptr;
    }
}


