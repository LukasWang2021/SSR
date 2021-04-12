#include "tp_comm.h"

using namespace user_space;
using namespace log_space;
using namespace std;


/********rpc/servo_sampling/setSamplingConfiguration, ResponseMessageType_Uint64**********/
void TpComm::handleResponse0x0000845E(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x0000845E: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_Uint32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

/********rpc/servo_sampling/getSamplingConfiguration, ResponseMessageType_Uint64_Uint32List(count=2)**********/
void TpComm::handleResponse0x000106EE(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Uint32List_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x000106EE: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Uint32List*)task->response_data_ptr;
    }
}

/********rpc/servo_sampling/activateSamplingConfiguration, ResponseMessageType_Uint64**********/  
void TpComm::handleResponse0x0000CDDE(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x0000CDDE: failed to encode response package");
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

/********rpc/servo_sampling/setSamplingSync, ResponseMessageType_Uint64**********/ 
void TpComm::handleResponse0x00003743(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x00003743: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_Uint32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

/********rpc/servo_sampling/getSamplingSync, ResponseMessageType_Uint64_Uint32**********/   
void TpComm::handleResponse0x00006343(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Uint32_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x00006343: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Uint32*)task->response_data_ptr;
    }
}

/********rpc/servo_sampling/setSamplingChannel, ResponseMessageType_Uint64**********/  
void TpComm::handleResponse0x0000BACC(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x0000BACC: failed to encode response package");
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

/********rpc/servo_sampling/getSamplingChannel, ResponseMessageType_Uint64_ServoSamplingChannelList**********/
void TpComm::handleResponse0x0000556C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Uint32List_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x0000556C: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Uint32List*)task->response_data_ptr;
    }
}

/********rpc/servo_sampling/saveSamplingBufferData, ResponseMessageType_Uint64**********/  
void TpComm::handleResponse0x00004E41(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("rpc", "handleResponse0x00004E41: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_String*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

