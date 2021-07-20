#include "tp_comm.h"

using namespace user_space;
using namespace log_space;
using namespace std;


/********rpc/reg_manager/pr/addReg, ResponseMessageType_Uint64**********/	
void TpComm::handleResponse0x000154E7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x000154E7: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_PrRegData*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}
/********rpc/reg_manager/pr/deleteReg, ResponseMessageType_Uint64**********/	
void TpComm::handleResponse0x00001097(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x00001097: failed to encode response package");
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
/********rpc/reg_manager/pr/updateReg, ResponseMessageType_Uint64**********/	
void TpComm::handleResponse0x00009EF7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x00009EF7: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_PrRegData*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}
/********rpc/reg_manager/pr/getReg, ResponseMessageType_Uint64_PrRegData**********/	
void TpComm::handleResponse0x00017207(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_PrRegData_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x00017207: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_PrRegData*)task->response_data_ptr;
    }
}
/********rpc/reg_manager/pr/moveReg, ResponseMessageType_Uint64**********/	
void TpComm::handleResponse0x0000D7C7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x0000D7C7: failed to encode response package");
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
/********rpc/reg_manager/pr/getChangedList, ResponseMessageType_Uint64_BaseRegSummaryList**********/	
void TpComm::handleResponse0x0000B454(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_BaseRegSummaryList_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x0000B454: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_BaseRegSummaryList*)task->response_data_ptr;
    }
}
/********rpc/reg_manager/pr/getValidList, ResponseMessageType_Uint64_BaseRegSummaryList**********/	
void TpComm::handleResponse0x00009354(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_BaseRegSummaryList_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x00009354: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_BaseRegSummaryList*)task->response_data_ptr;
    }
}

