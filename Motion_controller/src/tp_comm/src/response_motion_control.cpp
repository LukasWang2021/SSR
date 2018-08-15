#include "tp_comm.h"

using namespace fst_comm;

//"/rpc/motion_control/stop"
void TpComm::handleResponse0x00001E70(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// "/rpc/motion_control/reset"
void TpComm::handleResponse0x00001D14(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/setManualMode"
void TpComm::handleResponse0x00012FB5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/setManualFrame"
void TpComm::handleResponse0x00009D05(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/doStepManualMove"
void TpComm::handleResponse0x000085D5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/doContinusManulMove"
void TpComm::handleResponse0x000173B5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/doGotoPointManulMove"
void TpComm::handleResponse0x00009B75(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_DoubleList*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

//"/rpc/motion_control/axis_group/getJointsFeedBack",	0x0000DFBB,	"RequestMessageType.Int32",	"ResponseMessageType.Bool_Int32List(count = 9)
void TpComm::handleResponse0x0000DFBB(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_DoubleList_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool_DoubleList*)task->response_data_ptr;
    }
}
