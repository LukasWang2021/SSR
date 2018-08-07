#include "tp_comm.h"
#include "common/common.h"

using namespace std;

//"/rpc/controller/getUserOpMode",	0x0x00000C05
void TpComm::handleResponse0x00000C05(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)         
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponseGetUserOpMode: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Int32*)task->response_data_ptr;
    }
}

//"/rpc/controller/getRunningStatus",	0x0x00000AB3
void TpComm::handleResponse0x00000AB3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)      
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponseGetRunningStatus: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Int32*)task->response_data_ptr;
    }
}


// "/rpc/controller/getInterpreterStatus",	0x0x00016483
void TpComm::handleResponse0x00016483(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)  
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponseGetInterpreterStatus: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Int32*)task->response_data_ptr;
    }
}

// "/rpc/controller/getRobotStatus",	0x0x00006F83
void TpComm::handleResponse0x00006F83(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)        
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponseGetRobotStatus: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Int32*)task->response_data_ptr;
    }
}

//"/rpc/controller/getCtrlStatus",	0x0000E9D3
void TpComm::handleResponse0x0000E9D3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)         
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponseGetCtrlStatus: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Int32*)task->response_data_ptr;
    }
}

//"/rpc/controller/getServoStatus",	0x0x0000D113
void TpComm::handleResponse0x0000D113(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)        
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponseGetServoStatus: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Int32*)task->response_data_ptr;
    }
}

//"/rpc/controller/getSafetyAlarm",	0x0x0000C00D
void TpComm::handleResponse0x0000C00D(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)        
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponseGetSafetyAlarm: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Int32*)task->response_data_ptr;
    }
}

// "/rpc/controller/callEstop",	0x0x00013940
void TpComm::handleResponse0x00013940(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)             
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponseCallEstop: failed to encode response package");// send
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

// "/rpc/controller/callReset",	0x0x000161E4
void TpComm::handleResponse0x000161E4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)             
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponseCallReset: failed to encode response package");// send
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

//"/rpc/controller/setUserOpMode",	0x0x00002ED5
void TpComm::handleResponse0x00002ED5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)         
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponseSetUserOpMode: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// add topic
void TpComm::handleResponse0x00000773(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponseSetUserOpMode: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Topic*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

void TpComm::handleResponse0x0000BB93(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponseSetUserOpMode: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_UnsignedInt32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}
