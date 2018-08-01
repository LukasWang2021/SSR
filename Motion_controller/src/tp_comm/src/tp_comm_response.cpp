#include "tp_comm.h"
#include "common/common.h"



using namespace std;

// "/tp_comm/test_request"
void TpComm::handleResponse0xcf0be243(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse0xcf0be243: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Int32*)task->response_data_ptr;
    }
}

void TpComm::handleResponse0x0000F933(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse0x0000F933: failed to encode response package");// send
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

void TpComm::handleResponse0x0000E9D3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse0x0000E9D3: failed to encode response package");// send
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

void TpComm::handleResponse0x00000AB3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse0x00000AB3: failed to encode response package");// send
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
void TpComm::handleResponse0x00010945(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse0x00010945: failed to encode response package");// send
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

void TpComm::handleResponse0x000067A4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse0x000067A4: failed to encode response package");// send
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

void TpComm::handleResponse0x00010685(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse0x00010685: failed to encode response package");// send
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

void TpComm::handleResponse0x00010AB3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_String_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse0x00010AB3: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_String*)task->response_data_ptr;
    }
}

void TpComm::handleResponse0x00017C25(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_UnsignedInt64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse0x00017C25: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_UnsignedInt64*)task->response_data_ptr;
    }
}

void TpComm::handleResponse0x000093EE(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Int32List_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse0x000093EE: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Int32List*)task->response_data_ptr;
    }
}

void TpComm::handleResponse0x0000FC15(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_PublishTable_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse0x0000FC15: failed to encode response package");// send
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

void TpComm::handleResponse0x00013EA3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse0x00013EA3: failed to encode response package");// send
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

void TpComm::handleResponse0x00009BC5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_RpcTable_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x00009BC5: failed to encode response package");// send
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

//"/rpc/register/addR"
void TpComm::handleResponse0x00007CF2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x00007CF2: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_RegisterR*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

//  "/rpc/register/updateR"
void TpComm::handleResponse0x000031B2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x000031B2: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_RegisterR*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// delete r
void TpComm::handleResponse0x00013062(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x00013062: failed to encode response package");// send
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

// get r
void TpComm::handleResponse0x000116F2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_RegisterR_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x000116F2: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_UnsignedInt32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_RegisterR*)task->response_data_ptr;
    }
}

// // "/rpc/register/setActivateR"
void TpComm::handleResponse0x000098C2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x000098C2: failed to encode response package");// send
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

// get activate r
void TpComm::handleResponse0x00005602(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_UnsignedInt32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x00005602: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (ResponseMessageType_UnsignedInt32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// add pr
void TpComm::handleResponse0x0000F862(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x0000F862: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_RegisterPR*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// update pr
void TpComm::handleResponse0x0000C032(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
        if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x0000C032: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_RegisterPR*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// delete pr
void TpComm::handleResponse0x00005392(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x00005392: failed to encode response package");// send
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

// get pr
void TpComm::handleResponse0x000170A2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_RegisterPR_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x000170A2: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_UnsignedInt32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_RegisterPR*)task->response_data_ptr;
    }
}

// set activate pr
void TpComm::handleResponse0x000116B2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x000116B2: failed to encode response package");// send
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

// get activate pr
void TpComm::handleResponse0x0000F402(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_UnsignedInt32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x0000F402: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (ResponseMessageType_UnsignedInt32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}


// add mr
void TpComm::handleResponse0x0000F892(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x0000F892: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_RegisterMR*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// update mr
void TpComm::handleResponse0x0000C1C2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x0000C1C2: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_RegisterMR*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// delete mr
void TpComm::handleResponse0x000053E2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x000053E2: failed to encode response package");// send
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

// get mr
void TpComm::handleResponse0x000170D2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_RegisterMR_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x000170D2: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_UnsignedInt32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_RegisterMR*)task->response_data_ptr;
    }
}

// set activate mr
void TpComm::handleResponse0x00011642(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x00011642: failed to encode response package");// send
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

// get activate mr
void TpComm::handleResponse0x0000F3B2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
        if(!encodeResponsePackage(task->hash, ResponseMessageType_UnsignedInt32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x0000F3B2: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (ResponseMessageType_UnsignedInt32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}


//add sr
void TpComm::handleResponse0x0000F932(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x0000F932: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_RegisterSR*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// update sr
void TpComm::handleResponse0x0000BF62(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
        if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x0000BF62: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_RegisterSR*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// delete sr
void TpComm::handleResponse0x00005342(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
        if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x00005342: failed to encode response package");// send
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

// get sr
void TpComm::handleResponse0x00017172(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_RegisterSR_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x00017172: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_UnsignedInt32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_RegisterSR*)task->response_data_ptr;
    }
}

// set activate sr
void TpComm::handleResponse0x000115E2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x000115E2: failed to encode response package");// send
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

// get activate sr
void TpComm::handleResponse0x0000F352(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_UnsignedInt32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_INFO("Here : get rpc table : hash = %x, send_buffer_size = %d", task->hash, send_buffer_size);
        FST_ERROR("handleResponse0x0000F352: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (ResponseMessageType_UnsignedInt32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}
