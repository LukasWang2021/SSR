#include "tp_comm.h"

using namespace fst_comm;

// "/rpc/controller/addRegTopic",	0x0000BA13,	"RequestMessageType.Topic",	"ResponseMessageType.Bool",
void TpComm::handleResponse0x0000BA13(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
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

// rpc/reg_manager/r/addReg "RequestMessageType.RRegData",	"ResponseMessageType.Bool"
void TpComm::handleResponse0x00004FF7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_RRegData*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// "/rpc/reg_manager/r/deleteReg" "RequestMessageType.Int32",	"ResponseMessageType.Bool"
void TpComm::handleResponse0x000012F7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
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

// "/rpc/reg_manager/r/updateReg" "RequestMessageType.RRegData",	"ResponseMessageType.Bool"
void TpComm::handleResponse0x00005757(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_RRegData*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// "/rpc/reg_manager/r/getReg" "RequestMessageType.Int32",	"ResponseMessageType.Bool_RRegData"
void TpComm::handleResponse0x0000EAB7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_RRegData_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool_RRegData*)task->response_data_ptr;
    }
}

//  "/rpc/reg_manager/r/moveReg" "RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.Bool
void TpComm::handleResponse0x0000C877(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
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

// "/rpc/reg_manager/r/getChangedList" "RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList
void TpComm::handleResponse0x0000A904(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_BaseRegSummaryList_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_BaseRegSummaryList*)task->response_data_ptr;
    }
}

// "/rpc/reg_manager/r/getValidList" "RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList
void TpComm::handleResponse0x00008CE4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_BaseRegSummaryList_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_BaseRegSummaryList*)task->response_data_ptr;
    }
}



// "/rpc/reg_manager/mr/addReg" "RequestMessageType.MrRegData",	"ResponseMessageType.Bool"
void TpComm::handleResponse0x000097E7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_MrRegData*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// "/rpc/reg_manager/mr/deleteReg" RequestMessageType.Int32",	"ResponseMessageType.Bool",
void TpComm::handleResponse0x0000E5D7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
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

// "/rpc/reg_manager/mr/updateReg" "RequestMessageType.MrRegData",	"ResponseMessageType.Bool"
void TpComm::handleResponse0x0000E9B7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_MrRegData*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// "/rpc/reg_manager/mr/getReg" "RequestMessageType.Int32",	"ResponseMessageType.Bool_MrRegData"
void TpComm::handleResponse0x0000B507(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_MrRegData_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool_MrRegData*)task->response_data_ptr;
    }
}

// "/rpc/reg_manager/mr/moveReg" "RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.Bool
void TpComm::handleResponse0x00015BA7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
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

// "/rpc/reg_manager/mr/getChangedList","RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList
void TpComm::handleResponse0x00001774(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_BaseRegSummaryList_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_BaseRegSummaryList*)task->response_data_ptr;
    }
}

// "/rpc/reg_manager/mr/getValidList",	0x00015CF4,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList
void TpComm::handleResponse0x00015CF4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_BaseRegSummaryList_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_BaseRegSummaryList*)task->response_data_ptr;
    }
}


// "/rpc/reg_manager/sr/addReg",	0x000161E7,	"RequestMessageType.SrRegData",	"ResponseMessageType.Bool"
void TpComm::handleResponse0x000161E7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_SrRegData*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// "/rpc/reg_manager/sr/deleteReg",	0x0000B817,	"RequestMessageType.Int32",	"ResponseMessageType.Bool"
void TpComm::handleResponse0x0000B817(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
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

// "/rpc/reg_manager/sr/updateReg",	0x000119F7,	"RequestMessageType.SrRegData",	"ResponseMessageType.Bool"
void TpComm::handleResponse0x000119F7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_SrRegData*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// "/rpc/reg_manager/sr/getReg",	0x00017F07,	"RequestMessageType.Int32",	"ResponseMessageType.Bool_SrRegData"
void TpComm::handleResponse0x00017F07(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_SrRegData_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool_SrRegData*)task->response_data_ptr;
    }
}

// "/rpc/reg_manager/sr/moveReg",	0x00002127,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.Bool
void TpComm::handleResponse0x00002127(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
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

// "/rpc/reg_manager/sr/getChangedList",	0x00004834,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList"
void TpComm::handleResponse0x00004834(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_BaseRegSummaryList_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_BaseRegSummaryList*)task->response_data_ptr;
    }
}

// "/rpc/reg_manager/sr/getValidList",	0x00009854,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList
void TpComm::handleResponse0x00009854(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_BaseRegSummaryList_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_BaseRegSummaryList*)task->response_data_ptr;
    }
}



// "/rpc/reg_manager/pr/addReg",	0x000154E7,	"RequestMessageType.PrRegData",	"ResponseMessageType.Bool",	
void TpComm::handleResponse0x000154E7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_PrRegData*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// "/rpc/reg_manager/pr/deleteReg",	0x00001097,	"RequestMessageType.Int32",	"ResponseMessageType.Bool",	
void TpComm::handleResponse0x00001097(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
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

// "/rpc/reg_manager/pr/updateReg",	0x00009EF7,	"RequestMessageType.PrRegData",	"ResponseMessageType.Bool",	
void TpComm::handleResponse0x00009EF7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_PrRegData*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// "/rpc/reg_manager/pr/getReg",	0x00017207,	"RequestMessageType.Int32",	"ResponseMessageType.Bool_PrRegData",	
void TpComm::handleResponse0x00017207(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_PrRegData_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool_PrRegData*)task->response_data_ptr;
    }
}

// "/rpc/reg_manager/pr/moveReg",	0x0000D7C7,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.Bool",
void TpComm::handleResponse0x0000D7C7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
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

// "/rpc/reg_manager/pr/getChangedList",	0x0000B454,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList",
void TpComm::handleResponse0x0000B454(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_BaseRegSummaryList_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_BaseRegSummaryList*)task->response_data_ptr;
    }
}

// "/rpc/reg_manager/pr/getValidList",	0x00009354,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList",
void TpComm::handleResponse0x00009354(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_BaseRegSummaryList_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_BaseRegSummaryList*)task->response_data_ptr;
    }
}



// "/rpc/reg_manager/hr/addReg",	0x00016CE7,	"RequestMessageType.HrRegData",	"ResponseMessageType.Bool"
void TpComm::handleResponse0x00016CE7(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_HrRegData*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// "/rpc/reg_manager/hr/deleteReg",	0x00003D17,	"RequestMessageType.Int32",	"ResponseMessageType.Bool",
void TpComm::handleResponse0x00003D17(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
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

// "/rpc/reg_manager/hr/updateReg",	0x0000CB77,	"RequestMessageType.HrRegData",	"ResponseMessageType.Bool"
void TpComm::handleResponse0x0000CB77(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_HrRegData*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool*)task->response_data_ptr;
    }
}

// "/rpc/reg_manager/hr/getReg",	0x00000367,	"RequestMessageType.Int32",	"ResponseMessageType.Bool_HrRegData"
void TpComm::handleResponse0x00000367(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Bool_HrRegData_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Bool_HrRegData*)task->response_data_ptr;
    }
}

// "/rpc/reg_manager/hr/moveReg",	0x00014A87,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.Bool
void TpComm::handleResponse0x00014A87(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
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

// "/rpc/reg_manager/hr/getChangedList",	0x00012974,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList
void TpComm::handleResponse0x00012974(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_BaseRegSummaryList_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_BaseRegSummaryList*)task->response_data_ptr;
    }
}


// "/rpc/reg_manager/hr/getValidList",	0x00006B54,	"RequestMessageType.Int32List(count = 2) ",	"ResponseMessageType.BaseRegSummaryList
void TpComm::handleResponse0x00006B54(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_BaseRegSummaryList_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32List*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_BaseRegSummaryList*)task->response_data_ptr;
    }
}
