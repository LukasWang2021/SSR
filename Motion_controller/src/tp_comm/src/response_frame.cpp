#include "tp_comm.h"

using namespace user_space;
using namespace log_space;
using namespace std;


/********rpc/tool_manager/addTool, ResponseMessageType_Uint64**********/
void TpComm::handleResponse0x0000A22C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x0000A22C: failed to encode response package");
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_ToolInfo*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

/********rpc/tool_manager/deleteTool, ResponseMessageType_Uint64**********/	
void TpComm::handleResponse0x00010E4C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x00010E4C: failed to encode response package");
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

/********rpc/tool_manager/updateTool, ResponseMessageType_Uint64**********/
void TpComm::handleResponse0x0000C78C(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x0000C78C: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_ToolInfo*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

/********rpc/tool_manager/moveTool, ResponseMessageType_Uint64**********/	
void TpComm::handleResponse0x000085FC(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x000085FC: failed to encode response package");// send
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

/********rpc/tool_manager/getToolInfoById, ResponseMessageType_Uint64_ToolInfo**********/	
void TpComm::handleResponse0x00009E34(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_ToolInfo_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x00009E34: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_ToolInfo*)task->response_data_ptr;
    }
}

/********rpc/tool_manager/getAllValidToolSummaryInfo, ResponseMessageType_Uint64_ToolSummaryList**********/
void TpComm::handleResponse0x0001104F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_ToolSummaryList_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x0001104F: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_ToolSummaryList*)task->response_data_ptr;
    }
}


/********rpc/coordinate_manager/addUserCoord, ResponseMessageType_Uint64**********/
void TpComm::handleResponse0x00016764(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x00016764: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_UserCoordInfo*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

/********rpc/coordinate_manager/deleteUserCoord, ResponseMessageType_Uint64**********/
void TpComm::handleResponse0x0000BAF4(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x0000BAF4: failed to encode response package");// send
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

/********rpc/coordinate_manager/updateUserCoord, ResponseMessageType_Uint64**********/	
void TpComm::handleResponse0x0000EC14(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x0000EC14: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_UserCoordInfo*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

/********rpc/coordinate_manager/moveUserCoord, ResponseMessageType_Uint64**********/
void TpComm::handleResponse0x0000E104(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x0000E104: failed to encode response package");// send
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

/********rpc/coordinate_manager/getUserCoordInfoById, ResponseMessageType_Uint64_UserCoordInfo**********/
void TpComm::handleResponse0x00004324(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_UserCoordInfo_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x00004324: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_UserCoordInfo*)task->response_data_ptr;
    }
}

/********rpc/coordinate_manager/getAllValidUserCoordSummaryInfo, ResponseMessageType_Uint64_UserCoordSummaryList**********/
void TpComm::handleResponse0x0001838F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_UserCoordSummaryList_fields, task->response_data_ptr, send_buffer_size))
    {
        LogProducer::error("comm", "handleResponse0x0001838F: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_UserCoordSummaryList*)task->response_data_ptr;
    }
}
