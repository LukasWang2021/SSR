#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>
#include "error_monitor.h"
#include "error_code.h"
#include "tp_comm.h"

using namespace fst_base;
using namespace fst_comm;
using namespace std;

//"/rpc/param_manager/getParamInfoList"
void TpComm::handleRequest0x0000F0B4(int recv_bytes)
{
   // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_ParamInfoList* response_data_ptr = new ResponseMessageType_Uint64_ParamInfoList;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x0000F0B4, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}
//"/rpc/param_manager/setParamInfo"
void TpComm::handleRequest0x0001393F(int recv_bytes)
{
   // create object for request and response package
    RequestMessageType_Int32_ParamInfo* request_data_ptr = new RequestMessageType_Int32_ParamInfo;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x0001393F, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_ParamInfo_fields, -1);
}
