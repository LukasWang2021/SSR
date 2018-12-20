#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

#include "error_monitor.h"
#include "error_code.h"
#include "tp_comm.h"

using namespace fst_base;
using namespace fst_comm;

void TpComm::handleRequest0x0000C1E0(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_DeviceInfoList* response_data_ptr = new ResponseMessageType_Uint64_DeviceInfoList;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x0000C1E0, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/device_manager/get_FRP8A_IoDeviceInfo"
void TpComm::handleRequest0x00006BAF(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_IoDeviceInfo* response_data_ptr = new ResponseMessageType_Uint64_IoDeviceInfo;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00006BAF, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}
//"/rpc/device_manager/getModbusIoDeviceInfo"
void TpComm::handleRequest0x0001421F(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_IoDeviceInfo* response_data_ptr = new ResponseMessageType_Uint64_IoDeviceInfo;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0001421F, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/device_manager/getIoDeviceInfoList"
void TpComm::handleRequest0x000024A4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_IoDeviceInfoList* response_data_ptr = new ResponseMessageType_Uint64_IoDeviceInfoList;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x000024A4, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}
