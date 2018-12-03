#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

#include "error_monitor.h"
#include "error_code.h"
#include "tp_comm.h"

using namespace fst_base;
using namespace fst_comm;

// "/rpc/modbus/newServer"
void TpComm::handleRequest0x0000D1B2(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_ModbusServerConfig* request_data_ptr = new RequestMessageType_ModbusServerConfig;
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

    handleRequestPackage(0x0000D1B2, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_ModbusServerConfig_fields, -1);
}
// "/rpc/modbus/deleteServer"
void TpComm::handleRequest0x00006C22(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
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

    handleRequestPackage(0x00006C22, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

// "/rpc/modbus/saveServerConfig"
void TpComm::handleRequest0x000050E7(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_ModbusServerConfig* request_data_ptr = new RequestMessageType_ModbusServerConfig;
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

    handleRequestPackage(0x000050E7, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_ModbusServerConfig_fields, -1);
}

// "/rpc/modbus/newClient"
void TpComm::handleRequest0x00009F84(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_ModbusClientConfig* request_data_ptr = new RequestMessageType_ModbusClientConfig;
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

    handleRequestPackage(0x00009F84, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_ModbusClientConfig_fields, -1);
}

// "/rpc/modbus/deleteClient"
void TpComm::handleRequest0x00014CF4(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
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

    handleRequestPackage(0x00014CF4, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

// "/rpc/modbus/saveClientConfig"
void TpComm::handleRequest0x00002B57(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_ModbusClientConfig* request_data_ptr = new RequestMessageType_ModbusClientConfig;
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

    handleRequestPackage(0x00002B57, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_ModbusClientConfig_fields, -1);
}

//"/rpc/modbus/getServerConfig"
void TpComm::handleRequest0x00016947(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_ModbusServerConfig* response_data_ptr = new ResponseMessageType_Uint64_ModbusServerConfig;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00016947, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/modbus/getClientConfig"
void TpComm::handleRequest0x0000FC17(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_ModbusClientConfig* response_data_ptr = new ResponseMessageType_Uint64_ModbusClientConfig;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000FC17, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/modbus/getConnectionStatus"
void TpComm::handleRequest0x0000E973(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Bool* response_data_ptr = new ResponseMessageType_Uint64_Bool;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("handleRequest: can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000E973, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

// "/rpc/modbus/setConnectionStatus"
void TpComm::handleRequest0x00010C63(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Bool* request_data_ptr = new RequestMessageType_Bool;
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

    handleRequestPackage(0x00010C63, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Bool_fields, -1);
}
