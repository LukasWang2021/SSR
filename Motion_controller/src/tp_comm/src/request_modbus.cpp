#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

#include "error_monitor.h"
#include "error_code.h"
#include "tp_comm.h"

using namespace fst_base;
using namespace fst_comm;

//"/rpc/modbus/setStartMode"
void TpComm::handleRequest0x0000D3A5(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32* request_data_ptr = new RequestMessageType_Int32;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x0000D3A5, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_fields, -1);
}

//"/rpc/modbus/getStartMode"
void TpComm::handleRequest0x000041C5(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Int32* response_data_ptr = new ResponseMessageType_Uint64_Int32;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x000041C5, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/modbus/setServerEnableStatus"
void TpComm::handleRequest0x00004033(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Bool* request_data_ptr = new RequestMessageType_Bool;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00004033, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Bool_fields, -1);
}

//"/rpc/modbus/getServerEnableStatus"
void TpComm::handleRequest0x00004C33(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Bool* response_data_ptr = new ResponseMessageType_Uint64_Bool;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00004C33, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/modbus/setServerStartInfo"
void TpComm::handleRequest0x0001300F(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_ModbusServerStartInfo* request_data_ptr = new RequestMessageType_ModbusServerStartInfo;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x0001300F, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_ModbusServerStartInfo_fields, -1);
}

//"/rpc/modbus/getServerStartInfo"
void TpComm::handleRequest0x000018AF(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_ModbusServerStartInfo* response_data_ptr = new ResponseMessageType_Uint64_ModbusServerStartInfo;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x000018AF, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/modbus/setServerAllFunctionAddrInfo"
void TpComm::handleRequest0x0000A4BF(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_ModbusAllFucntionAddrInfo* request_data_ptr = new RequestMessageType_ModbusAllFucntionAddrInfo;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x0000A4BF, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_ModbusAllFucntionAddrInfo_fields, -1);
}

//"/rpc/modbus/getServerAllFunctionAddrInfo"
void TpComm::handleRequest0x00005E1F(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_ModbusAllFucntionAddrInfo* response_data_ptr = new ResponseMessageType_Uint64_ModbusAllFucntionAddrInfo;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00005E1F, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/modbus/getServerConfigParams"
void TpComm::handleRequest0x0000E2E3(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_ModbusServerConfigParams* response_data_ptr = new ResponseMessageType_Uint64_ModbusServerConfigParams;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x0000E2E3, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/modbus/openServer"
void TpComm::handleRequest0x00010912(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x00010912, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/modbus/closeServer"
void TpComm::handleRequest0x000045B2(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }
    
    handleRequestPackage(0x000045B2, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}

//"/rpc/modbus/getServerRunningStatus"
void TpComm::handleRequest0x00000953(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Void* request_data_ptr = new RequestMessageType_Void;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_Bool* response_data_ptr = new ResponseMessageType_Uint64_Bool;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00000953, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Void_fields, -1);
}


//"/rpc/modbus/writeCoils"
void TpComm::handleRequest0x0000BD83(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_ModbusStatusInfo* request_data_ptr = new RequestMessageType_Int32_ModbusStatusInfo;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000BD83, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_ModbusStatusInfo_fields, -1);
}

//"/rpc/modbus/readCoils"
void TpComm::handleRequest0x0000A433(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_ModbusFunctionAddrInfo* request_data_ptr = new RequestMessageType_Int32_ModbusFunctionAddrInfo;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_ModbusStatusInfo* response_data_ptr = new ResponseMessageType_Uint64_ModbusStatusInfo;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000A433, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_ModbusFunctionAddrInfo_fields, -1);
}

//"/rpc/modbus/readDiscreteInputs"
void TpComm::handleRequest0x0000C063(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_ModbusFunctionAddrInfo* request_data_ptr = new RequestMessageType_Int32_ModbusFunctionAddrInfo;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_ModbusStatusInfo* response_data_ptr = new ResponseMessageType_Uint64_ModbusStatusInfo;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x0000C063, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_ModbusFunctionAddrInfo_fields, -1);
}

//"/rpc/modbus/writeHoldingRegs"
void TpComm::handleRequest0x00008C43(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_ModbusRegInfo* request_data_ptr = new RequestMessageType_Int32_ModbusRegInfo;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64* response_data_ptr = new ResponseMessageType_Uint64;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00008C43, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_ModbusRegInfo_fields, -1);
}

//"/rpc/modbus/readHoldingRegs"
void TpComm::handleRequest0x00003583(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_ModbusFunctionAddrInfo* request_data_ptr = new RequestMessageType_Int32_ModbusFunctionAddrInfo;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_ModbusRegInfo* response_data_ptr = new ResponseMessageType_Uint64_ModbusRegInfo;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x00003583, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_ModbusFunctionAddrInfo_fields, -1);
}

//"/rpc/modbus/readInputRegs"
void TpComm::handleRequest0x000072C3(int recv_bytes)
{
    // create object for request and response package
    RequestMessageType_Int32_ModbusFunctionAddrInfo* request_data_ptr = new RequestMessageType_Int32_ModbusFunctionAddrInfo;
    if(request_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for request_data");
        return;
    }
    ResponseMessageType_Uint64_ModbusRegInfo* response_data_ptr = new ResponseMessageType_Uint64_ModbusRegInfo;
    if(response_data_ptr == NULL)
    {
        ErrorMonitor::instance()->add(TP_COMM_MEMORY_OPERATION_FAILED);
        FST_ERROR("Can't allocate memory for response_data");
        delete request_data_ptr;
        return;
    }

    handleRequestPackage(0x000072C3, (void*)request_data_ptr, (void*)response_data_ptr, 
        recv_bytes, RequestMessageType_Int32_ModbusFunctionAddrInfo_fields, -1);
}


