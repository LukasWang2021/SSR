#include "tp_comm.h"

using namespace fst_comm;

//"/rpc/modbus/setStartMode"
void TpComm::handleResponse0x0000D3A5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
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

//"/rpc/modbus/getStartMode"
void TpComm::handleResponse0x000041C5(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Int32_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Int32*)task->response_data_ptr;
    }
}

//"/rpc/modbus/setServerEnableStatus"
void TpComm::handleResponse0x00004033(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Bool*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/modbus/getServerEnableStatus"
void TpComm::handleResponse0x00004C33(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Bool*)task->response_data_ptr;
    }
}

//"/rpc/modbus/setServerStartInfo"
void TpComm::handleResponse0x0001300F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_ModbusServerStartInfo*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/modbus/getServerStartInfo"
void TpComm::handleResponse0x000018AF(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_ModbusServerStartInfo_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_ModbusServerStartInfo*)task->response_data_ptr;
    }
}

//"/rpc/modbus/setServerAllFunctionAddrInfo"
void TpComm::handleResponse0x0000A4BF(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_ModbusAllFucntionAddrInfo*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/modbus/getServerAllFunctionAddrInfo"
void TpComm::handleResponse0x00005E1F(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_ModbusAllFucntionAddrInfo_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_ModbusAllFucntionAddrInfo*)task->response_data_ptr;
    }
}
//"/rpc/modbus/getServerConfigParams"
void TpComm::handleResponse0x0000E2E3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_ModbusServerConfigParams_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_ModbusServerConfigParams*)task->response_data_ptr;
    }
}
//"/rpc/modbus/openServer"
void TpComm::handleResponse0x00010912(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}
//"/rpc/modbus/closeServer"
void TpComm::handleResponse0x000045B2(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/modbus/getServerRunningStatus"
void TpComm::handleResponse0x00000953(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_Bool_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Void*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_Bool*)task->response_data_ptr;
    }
}


//"/rpc/modbus/writeCoils"
void TpComm::handleResponse0x0000BD83(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_ModbusStatusInfo*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/modbus/readCoils"
void TpComm::handleResponse0x0000A433(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_ModbusStatusInfo_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_ModbusFunctionAddrInfo*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_ModbusStatusInfo*)task->response_data_ptr;
    }
}

//"/rpc/modbus/readDiscreteInputs"
void TpComm::handleResponse0x0000C063(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_ModbusStatusInfo_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_ModbusFunctionAddrInfo*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_ModbusStatusInfo*)task->response_data_ptr;
    }
}

//"/rpc/modbus/writeHoldingRegs"
void TpComm::handleResponse0x00008C43(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_ModbusRegInfo*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64*)task->response_data_ptr;
    }
}

//"/rpc/modbus/readHoldingRegs"
void TpComm::handleResponse0x00003583(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_ModbusRegInfo_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_ModbusFunctionAddrInfo*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_ModbusRegInfo*)task->response_data_ptr;
    }
}

//"/rpc/modbus/readInputRegs"
void TpComm::handleResponse0x000072C3(std::vector<TpRequestResponse>::iterator& task, int& send_buffer_size)
{
    if(!encodeResponsePackage(task->hash, ResponseMessageType_Uint64_ModbusRegInfo_fields, task->response_data_ptr, send_buffer_size))
    {
        FST_ERROR("handleResponse: failed to encode response package");// send
    }
    if(task->request_data_ptr != NULL)
    {
        delete (RequestMessageType_Int32_ModbusFunctionAddrInfo*)task->request_data_ptr;
    }
    if(task->response_data_ptr != NULL)
    {
        delete (ResponseMessageType_Uint64_ModbusRegInfo*)task->response_data_ptr;
    }
}

