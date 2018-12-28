#include "controller_rpc.h"

using namespace fst_ctrl;

//"/rpc/modbus/setStartMode"
void ControllerRpc::handleRpc0x0000D3A5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = modbus_manager_ptr_->setStartMode(rq_data_ptr->data.data);

    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/setStartMode"));
}

//"/rpc/modbus/getStartMode"	
void ControllerRpc::handleRpc0x000041C5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);

    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.data = modbus_manager_ptr_->getStartMode();

    recordLog(INTERPRETER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getStartMode"));
}
//"/rpc/modbus/setServerEnableStatus"
void ControllerRpc::handleRpc0x00004033(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Bool* rq_data_ptr = static_cast<RequestMessageType_Bool*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = modbus_manager_ptr_->setServerEnableStatus(rq_data_ptr->data.data);

    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/setServerEnableStatus"));
}
//"/rpc/modbus/getServerEnableStatus"	
void ControllerRpc::handleRpc0x00004C33(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_Bool* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Bool*>(response_data_ptr);

    rs_data_ptr->error_code.data = modbus_manager_ptr_->getServerEnableStatus(rs_data_ptr->data.data);

    recordLog(INTERPRETER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getServerEnableStatus"));

}
//"/rpc/modbus/setServerStartInfo"	
void ControllerRpc::handleRpc0x0001300F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_ModbusServerStartInfo* rq_data_ptr = static_cast<RequestMessageType_ModbusServerStartInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    ModbusServerStartInfo start_info;
    start_info.name = rq_data_ptr->data.name;
    start_info.response_delay = rq_data_ptr->data.response_delay;
    start_info.ip = rq_data_ptr->data.ip;

    rs_data_ptr->data.data = modbus_manager_ptr_->setServerStartInfo(start_info);

    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/setServerStartInfo"));
}

//"/rpc/modbus/getServerStartInfo"	
void ControllerRpc::handleRpc0x000018AF(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_ModbusServerStartInfo* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ModbusServerStartInfo*>(response_data_ptr);

    ModbusServerStartInfo start_info;
    rs_data_ptr->error_code.data = modbus_manager_ptr_->getServerStartInfo(start_info);

    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.has_ip = true;
        strcpy(rs_data_ptr->data.ip, start_info.ip.c_str());
        rs_data_ptr->data.ip[127] = '0';
        strcpy(rs_data_ptr->data.name, start_info.name.c_str());
        rs_data_ptr->data.name[127] = '0';
        rs_data_ptr->data.response_delay = start_info.response_delay;
    }

    recordLog(INTERPRETER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getServerStartInfo"));
}

//"/rpc/modbus/setServerAllFunctionAddrInfo"	
void ControllerRpc::handleRpc0x0000A4BF(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_ModbusAllFucntionAddrInfo* rq_data_ptr = static_cast<RequestMessageType_ModbusAllFucntionAddrInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    ModbusServerRegInfo reg_info;
    reg_info.coil.addr = rq_data_ptr->data.coil.address;
    reg_info.coil.max_nb = rq_data_ptr->data.coil.number;
    reg_info.discrepte_input.addr = rq_data_ptr->data.discrepte_input.address;
    reg_info.discrepte_input.max_nb = rq_data_ptr->data.discrepte_input.number;
    reg_info.holding_reg.addr = rq_data_ptr->data.holding_reg.address;
    reg_info.holding_reg.max_nb = rq_data_ptr->data.holding_reg.number;
    reg_info.input_reg.addr = rq_data_ptr->data.input_reg.address;
    reg_info.input_reg.max_nb = rq_data_ptr->data.input_reg.number;

    rs_data_ptr->data.data = modbus_manager_ptr_->setServerRegInfo(reg_info);

    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/setServerAllFunctionAddrInfo"));
}

//"/rpc/modbus/getServerAllFunctionAddrInfo"
void ControllerRpc::handleRpc0x00005E1F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_ModbusAllFucntionAddrInfo* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ModbusAllFucntionAddrInfo*>(response_data_ptr);

    ModbusServerRegInfo reg_info;
    rs_data_ptr->error_code.data = modbus_manager_ptr_->getServerRegInfo(reg_info);

    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.coil.address = reg_info.coil.addr;
        rs_data_ptr->data.coil.number = reg_info.coil.max_nb;
        rs_data_ptr->data.discrepte_input.address = reg_info.discrepte_input.addr;
        rs_data_ptr->data.discrepte_input.number = reg_info.discrepte_input.max_nb;
        rs_data_ptr->data.holding_reg.address = reg_info.holding_reg.addr;
        rs_data_ptr->data.holding_reg.number = reg_info.holding_reg.max_nb;
        rs_data_ptr->data.input_reg.address = reg_info.input_reg.addr;
        rs_data_ptr->data.input_reg.number = reg_info.input_reg.max_nb;
    }

    recordLog(INTERPRETER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getServerAllFunctionAddrInfo"));
}
//"/rpc/modbus/getServerConfigParams"
void ControllerRpc::handleRpc0x0000E2E3(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_ModbusServerConfigParams* rs_data_ptr = 
        static_cast<ResponseMessageType_Uint64_ModbusServerConfigParams*>(response_data_ptr);

    ModbusServerConfigParams params;
    rs_data_ptr->error_code.data = modbus_manager_ptr_->getServerConfigParams(params);

    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.is_enable = params.is_enable;
        rs_data_ptr->data.start_info.has_ip = true;
        strcpy(rs_data_ptr->data.start_info.ip, params.start_info.ip.c_str());
        rs_data_ptr->data.start_info.ip[127] = '0';
        strcpy(rs_data_ptr->data.start_info.name, params.start_info.name.c_str());
        rs_data_ptr->data.start_info.name[127] = '0';
        rs_data_ptr->data.start_info.response_delay = params.start_info.response_delay;

        rs_data_ptr->data.function_addr_info.coil.address = params.reg_info.coil.addr;
        rs_data_ptr->data.function_addr_info.coil.number = params.reg_info.coil.max_nb;
        rs_data_ptr->data.function_addr_info.discrepte_input.address = params.reg_info.discrepte_input.addr;
        rs_data_ptr->data.function_addr_info.discrepte_input.number = params.reg_info.discrepte_input.max_nb;
        rs_data_ptr->data.function_addr_info.holding_reg.address = params.reg_info.holding_reg.addr;
        rs_data_ptr->data.function_addr_info.holding_reg.number = params.reg_info.holding_reg.max_nb;
        rs_data_ptr->data.function_addr_info.input_reg.address = params.reg_info.input_reg.addr;
        rs_data_ptr->data.function_addr_info.input_reg.number = params.reg_info.input_reg.max_nb;
    }

    recordLog(INTERPRETER_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getServerConfigParams"));
}

//"/rpc/modbus/openServer"	
void ControllerRpc::handleRpc0x00010912(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = modbus_manager_ptr_->openServer();

    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/openServer"));
}
//"/rpc/modbus/closeServer"	
void ControllerRpc::handleRpc0x000045B2(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = modbus_manager_ptr_->closeServer();

    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/closeServer"));
}
//"/rpc/modbus/getServerRunningStatus"
void ControllerRpc::handleRpc0x00000953(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_Bool* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Bool*>(response_data_ptr);

    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.data = modbus_manager_ptr_->isServerRunning();

    recordLog(INTERPRETER_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/getServerRunningStatus"));
}

//"/rpc/modbus/writeCoils"
void ControllerRpc::handleRpc0x0000BD83(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_ModbusStatusInfo* rq_data_ptr = static_cast<RequestMessageType_Int32_ModbusStatusInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (rq_data_ptr->data2.number != rq_data_ptr->data2.value_count)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/writeCoils"));
    }

    uint8_t value[rq_data_ptr->data2.value_count];

    for (unsigned int i = 0; i != rq_data_ptr->data2.value_count; ++i)
    {
        value[i] = static_cast<uint8_t>(rq_data_ptr->data2.value[i]);
    }

    rs_data_ptr->data.data = modbus_manager_ptr_->writeCoils(rq_data_ptr->data1.data, rq_data_ptr->data2.address,
        rq_data_ptr->data2.value_count, value);

    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/writeCoils"));
}

//"/rpc/modbus/readCoils"
void ControllerRpc::handleRpc0x0000A433(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_ModbusFunctionAddrInfo* rq_data_ptr = static_cast<RequestMessageType_Int32_ModbusFunctionAddrInfo*>(request_data_ptr);
    ResponseMessageType_Uint64_ModbusStatusInfo* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ModbusStatusInfo*>(response_data_ptr);

    uint8_t value[rq_data_ptr->data2.number];

    rs_data_ptr->error_code.data = modbus_manager_ptr_->readCoils(rq_data_ptr->data1.data, 
        rq_data_ptr->data2.address, rq_data_ptr->data2.number, value);

    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.address = rq_data_ptr->data2.address;
        rs_data_ptr->data.number = rq_data_ptr->data2.number;
        rs_data_ptr->data.value_count = rq_data_ptr->data2.number;

        for (int i = 0; i != rs_data_ptr->data.value_count; ++i)
        {
            rs_data_ptr->data.value[i] = static_cast<bool>(value[i]);
        }
    }
    else 
    {
        rs_data_ptr->data.address = rq_data_ptr->data2.address;
        rs_data_ptr->data.number = rq_data_ptr->data2.number;
        rs_data_ptr->data.value_count = 0;
    }

    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/readCoils"));
}

//"/rpc/modbus/readDiscreteInputs"
void ControllerRpc::handleRpc0x0000C063(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_ModbusFunctionAddrInfo* rq_data_ptr = static_cast<RequestMessageType_Int32_ModbusFunctionAddrInfo*>(request_data_ptr);
    ResponseMessageType_Uint64_ModbusStatusInfo* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ModbusStatusInfo*>(response_data_ptr);

    uint8_t value[rq_data_ptr->data2.number];

    rs_data_ptr->error_code.data = modbus_manager_ptr_->readDiscreteInputs(rq_data_ptr->data1.data, 
        rq_data_ptr->data2.address, rq_data_ptr->data2.number, value);

    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.address = rq_data_ptr->data2.address;
        rs_data_ptr->data.number = rq_data_ptr->data2.number;
        rs_data_ptr->data.value_count = rq_data_ptr->data2.number;

        for (int i = 0; i != rs_data_ptr->data.value_count; ++i)
        {
            rs_data_ptr->data.value[i] = static_cast<bool>(value[i]);
        }
    }
    else 
    {
        rs_data_ptr->data.address = rq_data_ptr->data2.address;
        rs_data_ptr->data.number = rq_data_ptr->data2.number;
        rs_data_ptr->data.value_count = 0;
    }
    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/readDiscreteInputs"));
}

//"/rpc/modbus/writeHoldingRegs"
void ControllerRpc::handleRpc0x00008C43(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_ModbusRegInfo* rq_data_ptr = static_cast<RequestMessageType_Int32_ModbusRegInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (rq_data_ptr->data2.number != rq_data_ptr->data2.value_count)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/writeHoldingRegs"));
    }

    uint16_t value[rq_data_ptr->data2.value_count];

    for (unsigned int i = 0; i != rq_data_ptr->data2.value_count; ++i)
    {
        value[i] = static_cast<uint16_t>(rq_data_ptr->data2.value[i]);
    }

    rs_data_ptr->data.data = modbus_manager_ptr_->writeHoldingRegs(rq_data_ptr->data1.data, rq_data_ptr->data2.address,
        rq_data_ptr->data2.value_count, value);

    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/writeHoldingRegs"));
}

//"/rpc/modbus/readHoldingRegs"
void ControllerRpc::handleRpc0x00003583(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_ModbusFunctionAddrInfo* rq_data_ptr = static_cast<RequestMessageType_Int32_ModbusFunctionAddrInfo*>(request_data_ptr);
    ResponseMessageType_Uint64_ModbusRegInfo* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ModbusRegInfo*>(response_data_ptr);

    uint16_t value[rq_data_ptr->data2.number];

    rs_data_ptr->error_code.data = modbus_manager_ptr_->readHoldingRegs(rq_data_ptr->data1.data,
        rq_data_ptr->data2.address, rq_data_ptr->data2.number, value);

    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.address = rq_data_ptr->data2.address;
        rs_data_ptr->data.number = rq_data_ptr->data2.number;
        rs_data_ptr->data.value_count = rq_data_ptr->data2.number;

        for (int i = 0; i != rs_data_ptr->data.value_count; ++i)
        {
            rs_data_ptr->data.value[i] = static_cast<uint16_t>(value[i]);
        }
    }
    else 
    {
        rs_data_ptr->data.address = rq_data_ptr->data2.address;
        rs_data_ptr->data.number = rq_data_ptr->data2.number;
        rs_data_ptr->data.value_count = 0;
    }

    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/readHoldingRegs"));
}

//"/rpc/modbus/readInputRegs"
void ControllerRpc::handleRpc0x000072C3(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_ModbusFunctionAddrInfo* rq_data_ptr = static_cast<RequestMessageType_Int32_ModbusFunctionAddrInfo*>(request_data_ptr);
    ResponseMessageType_Uint64_ModbusRegInfo* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ModbusRegInfo*>(response_data_ptr);

    uint16_t value[rq_data_ptr->data2.number];

    rs_data_ptr->error_code.data = modbus_manager_ptr_->readInputRegs(
        rq_data_ptr->data1.data, rq_data_ptr->data2.address, rq_data_ptr->data2.number, value);

    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.address = rq_data_ptr->data2.address;
        rs_data_ptr->data.number = rq_data_ptr->data2.number;
        rs_data_ptr->data.value_count = rq_data_ptr->data2.number;

        for (int i = 0; i != rs_data_ptr->data.value_count; ++i)
        {
            rs_data_ptr->data.value[i] = static_cast<uint16_t>(value[i]);
        }
    }
    else 
    {
        rs_data_ptr->data.address = rq_data_ptr->data2.address;
        rs_data_ptr->data.number = rq_data_ptr->data2.number;
        rs_data_ptr->data.value_count = 0;
    }

    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/readInputRegs"));
}