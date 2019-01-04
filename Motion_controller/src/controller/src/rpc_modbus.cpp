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

//"/rpc/modbus/addClient"
void ControllerRpc::handleRpc0x00012E44(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_ModbusClientStartInfo* rq_data_ptr = static_cast<RequestMessageType_ModbusClientStartInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    ModbusClientStartInfo start_info;
    start_info.id = rq_data_ptr->data.id;
    start_info.ip = rq_data_ptr->data.ip;
    start_info.name = rq_data_ptr->data.name;
    start_info.port = rq_data_ptr->data.port;
    start_info.scan_rate = rq_data_ptr->data.scan_rate;
    start_info.response_timeout = rq_data_ptr->data.response_timeout;

    rs_data_ptr->data.data = modbus_manager_ptr_->addClient(start_info);
    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/addClient"));
}

//"/rpc/modbus/deleteClient"
void ControllerRpc::handleRpc0x00014CF4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    rs_data_ptr->data.data = modbus_manager_ptr_->deleteClient(rq_data_ptr->data.data);
    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/deleteClient"));
}

//"/rpc/modbus/getClientIdList"
void ControllerRpc::handleRpc0x000046C4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32List*>(response_data_ptr);
    
    vector<int> id_list;
    id_list.clear();
    
    rs_data_ptr->error_code.data = modbus_manager_ptr_->getClientIdList(id_list);
    rs_data_ptr->data.data_count = id_list.size();

    vector<int>::iterator it = id_list.begin();

    for (int i = 0; i != id_list.size(); ++i)
    {
        rs_data_ptr->data.data[i] = *it;
        it++;
    }
    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getClientIdList"));
}

//"/rpc/modbus/setClientEnableStatus"
void ControllerRpc::handleRpc0x00002AD3(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Bool* rq_data_ptr = static_cast<RequestMessageType_Int32_Bool*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = modbus_manager_ptr_->setClientEnableStatus(
        rq_data_ptr->data1.data, rq_data_ptr->data2.data);

    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/setClientEnableStatus"));
}
//"/rpc/modbus/getClientEnableStatus"
void ControllerRpc::handleRpc0x00018573(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Bool* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Bool*>(response_data_ptr);

    rs_data_ptr->error_code.data = modbus_manager_ptr_->getClientEnableStatus(
        rq_data_ptr->data.data, rs_data_ptr->data.data);

    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("rpc/modbus/getClientEnableStatus"));
}
//"/rpc/modbus/setClientAllFunctionAddrInfo"
void ControllerRpc::handleRpc0x0000A4CF(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_ModbusAllFucntionAddrInfo* rq_data_ptr = static_cast<RequestMessageType_Int32_ModbusAllFucntionAddrInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    ModbusClientRegInfo reg_info;
    reg_info.coil.addr = rq_data_ptr->data2.coil.address;
    reg_info.coil.max_nb = rq_data_ptr->data2.coil.number;
    reg_info.discrepte_input.addr = rq_data_ptr->data2.discrepte_input.address;
    reg_info.discrepte_input.max_nb = rq_data_ptr->data2.discrepte_input.number;
    reg_info.holding_reg.addr = rq_data_ptr->data2.holding_reg.address;
    reg_info.holding_reg.max_nb = rq_data_ptr->data2.holding_reg.number;
    reg_info.input_reg.addr = rq_data_ptr->data2.input_reg.address;
    reg_info.input_reg.max_nb = rq_data_ptr->data2.input_reg.number;

    rs_data_ptr->data.data = modbus_manager_ptr_->setClientRegInfo(rq_data_ptr->data1.data, reg_info);

    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/setClientAllFunctionAddrInfo"));
}
//"/rpc/modbus/getClientAllFunctionAddrInfo"
void ControllerRpc::handleRpc0x0000132F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_ModbusAllFucntionAddrInfo* rs_data_ptr 
        = static_cast<ResponseMessageType_Uint64_ModbusAllFucntionAddrInfo*>(response_data_ptr);

    ModbusClientRegInfo reg_info;
    rs_data_ptr->error_code.data = modbus_manager_ptr_->getClientRegInfo(rq_data_ptr->data.data, reg_info);

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

    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getClientAllFunctionAddrInfo"));
}

//"/rpc/modbus/updateClientStartInfo"
void ControllerRpc::handleRpc0x00008C7F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_ModbusClientStartInfo* rq_data_ptr = static_cast<RequestMessageType_ModbusClientStartInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    ModbusClientStartInfo start_info;
    start_info.id = rq_data_ptr->data.id;
    start_info.ip = rq_data_ptr->data.ip;
    start_info.name = rq_data_ptr->data.name;
    start_info.port = rq_data_ptr->data.port;
    start_info.scan_rate = rq_data_ptr->data.scan_rate;
    start_info.response_timeout = rq_data_ptr->data.response_timeout;

    rs_data_ptr->data.data = modbus_manager_ptr_->updateClientStartInfo(start_info);
    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/updateClientStartInfo"));
}

//"/rpc/modbus/getClientStartInfo"
void ControllerRpc::handleRpc0x0000084F(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_ModbusClientStartInfo* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ModbusClientStartInfo*>(response_data_ptr);

    ModbusClientStartInfo start_info;
    rs_data_ptr->error_code.data = modbus_manager_ptr_->getClientStartInfo(rq_data_ptr->data.data, start_info);

    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.id = start_info.id ;
        strcpy(rs_data_ptr->data.ip, start_info.ip.c_str());
        rs_data_ptr->data.ip[127] = '0';
        strcpy(rs_data_ptr->data.name, start_info.name.c_str());
        rs_data_ptr->data.name[127] = '0';
        rs_data_ptr->data.port = start_info.port;
        rs_data_ptr->data.scan_rate = start_info.scan_rate;
        rs_data_ptr->data.response_timeout = start_info.response_timeout;
    }

    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getClientStartInfo"));
}

//"/rpc/modbus/getClientConfigParams"
void ControllerRpc::handleRpc0x00009833(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_ModbusClientConfigParams* rs_data_ptr 
        = static_cast<ResponseMessageType_Uint64_ModbusClientConfigParams*>(response_data_ptr);

    ModbusClientConfigParams config_params;
    rs_data_ptr->error_code.data = modbus_manager_ptr_->getClientConfigParams(rq_data_ptr->data.data, config_params);

    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.is_enable = config_params.is_enable;

        rs_data_ptr->data.start_info.id = rq_data_ptr->data.data;
        rs_data_ptr->data.start_info.port = config_params.start_info.port;
        rs_data_ptr->data.start_info.scan_rate = config_params.start_info.scan_rate;
        rs_data_ptr->data.start_info.response_timeout = config_params.start_info.response_timeout;

        strcpy(rs_data_ptr->data.start_info.ip, config_params.start_info.ip.c_str());
        rs_data_ptr->data.start_info.ip[127] = '0';
        strcpy(rs_data_ptr->data.start_info.name, config_params.start_info.name.c_str());
        rs_data_ptr->data.start_info.name[127] = '0';

        rs_data_ptr->data.function_addr_info.coil.address = config_params.reg_info.coil.addr;
        rs_data_ptr->data.function_addr_info.coil.number = config_params.reg_info.coil.max_nb;
        rs_data_ptr->data.function_addr_info.discrepte_input.address = config_params.reg_info.discrepte_input.addr;
        rs_data_ptr->data.function_addr_info.discrepte_input.number = config_params.reg_info.discrepte_input.max_nb;
        rs_data_ptr->data.function_addr_info.holding_reg.address = config_params.reg_info.holding_reg.addr;
        rs_data_ptr->data.function_addr_info.holding_reg.number = config_params.reg_info.holding_reg.max_nb;
        rs_data_ptr->data.function_addr_info.input_reg.address = config_params.reg_info.input_reg.addr;
        rs_data_ptr->data.function_addr_info.input_reg.number = config_params.reg_info.input_reg.max_nb;
    }

    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getClientConfigParams"));
}

//"/rpc/modbus/connectClient"
void ControllerRpc::handleRpc0x00014594(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    rs_data_ptr->data.data = modbus_manager_ptr_->connectClient(rq_data_ptr->data.data);
    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/connectClient"));
}

//"/rpc/modbus/closeClient"
void ControllerRpc::handleRpc0x00006CA4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    rs_data_ptr->data.data = modbus_manager_ptr_->closeClient(rq_data_ptr->data.data);
    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/closeClient"));
}

//"/rpc/modbus/isClientConnected"
void ControllerRpc::handleRpc0x00002FC4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Bool* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Bool*>(response_data_ptr);
    rs_data_ptr->error_code.data = modbus_manager_ptr_->isConnected(rq_data_ptr->data.data, rs_data_ptr->data.data);
    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/isClientConnected"));
}

//"/rpc/modbus/getClientCtrlStatus"
void ControllerRpc::handleRpc0x000170E3(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32*>(response_data_ptr);
    rs_data_ptr->error_code.data = modbus_manager_ptr_->getClientCtrlState(rq_data_ptr->data.data, rs_data_ptr->data.data);
    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getClientCtrlStatus"));
}

//"/rpc/modbus/replaceClient"
void ControllerRpc::handleRpc0x0000C2F4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_ModbusClientStartInfo* rq_data_ptr = static_cast<RequestMessageType_Int32_ModbusClientStartInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    ModbusClientStartInfo start_info;
    start_info.id = rq_data_ptr->data2.id;
    start_info.ip = rq_data_ptr->data2.ip;
    start_info.name = rq_data_ptr->data2.name;
    start_info.port = rq_data_ptr->data2.port;
    start_info.scan_rate = rq_data_ptr->data2.scan_rate;
    start_info.response_timeout = rq_data_ptr->data2.response_timeout;

    rs_data_ptr->data.data = modbus_manager_ptr_->replaceClient(rq_data_ptr->data1.data, start_info);
    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/getClientCtrlStatus"));
}

//"/rpc/modbus/getConnectedClientList"
void ControllerRpc::handleRpc0x00001DC4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_Int32List* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Int32List*>(response_data_ptr);

    vector<int> id_list;
    id_list.clear();
    
    rs_data_ptr->error_code.data = modbus_manager_ptr_->getConnectedClientIdList(id_list);
    rs_data_ptr->data.data_count = id_list.size();

    vector<int>::iterator it = id_list.begin();

    for (int i = 0; i != id_list.size(); ++i)
    {
        rs_data_ptr->data.data[i] = *it;
        it++;
    }
    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getConnectedClientList"));
}

