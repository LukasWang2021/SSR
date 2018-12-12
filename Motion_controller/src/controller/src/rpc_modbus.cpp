#include "controller_rpc.h"

using namespace fst_ctrl;

//"/rpc/modbus/setStartMode"
void ControllerRpc::handleRpc0x0000D3A5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_ModbusStartMode* rq_data_ptr = static_cast<RequestMessageType_ModbusStartMode*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = modbus_manager_ptr_->setStartMode(rq_data_ptr->data.start_mode);

    printf("rq_data_ptr->data.start_mode = %d\n", rq_data_ptr->data.start_mode);
    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/setStartMode"));
}

//"/rpc/modbus/getStartMode"
void ControllerRpc::handleRpc0x000041C5(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_ModbusStartMode* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ModbusStartMode*>(response_data_ptr);

    rs_data_ptr->error_code.data = SUCCESS;
    int start_mode = modbus_manager_ptr_->getStartMode();
    rs_data_ptr->data.start_mode =  static_cast<MessageType_ModbusStartModeType>(start_mode);

    printf("rq_data_ptr->data.start_mode = %d\n", rs_data_ptr->data.start_mode);

    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getStartMode"));
}

//"/rpc/modbus/setServerConnectStatus"
void ControllerRpc::handleRpc0x0000DB23(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Bool* rq_data_ptr = static_cast<RequestMessageType_Bool*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);
    rs_data_ptr->data.data = modbus_manager_ptr_->setConnectStatusToServer(rq_data_ptr->data.data);

    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/setServerConnectStatus"));
}

//"/rpc/modbus/getServerConnectStatus"
void ControllerRpc::handleRpc0x00001B23(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_Bool* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Bool*>(response_data_ptr);
    rs_data_ptr->error_code.data = modbus_manager_ptr_->getConnectStatusFromServer(rs_data_ptr->data.data);

    if (rs_data_ptr->data.data)
    {
        printf("get connect status is true : %d\n", rs_data_ptr->data.data);
    }
    else
    {
        printf("get connect status is false\n");
    }

    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getServerConnectStatus"));
}

//"/rpc/modbus/setServerConfig"
void ControllerRpc::handleRpc0x00017547(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_ModbusServerConfig* rq_data_ptr = static_cast<RequestMessageType_ModbusServerConfig*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    fst_hal::ModbusServerConfig server_config;
    server_config.response_delay = rq_data_ptr->data.response_delay;
    server_config.reg_info.coil.addr = rq_data_ptr->data.coil.address;
    server_config.reg_info.coil.max_nb = rq_data_ptr->data.coil.number;
    server_config.reg_info.discrepte_input.addr = rq_data_ptr->data.discrepte_input.address;
    server_config.reg_info.discrepte_input.max_nb = rq_data_ptr->data.discrepte_input.number;
    server_config.reg_info.holding_reg.addr = rq_data_ptr->data.holding_reg.address;
    server_config.reg_info.holding_reg.max_nb = rq_data_ptr->data.holding_reg.number;
    server_config.reg_info.input_reg.addr = rq_data_ptr->data.input_reg.address;
    server_config.reg_info.input_reg.max_nb = rq_data_ptr->data.input_reg.number;

    rs_data_ptr->data.data = modbus_manager_ptr_->setConfigToServer(server_config);
    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/setServerConfig"));
}

//"/rpc/modbus/getServerConfig"
void ControllerRpc::handleRpc0x00016947(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_ModbusServerConfig* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ModbusServerConfig*>(response_data_ptr);

    *rs_data_ptr = ResponseMessageType_Uint64_ModbusServerConfig_init_default;

    fst_hal::ModbusServerConfig server_config;
    rs_data_ptr->error_code.data = modbus_manager_ptr_->getConfigFromServer(server_config);

    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.response_delay = server_config.response_delay;
        rs_data_ptr->data.coil.address = server_config.reg_info.coil.addr;
        rs_data_ptr->data.coil.number = server_config.reg_info.coil.max_nb;
        rs_data_ptr->data.discrepte_input.address = server_config.reg_info.discrepte_input.addr;
        rs_data_ptr->data.discrepte_input.number = server_config.reg_info.discrepte_input.max_nb;
        rs_data_ptr->data.holding_reg.address = server_config.reg_info.holding_reg.addr;
        rs_data_ptr->data.holding_reg.number = server_config.reg_info.holding_reg.max_nb;
        rs_data_ptr->data.input_reg.address = server_config.reg_info.input_reg.addr;
        rs_data_ptr->data.input_reg.number = server_config.reg_info.input_reg.max_nb;
    }

    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getServerConfig"));
}

//"/rpc/modbus/getServerStartInfo"
void ControllerRpc::handleRpc0x000018AF(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_ModbusServerStartInfo* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ModbusServerStartInfo*>(response_data_ptr);

    ModbusServerStartInfo server_start_info;
    rs_data_ptr->error_code.data = modbus_manager_ptr_->getStartInfoFromServer(server_start_info);

    if (rs_data_ptr->error_code.data == SUCCESS)
    {
        rs_data_ptr->data.port = server_start_info.port;
        strcpy(rs_data_ptr->data.ip, server_start_info.ip.c_str());
        rs_data_ptr->data.ip[127] = '0';
    }

    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getServerStartInfo"));
}

//"/rpc/modbus/openServer"
void ControllerRpc::handleRpc0x00010912(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = modbus_manager_ptr_->openServer();
    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/openServer"));
}

//"/rpc/modbus/closeServer"
void ControllerRpc::handleRpc0x000045B2(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = modbus_manager_ptr_->closeServer();
    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/closeServer"));
}

//"/rpc/modbus/addClient"
void ControllerRpc::handleRpc0x00012E44(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = modbus_manager_ptr_->addClient(rq_data_ptr->data.data);
    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/addClient"));
}

//"/rpc/modbus/setClientConnectStatus"
void ControllerRpc::handleRpc0x000099D3(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32_Bool* rq_data_ptr = static_cast<RequestMessageType_Int32_Bool*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = modbus_manager_ptr_->setConnectStatusToClient(rq_data_ptr->data1.data, rq_data_ptr->data2.data);
    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/setClientConnectStatus"));
}

//"/rpc/modbus/getClientConnectStatus"
void ControllerRpc::handleRpc0x00010A53(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_Bool* rs_data_ptr = static_cast<ResponseMessageType_Uint64_Bool*>(response_data_ptr);

    rs_data_ptr->error_code.data = modbus_manager_ptr_->getConnectStatusFromClient(rq_data_ptr->data.data, rs_data_ptr->data.data);
    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getClientConnectStatus"));
}

//"/rpc/modbus/setClientConfig"
void ControllerRpc::handleRpc0x0000D017(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_ModbusClientConfig* rq_data_ptr = static_cast<RequestMessageType_ModbusClientConfig*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    fst_hal::ModbusClientConfig client_config;
    client_config.name = rq_data_ptr->data.name;
    client_config.port = rq_data_ptr->data.port;
    client_config.ip = rq_data_ptr->data.ip;
    client_config.response_timeout_sec = rq_data_ptr->data.response_timeout / 1000;
    client_config.response_timeout_usec = (rq_data_ptr->data.response_timeout % 1000) * 1000;
    client_config.bytes_timeout_sec = rq_data_ptr->data.bytes_timeout / 1000;
    client_config.bytes_timeout_usec = (rq_data_ptr->data.bytes_timeout % 1000) * 1000;
    client_config.reg_info.coil.addr = rq_data_ptr->data.coil.address;
    client_config.reg_info.coil.max_nb = rq_data_ptr->data.coil.number;
    client_config.reg_info.discrepte_input.addr = rq_data_ptr->data.discrepte_input.address;
    client_config.reg_info.discrepte_input.max_nb = rq_data_ptr->data.discrepte_input.number;
    client_config.reg_info.holding_reg.addr = rq_data_ptr->data.holding_reg.address;
    client_config.reg_info.holding_reg.max_nb = rq_data_ptr->data.holding_reg.number;
    client_config.reg_info.input_reg.addr = rq_data_ptr->data.input_reg.address;
    client_config.reg_info.input_reg.max_nb = rq_data_ptr->data.input_reg.number;

    rs_data_ptr->data.data = modbus_manager_ptr_->setConfigToClient(rq_data_ptr->data.id, client_config);
    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/setClientConfig"));
}

//"/rpc/modbus/getClientConfig"
void ControllerRpc::handleRpc0x0000FC17(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64_ModbusClientConfig* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ModbusClientConfig*>(response_data_ptr);

    fst_hal::ModbusClientConfig client_config;
    rs_data_ptr->error_code.data = modbus_manager_ptr_->getConfigFromClient(rq_data_ptr->data.data, client_config);

    if (rs_data_ptr->error_code.data  == SUCCESS)
    {
        rs_data_ptr->data.port = client_config.port;
        strcpy(rs_data_ptr->data.ip, client_config.ip.c_str());
        rs_data_ptr->data.ip[127] = '0';
        strcpy(rs_data_ptr->data.name, client_config.name.c_str());
        rs_data_ptr->data.name[127] = '0';

        rs_data_ptr->data.response_timeout = client_config.response_timeout_sec * 1000 + 
            client_config.response_timeout_usec / 1000;
        rs_data_ptr->data.bytes_timeout = client_config.bytes_timeout_sec * 1000 +
            client_config.bytes_timeout_usec / 1000;

        rs_data_ptr->data.coil.address = client_config.reg_info.coil.addr;
        rs_data_ptr->data.coil.number = client_config.reg_info.coil.max_nb;
        rs_data_ptr->data.discrepte_input.address = client_config.reg_info.discrepte_input.addr;
        rs_data_ptr->data.discrepte_input.number = client_config.reg_info.discrepte_input.max_nb;
        rs_data_ptr->data.holding_reg.address = client_config.reg_info.holding_reg.addr;
        rs_data_ptr->data.holding_reg.number = client_config.reg_info.holding_reg.max_nb;
        rs_data_ptr->data.input_reg.address = client_config.reg_info.input_reg.addr;
        rs_data_ptr->data.input_reg.number = client_config.reg_info.input_reg.max_nb;
    }
    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getClientConfig"));
}

//"/rpc/modbus/openClient"
void ControllerRpc::handleRpc0x00000544(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = modbus_manager_ptr_->openClient(rq_data_ptr->data.data);
    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/openClient"));
}

//"/rpc/modbus/closeClient"
void ControllerRpc::handleRpc0x00006CA4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = modbus_manager_ptr_->closeClient(rq_data_ptr->data.data);
    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/closeClient"));
}

//"/rpc/modbus/deleteClient"
void ControllerRpc::handleRpc0x00014CF4(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Int32* rq_data_ptr = static_cast<RequestMessageType_Int32*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    rs_data_ptr->data.data = modbus_manager_ptr_->deleteClient(rq_data_ptr->data.data);
    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/deleteClient"));
}

//"/rpc/modbus/getClientSummaryInfoList"
void ControllerRpc::handleRpc0x0000B424(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_Void* rq_data_ptr = static_cast<RequestMessageType_Void*>(request_data_ptr);
    ResponseMessageType_Uint64_ModbusClientSummaryInfoList* rs_data_ptr = 
        static_cast<ResponseMessageType_Uint64_ModbusClientSummaryInfoList*>(response_data_ptr);

    rs_data_ptr->error_code.data = SUCCESS;
    rs_data_ptr->data.info_count = 1;

    string client_name;
    //modbus_manager_ptr_->getClientIdAndName(rs_data_ptr->data.info[0].id, client_name);
    strcpy(rs_data_ptr->data.info[0].name, client_name.c_str());

    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/getClientSummaryInfoList"));
}

//"/rpc/modbus/writeCoils"
void ControllerRpc::handleRpc0x0000BD83(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_ModbusStatusInfo* rq_data_ptr = static_cast<RequestMessageType_ModbusStatusInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (rq_data_ptr->data.number > rq_data_ptr->data.value_count)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/writeCoils"));
    }

    uint8_t value[rq_data_ptr->data.number];

    for (unsigned int i = 0; i != rq_data_ptr->data.number; ++i)
    {
        value[i] = static_cast<uint8_t>(rq_data_ptr->data.value[i]);//(uint8_t)(rq_data_ptr->data.value[i]);
    }

    rs_data_ptr->data.data = modbus_manager_ptr_->writeCoils(rq_data_ptr->data.id, rq_data_ptr->data.address,
        rq_data_ptr->data.number, value);

    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/writeCoils"));
}

//"/rpc/modbus/readCoils"
void ControllerRpc::handleRpc0x0000A433(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_ModbusStatusAddrInfo* rq_data_ptr = static_cast<RequestMessageType_ModbusStatusAddrInfo*>(request_data_ptr);
    ResponseMessageType_Uint64_ModbusStatusValueList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ModbusStatusValueList*>(response_data_ptr);

    uint8_t value[rq_data_ptr->data.number];

    rs_data_ptr->error_code.data = modbus_manager_ptr_->readCoils(rq_data_ptr->data.id, 
        rq_data_ptr->data.address, rq_data_ptr->data.number, value);

    rs_data_ptr->data.value_count = rq_data_ptr->data.number;
    for (int i = 0; i != rq_data_ptr->data.number; ++i)
    {
        rs_data_ptr->data.value[i] = static_cast<bool>(value[i]);
    }
    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/readCoils"));
}

//"/rpc/modbus/readDiscreteInputs"
void ControllerRpc::handleRpc0x0000C063(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_ModbusStatusAddrInfo* rq_data_ptr = static_cast<RequestMessageType_ModbusStatusAddrInfo*>(request_data_ptr);
    ResponseMessageType_Uint64_ModbusStatusValueList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ModbusStatusValueList*>(response_data_ptr);

    uint8_t value[rq_data_ptr->data.number];

    rs_data_ptr->error_code.data = modbus_manager_ptr_->readDiscreteInputs(rq_data_ptr->data.id, 
        rq_data_ptr->data.address, rq_data_ptr->data.number, value);

    rs_data_ptr->data.value_count = rq_data_ptr->data.number;
    for (int i = 0; i != rq_data_ptr->data.number; ++i)
    {
        rs_data_ptr->data.value[i] = static_cast<bool>(value[i]);
    }
    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/readDiscreteInputs"));
}

//"/rpc/modbus/writeHoldingRegs"
void ControllerRpc::handleRpc0x00008C43(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_ModbusRegInfo* rq_data_ptr = static_cast<RequestMessageType_ModbusRegInfo*>(request_data_ptr);
    ResponseMessageType_Uint64* rs_data_ptr = static_cast<ResponseMessageType_Uint64*>(response_data_ptr);

    if (rq_data_ptr->data.number > rq_data_ptr->data.value_count)
    {
        rs_data_ptr->data.data = CONTROLLER_INVALID_OPERATION;
        recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/writeHoldingRegs"));
    }

    uint16_t value[rq_data_ptr->data.number];

    for (unsigned int i = 0; i != rq_data_ptr->data.number; ++i)
    {
        value[i] = static_cast<uint16_t>(rq_data_ptr->data.value[i]);//(uint8_t)(rq_data_ptr->data.value[i]);
    }

    rs_data_ptr->data.data = modbus_manager_ptr_->writeHoldingRegs(rq_data_ptr->data.id, rq_data_ptr->data.address, 
        rq_data_ptr->data.number, value);

    recordLog(MODBUS_LOG, rs_data_ptr->data.data, std::string("/rpc/modbus/writeHoldingRegs"));
}

//"/rpc/modbus/readHoldingRegs"
void ControllerRpc::handleRpc0x00003583(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_ModbusRegAddrInfo* rq_data_ptr = static_cast<RequestMessageType_ModbusRegAddrInfo*>(request_data_ptr);
    ResponseMessageType_Uint64_ModbusRegValueList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ModbusRegValueList*>(response_data_ptr);

    uint16_t value[rq_data_ptr->data.number];

    rs_data_ptr->error_code.data = modbus_manager_ptr_->readHoldingRegs(rq_data_ptr->data.id, 
        rq_data_ptr->data.address, rq_data_ptr->data.number, value);

    rs_data_ptr->data.value_count = rq_data_ptr->data.number;
    for (int i = 0; i != rq_data_ptr->data.number; ++i)
    {
        rs_data_ptr->data.value[i] = static_cast<uint32_t>(value[i]);
    }

    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/readHoldingRegs"));
}

//"/rpc/modbus/readInputRegs"
void ControllerRpc::handleRpc0x000072C3(void* request_data_ptr, void* response_data_ptr)
{
    RequestMessageType_ModbusRegAddrInfo* rq_data_ptr = static_cast<RequestMessageType_ModbusRegAddrInfo*>(request_data_ptr);
    ResponseMessageType_Uint64_ModbusRegValueList* rs_data_ptr = static_cast<ResponseMessageType_Uint64_ModbusRegValueList*>(response_data_ptr);

    uint16_t value[rq_data_ptr->data.number];

    rs_data_ptr->error_code.data = modbus_manager_ptr_->readInputRegs(rq_data_ptr->data.id, 
        rq_data_ptr->data.address, rq_data_ptr->data.number, value);

    rs_data_ptr->data.value_count = rq_data_ptr->data.number;
    for (int i = 0; i != rq_data_ptr->data.number; ++i)
    {
        rs_data_ptr->data.value[i] = static_cast<uint32_t>(value[i]);
    }

    recordLog(MODBUS_LOG, rs_data_ptr->error_code.data, std::string("/rpc/modbus/readInputRegs"));
}

