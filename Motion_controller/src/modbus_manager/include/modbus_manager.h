#ifndef _MODBUS_MANAGER_HPP
#define _MODBUS_MANAGER_HPP

#include <string>
#include <mutex>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include "common_log.h"
#include "base_device.h"
#include "parameter_manager/parameter_manager_param_group.h"

#include "modbus_manager_param.h"
#include "modbus_client_manager.h"
#include "modbus_server.h"

using namespace std;
using namespace fst_hal;

namespace fst_hal
{
enum ModbusStartMode{MODBUS_START_MODE_INVALID = 0, MODBUS_CLIENT = 1, MODBUS_SERVER = 2, };
class ModbusManager: public BaseDevice
{
public:
    ModbusManager(int address);
    ~ModbusManager();

    bool init();
    bool isModbusValid();

    ErrorCode setStartMode(int start_mode); // enum ModbusStartMode
    int getStartMode();

    ErrorCode writeCoils(int id, int addr, int nb, uint8_t *dest);
    ErrorCode readCoils(int id, int addr, int nb, uint8_t *dest);
    ErrorCode readDiscreteInputs(int id, int addr, int nb, uint8_t *dest);
    ErrorCode writeHoldingRegs(int id, int addr, int nb, uint16_t *dest);
    ErrorCode readHoldingRegs(int id, int addr, int nb, uint16_t *dest);
    ErrorCode readInputRegs(int id, int addr, int nb, uint16_t *dest);

    // for server
    ErrorCode openServer();
    ErrorCode closeServer();
    bool isServerRunning();

    ErrorCode setServerEnableStatus(bool &status);
    ErrorCode getServerEnableStatus(bool &status);
    ErrorCode setServerStartInfo(ModbusServerStartInfo &start_info);
    ErrorCode getServerStartInfo(ModbusServerStartInfo &start_info);
    ErrorCode setServerRegInfo(ModbusServerRegInfo &config_reg_info);
    ErrorCode getServerRegInfo(ModbusServerRegInfo &config_reg_info);
    ErrorCode getServerConfigParams(ModbusServerConfigParams &params);

    ErrorCode getServerConfigCoilInfo(ModbusRegAddrInfo &info); 
    ErrorCode getServerConfigDiscrepteInputInfo(ModbusRegAddrInfo &info);
    ErrorCode getServerConfigHoldingRegInfo(ModbusRegAddrInfo &info);
    ErrorCode getServerConfigInputRegInfo(ModbusRegAddrInfo &info);

    // for client
    ErrorCode addClient(ModbusClientStartInfo &start_info);
    ErrorCode deleteClient(int client_id);
    ErrorCode replaceClient(int &replaced_id, ModbusClientStartInfo &start_info);
    ErrorCode getClientIdList(vector<int> &id_list);

    ErrorCode getClientCtrlState(int client_id, int &ctrl_state);
    ErrorCode connectClient(int client_id);
    ErrorCode closeClient(int client_id);
    ErrorCode isConnected(int client_id, bool &is_connected);
    ErrorCode scanClientDataArea(int &client_id);
    ErrorCode scanAllClientDataArea();
    ErrorCode getConnectedClientIdList(vector<int> &id_list);

    ErrorCode setClientEnableStatus(int client_id, bool &status);
    ErrorCode setClientRegInfo(int client_id, ModbusClientRegInfo &reg_info);
    ErrorCode updateClientStartInfo(ModbusClientStartInfo &start_info);

    ErrorCode getClientStartInfo(int client_id, ModbusClientStartInfo &start_info);
    ErrorCode getClientEnableStatus(int client_id, bool &status);
    ErrorCode getClientRegInfo(int client_id, ModbusClientRegInfo &reg_info);
    ErrorCode getClientConfigParams(int client_id, ModbusClientConfigParams &client_config_params);
    ErrorCode getClientConfigParamsList(vector<ModbusClientConfigParams> &client_config_params_list);

    ErrorCode getClientScanRate(int client_id, int &scan_rate);

private:
    ModbusManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;

    ModbusClientManager* client_manager_ptr_;
    ModbusServer* server_;

    int start_mode_;
    int address_;

    ErrorCode writeCoilsToServer(int addr, int nb, uint8_t *dest);
    ErrorCode readCoilsFromServer(int addr, int nb, uint8_t *dest);
    ErrorCode readDiscreteInputsFromServer(int addr, int nb, uint8_t *dest);
    ErrorCode writeHoldingRegsToServer(int addr, int nb, uint16_t *dest);
    ErrorCode readHoldingRegsFromServer(int addr, int nb, uint16_t *dest);
    ErrorCode readInputRegsFromServer(int addr, int nb, uint16_t *dest);

    ErrorCode writeCoilsByClient(int client_id, int addr, int nb, uint8_t *dest);
    ErrorCode readCoilsByClient(int client_id, int addr, int nb, uint8_t *dest);
    ErrorCode readDiscreteInputsByClient(int client_id, int addr, int nb, uint8_t *dest);
    ErrorCode writeHoldingRegsByClient(int client_id, int addr, int nb, uint16_t *dest);
    ErrorCode readHoldingRegsByClient(int client_id, int addr, int nb, uint16_t *dest);
    ErrorCode writeAndReadHoldingRegsByClient(int client_id, int write_addr, int write_nb, const uint16_t *write_dest,
        int read_addr, int read_nb, uint16_t *read_dest);
    ErrorCode readInputRegsByClient(int client_id, int addr, int nb, uint16_t *dest);

    ModbusManager();
};
}

#endif


