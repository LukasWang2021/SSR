#ifndef MODBUS_MANAGER
#define MODBUS_MANAGER

#include <string>
#include <mutex>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include "net_connection.h"
#include "common_log.h"
#include "base_device.h"
#include "parameter_manager/parameter_manager_param_group.h"

#include "tcp_client.h"
#include "tcp_server.h"

#include "modbus_manager_param.h"
#include "modbus_client_param.h"
#include "modbus_server_param.h"

using namespace std;
using namespace fst_hal;

namespace fst_hal
{
enum ModbusStartMode{CLIENT = 1, SERVER = 2, };
class ModbusManager: public BaseDevice
{
public:
    ModbusManager(int address);
    ~ModbusManager();

    bool init();
    bool isValid();

    ErrorCode setStartMode(int start_mode); // enum ModbusStartMode
    int getStartMode();

    ErrorCode setScanRate(int scan_rate);
    int getScanRate();

    ErrorCode writeCoils(int id, int addr, int nb, uint8_t *dest);
    ErrorCode readCoils(int id, int addr, int nb, uint8_t *dest);
    ErrorCode readDiscreteInputs(int id, int addr, int nb, uint8_t *dest);
    ErrorCode writeHoldingRegs(int id, int addr, int nb, uint16_t *dest);
    ErrorCode readHoldingRegs(int id, int addr, int nb, uint16_t *dest);
    ErrorCode readInputRegs(int id, int addr, int nb, uint16_t *dest);

    // for server
    ErrorCode openServer();
    ErrorCode closeServer();

    ErrorCode setConnectStatusToServer(bool &status);
    ErrorCode getConnectStatusFromServer(bool &status);
    ErrorCode setConfigToServer(ModbusServerConfig &config);
    ErrorCode getConfigFromServer(ModbusServerConfig &config);

    ErrorCode getCoilInfoFromServer(ModbusRegAddrInfo &info); 
    ErrorCode getDiscrepteInputInfoFromServer(ModbusRegAddrInfo &info);
    ErrorCode getHoldingRegInfoFromServer(ModbusRegAddrInfo &info);
    ErrorCode getInputRegInfoFromServer(ModbusRegAddrInfo &info);
    ErrorCode getServerRegInfoFromServer(ModbusServerRegInfo &info);
    ErrorCode getStartInfoFromServer(ModbusServerStartInfo &info);

    // for client
    ErrorCode addClient(int client_id);
    ErrorCode deleteClient(int client_id);
    ErrorCode openClient(int client_id);
    ErrorCode closeClient(int client_id); // to modify
    void getClientIdAndName(int id, string name);

    ErrorCode setConnectStatusToClient(int client_id, bool status);
    ErrorCode getConnectStatusFromClient(int client_id, bool status);

    ErrorCode setConfigToClient(int client_id, ModbusClientConfig config);
    ErrorCode getConfigFromClient(int client_id, ModbusClientConfig config); // to modify

private:
    ModbusManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    NetConnection net_connect_;

    string server_ip_;
    int server_port_;

    int client_scan_rate_;
    fst_base::ThreadHelp thread_ptr_;

    ModbusTCPClient* client_;
    ModbusTCPServer* server_;

    int start_mode_;
    int address_;

    bool isClientRunning(int client_id);
    bool isServerRunning();

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
};
}

#endif

//void modbusTcpAllClientRoutineThreadFunc(void* arg);
