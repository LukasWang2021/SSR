#ifndef MODBUS_MANAGER
#define MODBUS_MANAGER

#include <string>
#include <mutex>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

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
    ErrorCode openModbus(int start_mode);
    ErrorCode closeModbus();
    int getStartMode();

    // for server
    ErrorCode getServerInfo(ServerInfo& info);
    ErrorCode getServerCoilInfo(ModbusRegAddrInfo& info);
    ErrorCode getServerDiscrepteInputInfo(ModbusRegAddrInfo& info);
    ErrorCode getServerHoldingRegInfo(ModbusRegAddrInfo& info);
    ErrorCode getServerInputRegInfo(ModbusRegAddrInfo& info);
    ErrorCode getServerIp(string& ip);
    ErrorCode getServerPort(int& port);

    // for client
    ErrorCode setClientIp(string ip);
    ErrorCode setClientPort(int port);
    ErrorCode setResponseTimeout(timeval timeout);
    ErrorCode setBytesTimeout(timeval timeout);
    ErrorCode setClientInfo(ClientInfo info);

    ErrorCode getResponseTimeout(timeval& timeout);
    ErrorCode getBytesTimeout(timeval& timeout);

    ErrorCode writeCoils(int addr, int nb, uint8_t *dest);
    ErrorCode readCoils(int addr, int nb, uint8_t *dest);

    ErrorCode readDiscreteInputs(int addr, int nb, uint8_t *dest);

    ErrorCode writeHoldingRegs(int addr, int nb, uint16_t *dest);
    ErrorCode readHoldingRegs(int addr, int nb, uint16_t *dest);
    ErrorCode writeAndReadHoldingRegs(int write_addr, int write_nb, const uint16_t *write_dest,
        int read_addr, int read_nb, uint16_t *read_dest);

    ErrorCode readInputRegs(int addr, int nb, uint16_t *dest);

private:
    ModbusManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    static ModbusManager* instance_;

    string server_ip_;
    int server_port_;

    ModbusTCPClient* client_;
    ModbusTCPServer* server_;

    int start_mode_;
    int address_;

    bool is_valid_;

    bool isServerRunning();

};
}

#endif


