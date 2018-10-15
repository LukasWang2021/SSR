#ifndef MODBUS_MANAGER
#define MODBUS_MANAGER

#include <string>
#include <mutex>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include "common_log.h"
#include "parameter_manager/parameter_manager_param_group.h"

#include "tcp_client.h"
#include "tcp_server.h"
#include "modbus_map.h"
#include "modbus_manager_param.h"

using namespace std;
using namespace fst_modbus;

namespace fst_modbus
{
class ModbusManager
{
public:
    ModbusManager();
    ~ModbusManager();

    ErrorCode init();
    static ModbusManager* getInstance();
    ErrorCode initClient();
    ErrorCode initServer();
    void initMap();
    
    void setServerIP(string ip) { server_ip_ = ip; }
    void setServerPort(int port) { server_port_ = port; }
    string getServerIP() { return server_ip_; }
    int getServerPort() { return server_port_; }

    ErrorCode setValueById(int id, void* value);
    ErrorCode getValueById(int id, void* value);

    ModbusTCPClient* client_;
    ModbusTCPServer* server_;
    ModbusMap* map_;

private:
    ModbusManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    static ModbusManager* instance_;
    // component params
    string server_ip_;
    int server_port_;
};
}

#endif


