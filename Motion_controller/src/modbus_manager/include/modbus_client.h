#ifndef _MODBUS_CLIENT_HPP
#define _MODBUS_CLIENT_HPP

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>

#include "common_log.h"
#include "thread_help.h"

#include "modbus/modbus-private.h"
#include "modbus/modbus-tcp.h"

#include "modbus_client_config_param.h"

using namespace std;
using namespace fst_hal;

namespace fst_hal
{
typedef enum
{
    MODBUS_CLIENT_CTRL_DISABLED       = 0,
    MODBUS_CLIENT_CTRL_ENABLED    = 1,
    MODBUS_CLIENT_CTRL_CONNECTED  = 2,
    MODBUS_CLIENT_CTRL_OPERATIONAL = 3,
}ModbusClientCtrlState;

class ModbusClient
{
public:
    ModbusClient(int id, bool is_debug, int log_level);
     ~ModbusClient();

    ErrorCode init();
    ErrorCode open();
    void close();
    bool isRunning();

    ErrorCode setEnableStatus(bool status);
    bool getEnableStatus();

    ErrorCode setStartInfo(ModbusClientStartInfo start_info);
    ModbusClientStartInfo getStartInfo();

    ErrorCode setRegInfo(ModbusClientRegInfo reg_info);
    ModbusClientRegInfo getRegInfo();

    ModbusClientConfigParams getConfigParams();

    int getScanRate();
    int getId();
    int getCtrlState();

    ErrorCode writeCoils(int addr, int nb, uint8_t *dest);
    ErrorCode readCoils(int addr, int nb, uint8_t *dest);
    ErrorCode readDiscreteInputs(int addr, int nb, uint8_t *dest);
    ErrorCode writeHoldingRegs(int addr, int nb, uint16_t *dest);
    ErrorCode readHoldingRegs(int addr, int nb, uint16_t *dest);
    ErrorCode writeAndReadHoldingRegs(int write_addr, int write_nb, const uint16_t *write_dest,
        int read_addr, int read_nb, uint16_t *read_dest);
    ErrorCode readInputRegs(int addr, int nb, uint16_t *dest);

private:
    enum {SOCKET_ID_MIN = 1, SOCKET_ID_MAX = 247, };
    enum RegsOneOpNum{ STATUS_ONE_OP_NUM = 1024, REGISTER_ONE_OP_NUM = 121, };

    int log_level_;
    bool is_debug_;

    bool is_config_param_valid_;
    int ctrl_state_;
    bool is_running_;

    modbus_t* ctx_;
    int socket_;

    ModbusClientConfigParams config_param_;

    fst_log::Logger* log_ptr_;
    fst_base::ThreadHelp thread_help_;

    bool checkConfigParamValid();
    ModbusClient();
};
}

#endif
