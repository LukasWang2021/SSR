#ifndef _MODBUS_TCP_CLIENT_HPP
#define _MODBUS_TCP_CLIENT_HPP

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>

#include "parameter_manager/parameter_manager_param_group.h"
#include "common_log.h"

#include "modbus/modbus-private.h"
#include "modbus/modbus-tcp.h"

#include "modbus_client_param.h"

using namespace std;
using namespace fst_hal;

namespace fst_hal
{
class ModbusTCPClient
{
public:
    ModbusTCPClient(string file_path);
     ~ModbusTCPClient();

    ErrorCode setResponseTimeout(timeval timeout);
    ErrorCode setBytesTimeout(timeval timeout);
    ErrorCode getResponseTimeout(timeval& timeout);
    ErrorCode getBytesTimeout(timeval& timeout);

    ErrorCode setIp(string ip);
    ErrorCode setPort(int port);
    string getIp();
    int getPort();

    ErrorCode init();
    ErrorCode initParam();

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

    modbus_t* ctx_;
    bool is_debug_;
    int socket_;
    int port_;
    string ip_;
    string comm_type_;
    timeval response_timeout_;
    timeval bytes_timeout_;

    ModbusClientParam* param_ptr_;
    fst_log::Logger* log_ptr_;
};
}

#endif
