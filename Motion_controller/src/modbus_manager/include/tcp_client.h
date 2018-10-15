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

#include "modbus_manager_param.h"

using namespace std;
using namespace fst_modbus;

namespace fst_modbus
{
typedef struct
{
    int addr;  //start address 
    int nb; //number of address
    uint8_t *dest; //store data
}ModbusStatus; // for coil and discrete input

typedef struct
{
    int addr; //start address
    int nb; //number of address
    uint16_t *dest; //store data
}ModbusRegs; //for holding regs and input regs

class ModbusTCPClient
{
public:
    ModbusTCPClient(string ip, int port);
     ~ModbusTCPClient();

    void setDebug(bool flag);
    void setSocket(int s);
    void setResponseTimeout(timeval& timeout);
    void setBytesTimeout(timeval& timeout);

    ErrorCode init();
    int getSocket();
    ErrorCode getResponseTimeout(timeval& timeout);
    ErrorCode getByteTimeout(timeval& timeout);

    ErrorCode readCoils(ModbusStatus& status);
    ErrorCode readDiscreteInputs(ModbusStatus& status); //input bits
    ErrorCode readInputRegs(ModbusRegs& regs);
    ErrorCode readHoldingRegs(ModbusRegs& regs);
    ErrorCode writeSingleCoil(int coil_addr, uint8_t status); //bit
    ErrorCode writeSingleHoldingReg(int reg_addr, uint16_t value);
    ErrorCode writeAndReadSingleCoil(int addr, uint8_t& write_status, uint8_t& read_status);
    ErrorCode writeAndReadSingleHoldingReg(int addr, uint16_t& write_reg, uint16_t& read_reg);
    ErrorCode writeCoils(ModbusStatus& status);
    ErrorCode writeHoldingRegs(ModbusRegs& regs);
    ErrorCode writeAndReadCoils(ModbusStatus& write_status, ModbusStatus& read_status);
    ErrorCode writeAndReadHoldingRegs(ModbusRegs& write_regs, ModbusRegs& read_regs);

private:
    enum {SOCKET_ID_MIN = 1, SOCKET_ID_MAX = 247, };
    enum RegsOneOpNum{ STATUS_ONE_OP_NUM = 1968, REGISTER_ONE_OP_NUM = 121, };
    modbus_t* ctx_;

    bool is_debug_;
    int socket_;
    int port_;
    string ip_;
    timeval response_timeout_;
    timeval bytes_timeout_;

    ModbusManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    fst_parameter::ParamGroup tcp_client_yaml_help_;
    std::string tcp_client_file_path_;

    bool loadComponentParams();
    bool saveComponentParams();
};
}

#endif
