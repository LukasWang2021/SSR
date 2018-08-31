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

#include "modbus-private.h"
#include "modbus-rtu.h"

#include "modbus_manager_param.h"
#include "local_ip.h"

using namespace std;

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
}ModbusRegisters; //for holding regs and input regs

class ModbusTCPClient
{
public:
    ModbusTCPClient(int port);
     ~ModbusTCPClient();

    /***************************************
    Function : set debug mode or not
    return true : if operation succeed 
    return false : if operation failed
    ***************************************/
    bool setDebug(bool flag);

    /***************************************
    Function : set error recovery mode, 
        mode :  MODBUS_ERROR_RECOVERY_NONE = 0,
                MODBUS_ERROR_RECOVERY_LINK = 1,
                MODBUS_ERROR_RECOVERY_PROTOCOL = 2 
    return true : if operation succeed 
    return false : if operation failed
    ***************************************/
    bool setErrorRecovery(int error_recovery);

    /***************************************
    Function : bulk read coils, 
    return true : if operation succeed 
    return false : if operation failed
    ***************************************/
    bool readCoils(ModbusStatus& status);

    /***************************************
    Function : bulk read discrete inputs, 
    return true : if operation succeed 
    return false : if operation failed
    ***************************************/
    bool readDiscreteInputs(ModbusStatus& status); //input bits

    /***************************************
    Function : bulk read Input Registers, 
    return true : if operation succeed 
    return false : if operation failed
    ***************************************/
    bool readInputRegs(ModbusRegisters& regs);

    /***************************************
    Function : bulk read Holding Registers, 
    return true : if operation succeed 
    return false : if operation failed
    ***************************************/
    bool readHoldingRegs(ModbusRegisters& regs);

    /***************************************
    Function : write singal coil, 
    return true : if operation succeed 
    return false : if operation failed
    ***************************************/
    bool writeSingleCoil(int coil_addr, uint8_t status); //bit

    /***************************************
    Function : bulk write coils,
    return true : if operation succeed
    return false : if operation failed
    ***************************************/
    bool writeCoils(ModbusStatus& status);

    /***************************************
    Function : write singal holding register, 
    return true : if operation succeed 
    return false : if operation failed
    ***************************************/
    bool writeSingleHoldingReg(int reg_addr, uint16_t value);

    /***************************************
    Function : bulk write holding registers,
    return true : if operation succeed
    return false : if operation failed
    ***************************************/
    bool writeHoldingRegs(ModbusRegisters& regs);

    /***************************************
    Function : write and read singal coil,
    return true : if operation succeed
    return false : if operation failed
    ***************************************/
    bool writeAndReadSingleCoil(int addr, uint8_t& write_status, uint8_t& read_status);

    /***************************************
    Function : write and read singal holding registers,
    return true : if operation succeed
    return false : if operation failed
    ***************************************/
    bool writeAndReadSingleHoldingReg(int addr, uint16_t& write_reg, uint16_t& read_reg);

    /***************************************
    Function :bulk  write and read coils,
    return true : if operation succeed
    return false : if operation failed
    ***************************************/
    bool writeAndReadCoils(ModbusStatus& write_status, ModbusStatus& read_status);

    /***************************************
    Function :bulk  write and read holding registers,
    return true : if operation succeed
    return false : if operation failed
    ***************************************/
    bool writeAndReadHoldingRegs(ModbusRegisters& write_regs, ModbusRegisters& read_regs);

    /***************************************
    Function : store the timeout interval used to wait 
        for a response in the timeout argument.
    return true : if operation succeed
    return false : if operation failed
    ***************************************/
    bool getResponseTimeout(timeval& timeout);

    /***************************************
    Function : set the timeout interval used to wait for a response
    return true : if operation succeed
    return false : if operation failed
    ***************************************/
    bool setResponseTimeout(timeval& timeout);

    /***************************************
    Function : store the timeout interval between two consecutive bytes
        of the same message in the timeout argument.
    return true : if operation succeed
    return false : if operation failed
    ***************************************/
    bool getByteTimeout(timeval& timeout);

    /***************************************
    Function : set the timeout interval between two consecutive bytes 
        of the same message
    return true : if operation succeed
    return false : if operation failed
    ***************************************/    
    bool setByteTimeout(timeval& timeout);

    /***************************************
    Function : init ModbusTCPClient obj,
    return true : if operation succeed
    return false : if operation failed
    ***************************************/
    bool init();

    /***************************************
    Function : set socket for modbus,
    return true : if operation succeed
    return false : if operation failed
    ***************************************/
    bool setSocket(int socket);

    /***************************************
    Function : set socket for modbus,
    return souket value : if operation succeed
    return -1 : if operation failed
    ***************************************/
    int getSocket();

    /***************************************
    Function :get number of failed operation,
    return number
    ***************************************/
    int getFailedOperationNumber();

    int log_level_;
private:
    enum {SOCKET_ID_MIN = 1, SOCKET_ID_MAX = 247, };
    modbus_t* ctx_;
    int nb_fail_;

    LocalIP local_ip_;
    ModbusManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
};
}

#endif
