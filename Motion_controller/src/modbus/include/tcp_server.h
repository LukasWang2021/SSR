#ifndef _MODBUS_TCP_SERVER_HPP
#define _MODBUS_TCP_SERVER_HPP

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>

#include "modbus/modbus-private.h"
#include "modbus/modbus-rtu.h"
#include "modbus_manager_param.h"
#include "local_ip.h"

#include "parameter_manager/parameter_manager_param_group.h"
#include "common_log.h"

using namespace std;

namespace fst_modbus
{
class ModbusTCPServer
{
public:
    ModbusTCPServer(int port);
     ~ModbusTCPServer();

    /***************************************
    Function : init ModbusTCPServer obj,
    return true : if operation succeed
    return false : if operation failed
    ***************************************/
    bool init(int nb_connection);

    /***************************************
    Function : set debug mode or not
    return true : if operation succeed 
    return false : if operation failed
    ***************************************/
    bool setDebug(bool flag);

    /***************************************
    Function : allocate four arrays to store 
        coils, discrete inputs, holding registers and inputs registers
    return true : if operation succeed 
    return false : if operation failed
    ***************************************/
    bool mapping_new_start_address(
        unsigned int start_colis, unsigned int nb_colis,
        unsigned int start_discrete_inputs, unsigned int nb_discrete_inputs,
        unsigned int start_holding_registers, unsigned int nb_holding_registers,
        unsigned int start_input_registers, unsigned int nb_input_registers);

    /***************************************
    Function : allocate four arrays to store 
        coils, discrete inputs, holding registers and inputs registers
    return true : if operation succeed 
    return false : if operation failed
    ***************************************/
    bool mapping_new(unsigned int nb_colis, unsigned int nb_discrete_inputs,
        unsigned int nb_holding_registers, unsigned int nb_input_registers);

    /***************************************
    Function : free the four arrays
    return void 
    ***************************************/
    void mapping_free();

    /***************************************
    Function :  receive an indication request from the socket 
    return size of request: if operation succeed 
    return -1 : if operation failed
    ***************************************/
    int receive(uint8_t *req);

    /***************************************
    Function :  send an response
    return size of reply: if operation succeed 
    return -1 : if operation failed
    ***************************************/
    int reply(const uint8_t *req, int req_length);

    /***************************************
    Function :  listen to sockets, 
        nb_connection is number of socket to listen, 
        generally, nb_connection = 1
    return size of reply: if operation succeed 
    return -1 : if operation failed
    ***************************************/
    int listen(int nb_connection);

    /***************************************
    Function :  accept frames from a client
    return socket value: if operation succeed 
    return -1 : if operation failed
    ***************************************/
    int accept();

    /***************************************
    Function :  close modbus server socket
    return void
    ***************************************/
    void close();

    int log_level_;
private:
    modbus_t* ctx_;
    modbus_mapping_t* mb_mapping_;
    int server_socket_;

    LocalIP local_ip_;
    ModbusManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
};
}

#endif
