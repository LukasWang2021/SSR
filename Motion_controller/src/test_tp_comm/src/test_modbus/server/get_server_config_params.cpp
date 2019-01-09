#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include <nanomsg/nn.h>
#include <nanomsg/reqrep.h>
#include <nanomsg/ws.h>

#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>
#include "protoc.h"
#include <time.h>

#include "tp_comm_test.h"


#define MAX_REQ_BUFFER_SIZE     (65535)

using namespace std;

int main()
{
    TpCommTest test;
    if (!test.initRpcSocket())
    {
        cout << "Request : socket init failed" << endl;
        return -1;
    }

    uint8_t buf[MAX_REQ_BUFFER_SIZE];
    int buf_size = MAX_REQ_BUFFER_SIZE;

    unsigned int hash_value = 0x0000E2E3;

    RequestMessageType_Void msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP;

    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_Void_fields, buf, buf_size))
    {
        cout << "Request : encode buf failed" << endl;
        return -1;
    }

    if (!test.sendRequestBuf(buf, buf_size))
    {
        cout << "Request : send buf failed" << endl;
        return -1;
    }

    buf_size = MAX_REQ_BUFFER_SIZE;
    if (!test.recvResponseBuf(buf, buf_size))
    {
        cout << "Reply : recv buf failed, buf size = " << buf_size << endl;
        return -1;
    }

    ResponseMessageType_Uint64_ModbusServerConfigParams recv_msg;
    unsigned int recv_hash = 0;
    if (!test.decodeResponseMessageType(recv_hash, (void*)&recv_msg, ResponseMessageType_Uint64_ModbusServerConfigParams_fields, buf, buf_size))
    {
        cout << "Reply : recv msg decode failed" << endl;
        return -1;
    }
    if (!test.checkHash(recv_hash, hash_value))
    {
        cout << "Reply : hash error ,hash = " << recv_hash << endl;
        return -1;
    }

    cout << "Reply : msg.header.time_stamp = " << recv_msg.header.time_stamp << endl;
    cout << "Reply : msg.header.package_left = " << recv_msg.header.package_left << endl;
    cout << "Reply : msg.header.error_code = " << recv_msg.header.error_code << endl;
    cout << "Reply : msg.property.authority = " << recv_msg.property.authority << endl;
    cout << "Reply : msg.error_code.data = " << std::hex << recv_msg.error_code.data << endl;
    printf("Reply : msg.error_code.data = %x\n", recv_msg.error_code.data);

    cout << "Reply : msg.data.is_enable = " << recv_msg.data.is_enable<< endl;

    cout << "Reply : msg.data.start_info.has_ip = " << recv_msg.data.start_info.has_ip << endl;
    cout << "Reply : msg.data.start_info.ip = " << recv_msg.data.start_info.ip << endl;
    cout << "Reply : msg.data.start_info.name = " << recv_msg.data.start_info.name << endl;
    cout << "Reply : msg.data.start_info.response_delay = " << recv_msg.data.start_info.response_delay << endl;

    cout << "Reply : msg.data.function_addr_info.coil.address = " << recv_msg.data.function_addr_info.coil.address << endl;
    cout << "Reply : msg.data.function_addr_info.coil.number = " <<  recv_msg.data.function_addr_info.coil.number << endl;
    cout << "Reply : msg.data.function_addr_info.discrepte_input.address = " << recv_msg.data.function_addr_info.discrepte_input.address << endl;
    cout << "Reply : msg.data.function_addr_info.discrepte_input.number = " << recv_msg.data.function_addr_info.discrepte_input.number << endl;
    cout << "Reply : msg.data.function_addr_info.holding_reg.address = " << recv_msg.data.function_addr_info.holding_reg.address << endl;
    cout << "Reply : msg.data.function_addr_info.holding_reg.number = " << recv_msg.data.function_addr_info.holding_reg.number << endl;
    cout << "Reply : msg.data.function_addr_info.input_reg.address = " << recv_msg.data.function_addr_info.input_reg.address << endl;
    cout << "Reply : msg.data.function_addr_info.input_reg.number = " << recv_msg.data.function_addr_info.input_reg.number << endl;

    usleep(200000);

    return 0;
}
