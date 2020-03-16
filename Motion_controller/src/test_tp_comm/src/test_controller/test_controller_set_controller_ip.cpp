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

int main(int argc, char* argv[])
{
    int needed_data_count = 4;
    if (argc != 5 && argc != 9 && argc != 13)
    {
        cout << "IP, network(opt), gateway(opt) are needed" << endl;
        return -1;
    }

    TpCommTest test;
    if (!test.initRpcSocket())
    {
        cout << "Request : socket init failed" << endl;
        return -1;
    }

    uint8_t buf[MAX_REQ_BUFFER_SIZE];
    int buf_size = MAX_REQ_BUFFER_SIZE;

    unsigned int hash_value = 0x0000B090;

    RequestMessageType_Int32List msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP;
    msg.data.data_count = argc - 1;

    for (size_t i = 0; i !=  msg.data.data_count; ++i)
    {
        msg.data.data[i] = atol(argv[1+i]);
    }

    for (size_t i = 0; i !=  msg.data.data_count; ++i)
    {
        cout << "rec : msg. msg.data.data[" << i << "] = " << msg.data.data[i] << endl;
    }

    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_Int32List_fields, buf, buf_size))
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

    ResponseMessageType_Uint64 recv_msg;
    unsigned int recv_hash = 0;
    if (!test.decodeResponseMessageType(recv_hash, (void*)&recv_msg, ResponseMessageType_Uint64_fields, buf, buf_size))
    {
        cout << "Reply : recv msg decode failed" << endl;
        return -1;
    }
    if (!test.checkHash(recv_hash, hash_value))
    {
        cout << "Reply : hash error ,hash = " << recv_hash << endl;
        return -1;
    }

    cout << "Reply : recv_msg.header.time_stamp = " << recv_msg.header.time_stamp << endl;
    cout << "Reply : recv_msg.header.package_left = " << recv_msg.header.package_left << endl;
    cout << "Reply : recv_msg.header.error_code = " << recv_msg.header.error_code << endl;
    cout << "Reply : recv_msg.property.authority = " << recv_msg.property.authority << endl;
    cout << "Reply : recv_msg.data.data = " <<recv_msg.data.data << endl;

    usleep(200000);

    return 0;
}
