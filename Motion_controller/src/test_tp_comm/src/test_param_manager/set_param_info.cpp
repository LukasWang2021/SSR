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
    if (argc < needed_data_count + 1)
    {
        cout << "more parameters are needed" << endl;
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

    unsigned int hash_value = 0x0001393F;

    RequestMessageType_Int32_ParamInfo msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP;

    msg.data1.data = atoi(argv[1]);
    strcpy(msg.data2.name, argv[2]);
    msg.data2.type = atoi(argv[3]);
 
    if(msg.data2.type == 0)
    {
        msg.data2.data.size = sizeof(int);
        int data = atoi(argv[4]);
        memcpy(&msg.data2.data.bytes, &data, msg.data2.data.size);
    }
    else if (msg.data2.type == 1)
    {
        msg.data2.data.size = sizeof(double);
        int data = atof(argv[4]);
        memcpy(&msg.data2.data.bytes, &data, msg.data2.data.size);
    }
    else if (msg.data2.type == 2)
    {
        msg.data2.data.size = sizeof(bool);
        int data = atoi(argv[4]);
        memcpy(&msg.data2.data.bytes, &data, msg.data2.data.size);
    }
    else
    {
        msg.data2.data.size = sizeof(int);
        int data = atoi(argv[4]);
        memcpy(&msg.data2.data.bytes, &data, msg.data2.data.size);
    }
   //strcpy(msg.data2.data.bytes, argv[4]);

    cout << "param : data1 = " << msg.data1.data << endl;
    cout << "param : data2.name = " << msg.data2.name << endl;
    cout << "param : data2.type = " << msg.data2.type << endl;
    cout << "param : data2.data.bytes = " << msg.data2.data.bytes << endl;
    cout << "param : data2.data.size = " << msg.data2.data.size << endl;

    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_Int32_ParamInfo_fields, buf, buf_size))
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

    cout << "Reply : msg.header.time_stamp = " << recv_msg.header.time_stamp << endl;
    cout << "Reply : msg.header.package_left = " << recv_msg.header.package_left << endl;
    cout << "Reply : msg.header.error_code = " << recv_msg.header.error_code << endl;
    cout << "Reply : msg.property.authority = " << recv_msg.property.authority << endl;
    cout << "Reply : msg.data.data = " << recv_msg.data.data << endl;

    usleep(200000);

    return 0;
}