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

    unsigned int hash_value = 0x0000EC64;

    RequestMessageType_Int32_DoubleList send_msg;
    send_msg.header.time_stamp = 122;
    send_msg.property.authority = Comm_Authority_TP;
    send_msg.data1.data = 1;
    send_msg.data2.data_count = 9;
    send_msg.data2.data[0] = 1.1;
    send_msg.data2.data[1] = 1.2;
    send_msg.data2.data[2] = 1.3;
    send_msg.data2.data[3] = 1.4;
    send_msg.data2.data[4] = 1.5;
    send_msg.data2.data[5] = 1.6;
    send_msg.data2.data[6] = 1.7;
    send_msg.data2.data[7] = 1.8;
    send_msg.data2.data[8] = 1.9;

    if (!test.generateRequestMessageType(hash_value, (void*)&send_msg, RequestMessageType_Int32_DoubleList_fields, buf, buf_size))
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

    ResponseMessageType_Uint64_Int32List recv_msg;
    unsigned int recv_hash = 0;
    if (!test.decodeResponseMessageType(recv_hash, (void*)&recv_msg, ResponseMessageType_Uint64_Int32List_fields, buf, buf_size))
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
    cout << "Reply : recv_msg.error_code = " << recv_msg.error_code.data << endl;
    cout << "Reply : recv_msg.data.data_count = " << recv_msg.data.data_count << endl;

    for (int i = 0; i != recv_msg.data.data_count; ++i)
    {
        cout << "Reply : recv_msg.data.data[i]" << recv_msg.data.data[i] << endl;
    }

    usleep(200000);

    return 0;
}
