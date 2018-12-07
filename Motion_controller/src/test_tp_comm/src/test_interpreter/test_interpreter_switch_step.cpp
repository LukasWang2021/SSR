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

    unsigned int hash_value = 0x000140F0;

    RequestMessageType_Int32 int32_msg;
    int32_msg.header.time_stamp = 122;
    int32_msg.property.authority = Comm_Authority_TP;
    int step = 0;
    cout << "Please input step that you want to switch (0 - 2): " << endl;
    cin >> step;

    if ((step < 0) && (2 < step))
    {
        cout << "step value error" << endl;
        return -1;
    }

    int32_msg.data.data = step;
    cout << "step value is " << step << endl;

    if (!test.generateRequestMessageType(hash_value, (void*)&int32_msg, RequestMessageType_Int32_fields, buf, buf_size))
    {
        cout << "Request : encode buf failed" << endl;
        return -1;
    }

    if (!test.sendRequestBuf(buf, buf_size))
    {
        cout << "Request : send buf falied" << endl;
        return -1;
    }

    buf_size = MAX_REQ_BUFFER_SIZE;
    if (!test.recvResponseBuf(buf, buf_size))
    {
        cout << "Reply : recv buf success, buf size = " << buf_size << endl;
        return -1;
    }

    ResponseMessageType_Uint64 msg;
    unsigned int recv_hash = 0;
    if (!test.decodeResponseMessageType(recv_hash, (void*)&msg, ResponseMessageType_Uint64_fields, buf, buf_size))
    {
        cout << "Reply : recv msg decode success" << endl;
        return -1;
    }
    if (!test.checkHash(recv_hash, hash_value))
    {
        cout << "Reply : hash error ,hash = " << recv_hash << endl;
        return -1;
    }

    cout << "Reply : msg.header.time_stamp = " << msg.header.time_stamp << endl;
    cout << "Reply : msg.header.package_left = " << msg.header.package_left << endl;
    cout << "Reply : msg.header.error_code = " << msg.header.error_code << endl;
    cout << "Reply : msg.property.authority = " << msg.property.authority << endl;
    cout << "Reply : msg.data.data = " << msg.data.data << endl;

    usleep(200000);

    test.shutdownRpcSocket();
    return 0;
}
