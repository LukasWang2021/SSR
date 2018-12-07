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

    unsigned int hash_value = 0x00012353;

    RequestMessageType_Int32 msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP;
    int cin_data = 0;
    cout << "Please input data/id you want to set:" << endl;
    cin >> cin_data;
    msg.data.data = cin_data;

    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_Int32_fields, buf, buf_size))
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

    ResponseMessageType_Uint64_DoubleList recv_msg;
    unsigned int recv_hash = 0;
    if (!test.decodeResponseMessageType(recv_hash, (void*)&recv_msg, ResponseMessageType_Uint64_DoubleList_fields, buf, buf_size))
    {
        cout << "Reply : recv msg decode success" << endl;
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
    cout << "Reply : msg.error_code = " << recv_msg.error_code.data << endl;
    cout << "Reply : msg.data.data_count = " << recv_msg.data.data_count << endl;

    for (int i = 0; i != recv_msg.data.data_count; ++i)
    {
        cout << "Reply : msg.data.data_count = " << recv_msg.data.data[i] << endl;
    }

    usleep(200000);

    test.shutdownRpcSocket();
    return 0;
}
