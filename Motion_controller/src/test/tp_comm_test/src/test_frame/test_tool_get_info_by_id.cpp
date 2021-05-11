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
    int needed_data_count = 1;
    if (argc < needed_data_count + 1)
    {
        cout << "more parameters are needed: frame_id" << endl;
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

    unsigned int hash_value = 0x00009E34;

    RequestMessageType_Int32 msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP;
    msg.data.data = atoi(argv[1]);

    cout << "value is " << msg.data.data << endl;

    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_Int32_fields, buf, buf_size))
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

    ResponseMessageType_Uint64_ToolInfo recv_msg;
    unsigned int recv_hash = 0;
    if (!test.decodeResponseMessageType(recv_hash, (void*)&recv_msg, ResponseMessageType_Uint64_ToolInfo_fields, buf, buf_size))
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
    cout << "Reply : msg.error_code = " << recv_msg.error_code.data << endl;
    cout << "Reply : msg.data.id = " << recv_msg.data.id << endl;
    cout << "Reply : msg.data.name = " << recv_msg.data.name << endl;
    cout << "Reply : msg.data.comment = " << recv_msg.data.comment << endl;
    cout << "Reply : msg.data.group_id = " << recv_msg.data.group_id << endl;
    cout << "Reply : msg.data.data.x = " << recv_msg.data.data.data[0] << endl;
    cout << "Reply : msg.data.data.y = " << recv_msg.data.data.data[1] << endl;
    cout << "Reply : msg.data.data.z = " << recv_msg.data.data.data[2] << endl;
    cout << "Reply : msg.data.data.a = " << recv_msg.data.data.data[3] << endl;
    cout << "Reply : msg.data.data.b = " << recv_msg.data.data.data[4] << endl;
    cout << "Reply : msg.data.data.c = " << recv_msg.data.data.data[5] << endl;

    usleep(200000);

    return 0;
}

