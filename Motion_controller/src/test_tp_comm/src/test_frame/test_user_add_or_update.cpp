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

    int hash_cin = 0;
    cout << "Please cin hash type : "<< endl 
        << "1: /rpc/tool_manager/addUser, hash = 0x00016764;" << endl
        << "2: /rpc/tool_manager/updateUser, hash = 0x0000EC14" << endl;
    cin >> hash_cin;

    unsigned int hash_value = 0;
    if (1 == hash_cin)
        hash_value = 0x00016764;
    else if (2 == hash_cin)
        hash_value = 0x0000EC14;
    else
    {
        cout << "hash type input error" << endl;
        return -1;
    }

    RequestMessageType_UserCoordInfo msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP;
    msg.data.id = 2;
    char name[32] = "Hello User frame";
    memcpy(msg.data.name, name, sizeof(name));
    memcpy(msg.data.comment, name, sizeof(name));
    msg.data.group_id = 3;
    msg.data.data.x = 1.1;
    msg.data.data.y = 1.2;
    msg.data.data.z = 1.3;
    msg.data.data.a = 1.4;
    msg.data.data.b = 1.5;
    msg.data.data.c = 1.6;

    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_ToolInfo_fields, buf, buf_size))
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

    ResponseMessageType_Uint64 recv_msg;
    unsigned int recv_hash = 0;
    if (!test.decodeResponseMessageType(recv_hash, (void*)&recv_msg, ResponseMessageType_Uint64_fields, buf, buf_size))
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
    cout << "Reply : msg.data.data = " << recv_msg.data.data << endl;

    usleep(200000);

    test.shutdownRpcSocket();
    return 0;
}
