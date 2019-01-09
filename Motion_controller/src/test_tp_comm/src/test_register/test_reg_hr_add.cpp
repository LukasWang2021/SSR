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
    TpCommTest test;
    if (!test.initRpcSocket())
    {
        cout << "Request : socket init failed" << endl;
        return -1;
    }

    uint8_t buf[MAX_REQ_BUFFER_SIZE];
    int buf_size = MAX_REQ_BUFFER_SIZE;

    unsigned int hash_value = 0x00016CE7;

    RequestMessageType_HrRegData msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP;
    msg.data.id = atoi(argv[1]);
    char name[32] = "Hello Reg";
    memcpy(msg.data.name, name, sizeof(name));
    memcpy(msg.data.comment, name, sizeof(name));
    msg.data.group_id = atoi(argv[2]);

    msg.data.joints.data_count = 9;
    msg.data.joints.data[0] = 1.1;
    msg.data.joints.data[1] = 1.2;
    msg.data.joints.data[2] = 1.3;
    msg.data.joints.data[3] = 1.4;
    msg.data.joints.data[4] = 1.5;
    msg.data.joints.data[5] = 1.6;
    msg.data.joints.data[6] = 1.7;

    msg.data.diffs.data_count = 9;
    msg.data.diffs.data[0] = true;
    msg.data.diffs.data[1] = true;
    msg.data.diffs.data[2] = true;
    msg.data.diffs.data[3] = true;
    msg.data.diffs.data[4] = true;
    msg.data.diffs.data[5] = true;
    msg.data.diffs.data[6] = true;

    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_HrRegData_fields, buf, buf_size))
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
