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
        << "1: /rpc/reg_manager/sr/addReg, hash = 0x000161E7;" << endl
        << "2: /rpc/reg_manager/sr/updateReg, hash = 0x000119F7" << endl;
    cin >> hash_cin;

    unsigned int hash_value = 0;
    if (1 == hash_cin)
        hash_value = 0x000161E7;
    else if (2 == hash_cin)
        hash_value = 0x000119F7;
    else
    {
        cout << "hash type input error" << endl;
        return -1;
    }

    RequestMessageType_SrRegData msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP;
    cout << "Please input id:" << endl;
    int id = 0;
    cin >> id;
    msg.data.id = id;
    char name[32] = "你好";
    printf("please input SR value :");
    cin >> name;    
    memcpy(msg.data.name, name, sizeof(name));
    memcpy(msg.data.comment, name, sizeof(name));
    memcpy(msg.data.value, name, sizeof(name));

    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_SrRegData_fields, buf, buf_size))
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
}
