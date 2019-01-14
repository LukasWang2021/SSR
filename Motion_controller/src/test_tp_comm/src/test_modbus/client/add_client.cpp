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
    unsigned int hash_value = 0x00012E44;

    RequestMessageType_ModbusClientStartInfo msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP;
    msg.data.id = atoi(argv[1]);
    //msg.data.name = atoi(argv[1]);
    memcpy(msg.data.name, argv[2], sizeof(argv[2]));
    //memcpy(msg.data.ip, argv[3], sizeof(argv[3]));
    strcpy(msg.data.ip, argv[3]);
    printf("argv[3] = %s\n", argv[3]);
    msg.data.port = atoi(argv[4]);
    msg.data.scan_rate = atoi(argv[5]);
    msg.data.response_timeout = atoi(argv[6]);

    printf("add client id = %d\n", msg.data.id);
    printf("add client name = %s\n", msg.data.name);
    printf("add client ip = %s\n", msg.data.ip);
    printf("add client port = %d\n", msg.data.port);
    printf("add client scan_rate = %d\n", msg.data.scan_rate);
    printf("add client response_timeout = %d\n", msg.data.response_timeout);

    if (sizeof(msg.data.ip) < 6)
    {
        printf("argv[3] = %s", argv[3]);
        printf("ip size = %d", sizeof(msg.data.ip));
        printf("ip = %s", msg.data.ip);
        return -1;
    }

    TpCommTest test;
    if (!test.initRpcSocket())
    {
        printf("Request : socket init failed \n");
        return -1;
    }

    uint8_t buf[MAX_REQ_BUFFER_SIZE];
    int buf_size = MAX_REQ_BUFFER_SIZE;

    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_ModbusClientStartInfo_fields, buf, buf_size))
    {
        printf("Request : encode buf failed \n");
        return -1;
    }

    if (!test.sendRequestBuf(buf, buf_size))
    {
        printf("Request : send buf falied!\n");
        return -1;
    }

    buf_size = MAX_REQ_BUFFER_SIZE;
    if (!test.recvResponseBuf(buf, buf_size))
    {
        printf("Reply : recv buf failed, buf size = %d\n", buf_size);
        return -1;
    }

    ResponseMessageType_Uint64 recv_msg;
    unsigned int recv_hash = 0;
    if (!test.decodeResponseMessageType(recv_hash, (void*)&recv_msg, ResponseMessageType_Uint64_fields, buf, buf_size))
    {
        printf("Reply : recv msg decode failed\n");
        return -1;
    }
    if (!test.checkHash(recv_hash, hash_value))
    {
        printf("Reply : hash error ,hash = 0x%x\n", recv_hash);
        return -1;
    }

    printf("Reply : msg.header.time_stamp = %d\n", recv_msg.header.time_stamp);
    printf("Reply : msg.header.package_left = %d\n", recv_msg.header.package_left);
    printf("Reply : msg.header.error_code = 0x%x\n", recv_msg.header.error_code);
    printf("Reply : msg.property.authority = %d\n", recv_msg.property.authority);
    printf("Reply : msg.data.data = 0x%x\n", recv_msg.data.data);
    cout << "Reply : msg.data.data = " << std::hex << recv_msg.data.data << endl;

    usleep(200000);

    return 0;
}