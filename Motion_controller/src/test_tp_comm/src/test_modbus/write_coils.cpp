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

int main(int argc, char** argv)
{
    TpCommTest test;
    if (!test.initRpcSocket())
    {
        printf("Request : socket init failed\n");
        return -1;
    }

    uint8_t buf[MAX_REQ_BUFFER_SIZE];
    int buf_size = MAX_REQ_BUFFER_SIZE;

    unsigned int hash_value = 0x0000BD83;

    RequestMessageType_Int32_ModbusStatusInfo msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP;

    int client_id = atoi(argv[1]);
    int address = atoi(argv[2]);
    int number = atoi(argv[3]);
    int value_count = atoi(argv[4]);
    bool value_0 = atoi(argv[5]);
    bool value_1 = atoi(argv[6]);
    bool value_2 = atoi(argv[7]);
    msg.data1.data = client_id;
    msg.data2.address = address;
    msg.data2.number = number;
    msg.data2.value_count = value_count;
    msg.data2.value[0] = value_0;
    msg.data2.value[1] = value_1;
    msg.data2.value[2] = value_2;

    printf("client_id = %d; value_1 = %d; number = %d; value_count = %d; value = %d\n",
        msg.data1.data, msg.data2.address, msg.data2.number, msg.data2.value_count);

    printf("value_0 = %d; address = %d; value_2 = %d\n",
        value_0, value_1, value_2);

    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_Int32_ModbusStatusInfo_fields, buf, buf_size))
    {
        printf("Request : encode buf failed!\n");
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
        printf("Reply : recv buf falied, buf size = %d\n", buf_size);
        return -1;
    }

    ResponseMessageType_Uint64 recv_msg;
    unsigned int recv_hash = 0;
    if (!test.decodeResponseMessageType(recv_hash, (void*)&recv_msg, ResponseMessageType_Uint64_fields, buf, buf_size))
    {
        printf( "Reply : recv msg decode failed\n");
        return -1;
    }
    if (!test.checkHash(recv_hash, hash_value))
    {
        printf("Reply : hash error ,hash = 0x%x\n", recv_hash);
        return -1;
    }

    printf("Reply : msg.header.time_stamp = %d\n", recv_msg.header.time_stamp);
    printf("Reply : msg.header.package_left = %d\n", recv_msg.header.package_left);
    printf("Reply : msg.property.authority = %d\n", recv_msg.property.authority);
    printf("Reply : msg.data.data = 0x%x\n", recv_msg.data.data);
    cout << "Reply : msg.data.data = " << std::hex << recv_msg.data.data << endl;;

    usleep(200000);

    return 0;
}
