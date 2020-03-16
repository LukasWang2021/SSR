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
    int needed_data_count = 1;
    if (argc < needed_data_count + 1)
    {
        cout << "more parameters are needed" << endl;
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

    unsigned int hash_value = 0x00002FC4;

    RequestMessageType_Int32 msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP;
    msg.data.data = atoi(argv[1]);

    printf("add client id = %d\n", msg.data.data);

    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_Int32_fields, buf, buf_size))
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

    ResponseMessageType_Uint64_Bool recv_msg;
    unsigned int recv_hash = 0;
    if (!test.decodeResponseMessageType(recv_hash, (void*)&recv_msg, ResponseMessageType_Uint64_Bool_fields, buf, buf_size))
    {
        printf("Reply : recv msg decode failed\n");
        return -1;
    }
    if (!test.checkHash(recv_hash, hash_value))
    {
        printf("Reply : hash error ,hash = 0x%x\n", recv_hash);
        return -1;
    }

    printf("Reply : msg.header.time_stamp = %lld\n", recv_msg.header.time_stamp);
    printf("Reply : msg.header.package_left = %d\n", recv_msg.header.package_left);
    printf("Reply : msg.header.error_code = 0x%llx\n", recv_msg.header.error_code);
    printf("Reply : msg.property.authority = %d\n", recv_msg.property.authority);
    printf("Reply : msg.data.data = 0x%x\n", recv_msg.data.data);
    cout << "Reply : msg.error_code.data = " << std::hex << recv_msg.error_code.data << endl;
    cout << "Reply : msg.data.data = " << std::hex << recv_msg.data.data << endl;

    usleep(200000);

    return 0;
}
