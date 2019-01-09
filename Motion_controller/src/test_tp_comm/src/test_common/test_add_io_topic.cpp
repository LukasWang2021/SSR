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

    unsigned int hash_value = 0x000058F3;

    RequestMessageType_Topic msg = RequestMessageType_Topic_init_default;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP;
    msg.data.topic_hash = 0x12345678;
    msg.data.time_min = 100;
    msg.data.time_max = 1000;
    msg.data.element_hash_list_count = 10;

    uint32_t port_type = 1;
    port_type = (port_type << 16);
    uint32_t port_offset = 1;
    msg.data.element_hash_list[0] = port_type + port_offset;
    printf("msg.data.element_hash_list[0] = 0x%08x\n", msg.data.element_hash_list[0]);

    port_offset = 2;
    msg.data.element_hash_list[1] = port_type + port_offset;
    printf("msg.data.element_hash_list[1] = 0x%08x\n", msg.data.element_hash_list[1]);

    port_offset = 3;
    msg.data.element_hash_list[2] = port_type + port_offset;
    printf("msg.data.element_hash_list[2] = 0x%08x\n", msg.data.element_hash_list[2]);

    port_offset = 4;
    msg.data.element_hash_list[3] = port_type + port_offset;
    printf("msg.data.element_hash_list[3] = 0x%08x\n", msg.data.element_hash_list[3]);

    port_offset = 5;
    msg.data.element_hash_list[4] = port_type + port_offset;
    printf("msg.data.element_hash_list[4] = 0x%08x\n", msg.data.element_hash_list[4]);

    port_offset = 6;
    msg.data.element_hash_list[5] = port_type + port_offset;
    printf("msg.data.element_hash_list[5] = 0x%08x\n", msg.data.element_hash_list[5]);

    port_offset = 7;
    msg.data.element_hash_list[6] = port_type + port_offset;
    printf("msg.data.element_hash_list[6] = 0x%08x\n", msg.data.element_hash_list[6]);

    port_offset = 8;
    msg.data.element_hash_list[7] = port_type + port_offset;
    printf("msg.data.element_hash_list[7] = 0x%08x\n", msg.data.element_hash_list[7]);

    port_offset = 9;
    msg.data.element_hash_list[8] = port_type + port_offset;
    printf("msg.data.element_hash_list[8] = 0x%08x\n", msg.data.element_hash_list[8]);

    port_offset = 10;
    msg.data.element_hash_list[9] = port_type + port_offset;
    printf("msg.data.element_hash_list[9] = 0x%08x\n", msg.data.element_hash_list[9]);

    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_Topic_fields, buf, buf_size))
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
