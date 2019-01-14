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

    unsigned int hash_value = 0x000163A3;

    RequestMessageType_Topic msg = RequestMessageType_Topic_init_default;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP;
    msg.data.topic_hash = 0x00001234;
    msg.data.time_min = 100;
    msg.data.time_max = 1000;
    msg.data.element_hash_list_count = 14;

    int32_t reg_type = MessageType_RegType_PR;
    reg_type = (reg_type << 16);
    int32_t reg_index = 1;
    msg.data.element_hash_list[0] = reg_type + reg_index;
    printf("msg.data.element_hash_list[0] = 0x%08x\n", msg.data.element_hash_list[0]);
    reg_index = 2;
    msg.data.element_hash_list[1] = reg_type + reg_index;
    printf("msg.data.element_hash_list[1] = 0x%08x\n", msg.data.element_hash_list[1]);

    reg_type = MessageType_RegType_HR;
    reg_type = (reg_type << 16);
    reg_index = 1;
    msg.data.element_hash_list[2] = reg_type + reg_index;
    printf("msg.data.element_hash_list[2] =  0x%08x\n", msg.data.element_hash_list[2]);
    reg_index = 2;
    msg.data.element_hash_list[3] = reg_type + reg_index;
    printf("msg.data.element_hash_list[3] =  0x%08x\n", msg.data.element_hash_list[3]);

    reg_type = MessageType_RegType_SR;
    reg_type = (reg_type << 16);
    reg_index = 1;
    msg.data.element_hash_list[4] = reg_type + reg_index;
    printf("msg.data.element_hash_list[4] =  0x%08x\n", msg.data.element_hash_list[4]);
    reg_index = 2;
    msg.data.element_hash_list[5] = reg_type + reg_index;
    printf("msg.data.element_hash_list[5] =  0x%08x\n", msg.data.element_hash_list[5]);

    reg_type = MessageType_RegType_MR;
    reg_type = (reg_type << 16);
    reg_index = 1;
    msg.data.element_hash_list[6] = reg_type + reg_index;
    printf("msg.data.element_hash_list[6] =  0x%08x\n", msg.data.element_hash_list[6]);
    reg_index = 2;
    msg.data.element_hash_list[7] = reg_type + reg_index;
    printf("msg.data.element_hash_list[7] =  0x%08x\n", msg.data.element_hash_list[7]);

    reg_type = MessageType_RegType_R;
    reg_type = (reg_type << 16);
    reg_index = 1;
    msg.data.element_hash_list[8] = reg_type + reg_index;
    printf("msg.data.element_hash_list[8] =  0x%08x\n", msg.data.element_hash_list[8]);
    reg_index = 2;
    msg.data.element_hash_list[9] = reg_type + reg_index;
    printf("msg.data.element_hash_list[9] =  0x%08x\n", msg.data.element_hash_list[9]);
    reg_index = 3;
    msg.data.element_hash_list[10] = reg_type + reg_index;
    printf("msg.data.element_hash_list[10] =  0x%08x\n", msg.data.element_hash_list[10]);
    reg_index = 4;
    msg.data.element_hash_list[11] = reg_type + reg_index;
    printf("msg.data.element_hash_list[11] =  0x%08x\n", msg.data.element_hash_list[11]);
    reg_index = 5;
    msg.data.element_hash_list[12] = reg_type + reg_index;
    printf("msg.data.element_hash_list[12] =  0x%08x\n", msg.data.element_hash_list[12]);
    reg_index = 6;
    msg.data.element_hash_list[13] = reg_type + reg_index;
    printf("msg.data.element_hash_list[13] =  0x%08x\n", msg.data.element_hash_list[13]);

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
