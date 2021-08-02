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
    int needed_data_count = 3;
    if (argc < needed_data_count + 1)
    {
        cout << "more parameters are needed : id, name, comment size." << endl;
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

    unsigned int hash_value = 0x00009EF7;

    RequestMessageType_PrRegData msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP_SIMMULATOR;
    msg.data.id = atoi(argv[1]);

    string name = argv[2];
    strncpy(msg.data.name, name.c_str(), name.size());
    msg.data.name[name.size()] = 0;

    char comment[256] = "hellohellohellohellohellohellohellohellohellohellohellohellohellohellohellohellohellohellohellohellohellohellohellohellohellohello";
    memcpy(msg.data.comment, comment, 256);

    int comment_size = atoi(argv[3]);
    msg.data.comment[comment_size] = 0;

    printf("name size = %zu, comment size = %d\n", name.size(), comment_size);
    printf("name = %s, comment = %s\n", name.c_str(), msg.data.comment);

    msg.data.group_id = 0;
    msg.data.pos_type = 1;
    msg.data.pos.data_count = 9;
    msg.data.pos.data[0] = 1.1;
    msg.data.pos.data[1] = 1.2;
    msg.data.pos.data[2] = 1.3;
    msg.data.pos.data[3] = 1.4;
    msg.data.pos.data[4] = 1.5;
    msg.data.pos.data[5] = 1.6;
    msg.data.pos.data[6] = 1.7;

    msg.data.posture.wrist_flip = 1;
    msg.data.posture.arm_up_down = 1;
    msg.data.posture.arm_back_front = -1;
    msg.data.posture.arm_left_right = 1;

    msg.data.posture.turn_cycle.data_count = 9;
    msg.data.posture.turn_cycle.data[0] = 1;
    msg.data.posture.turn_cycle.data[1] = 1;
    msg.data.posture.turn_cycle.data[2] = 1;
    msg.data.posture.turn_cycle.data[3] = 1;
    msg.data.posture.turn_cycle.data[4] = 1;
    msg.data.posture.turn_cycle.data[5] = 1;
    msg.data.posture.turn_cycle.data[6] = 1;

    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_PrRegData_fields, buf, buf_size))
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
    cout << "Reply : msg.data.data = 0x" << std::hex << recv_msg.data.data << endl;

    usleep(200000);

    return 0;
}
