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
    if (argc <= 12)
    {
        cout << "12 parameters needed: group-id, tf-id, uf-id, joint[0] ~ joint[8]" << endl;
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

    unsigned int hash_value = 0x0000B6D4;

    RequestMessageType_Int32List_DoubleList msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP_SIMMULATOR;
    msg.data1.data_count = 3;
    msg.data2.data_count = 9;

    msg.data1.data[0] = atoi(argv[1]);
    msg.data1.data[1] = atoi(argv[2]);
    msg.data1.data[2] = atoi(argv[3]);

    msg.data2.data[0] = atof(argv[4]);
    msg.data2.data[1] = atof(argv[5]);
    msg.data2.data[2] = atof(argv[6]);
    msg.data2.data[3] = atof(argv[7]);
    msg.data2.data[4] = atof(argv[8]);
    msg.data2.data[5] = atof(argv[9]);
    msg.data2.data[6] = atof(argv[10]);
    msg.data2.data[7] = atof(argv[11]);
    msg.data2.data[8] = atof(argv[12]);

    cout << "group-id: " << msg.data1.data[0] << ", tf-id: " << msg.data1.data[1] << ", uf-id: " << msg.data1.data[2] << endl;
    cout << "joint: " << msg.data2.data[0] << ", " << msg.data2.data[1] << ", " << msg.data2.data[2] << ", " << msg.data2.data[3] << ", " 
         << msg.data2.data[4] << ", " << msg.data2.data[5] << ", " << msg.data2.data[6] << ", " << msg.data2.data[7] << ", " << msg.data2.data[8] << endl;

    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_Int32List_DoubleList_fields, buf, buf_size))
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

    ResponseMessageType_Uint64_PoseAndPosture recv_msg;
    unsigned int recv_hash = 0;

    if (!test.decodeResponseMessageType(recv_hash, (void*)&recv_msg, ResponseMessageType_Uint64_PoseAndPosture_fields, buf, buf_size))
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
    cout << "Reply : msg.error_code.data = 0x" << hex << recv_msg.error_code.data << dec << endl;
    cout << "Reply : msg.data.pose.data_count = " << recv_msg.data.pose.data_count << endl;

    for (size_t i = 0; i != recv_msg.data.pose.data_count; ++i)
    {
        cout << "Reply : msg.data.pose.data["<< i << "] = " << recv_msg.data.pose.data[i] << endl;
    }

    cout << "Reply : msg.data.posture.wrist_flip = " << recv_msg.data.posture.wrist_flip << endl;
    cout << "Reply : msg.data.posture.arm_up_down = " << recv_msg.data.posture.arm_up_down << endl;
    cout << "Reply : msg.data.posture.arm_back_front = " << recv_msg.data.posture.arm_back_front << endl;
    cout << "Reply : msg.data.posture.arm_left_right = " << recv_msg.data.posture.arm_left_right << endl;
    cout << "Reply : msg.data.posture.turn_cycle.data_count = " << recv_msg.data.posture.turn_cycle.data_count << endl;

    for (size_t i = 0; i != recv_msg.data.posture.turn_cycle.data_count; ++i)
    {
        cout << "Reply : msg.data.posture.turn_cycle.data["<< i << "] = " << recv_msg.data.posture.turn_cycle.data[i] << endl;
    }

    return 0;
}
