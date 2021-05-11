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
    int needed_data_count = 8;
    if (argc < needed_data_count + 1)
    {
        cout << "more parameters are needed: uf_id, tf_id, x, y, z, a, b, c" << endl;
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

    unsigned int hash_value = 0x00010C05;

    RequestMessageType_Int32_UFTF_PoseAndPosture msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP;
    msg.data1.data = 0;
    msg.data2.uf_id.data = atoi(argv[1]);
    msg.data2.tf_id.data = atoi(argv[2]);
    msg.data2.pose_and_posture.pose.data_count = 6;
    msg.data2.pose_and_posture.pose.data[0] = atof(argv[3]);
    msg.data2.pose_and_posture.pose.data[1] = atof(argv[4]);
    msg.data2.pose_and_posture.pose.data[2] = atof(argv[5]);
    msg.data2.pose_and_posture.pose.data[3] = atof(argv[6]);
    msg.data2.pose_and_posture.pose.data[4] = atof(argv[7]);
    msg.data2.pose_and_posture.pose.data[5] = atof(argv[8]);

    msg.data2.pose_and_posture.posture.arm_back_front = 1;
    msg.data2.pose_and_posture.posture.arm_up_down = 1;
    msg.data2.pose_and_posture.posture.wrist_flip = 1;

    msg.data2.pose_and_posture.posture.turn_cycle.data_count = 9;
    msg.data2.pose_and_posture.posture.turn_cycle.data[0] = 0;
    msg.data2.pose_and_posture.posture.turn_cycle.data[1] = 0;
    msg.data2.pose_and_posture.posture.turn_cycle.data[2] = 0;
    msg.data2.pose_and_posture.posture.turn_cycle.data[3] = 0;
    msg.data2.pose_and_posture.posture.turn_cycle.data[4] = 0;
    msg.data2.pose_and_posture.posture.turn_cycle.data[5] = 0;
    msg.data2.pose_and_posture.posture.turn_cycle.data[6] = 0;
    msg.data2.pose_and_posture.posture.turn_cycle.data[7] = 0;
    msg.data2.pose_and_posture.posture.turn_cycle.data[8] = 0;


    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_Int32_DoubleList_fields, buf, buf_size))
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
