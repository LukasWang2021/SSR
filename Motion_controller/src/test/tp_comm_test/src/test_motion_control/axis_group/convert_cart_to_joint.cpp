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
    if (argc <= 13)
    {
        cout << "at least 13 parameters needed: group-id, tf-id, uf-id, x, y, z, a, b, c, flip, wrist, elbow, arm, {turn[0] ~ turn[8]}" << endl;
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

    unsigned int hash_value = 0x00010FD4;

    RequestMessageType_Int32_UFTF_PoseAndPosture msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP_SIMMULATOR;
    msg.data1.data = atoi(argv[1]);
    msg.data2.tf_id.data = atoi(argv[2]);
    msg.data2.uf_id.data = atoi(argv[3]);
    msg.data2.pose_and_posture.pose.data_count = 6;
    msg.data2.pose_and_posture.pose.data[0] = atof(argv[4]);
    msg.data2.pose_and_posture.pose.data[1] = atof(argv[5]);
    msg.data2.pose_and_posture.pose.data[2] = atof(argv[6]);
    msg.data2.pose_and_posture.pose.data[3] = atof(argv[7]);
    msg.data2.pose_and_posture.pose.data[4] = atof(argv[8]);
    msg.data2.pose_and_posture.pose.data[5] = atof(argv[9]);
    msg.data2.pose_and_posture.posture.wrist_flip = atoi(argv[10]);
    msg.data2.pose_and_posture.posture.arm_up_down = atoi(argv[11]);
    msg.data2.pose_and_posture.posture.arm_back_front = atoi(argv[12]);
    msg.data2.pose_and_posture.posture.arm_left_right = atoi(argv[13]);
    msg.data2.pose_and_posture.posture.turn_cycle.data_count = 9;

    if (argc <= 22)
    {
        memset(msg.data2.pose_and_posture.posture.turn_cycle.data, 0, msg.data2.pose_and_posture.posture.turn_cycle.data_count * sizeof(int));
    }
    else
    {
        msg.data2.pose_and_posture.posture.turn_cycle.data[0] = atoi(argv[14]);
        msg.data2.pose_and_posture.posture.turn_cycle.data[1] = atoi(argv[15]);
        msg.data2.pose_and_posture.posture.turn_cycle.data[2] = atoi(argv[16]);
        msg.data2.pose_and_posture.posture.turn_cycle.data[3] = atoi(argv[17]);
        msg.data2.pose_and_posture.posture.turn_cycle.data[4] = atoi(argv[18]);
        msg.data2.pose_and_posture.posture.turn_cycle.data[5] = atoi(argv[19]);
        msg.data2.pose_and_posture.posture.turn_cycle.data[6] = atoi(argv[20]);
        msg.data2.pose_and_posture.posture.turn_cycle.data[7] = atoi(argv[21]);
        msg.data2.pose_and_posture.posture.turn_cycle.data[8] = atoi(argv[22]);
    }

    cout << "group-id: " << msg.data1.data << ", tf-id: " << msg.data2.tf_id.data << ", uf-id: " << msg.data2.uf_id.data << endl;
    cout << "x = " <<  msg.data2.pose_and_posture.pose.data[0] << ", y = " <<  msg.data2.pose_and_posture.pose.data[1] << ", z = " <<  msg.data2.pose_and_posture.pose.data[2]
         << ", a = " <<  msg.data2.pose_and_posture.pose.data[3] << ", b = " <<  msg.data2.pose_and_posture.pose.data[4] << ", c = " <<  msg.data2.pose_and_posture.pose.data[5] << endl;
    cout << "wrist-flip: " << msg.data2.pose_and_posture.posture.wrist_flip << ", arm-up-down: " << msg.data2.pose_and_posture.posture.arm_up_down
         << ", arm-back-front: " << msg.data2.pose_and_posture.posture.arm_back_front << ", arm-left-right: " << msg.data2.pose_and_posture.posture.arm_left_right << endl;
    cout << "turn: " << msg.data2.pose_and_posture.posture.turn_cycle.data[0] << ", " << msg.data2.pose_and_posture.posture.turn_cycle.data[1] << ", " << msg.data2.pose_and_posture.posture.turn_cycle.data[2]
         << ", " << msg.data2.pose_and_posture.posture.turn_cycle.data[3] << ", " << msg.data2.pose_and_posture.posture.turn_cycle.data[4] << ", " << msg.data2.pose_and_posture.posture.turn_cycle.data[5]
         << ", " << msg.data2.pose_and_posture.posture.turn_cycle.data[6] << ", " << msg.data2.pose_and_posture.posture.turn_cycle.data[7] << ", " << msg.data2.pose_and_posture.posture.turn_cycle.data[8] << endl;
    
    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_Int32_UFTF_PoseAndPosture_fields, buf, buf_size))
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

    ResponseMessageType_Uint64_DoubleList recv_msg;
    unsigned int recv_hash = 0;

    if (!test.decodeResponseMessageType(recv_hash, (void*)&recv_msg, ResponseMessageType_Uint64_DoubleList_fields, buf, buf_size))
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
    cout << "Reply : msg.data.data_count = " << recv_msg.data.data_count << endl;

    for (size_t i = 0; i != recv_msg.data.data_count; ++i)
    {
        cout << "Reply : msg.data.data["<< i << "] = " << recv_msg.data.data[i] << endl;
    }

    return 0;
}
