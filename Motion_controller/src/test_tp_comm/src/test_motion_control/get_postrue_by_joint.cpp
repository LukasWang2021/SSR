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
    if (argc <= 8)
    {
        printf("nine parameter is needed: user_port and port_value\n");
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

    unsigned int hash_value = 0x0000EC64;

    RequestMessageType_Int32_DoubleList send_msg;
    send_msg.header.time_stamp = 122;
    send_msg.property.authority = Comm_Authority_TP;
    send_msg.data1.data = 1;
    send_msg.data2.data_count = 9;
    send_msg.data2.data[0] = atof(argv[1]);
    send_msg.data2.data[1] = atof(argv[2]);
    send_msg.data2.data[2] = atof(argv[3]);
    send_msg.data2.data[3] = atof(argv[4]);
    send_msg.data2.data[4] = atof(argv[5]);
    send_msg.data2.data[5] = atof(argv[6]);
    send_msg.data2.data[6] = atof(argv[7]);
    send_msg.data2.data[7] = atof(argv[8]);
    send_msg.data2.data[8] = atof(argv[9]);

    if (!test.generateRequestMessageType(hash_value, (void*)&send_msg, RequestMessageType_Int32_DoubleList_fields, buf, buf_size))
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

    ResponseMessageType_Uint64_Posture recv_msg;
    unsigned int recv_hash = 0;
    if (!test.decodeResponseMessageType(recv_hash, (void*)&recv_msg, ResponseMessageType_Uint64_Posture_fields, buf, buf_size))
    {
        cout << "Reply : recv msg decode failed" << endl;
        return -1;
    }
    if (!test.checkHash(recv_hash, hash_value))
    {
        cout << "Reply : hash error ,hash = " << recv_hash << endl;
        return -1;
    }

    cout << "Reply : recv_msg.header.time_stamp = " << recv_msg.header.time_stamp << endl;
    cout << "Reply : recv_msg.header.package_left = " << recv_msg.header.package_left << endl;
    cout << "Reply : recv_msg.header.error_code = " << recv_msg.header.error_code << endl;
    cout << "Reply : recv_msg.property.authority = " << recv_msg.property.authority << endl;
    cout << "Reply : recv_msg.error_code = " << recv_msg.error_code.data << endl;
    cout << "Reply : recv_msg.data.wrist_flip = " << recv_msg.data.wrist_flip << endl;
    cout << "Reply : recv_msg.data.arm_up_down = " << recv_msg.data.arm_up_down << endl;
    cout << "Reply : recv_msg.data.arm_back_front = " << recv_msg.data.arm_back_front << endl;
    cout << "Reply : recv_msg.data.arm_left_right = " << recv_msg.data.arm_left_right << endl;
    cout << "Reply : recv_msg.data.turn_cycle.data_count = " << recv_msg.data.turn_cycle.data_count << endl;

    for (size_t i = 0; i != recv_msg.data.turn_cycle.data_count; ++i)
    {
        cout << "Reply : recv_msg.data.turn_cycle.data[]=" << recv_msg.data.turn_cycle.data[i] << endl;
    }

    usleep(200000);

    return 0;
}
