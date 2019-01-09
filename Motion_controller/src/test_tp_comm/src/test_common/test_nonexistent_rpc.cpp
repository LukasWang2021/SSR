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

    unsigned int hash_value = 0x87654321;

    RequestMessageType_Topic msg = RequestMessageType_Topic_init_default;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP;
    msg.data.topic_hash = 0x12345678;
    msg.data.time_min = 100;
    msg.data.time_max = 1000;
    msg.data.element_hash_list_count = 0;


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

    cout << "Recv buf success!" << endl;

    ResponseMessageType_Void recv_msg;
    unsigned int recv_hash = 0;

    memcpy(&recv_hash, buf, 4);

    test.checkHash(recv_hash, hash_value);

    printf("Reply : hash error ,hash = %x\n", recv_hash);
    usleep(200000);
    return 0;
}
