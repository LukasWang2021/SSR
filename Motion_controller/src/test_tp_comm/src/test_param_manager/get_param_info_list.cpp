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
    int needed_data_count = 1;
    if (argc < needed_data_count + 1)
    {
        cout << "more parameters are needed" << endl;
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

    unsigned int hash_value = 0x0000F0B4;

    RequestMessageType_Int32 msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP;
    msg.data.data = atoi(argv[1]);
    cout << "msg.data.data = " << msg.data.data << endl;

    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_Int32_fields, buf, buf_size))
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

    ResponseMessageType_Uint64_ParamInfoList recv_msg;
    unsigned int recv_hash = 0;
    if (!test.decodeResponseMessageType(recv_hash, (void*)&recv_msg, ResponseMessageType_Uint64_ParamInfoList_fields, buf, buf_size))
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
    cout << "Reply : msg.error_code = " <<recv_msg.error_code.data << endl;
    cout << "Reply : msg.data.param_info_count = " << recv_msg.data.param_info_count << endl;

    for (int i = 0; i != recv_msg.data.param_info_count; ++i)
    {
        cout << "Reply : msg.data.data.param_info[i].name" << recv_msg.data.param_info[i].name << endl;
        cout << "Reply : msg.data.data.param_info[i].type" << recv_msg.data.param_info[i].type << endl;
        cout << "Reply : msg.data.data.param_info[i].data" << std::putwchar <<  recv_msg.data.param_info[i].data.bytes << endl;
        printf("Reply : data =%02x-%02x-%02x-%02x--%02x-%02x-%02x-%02x\n", 
        recv_msg.data.param_info[i].data.bytes[7],recv_msg.data.param_info[i].data.bytes[6],
        recv_msg.data.param_info[i].data.bytes[5],recv_msg.data.param_info[i].data.bytes[4],
        recv_msg.data.param_info[i].data.bytes[3],recv_msg.data.param_info[i].data.bytes[2],
        recv_msg.data.param_info[i].data.bytes[1],recv_msg.data.param_info[i].data.bytes[0]);
    }

    usleep(200000);

    return 0;
}