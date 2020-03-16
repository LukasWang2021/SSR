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

    unsigned int hash_value = 0x0000C1E0;

    RequestMessageType_Void void_msg;
    void_msg.header.time_stamp = 122;
    void_msg.property.authority = Comm_Authority_TP;

    if (!test.generateRequestMessageType(hash_value, (void*)&void_msg, RequestMessageType_Void_fields, buf, buf_size))
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

    ResponseMessageType_Uint64_DeviceInfoList recv_msg;
    unsigned int recv_hash = 0;
    if (!test.decodeResponseMessageType(recv_hash, (void*)&recv_msg, ResponseMessageType_Uint64_DeviceInfoList_fields, buf, buf_size))
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
    cout << "Reply : msg.error_code = " << recv_msg.error_code.data << endl;
    cout << "Reply : msg.data.device_info_count = " << recv_msg.data.device_info_count << endl;

    for (size_t i = 0; i != recv_msg.data.device_info_count; ++i)
    {
        cout << "Reply : msg.data.device_info["<< i <<"].index = " << recv_msg.data.device_info[i].index << endl;
        cout << "Reply : msg.data.device_info["<< i <<"].address = " << recv_msg.data.device_info[i].address << endl;
        cout << "Reply : msg.data.device_info["<< i <<"].type = " << recv_msg.data.device_info[i].type << endl;
        cout << "Reply : msg.data.device_info["<< i <<"].is_valid = " << recv_msg.data.device_info[i].is_valid << endl;

   }

    usleep(200000);

    return 0;
}
