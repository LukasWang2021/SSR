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

    double offset;
    int group_id, axis_id;
    cout << "Calibrate zero offset of single axis" << endl;
    cout << "Input group ID:" << endl;
    cin >> group_id;
    cout << "Input axis ID:" << endl;
    cin >> axis_id;

    do {
        unsigned int hash_value = 0x000131D4;
        RequestMessageType_Int32List msg;
        msg.header.time_stamp = 122;
        msg.property.authority = Comm_Authority_TP;
        msg.data.data_count = 2;
        msg.data.data[0] = group_id;
        msg.data.data[1] = axis_id;

        if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_Int32List_fields, buf, buf_size))
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
        cout << "Reply : msg.error_code.data = " << recv_msg.error_code.data << endl;
        cout << "Reply : msg.data.data_count = " << recv_msg.data.data_count << endl;
        cout << "Reply : msg.data.data = " << recv_msg.data.data[0] << ", " << 
                                            recv_msg.data.data[1] << ", " << 
                                            recv_msg.data.data[2] << ", " << 
                                            recv_msg.data.data[3] << ", " << 
                                            recv_msg.data.data[4] << ", " << 
                                            recv_msg.data.data[5] << ", " << 
                                            recv_msg.data.data[6] << ", " << 
                                            recv_msg.data.data[7] << ", " << 
                                            recv_msg.data.data[8] << ", " << endl;
        
        if (recv_msg.error_code.data != 0)
        {
             cout << "Calibrate fail, do not save offset" << endl;
            return 0;
        }

        offset = recv_msg.data.data[axis_id];
    } while (false);

    string str;
    cout << "Save new offset (yes/no) ?" << endl;
    cin >> str;

    if (str != "yes")
    {
        cout << "Do not save offset" << endl;
        return 0;
    }

    cout << "Save offset" << endl;
    unsigned int hash_value = 0x00012404;
    RequestMessageType_Int32List_Double msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP;
    msg.data1.data[0] = group_id;
    msg.data1.data[1] = axis_id;
    msg.data1.data_count = 2;
    msg.data2.data = offset;

    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_Int32List_Double_fields, buf, buf_size))
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
    cout << "Reply : msg.error_code.data = " << recv_msg.data.data << endl;

    if (recv_msg.data.data == 0)
    {
        cout << "Save offset success" << endl;
    }
    else
    {
        cout << "Save offset fail" << endl;
    }

    return 0;
}
