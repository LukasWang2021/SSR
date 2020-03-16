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

    int hash_cin = atoi(argv[1]);
    cout << "Please cin hash type : "<< endl 
        << "1: /rpc/reg_manager/r/getChangedList, hash = 0x0000A904;" << endl
        << "2: /rpc/reg_manager/mr/getChangedList, hash = 0x00001774;" << endl
        << "3: /rpc/reg_manager/sr/getChangedList, hash = 0x00004834;" << endl
        << "4: /rpc/reg_manager/pr/getChangedList, hash = 0x0000B454;" << endl
        << "5: /rpc/reg_manager/hr/getChangedList, hash = 0x00012974;" << endl;
    //cin >> hash_cin;

    unsigned int hash_value = 0;
    if (1 == hash_cin)
        hash_value = 0x0000A904;
    else if (2 == hash_cin)
        hash_value = 0x00001774;
    else if (3 == hash_cin)
        hash_value = 0x00004834;
    else if (4 == hash_cin)
        hash_value = 0x0000B454;
    else if (5 == hash_cin)
        hash_value = 0x00012974;
    else
    {
        cout << "hash type input error" << endl;
        return -1;
    }

    RequestMessageType_Int32List msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP;
    msg.data.data_count = 2;

    msg.data.data[0] = atoi(argv[2]);
    cout << "start id:" << msg.data.data[0] << endl;

    msg.data.data[1] = atoi(argv[3]);
    cout << "end id:" << msg.data.data[1] << endl;

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

    ResponseMessageType_Uint64_BaseRegSummaryList recv_msg;
    unsigned int recv_hash = 0;
    if (!test.decodeResponseMessageType(recv_hash, (void*)&recv_msg,ResponseMessageType_Uint64_BaseRegSummaryList_fields, buf, buf_size))
    {
        cout << "Reply : recv msg decode success" << endl;
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
    cout << "Reply : msg.data.summary_count = " << recv_msg.data.summary_count << endl;
    for (size_t i = 0; i != recv_msg.data.summary_count; ++i)
    {
        cout << "Reply : msg.data.summary[" << i << "].id = " << recv_msg.data.summary[i].id << endl;
        cout << "Reply : msg.data.summary[" << i << "].name = " << recv_msg.data.summary[i].name << endl;
        cout << "Reply : msg.data.summary[" << i << "].comment = " << recv_msg.data.summary[i].comment << endl;
    }

    usleep(200000);

    return 0;
}

