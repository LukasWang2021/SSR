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
    int needed_data_count = 2;
    if (argc < needed_data_count + 1)
    {
        cout << "more parameters are needed : rpc index, register id" << endl;
        cout <<  "rpc hash index : "<< endl;
        cout << "1: /rpc/reg_manager/r/deleteReg" << endl;
        cout << "2: /rpc/reg_manager/mr/deleteReg" << endl;
        cout << "3: /rpc/reg_manager/sr/deleteReg" << endl;
        cout << "4: /rpc/reg_manager/pr/deleteReg" << endl;
        cout << "5: /rpc/reg_manager/hr/deleteReg" << endl;
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
        << "1: /rpc/reg_manager/r/deleteReg, hash = 0x000012F7;" << endl
        << "2: /rpc/reg_manager/mr/deleteReg, hash = 0x0000E5D7;" << endl
        << "3: /rpc/reg_manager/sr/deleteReg, hash = 0x0000B817;" << endl
        << "4: /rpc/reg_manager/pr/deleteReg, hash = 0x00001097;" << endl
        << "5: /rpc/reg_manager/hr/deleteReg, hash = 0x00003D17;" << endl;
    //cin >> hash_cin;

    unsigned int hash_value = 0;
    if (1 == hash_cin)
        hash_value = 0x000012F7;
    else if (2 == hash_cin)
        hash_value = 0x0000E5D7;
    else if (3 == hash_cin)
        hash_value = 0x0000B817;
    else if (4 == hash_cin)
        hash_value = 0x00001097;
    else if (5 == hash_cin)
        hash_value = 0x00003D17;
    else
    {
        cout << "hash type input error" << endl;
        return -1;
    }

    RequestMessageType_Int32 msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP_SIMMULATOR;
    msg.data.data = atoi(argv[2]);
    printf("id = %d\n", msg.data.data);

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
