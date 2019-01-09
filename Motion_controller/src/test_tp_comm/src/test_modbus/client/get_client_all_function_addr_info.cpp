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

int main(int argc, char** argv)
{
    TpCommTest test;
    if (!test.initRpcSocket())
    {
        printf("Request : socket init failed!\n");
        return -1;
    }

    uint8_t buf[MAX_REQ_BUFFER_SIZE];
    int buf_size = MAX_REQ_BUFFER_SIZE;

    int client_id = atoi(argv[1]);
    printf("client_id = %d\n", client_id);

    RequestMessageType_Int32 msg;
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP;
    msg.data.data = client_id;

    unsigned int hash_value = 0x0000132F;

    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_Int32_fields, buf, buf_size))
    {
        printf("Request : encode buf failed!\n");
        return -1;
    }

    if (!test.sendRequestBuf(buf, buf_size))
    {
        printf("Request : send buf falied!\n");
        return -1;
    }

    buf_size = MAX_REQ_BUFFER_SIZE;
    if (!test.recvResponseBuf(buf, buf_size))
    {
        printf("Reply : recv buf failed, buf size = %d!\n", buf_size);
        return -1;
    }

    ResponseMessageType_Uint64_ModbusAllFucntionAddrInfo recv_msg;
    unsigned int recv_hash = 0;
    if (!test.decodeResponseMessageType(recv_hash, (void*)&recv_msg, ResponseMessageType_Uint64_ModbusAllFucntionAddrInfo_fields, buf, buf_size))
    {
        printf("Reply : recv msg decode failed!\n");
        return -1;
    }

    if (!test.checkHash(recv_hash, hash_value))
    {
        printf("Reply : hash error ,hash = 0x%x\n", recv_hash);
        return -1;
    }

    printf("Reply : msg.header.time_stamp = %d\n", recv_msg.header.time_stamp);
    printf("Reply : msg.header.error_code = 0x%x\n", recv_msg.header.error_code);
    printf("Reply : msg.property.authority = %d\n", recv_msg.property.authority);
    printf("Reply : msg.error_code.data = 0x%x\n", recv_msg.error_code.data);
    cout << "Reply : msg.error_code.data = " << std::hex << recv_msg.error_code.data;

    if (recv_msg.error_code.data != 0)
    {
        printf("Reply :get client config failed\n");
        return -1;
    }

    printf("Reply : msg.data.coil.address = %d\n", recv_msg.data.coil.address);
    printf("Reply : msg.data.coil.number = %d\n", recv_msg.data.coil.number);
    printf("Reply : msg.data.discrepte_input.address = %d\n", recv_msg.data.discrepte_input.address);
    printf("Reply : msg.data.discrepte_input.number = %d\n", recv_msg.data.discrepte_input.number);
    printf("Reply : msg.data.holding_reg.address = %d\n", recv_msg.data.holding_reg.address);
    printf("Reply : msg.data.holding_reg.number = %d\n", recv_msg.data.holding_reg.number);
    printf("Reply : msg.data.input_reg.address = %d\n", recv_msg.data.input_reg.address);
    printf("Reply : msg.data.input_reg.number = %d\n", recv_msg.data.input_reg.number);

    usleep(200000);

    return 0;
}
