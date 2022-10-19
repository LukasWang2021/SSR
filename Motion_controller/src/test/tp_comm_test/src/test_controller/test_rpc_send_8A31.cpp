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
//#include "request_transmatrixlist.pb.h"

#define MAX_REQ_BUFFER_SIZE     (65535)

using namespace std;

int main(int argc, char* argv[])
{
    /*
    int needed_data_count = 1;
    if (argc < needed_data_count + 1)
    {
        cout << "more parameters are needed:argv" << endl;
        return -1;
    }*/

    TpCommTest test;
    if (!test.initRpcSocket())
    {
        cout << "Request : socket init failed" << endl;
        return -1;
    }
    uint8_t buf[MAX_REQ_BUFFER_SIZE];
    int buf_size = MAX_REQ_BUFFER_SIZE;
    unsigned int hash_value = 0x00008A31;
    RequestMessageType_TransMatrixList msg;
    //memset(msg,RequestMessageType_TransMatrixList_init_zero,sizeof(RequestMessageType_TransMatrixList));
    msg.header.time_stamp = 122;
    msg.property.authority = Comm_Authority_TP_SIMMULATOR;

    double m_data[96] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,
                        17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,
                        33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,
                        49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,
                        65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,
                        81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96};
    int m_state[6] = {1,2,3,4,5,6};
    //msg.data.data = atof(argv[1]);
    msg.data.matrices_count = 6;
    for(int i=0;i<msg.data.matrices_count;i++)
    {
        msg.data.matrices[i].state = m_state[i];
        msg.data.matrices[i].matrix_count = 16;
        memcpy(&msg.data.matrices[i].matrix[0], &m_data[16 * i], 16 * sizeof(double));
        printf("state=%d  \n[%lf,%lf,%lf,%lf]\n[%lf,%lf,%lf,%lf]\n[%lf,%lf,%lf,%lf]\n[%lf,%lf,%lf,%lf]\n",msg.data.matrices[i].state,\
msg.data.matrices[i].matrix[0],msg.data.matrices[i].matrix[1],msg.data.matrices[i].matrix[2],msg.data.matrices[i].matrix[3],\
msg.data.matrices[i].matrix[4],msg.data.matrices[i].matrix[5],msg.data.matrices[i].matrix[6],msg.data.matrices[i].matrix[7],\
msg.data.matrices[i].matrix[8],msg.data.matrices[i].matrix[9],msg.data.matrices[i].matrix[10],msg.data.matrices[i].matrix[11],\
msg.data.matrices[i].matrix[12],msg.data.matrices[i].matrix[13],msg.data.matrices[i].matrix[14],msg.data.matrices[i].matrix[15]);
    }
    if (!test.generateRequestMessageType(hash_value, (void*)&msg, RequestMessageType_TransMatrixList_fields, buf, buf_size))
    {
        cout << "Request : encode buf failed" << endl;
        return -1;
    }
    printf("\ngenerateRequestMessageType to buf, buf_size=%d\n",buf_size);
    for(int i = 0;i< buf_size;i++)
    {
        printf("%x ",*(buf+i));
        if(i%15 == 0 && i!=0){printf("\n");}
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
    printf("Reply : error_code = 0x%llx\n", (long long unsigned int)recv_msg.data.data);

    usleep(200000);

    return 0;
}
