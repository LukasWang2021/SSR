#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/ws.h>

#include <pb_encode.h>
#include <pb_decode.h>
#include <pb_common.h>

#include <time.h>
#include "protoc.h"
#include "tp_comm_test.h"

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
    if (!test.initPublishSocket())
    {
        cout << "Subscribe : socket init failed" << endl;
        return -1;
    }

    unsigned int hash_value = atoi(argv[1]);
    if(!test.enablePublishTopic(hash_value)) return -1;
    printf("Subscribe : enable topic = %x\n", hash_value);

    int i = 0;

    for (;;) 
    {
        cout << "===================  sub test " << i << "==================" << endl;
        i++;
        uint8_t sub_buf[1024];
        int bytes = 0;

        if ((bytes = test.recvPublishBuf(sub_buf)) < 0)
            cout << "subscribe : recv failed" << endl;
        else
            cout << "subscribe : recv buf size = " << bytes << endl;

        unsigned int recv_hash = 0;
        Comm_Publish msg;
        if(!test.decodeCommPublishBuf(sub_buf, bytes, recv_hash, msg))
        {
            cout << "Subscribe : decode topic failed" << endl;
            return -1;
        }

        printf("Sub : recv topic hash = %x\n", recv_hash);
        printf("Sub : recv time_stamp = %lld\n", msg.time_stamp);
        printf("Sub : recv element_count = %d\n", msg.element_count);

        for (size_t i = 0 ; i != msg.element_count; ++i)
        {
            switch(msg.element[i].hash)
            {
                case 0x00011843:
                {
                    MessageType_ModbusClientCtrlStatusList client_data;
                    if(!test.decodeMessageType(msg.element[i].data.bytes, msg.element[i].data.size, 
                        (void*)&client_data, MessageType_ModbusClientCtrlStatusList_fields))
                    {
                        printf("Sub : parse data msg failed !!\n");
                    }
                    else 
                    {
                        printf("Sub : parse data msg success, hash = 0x%x, ctrl_status_count = %d\n",
                            msg.element[i].hash, client_data.ctrl_status_count);
                        for (size_t i = 0; i != client_data.ctrl_status_count; ++i)
                        {
                            printf("Sub : client_data[%d].id = %d, client_data[%d].status = %d\n",i, 
                                client_data.ctrl_status[i].id, i, client_data.ctrl_status[i].status);
                        }
                    }
                }
                    break;
                default:
                {
                    MessageType_Int32 int32_data;
                    if(!test.decodeMessageType(msg.element[i].data.bytes, msg.element[i].data.size, 
                                                (void*)&int32_data, MessageType_Int32_fields))
                        printf("Sub : parse data msg failed !!\n");
                    else
                    {
                        printf("Sub : parse data msg success, hash = %x, data = %d\n",
                            msg.element[i].hash, int32_data.data);
                    }
                }
            };
        }

        usleep(500000);
    }
    return 0;
}

