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

int main()
{
    TpCommTest test;
    if (!test.initPublishSocket())
    {
        cout << "Subscribe : socket init failed" << endl;
        return -1;
    }

    unsigned int hash_value = 0x12345678;
    if(!test.enablePublishTopic(hash_value)) return -1;
    printf("Subscribe : enable topic = %x\n", hash_value);

    unsigned int hash_value_one = 0x12345611;
    if(!test.enablePublishTopic(hash_value_one)) return -1;
    printf("Subscribe : enable topic = %x\n", hash_value_one);

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
        printf("Sub : recv time_stamp = %d\n", msg.time_stamp);
        printf("Sub : recv element_count = %d\n", msg.element_count);

        for (int i = 0 ; i != msg.element_count; ++i)
        {
            switch(msg.element[i].hash)
            {
                case 0x000123C3:
                {
                    MessageType_Uint32 uint32_data;
                    if(!test.decodeMessageType(msg.element[i].data.bytes, msg.element[i].data.size, 
                        (void*)&uint32_data, MessageType_Uint32_fields))
                    {
                        printf("Sub : parse data msg failed !!\n");
                    }
                    else 
                    {
                        printf("Sub : parse data msg success, hash = 0x%x, data = %d\n",
                            msg.element[i].hash, uint32_data.data);
                    }
                }
                    break;
                case 0x00006D93:
                {
                    MessageType_IoBoardStatusList io;
                    if(!test.decodeMessageType(msg.element[i].data.bytes, msg.element[i].data.size, 
                        (void*)&io, MessageType_IoBoardStatusList_fields))
                    {
                        printf("Sub : parse data msg failed !!\n");
                    }
                    else 
                    {
                        printf("Sub : parse data msg success, hash = 0x%x, io_board_count = %d\n",
                            msg.element[i].hash, io.io_board_count);

                        for (int i = 0; i != io.io_board_count; ++i)
                        {
                            printf("io_board[%d].id = %d\n", i, io.io_board[i].id);
                            printf("io_board[%d].DI = %d\n", i, io.io_board[i].DI);
                            printf("io_board[%d].DO = %d\n", i, io.io_board[i].DO);
                            printf("io_board[%d].RI = %d\n", i, io.io_board[i].RI);
                            printf("io_board[%d].RO = %d\n", i, io.io_board[i].RO);
                            printf("io_board[%d].valid = %d\n", i, io.io_board[i].valid);
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
