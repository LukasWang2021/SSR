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
    if(!test.enablePublishTopic(hash_value)) 
    {
        printf("failed to enable topic!");
        return -1;
    }

    printf("Subscribe : success to enable topic = %x\n", hash_value);

    int i = 0;

    for (;;)
    {
        cout << "===================  sub test " << i << "==================" << endl;
        i++;
        uint8_t sub_buf[1024];
        int bytes = 0;

        if ((bytes = test.recvPublishBuf(sub_buf)) < 0)
        {
            cout << "subscribe : recv failed" << endl;
            continue;
        }

        cout << "subscribe : recv buf size = " << bytes << endl;

        unsigned int recv_hash = 0;
        Comm_Publish msg;
        if(!test.decodeCommPublishBuf(sub_buf, bytes, recv_hash, msg))
        {
            cout << "Subscribe : decode msg failed" << endl;
            continue;
        }

        printf("Sub : recv topic hash = %x\n", recv_hash);
        printf("Sub : recv time_stamp = %d\n", msg.time_stamp);
        printf("Sub : recv element_count = %d\n", msg.element_count);

        MessageType_Uint32 io_value;

        for (int i = 0 ; i != msg.element_count; ++i)
        {
            //int device_index = (msg.element[i].hash >> 24);
            uint32_t port_type = (msg.element[i].hash >> 16) & 0x0000FFFF;
            uint32_t port_offset = msg.element[i].hash & 0x0000FFFF;
            //cout << "Sub : device_index = " << device_index << endl;
            cout << "Sub : port_type = " << port_type << endl;
            cout << "Sub : port_offset = " << port_offset << endl;

            /*int io_type =  (msg.element[i].hash >> 16);
            int io_index = (msg.element[i].hash & 0x00ff);
            cout << "Sub : io_type = " << io_type << endl;
            cout << "Sub : io_index = " << io_index << endl;
            */
            if(!test.decodeMessageType(msg.element[i].data.bytes, msg.element[i].data.size, 
                (void*)&io_value, MessageType_Uint32_fields))
            {
                cout << "Sub : decode io value failed !!" << endl;
                continue;
            }

            cout << "Sub : io_value.data = " << io_value.data << endl;
        }

        usleep(500000);
    }

    return 0;
}