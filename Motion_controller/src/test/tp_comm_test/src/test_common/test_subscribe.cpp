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

    int i = 0;

    for (;;) 
    {
        cout << "===================  sub test " << i << "==================" << endl;
        i++;
        uint8_t sub_buf[2048];
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

        cout << "Sub : recv topic hash = " << std::hex << recv_hash << std::dec << endl;
        cout << "Sub : recv time_stamp = " << msg.time_stamp << endl;
        cout << "Sub : recv element_count = " << msg.element_count << endl;

        for (unsigned int i = 0 ; i != msg.element_count; ++i)
        {
            switch(msg.element[i].hash)
            {
                case 0x0001715B://"/publish/axes_feedback"
                {
                    MessageType_AxisFeedbackList fdb;
                    if(!test.decodeMessageType(msg.element[i].data.bytes, msg.element[i].data.size, 
                        (void*)&fdb, MessageType_AxisFeedbackList_fields))
                    {
                        printf("Sub : parse data msg failed !!\n");
                    }
                    else 
                    {
                        printf("Sub : /publish/axes_feedback, hash = 0x%x\n", msg.element[i].hash);
                        for (unsigned int i = 0; i < fdb.data_count; ++i)
                        {
                            printf("axis[%d], isr=%u, state=%u, pos=%lf, vel=%lf, torque=%lf\n", i, fdb.data[i].data1.data[0], fdb.data[i].data1.data[1],
                                fdb.data[i].data2.data[0], fdb.data[i].data2.data[1], fdb.data[i].data2.data[2]);
                            //fflush(stdout);
                        }
                    }
                }
                    break;
                case 0x0001128B://"/publish/servo1001/servos_feedback"
                {
                    MessageType_ServoFeedbackList fdb;
                    if(!test.decodeMessageType(msg.element[i].data.bytes, msg.element[i].data.size, 
                        (void*)&fdb, MessageType_ServoFeedbackList_fields))
                    {
                        printf("Sub : parse data msg failed !!\n");
                    }
                    else 
                    {
                        printf("Sub : /publish/servo1001/servos_feedback, hash = 0x%x, count=%d\n", msg.element[i].hash, fdb.data_count);
                        for (unsigned int i = 0; i < fdb.data_count; ++i)
                        {
                            printf("servo[%d] count=%d: ", i, fdb.data[i].data_count);
                            for (unsigned int j = 0; j < fdb.data[i].data_count; ++j)
                            {
                                printf("%d, ", fdb.data[i].data[j]);
                            }
                            printf("\n");
                        }
                    }
                }
                    break;
                case 0x00012FFB://"/publish/servo1001/cpu_feedback"
                {
                    MessageType_Uint32List fdb;
                    if(!test.decodeMessageType(msg.element[i].data.bytes, msg.element[i].data.size, 
                        (void*)&fdb, MessageType_Uint32List_fields))
                    {
                        printf("Sub : parse data msg failed !!\n");
                    }
                    else 
                    {
                        printf("Sub : /publish/servo1001/cpu_feedback, hash = 0x%x, count=%d\n", msg.element[i].hash, fdb.data_count);
                        for (size_t i = 0; i < fdb.data_count; ++i)
                        {
                            printf("%d, ", fdb.data[i]);
                        }
                        printf("\n");
                        //printf("ctrl_sync={%d,%d,%d,%d,%d,%d,%d,%d}, sampling_sync=%d\n", 
                        //    fdb.data[0], fdb.data[1], fdb.data[2], fdb.data[3], fdb.data[4], fdb.data[5], fdb.data[6], fdb.data[7], fdb.data[8]);
                    }
                }
                    break;
                
                case 0x00013C8B://"/publish/io1000/io_feedback"
                {
                    MessageType_Uint32List fdb;
                    if(!test.decodeMessageType(msg.element[i].data.bytes, msg.element[i].data.size, 
                        (void*)&fdb, MessageType_Uint32List_fields))
                    {
                        printf("Sub : parse data msg failed !!\n");
                    }
                    else 
                    {
                        printf("Sub : /publish/io1000/io_feedback, hash = 0x%x, count=%d\n", msg.element[i].hash, fdb.data_count);
                        for (size_t i = 0; i < fdb.data_count; ++i)
                        {
                            printf("%d, ", fdb.data[i]);
                        }
                        printf("\n");
                    }
                    break;
                }

                case 0x0001472B://"/publish/iosafety/safety_feedback"
                {
                    MessageType_Uint32List fdb;
                    if(!test.decodeMessageType(msg.element[i].data.bytes, msg.element[i].data.size, 
                        (void*)&fdb, MessageType_Uint32List_fields))
                    {
                        printf("Sub : parse data msg failed !!\n");
                    }
                    else 
                    {
                        printf("Sub : /publish/iosafety/safety_feedback, hash = 0x%x, count=%d\n", msg.element[i].hash, fdb.data_count);
                        for (size_t i = 0; i < fdb.data_count; ++i)
                        {
                            printf("%d, ", fdb.data[i]);
                        }
                        printf("\n");
                    }
                    break;
                }
                
                case 0x0000AEAB://"/publish/torque_feedback",
                {
                    MessageType_DoubleList fdb;
                    if(!test.decodeMessageType(msg.element[i].data.bytes, msg.element[i].data.size, 
                        (void*)&fdb, MessageType_DoubleList_fields))
                    {
                        printf("Sub : parse data msg failed !!\n");
                    }
                    else 
                    {
                        printf("Sub : /publish/torque_feedback = 0x%x, count=%d\n", msg.element[i].hash, fdb.data_count);
                        for (size_t i = 0; i < fdb.data_count; ++i)
                        {
                            printf("%lf, ", fdb.data[i]);
                        }
                        printf("\n");
                    }
                    break;
                }

                default:
                {
                    printf("unknow hash\n");
                    break;
                }
            };
        }
        printf("\n");
        usleep(90000);
    }
    return 0;
}
