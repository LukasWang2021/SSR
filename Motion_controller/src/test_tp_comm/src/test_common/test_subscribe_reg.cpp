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

    unsigned int hash_value = 0x12341234;
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
        printf("Sub : recv time_stamp = %d\n", msg.time_stamp);
        printf("Sub : recv element_count = %d\n", msg.element_count);

        MessageType_PrValue pr_value;
        MessageType_SrValue sr_value;
        MessageType_HrValue hr_value;
        MessageType_MrValue mr_value;
        MessageType_RValue r_value;

        for (int i = 0 ; i != msg.element_count; ++i)
        {
            int reg_type =  (msg.element[i].hash >> 16);
            int reg_index = (msg.element[i].hash & 0x00ff);
            cout << "Sub : reg type = " << reg_type << endl;
            cout << "Sub : reg index = " << reg_index << endl;
            switch (reg_type)
            {
                case MessageType_RegType_PR:
                {
                    if(!test.decodeMessageType(msg.element[i].data.bytes, msg.element[i].data.size, 
                        (void*)&pr_value, MessageType_PrValue_fields))
                    {
                        cout << "Sub : decode pr value failed !!" << endl;
                    }
                    else
                    {
                        cout << "Sub : decode pr value success!" << endl;
                        cout << "Reply : pr_value.is_valid = " << pr_value.is_valid << endl;
                        cout << "Reply : pr_value.group_id = " << pr_value.group_id << endl;
                        cout << "Reply : pr_value.pos_type = " << pr_value.pos_type << endl;
                        cout << "Reply : pr_value.pos.data_count = " << pr_value.pos.data_count << endl;
                        cout << "Reply : pr_value.posture.data_count = " << pr_value.posture.data_count << endl;
                    
                        for (int i = 0; i != pr_value.pos.data_count; ++i)
                        {
                            cout << "Reply : pr_value..pos.data[" << i << "] = " << pr_value.pos.data[i] << endl;
                        }
                    
                        for (int i = 0; i != pr_value.posture.data_count; ++i)
                        {
                            cout << "Reply : pr_value.posture.data[" << i << "] = " << pr_value.posture.data[i] << endl;
                        }
                    }
                }
                    break;
                case MessageType_RegType_SR:
                {
                    if(!test.decodeMessageType(msg.element[i].data.bytes, msg.element[i].data.size, 
                        (void*)&sr_value, MessageType_SrValue_fields))
                    {
                        cout << "Sub : decode sr value failed !!" << endl;
                    }
                    else
                    {
                        cout << "Sub : decode sr value success!" << endl;
                        cout << "Reply : sr_value.is_valid = " << sr_value.is_valid << endl;
                        printf("Reply : sr_value.value = %s\n", sr_value.data);
                        //cout << "Reply : sr_value.value = " << sr_value.data << endl;
                    }
                }
                    break;
                case MessageType_RegType_HR:
                {
                    if(!test.decodeMessageType(msg.element[i].data.bytes, msg.element[i].data.size, 
                        (void*)&hr_value, MessageType_HrValue_fields))
                    {
                        cout << "Sub : decode hr value failed !!" << endl;
                    }
                    else
                    {
                        cout << "Sub : decode hr value success!" << endl;
                        cout << "Reply : hr_value.is_valid = " << hr_value.is_valid << endl;
                        cout << "Reply : hr_value.group_id = " << hr_value.group_id << endl;
                        cout << "Reply : hr_value.joints.data_count = " <<hr_value.joint_pos.data_count << endl;
                        cout << "Reply : hr_value.diffs.data_count = " << hr_value.diff_pos.data_count << endl;
                    
                        for (int i = 0; i != hr_value.joint_pos.data_count; ++i)
                        {
                            cout << "Reply : hr_value.joint_pos.data[" << i << "] = " << hr_value.joint_pos.data[i] << endl;
                        }
                    
                        for (int i = 0; i != hr_value.diff_pos.data_count; ++i)
                        {
                            cout << "Reply : hr_value.diff_pos.data[" << i << "] = " << hr_value.diff_pos.data[i] << endl;
                        }
                    }
                }
                    break;
                case MessageType_RegType_MR:
                {
                    if(!test.decodeMessageType(msg.element[i].data.bytes, msg.element[i].data.size, 
                        (void*)&mr_value, MessageType_MrValue_fields))
                    {
                        cout << "Sub : decode mr data failed !!" << endl;
                    }
                    else
                    {
                        cout << "Sub : decode mr data success!" << endl;
                        cout << "Reply : mr_value.is_valid = " << mr_value.is_valid << endl;
                        cout << "Reply : mr_data.value = " << mr_value.data << endl;
                    }
                }
                    break;
                case MessageType_RegType_R:
                {
                    if(!test.decodeMessageType(msg.element[i].data.bytes, msg.element[i].data.size, 
                        (void*)&r_value, MessageType_RValue_fields))
                    {
                        cout << "Sub : decode r value failed !!" << endl;
                    }
                    else
                    {
                        cout << "Sub : decode r value success!" << endl;
                        cout << "Reply : r_value.is_valid = " << r_value.is_valid << endl;
                        cout << "Reply : r_value.value = " << r_value.data << endl;
                    }
                }
                    break;
                default:
                    cout << "Sub : reg type error!" << endl;
            }
        }

        usleep(500000);
    }
    return 0;
}