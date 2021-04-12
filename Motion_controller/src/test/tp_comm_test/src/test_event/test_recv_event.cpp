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


#define MAX_EVENT_BUFFER_SIZE     (65535)

using namespace std;

int main()
{
    TpCommTest test;
    if (!test.initEventSocket())
    {
        cout << "Event : socket init failed" << endl;
        return -1;
    }

    uint8_t buf[MAX_EVENT_BUFFER_SIZE];
    int buf_size = MAX_EVENT_BUFFER_SIZE;

	while (!test.recvEventBuf(buf, buf_size))
	{
	    usleep(10*1000);
	}

	EventMessageType_Uint64 msg;

	if (!test.decodeEventMessageType(EventMessageType_Uint64_fields, buf, buf_size, (void*)&msg))
    {
        cout << "event: recv msg decode failed" << endl;
        return -1;
    }

    std::cout<<"event : msg.header.time_stamp = "<<msg.header.time_stamp<<std::endl;
	std::cout<<"event : msg.data.data = "<<std::hex<<msg.data.data<<std::endl;

	usleep(200000);

	return 0;

}
