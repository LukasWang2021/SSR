#pragma once
#include <stdint.h>


#define COMM_INTERFACE_API __declspec(dllexport)
#define HASH_BYTE_SIZE	4
#define COMM_BUFFER_SIZE 65535	//byte size
#define TOPIC_HASH	0x12345678
#define MAX_EVENT_NUMBER 8
#define AXIS_NUM 14
#define GROUP_NUM 0
#define TOPIC_ELEM_NUM 3
typedef uint64_t ErrorCode;