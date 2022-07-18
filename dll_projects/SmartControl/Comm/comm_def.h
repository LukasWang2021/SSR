#pragma once
#include <stdint.h>
#ifdef _WIN_PLAT
#include <windows.h>
#else
#include <unistd.h>
#endif

#ifdef _WIN_PLAT
#define COMM_INTERFACE_API __declspec(dllexport)
#else
#define COMM_INTERFACE_API
#endif

#define HASH_BYTE_SIZE	4
#define COMM_BUFFER_SIZE 65535	//byte size
#define TOPIC_HASH	0x12345678
#define MAX_EVENT_NUMBER 8
#define AXIS_NUM 16
#define GROUP_NUM 1
#define TOPIC_ELEM_NUM 6
typedef uint64_t ErrorCode;