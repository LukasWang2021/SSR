/**********************************************
File: middleware_to_sharedmem.h
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
Instruction: Main algorithm to operate on RAM
Author: Feng.Wu 16-Aug-2016
Modifier:
**********************************************/
#ifndef MIDDLEWARE_TO_SHAREDMEM_H_
#define MIDDLEWARE_TO_SHAREDMEM_H_

#include "middleware_to_sharedmem_ptr.h"

#define MEM_FALSE 0
#define MEM_TRUE 1
#define MEM_WRITE 0
#define MEM_READ 1
#define MEM_WRITE_TURN 11
#define MEM_READ_TURN 12
#define MEM_READ_ALREADY 0
#define MEM_WRITE_ALREADY 1

#ifdef __cplusplus
	extern "C"{
#endif

int searchIndex(const int handle, const char *name);

int readWriteSharedMemByIndex(const int handle, void *structure, int index, int access_type);

int readWriteSharedMem(const int handle, void *structure, const char *name, int access_type);

#ifdef __cplusplus
}
#endif

#endif //MIDDLEWARE_TO_SHAREDMEM_H_
