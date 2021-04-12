/*************************************************************************
	> File Name: nvram.h
	> Author: 
	> Mail: 
	> Created Time: 2020年07月02日 星期四 14时02分08秒
 ************************************************************************/

#ifndef _NVRAM_H
#define _NVRAM_H

#include <stdint.h>
#include <pthread.h>

#define NVRAM_BLOCK_DATA_SIZE 0x400	// 1K
#define NVRAM_BLOCK_CTRL_SIZE 0x10 // 16B
#define NVRAM_BLOCK_SIZE (NVRAM_BLOCK_DATA_SIZE + NVRAM_BLOCK_CTRL_SIZE)

#define NVARM_SIZE 0x20000		// 128K
#define NVRAM_CTRL_SIZE 0x40  	// 64B
#define NVRAM_BACKUP_SIZE NVRAM_BLOCK_SIZE
#define NVRAM_BLOCK_NUM ((NVARM_SIZE - NVRAM_CTRL_SIZE - NVRAM_BACKUP_SIZE) / NVRAM_BLOCK_SIZE)
#define NVRAM_MAGIC_NUMBER (0x5A9E0FB6)

#define NVRAM_IMAGE_DATA_SIZE (NVRAM_BLOCK_DATA_SIZE * NVRAM_BLOCK_NUM)

namespace rtm_nvram
{


struct NvramCtrl
{
	uint32_t magic_number;
	uint32_t start_address;
	uint8_t reserve[];
};

struct NvramBlock
{
	uint8_t data[NVRAM_BLOCK_DATA_SIZE];
	uint8_t ctrl[NVRAM_BLOCK_CTRL_SIZE];
};

struct NvramImage
{
	pthread_mutex_t mutex;
	pthread_mutexattr_t mutexattr;
	uint32_t magic_number;
	uint32_t write_cnt[NVRAM_BLOCK_NUM];
	uint8_t data[NVRAM_IMAGE_DATA_SIZE];
};

}
#endif
