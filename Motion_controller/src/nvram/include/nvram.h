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
//for nvram data structure
#define NVRAM_BLOCK_DATA_SIZE 0x400	// 1K
#define NVRAM_BLOCK_CTRL_SIZE 0x10 // 16B
#define NVRAM_BLOCK_SIZE (NVRAM_BLOCK_DATA_SIZE + NVRAM_BLOCK_CTRL_SIZE)
//for share memory data structure
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
	uint32_t magic_number;   //初始化后设置幻数，非幻数则数据无效
	uint32_t start_address;  //存储数据的起始地址，指向backup_block地址
	uint8_t reserve[];
};

struct NvramBlock                        //存储数据块的结构
{
	uint8_t data[NVRAM_BLOCK_DATA_SIZE]; //存储数据 
	uint8_t ctrl[NVRAM_BLOCK_CTRL_SIZE]; //上述数据的效验结果和区块序号
};

struct NvramImage                        //共享内存的数据结构
{
	pthread_mutex_t mutex;               //在共享内存中创建互斥锁
	pthread_mutexattr_t mutexattr;       //设置上述互斥锁属性为PTHREAD_PROCESS_SHARED
	uint32_t magic_number;               //幻数存在代表数据有效
	uint32_t write_cnt[NVRAM_BLOCK_NUM]; //当某数据块[i]有写入，则write_cnt[i]非零，代表有数据更新
	uint8_t data[NVRAM_IMAGE_DATA_SIZE]; //整片存储数据区
};

}
#endif
