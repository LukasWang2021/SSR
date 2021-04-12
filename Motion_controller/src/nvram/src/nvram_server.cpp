/*************************************************************************
	> File Name: nvram_server.cpp
	> Author: 
	> Mail: 
	> Created Time: 2020年07月02日 星期四 14时26分14秒
 ************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <fstream>
#include "thread_help.h"
#include "rtm_spi.h"
#include "nvram.h"
#include "log_manager_producer.h"

#define NVRAM_INSTRUCTION_READ 	0x03
#define NVRAM_INSTRUCTION_WRITE 0x02
#define NVRAM_RW_BUFFER_SIZE (2 * 1024)

using namespace std;
using namespace log_space;
using namespace rtm_spi;
using namespace rtm_nvram;

bool g_running;

uint32_t g_data_block_address;
uint32_t g_backup_block_address;

uint8_t g_read_buffer[NVRAM_RW_BUFFER_SIZE];
uint8_t g_write_buffer[NVRAM_RW_BUFFER_SIZE];

NvramImage *g_nvram_image;

bool readNvram(uint32_t address, uint8_t *data, uint32_t length)
{
	if (length + 4 > NVRAM_RW_BUFFER_SIZE || address + length > NVARM_SIZE)
	{
		LogProducer::error("nvramServer", "Read nvram error, address: 0x%x, length: %d", address, length);
		return false;
	}

	g_write_buffer[0] = NVRAM_INSTRUCTION_READ;
	g_write_buffer[1] = (uint8_t)(address >> 16);
	g_write_buffer[2] = (uint8_t)(address >> 8);
	g_write_buffer[3] = (uint8_t)(address);
	transferData(g_write_buffer, g_read_buffer, length + 4);
	/*
	printf("readNvram w: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
		g_write_buffer[0], g_write_buffer[1], g_write_buffer[2], g_write_buffer[3],
		g_write_buffer[4], g_write_buffer[5], g_write_buffer[6], g_write_buffer[7],
		g_write_buffer[8], g_write_buffer[9], g_write_buffer[10], g_write_buffer[11],
		g_write_buffer[12], g_write_buffer[3], g_write_buffer[14], g_write_buffer[15]);
	printf("readNvram r: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
		g_read_buffer[0], g_read_buffer[1], g_read_buffer[2], g_read_buffer[3],
		g_read_buffer[4], g_read_buffer[5], g_read_buffer[6], g_read_buffer[7],
		g_read_buffer[8], g_read_buffer[9], g_read_buffer[10], g_read_buffer[11],
		g_read_buffer[12], g_read_buffer[3], g_read_buffer[14], g_read_buffer[15]);
	*/
	memcpy(data, &g_read_buffer[4], length);
	return true;
}

bool writeNvram(uint32_t address, uint8_t *data, uint32_t length)
{
	if (length + 4 > NVRAM_RW_BUFFER_SIZE || address + length > NVARM_SIZE)
	{
		LogProducer::error("nvramServer", "Write nvram error, address: 0x%x, length: %d", address, length);
		return false;
	}

	g_write_buffer[0] = NVRAM_INSTRUCTION_WRITE;
	g_write_buffer[1] = (uint8_t)(address >> 16);
	g_write_buffer[2] = (uint8_t)(address >> 8);
	g_write_buffer[3] = (uint8_t)(address);
	memcpy(&g_write_buffer[4], data, length);
	/*
	printf("writeNvram: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
		g_write_buffer[0], g_write_buffer[1], g_write_buffer[2], g_write_buffer[3],
		g_write_buffer[4], g_write_buffer[5], g_write_buffer[6], g_write_buffer[7],
		g_write_buffer[8], g_write_buffer[9], g_write_buffer[10], g_write_buffer[11],
		g_write_buffer[12], g_write_buffer[3], g_write_buffer[14], g_write_buffer[15]);
	*/
	transferData(g_write_buffer, g_read_buffer, length + 4);
	return true;
}

int createNvramImage(void)
{
	LogProducer::info("nvramServer", "Prepare share memory ...");
    int fd = shm_open("rtm_nvram", O_CREAT|O_RDWR, 00777);
    
    if (-1 == fd)
	{
        LogProducer::error("nvramServer", "Fail to opening rtm_nvram");
        return -1;
    }

    int lock = flock(fd, LOCK_EX | LOCK_NB);
    
    if (lock == -1)
	{
        LogProducer::error("nvramServer", "Fail to take over the 'rtm_nvram', it has controlled by another server");
        return -2;
    }

    ftruncate(fd, sizeof(NvramImage));
    char *ptr = (char*) mmap(NULL, sizeof(NvramImage), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    
    if (ptr == MAP_FAILED)
	{
        close(fd);
        LogProducer::error("nvramServer", "Fail to mapping rtm_nvram");
        return -3;
    }

    memset(ptr, 0, sizeof(NvramImage));
	g_nvram_image = (NvramImage*)ptr;
	pthread_mutexattr_init(&g_nvram_image->mutexattr);
	pthread_mutexattr_setpshared(&g_nvram_image->mutexattr, PTHREAD_PROCESS_SHARED);
	pthread_mutex_init(&g_nvram_image->mutex, &g_nvram_image->mutexattr);
    LogProducer::info("nvramServer", "createNvramImage Success!");
    return 0;
}

void uploadNvramToImage(void)
{
	NvramBlock nvram_block;
	uint32_t nvram_address = g_data_block_address;
	uint32_t image_address = 0;
	LogProducer::info("nvramServer", "Upload nvram data to image ...");
	pthread_mutex_lock(&g_nvram_image->mutex);

	for (uint32_t i = 0; i < NVRAM_BLOCK_NUM; i++)
	{
		readNvram(nvram_address, (uint8_t*)&nvram_block, sizeof(nvram_block));
		memcpy(g_nvram_image->data + image_address, nvram_block.data, NVRAM_BLOCK_DATA_SIZE);
		nvram_address += NVRAM_BLOCK_SIZE;
		image_address += NVRAM_BLOCK_DATA_SIZE;
		g_nvram_image->write_cnt[i] = 0;
	}

	g_nvram_image->magic_number = NVRAM_MAGIC_NUMBER;
	pthread_mutex_unlock(&g_nvram_image->mutex);
	LogProducer::info("nvramServer", "Upload nvram data to image success, image start working");
}

uint8_t checkSumData(uint8_t *data, uint32_t length)
{
	uint8_t check_sum = 0;

	for (uint32_t i = 0; i < length; i++)
	{
		check_sum ^= data[i];
	}

	return check_sum;
}

bool checkNvramData(void)
{
	NvramCtrl nvram_ctrl;
	NvramBlock nvram_block, backup_block;
	// 检查Nvram中的幻数是否正确，幻数不正确表示Nvram中的数据不可用
	readNvram(0, (uint8_t*)&nvram_ctrl, sizeof(NvramCtrl));
	LogProducer::info("nvramServer", "Check nvram data ...");

	if (nvram_ctrl.magic_number != NVRAM_MAGIC_NUMBER)
	{
		LogProducer::error("nvramServer", "Magic number 0x%x is not correct", nvram_ctrl.magic_number);
		return false;
	}

	uint32_t backup_block_index = 0xFFFFFFFF;
	uint32_t address = nvram_ctrl.start_address;
	LogProducer::info("nvramServer", "Nvram data start at 0x%x", nvram_ctrl.start_address);
	// 检查备份block中的数据是否可用，如果备份中的block有效，可用于恢复指定的数据block
	readNvram(address, (uint8_t*)&backup_block, sizeof(backup_block));

	if (checkSumData(backup_block.data, sizeof(backup_block.data)) == backup_block.ctrl[0])
	{
		backup_block_index = *(uint32_t*)(backup_block.ctrl + 4);
		LogProducer::info("nvramServer", "Backup block valid, backup of block %d", backup_block_index);
	}

	// 以此检查每个数据block是否可用
	for (uint32_t i = 0; i < NVRAM_BLOCK_NUM; i++)
	{
		address += NVRAM_BLOCK_SIZE;
		readNvram(address, (uint8_t*)&nvram_block, sizeof(nvram_block));

		if (checkSumData(nvram_block.data, sizeof(nvram_block.data)) != nvram_block.ctrl[0])
		{
			if (backup_block_index == *(uint32_t*)(nvram_block.ctrl + 4))
			{
				LogProducer::info("nvramServer", "Block %d invalid, recovery from backup block", *(uint32_t*)(nvram_block.ctrl + 4));
				writeNvram(address, (uint8_t*)&backup_block, sizeof(backup_block));
			}
			else
			{
				LogProducer::error("nvramServer", "Block %d invalid", *(uint32_t*)(nvram_block.ctrl + 4));
				return false;
			}
		}
	}

	g_backup_block_address = nvram_ctrl.start_address;
	g_data_block_address = nvram_ctrl.start_address + NVRAM_BLOCK_SIZE;
	LogProducer::info("nvramServer", "Nvram data check passed");
	return true;
}

void initNvramData(void)
{
	uint8_t buffer[NVRAM_CTRL_SIZE];
	NvramCtrl *nvram_ctrl = (NvramCtrl*)buffer;
	NvramBlock nvram_block;
	uint32_t address = 0;
	LogProducer::info("nvramServer", "Init nvram ctrl block");
	memset(buffer, 0, sizeof(buffer));
	nvram_ctrl->magic_number = NVRAM_MAGIC_NUMBER;
	nvram_ctrl->start_address = address + NVRAM_CTRL_SIZE;
	writeNvram(address, buffer, sizeof(buffer));
	LogProducer::info("nvramServer", "Init nvram backup block");
	memset(nvram_block.data, 0, sizeof(nvram_block.data));
	nvram_block.ctrl[0] = checkSumData(nvram_block.data, sizeof(nvram_block.data));
	*(uint32_t*)(nvram_block.ctrl + 4) = 0xFFFFFFFF;
	address = nvram_ctrl->start_address;
	writeNvram(address, (uint8_t*)&nvram_block, sizeof(nvram_block));
	LogProducer::info("nvramServer", "Init nvram data block");

	for (uint32_t i = 0; i < NVRAM_BLOCK_NUM; i++)
	{
		address += NVRAM_BLOCK_SIZE;
		*(uint32_t*)(nvram_block.ctrl + 4) = i;
		writeNvram(address, (uint8_t*)&nvram_block, sizeof(nvram_block));
	}

	g_backup_block_address = nvram_ctrl->start_address;
	g_data_block_address = nvram_ctrl->start_address + NVRAM_BLOCK_SIZE;
	LogProducer::info("nvramServer", "Init nvram success");
}

static void sigintHandle(int num)
{
    LogProducer::warn("nvramServer", "Interrupt request catched.");
	usleep(500 * 1000);
    g_running = false;
}

static void* server_func(void*)
{
	LogProducer::info("nvramServer", "Server thread start");
	NvramBlock block;
	uint32_t address;

	while (g_running)
	{
		for (uint32_t i = 0; i < NVRAM_BLOCK_NUM; i++)
		{
			if (g_nvram_image->write_cnt[i] == 0)
			{
				// 这个block没有被更新过，不需要同步Nvram，直接跳过
				continue;
			}

			//LogProducer::info("nvramServer", "Block %d write cnt %d, download this block to nvram ...", i, g_nvram_image->write_cnt[i]);
			pthread_mutex_lock(&g_nvram_image->mutex);
			memcpy(block.data, g_nvram_image->data + NVRAM_BLOCK_DATA_SIZE * i, NVRAM_BLOCK_DATA_SIZE);
			g_nvram_image->write_cnt[i] = 0;
			pthread_mutex_unlock(&g_nvram_image->mutex);
			block.ctrl[0] = checkSumData(block.data, sizeof(block.data));
			*(uint32_t*)(block.ctrl + 4) = i;
			// 先写入backup block，之后再写入data block
			address = g_backup_block_address;
			writeNvram(address, (uint8_t*)&block, sizeof(block));
			usleep(1000);
			address = g_data_block_address + NVRAM_BLOCK_SIZE * i;
			writeNvram(address, (uint8_t*)&block, sizeof(block));
			usleep(1000);
			//LogProducer::info("nvramServer", "Download success");
		}

		usleep(100 * 1000);
	}

	LogProducer::info("nvramServer", "Server thread exit");
	return NULL;
}

int main(int argc, char **argv)
{
	log_space::LogProducer log_manager;
	uint32_t fake_isr = 0;
    log_manager.init("nvram_server", &fake_isr);

	int res = initSpi();

	if (res != 0)
	{
		LogProducer::error("nvramServer", "Fail to init SPI, code = %d", res);
		return res;
	}

	if (!checkNvramData())
	{
		initNvramData();
	}

	res = createNvramImage();

	if (res != 0)
	{
		LogProducer::error("nvramServer", "Fail to create nvram image, code = %d", res);
		return res;
	}

	uploadNvramToImage();
	g_running = true;

	base_space::ThreadHelp server_thread;
	server_thread.run(server_func, NULL, 20);
    signal(SIGINT, sigintHandle);
	server_thread.join();
    return 0;
}



