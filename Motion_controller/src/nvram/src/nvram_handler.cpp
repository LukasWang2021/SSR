/*************************************************************************
	> File Name: nvram_handler.cpp
	> Author: 
	> Mail: 
	> Created Time: 2020年07月03日 星期五 13时41分38秒
 ************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <iostream>
#include "nvram_handler.h"

using namespace std;

namespace rtm_nvram
{

NvramHandler::NvramHandler()
{
	image_ptr_ = NULL;
}

NvramHandler::~NvramHandler()
{
	image_ptr_ = NULL;
}

bool NvramHandler::init(void)
{
    int fd = shm_open("rtm_nvram", O_CREAT|O_RDWR, 00777);
    
    if (-1 == fd)
	{
		close(fd);
        return false;
    }
	ftruncate(fd, sizeof(NvramImage));//necessary if using O_CREAT.
    void *ptr = (char*)mmap(NULL, sizeof(NvramImage), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    
    if (ptr == MAP_FAILED)
	{
        close(fd);
        return false;
    }
	close(fd);
	memset(ptr, 0, sizeof(NvramImage));

	image_ptr_ = (NvramImage*)ptr;
	image_ptr_->magic_number = NVRAM_MAGIC_NUMBER;

    return true;
}

bool NvramHandler::readNvram(uint32_t address, uint8_t *data, uint32_t length)
{
	if (image_ptr_->magic_number != NVRAM_MAGIC_NUMBER) return false;
	if (address + length > NVRAM_IMAGE_DATA_SIZE) return false;
	pthread_mutex_lock(&image_ptr_->mutex);
	memcpy(data, image_ptr_->data + address, length);
	pthread_mutex_unlock(&image_ptr_->mutex);
	return true;
}

bool NvramHandler::writeNvram(uint32_t address, const uint8_t *data, uint32_t length)
{
	if (image_ptr_->magic_number != NVRAM_MAGIC_NUMBER) return false;
	if (address + length > NVRAM_IMAGE_DATA_SIZE) return false;
	uint32_t block_index = address >> 10;
	uint32_t block_last = ((block_index + 1) << 10) - address;
	uint32_t write_size = length <= block_last ? length : block_last;
	pthread_mutex_lock(&image_ptr_->mutex);

	while (write_size > 0)
	{
		memcpy(image_ptr_->data + address, data, write_size);
		image_ptr_->write_cnt[block_index]++;
		block_index++;
		data += write_size;
		length -= write_size;
		address += write_size;
		write_size = length <= NVRAM_BLOCK_DATA_SIZE ? length : NVRAM_BLOCK_DATA_SIZE;
	}

	pthread_mutex_unlock(&image_ptr_->mutex);
	return true;
}

}

