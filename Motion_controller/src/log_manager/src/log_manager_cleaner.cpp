#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>  
#include <string.h>
#include <iostream>
#include <sys/time.h>
#include <sys/mman.h>  
#include <fcntl.h>  
#include <sys/stat.h> 
#include <signal.h>
#include <dirent.h>
#include <limits.h>
#include <getopt.h>
#include "log_manager_comsumer.h"

using namespace log_space;


int main()
{
    // point to shared memory
    int fd = open(LOG_SHMEM_NAME, O_RDWR);
    uint8_t* shmem_ptr_ = (uint8_t *)mmap(NULL, LOG_SHMEM_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, LOG_SHMEM_ADDRESS);
    if (shmem_ptr_ == MAP_FAILED)
	{
		close(fd);
		printf("Failed to map share memory\n");
		return -1;
	}
	close(fd);


	//  clean memory occupied value

    uint32_t block_step_ = LOG_CTRL_AREA_SIZE + LOG_ITEM_AREA_SIZE * LOG_BLOCK_TEXT_ITEM;
	LogControlArea * ctrl_area_ptr = (LogControlArea *)(shmem_ptr_);
	for (size_t i = 0; i < LOG_BLOCK_NUMBER; ++i)
	{
            ctrl_area_ptr->shm_occupied = 0;
            ctrl_area_ptr = (LogControlArea *)((uint8_t *)ctrl_area_ptr + block_step_);
    }

    // delete pointer
    int test = munmap(shmem_ptr_, LOG_SHMEM_SIZE);
	if(test != 0)
	{
		printf("ERROR ----- Log_Shared_Memory cannot be released ----- ERROR\n");
	}
	printf("SUCCESS ----- Log_Shared_Memory has been released ----- SUCCESS\n");
    return 0;
}