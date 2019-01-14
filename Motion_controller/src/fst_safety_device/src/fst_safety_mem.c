/**********************************************
Copyright 漏 2016 Foresight-Robotics Ltd. All rights reserved.
File:       fst_safety_mem.c
Author:     Feng.Wu 
Create:     20-Sep-2017
Modify:     26-Sep-2018
Summary:    dealing with safety board
**********************************************/

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include "fst_safety_mem.h"

#define SAFETY_LEN	0x80    
#define SAFETY_BASE	0xC00A0000

#define	R_SAFETY_BASE	0x0C	/* the address of date that read from safety. offset with a base 0xC00A0000.*/
#define	W_SAFETY_BASE	0x04	/* the address of date that written to safety. offset with a base 0xC00A000. */
#define STATE_SAFETY_BASE 0x1C    /* the state address of FPGA communication.offset with the base. */

#define RW_MASK	0xFF00
#define R_MASK	0x0100
#define W_MASK	0x0200
#define	FRAME_MASK 0x00FF
#define FIRST_FRAME 0x0001
#define SECOND_FRAME 0x0002

#define STATE_HEARTBEAT_MASK 0x0F
#define STATE_ENABLE_MASK 0x01

static int s_safety_fd;
static void *s_safety_start_ptr;
//static char *s_fake_array[80]; // for fake only
static char s_fake_array[80] = {0}; // for fake only
static int s_safety_virtual;

struct	Safety {
	int	*ptr;
	pthread_mutex_t	mutex;
	char data[8];
};

static struct Safety s_safety_xmit;
static struct Safety s_safety_recv;
static int *s_safety_state_ptr;

unsigned long long int openSafety(void) {
    s_safety_virtual = 0;
    unsigned long long int ret = 0;
    //open physical memory device.
    s_safety_fd = open("/dev/mem", O_RDWR);
    if (s_safety_fd == -1) {
         printf("%s(%d)-%s: Failed to open /dev/mem .\n", __FILE__,__LINE__,__FUNCTION__);
         return ERR_SAFETY_FILE_OPEN;
    }
    else {
         printf("Open /dev/mem OK .\n");
    }
    // map the physical address to virtual address in user space.
    s_safety_start_ptr = mmap(NULL, SAFETY_LEN, PROT_WRITE|PROT_READ,
                               MAP_SHARED, s_safety_fd, SAFETY_BASE);
    if (s_safety_start_ptr == (void *)-1) {
        printf("%s(%d)-%s: Failed to mmap /dev/mem .\n", __FILE__,__LINE__,__FUNCTION__);
        return ERR_SAFETY_FILE_MAP;
    }
    else {
        printf("mmap /dev/mem OK .\n");
    }

    // three operation addresses.
    s_safety_recv.ptr = (int *)(s_safety_start_ptr + R_SAFETY_BASE);  //address to receive data
    s_safety_xmit.ptr = (int *)(s_safety_start_ptr + W_SAFETY_BASE);  //address to transmit data
    s_safety_state_ptr = (int *)(s_safety_start_ptr + STATE_SAFETY_BASE); //address to read the state of FPGA+io_board

	if (pthread_mutex_init(&s_safety_recv.mutex, NULL) != 0) {
        ret = ERR_SAFETY_PTHREAD_INIT;
		printf("Failed to init mutex.\n");
	}
	if (pthread_mutex_init(&s_safety_xmit.mutex, NULL) != 0) {
        ret = ERR_SAFETY_PTHREAD_INIT;
		printf("Failed to init mutex.\n");
	}

    //init the transmit data.
    s_safety_xmit.data[0] = 0;
    s_safety_xmit.data[1] = 0;
    s_safety_xmit.data[2] = 0;
    s_safety_xmit.data[3] = 0;

	return ret;
}

void closeSafety(void) {
	munmap(s_safety_start_ptr, SAFETY_LEN);
	close(s_safety_fd);
}

unsigned long long int getSafety(int *data, int frame) {
	int rw_mask;
	int frame_mask;
    struct Safety *safety;

    //check frame value.
	rw_mask = frame & RW_MASK;//check read or write frame. RW_MASK = 0xFF00.
    frame_mask = frame & FRAME_MASK;//check frame sequence. FRAME_MASK = 0x00FF.
	switch (rw_mask) {
		case R_MASK: // get the reading data.
			safety = &s_safety_recv;
            if (frame_mask > SECOND_FRAME)
                return ERR_SAFETY_FRAME;
			break;
		case W_MASK: //get the writing data.
			safety = &s_safety_xmit;
            if (frame_mask > FIRST_FRAME)
                 return ERR_SAFETY_FRAME;
			break;
		default:
			return ERR_SAFETY_FRAME;
	}

    pthread_mutex_lock(&safety->mutex);
    if (frame_mask == FIRST_FRAME) //get the first frame data.
	  data = (int *)(&safety->data);
	if (frame_mask == SECOND_FRAME)  //get the second frame data.
	  data = (int *)(&safety->data) + 1;
    pthread_mutex_unlock(&safety->mutex);

	return SUCCESS;
}

unsigned long long int setSafety(int data, int frame) {
    int rw_mask;
	int frame_mask;
	char *pdata;

	rw_mask = frame & RW_MASK; //check read or write frame.
	if (rw_mask != W_MASK)
	  return ERR_SAFETY_FRAME;

	frame_mask = frame & FRAME_MASK;  // check frame sequence.
    if (frame_mask > FIRST_FRAME)
      return ERR_SAFETY_FRAME;

	//pdata = (char *)(&data);
	pthread_mutex_lock(&s_safety_xmit.mutex);
	if (frame_mask == FIRST_FRAME){ //set the first frame of data.
        memcpy(s_safety_xmit.data, &data, sizeof(data));
        //s_safety_xmit.data[0] = *(pdata + 0);
        //s_safety_xmit.data[1] = *(pdata + 1);
        //s_safety_xmit.data[2] = *(pdata + 2);
        //s_safety_xmit.data[3] = *(pdata + 3);
	}
	pthread_mutex_unlock(&s_safety_xmit.mutex);

	return SUCCESS;
}

unsigned long long int checkStatus(void)
{
    if (s_safety_virtual == 1)
        return SUCCESS;

    unsigned long long int ret = 0;
    char *pstatus = (char *)s_safety_state_ptr;
    static char pre_heartbeat = 1;// check double times??
    static int double_count_heartbeat = 0; //to check double time.

    //check heartbeat from FPGA-safety
    char heartbeat = *(pstatus + 3);
    heartbeat = heartbeat & STATE_HEARTBEAT_MASK;
    if (heartbeat != pre_heartbeat){ //indicate heartbeat from FPGA running.
        pre_heartbeat = heartbeat;
    }else if (heartbeat == pre_heartbeat){
        //printf("%s(%d)-%s: The forth byte of FPGA heartbeat: 0x%x\n", 
        //       __FILE__,__LINE__,__FUNCTION__, heartbeat);
        double_count_heartbeat++;
        pre_heartbeat = heartbeat;
        if (double_count_heartbeat >= 2){
            double_count_heartbeat = 0;
            return ERR_SAFETY_FPGA_CORE0_NOT_CONNECT;
        }
    }

    //check enable value.
    char enable = *pstatus;
    enable = enable & STATE_ENABLE_MASK; // get enble state= 0x01
    if (enable != 1){
        //printf("%s(%d)-%s: The first byte of FPGA status: 0x%x\n", 
        //        __FILE__,__LINE__,__FUNCTION__,*pstatus);
        return ERR_SAFETY_FPGA_MCU_NOT_CONNECT;
    }

    //check core1_pulse.
    char pulse = *(pstatus + 2);
    char core1_pulse = (pulse >> 3) & 0x01; 
    if(core1_pulse != 1){
        //printf("%s(%d)-%s: The third byte of FPGA status: 0x%x\n", 
        //        __FILE__,__LINE__,__FUNCTION__,*(pstatus+2));
        return ERR_SAFETY_FPGA_CORE1_NOT_CONNECT;
    }
    return ret;
}

unsigned long long int autorunSafetyData() {
    unsigned long long int ret = 0;
    // write down.
    int *pwrite = s_safety_xmit.ptr;
    int *pdata_write = (int *)(&s_safety_xmit.data);
    if (pthread_mutex_lock(&s_safety_xmit.mutex) == -1){
        printf("Failed to lock mutex.\n");
        return ERR_SAFETY_PTHREAD_LOCK;
    }

    *pwrite = *pdata_write;

    if (pthread_mutex_unlock(&s_safety_xmit.mutex) == -1){
        printf("Failed to unlock mutex.\n");
        return ERR_SAFETY_PTHREAD_UNLOCK;
    }
    // read.
    int *pdata_read = (int *)(&s_safety_recv.data);
    int *precv = s_safety_recv.ptr;
    if (pthread_mutex_lock(&s_safety_recv.mutex) == -1){
        printf("Failed to lock mutex.\n");
        return ERR_SAFETY_PTHREAD_LOCK;
    }

    *pdata_read = *precv;
    pdata_read++;
    precv++;
    *pdata_read = *precv;

    if (pthread_mutex_unlock(&s_safety_recv.mutex) == -1){
        printf("Failed to unlock mutex.\n");
        return ERR_SAFETY_PTHREAD_UNLOCK;
    }
    //check FPGA status.
    ret = checkStatus();
    return ret;
}


int fake_init(void) {
    s_safety_virtual = 1;
	int ret = 0;

    s_safety_start_ptr = &s_fake_array;
    s_safety_recv.ptr = (int *)(s_safety_start_ptr + R_SAFETY_BASE);
    s_safety_xmit.ptr = (int *)(s_safety_start_ptr + W_SAFETY_BASE);
    //s_safety_recv.ptr = xmit.ptr; // for fake connection.
    //s_safety_state_ptr = (int *)(s_safety_start_ptr + STATE_SAFETY_BASE);

	if (pthread_mutex_init(&s_safety_recv.mutex, NULL) != 0) {
		ret = -1;
		return ret;
	}

    if (pthread_mutex_init(&s_safety_xmit.mutex, NULL) != 0) {
		ret = -1;
		return ret;
	}

    s_safety_xmit.data[0] = 0;
    s_safety_xmit.data[1] = 0;
    s_safety_xmit.data[2] = 0;
    s_safety_xmit.data[3] = 0;

    return ret;
}


