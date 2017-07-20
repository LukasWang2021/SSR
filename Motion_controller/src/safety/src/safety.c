#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include "safety/safety.h"

#define LEN	0X40
// #define SAFETY_MAX_FRAME 4
#define SAFETY_BASE		0xC1013000
// #define R_SAFETY_BASE	0X10
// #define W_SAFETY_BASE	0X14

// #define FIRST_FRAME		3
// #define	SECOND_FRAME	2
// #define	THIRD_FRAME		1
// #define	FOURTH_FRAME	0

// #define SAFETY_CMD	0XF5

#define RW_MASK	0XFF00
#define R_MASK	0X0100
#define W_MASK	0X0200
#define	FRAME_MASK 0X00FF

// #define ERR_SAFETY_FILE_OPEN (unsigned long long int)0x00010002008E0001   /*can't open file when initializing safety board*/
// #define ERR_SAFETY_FILE_MAP (unsigned long long int)0x00010002008E0002   /*Mapping is failed.*/
// #define ERR_SAFETY_RECV_CMD (unsigned long long int)0x00010002008E0015   /*Command is not received from the safety board*/
// #define ERR_SAFETY_NOT_CONNECT (unsigned long long int)0x00010002008E0016   /*Safety board is not connected with, and there is not heartbeat from the safety board.*/
// #define ERR_SAFETY_PTHREAD_INIT (unsigned long long int)0x00000001008E001F   /*Mutex initialization is failed.*/
// #define ERR_SAFETY_PTHREAD_LOCK (unsigned long long int)0x00000001008E0020   /*Mutex lock is failed*/
// #define ERR_SAFETY_PTHREAD_UNLOCK (unsigned long long int)0x00000001008E0021   /*Mutex unlock is failed*/
// #define ERR_SAFETY_FRAME (unsigned long long int)0x00000001008E0029   /*The frame is out of range.*/

static int fd;
static void *ptr;

struct	Safety {
	int	fd;
	int	*ptr;
	pthread_mutex_t	mutex;
	char	data[SAFETY_MAX_FRAME];
	char heartbeat;
	char count;
};


static struct Safety	xmit;
static struct Safety	recv;

unsigned long long int initSafety() {
	int ret = 0;
	fd = open("/dev/mem", O_RDWR);
	if (fd == -1) {
		return ERR_SAFETY_FILE_OPEN;
	}
	
	ptr = mmap(NULL, LEN, PROT_WRITE|PROT_READ, MAP_SHARED, fd, SAFETY_BASE);
	if (ptr == (void *)-1) {
		return ERR_SAFETY_FILE_MAP;
	}

	recv.ptr = (int *)(ptr + R_SAFETY_BASE);
	xmit.ptr = (int *)(ptr + W_SAFETY_BASE);

	if (pthread_mutex_init(&recv.mutex, NULL) != 0) {
		printf("Failed to init mutex.\n");
	}
	if (pthread_mutex_init(&xmit.mutex, NULL) != 0) {
		printf("Failed to init mutex.\n");
	}

	xmit.data[FIRST_FRAME] = (char)SAFETY_CMD;
	xmit.data[SECOND_FRAME] = 0;
	xmit.data[THIRD_FRAME] = 0;
	xmit.data[FOURTH_FRAME] = 0;
	xmit.data[FIFTH_FRAME] = 0;
	xmit.heartbeat = 0;
	xmit.count = 0x9;

	recv.heartbeat = 0;
	recv.count = 0;

	return (unsigned long long int)ret;
}

unsigned long long int openSafety() {
	return initSafety();
}

void closeSafety() {
	munmap(ptr, LEN);
	close(fd);
}

int getSafety(int frame, unsigned long long int *err) {
	int mask;
	int frame_mask;
    struct Safety	*safety;
	int *pdata;
	char ret;

	*err = 0;

    
	mask = frame & RW_MASK;
	switch (mask) {
		case R_MASK:
			safety = &recv;
			break;
		case W_MASK:
			safety = &xmit;
			break;
		default:
			*err = ERR_SAFETY_FRAME; 
			return 0;
	}

	frame_mask = frame & FRAME_MASK;
/*	if ((frame_mask >= 1) & (frame_mask <= R_SAFETY_MAX_FRAME)) {
		pthread_mutex_lock(&safety->mutex);
		frame_mask = frame_mask - 1;
		ret = safety->data[frame_mask];
		pthread_mutex_unlock(&safety->mutex);
		return ret;
	}
	else {
		*err = ERR_SAFETY_FRAME;
		return 0;
	}
*/
	pthread_mutex_lock(&safety->mutex); 
	if (frame_mask == 1)
	  pdata = (int *)(&safety->data);
	if (frame_mask == 2)
	  pdata = (int *)(&safety->data) + 1;
	pthread_mutex_unlock(&safety->mutex);

	return *pdata;
}

unsigned long long int setSafety(int data, int frame) {
    unsigned long long int ret = 0;
	int frame_mask;
	int mask;
	char *pdata;

	mask = frame & RW_MASK;
	if (mask != W_MASK)
	  return ERR_SAFETY_FRAME;

	frame_mask = frame & FRAME_MASK;
/*	if ((frame_mask >= 1) & (frame_mask <= W_SAFETY_MAX_FRAME)) {
		pthread_mutex_lock(&xmit.mutex);
		frame_mask = frame_mask - 1;
		xmit.data[frame_mask] = data;
		pthread_mutex_unlock(&xmit.mutex);
	}
	else
	  ret = ERR_SAFETY_FRAME;
*/

	pdata = (char *)(&data);
	pthread_mutex_lock(&xmit.mutex);
	if (frame_mask == 1) {
		xmit.data[2] = *(pdata + 2);
		xmit.data[3] = *(pdata + 3);
	}
	if (frame_mask == 2){
		 xmit.data[4] = *(pdata + 0);
		 xmit.data[5] = *(pdata + 1);
		 xmit.data[6] = *(pdata + 2);
		 xmit.data[7] = *(pdata + 3);
	}
	pthread_mutex_unlock(&xmit.mutex);
	return ret;
}

void writeSafety() {
	int *pb = xmit.ptr;
	int *p = (int *)(&xmit.data);
	
	*pb = *p;
//printf("writeSafety: p[0] = %x\t", *p);
	pb++;
	p++;
	*pb = *p;

	
//	*xmit.ptr = *(int *)xmit.data;
//	printf("**** write ****\nwrite: %x\n",*(int *)xmit.data); // need to remove
}

void readSafety() {
	int *p = (int *)(&recv.data);
	int *pb = recv.ptr;

	*p = *pb;
	p++;
	pb++;
	*p = *pb;

//	p = (int *)recv.data;
//	*p = *recv.ptr;

//	printf("**** read ****\nread: %x\n",*(int *)recv.data); // need to remove
}

int getHeartbeat() {
	int err = 0;
//	pthread_mutex_lock(&recv.mutex);
	if (recv.heartbeat == recv.data[R_HB_FRAME])
	  recv.count++;
	else
	  recv.count = 0;
	recv.heartbeat = recv.data[R_HB_FRAME];
//	pthread_mutex_unlock(&recv.mutex);
//	printf("getHeartbeat\n%x\n",(unsigned)recv.heartbeat);
	if (recv.count >= NO_HB_TIMES) {
		recv.count = NO_HB_TIMES;
		err = -1;
	}
	return err;
}

void setHeartbeat() {
	xmit.heartbeat = (char)xmit.count;
	xmit.data[W_HB_FRAME] = xmit.heartbeat;

	if (xmit.count < (char)0x0f)
	  xmit.count++;
	else
	  xmit.count = 0;

//	printf("setHeartbeat\n,xmit.heartbeat = %x\txmit.data[W_HB_FRAME] = %x\n",xmit.heartbeat,xmit.data[W_HB_FRAME]);
}

int checkSafetyCmd(char cmd) {
	int err;
	if (cmd == 0xf5)
	  err = 0;
	else
	  err = -1;
	return err;
}

unsigned long long int autorunSafetyData() {
	int err = 0;
	unsigned long long int ret = 0;
	
	err = pthread_mutex_lock(&recv.mutex);
	if (err == -1)
	  printf("Failed to lock mutex.\n"); //NOT_LOCK;
	
	readSafety();

	err = checkSafetyCmd(recv.data[FIRST_FRAME]);
	if (err == -1) {
		int *p = (int *)recv.data;
		if (*p == (int)0){
		  ret = ERR_SAFETY_NOT_CONNECT; //NOT_CONNECTED
		 // break;
		}
		ret= ERR_SAFETY_RECV_CMD; //NOT_SAFETY_CMD;
		//break;
	}

	err = getHeartbeat();
	if (err == -1)
    {
        pthread_mutex_unlock(&recv.mutex);
	    return ERR_SAFETY_NOT_CONNECT; //NO_HEARTBEAT;
    }
	
	err = pthread_mutex_unlock(&recv.mutex);
	if (err == -1)
	  printf("Failed to unlock mutex.\n"); //NOT_UNLOCK;

	err = pthread_mutex_lock(&xmit.mutex);
	if (err == -1)
	  printf("Failed to lock mutex.\n"); //NOT_LOCK;

	setHeartbeat();
	writeSafety();
	err = pthread_mutex_unlock(&xmit.mutex);
	if (err == -1)
	  printf("Failed to unlock mutex.\n"); //NOT_UNLOCK;
	return ret;
}

int fake_init() {
	int ret;
	ptr = malloc(40);
	recv.ptr = (int *)(ptr + R_SAFETY_BASE);
	xmit.ptr = (int *)(ptr + W_SAFETY_BASE);
	if (pthread_mutex_init(&recv.mutex, NULL) != 0) {
		ret = -1;
		return ret;
	}
	if (pthread_mutex_init(&recv.mutex, NULL) != 0) {
		ret = -1;
		return ret;
	}
	xmit.data[FIRST_FRAME] = (char)SAFETY_CMD;
	xmit.data[SECOND_FRAME] = 0;
	xmit.data[THIRD_FRAME] = 0;
	xmit.data[FOURTH_FRAME] = 0;
	xmit.data[FIFTH_FRAME] = 0;
	xmit.count = 0x9;
	xmit.heartbeat = 0xa0;
*recv.ptr = 0xf5abcd11;
	recv.heartbeat = 0;
	recv.count = 0;
}

int fake_connection() {
	*recv.ptr = *xmit.ptr;
}
