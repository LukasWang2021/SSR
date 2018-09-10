/**********************************************
Copyright ¿?2016 Foresight-Robotics Ltd. All rights reserved.
File:       safety.c
Author:     Shuguo.Zhang Feng.Wu 
Create:     20-Sep-2017
Modify:     22-Sep-2017
Summary:    dealing with safety board
**********************************************/

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include "safety.h"

#define SAFETY_LEN      0x80    //modified by Feng.Wu
#define SAFETY_BASE             0xC00A0000

#define R_SAFETY_BASE   0x0C    /* the address of date that read from safety MCU1, and 0x08 is a offset with a base 0xC1013000. by Feng.Wu*/
#define RR_SAFETY_BASE  0x14    /* the address of date that read from safety MCU2, and 0x10 is a offset with a base 0xC1013000. by Feng.Wu*/
#define W_SAFETY_BASE   0x04    /* the address of date that written to safety, and 0x18 is a offset with a base C1013000. */

#define RW_MASK 0XFF00
#define R_MASK  0X0100
#define W_MASK  0X0200
#define FRAME_MASK 0X00FF

static int fd;
static void *ptr;

struct  Safety {
        int     fd;
        int     *ptr;
        pthread_mutex_t mutex;
        char data[SAFETY_MAX_FRAME];
        char heartbeat;
        char count;
};


static struct Safety    xmit;
static struct Safety    recv_one;   // add by Feng.Wu
static struct Safety    recv_two;   // add by Feng.Wu
static char *write_ptr;
static char *read_ptr;

unsigned long long int initSafety(void) {
   int ret = 0;
   fd = open("/dev/mem", O_RDWR);
   if (fd == -1) {
        printf("Failed to open /dev/mem .\n");
        return ERR_SAFETY_FILE_OPEN;
   }
   else {
        printf("Open /dev/mem OK .\n");
   }
                
   ptr = mmap(NULL, SAFETY_LEN, PROT_WRITE|PROT_READ, 
                               MAP_SHARED, fd, SAFETY_BASE);
    if (ptr == (void *)-1) {
        printf("Failed to mmap /dev/mem .\n");
        return ERR_SAFETY_FILE_MAP;
    }
    else {
        printf("mmap /dev/mem OK .\n");
    }

    recv_one.ptr = (int *)(ptr + R_SAFETY_BASE);    // add by Feng.Wu
    recv_two.ptr = (int *)(ptr + RR_SAFETY_BASE);   // add by Feng.Wu
        xmit.ptr = (int *)(ptr + W_SAFETY_BASE);
//    write_ptr = (char *)ptr + W_SEQ_SAFETY_BASE;
//    read_ptr = (char*)ptr + R_SEQ_SAFETY_BASE;

        if (pthread_mutex_init(&recv_one.mutex, NULL) != 0) {
                printf("Failed to init mutex.\n");
        }
    if (pthread_mutex_init(&recv_two.mutex, NULL) != 0) {
                printf("Failed to init mutex.\n");
        }
        if (pthread_mutex_init(&xmit.mutex, NULL) != 0) {
                printf("Failed to init mutex.\n");
        }

        xmit.data[FIRST_FRAME] = (char)SAFETY_CMD_SEND;
        xmit.data[SECOND_FRAME] = (char)SAFETY_CORE_SEND;
        xmit.data[THIRD_FRAME] = 0;
        xmit.data[FOURTH_FRAME] = 0;
        xmit.data[FIFTH_FRAME] = 0;
    xmit.data[SIXTH_FRAME] = 0; // add by Feng.Wu
        xmit.heartbeat = 0;
        xmit.count = 0x9;

        recv_one.heartbeat = 0;
        recv_one.count = 0;

    recv_two.heartbeat = 0; // add by Feng.Wu
    recv_two.count = 0;     // add by Feng.Wu

        return (unsigned long long int)ret;
}

unsigned long long int openSafety(void) {
        return initSafety();
}

void closeSafety(void) {
        munmap(ptr, SAFETY_LEN);
        close(fd);
}

int getSafety(int frame, unsigned long long int *err) {
        int mask;
        int frame_mask;
    struct Safety   *safety;
        int *pdata;
        char ret;

        *err = 0;

    
        mask = frame & RW_MASK;
        switch (mask) {
                case R_MASK:
                        safety = &recv_one;
                        break;
                case W_MASK:
                        safety = &xmit;
                        break;
                default:
                        *err = ERR_SAFETY_FRAME; 
                        return 0;
        }

        frame_mask = frame & FRAME_MASK;
/*      if ((frame_mask >= 1) & (frame_mask <= R_SAFETY_MAX_FRAME)) {
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
 //       printf("mask = %d. \n", mask);
  //        printf("frame_mask = %d. \n", frame_mask);
  //     printf("0502 read %08X(%08X) and %08X(%08X) at %d. \n", 
  //                  (int *)(&safety->data), *(int *)(&safety->data), 
  //                  (int *)(&safety->data) + 1, *((int *)(&safety->data) + 1), 
  //                      frame_mask);
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
/*      if ((frame_mask >= 1) & (frame_mask <= W_SAFETY_MAX_FRAME)) {
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
        if (frame_mask == 2){
                 xmit.data[4] = *(pdata + 0);
                 xmit.data[5] = *(pdata + 1);
                 xmit.data[6] = *(pdata + 2);
                 xmit.data[7] = *(pdata + 3);
        }
        pthread_mutex_unlock(&xmit.mutex);
        return ret;
}

//   void setHeartbeat() {
//      xmit.heartbeat = (char)xmit.count;
//      xmit.data[W_HB_FRAME] = xmit.heartbeat;
//   
//      if (xmit.count < (char)0x0f)
//        xmit.count++;
//      else
//        xmit.count = 0;
//   
//      //printf("\nsetHeartbea: xmit.heartbeat = %x\txmit.data[W_HB_FRAME] = %x\n",xmit.heartbeat,xmit.data[W_HB_FRAME]);
//   }

void writeSafety(void) {
        int *pb = xmit.ptr;
        int *p = (int *)(&xmit.data);
        
//    printf("writeSafety: p[0] = %x\t", *p);
    *pb = *p;
//      pb++;
//      p++;
//      *pb = *p;

//    printf("writeSafety: p[1] = %x\t", *p);
//    printf("**** write **** write: %x\n",*(int *)xmit.data); // need to remove
}

void safetyWriteDownload(void)  // add by Feng.Wu
{
    int err = 0;
    err = pthread_mutex_lock(&xmit.mutex);
        if (err == -1)
          printf("Failed to lock mutex.\n"); //NOT_LOCK;

//    setHeartbeat();
    writeSafety();

    err = pthread_mutex_unlock(&xmit.mutex);
        if (err == -1)
          printf("Failed to unlock mutex.\n"); //NOT_UNLOCK;

}

void safetySetSeq(char seq)
{
    *write_ptr = seq;
    //printf("safetysetseq, safety_seq = %x, write_seq = %x\n", *write_ptr, seq);
}

void safetyGetSeq(char *seq)
{
    *seq = *(char *)read_ptr;
}

int checkSafetyCmd(char cmd) {
        int err;
        if (cmd == 0x5f)
          err = 0;
        else
          err = -1;
        return err;
}

unsigned long long int readSafety(void) {
    static int show_diff = 0 ;
        int *p = (int *)(&recv_one.data);
        int *pb_one = recv_one.ptr;
    int *pb_two = recv_two.ptr;

    // check CMD.
    char cmd = *((char *)pb_one);
    int err = checkSafetyCmd(cmd);
        if (err == -1) {
                if (*pb_one == (int)0){
            return ERR_SAFETY_NOT_CONNECT; //NOT_CONNECTED
                }
                return ERR_SAFETY_RECV_CMD; //NOT_SAFETY_CMD;
        }
    cmd = *((char *)pb_two);
    err = checkSafetyCmd(cmd);
        if (err == -1) {
                if (*pb_two == (int)0){
            return ERR_SAFETY_NOT_CONNECT; //NOT_CONNECTED
                }
                return ERR_SAFETY_RECV_CMD; //NOT_SAFETY_CMD;
        }


    // check if data from MCU1 and MCU2 are same.
    pb_one++;
    pb_two++;
    if (*pb_one != *pb_two){
        if(show_diff < 10)
           printf("mcu1 = %08x, mcu2 = %08x\n", *pb_one, *pb_two);
        show_diff++ ;
        // return ERR_SAFETY_RECV_DIFF;
    }
    pb_two--; // pb_one--;

    // update data if everything is ok.
     *p = *pb_two; //   *p = *pb_one;
   p++;  //     p++;
   pb_two++;  //        pb_one++;
   *p = *pb_two;  // *p = *pb_one;

        //printf("**** read ****\nread: %x\n",*(int *)recv_one.data); // need to remove
    return 0;
}

int getHeartbeat(void) {
        int err = 0;
        if (recv_one.heartbeat == recv_one.data[R_HB_FRAME])
          recv_one.count++;
        else
          recv_one.count = 0;
        recv_one.heartbeat = recv_one.data[R_HB_FRAME];
        //printf("getHeartbeat: %x\n",(unsigned)recv_one.heartbeat);
        if (recv_one.count >= NO_HB_TIMES) {
                recv_one.count = NO_HB_TIMES;
                err = -1;
        }
        return err;
}

unsigned long long int safetyReadUpload(void)  // add by Feng.Wu
{
    int err = 0;
    unsigned long long int ret = 0;
    err = pthread_mutex_lock(&recv_one.mutex);
        if (err == -1)
          printf("Failed to lock mutex.\n"); //NOT_LOCK;
        
        ret = readSafety();
    if (ret != 0){
        pthread_mutex_unlock(&recv_one.mutex);
        return ret;
    }

        err = getHeartbeat();
        if (err == -1){
        pthread_mutex_unlock(&recv_one.mutex);
            return ERR_SAFETY_NOT_CONNECT; //NO_HEARTBEAT;
    }

    err = pthread_mutex_unlock(&recv_one.mutex);
        if (err == -1)
          printf("Failed to unlock mutex.\n"); //NOT_UNLOCK;

    return ret;
}

unsigned long long int autorunSafetyData() {
    unsigned long long int ret = 0;
    static char seq = 0;
    static char read_counter = 0;

    // step1: write download data.
    safetyWriteDownload();

    // wait for data update.
    usleep(150);

    // step2: update data.
    ret = safetyReadUpload();
    if (ret != 0){
        return ret;
    }
    else{
        read_counter++;
    }
    if (read_counter > NO_HB_TIMES)
        return ERR_SAFETY_NOT_CONNECT;

    return ret;
}

int fake_init() {
        int ret;
        ptr = malloc(80);
        recv_one.ptr = (int *)(ptr + R_SAFETY_BASE);    // modified by Feng.Wu
    recv_two.ptr = (int *)(ptr + RR_SAFETY_BASE);   // modified by Feng.Wu
        xmit.ptr = (int *)(ptr + W_SAFETY_BASE);
        if (pthread_mutex_init(&recv_one.mutex, NULL) != 0) {
                ret = -1;
                return ret;
        }
        if (pthread_mutex_init(&recv_two.mutex, NULL) != 0) {
                ret = -1;
                return ret;
        }
    if (pthread_mutex_init(&xmit.mutex, NULL) != 0) {
                ret = -1;
                return ret;
        }

        xmit.data[FIRST_FRAME]  = 0; // modified by Feng,Wu
        xmit.data[SECOND_FRAME] = 0;
        xmit.data[THIRD_FRAME]  = 0;
        xmit.data[FOURTH_FRAME] = 0;
        xmit.data[FIFTH_FRAME]  = 0;
        xmit.data[SIXTH_FRAME]  = 0;
        xmit.count = 0x9;
        xmit.heartbeat = 0xa0;

    *recv_one.ptr = 0xf5abcd11;
        recv_one.heartbeat = 0;
        recv_one.count = 0;

    recv_one.heartbeat = 0;
        recv_one.count = 0;
    return ret;
}

int fake_connection() {
        *recv_one.ptr = *xmit.ptr;
}


