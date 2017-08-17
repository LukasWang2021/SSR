#ifndef _LOGFIFO_H_
#define _LOGFIFO_H_
#ifdef CPU1_SHAREDMEM
#include "include/Traj_reuse.h"
#include "Trouble_code.h"
#else
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>

typedef long long int data64b_t;

typedef struct {
     volatile int counter;
} atomic_t;

#define atomic_read(v)  ((v)->counter)
#define atomic_set(v,i) (((v)->counter) = (i))

static inline int atomic_cmpxchg(atomic_t *ptr, int old, int newv)
{

     return __sync_val_compare_and_swap(&ptr->counter,old,newv);

}
#endif

#define TOTAL_NUMBER_OF_VAR 256 /*name was limited to 9 chars*/
#define TOTAL_NUMBER_OF_TRIGER 32
#define TOTAL_VAR_IN_RECORD 24
/*can be changed; get data from fifo every 10ms, can be delayed to 50ms
    FIFO is 55k
    speed loop is 125us, 16 is suitable;
    speed loop is 200us, 24 is suitable;
*/

typedef struct
{
    data64b_t data[TOTAL_VAR_IN_RECORD];
    int length;// length of data;
    char time_flag;//0:before trigger point; 1 after trigger point
    char buf_flag;
}LOG_RECORD_T;

extern int pushRecord(LOG_RECORD_T* rec);

extern void init_LOG_FIFO(void);

extern void clearRecord(void);

#ifndef CPU1_SHAREDMEM
#ifdef __cplusplus
	extern "C"{
#endif
int open_LOG_FIFO(void *ptr);

int getRecord(LOG_RECORD_T* rec);

#ifdef __cplusplus
}
#endif

#endif


#endif
