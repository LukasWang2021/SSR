
#include "logfifo.h"


#define FIFO_START_ADDRESS  0x1D102000   /*start at 8k*/

#define FIFO_TOTAL_MEMSIZE  0xDD00      /*55k, end at 63k*/

#define FIFO_RECORD_TOTALNUM      (FIFO_TOTAL_MEMSIZE/sizeof(LOG_RECORD_T)-1)

typedef struct
{
	atomic_t in_index;
    atomic_t out_index;
    int trigger_flag;
}LOG_HEADER_T;

#define FIFO_HEADER_SIZE    sizeof(LOG_HEADER_T)

//#define FIFO_HEADER_ADDRESS FIFO_START_ADDRESS+FIFO_TOTAL_MEMSIZE

LOG_HEADER_T * log_header;

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//FIFO management



static atomic_t *in_index; 
static atomic_t *out_index; 

static LOG_RECORD_T* log_buf[FIFO_RECORD_TOTALNUM+1];


static void memcpy_record(LOG_RECORD_T *dst,const LOG_RECORD_T *src)
{
    int i;
    dst->time_flag = src->time_flag;
    dst->length = src->length;
    for (i = 0;i<dst->length;++i)
    {
        dst->data[i] = src->data[i];
    }
}


static inline int get_numof_item(int l_in_index,int l_out_index)
{
    int num = 0;
    if (l_in_index >= l_out_index)
    {
        num = l_in_index - l_out_index;
    }
    else  //in_index < out_index
    {
        num = (l_in_index - 0) + (FIFO_RECORD_TOTALNUM + 1 - l_out_index);
    }
    return num;
}

static inline int increase_index(int index)
{
    if(FIFO_RECORD_TOTALNUM == index)
    {
        index = 0;
    }
    else
    {
        ++index;
    }
    return index;
}



static int fetch_log(LOG_RECORD_T * buf)
{
    int target;
    int next;
    target = atomic_read(out_index);
    while((get_numof_item(atomic_read(in_index),target)>0)&&\
        (log_buf[target]->buf_flag>0))
    {
        next = increase_index(target);
        if(atomic_cmpxchg(out_index,target,next) == target)
        {
            if(NULL!=buf)
                memcpy_record(buf,log_buf[target]);
            log_buf[target]->buf_flag = 0;
            return 0;
        }

        target = atomic_read(out_index);
    }
    return -1;
}



static int push_log(LOG_RECORD_T * logrec)
{
    int target;
    int next;
    target = atomic_read(in_index);
    while((get_numof_item(target,atomic_read(out_index))<FIFO_RECORD_TOTALNUM)&&\
        (0==log_buf[target]->buf_flag))
    {
        next = increase_index(target);     

        if(atomic_cmpxchg(in_index,target,next) == target)
        {
            memcpy_record(log_buf[target],logrec);

            log_buf[target]->buf_flag = 1;
            return 0;
        }	

        target = atomic_read(in_index);
    }
    //Never call "reportTroubleCode" in jtac_printf and sub funciton
    //reportTroubleCode(BM_DTC_E07);
    //fifo_full_flag = 1;
	return -1;

}

static void locateLogFifo(void * fifo_addr)
{
    unsigned int addr = (unsigned int)fifo_addr;
    int i;
    for (i = 0;i<FIFO_RECORD_TOTALNUM+1;++i)
    {
        log_buf[i] = (LOG_RECORD_T*)(addr + i*sizeof(LOG_RECORD_T));
    }   
    log_header = (LOG_HEADER_T *)(addr + FIFO_TOTAL_MEMSIZE);
    in_index = &log_header->in_index; 
    out_index = &log_header->out_index;     
}


////////////////////////////////////////////////////////////BM APIs/////////////////////////////////////////////

int pushRecord(LOG_RECORD_T* rec)
{
    if (0 == log_header->trigger_flag)
    {
        if(get_numof_item(atomic_read(in_index),atomic_read(out_index))>=FIFO_RECORD_TOTALNUM/2)
        {
            fetch_log(NULL);
        }        
    }
    rec->time_flag = log_header->trigger_flag;
    return push_log(rec);
}

void init_LOG_FIFO(void)
{
    int i;
    locateLogFifo((void *)FIFO_START_ADDRESS);

    for (i = 0;i<FIFO_RECORD_TOTALNUM+1;++i)
    {
        log_buf[i]->buf_flag = 0;
    }   
    
    log_header->trigger_flag = 0;
    atomic_set(in_index,0);
    atomic_set(out_index,0);
}

void Log_trigger(int flag)
{
    log_header->trigger_flag = flag;
}


////////////////////////////////////////////////////////////Linux APIs/////////////////////////////////////////////

int getRecord(LOG_RECORD_T* rec)
{
    if (1 == log_header->trigger_flag)
    {
        return fetch_log(rec);
    }

    return -1;
}

int open_LOG_FIFO(void *ptr)
{
    if (NULL!=ptr)
        locateLogFifo(ptr+(FIFO_START_ADDRESS - 0x1D100000));
    return 0;
}


