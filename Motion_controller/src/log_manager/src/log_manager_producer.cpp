#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>  
#include <sys/mman.h>  
#include <fcntl.h>   
#include "log_manager_producer.h"

using namespace log_space;

std::mutex LogProducer::occupied_mutex_;
std::map<unsigned long, LogControlArea*> LogProducer::pid_map_ptr_;
int32_t LogProducer::log_level_ = LOG_INFO;
uint32_t* LogProducer::isr_count_ptr_ = NULL;


LogProducer::LogProducer(void):
	is_valid_(false),
	shmem_ptr_(NULL),
	ctrl_area_ptr_(NULL)
{
}

LogProducer::~LogProducer(void)
{
	ctrl_area_ptr_->shm_occupied = 0;
    if(is_valid_ == true)
    {
    	munmap(shmem_ptr_, LOG_SHMEM_SIZE);
    }
}

bool LogProducer::init(const char *thread_name, uint32_t *isr_count_ptr)
{
    printf("Initializing log producer (%s).", thread_name);

	if (isr_count_ptr == NULL)
    {
        return false;
    }
	//init ISR feedback
	isr_count_ptr_ = isr_count_ptr;

    //open share memory
    int fd = open(LOG_SHMEM_NAME, O_RDWR);
	shmem_ptr_ = (uint8_t *)mmap(NULL, LOG_SHMEM_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, LOG_SHMEM_ADDRESS);
    close(fd);
	if (shmem_ptr_ == MAP_FAILED)
	{
		printf("Failed to map share memory\n");
		return false;
	}

	//init data pointer
	ctrl_area_ptr_ = (LogControlArea *)(shmem_ptr_);
	int32_t item_count = 1;
	// lock it in case the grab of producer threads
	occupied_mutex_.lock();
    while (ctrl_area_ptr_->shm_occupied == LOG_OCCUPIED_NUMBER)
    {
    	++item_count;
		if (item_count > LOG_BLOCK_NUMBER)
		{
		    occupied_mutex_.unlock();
		    printf("The log blocks are all occupied\n");
			return false;
		}
        ctrl_area_ptr_ = (LogControlArea *)((uint8_t *)ctrl_area_ptr_ + LOG_CTRL_AREA_SIZE + LOG_ITEM_AREA_SIZE * LOG_BLOCK_TEXT_ITEM);
    }

	//init shmem control area data
	ctrl_area_ptr_->head_index = 1;
	ctrl_area_ptr_->tail_index = 1;
	ctrl_area_ptr_->lost_item_count = 0;
	ctrl_area_ptr_->max_item = LOG_BLOCK_TEXT_ITEM;
	if (strlen(thread_name) > (LOG_NAME_SIZE - 1)) {
        memcpy(ctrl_area_ptr_->thread_name, thread_name, (LOG_NAME_SIZE - 1));
        ctrl_area_ptr_->thread_name[LOG_NAME_SIZE - 1] = '\0';
    }
    else {
        memset(ctrl_area_ptr_->thread_name, 0, LOG_NAME_SIZE);
        strcpy(ctrl_area_ptr_->thread_name, thread_name);
    }
	ctrl_area_ptr_->shm_occupied = LOG_OCCUPIED_NUMBER;
	occupied_mutex_.unlock();

    //build the map between thread id and LogControlArea ptr.
    pthread_t pid = pthread_self();
    pid_map_ptr_[pid] = ctrl_area_ptr_;

    is_valid_ = true;
	
    printf("Area(%d)\n", item_count);
	return true;
	
}


void LogProducer::setLoggingLevel(MessageLevel level)
{
    log_level_ = level;
}

void LogProducer::debug(const char *module_name, const char *format, ...)
{
    LogControlArea *ctrl_area_using_ptr = NULL;
    if(relocatePtrByThreadId(ctrl_area_using_ptr) == false)
        return;
    
    LogItemArea item;
    memset(&item, 0 ,sizeof(item));
    item.level = LOG_DEBUG;

	va_list vp;
	va_start(vp, format);
	int len = vsnprintf(item.text_buf, LOG_ITEM_AREA_TEXT_SIZE, format, vp);
	va_end(vp);

	if(len >= LOG_ITEM_AREA_TEXT_SIZE - 1)
	{
		len = LOG_ITEM_AREA_TEXT_SIZE - 2;
	}
	item.text_buf[len++] = '\n';
	item.text_buf[len] = '\0';

    constructItem(module_name, ctrl_area_using_ptr, &item);
    writeShareMemory(ctrl_area_using_ptr, &item);
}

void LogProducer::info(const char *module_name, const char *format, ...)
{
    LogControlArea *ctrl_area_using_ptr = NULL;
    if(relocatePtrByThreadId(ctrl_area_using_ptr) == false)
        return;
   
    LogItemArea item;
    memset(&item, 0 ,sizeof(item));
    item.level = LOG_INFO;

	va_list vp;
	va_start(vp, format);
	int len = vsnprintf(item.text_buf, LOG_ITEM_AREA_TEXT_SIZE, format, vp);
	va_end(vp);

	if(len >= LOG_ITEM_AREA_TEXT_SIZE - 1)
	{
		len = LOG_ITEM_AREA_TEXT_SIZE - 2;
	}
	item.text_buf[len++] = '\n';
	item.text_buf[len] = '\0';

    constructItem(module_name, ctrl_area_using_ptr, &item);
    writeShareMemory(ctrl_area_using_ptr, &item);
}
void LogProducer::warn(const char *module_name, const char *format, ...)
{
    LogControlArea *ctrl_area_using_ptr = NULL;
    if(relocatePtrByThreadId(ctrl_area_using_ptr) == false)
        return;
    
    LogItemArea item;
    memset(&item, 0 ,sizeof(item));
    item.level = LOG_WARN;

	va_list vp;
	va_start(vp, format);
	int len = vsnprintf(item.text_buf, LOG_ITEM_AREA_TEXT_SIZE, format, vp);
	va_end(vp);

	if(len >= LOG_ITEM_AREA_TEXT_SIZE - 1)
	{
		len = LOG_ITEM_AREA_TEXT_SIZE - 2;
	}
	item.text_buf[len++] = '\n';
	item.text_buf[len] = '\0';

    constructItem(module_name, ctrl_area_using_ptr, &item);
    writeShareMemory(ctrl_area_using_ptr, &item);
}
void LogProducer::error(const char *module_name, const char *format, ...)
{
    LogControlArea *ctrl_area_using_ptr = NULL;
    if(relocatePtrByThreadId(ctrl_area_using_ptr) == false)
        return;
    
    LogItemArea item;
    memset(&item, 0 ,sizeof(item));
    item.level = LOG_ERROR;

	va_list vp;
	va_start(vp, format);
	int len = vsnprintf(item.text_buf, LOG_ITEM_AREA_TEXT_SIZE, format, vp);
	va_end(vp);

	if(len >= LOG_ITEM_AREA_TEXT_SIZE - 1)
	{
		len = LOG_ITEM_AREA_TEXT_SIZE - 2;
	}
	item.text_buf[len++] = '\n';
	item.text_buf[len] = '\0';

    constructItem(module_name, ctrl_area_using_ptr, &item);
    writeShareMemory(ctrl_area_using_ptr, &item);
}

bool LogProducer::relocatePtrByThreadId(LogControlArea* &ctrl_area_using_ptr)
{
    //check which thread is using log.
    pthread_t pid = pthread_self();
    std::map<unsigned long, LogControlArea*>::iterator it;
    it = pid_map_ptr_.find(pid);
    if (it == pid_map_ptr_.end())
    {
        printf("logproducer failed to relocate ptr\n");
        return false;
    }
    else
    {
        ctrl_area_using_ptr = pid_map_ptr_[pid];
    }

    if (ctrl_area_using_ptr == NULL)
        return false;
    
    return true;
}

void LogProducer::constructItem(const char *module_name, const LogControlArea *ctrl_area_using_ptr, LogItemArea *pitem)
{
    struct timeval time_now;
	gettimeofday(&time_now, NULL);
    uint64_t sec = time_now.tv_sec;
    uint64_t usec = time_now.tv_usec;
    std::string temp_str = pitem->text_buf;
  
    snprintf(pitem->text_buf, LOG_ITEM_AREA_TEXT_SIZE, "[%u][%u][%s][%llu.%06llu][%s]%s", pitem->level, *isr_count_ptr_, 
                ctrl_area_using_ptr->thread_name, sec, usec, module_name, temp_str.c_str());
    pitem->text_buf[LOG_ITEM_AREA_TEXT_SIZE - 1] = '\0';
}

void LogProducer::writeShareMemory(LogControlArea *ctrl_area_using_ptr, const LogItemArea *pitem)
{
    if (pitem->level < log_level_) 
		return;

	uint32_t current_tail = ctrl_area_using_ptr->tail_index;
	uint32_t new_tail = current_tail + 1;
	if (current_tail == ctrl_area_using_ptr->max_item)
	{
		new_tail = 1;
	}

    // log items are full.
	uint32_t head = ctrl_area_using_ptr->head_index;
    if (head == new_tail)
    {
		uint32_t lost_count = ctrl_area_using_ptr->lost_item_count;
		++lost_count;
		ctrl_area_using_ptr->lost_item_count = lost_count;	

        printf("%s", pitem->text_buf);
    }
	else
    {
        char * item_ptr = (char *)ctrl_area_using_ptr + LOG_CTRL_AREA_SIZE + (current_tail - 1) * LOG_ITEM_AREA_SIZE;
		memcpy(item_ptr, pitem, LOG_ITEM_AREA_SIZE);
		ctrl_area_using_ptr->tail_index = new_tail;   
    }

}


