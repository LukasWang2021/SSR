#include "log_manager_producer_bare.h"

#define LOG_TEXT_SIZE_BARE 448

using namespace virtual_servo_device;


typedef struct{
    unsigned int        level;          //MessageLevel
    unsigned int        isr_count; 
	unsigned int        cpu_id;
    char                thread_name[LOG_NAME_SIZE];
    char                text[LOG_TEXT_SIZE_BARE];
}LogItemContextBare;

typedef struct{
	LogControlArea*   ctrl_area_ptr;
	char*             text_area_ptr;
	uint32_t*         isr_count_ptr;
	uint8_t           is_valid;
	MessageLevel      log_level;
	
}LogManagerData;

static LogManagerData s_log[LOG_BLOCK_NUMBER];
static int32_t s_index = 0;
static LogItemArea s_text_buf;

int32_t virtual_servo_device::initLogProducerBare(const char *thread_name, uint32_t *isr_count_ptr)
{	
	//open share memory
	char* shmem_ptr = (char *)LOG_SHMEM_ADDRESS;
	
	//init data pointer
	s_log[s_index].ctrl_area_ptr = (LogControlArea *)(shmem_ptr);
	int item_count = 1;
    while (s_log[s_index].ctrl_area_ptr->shm_occupied == LOG_OCCUPIED_NUMBER)
    {
    	++item_count;
		if (item_count > LOG_BLOCK_NUMBER)
		{
			return -1;
		}		
        s_log[s_index].ctrl_area_ptr = (LogControlArea *)((char *)(s_log[s_index].ctrl_area_ptr) + LOG_CTRL_AREA_SIZE + LOG_ITEM_AREA_SIZE * LOG_BLOCK_TEXT_ITEM);
    }

	s_log[s_index].ctrl_area_ptr->shm_occupied = LOG_OCCUPIED_NUMBER;	
	s_log[s_index].text_area_ptr = (char *)s_log[s_index].ctrl_area_ptr + LOG_CTRL_AREA_SIZE;

	//init shmem control area data
	s_log[s_index].ctrl_area_ptr->head_index = 1;
	s_log[s_index].ctrl_area_ptr->tail_index = 1;
	s_log[s_index].ctrl_area_ptr->lost_item_count = 0;
	s_log[s_index].ctrl_area_ptr->max_item = LOG_BLOCK_TEXT_ITEM;
	if (strlen(thread_name) > (LOG_NAME_SIZE - 1)) {
        memcpy(s_log[s_index].ctrl_area_ptr->thread_name, thread_name, (LOG_NAME_SIZE - 1));
        s_log[s_index].ctrl_area_ptr->thread_name[LOG_NAME_SIZE - 1] = '\0';
    }
    else {
        memset(s_log[s_index].ctrl_area_ptr->thread_name, 0, LOG_NAME_SIZE);
        strcpy(s_log[s_index].ctrl_area_ptr->thread_name, thread_name);
    }

	//init ISR feedback
	s_log[s_index].isr_count_ptr = isr_count_ptr;

	s_log[s_index].is_valid = 0x0F;
	s_log[s_index].log_level = LOG_INFO;

	++s_index;
	return s_index;
}

void virtual_servo_device::setLoggingLevelBare(int32_t fd, MessageLevel level)
{
    int index = fd - 1;
	if (s_log[index].is_valid != 0x0F)
	{
	    return;
	}
	s_log[index].log_level = level;
}


void constructItemBare(int32_t index, MessageLevel level, const char *string, LogItemContextBare *pitem)
{
	pitem->level = level;
	pitem->isr_count = *(s_log[index].isr_count_ptr);
	pitem->cpu_id = SELF_CPU_ID;
	memcpy(pitem->thread_name, s_log[index].ctrl_area_ptr->thread_name, LOG_NAME_SIZE);

    int len = strlen(string);
    if (len > (LOG_TEXT_SIZE_BARE - 2)) {
        memcpy(pitem->text, string, (LOG_TEXT_SIZE_BARE - 2));
		pitem->text[LOG_TEXT_SIZE_BARE - 2] = '\n';
        pitem->text[LOG_TEXT_SIZE_BARE - 1] = '\0';
    }
    else {
        memset(pitem->text, 0, LOG_TEXT_SIZE_BARE);
        strcpy(pitem->text, string);
	    pitem->text[len++] = '\n';
		pitem->text[len] = '\0';
    }

}

void writeShareMemoryFromBare(int32_t index, LogItemContextBare *pitem)
{
	uint32_t current_tail = s_log[index].ctrl_area_ptr->tail_index;
	uint32_t new_tail = current_tail + 1;
	if (current_tail == s_log[index].ctrl_area_ptr->max_item)
	{
	    new_tail = 1;
	}
	
	// log items are full.
	uint32_t head = s_log[index].ctrl_area_ptr->head_index;
	if (head == new_tail)
	{
	    uint32_t lost_count = s_log[index].ctrl_area_ptr->lost_item_count;
		++lost_count;
		s_log[index].ctrl_area_ptr->lost_item_count = lost_count;   
	}
	else
	{
		char * item_ptr = s_log[index].text_area_ptr + (current_tail - 1) * LOG_ITEM_AREA_SIZE;
		memset(&s_text_buf, 0, sizeof(LogItemArea));

		char str[64] = {0};		
		switch(pitem->level)
		{
			case LOG_DEBUG:
			case LOG_INFO:
				strcpy(str, "\033[0m[%u][%u][%s]%s");
				break;
			case LOG_WARN:
				strcpy(str, "\033[33m[%u][%u][%s]%s");
				break;
			case LOG_ERROR:
				strcpy(str, "\033[31m[%u][%u][%s]%s");
				break;
			default:
				break;
		}

		s_text_buf.level = pitem->level;
		snprintf(s_text_buf.text_buf, LOG_ITEM_AREA_SIZE - 4, str, pitem->level, pitem->isr_count, 
					   pitem->thread_name, pitem->text);
			
		memcpy(item_ptr, &s_text_buf, LOG_ITEM_AREA_SIZE);
		s_log[index].ctrl_area_ptr->tail_index = new_tail;
	}

}

void virtual_servo_device::debugBare(int32_t fd, const char *string)
{
    if(fd < 1 || fd > LOG_BLOCK_NUMBER)
		return;
	
    int index = fd - 1;
	if (s_log[index].is_valid != 0x0F)
	    return;

	if (s_log[index].log_level > LOG_DEBUG) 
		return;

	LogItemContextBare item;
	memset(&item, 0 ,sizeof(item));
    constructItemBare(index, LOG_DEBUG, string, &item);
    writeShareMemoryFromBare(index, &item);
}
void virtual_servo_device::infoBare(int32_t fd, const char *string)
{
	if(fd < 1 || fd > LOG_BLOCK_NUMBER)
			return;
	
    int index = fd - 1;
	if (s_log[index].is_valid != 0x0F)
	    return;

	if (s_log[index].log_level > LOG_INFO) 
		return;

	LogItemContextBare item;
	memset(&item, 0 ,sizeof(item));
    constructItemBare(index, LOG_INFO, string, &item);
    writeShareMemoryFromBare(index, &item);
}
void virtual_servo_device::warnBare(int32_t fd, const char *string)
{
	if(fd < 1 || fd > LOG_BLOCK_NUMBER)
			return;
	
    int index = fd - 1;
	if (s_log[index].is_valid != 0x0F)
	    return;

	if (s_log[index].log_level > LOG_WARN) 
		return;

	LogItemContextBare item;
	memset(&item, 0 ,sizeof(item));
    constructItemBare(index, LOG_WARN, string, &item);
    writeShareMemoryFromBare(index, &item);

}
void virtual_servo_device::errorBare(int32_t fd, const char *string)
{
	if(fd < 1 || fd > LOG_BLOCK_NUMBER)
			return;
	
    int index = fd - 1;
	if (s_log[index].is_valid != 0x0F)
	    return;

	if (s_log[index].log_level > LOG_ERROR) 
		return;

	LogItemContextBare item;
	memset(&item, 0 ,sizeof(item));
	constructItemBare(index, LOG_ERROR, string, &item);
    writeShareMemoryFromBare(index, &item);	
}

int32_t virtual_servo_device::unblindLogProducerBare(int32_t fd)
{
    if(fd < 1 || fd > LOG_BLOCK_NUMBER)
		return -1;
	
    int index = fd - 1;
	if (s_log[index].is_valid != 0x0F)
	    return -1;

	s_log[index].is_valid = 0;
	s_log[index].ctrl_area_ptr = NULL;
	s_log[index].text_area_ptr = NULL;
	s_log[index].isr_count_ptr = NULL;
	s_log[index].log_level = LOG_INFO;

	return 0;
}





