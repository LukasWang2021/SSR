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

LogComsumer* LogComsumer::instance_ = NULL;

LogComsumer::LogComsumer(void):
    display_enable_(false),
	log_enable_(true),
	is_exit_(false),
	block_step_(0),
	max_log_size_(0),
	retain_log_size_(0),
	display_level_(0),
	shmem_ptr_(NULL)
{
    param_ptr_ = new LogManagerConfig();
}

LogComsumer::~LogComsumer(void)
{
	if(param_ptr_ != NULL)
	{
		delete param_ptr_;
		param_ptr_ = NULL;
	}
	if (log_queue_ptr_ != NULL)
	{
		delete[] log_queue_ptr_;
		log_queue_ptr_ = NULL;
	}
	munmap(shmem_ptr_, LOG_SHMEM_SIZE);
	//printf("~LogComsumer success\n");
}

LogComsumer* LogComsumer::getInstance()
{
    if (instance_ == NULL)
    {
    	instance_ = new LogComsumer();
    }
	return instance_;
}


bool LogComsumer::init(int32_t level, std::vector<std::string> &thread_list)
{
    //load params
    if(!param_ptr_->loadParam()){
        //printf("Failed to load log comsumer config files\n");
		return false;
    }

	log_queue_ptr_ = new LogControlBlock[LOG_BLOCK_NUMBER];
	initLogQueue();

	//open share memory
	int fd = open(LOG_SHMEM_NAME, O_RDWR);
	shmem_ptr_ = (uint8_t *)mmap(NULL, LOG_SHMEM_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, LOG_SHMEM_ADDRESS);
	if (shmem_ptr_ == MAP_FAILED)
	{
		close(fd);
		//printf("Failed to map share memory\n");
		return false;
	}
	close(fd);

	//create log_path ="/root/ftp/log/" directory if not exist.
	if (!createLogPath(param_ptr_->log_path_.c_str()))
	{
		return false;
	}
	
	block_step_ = LOG_CTRL_AREA_SIZE + LOG_ITEM_AREA_SIZE * LOG_BLOCK_TEXT_ITEM;
	max_log_size_ = param_ptr_->max_log_size_ * 1024 *1024;
	retain_log_size_ = max_log_size_ * (param_ptr_->percent_log_retain_ * 0.01);

    display_enable_ = param_ptr_->display_enable_;
	log_enable_ = param_ptr_->log_enable_;
	if (level >= LOG_DEBUG && level <= LOG_ERROR)
	{
		display_level_ = level;
	}
	display_thread_list_ = thread_list;

	//printf("Display_enable: %d\nDisplay_level: %d\nLog_enable: %d\n",display_enable_, display_level_, log_enable_);

	return true;	
}

void LogComsumer::runLogComsuming(void)
{
	//create log_path ="/root/log/" directory if not exist.
	if (!createLogPath(param_ptr_->log_path_.c_str()))
	{
		return;
	}

	LogControlArea * ctrl_area_ptr = (LogControlArea *)(shmem_ptr_);
	for (size_t i = 0; i < LOG_BLOCK_NUMBER; ++i)
	{
		if (ctrl_area_ptr->shm_occupied == LOG_OCCUPIED_NUMBER)
		{
			log_queue_ptr_[i].ctrl_area_ptr = ctrl_area_ptr;
			log_queue_ptr_[i].text_area_ptr = (char *)ctrl_area_ptr + LOG_CTRL_AREA_SIZE;
			writeLogFiles(&log_queue_ptr_[i]);
		}

		if (ctrl_area_ptr->shm_occupied != LOG_OCCUPIED_NUMBER)
		{
			//close file.
			if (log_queue_ptr_[i].file_fd != -1)
			{
				close(log_queue_ptr_[i].file_fd);
				log_queue_ptr_[i].file_fd = -1;
			}
		}
		ctrl_area_ptr = (LogControlArea *)((uint8_t *)ctrl_area_ptr + block_step_);
	}

	usleep(param_ptr_->cycle_time_);
}

void LogComsumer::setExit(void)
{
    is_exit_ = true;
}
bool LogComsumer::isExit(void)
{
    return is_exit_;
}

void LogComsumer::writeLogFiles(LogControlBlock *log_block_ptr)
{
    // create a new file if not exist
    if (log_block_ptr->file_fd == -1)
    {
        checkLogDirSpace(param_ptr_->log_path_.c_str());
    
    	time_t seconds = time(NULL);
		tm *local = localtime(&seconds);
		char buf[64] = {0};
        strftime(buf, 64, "_%Y-%m-%d_%H-%M-%S", local);
        std::string path = param_ptr_->log_path_;
		path = path + log_block_ptr->ctrl_area_ptr->thread_name + buf + ".log";

		log_block_ptr->file_fd = open(path.c_str(), O_RDWR|O_CREAT);
		if (log_block_ptr->file_fd == -1)
		{
			//printf("failed to create file: %s\n", path.c_str());
			return;
		}
		log_block_ptr->file_name = path;
    }

    // write log file
	uint32_t head = log_block_ptr->ctrl_area_ptr->head_index;
	uint32_t tail = log_block_ptr->ctrl_area_ptr->tail_index;
	char * head_ptr = log_block_ptr->text_area_ptr + (head - 1) * LOG_ITEM_AREA_SIZE;
	if (head == tail)
	{
		return;
	}
	else if (head < tail)
	{
		for (uint32_t i = head; i < tail; ++i)
		{
		    if (log_enable_ == true)
		    {
			    write(log_block_ptr->file_fd, ((LogItemArea *)head_ptr)->text_buf, strlen(((LogItemArea *)head_ptr)->text_buf));
		    }
			displayLog(head_ptr, log_block_ptr->ctrl_area_ptr->thread_name);
			
			head_ptr = head_ptr + LOG_ITEM_AREA_SIZE;
		}	
		log_block_ptr->log_item_cnt += tail - head;
	}
	else if (tail < head)
	{
		uint32_t max_item = log_block_ptr->ctrl_area_ptr->max_item;
		for(uint32_t i = head; i < (max_item + 1); ++i)
		{
		    if (log_enable_ == true)
		    {
			    write(log_block_ptr->file_fd, ((LogItemArea *)head_ptr)->text_buf, strlen(((LogItemArea *)head_ptr)->text_buf));
		    }
			displayLog(head_ptr, log_block_ptr->ctrl_area_ptr->thread_name);
			
			head_ptr = head_ptr + LOG_ITEM_AREA_SIZE;
		}
		
        char * start_ptr = log_block_ptr->text_area_ptr;
		for(uint32_t j = 1; j < tail; ++j)
		{
		    if (log_enable_ == true)
		    {
			    write(log_block_ptr->file_fd, ((LogItemArea *)start_ptr)->text_buf, strlen(((LogItemArea *)start_ptr)->text_buf));
		    }	
			displayLog(start_ptr, log_block_ptr->ctrl_area_ptr->thread_name);

			start_ptr = start_ptr + LOG_ITEM_AREA_SIZE;
		}
		log_block_ptr->log_item_cnt += max_item + tail - head;
	}
	log_block_ptr->ctrl_area_ptr->head_index = tail;

    // write the count of the lost log items.	
	if(log_block_ptr->ctrl_area_ptr->lost_item_count != 0
		&& log_block_ptr->lost_item_cnt != log_block_ptr->ctrl_area_ptr->lost_item_count)
	{
		log_block_ptr->lost_item_cnt = log_block_ptr->ctrl_area_ptr->lost_item_count;

		LogItemArea item;
		item.level = LOG_WARN;
		struct timeval time_now;
	    gettimeofday(&time_now, NULL);
	    uint64_t sec = time_now.tv_sec;
	    uint64_t usec = time_now.tv_usec;

        #ifdef COMPILE_IN_ARM
		snprintf(item.text_buf, LOG_ITEM_AREA_SIZE - 4, "\033[33m[%llu.%llu][%s]%u logs are lost\n", sec, usec, 
			log_block_ptr->ctrl_area_ptr->thread_name, log_block_ptr->lost_item_cnt);
        #else
		snprintf(item.text_buf, LOG_ITEM_AREA_SIZE - 4, "\033[33m[%lu.%lu][%s]%u logs are lost\n", sec, usec, 
			log_block_ptr->ctrl_area_ptr->thread_name, log_block_ptr->lost_item_cnt);
        #endif

        if (log_enable_ == true)
        {
			write(log_block_ptr->file_fd, item.text_buf, strlen(item.text_buf));
        }
		displayLog((char *)&item, log_block_ptr->ctrl_area_ptr->thread_name);
		
		++log_block_ptr->log_item_cnt;
	}

    // log file is full, so close it. Then create another file in the next loop.
	if (log_block_ptr->log_item_cnt > param_ptr_->max_file_log_item_)
	{
		log_block_ptr->log_item_cnt = 0;
		close(log_block_ptr->file_fd);
		log_block_ptr->file_fd = -1;
	}
}

void LogComsumer::displayLog(char *item_ptr, std::string thread_name)
{
    if (display_enable_ == false)
    {
    	return;
    }
	
    LogItemArea * log_item_ptr = (LogItemArea *)item_ptr;
	if (log_item_ptr->level < display_level_)
	{
		return;
	}

	size_t size = display_thread_list_.size();
	if (size == 0)
	{
		displayItem(log_item_ptr->level, log_item_ptr->text_buf);
	}
	else
	{
	    for (size_t i = 0; i < size; ++i)
	    {
		    if (thread_name == display_thread_list_[i])
		    {
				displayItem(log_item_ptr->level, log_item_ptr->text_buf);
				break;
		    }
	    }
	}
}

void LogComsumer::displayItem(int32_t level, char *item_ptr)
{
	std::string str;
    switch(level)
    {
        case LOG_DEBUG: 
        case LOG_INFO:
		    printf("\033[0m%s", item_ptr);
            break;
        case LOG_WARN:
		    printf("\033[33m%s\033[0m", item_ptr);
            break;
        case LOG_ERROR:
		    printf("\033[31m%s\033[0m", item_ptr);
            break;
        default:
		    printf("\033[41m%s\033[0m", item_ptr);
            break;
    }
}

void LogComsumer::initLogQueue(void)
{
    for(size_t i = 0; i < LOG_BLOCK_NUMBER; ++i)
    {
		log_queue_ptr_[i].ctrl_area_ptr = NULL;
		log_queue_ptr_[i].text_area_ptr = NULL;
		log_queue_ptr_[i].log_item_cnt = 0;
		log_queue_ptr_[i].lost_item_cnt = 0;
		log_queue_ptr_[i].file_fd = -1;
    }
}

bool LogComsumer::createLogPath(const char *dir_path)
{
	//create log_path ="/root/log/" directory
	if (access(dir_path, 0) == -1)
	{
		if (mkdir(dir_path, 0777) != 0)
		{
		    printf("can't create directory: %s\n", dir_path);
			return false;
		}
		chmod(dir_path, S_IRWXU | S_IRWXG | S_IRWXO);

		//in case the log directory is deleted unexpectly.
	    for(size_t i = 0; i < LOG_BLOCK_NUMBER; ++i)
        {
		    log_queue_ptr_[i].log_item_cnt = 0;
			if (log_queue_ptr_[i].file_fd != -1)
			{
				close(log_queue_ptr_[i].file_fd);
				log_queue_ptr_[i].file_fd = -1;
			}
        }
	}
	return true;
}


bool LogComsumer::checkLogDirSpace(const char *dir_path)
{
    int64_t total_size = getLogDirSize(param_ptr_->log_path_.c_str());
	if (total_size > max_log_size_)
	{
		while (getLogDirSize(param_ptr_->log_path_.c_str()) > retain_log_size_)
		{
			if (!deleteOldestLogFile(param_ptr_->log_path_.c_str()))
			{
				break;
			}
		}
	}
	return true;
}

int64_t LogComsumer::getLogDirSize(const char *dir_path)
{
	int64_t total_size = 0;
	DIR *dp;
	struct dirent *entry;
	struct stat info;
	if ((dp = opendir(dir_path)) == NULL)
		return -1;
	
	lstat(dir_path, &info);
	total_size += info.st_size;
	
	while ((entry = readdir(dp)) != NULL)
	{
		char subdir[512];
		sprintf(subdir, "%s/%s", dir_path, entry->d_name);
		lstat(subdir, &info);
	
		if (S_ISDIR(info.st_mode))
		{
			if (strcmp(".", entry->d_name) == 0 || strcmp("..", entry->d_name) == 0)
				continue;
			int64_t subdir_size = getLogDirSize(subdir);
			total_size += subdir_size;
		}
		else
		{
			total_size += info.st_size;
		}
	}
	closedir(dp);
	return total_size;
}


bool LogComsumer::deleteOldestLogFile(const char *dir_path)
{
	// open the directory. 
	DIR *dp;
	if ((dp = opendir(dir_path)) == NULL)
	{
		printf("No such directory: %s\n", dir_path);
		return false;
	}

    //find the oldest file.
    struct dirent *entry;
	struct stat info;
	std::string dir = dir_path;
	std::string file_path;
	std::string oldest_file_path;
	long modify_time = LONG_MAX;
	
	while ((entry = readdir(dp)) != NULL)
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;
        else if (entry->d_type == DT_DIR)  //dir
            continue;
		else if (entry->d_type == DT_REG) //file
        {
            file_path = dir + entry->d_name;

            //the file being used will not be deleted
			bool in_use = false;
			for(size_t i = 0; i < LOG_BLOCK_NUMBER; ++i)
            {
			    if (log_queue_ptr_[i].file_fd != -1 && log_queue_ptr_->file_name == file_path)
			    {
				    in_use = true;
			    }
            }
			//find the older file.
            if(lstat(file_path.c_str(), &info) == 0 && in_use == false)
            {
                if(modify_time > info.st_mtime)
                {
                	modify_time = info.st_mtime;
					oldest_file_path = file_path;
                }
            }
        }
    }
    closedir(dp);
	remove(oldest_file_path.c_str());

    return true;
}


LogComsumer * g_comsumer_ptr = NULL;

const char* g_short_options = "l:t:";
struct option g_long_options[] = {
    {"level", 1, NULL, 'l'}, 
    {"thread", 1, NULL, 't'},
    {0,0,0,0},
};

void logComsumerExit(int dunno)
{
    std::cout<<"Log Comsumer is exiting."<<std::endl;
	g_comsumer_ptr->setExit();
}

int main(int argc, char *argv[])
{	
    int level = 0;
	std::vector<std::string> list;
    int c;
	while((c = getopt_long(argc, argv, g_short_options, g_long_options, NULL)) != -1)
	{
		switch (c)
		{
			case 'l':
				level = atoi(optarg);
				printf("Display level argv: %d\n", level);
				break;
			case 't':
				list.push_back(optarg);
				break;
		}
	}

	for (int i = optind; i < argc; ++i)
	{
		list.push_back(argv[i]);
	}
	for(size_t i = 0; i < list.size(); ++i)
	{
		printf("Thread name argv:%s\n", list[i].c_str());
	}
	
    LogComsumer * comsumer_ptr = LogComsumer::getInstance();
	if (comsumer_ptr != NULL)
	{
		bool ret = comsumer_ptr->init(level, list);
		if (ret == false)
		{
			std::cout<<"Failed to init Log Comsumer"<<std::endl;
			return -1;
		}

		g_comsumer_ptr = comsumer_ptr;
		signal(SIGINT, logComsumerExit);
		signal(SIGTERM, logComsumerExit);
		while(!comsumer_ptr->isExit())
		{
		    comsumer_ptr->runLogComsuming();
		}
		
	}
	delete comsumer_ptr;
	std::cout<<"Log Comsumer exit success."<<std::endl;
	return 0;
}

