/*************************************************************************
	> File Name: log_manager_server.cpp
	> Author: 
	> Mail: 
	> Created Time: 2016年12月23日 星期五 18时01分09秒
 ************************************************************************/

#include <stdarg.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <map>
#include <stdexcept>
#include <iostream>
#include <log_manager_version.h>
#include <log_manager/log_manager_server.h>

#include <stdlib.h>  
#include <sys/mman.h>  
#include <fcntl.h>
#include <sys/stat.h> 
#include <sys/file.h> 

using std::string;
using std::vector;
using std::cout;
using std::endl;

using fst_log::ControlArea;
using fst_log::LogFlag;
using fst_log::LogItem;
using fst_log::LogControlBlock;
using fst_log::MSG_LEVEL_LOG;
using fst_log::MSG_LEVEL_INFO;
using fst_log::MSG_LEVEL_WARN;
using fst_log::MSG_LEVEL_ERROR;

// global running flag:
//      true->running
//      false->quit
bool g_running = true;


ControlArea *ctrl_area;
LogFlag     *flag_area;
LogItem     *text_area;

LogControlBlock    g_server_log;
LogControlBlock    *g_lcb_ptr_queue[MAX_LOG_CONTROL_BLOCK];

LogItem g_item_pool[SERVER_ITEM_POOL_SIZE];
int     pool_in, pool_out;

// protect g_item_pool
pthread_mutex_t g_item_pool_mutex;

void log(const char *format, ...)
{
    char buf[LOG_ITEM_SIZE];
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    int len = sprintf(buf, "[  LOG][%ld.%6ld]", time_now.tv_sec, time_now.tv_usec);

    va_list vp;
    va_start(vp, format);
    len = len + vsnprintf(buf + len, LOG_ITEM_SIZE - len - 1, format, vp);
    va_end(vp);
    
    if (len > LOG_ITEM_SIZE - 2)
        len = LOG_ITEM_SIZE - 2;
    
    buf[len]     = '\n';
    buf[len + 1] = '\0';
    
    //printf("\033[0m%s", buf);
    if (g_server_log.working)
        g_server_log.file_handle << buf;
}

void info(const char *format, ...)
{
    char buf[LOG_ITEM_SIZE];
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    int len = sprintf(buf, "[ INFO][%ld.%6ld]", time_now.tv_sec, time_now.tv_usec);

    va_list vp;
    va_start(vp, format);
    len = len + vsnprintf(buf + len, LOG_ITEM_SIZE - len - 1, format, vp);
    va_end(vp);
    
    if (len > LOG_ITEM_SIZE - 2)
        len = LOG_ITEM_SIZE - 2;
    
    buf[len]     = '\n';
    buf[len + 1] = '\0';
    
    printf("\033[0m%s", buf);
    if (g_server_log.working)
        g_server_log.file_handle << buf;
}

void warn(const char *format, ...)
{
    char buf[LOG_ITEM_SIZE] = {0};
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    int len = sprintf(buf, "[ WARN][%ld.%6ld]", time_now.tv_sec, time_now.tv_usec);

    va_list vp;
    va_start(vp, format);
    len = len + vsnprintf(buf + len, LOG_ITEM_SIZE - len - 1, format, vp);
    va_end(vp);
    
    if (len > LOG_ITEM_SIZE - 2)
        len = LOG_ITEM_SIZE - 2;
    
    buf[len] = '\n';
    buf[len + 1] = '\0';
    
    printf("\033[33m%s\033[0m", buf);
    if (g_server_log.working)
        g_server_log.file_handle << buf;
}

void error(const char *format, ...)
{
    char buf[LOG_ITEM_SIZE] = {0};
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    int len = sprintf(buf, "[ERROR][%ld.%6ld]", time_now.tv_sec, time_now.tv_usec);

    va_list vp;
    va_start(vp, format);
    len += vsnprintf(buf + len, LOG_ITEM_SIZE - len - 1, format, vp);
    va_end(vp);
    
    if (len > LOG_ITEM_SIZE - 2)
        len = LOG_ITEM_SIZE - 2;
    
    buf[len] = '\n';
    buf[len + 1] = '\0';
    
    printf("\033[31m%s\033[0m", buf);
    if (g_server_log.working)
        g_server_log.file_handle << buf;
}

uintmax_t totalLogFileSize(boost::filesystem::path &path)
{
    uintmax_t total = 0;
    int reg_cnt = 0;
    int oth_cnt = 0;

    boost::filesystem::directory_iterator beg_iter(path);
    boost::filesystem::directory_iterator end_iter;
    
    for (; beg_iter != end_iter; ++beg_iter) {
        if (boost::filesystem::is_regular_file(*beg_iter)) {
            total += file_size(*beg_iter);
            reg_cnt++;
        }
        else {
            oth_cnt++;
        }
    }

    log("%d regular files, %d other files", reg_cnt, oth_cnt);
    log("Total log file size: %d", total);
    return total;
}

bool delOldestLogFile(boost::filesystem::path &path)
{
    time_t old_time = 0x7FFFFFFF;

    boost::filesystem::path tmp;
    boost::filesystem::path old_file;
    boost::filesystem::directory_iterator beg_iter(path);
    boost::filesystem::directory_iterator end_iter;

    for (; beg_iter != end_iter; ++beg_iter) {
        if (boost::filesystem::is_regular_file(*beg_iter)) {
            if (last_write_time(*beg_iter) < old_time) {

                int i = 0;
                bool in_use = false;
                
                tmp = *beg_iter;
                string name = tmp.string();

                while (i < MAX_LOG_CONTROL_BLOCK) {
                    if (NULL != g_lcb_ptr_queue[i]) {
                        if (name == g_lcb_ptr_queue[i]->file_name) {
                            in_use = true;
                            break;
                        }
                    }

                    i++;
                }

                if (!in_use) {
                    old_file = *beg_iter;
                    old_time = last_write_time(*beg_iter);
                }
            }
        }
    }

    if (old_time < 0x7FFFFFFF) {
        double size_in_MB = file_size(old_file) / 1024 / 1024;

        remove(old_file);
        
        log("Delete log file: %s", old_file.c_str());
        log("Free %.1fMB of log space", size_in_MB);

        return true;
    }
    else {
        error("Fail to find out the oldest log file");
        return false;
    }
}

void checkLogSpace(boost::filesystem::path &path)
{
    if (totalLogFileSize(path) > NO_ENOUGH_LOG_SPACE_WARNING) {
        warn("Log space is full, please clear useless log files manually.");
        warn("Otherwise, the oldest log file will be deleted automatically.");

        if (totalLogFileSize(path) > MAX_LOG_FILE_SPACE) {
            while (totalLogFileSize(path) > NO_ENOUGH_LOG_SPACE_WARNING) {
                if (!delOldestLogFile(path)) break;
            }

            if (totalLogFileSize(path) > NO_ENOUGH_LOG_SPACE_WARNING) {
                error("Error while checking log space.");
                error(" -No enough space and fail to delete old files in log space");
            }
        }
    }
}

char buildLogControlBlock(string &name)
{
    info(" -Constructing log-control-block ...");

    for (int i = 0; i < MAX_LOG_CONTROL_BLOCK; ++i) {
        if (NULL == g_lcb_ptr_queue[i]) {
            g_lcb_ptr_queue[i] = new fst_log::LogControlBlock();
            
            if (NULL != g_lcb_ptr_queue[i]) {
                g_lcb_ptr_queue[i]->id      = i + 1;
                g_lcb_ptr_queue[i]->working = false;
                g_lcb_ptr_queue[i]->name    = name;
    
                char buf[64] = {0};
                time_t time_now = time(NULL);
                tm *local = localtime(&time_now);
                strftime(buf, 64, "_%Y%m%d%H%M%S", local);
                boost::filesystem::path path = "/root/log/";
                string file_name = path.string();
                file_name = file_name + name + buf + ".log";

                checkLogSpace(path);

                g_lcb_ptr_queue[i]->file_name = file_name;
                g_lcb_ptr_queue[i]->character_cnt = 0;
                g_lcb_ptr_queue[i]->file_create_time = time_now;
                g_lcb_ptr_queue[i]->file_handle.open(file_name.c_str(), std::ios::app);
                g_lcb_ptr_queue[i]->serial_num = 0;

                if (g_lcb_ptr_queue[i]->file_handle.is_open()) {
                    g_lcb_ptr_queue[i]->working = true;
                    //lockFile(g_lcb_ptr_queue[i]->file_name);
                    info(" -Success!");
                    return g_lcb_ptr_queue[i]->id;
                }
                else {
                    error(" -Fail to open a log file");
                    delete g_lcb_ptr_queue[i];
                    g_lcb_ptr_queue[i] = NULL;

                    return 0;
                }
            }
            else {
                error(" -Fail to construct log-control-block");
                log(" -Get a NULL pointer while new a LCB");
            }
        }
        else {
            continue;
        }
    }

    error(" -No LCB available ...");
    return 0;
}

string getNameFromID(char id)
{
    string name;

    if (NULL != g_lcb_ptr_queue[id - 1])
        name = g_lcb_ptr_queue[id - 1]->name;
    else {
        log("Fail to get channel name from ID=%d", id);
        log("Could not find corresponding LCB from this ID");
        name = "";
    }

    return name;
}

void public_thread(void)
{
    log("Constructing log-in server ...");

    ctrl_area->register_block.flag_out  = false;
    ctrl_area->register_block.id        = 0x00;

    info(" -Success!");

    char buffer[DIRECTORY_BUF_SIZE] = {0};
   
    info("Waiting for Log-in request ...");

    int loop = 0;
    int cnt_down = 10;

    while (g_running) {
        if (true == ctrl_area->register_block.flag_in) {
            ctrl_area->register_block.flag_in   = false;

            if (0 == ctrl_area->register_block.id) {
                // Log-in request
                warn("Log-in request received, client name='%s'", ctrl_area->register_block.name);
                string name = ctrl_area->register_block.name;
                ctrl_area->register_block.id    = buildLogControlBlock(name);
                info(" -ID=%d assigned to the client", ctrl_area->register_block.id);
                ctrl_area->register_block.flag_out  = true;
            }
            else {
                // Log-out request
                warn("Log-out request received, id=%d, client name='%s'",
                     ctrl_area->register_block.id, getNameFromID(ctrl_area->register_block.id).c_str());
                
                if (NULL != g_lcb_ptr_queue[ctrl_area->register_block.id - 1]) {
                    g_lcb_ptr_queue[ctrl_area->register_block.id - 1]->working    = false;
                    cnt_down = 10;
                    log("The corresponding LCB will be deleted in about 10 seconds");
                    ctrl_area->register_block.flag_out  = true;
                }
            }
        }
        else {
            usleep(20 * 1000);
        }

        loop++;
        if (loop > 50) {
            loop = 0;
            cnt_down--;

            if (cnt_down == 0) {
                cnt_down = 10;

                for (int i = 0; i < MAX_LOG_CONTROL_BLOCK; ++i) {
                    if (g_lcb_ptr_queue[i] != NULL) {
                        if (g_lcb_ptr_queue[i]->working == false) {
                            warn("Free useless LCB, id = %d, channel-name=%s",
                                 g_lcb_ptr_queue[i]->id, g_lcb_ptr_queue[i]->name.c_str());

                            if (g_lcb_ptr_queue[i]->file_handle.is_open()) {
                                //unlockFile(g_lcb_ptr_queue[i]->file_name);
                                g_lcb_ptr_queue[i]->file_handle.close();
                            }
                        
                            delete g_lcb_ptr_queue[i];
                            g_lcb_ptr_queue[i] = NULL;
                        }
                    }
                }
            }

        }
    }  // while (g_running)

    log("Public-thread terminated.");
}

void receive_thread(void)
{
    log("Receive-thread running ...");

    int out, next, cnt;
    char buf[LOG_ITEM_SIZE];

    while (g_running) {
        out = ctrl_area->index_out;
        
        if (flag_area[out] == ITEM_INUSE) {
            next = out + 1;
            if (next == LOG_ITEM_COUNT) next = 0;

            if (true == ctrl_area->index_out.compare_exchange_weak(out, next)) {
                pthread_mutex_lock(&g_item_pool_mutex);
                if ((pool_in + 1) % SERVER_ITEM_POOL_SIZE != pool_out) {
                    // item pool has space for new item
                    g_item_pool[pool_in] = text_area[out];
                    pool_in = (pool_in + 1) % SERVER_ITEM_POOL_SIZE;
                    cnt++;
                }
                pthread_mutex_unlock(&g_item_pool_mutex);

                flag_area[out] = ITEM_FREE;
            }
        }
        else {
            // log("Get %d items from share-mem", cnt);
            cnt = 0;
            usleep(10 * 1000);
        }
    }

    log("Receive-thread terminated.");
}


void io_thread(void)
{
    log("IO-thread running ...");
    
    int len  = 0;
    int loop = 0;
    char buf[LOG_ITEM_SIZE];

    LogItem items[100];
    int num_of_item;

    while (g_running) {
        if (pool_in != pool_out) {
            // something need to do
            num_of_item = 0;
            pthread_mutex_lock(&g_item_pool_mutex);
            
            while (pool_in != pool_out) {
                items[num_of_item] = g_item_pool[pool_out];
                num_of_item++;
                pool_out = (pool_out + 1) % SERVER_ITEM_POOL_SIZE;

                if (num_of_item > 99)  break;
            }

            pthread_mutex_unlock(&g_item_pool_mutex);

            // log("Get %d items from item-pool", num_of_item);

            for (int i = 0; i < num_of_item; ++i) {
                memset(buf, 0, sizeof(buf));

                switch (items[i].level)
                {
                    case MSG_LEVEL_LOG:
                        len = sprintf(buf, "[  LOG][%ld.%6ld]%s",
                                      items[i].stamp.tv_sec, items[i].stamp.tv_usec, items[i].text);
                        break;

                    case MSG_LEVEL_INFO:
                        len = sprintf(buf, "[ INFO][%ld.%6ld]%s",
                                      items[i].stamp.tv_sec, items[i].stamp.tv_usec, items[i].text);
                        break;

                    case MSG_LEVEL_WARN:
                        len = sprintf(buf, "[ WARN][%ld.%6ld]%s",
                                      items[i].stamp.tv_sec, items[i].stamp.tv_usec, items[i].text);
                        break;

                    case MSG_LEVEL_ERROR:
                        len = sprintf(buf, "[ERROR][%ld.%6ld]%s",
                                      items[i].stamp.tv_sec, items[i].stamp.tv_usec, items[i].text);
                        break;

                    default:
                        len = sprintf(buf, "[OTHER][%ld.%6ld]%s",
                                      items[i].stamp.tv_sec, items[i].stamp.tv_usec, items[i].text);
                }

                // len = len + sprintf(buf + len, "%s", items[i].text);

                if (NULL != g_lcb_ptr_queue[items[i].id - 1]) {
                    LogControlBlock *plcb = g_lcb_ptr_queue[items[i].id - 1];
                    
                    unsigned short tmp_num = plcb->serial_num;
                    tmp_num++;

                    if (tmp_num != items[i].number) {
                        // Some log items were lost beyond this item
                        int num;

                        if (items[i].number > plcb->serial_num + 1)
                            num = items[i].number - (plcb->serial_num + 1);
                        else
                            num = 65536 + items[i].number - (plcb->serial_num + 1);

                        char tmp[LOG_ITEM_SIZE] = {0};
                        int tmp_len = sprintf(tmp, "[  LOG][%ld.%6ld]",
                                        items[i].stamp.tv_sec, items[i].stamp.tv_usec);
                        tmp_len = tmp_len + sprintf(tmp + tmp_len, "<<<%d items have been lost here>>>\n", num);
                        
                        plcb->file_handle << tmp;
                        plcb->character_cnt += len;
                    }

                    plcb->file_handle << buf;
                    plcb->character_cnt += len;
                    plcb->serial_num = items[i].number;
                }
                else {
                    log("Could not find corresponding LCB from ID=%d", items[i].id);
                    log("It may caused by unregistered client or abnormal aborts");
                }
            }
        }
        else {
            usleep(50 * 1000);
        }

        loop++;

        if (loop > 200) {
            loop = 0;
         
            for (int i = 0; i < MAX_LOG_CONTROL_BLOCK; ++i)
                if (g_lcb_ptr_queue[i] != NULL)
                    g_lcb_ptr_queue[i]->file_handle.flush();

            for (int i = 0; i < MAX_LOG_CONTROL_BLOCK; ++i) {
                if (g_lcb_ptr_queue[i] != NULL) {
                    time_t time_now = time(NULL);
                    
                    if (g_lcb_ptr_queue[i]->character_cnt > MAX_LOG_FILE_SIZE || 
                        time_now - g_lcb_ptr_queue[i]->file_create_time > MAX_LOG_FILE_RETENTION_TIME)
                    {
                        log("Log file reach its limit, open a new file automatically ...");
                        log(" -Channel=%s, ID=%d", g_lcb_ptr_queue[i]->name.c_str(), g_lcb_ptr_queue[i]->id);
                        log(" -File:%s", g_lcb_ptr_queue[i]->file_name.c_str());

                        char buf[DIRECTORY_BUF_SIZE] = {0};

                        

                        boost::filesystem::path path = "/root/log/";
                        string file_name = path.string();
                        tm *local = localtime(&time_now);
                        strftime(buf, 64, "_%Y%m%d%H%M%S", local);
                        file_name = file_name + g_lcb_ptr_queue[i]->name + buf + ".log";

                        checkLogSpace(path);

                        //unlockFile(g_lcb_ptr_queue[i]->file_name);

                        g_lcb_ptr_queue[i]->file_name = file_name;
                        g_lcb_ptr_queue[i]->character_cnt = 0;
                        g_lcb_ptr_queue[i]->file_create_time = time_now;
                        g_lcb_ptr_queue[i]->file_handle.close();
                        g_lcb_ptr_queue[i]->file_handle.open(file_name.c_str(), std::ios::app);

                        if (g_lcb_ptr_queue[i]->file_handle.is_open()) {
                            //lockFile(g_lcb_ptr_queue[i]->file_name);
                            log(" -Success!");
                        }
                        else {
                            error(" -Fail to open log file");
                            delete g_lcb_ptr_queue[i];
                            g_lcb_ptr_queue[i] = NULL;
                        }
                    }
                }
            }
        }
    }

    while (pool_in != pool_out) {
        // write items to their files before exit
        memset(buf, 0, sizeof(buf));

        switch (g_item_pool[pool_out].level)
        {
            case MSG_LEVEL_LOG:
                len = sprintf(buf, "[  LOG][%ld.%6ld]",
                              g_item_pool[pool_out].stamp.tv_sec, g_item_pool[pool_out].stamp.tv_usec);
                break;

            case MSG_LEVEL_INFO:
                len = sprintf(buf, "[ INFO][%ld.%6ld]",
                              g_item_pool[pool_out].stamp.tv_sec, g_item_pool[pool_out].stamp.tv_usec);
                break;

            case MSG_LEVEL_WARN:
                len = sprintf(buf, "[ WARN][%ld.%6ld]",
                              g_item_pool[pool_out].stamp.tv_sec, g_item_pool[pool_out].stamp.tv_usec);
                break;

            case MSG_LEVEL_ERROR:
                len = sprintf(buf, "[ERROR][%ld.%6ld]",
                              g_item_pool[pool_out].stamp.tv_sec, g_item_pool[pool_out].stamp.tv_usec);
                break;

            default:
                len = sprintf(buf, "[OTHER][%ld.%6ld]",
                              g_item_pool[pool_out].stamp.tv_sec, g_item_pool[pool_out].stamp.tv_usec);
        }

        len = len + sprintf(buf + len, "%s", g_item_pool[pool_out].text);

        if (NULL != g_lcb_ptr_queue[g_item_pool[pool_out].id - 1]) {
            LogControlBlock *plcb = g_lcb_ptr_queue[g_item_pool[pool_out].id - 1];

            if (plcb->serial_num + 1 != g_item_pool[pool_out].number) {
                // Some log items were lost beyond this item
                int num;

                if (g_item_pool[pool_out].number > plcb->serial_num + 1)
                    num = g_item_pool[pool_out].number - (plcb->serial_num + 1);
                else
                    num = 65536 + g_item_pool[pool_out].number - (plcb->serial_num + 1);

                char tmp[LOG_ITEM_SIZE] = {0};
                int tmp_len = sprintf(tmp, "[  LOG][%ld.%6ld]",
                                      g_item_pool[pool_out].stamp.tv_sec, g_item_pool[pool_out].stamp.tv_usec);
                tmp_len = tmp_len + sprintf(tmp + tmp_len, "<<<%d items have been lost here>>>\n", num);

                plcb->file_handle << tmp;
                plcb->character_cnt += tmp_len;
            }

            plcb->file_handle << buf;
            plcb->character_cnt += len;
            plcb->serial_num = g_item_pool[pool_out].number;
        }
        else {
            log("Could not find corresponding LCB from ID=%d", g_item_pool[pool_out].id);
            log("It may caused by unregistered client or abnormal aborts");
        }
        
        pool_out = (pool_out + 1) % SERVER_ITEM_POOL_SIZE;
    }
    /*
    for (int i = 0; i < MAX_LOG_CONTROL_BLOCK; ++i)
        if (g_lcb_ptr_queue[i] != NULL)
            printf("id=%d, char=%ld\n", i + 1, g_lcb_ptr_queue[i]->character_cnt);
    */
    
    log("IO-thread terminated.");
}

void buildLogControlArea(ControlArea *ptr)
{
    new (ptr) ControlArea;
}

static void sigintHandle(int num)
{
    log("Interrupt request catched.");
    
    g_running = false;
    
    usleep(50 * 1000);
}

int initLogSpace(void)
{
    char buf[DIRECTORY_BUF_SIZE] = {0};
    boost::filesystem::path path = "/root/log/";
    info("Log directory: %s", path.string().c_str());

    if (!boost::filesystem::is_directory(path)) {
        boost::filesystem::create_directories(path);
        warn("log directory not exist ... created");
    }

    double used = (double)totalLogFileSize(path) / 1024 / 1024;
    int total   = MAX_LOG_FILE_SPACE / 1024 / 1024;
    info("Log space usage (used / total): %.1fMB / %dMB", used, total);
    
    checkLogSpace(path);
    
    if (used > MAX_LOG_FILE_SPACE) {
        used = totalLogFileSize(path);
        info("Log space usage (used / total): %dMB / %dMB",
                used / 1024 / 1024,
                MAX_LOG_FILE_SPACE / 1024 / 1024
        );
    }

    return 0;
}

int initShareMemory(void)
{
    info("Prepare share memory ...");

    int fd = shm_open("fst_log_shm", O_CREAT|O_RDWR, 00777);
    
    if (-1 == fd) {
        error(" -Error in openMem(): failed on opening sharedmem");
        return -1;
    }

    int lock = flock(fd, LOCK_EX | LOCK_NB);
    
    if (lock == -1) {
        error(" -Fail to take over the 'fst_log_shm', it has controlled by another server");
        return -2;
    }

    ftruncate(fd, LOG_MEM_SIZE);
    
    char *ptr = (char*) mmap(NULL, LOG_MEM_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    
    if (ptr == MAP_FAILED) {
        close(fd);
        error(" -Error in openMem(): failed on mapping process sharedmem");
        return -3;
    }

    memset(ptr, 0, LOG_MEM_SIZE);

    //buildLogControlArea(ctrl_area);
   
    ctrl_area = GET_CONTROL_AREA_PTR(ptr);
    flag_area = GET_FLAG_AREA_PTR(ptr);
    text_area = GET_TEXT_AREA_PTR(ptr);

    info(" -Success!");
    return 0;
}

int initServerLog(void)
{
    g_server_log.name    = "log_server";
    g_server_log.working = false;

    char buf[DIRECTORY_BUF_SIZE] = {0};
    boost::filesystem::path path = "/root/log/";
    string file_name = path.string();
    time_t time_now = time(NULL);
    tm *local = localtime(&time_now);
    strftime(buf, 64, "_%Y%m%d%H%M%S", local);
    file_name = file_name + g_server_log.name + buf + ".log";

    g_server_log.file_name = file_name;
    g_server_log.character_cnt = 0;
    g_server_log.file_create_time = time_now;
    g_server_log.file_handle.open(file_name.c_str(), std::ios::app);

    if (g_server_log.file_handle.is_open()) {
        g_server_log.working = true;
        info("Server log initialized, logging to file: %s", g_server_log.file_name.c_str());
        //lockFile(g_server_log.file_name);
        return 0;
    }
    else {
        g_server_log.working = false;
        error("Fail to open server log file, all logs from server will lost");

        return -1;
    }
}

/*
bool lockFile(string &file)
{
    string cmd;
    cmd = "chattr +a " + file;
    FILE *ptr = popen(cmd.c_str(), "r");
    if (ptr != NULL) {
        string  ret;
        int     cnt = 256;
        char    con[cnt];

        while(fgets(con, cnt, ptr))
            ret += con;

        log("Lock file: %s.", file.c_str());
        if (ret != "")
            warn("%s", ret.c_str());
        
        pclose(ptr);
        return true;
    }
    else {
        warn("Fail to lock file: %s.", file.c_str());
        return false;
    }
}

bool unlockFile(string &file)
{
    string cmd;
    cmd = "chattr -a " + file;
    
    FILE *ptr = popen(cmd.c_str(), "r");

    if (ptr != NULL) {
        string  ret;
        int     cnt = 256;
        char    con[cnt];

        while(fgets(con, cnt, ptr))
            ret += con;

        log("Unlock file: %s.", file.c_str());
        if (ret != "")
            warn("%s", ret.c_str());
        
        pclose(ptr);
        return true;
    }
    else {
        warn("Fail to unlock file: %s.", file.c_str());
        return false;
    }
}
*/

int main(int argc, char **argv)
{
    signal(SIGINT, sigintHandle);

    initServerLog();

    info("Log Manager Version %d.%d.%d",  log_manager_VERSION_MAJOR,
                                            log_manager_VERSION_MINOR,
                                            log_manager_VERSION_PATCH);

    if (initShareMemory() != 0) {
        error("Fail to initialize share memory");
        return -1;
    }

    if (initLogSpace() != 0) {
        error("Fail to initialize log space");
        return -1;
    }

    for (int i = 0; i < MAX_LOG_CONTROL_BLOCK; ++i) {
        g_lcb_ptr_queue[i] = NULL;
    }

    memset(&g_item_pool, 0, sizeof(g_item_pool));
    pool_in  = 0;
    pool_out = 0;
    pthread_mutex_init(&g_item_pool_mutex, NULL);

    boost::thread log_public(&public_thread);
    boost::thread log_receive(&receive_thread);
    boost::thread log_io(&io_thread);

    log_public.join();
    log_receive.join();
    log_io.join();

    for (int i = 0; i < MAX_LOG_CONTROL_BLOCK; ++i) {
        if (g_lcb_ptr_queue[i] != NULL) {
            if (g_lcb_ptr_queue[i]->file_handle.is_open())
                g_lcb_ptr_queue[i]->file_handle.close();
            //unlockFile(g_lcb_ptr_queue[i]->file_name);
            delete g_lcb_ptr_queue[i];
            g_lcb_ptr_queue[i] = NULL;
        }
    }

    warn("Log server exit.");
    //unlockFile(g_server_log.file_name);
    g_server_log.file_handle.close();
    
    return 0;
}
