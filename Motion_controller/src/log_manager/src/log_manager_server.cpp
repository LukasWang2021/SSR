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
#include <comm_interface/comm_interface.h>
#include <log_manager_version.h>
#include <log_manager/log_manager_logger.h>
#include <log_manager/log_manager_server.h>

using std::string;
using std::vector;
using std::cout;
using std::endl;


// global running flag:
//      true->running
//      false->quit
bool g_running = true;

// log structure queue
vector<fst_log::LogServerStruct*>   g_log_structure_ptr_queue;

// lock on g_log_structure_ptr_queue
pthread_mutex_t g_log_structure_ptr_queue_mutex;

void info(const char *format, ...)
{
    char buf[SINGLE_LOG_SIZE];
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    int len = sprintf(buf, "[ INFO][%ld.%6ld]", time_now.tv_sec, time_now.tv_usec);

    va_list vp;
    va_start(vp, format);
    len = len + vsnprintf(buf + len, SINGLE_LOG_SIZE - len - 1, format, vp);
    va_end(vp);
    
    if (len > SINGLE_LOG_SIZE - 2)
        len = SINGLE_LOG_SIZE - 2;
    
    buf[len]     = '\n';
    buf[len + 1] = '\0';
    
    printf("%s", buf);
}

void warn(const char *format, ...)
{
    char buf[SINGLE_LOG_SIZE] = {0};
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    int len = sprintf(buf, "[ WARN][%ld.%6ld]", time_now.tv_sec, time_now.tv_usec);

    va_list vp;
    va_start(vp, format);
    len = len + vsnprintf(buf + len, SINGLE_LOG_SIZE - len - 1, format, vp);
    va_end(vp);
    
    if (len > SINGLE_LOG_SIZE - 2)
        len = SINGLE_LOG_SIZE - 2;
    
    buf[len] = '\n';
    buf[len + 1] = '\0';
    
    printf("\033[33m%s\033[0m", buf);
}

void error(const char *format, ...)
{
    char buf[SINGLE_LOG_SIZE] = {0};
    struct timeval time_now;
    gettimeofday(&time_now, NULL);
    int len = sprintf(buf, "[ERROR][%ld.%6ld]", time_now.tv_sec, time_now.tv_usec);

    va_list vp;
    va_start(vp, format);
    len += vsnprintf(buf + len, SINGLE_LOG_SIZE - len - 1, format, vp);
    va_end(vp);
    
    if (len > SINGLE_LOG_SIZE - 2)
        len = SINGLE_LOG_SIZE - 2;
    
    buf[len] = '\n';
    buf[len + 1] = '\0';
    
    printf("\033[31m%s\033[0m", buf);
}

uintmax_t totalLogFileSize(boost::filesystem::path &path)
{
    uintmax_t total = 0;

    boost::filesystem::directory_iterator beg_iter(path);
    boost::filesystem::directory_iterator end_iter;
    
    for (; beg_iter != end_iter; ++beg_iter) {
        if (boost::filesystem::is_regular_file(*beg_iter)) {
            total += file_size(*beg_iter);
        }
    }

    // info("total log file size: %d", total);
    return total;
}

uintmax_t delOldestLogFile(boost::filesystem::path &path)
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

                while (i < g_log_structure_ptr_queue.size()) {
                    if (name == g_log_structure_ptr_queue[i]->log_file_name) {
                        in_use = true;
                        break;
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
        uintmax_t size = file_size(old_file);
        remove(old_file);
        warn("delete log file: %s", old_file.c_str());
        return size;
    }
    else {
        return 0;
    }
}

void checkLogSpace(boost::filesystem::path &path)
{
    if (totalLogFileSize(path) > NO_ENOUGH_LOG_SPACE_WARNING) {
        warn("Log space is full, please clear useless log files manually.");
        warn("Otherwise, the oldest log file will be deleted automatically.");

        if (totalLogFileSize(path) > MAX_LOG_FILE_SPACE)
            while (totalLogFileSize(path) > NO_ENOUGH_LOG_SPACE_WARNING)
                delOldestLogFile(path);
    }
}

bool buildLogStructure(const char *channel_name)
{
    LOCK;

    for (int i = 0; i < g_log_structure_ptr_queue.size(); ++i) {
        if (g_log_structure_ptr_queue[i]->channel_name == channel_name) {
            UNLOCK;
            error(" -Channel already exist, cannot construct a communication interface with name '%s'");        
            return false;
        }
    }

    UNLOCK;

    info(" -Constructing log structure ...");

    char buf[DIRECTOTY_BUF_SIZE] = {0};
    int length = readlink("/proc/self/exe", buf, sizeof(buf));
    
    if (length > 0 && length < sizeof(buf)) {
        boost::filesystem::path path(buf);
        path = path.parent_path().parent_path().parent_path();
        string file_name = path.string() + "/log/";
                
        memset(buf, 0, sizeof(buf));
        
        time_t time_now = time(NULL);
        tm *local = localtime(&time_now);
        strftime(buf, 64, "_%Y_%m_%d_%H_%M_%S", local);
        file_name = file_name + channel_name + buf + ".log";
    
        path = path / "/log/";
        checkLogSpace(path);

        fst_log::LogServerStruct *tmp_log_struct = NULL;
        try {
            tmp_log_struct = new fst_log::LogServerStruct;
        }
        catch (std::exception exc) {
            error(" -Constructing log structure exception: %s", exc.what());
            delete tmp_log_struct;

            return false;
        }
        
        try {
            tmp_log_struct->comm_interface_ptr = new fst_comm_interface::CommInterface;
        }
        catch (std::exception exc) {
            error(" -Constructing communication interface exception: %s", exc.what());

            delete tmp_log_struct->comm_interface_ptr;
            delete tmp_log_struct;

            return false;
        }

        if (tmp_log_struct->comm_interface_ptr->createChannel(COMM_REP, COMM_IPC, channel_name) != 0) {
            error(" -Cannot setup %s server", channel_name);

            delete tmp_log_struct->comm_interface_ptr;
            delete tmp_log_struct;

            return false;
        }

        tmp_log_struct->channel_name = channel_name;
        tmp_log_struct->log_file_name = file_name;
        tmp_log_struct->open_flag = true;
        tmp_log_struct->buffer_pool.reserve(10);
        tmp_log_struct->file_handle.open(file_name.c_str(), std::ios::app);
        tmp_log_struct->file_write_cnt = 0;
        tmp_log_struct->file_create_time = time_now;

        if (tmp_log_struct->file_handle.is_open()) {
            LOCK;
            g_log_structure_ptr_queue.push_back(tmp_log_struct);
            UNLOCK;
            
            info(" -Success!");
            return true;
        }
        else {
            error(" -Cannot construct log file");

            delete tmp_log_struct->comm_interface_ptr;
            delete tmp_log_struct;

            return false;
        }
    }
    else {
        error(" -Cannot construct log file");
        return false;
    }
}

void deleteLogStructure(const char *channel_name) {
    LOCK;

    for (int i = 0; i < g_log_structure_ptr_queue.size(); ++i) {
        if (g_log_structure_ptr_queue[i]->channel_name == channel_name) {
            vector<char*> segments;
            
            info("Delete log structure: '%s'", g_log_structure_ptr_queue[i]->channel_name.c_str());
            
            segments.assign(g_log_structure_ptr_queue[i]->buffer_pool.begin(),
                            g_log_structure_ptr_queue[i]->buffer_pool.end());
            g_log_structure_ptr_queue[i]->buffer_pool.clear();
            g_log_structure_ptr_queue[i]->open_flag = false;
            
            std::ofstream &handle = g_log_structure_ptr_queue[i]->file_handle;
            
            UNLOCK;
            
            do_io(handle, segments);
            
            LOCK;

            g_log_structure_ptr_queue[i]->file_handle.close();
            delete g_log_structure_ptr_queue[i]->comm_interface_ptr;
            delete g_log_structure_ptr_queue[i];
            g_log_structure_ptr_queue.erase(g_log_structure_ptr_queue.begin() + i);
            
            UNLOCK;
            
            break;
        }
    }

    UNLOCK;
    
    info(" -Success!");
}

void do_io(std::ofstream& handle, vector<char*> segments)
{
    for (int cnt = 0; cnt < segments.size(); ++cnt) {
        handle << segments[cnt];
        delete[] segments[cnt];
    }
}

void public_thread(void) {
    fst_comm_interface::CommInterface server;
    
    info("Constructing public server ...");
    
    if (server.createChannel(COMM_REP, COMM_IPC, "log_public") != 0) {
        error(" -Cannot setup public server.");
        g_running = false;
    }

    info(" -Success!");

    char buffer[DIRECTOTY_BUF_SIZE] = {0};
    string channel_to_close = "";
    
    while (g_running) {
        usleep(50 * 1000);
        
        memset(buffer, 0, sizeof(buffer));
        
        if (server.recv(buffer, sizeof(buffer), COMM_DONTWAIT) == 0) {
            info("A new log request received, name='%s'", buffer);
            
            if (buildLogStructure(buffer)) {
                if (server.send(buffer, sizeof(buffer), COMM_DONTWAIT) == 0) {
                    info(" -Log structure ready, logging '%s'", buffer);
                }
                else {
                    deleteLogStructure(buffer);
                    error(" -Lost communication with remote log client, the log structure removed.");
                }
            }
            else {
                error(" -Cannot construct log structure");
            }
        }

        LOCK;

        for (int i = 0; i < g_log_structure_ptr_queue.size(); ++i) {
            if (g_log_structure_ptr_queue[i]->open_flag == false) {
                channel_to_close = g_log_structure_ptr_queue[i]->channel_name;
                break;
            }
        }

        UNLOCK;

        if (channel_to_close != "") {
            deleteLogStructure(channel_to_close.c_str());
            channel_to_close = "";
        }
    }  // while (g_running)

    warn(" -Public thread terminated.");
}

void receive_thread(void) {
    char recv_buffer[LOG_BUFFER_SIZE] = {0};

    while (g_running) {
        usleep(5 * 1000);

        LOCK;

        for (int i = 0; i < g_log_structure_ptr_queue.size(); ++i) {
            if (g_log_structure_ptr_queue[i]->open_flag == false) {
                continue;
            }

            memset(recv_buffer, 0, sizeof(recv_buffer));
            
            if (g_log_structure_ptr_queue[i]->comm_interface_ptr->recv(recv_buffer, sizeof(recv_buffer), COMM_DONTWAIT) == 0) {
                char *buf = new char[LOG_BUFFER_SIZE];
                memcpy(buf, recv_buffer, LOG_BUFFER_SIZE);

                if (buf[0] == '\33' && buf[6] == '\33') {
                    if (buf[1] == 'C' && buf[2] == 'L' && buf[3] == 'O' && buf[4] == 'S' && buf[5] == 'E' ) {
                        memset(buf, 0, sizeof(buf));
                        buf[0] = 'O';   buf[1] = 'K';
                        
                        if (g_log_structure_ptr_queue[i]->comm_interface_ptr->send(buf, sizeof(buf), COMM_DONTWAIT) != 0)
                            error("Fail to response to client log-out request");
                        
                        g_log_structure_ptr_queue[i]->open_flag = false;
                        
                        delete[] buf;
                        
                        continue;
                    }
                }

                g_log_structure_ptr_queue[i]->buffer_pool.push_back(buf);

                char tmp[SINGLE_LOG_SIZE] = {0};
                tmp[0] = 'O';
                tmp[1] = 'K';

                if (g_log_structure_ptr_queue[i]->comm_interface_ptr->send(tmp, sizeof(tmp), COMM_DONTWAIT) != 0) {
                    error("Fail to response to client '%s' received-flag",
                          g_log_structure_ptr_queue[i]->channel_name.c_str());
                }
            }
        }

        UNLOCK;
    }

    warn(" -Receive thread terminated.");
}


void io_thread(void)
{
    bool no_io_task = true;

    while(g_running) {
        no_io_task = true;
        
        LOCK;

        for (int i = 0; i < g_log_structure_ptr_queue.size(); ++i) {
            if (g_log_structure_ptr_queue[i]->buffer_pool.size() > 0) {
                vector<char*> segments;
                std::ofstream &handle = g_log_structure_ptr_queue[i]->file_handle;
                
                segments.assign(g_log_structure_ptr_queue[i]->buffer_pool.begin(),
                                g_log_structure_ptr_queue[i]->buffer_pool.end());
                g_log_structure_ptr_queue[i]->buffer_pool.clear();
                
                UNLOCK;
                
                do_io(handle, segments);
                    
                time_t time_now = time(NULL);
                g_log_structure_ptr_queue[i]->file_write_cnt += segments.size();
                
                if (g_log_structure_ptr_queue[i]->file_write_cnt > MAX_LOG_FILE_WRITE_COUNT ||
                    time_now - g_log_structure_ptr_queue[i]->file_create_time > MAX_LOG_FILE_RETENTION_TIME )
                {
                    char buf[DIRECTOTY_BUF_SIZE] = {0};
                    
                    
                    readlink("/proc/self/exe", buf, sizeof(buf));
                    
                    boost::filesystem::path path(buf);
                    path = path.parent_path().parent_path().parent_path();
                    string file = path.string() + "/log/";
                
                    memset(buf, 0, sizeof(buf));
                    
                    tm *local = localtime(&time_now);
                    strftime(buf, 64, "_%Y_%m_%d_%H_%M_%S", local);
                    
                    file = file + g_log_structure_ptr_queue[i]->channel_name + buf + ".log";

                    path = path / "/log/";
                    checkLogSpace(path);

                    g_log_structure_ptr_queue[i]->file_handle.close();
                    g_log_structure_ptr_queue[i]->log_file_name = file;
                    g_log_structure_ptr_queue[i]->file_handle.open(file.c_str(), std::ios::app);
                    
                    if (g_log_structure_ptr_queue[i]->file_handle.is_open()) {
                        g_log_structure_ptr_queue[i]->file_create_time = time_now;
                        g_log_structure_ptr_queue[i]->file_write_cnt = 0;
                    }
                }

                no_io_task = false;
                break;
            }
            /*
            while (g_log_structure_ptr_queue[i]->buffer_pool.size() > 0) {
                char *segment = g_log_structure_ptr_queue[i]->buffer_pool.front();
                g_log_structure_ptr_queue[i]->buffer_pool.erase(g_log_structure_ptr_queue[i]->buffer_pool.begin());
                g_log_structure_ptr_queue[i]->file_handle << segment;
                delete[] segment;
            }
            */
        }
        if (no_io_task) {
            UNLOCK;
            usleep(50 * 1000);
        }
    }  // while (g_running)

    usleep(200 * 1000);
    info(" -IO thread is logging the logs in buffer pool.");

    LOCK;

    for (int i = 0; i < g_log_structure_ptr_queue.size(); ++i) {
        while (g_log_structure_ptr_queue[i]->buffer_pool.size() > 0) {
                char *segment = g_log_structure_ptr_queue[i]->buffer_pool.front();
                g_log_structure_ptr_queue[i]->buffer_pool.erase(g_log_structure_ptr_queue[i]->buffer_pool.begin());
                g_log_structure_ptr_queue[i]->file_handle << segment;
                delete[] segment;
        }
        // g_log_structure_ptr_queue[i]->file_handle.close();
    }

    UNLOCK;

    while (g_log_structure_ptr_queue.size() > 0) {
        deleteLogStructure(g_log_structure_ptr_queue.front()->channel_name.c_str());
    }

    warn(" -IO thread terminated.");
}

static void sigintHandle(int num)
{
    warn("Interrupt request catched.");
    g_running = false;
    
    usleep(500 * 1000);
}

uintmax_t max(uintmax_t a, uintmax_t b)
{
    return a > b ? a : b;
}

int main(int argc, char **argv)
{
    info("Log Manager Version %d.%d.%d",  log_manager_VERSION_MAJOR,
                                            log_manager_VERSION_MINOR,
                                            log_manager_VERSION_PATCH);

    signal(SIGINT, sigintHandle);
    g_log_structure_ptr_queue.reserve(32);
    pthread_mutex_init(&g_log_structure_ptr_queue_mutex, NULL);
    
    char buf[DIRECTOTY_BUF_SIZE] = {0};
    int length = readlink("/proc/self/exe", buf, sizeof(buf));

    if (length > 0 && length < sizeof(buf)) {
        boost::filesystem::path path(buf);
        path = path.parent_path().parent_path().parent_path();
        path += "/log/";
        
        info("Log directory: %s", path.string().c_str());

        if (!boost::filesystem::is_directory(path)) {
            boost::filesystem::create_directories(path);
            warn("log directory not exist ... created");
        }

        uintmax_t used = totalLogFileSize(path);
        info("Log space usage (used / total): %dMB / %dMB",
             used / 1024 / 1024,
             max(MAX_LOG_FILE_SPACE, used) / 1024 / 1024
        );
        
        checkLogSpace(path);
        
        if (used > MAX_LOG_FILE_SPACE) {
            used = totalLogFileSize(path);
            info("Log space usage (used / total): %dMB / %dMB",
                 used / 1024 / 1024,
                 max(MAX_LOG_FILE_SPACE, used) / 1024 / 1024
            );
        }
    }
    else if (length >= sizeof(buf)) {
        buf[DIRECTOTY_BUF_SIZE] = 0;
        error("log path=%s", buf);
        error("path overflow");
        
        return -1;
    }
    else {
        error("internal fault while getting working directory");

        return -1;
    }

    boost::thread log_public(&public_thread);
    boost::thread log_receive(&receive_thread);
    boost::thread log_io(&io_thread);
    
    log_public.join();
    log_receive.join();
    log_io.join();
  
    //usleep(5000 * 1000);
    warn("Log server exit.");
    return 0;
}
