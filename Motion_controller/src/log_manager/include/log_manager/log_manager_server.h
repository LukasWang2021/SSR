/*************************************************************************
	> File Name: log_manager_server.h
	> Author: 
	> Mail: 
	> Created Time: 2016年12月27日 星期二 14时08分56秒
 ************************************************************************/

#ifndef _LOG_MANAGER_SERVER_H
#define _LOG_MANAGER_SERVER_H
#include <fstream>
#include <string>

#include <log_manager/log_manager_common.h>

namespace fst_log {

struct LogServerStruct {
    std::string         channel_name;
    std::string         log_file_name;
    bool                open_flag;
    time_t              file_create_time;
    int                 file_write_cnt;
    std::ofstream       file_handle;
    std::vector<char*>  buffer_pool;
    fst_comm_interface::CommInterface *comm_interface_ptr;
};

}
#define DIRECTOTY_BUF_SIZE 256

#define LOCK    pthread_mutex_lock(&g_log_structure_ptr_queue_mutex);
#define UNLOCK  pthread_mutex_unlock(&g_log_structure_ptr_queue_mutex);

uintmax_t totalLogFileSize(boost::filesystem::path &path);
uintmax_t delOldestLogFile(boost::filesystem::path &path);
void checkLogSpace(boost::filesystem::path &path);
bool buildLogStructure(const char *channel_name);
void deleteLogStructure(const char *channel_name);
void do_io(std::ofstream& handle, std::vector<char*> segments);

void public_thread(void);
void receive_thread(void);
void io_thread(void);

uintmax_t max(uintmax_t a, uintmax_t b);

#endif
