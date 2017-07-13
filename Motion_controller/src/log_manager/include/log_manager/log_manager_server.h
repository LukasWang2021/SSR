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
    bool                open_flag;
    std::string         channel_name;
    std::ofstream       file_handle;
    std::vector<char*>  buffer_pool;
    fst_comm_interface::CommInterface *comm_interface_ptr;
};

}

#endif
