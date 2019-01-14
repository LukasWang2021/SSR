#include "interpreter_server.h"

using namespace fst_base;

bool InterpreterServer::addPublishTask(int interval, InterpreterPublish* data_ptr)
{
    if(interval < (param_ptr_->interpreter_server_cycle_time_/1000)
        || data_ptr == NULL)
    {
        return false;
    }
        
    ProcessCommPublish task;
    task.interval = interval;
    task.data_ptr = data_ptr;
    task.last_publish_time.tv_sec = 0;
    task.last_publish_time.tv_usec = 0;
    publish_list_mutex_.lock();
    publish_list_.push_back(task);
    publish_list_mutex_.unlock();
    return true;
}

void InterpreterServer::removePublishTask()
{
    publish_list_mutex_.lock();
    publish_list_.clear();
    publish_list_mutex_.unlock();
}

