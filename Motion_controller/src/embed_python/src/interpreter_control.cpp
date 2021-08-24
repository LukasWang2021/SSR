/* 2021.04.27 First try to embed the python to RTM-robot system
 * 2021.06.02 cross compile enviroment
 * 2021.06.09 axis module demo and cross compile for module
 * 2021.06.15 axis control wrapper
 * 2021.06.22 make cross compile enviroment auto build and install
 * 2021.06.24 support group module
 * 2021.06.28 support io device module
 * 2021.08.13 support motion module
 * 2021.08.18 fix thread exit bug for log
 * 2021.08.23 modified class for multi-thread
 */
#define PY_SSIZE_T_CLEAN
#include "Python.h"
#include "common_error_code.h"
#include "interpreter_control.h"
#include <unistd.h>
#include <sys/syscall.h>
#include "sem_help.h"
#include "interpreter_embed.h"
#include "error_queue.h"
#include "interpreter_group.h"

using namespace log_space;
using namespace base_space;

// self instance definition
InterpCtrl InterpCtrl::interp_ctrl_;

/* a global object not a member of InterpCtrl,
   it is to avoid modify other modules' compile path */
static InterpEmbed *interp_embed_ptr = NULL;

InterpCtrl::InterpCtrl(/* args */)
{
    curr_prog_ = "";
    curr_state_ = INTERP_IDLE;
    is_exit_ = false;
    sync_callbacks_.clear();
}

InterpCtrl::~InterpCtrl()
{
    is_exit_ = true;
    state_thread_.join();

    if(interp_embed_ptr != NULL)
        delete interp_embed_ptr;
}

bool InterpCtrl::setApi(group_space::MotionControl **group_ptr)
{
    if(group_ptr == NULL)
    {
        LogProducer::error("interpctrl", "application interface init failed");
        return false;
    }

    InterpGroup_Init(group_ptr);
    
    return true;
}

bool InterpCtrl::init(void)
{
    if(is_init_) return true;

    prog_sem_ptr_ = new SemHelp(0, 0);
    if(prog_sem_ptr_ == NULL)
    {
        LogProducer::error("interpctrl", "control semphore allocate failed");
        return false;
    }
    
    interp_embed_ptr = new InterpEmbed();
    if(interp_embed_ptr == NULL)
    {
        LogProducer::error("interpctrl", "embed object allocate failed");
        return false;
    }

    if(!config_.loadConfig())
    {
        LogProducer::error("interpctrl", "initialize failed with load configuration");
        return false;
    }

    if(!interp_embed_ptr->pyUpdatePath(config_.getModulePath()))
    {
        LogProducer::error("interpctrl", "initialize failed with update path");
        return false;
    }

    if(!interp_embed_ptr->pyResetInterp())
    {
        LogProducer::error("interpctrl", "initialize failed with reset");
        return false;
    }

    if(!interp_embed_ptr->pySetTrace(0, interp_embed_ptr))
    {
        LogProducer::error("interpctrl", "initialize failed with set trace");
        return false;
    }

    is_init_ = true;

    return true;
}

static void* interp_state_thread_func(void* arg)
{
    uint32_t isr_ptr = 0;
    InterpCtrl* interp_ctrl = static_cast<InterpCtrl*>(arg);
    log_space::LogProducer log_manager;
    log_manager.init("interpstate", &isr_ptr);
    LogProducer::warn("interpstate", "interpreter state thread TID is %ld", syscall(SYS_gettid));
    interp_ctrl->stateThreadFunc();
    std::cout<<"interp_state_thread exit"<<std::endl;

	return NULL;
}

void InterpCtrl::stateThreadFunc(void)
{
    while(!is_exit_)
    {
        sleep(1);
    }
}

bool InterpCtrl::run(void)
{
    bool ret = false;
    LogProducer::info("interpctrl", "starting interpreter");
    ret = state_thread_.run(interp_state_thread_func, this, config_.getThreadPriority());
    LogProducer::info("interpctrl", "interpreter started");
    return ret;
}

ErrorCode InterpCtrl::start(const std::string& prog)
{
    ErrorCode ret = interp_embed_ptr->start(prog);
    curr_state_ = INTERP_RUNNING;
    return ret;
}

ErrorCode InterpCtrl::pause(interpid_t id)
{
    ErrorCode ret = interp_embed_ptr->pause();
    curr_state_ = INTERP_PAUSE;
    return ret;
}

ErrorCode InterpCtrl::resume(interpid_t id)
{
    ErrorCode ret = interp_embed_ptr->resume();
    curr_state_ = INTERP_RUNNING;
    return ret;
}

ErrorCode InterpCtrl::abort(interpid_t id)
{
    ErrorCode ret = interp_embed_ptr->abort();
    curr_state_ = INTERP_IDLE;
    return ret;
}

ErrorCode InterpCtrl::forward(interpid_t id)
{
    return interp_embed_ptr->forward();
}

ErrorCode InterpCtrl::backward(interpid_t id)
{
    LogProducer::info("interpctrl", "backward not support yet");
    return 0;
}

ErrorCode InterpCtrl::jumpLine(interpid_t id, int line)
{
    LogProducer::info("interpctrl", "jumpLine not support yet");
    return 0;
}

bool InterpCtrl::regSyncCallback(const SyncCallback& callback)
{
    if(curr_state_ != INTERP_IDLE) return false;

    sync_callbacks_.push_back(callback);
    
    return true;
}

bool InterpCtrl::runSyncCallback(interpid_t id)
{
    bool ret = true;
    LogProducer::info("interpctrl", "start sync calls all %d functions", sync_callbacks_.size());
    auto it = sync_callbacks_.begin();
    for(int i = 0; it != sync_callbacks_.end(); ++it, ++i)
    {
        LogProducer::info("interpctrl", "call sync function %d", i);
        while(!(*it)() && ret) // motion control nextMovePerimit
        {
            if(interp_embed_ptr->getAbortFlag() || is_exit_)
            {
                LogProducer::info("interpctrl", "sync function %d call break", i);
                return false;
            }
            usleep(1000);
        }
        LogProducer::info("interpctrl", "sync function %d called", i);
    }

    return ret;
}

InterpState InterpCtrl::getState(interpid_t id)
{
    return interp_embed_ptr->getState();
}

std::string InterpCtrl::getProgName(interpid_t id)
{
    return interp_embed_ptr->getMainName();
}
