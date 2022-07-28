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
#include "interpreter_device.h"
#include "interpreter_register.h"
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
// static InterpEmbed *interp_embed_ptr = NULL;
static std::map<interpid_t, InterpEmbed *> embed_ptr_map;

InterpCtrl::InterpCtrl(/* args */)
{
    curr_state_ = INTERP_STATE_IDLE;
    is_exit_ = false;
    index_ = 0;
    sync_callbacks_.clear();
}

InterpCtrl::~InterpCtrl()
{
    is_exit_ = true;
    state_thread_.join();
    // abort(0);
    // auto iter = embed_ptr_map.begin();
    // for(; iter != embed_ptr_map.end(); ++iter)
    // {
    //     delete iter->second;
    // }
}

bool InterpCtrl::setApi(group_space::MotionControl **group_ptr, hal_space::BaseDevice *io_ptr)
{
    if(group_ptr == NULL)
    {
        LogProducer::error("interpctrl", "application interface init failed");
        return false;
    }
    InterpGroup_Init(group_ptr);
    InterpDevice_Init(io_ptr);
    InterpReg_Init();
    return true;
}

bool InterpCtrl::init(void)
{
    embed_ptr_map.clear();
    if(!config_.loadConfig())
    {
        LogProducer::error("interpctrl", "initialize failed with load configuration");
        return false;
    }
    InterpEmbed::config_ = config_;

    // if(reset() != 0) return false;

    if(!InterpEmbed::pyUpdatePath(config_.getModulePath()))
    {
        LogProducer::error("interpctrl", "initialize failed with update path");
        return false;
    }
    if(!InterpEmbed::pyResetInterp())
    {
        LogProducer::error("interpctrl", "initialize failed with reset");
        return false;
    }
    // here only create the main interpreter
    InterpEmbed *embed_ptr = new InterpEmbed(index_);
    if(embed_ptr == NULL)
    {
        LogProducer::error("interpctrl", "main interpreter object allocate failed");
        return false;
    }
    // insert to embed_ptr_map and increase the index
    embed_ptr_map.insert(std::make_pair(index_++, embed_ptr));
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
        int state = 0;
        auto iter = embed_ptr_map.begin();
        for(; iter != embed_ptr_map.end(); ++iter)
        {
            state |= iter->second->getState();
        }

        switch (state)
        {
        case 0x00: // no program in process
            curr_state_ = INTERP_STATE_IDLE;
            break;
        case 0x01: // b0001
            curr_state_ = INTERP_STATE_IDLE;
            break;
        case 0x02: // b0010
            curr_state_ = INTERP_STATE_RUNNING;
            break;
        case 0x03: // b0011 if one or more program is run state is running
            curr_state_ = INTERP_STATE_RUNNING;
            break;
        case 0x04: // b0100
            curr_state_ = INTERP_STATE_PAUSE;
            break;
        case 0x05: // b0101
            curr_state_ = INTERP_STATE_PAUSE;
            break;
        case 0x06: // b0110
            curr_state_ = INTERP_STATE_RUNNING;
            break;
        case 0x07: // b0111
            curr_state_ = INTERP_STATE_RUNNING;
            break;
        default:
            curr_state_ = INTERP_STATE_UNKNOWN;
            break;
        }

        usleep(config_.stateCycleTime());
    }
}

bool InterpCtrl::run(void)
{
    bool ret = false;//, ret2 = false;
    LogProducer::info("interpctrl", "starting interpreter");
    ret = state_thread_.run(interp_state_thread_func, this, config_.stateThreadPriority());
    LogProducer::info("interpctrl", "interpreter started");
    return ret;
}

ErrorCode InterpCtrl::start(const std::string& prog)
{
    if(curr_state_ != INTERP_STATE_IDLE)
    {
        LogProducer::error("interpctrl", "can only start in IDLE state, currrent %d", curr_state_);
        return INTERPRETER_ERROR_INVALID_STATE;
    }

    embed_ptr_map[0]->setMainName(prog);

    return embed_ptr_map[0]->start();
}

ErrorCode InterpCtrl::pause(interpid_t id)
{
    ErrorCode ret = 0;
    if(!checkValid(id)) return INTERPRETER_ERROR_INVALID_ARG;
    if(id == 0) // pause all
    {
        auto iter = embed_ptr_map.begin();
        for(; iter != embed_ptr_map.end(); ++iter)
        {
            ret = iter->second->pause();
        }
    }
    else
    {
        ret = embed_ptr_map[id]->pause();
    }

    return ret;
}

ErrorCode InterpCtrl::resume(interpid_t id)
{
    ErrorCode ret = 0;
    if(!checkValid(id)) return INTERPRETER_ERROR_INVALID_ARG;
    if(id == 0) // resume all
    {
        auto iter = embed_ptr_map.begin();
        for(; iter != embed_ptr_map.end(); ++iter)
        {
            ret = iter->second->resume();
        }
    }
    else
    {
        ret = embed_ptr_map[id]->resume();
    }
    return ret;
}

ErrorCode InterpCtrl::abort(interpid_t id)
{
    ErrorCode ret = 0;
    if(!checkValid(id)) return INTERPRETER_ERROR_INVALID_ARG;
    if(id == 0) // abort all
    {
        auto iter = embed_ptr_map.crbegin();
        for(; iter != embed_ptr_map.crend(); ++iter)
        {
            ret = iter->second->abort();
        }
        // InterpEmbed::pyResetInterp();/* for reload program file */
    }
    else
    {
        ret = embed_ptr_map[id]->abort();
    }
    return ret;
}

ErrorCode InterpCtrl::forward(interpid_t id)
{
    ErrorCode ret = 0;
    if(id == 0) // forward all
    {
        auto iter = embed_ptr_map.begin();
        for(; iter != embed_ptr_map.end(); ++iter)
        {
            ret = iter->second->forward();
        }
    }
    else
    {
        ret = embed_ptr_map[id]->forward();
    }
    return ret;
}

ErrorCode InterpCtrl::backward(interpid_t id)
{
    LogProducer::info("interpctrl", "backward not support yet");
    ErrorCode ret = 0;
    if(id == 0) // backward all
    {
        auto iter = embed_ptr_map.begin();
        for(; iter != embed_ptr_map.end(); ++iter)
        {
            ret = iter->second->backward();
        }
    }
    else
    {
        ret = embed_ptr_map[id]->backward();
    }
    return ret;
}

// setjumpline -> jump
ErrorCode InterpCtrl::jumpLine(interpid_t id, int line)
{
    if(!checkValid(id)) return INTERPRETER_ERROR_INVALID_ARG;

    LogProducer::info("interpctrl", "jumpLine not support yet");
    ErrorCode ret = 0;
    if(id == 0) // resume all
    {
        auto iter = embed_ptr_map.begin();
        for(; iter != embed_ptr_map.end(); ++iter)
        {
            ret = iter->second->jump(line);
        }
    }
    else
    {
        ret = embed_ptr_map[id]->jump(line);
    }
    return ret;
}

bool InterpCtrl::regSyncCallback(const SyncCallback& callback)
{
    if(curr_state_ != INTERP_STATE_IDLE) return false;

    sync_callbacks_.push_back(callback);
    return true;
}

bool InterpCtrl::runSyncCallback(interpid_t id)
{
    LogProducer::info("interpctrl", "start sync calls all %d functions", sync_callbacks_.size());
    auto it = sync_callbacks_.begin();
    for(int i = 0; it != sync_callbacks_.end(); ++it, ++i)
    {
        LogProducer::info("interpctrl", "call sync function %d", i);
        while(!(*it)()) // motion control nextMovePerimit
        {
            if(embed_ptr_map[id]->getAbortFlag() || is_exit_)
            {
                LogProducer::info("interpctrl", "sync function %d call break", i);
                return false;
            }
            usleep(1000);
        }
        LogProducer::info("interpctrl", "sync function %d called", i);
    }

    return true;
}

bool InterpCtrl::checkValid(interpid_t id)
{
    if(embed_ptr_map.find(id) == embed_ptr_map.end())
    {
        LogProducer::warn("interpctrl", "ID[%lu] of interpreter not found", id);
        return false;
    }
    return true;
}

InterpState InterpCtrl::getState(interpid_t id)
{
    if(!checkValid(id)) return INTERP_STATE_UNKNOWN;

    return embed_ptr_map[id]->getState();
}

std::string InterpCtrl::getProgName(interpid_t id)
{
    if(!checkValid(id)) return "##unknown##";

    return embed_ptr_map[id]->getMainName();
}

ErrorCode InterpCtrl::startNewFile(std::string file, bool in_real_thread)
{
    InterpEmbed *embed_ptr = new InterpEmbed(index_);
    if(embed_ptr == NULL)
    {
        LogProducer::error("interpctrl", "new interpreter object allocate failed");
        return INTERPRETER_ERROR_MEM_ALLOCATE_FAILED;
    }
    embed_ptr->setMainName(file);

    if(!embed_ptr->pyStartThread())
    {
        delete embed_ptr;
        LogProducer::error("interpctrl", "new interpreter start failed");
        return INTERPRETER_ERROR_CREATE_SUB_FAILED;
    }
    // insert to embed_ptr_map and increase the index
    embed_ptr_map.insert(std::make_pair(index_++, embed_ptr));

    return 0;
}

ErrorCode InterpCtrl::startNewFunc(void *pyfunc, bool in_real_thread)
{
    return 0;
}

ErrorCode InterpCtrl::delay(double seconds)
{
    if(!runSyncCallback())
        return INTERPRETER_ERROR_SYNC_CALL_FAILED;

    uint32_t delay_time = (uint32_t)(seconds * 1000 * 1000);

    usleep(delay_time);

    return 0;
}

ErrorCode InterpCtrl::reset(void)
{
    if(curr_state_ != INTERP_STATE_IDLE)
    {
        LogProducer::error("interpctrl", "try reset interpreter control failed need state %d, current %d", INTERP_STATE_IDLE, curr_state_);
        return INTERPRETER_ERROR_RESET_FAILED;
    }

    if(!InterpEmbed::pyUpdatePath(config_.getModulePath()))
    {
        LogProducer::error("interpctrl", "reset failed with update path");
        return INTERPRETER_ERROR_RESET_FAILED;
    }
    if(!InterpEmbed::pyResetInterp())
    {
        LogProducer::error("interpctrl", "reset failed with reset");
        return INTERPRETER_ERROR_RESET_FAILED;
    }

    return 0;
}
