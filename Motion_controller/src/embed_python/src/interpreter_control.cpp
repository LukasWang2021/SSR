/* 2021.04.27 First try to embed the python to RTM-robot system
 * 2021.05.17 Learn how to create module
 *            Sub-inerpreter test
 * 2021.05.25 Multi-thread test
 * 2021.05.26 threading for multi-interpreter test
 * 2021.06.02 cross compile enviroment
 * 2021.06.09 axis module demo and cross compile for module
 * 2021.06.15 axis control wrapper
 * 2021.06.18 run python script success in embed-robot-system
 * 2021.06.22 make cross compile enviroment auto build and install
 *            rewrite some classes
 * 2021.06.23 rotor jump solution about the user-program
 *            axis module interface complete
 * 2021.06.24 group module interface complete
 * 2021.06.28 test module for interfaces----to be continue
 * 2021.06.28 io device module
 */
#define PY_SSIZE_T_CLEAN
#include "Python.h"

#include "common_error_code.h"
#include "interpreter_control.h"
#include <unistd.h>
#include "sem_help.h"
#include "interpreter_embed.h"
#include "error_queue.h"
#include "interpreter_group.h"

using namespace log_space;
using namespace base_space;

static uint32_t g_isr_ptr;

// self instance
InterpCtrl InterpCtrl::interp_ctrl_;

static InterpEmbed *interp_embed_ptr = NULL;

InterpCtrl::InterpCtrl(/* args */)
{
    curr_line_ = 0;
    curr_prog_ = "";
    curr_state_ = INTERP_IDLE;
    curr_mode_ = INTERP_AUTO;
    sync_callbacks_.clear();
}

InterpCtrl::~InterpCtrl()
{
    if(state_sem_ptr_ != NULL)
        delete state_sem_ptr_;

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

    state_sem_ptr_ = new SemHelp();
    prog_sem_ptr_ = new SemHelp(0, 0);
    if(state_sem_ptr_ == NULL || prog_sem_ptr_ == NULL)
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

    if(!interp_embed_ptr->pyResetInterp())
    {
        LogProducer::error("interpctrl", "initialize failed with reset");
        return false;
    }

    is_init_ = true;

    return true;
}

static void* interp_prog_thread_func(void* arg)
{
    InterpCtrl* interp_ctrl = static_cast<InterpCtrl*>(arg);
    log_space::LogProducer log_manager;
    log_manager.init("interpctrl", &g_isr_ptr);

    LogProducer::info("interpctrl", "enter interpreter");

    interp_ctrl->interpProgThreadFunc();

    LogProducer::info("interpctrl", "interpreter exit");

	return NULL;
}

static void* interp_state_thread_func(void* arg)
{
    InterpCtrl* interp_ctrl = static_cast<InterpCtrl*>(arg);
    log_space::LogProducer log_manager;
    log_manager.init("interpstate", &g_isr_ptr);

    interp_ctrl->interpStateThreadFunc();

	return NULL;
}

void InterpCtrl::interpProgThreadFunc(void)
{
    for(;;)
    {
        LogProducer::info("interpctrl", "interpreter waitting program");
        waitStart();
        LogProducer::info("interpctrl", "interpreter recieved program %s", curr_prog_.c_str());
        interp_embed_ptr->pyRunFile(curr_prog_);
        LogProducer::info("interpctrl", "program %s exit", curr_prog_.c_str());
        curr_state_ = INTERP_IDLE;
    }
}

void InterpCtrl::interpStateThreadFunc(void)
{
    for(;;)
    {
        sleep(1);
    }
}

bool InterpCtrl::run(void)
{
    bool ret1 = false, ret2 = false;
    LogProducer::info("interpctrl", "starting interpreter");
    ret1 = interp_thread_.run(interp_state_thread_func, this, config_.getThreadPriority());
    ret2 = interp_thread_.run(interp_prog_thread_func, this, config_.getThreadPriority());
    LogProducer::info("interpctrl", "interpreter started");

    return ret1 && ret2;
}

ErrorCode InterpCtrl::start(const std::string& prog)
{
    // only in idle state
    if(curr_state_ != INTERP_IDLE)
    {
        LogProducer::error("interpctrl", "start program %s failed not in IDLE state", curr_prog_.c_str());
        return INTERPRETER_ERROR_INVALID_STATE;
    }

    curr_prog_ = config_.getProgPath() + prog;
    LogProducer::info("interpctrl", "start program %s", curr_prog_.c_str());

    curr_state_ = INTERP_RUNNING;
    prog_sem_ptr_->give();

    return 0;
}

ErrorCode InterpCtrl::pause(interpid_t id)
{
    if(curr_state_ != INTERP_RUNNING)
    {
        LogProducer::error("interpctrl", "pause program %d failed not in RUNNING state", id);
        return INTERPRETER_ERROR_INVALID_STATE;
    }

    LogProducer::info("interpctrl", "pause program %d", id);

    if(hold(id) != 0)
    {
        LogProducer::error("interpctrl", "program %d paused failed while holding", id);
        return INTERPRETER_ERROR_HOLD_FAILED;
    }

    curr_state_ = INTERP_PAUSE;

    LogProducer::info("interpctrl", "program %d paused", id);

    return 0;
}

ErrorCode InterpCtrl::resume(interpid_t id)
{
    if(curr_state_ != INTERP_PAUSE)
    {
        LogProducer::error("interpctrl", "resume program %d failed not in PAUSE state", id);
        return INTERPRETER_ERROR_INVALID_STATE;
    }

    LogProducer::info("interpctrl", "resume program %d", id);

    if(release() != 0)
    {
        LogProducer::error("interpctrl", "program %d resume failed while releasing", id);
        return INTERPRETER_ERROR_RELEASE_FAILED;
    }

    curr_state_ = INTERP_RUNNING;

    LogProducer::info("interpctrl", "program %d resumed", id);

    return 0;
}

ErrorCode InterpCtrl::abort(interpid_t id)
{
    if(curr_state_ == INTERP_IDLE)
    {
        LogProducer::warn("interpctrl", "abort in state IDLE ignored");
        return 0;
    }

    LogProducer::info("interpctrl", "abort program %d", id);

    if(release() != 0)
    {
        LogProducer::error("interpctrl", "program %d abort failed while releasing", id);
        return INTERPRETER_ERROR_RELEASE_FAILED;
    }
    
    curr_state_ = INTERP_IDLE;

    LogProducer::info("interpctrl", "program %d aborted", id);

    return 0;
}

ErrorCode InterpCtrl::forward(interpid_t id)
{
    LogProducer::info("interpctrl", "program %d forward execution", id);

    if(curr_mode_ != INTERP_STEP)
    {
        LogProducer::error("interpctrl", "program %d forward execution failed not in STEP mode", id);
        return INTERPRETER_ERROR_INVALID_MODE;
    }

    if(curr_state_ != INTERP_PAUSE)
    {
        LogProducer::error("interpctrl", "program %d forward execution failed not in PAUSE state", id);
        return INTERPRETER_ERROR_INVALID_STATE;
    }

    return 0;
}

ErrorCode InterpCtrl::backward(interpid_t id)
{
    LogProducer::info("interpctrl", "backward not support yet");
    return 0;
}

ErrorCode InterpCtrl::jumpLine(interpid_t id)
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

bool InterpCtrl::runSyncCallback(void)
{
    bool ret = true;
    auto it = sync_callbacks_.begin();
    for(; it != sync_callbacks_.end(); ++it)
    {
        ret &= (*it)();
    }

    return ret;
}

int InterpCtrl::hold(interpid_t id)
{
    // LogProducer::info("interpctrl", "try to hold interpreter control of %d", id);
    // semphore take is has ben hold by another this will block
    int ret = state_sem_ptr_->take();
    // LogProducer::info("interpctrl", "interpreter control of %d held return %d", id, ret);
    return ret;
}

int InterpCtrl::release(interpid_t id)
{
    // LogProducer::info("interpctrl", "try to release interpreter control of %d", id);
    int ret = 0;
    // state semphore give
    if(state_sem_ptr_->isTaken())
        ret = state_sem_ptr_->give();
    // LogProducer::info("interpctrl", "interpreter control of %d release return %d", id, ret);
    return ret;
}

void InterpCtrl::waitStart(void)
{
    prog_sem_ptr_->take();
}
