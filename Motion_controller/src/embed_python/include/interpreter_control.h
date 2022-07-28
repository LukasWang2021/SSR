/* This module provide the interface to control the interpreter's action.
 * Start,pasue,resume etc. Not support jump option for now.
 * forward option: step
 * backward option: option stack
 * !!! should support multi-thread
 */

#ifndef INTERPRETER_CONTROL_H
#define INTERPRETER_CONTROL_H

#include <string>
#include <functional>
#include <vector>
#include "axis.h"
#include "group.h"
#include "interpreter_config.h"
#include "thread_help.h"
#include <pthread.h>
#include "sem_help.h"
#include "motion_control.h"
#include "io_1000.h"

typedef uint64_t interpid_t;

typedef enum
{
    INTERP_STATE_IDLE = 0x01,
    INTERP_STATE_RUNNING = 0x02,
    INTERP_STATE_PAUSE = 0x04,
    INTERP_STATE_UNKNOWN = 0x08,
}InterpState;

typedef enum
{
    INTERP_MODE_AUTO = 0x01,
    INTERP_MODE_STEP = 0x02,
    INTERP_MODE_JUMP = 0x04,
    INTERP_MODE_UNKNOWN = 0x08,
}InterpMode;

class InterpCtrl
{
public:
    typedef std::function<bool(void)> SyncCallback;

private:
    static InterpCtrl interp_ctrl_;

    InterpState curr_state_;
    /*For synchronous with other thread.
    Register the sync-function use regSyncCallback function bellow.
    The sync-function will call in the trace function in every line execution*/
    std::vector<SyncCallback> sync_callbacks_;

    InterpConfig config_;
    bool is_init_;
    bool is_exit_;
    bool is_abort_;
    /* the interpreter object index, 0 means the main */
    interpid_t index_;

private:
    base_space::ThreadHelp state_thread_;

public:
    ~InterpCtrl();
    static InterpCtrl& instance(){ return interp_ctrl_; }

    // initialization, pls init api before init
    // bool setApi(axis_space::Axis **axis_ptr);
    bool init(void);
    bool setApi(group_space::MotionControl **group_ptr, hal_space::BaseDevice *io_ptr);
    bool run(void);

    ErrorCode startNewFile(std::string file, bool in_real_thread=true);
    ErrorCode startNewFunc(void *pyfunc, bool in_real_thread=true);

    // start and quit
    ErrorCode start(const std::string& prog);
    ErrorCode abort(interpid_t id=0);
    // interactive control
    ErrorCode pause(interpid_t id=0);
    ErrorCode resume(interpid_t id=0);
    ErrorCode forward(interpid_t id=0);
    // jump mode
    ErrorCode backward(interpid_t id=0);
    ErrorCode jumpLine(interpid_t id=0, int line=-1);

    ErrorCode reset(void);

    InterpState getState(interpid_t id=0);
    std::string getProgName(interpid_t id=0);

    /* These synchronous functions registered by caller.
       These functions must return bool(true/fase).*/
    bool regSyncCallback(const SyncCallback& callback);
    bool runSyncCallback(interpid_t id=0);

    void stateThreadFunc(void);

public:
    ErrorCode delay(double seconds);

private:
    InterpCtrl(/* args */);
    bool checkValid(interpid_t id);
};

#endif