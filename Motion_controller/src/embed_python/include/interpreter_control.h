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

typedef pthread_t interpid_t;

typedef enum
{
    INTERP_IDLE,
    INTERP_RUNNING,
    INTERP_PAUSE,
    INTERP_WAITING,
    INTERP_ERROR,
}InterpState;

typedef enum
{
    INTERP_AUTO,
    INTERP_STEP,
    INTERP_JUMP,
}InterpMode;

class InterpCtrl
{
public:
    typedef std::function<bool(void)> SyncCallback;

private:
    static InterpCtrl interp_ctrl_;

    /*Current running program*/
    std::string curr_prog_;
    /*Current line number of current program*/
    int curr_line_;
    InterpMode curr_mode_;
    InterpState curr_state_;
    /*For synchronous with other thread.
    Register the sync-function use regSyncCallback function bellow.
    The sync-function will call in the trace function in every line execution*/
    std::vector<SyncCallback> sync_callbacks_;

    InterpConfig config_;
    bool is_init_;
    bool is_paused_;
    bool is_aborted_;

private:
    base_space::ThreadHelp interp_thread_;
    base_space::SemHelp *state_sem_ptr_;
    base_space::SemHelp *prog_sem_ptr_;
    ErrorCode curr_err_; // for state machine

public:
    ~InterpCtrl();
    static InterpCtrl& instance(){ return interp_ctrl_; }

    // initialization, pls init api before init
    // bool setApi(axis_space::Axis **axis_ptr);
    bool init(void);
    bool setApi(group_space::MotionControl **group_ptr);
    bool run(void);
    void errorSet(ErrorCode err);

    // start and quit
    ErrorCode start(const std::string& prog);
    ErrorCode abort(interpid_t id=0);
    // interactive control
    ErrorCode pause(interpid_t id=0);
    ErrorCode resume(interpid_t id=0);
    ErrorCode forward(interpid_t id=0);
    // jump mode
    ErrorCode backward(interpid_t id=0);
    ErrorCode jumpLine(interpid_t id=0);

    // interpreter state
    InterpState getState(interpid_t id=0){ return curr_state_; }
    ErrorCode setState(InterpState state, interpid_t id=0){ curr_state_ = state; return 0; }
    // interpreter mode
    ErrorCode setMode(InterpMode mode, interpid_t id=0){ curr_mode_ = mode; return 0; }
    InterpMode getMode(interpid_t id=0){ return curr_mode_; }
    // curreen running program
    std::string getProgName(interpid_t id=0){ return curr_prog_; }
    /*Current line number of current program*/
    int getLineNumber(interpid_t id=0){ return curr_line_; };

    int hold(interpid_t id=0);
    int release(interpid_t id=0);

    /* These synchronous functions registered by caller.
       These functions must return bool(true/fase).*/
    bool regSyncCallback(const SyncCallback& callback);
    bool runSyncCallback(void);

    void progThreadFunc(void);
    void stateThreadFunc(void);

    bool isPause(int64_t idx);
    bool isAbort(int64_t idx);
private:
    InterpCtrl(/* args */);

    void waitStart(void);
};

#endif