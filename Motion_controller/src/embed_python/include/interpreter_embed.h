/* This module provide the interface to option
 * python enviroment or call python interface
 */

#ifndef INTERPRETER_EMBED_H
#define INTERPRETER_EMBED_H

#define PY_SSIZE_T_CLEAN
#include "Python.h"
#include "string"
#include "sem_help.h"
#include "interpreter_control.h"
#include "thread_help.h"

class InterpEmbed
{
private:
    PyObject      *trace_obj_;
    PyThreadState *self_ts_;

public:
    InterpEmbed(interpid_t id=0, std::string prog="");
    ~InterpEmbed();

public:
    static InterpConfig config_;

    /* update the python module search path specified by config file */
    static bool pyUpdatePath(std::string path);

    /* reset the all python interpreter */
    static bool pyResetInterp(void);

    bool pySetTrace(void);

    void pyRunFile(void); /* run the python script 'file'.
    The file is search from the path specified by configuration file 'program_path' item */

    /* start a new file :
       in real means this file will run in a real thread, 
       if false will run in a virtual thread
    */
    bool pyStartThread(void);

private:
    base_space::SemHelp *exec_sem_ptr_;
    base_space::SemHelp *exit_sem_ptr_;
    base_space::ThreadHelp prog_thread_;
    interpid_t id_;     // self index set by interpreter control
    interpid_t pyid_;   // self index create by python
    pthread_t  thread_id_;
    bool is_paused_;
    bool is_abort_;
    bool is_exit_;
    bool in_real_; // start with real thread or not

    InterpMode curr_mode_;
    InterpState curr_state_;

    /*Current line number of current program*/
    std::string main_prog_;      // xxx.py
    std::string main_path_name_; // robotdata/python/xxx.py
    int curr_line_;
    std::string curr_prog_;
    std::string curr_func_;

public:
    bool init(void);
    int hold(void);
    int release(void);
    void waitAborted(void);
    void progThreadFunc(void);
    void pyThreadFunc(void);

    ErrorCode start(void);
    ErrorCode pause(void);
    ErrorCode resume(void);
    ErrorCode abort(void);
    ErrorCode forward(void);
    ErrorCode backward(void);
    ErrorCode jump(int line);

    void setPauseFlag(bool flag) { is_paused_ = flag; }
    void setAbortFlag(bool flag) { is_abort_ = flag; }
    bool getPauseFlag(void) { return is_paused_; }
    bool getAbortFlag(void) { return is_abort_; }

    InterpMode getMode(void) { return curr_mode_; }
    InterpState getState(void) { return curr_state_; }
    void setMode(InterpMode mode) { curr_mode_ = mode; }
    void setState(InterpState state) { curr_state_ = state; }

    int getCurrLine(void) { return curr_line_; }
    std::string getCurrProg(void) { return curr_prog_; }
    std::string getCurrFunc(void) { return curr_func_; }

    void setCurrLine(int line) { curr_line_ = line; }
    void setCurrProg(std::string prog) { curr_prog_ = prog; }
    void setCurrFunc(std::string func) { curr_func_ = func; }

    interpid_t getId(void) { return id_; }
    interpid_t getPyId(void) { return pyid_; }

    std::string getMainName(void) { return main_prog_; }
    void setMainName(std::string name)
    {
        main_prog_ = name;
        main_path_name_ = config_.getProgPath() + main_prog_;
    }
private:
    bool inThreadInit(void);
};


#endif

