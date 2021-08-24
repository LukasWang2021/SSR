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

// PyThreadState* Py_NewInterpreter()
// uint64_t PyThreadState_GetID(PyThreadState *tstate)

// PyInterpreterState* PyInterpreterState_New()
// PyInterpreterState* PyInterpreterState_Get(void)
// int64_t PyInterpreterState_GetID(PyInterpreterState *interp)
class InterpEmbed
{
private:
    PyObject *trace_obj_;

public:
    InterpEmbed(/* args */);
    ~InterpEmbed();

    bool pyResetInterp(void); /*reset the python interpreter*/
    
    bool pyUpdatePath(std::string path); /*update the python module search path specified by config file*/

    void pyRunFile(const std::string& file); /*run the python script 'file'.
    The file is search from the path specified by configuration file 'program_path' item*/

    void pyRunString(const std::string& str); /*run the python script 'str'*/

    void pyErrTraceBack(void); /* error trace of python */

    bool pySetTrace(interpid_t idx, InterpEmbed *selfobj);

private:
    static InterpConfig config_;

private:
    base_space::ThreadHelp prog_thread_;
    interpid_t id_;
    bool is_paused_;
    bool is_abort_;
    bool is_exit_;
    base_space::SemHelp *exec_sem_ptr_;

    InterpMode curr_mode_;
    InterpState curr_state_;

    /*Current line number of current program*/
    std::string main_prog_;
    std::string main_path_name_;
    int curr_line_;
    std::string curr_prog_;
    std::string curr_func_;

public:
    int hold(void);
    int release(void);
    void progThreadFunc(void);

    ErrorCode start(std::string prog);
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
    std::string getMainName(void) { return main_prog_; }
};


#endif

