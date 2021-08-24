#define PY_SSIZE_T_CLEAN
#include "Python.h"

#include "common_error_code.h"
#include "interpreter_embed.h"
#include "log_manager_producer.h"
#include "interpreter_control.h"
#include "error_queue.h"
#include <sys/syscall.h>

using namespace log_space;
using namespace base_space;

InterpConfig InterpEmbed::config_;

InterpEmbed::InterpEmbed(/* args */)
{
    exec_sem_ptr_ = new SemHelp(0, 1);
    if(exec_sem_ptr_ == NULL)
        LogProducer::error("interpembed","semphore alocate failed");
    curr_mode_ = INTERP_AUTO;
    curr_state_ = INTERP_IDLE;
    id_ = 0;
    is_paused_ = false;
    is_abort_ = false;
    is_exit_ = false;
    curr_line_ = 0;
    curr_prog_ = "";
    curr_func_ = "";
}

InterpEmbed::~InterpEmbed()
{
    is_abort_ = true;
    is_exit_ = true;
    prog_thread_.join();
    if(exec_sem_ptr_ != NULL)
        delete exec_sem_ptr_;
}

// the parameter 'obj' is passed by PyEval_SetTrace
static int tracer(PyObject *obj, PyFrameObject *frame, int what, PyObject *arg)
{
    Py_buffer obj_buff;
    if(PyObject_GetBuffer(obj, &obj_buff, PyBUF_SIMPLE))
    {
        PyErr_SetString(PyExc_EOFError, "trace object parse error");
        LogProducer::warn("interpembed","interpreter trace object parse failed");
        PyBuffer_Release(&obj_buff);
        return -1;
    }
    InterpEmbed *embed_ptr = (InterpEmbed *)obj_buff.buf;
    PyBuffer_Release(&obj_buff);
    if(embed_ptr->getAbortFlag())
    {
        PyErr_SetString(PyExc_EOFError, "user call abort to exit");
        LogProducer::warn("interpembed","interpreter stop signal recieved");
        return -1;
    }

    // exec sem take
    embed_ptr->hold();

    // update execution info, python system lib will ignored
    PyCodeObject *code = PyFrame_GetCode(frame);
    const char *filename = PyUnicode_AsUTF8(code->co_filename);
    const char *funcname = PyUnicode_AsUTF8(code->co_name);
    int lineno = PyFrame_GetLineNumber(frame);
    if(strstr(filename, "/root/robot_data/python") != NULL ||
       strstr(filename, "/root/install/lib/module") != NULL)
    {
        embed_ptr->setCurrLine(lineno);
        embed_ptr->setCurrProg(filename);
        embed_ptr->setCurrFunc(funcname);
        LogProducer::info("interpembed", "program(%s) line[%d]", filename, lineno);
    }

    embed_ptr->release();

    // if step mode goto pause
    if(embed_ptr->getMode() != INTERP_AUTO)
        embed_ptr->pause();

    return 0;
}

bool InterpEmbed::pyResetInterp(void)
{
    // uninitialize the python interpreter
    Py_Finalize();
    /* prameter 0 means initialize the python interpreter and skip sighandler
       if it's 1 some mistake will happen when ctrl-c the controller process */
    Py_InitializeEx(0);

    return true;
}

bool InterpEmbed::pyUpdatePath(std::string path)
{
    // the python module search path compute for system enviroment
    wchar_t *curr_path = Py_GetPath();

    PyObject *py_curr_path = PyUnicode_FromWideChar(curr_path, wcslen(curr_path));
    LogProducer::info("interpembed", "path before:%s", PyUnicode_AsUTF8(py_curr_path));

    // get specified module path from config file
    PyObject *py_mod_path = PyUnicode_FromString(path.c_str());
    wchar_t *mod_path = PyUnicode_AsWideCharString(py_mod_path, NULL);

    // wide char string object from curr_path
    std::wstring ws_curr_path(curr_path);
    // wide char string object from mod_path
    std::wstring ws_mod_path(mod_path);
    
    // if the specified path not in system then update
    if(ws_curr_path.find(ws_mod_path) == ws_curr_path.npos)
    {
        ws_curr_path += ws_mod_path;
        Py_SetPath(ws_curr_path.c_str());
    }

    // free the memory allocate by py-interface
    PyMem_Free(mod_path);
    Py_DECREF(py_mod_path);
    Py_DECREF(py_curr_path);

    curr_path = Py_GetPath();
    py_curr_path = PyUnicode_FromWideChar(curr_path, wcslen(curr_path));
    LogProducer::info("interpembed", "path after:%s", PyUnicode_AsUTF8(py_curr_path));
    Py_DECREF(py_curr_path);

    return true;
}

void InterpEmbed::pyRunFile(const std::string& file)
{
    LogProducer::info("interpembed", "program %s will open", file.c_str());
    // this is the interpreter control thread
    FILE *fp = fopen(file.c_str(), "r");
    if(NULL == fp)
    {
        LogProducer::error("interpembed", "program %s open failed", file.c_str());
        base_space::ErrorQueue::instance().push(INTERPRETER_ERROR_PROG_NOT_EXIST);
        return;
    }
    LogProducer::info("interpembed", "start run program %s", file.c_str());
    PyRun_SimpleFileEx(fp, file.c_str(), 1); // 1:return with close file, 0:return but not close file
}

bool InterpEmbed::pySetTrace(interpid_t idx, InterpEmbed *selfobj)
{
    Py_buffer buf;
    /* NOTE: the buffer needn't be released as its object is NULL. */
    if (PyBuffer_FillInfo(&buf, NULL, (char *)selfobj, sizeof(*selfobj), 0, PyBUF_CONTIG) == -1)
    {
        return false;
        LogProducer::error("interpembed", "fill object to buffer failed");
    }
    /* build InterpEmbed object to PyObject and pass to tracer */
    trace_obj_ = PyMemoryView_FromBuffer(&buf);
    if(trace_obj_ == NULL)
    {
        LogProducer::error("interpembed", "set program[%ld] tracer failed", idx);
        return false;
    }
    id_ = idx;
    /* set the trace function.this function will call while ervery line execute */
    // PyAPI_FUNC(int) _PyEval_SetTrace(PyThreadState *tstate, Py_tracefunc func, PyObject *arg);
    PyEval_SetTrace(tracer, trace_obj_);
    return true;
}

int InterpEmbed::hold(void)
{
    /* semphore take is has ben hold by another this will block */
    return exec_sem_ptr_->take();
}

int InterpEmbed::release(void)
{
    if(exec_sem_ptr_->isTaken())
        return exec_sem_ptr_->give();

    return 0;
}

void InterpEmbed::progThreadFunc(void)
{
    while(!is_exit_)
    {
        LogProducer::info("interpembed", "start program %s", main_prog_.c_str());
        pyRunFile(main_path_name_);
        LogProducer::info("interpembed", "program %s exit", main_prog_.c_str());
        is_exit_ = true;
        curr_state_ = INTERP_IDLE;
    }
}

static void* interp_prog_thread_func(void* arg)
{
    char module_name[32];
    uint32_t isr_ptr = 0;
    log_space::LogProducer log_manager;
    InterpEmbed* interp_embed = static_cast<InterpEmbed*>(arg);
    sprintf(module_name, "interpembed_%ld", interp_embed->getId());
    log_manager.init(module_name, &isr_ptr);

    LogProducer::warn(module_name, "interpreter program thread TID is %ld", syscall(SYS_gettid));
    interp_embed->progThreadFunc();
    std::cout<<"interp_prog_thread exit"<<std::endl;

	return NULL;
}

ErrorCode InterpEmbed::start(std::string prog)
{
    // only in idle state
    if(curr_state_ != INTERP_IDLE)
    {
        LogProducer::error("interpembed", "start program %s failed not in IDLE state", prog);
        return INTERPRETER_ERROR_INVALID_STATE;
    }
    if(!InterpEmbed::config_.loadConfig())
    {
        LogProducer::error("interpembed", "configuration file load failed");
        return INTERPRETER_ERROR_CONFIG_LOAD_FAILED;
    }

    main_prog_ = prog;
    main_path_name_ = config_.getProgPath() + prog;
    is_exit_ = is_abort_ = is_paused_ = false;
    pyResetInterp();
    pySetTrace(0, this);/* for reload program file */

    if(!prog_thread_.run(interp_prog_thread_func, this, config_.getThreadPriority()))
    {
        return INTERPRETER_ERROR_START_THREAD_FAILED;
    }
    curr_state_ = INTERP_RUNNING;
    return 0;
}

ErrorCode InterpEmbed::pause(void)
{
    if(curr_state_ != INTERP_RUNNING)
    {
        LogProducer::warn("interpembed", "pause program %d failed not in RUNNING state", id_);
        return INTERPRETER_ERROR_INVALID_STATE;
    }

    LogProducer::info("interpembed", "try pause program %d", id_);

    if(hold() != 0)
    {
        LogProducer::error("interpembed", "program %d paused failed while holding", id_);
        return INTERPRETER_ERROR_HOLD_FAILED;
    }

    is_paused_ = true;
    curr_state_ = INTERP_PAUSE;

    LogProducer::info("interpembed", "program %d paused", id_);

    return 0;
}

ErrorCode InterpEmbed::resume(void)
{
    if(curr_state_ != INTERP_PAUSE)
    {
        LogProducer::error("interpembed", "resume program %d failed not in PAUSE state", id_);
        return INTERPRETER_ERROR_INVALID_STATE;
    }

    LogProducer::info("interpembed", "try resume program %d", id_);

    if(release() != 0)
    {
        LogProducer::error("interpembed", "program %d resume failed while releasing", id_);
        return INTERPRETER_ERROR_RELEASE_FAILED;
    }
    is_paused_ = false;
    curr_state_ = INTERP_RUNNING;

    LogProducer::info("interpembed", "program %d resumed", id_);

    return 0;
}

ErrorCode InterpEmbed::abort(void)
{
    if(curr_state_ == INTERP_IDLE)
    {
        LogProducer::error("interpembed", "abort program %d failed in IDLE state", id_);
        return INTERPRETER_ERROR_INVALID_STATE;
    }

    LogProducer::info("interpembed", "try abort program %d", id_);

    is_abort_ = true;
    prog_thread_.join();
    curr_state_ = INTERP_IDLE;

    LogProducer::info("interpembed", "program %d aborted", id_);

    return 0;
}

ErrorCode InterpEmbed::forward(void)
{
    if(curr_state_ == INTERP_RUNNING)
    {
        LogProducer::error("interpembed", "forward program %d failed in RUNNING state", id_);
        return INTERPRETER_ERROR_INVALID_STATE;
    }

    LogProducer::info("interpembed", "try forward program %d", id_);

    curr_mode_ = INTERP_STEP;

    LogProducer::info("interpembed", "program %d mode change to STEP", id_);

    return 0;
}

ErrorCode InterpEmbed::backward(void)
{
    if(curr_state_ == INTERP_RUNNING)
    {
        LogProducer::error("interpembed", "backward program %d failed in RUNNING state", id_);
        return INTERPRETER_ERROR_INVALID_STATE;
    }

    LogProducer::info("interpembed", "try forward program %d", id_);

    curr_mode_ = INTERP_JUMP;

    LogProducer::info("interpembed", "program %d mode change to JUMP", id_);

    return 0;
}

ErrorCode InterpEmbed::jump(int line)
{
    if(curr_state_ == INTERP_RUNNING)
    {
        LogProducer::error("interpembed", "forward program %d failed in RUNNING state", id_);
        return INTERPRETER_ERROR_INVALID_STATE;
    }

    LogProducer::info("interpembed", "try jump program %d", id_);

    curr_mode_ = INTERP_JUMP;

    LogProducer::info("interpembed", "program %d mode change to JUMP", id_);

    return 0;
}
