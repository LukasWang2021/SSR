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

InterpEmbed::InterpEmbed(interpid_t id, std::string prog)
{
    curr_mode_ = INTERP_MODE_AUTO;
    curr_state_ = INTERP_STATE_IDLE;
    id_ = id;
    main_prog_ = prog;
}

InterpEmbed::~InterpEmbed()
{
    is_abort_ = true;
    is_exit_ = true;
    prog_thread_.join();
    if(exec_sem_ptr_ != NULL)
        delete exec_sem_ptr_;

    if(exit_sem_ptr_ != NULL)
        delete exit_sem_ptr_;

    if(trace_obj_ != NULL)
        Py_DECREF(trace_obj_);
}

// the parameter 'obj' is passed by PyEval_SetTrace
// if return not zero python will stop
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
        PyErr_SetString(PyExc_EOFError, "user call abort to exit"); // exit python
        LogProducer::warn("interpembed","interpreter stop signal recieved");
        return -1;
    }

    // update execution info, python system lib will be ignored
    PyCodeObject *code = PyFrame_GetCode(frame);
    const char *filename = PyUnicode_AsUTF8(code->co_filename);
    const char *funcname = PyUnicode_AsUTF8(code->co_name);
    int lineno = PyFrame_GetLineNumber(frame);

    // filter python system lib: while the filename string do not find module-path and prog-path
    if(std::string(filename).find(InterpEmbed::config_.getProgPath()) == std::string::npos && 
       std::string(filename).find(InterpEmbed::config_.getModulePath()) == std::string::npos )
    {
        return 0;
    }

    embed_ptr->setCurrLine(lineno);
    embed_ptr->setCurrProg(filename);
    embed_ptr->setCurrFunc(funcname);
    LogProducer::info("interpembed", "program(%s) line[%d]", filename, lineno);
    // exec sem take
    Py_BEGIN_ALLOW_THREADS
    embed_ptr->hold();
    Py_END_ALLOW_THREADS

    embed_ptr->release();

    // if step mode goto pause
    if(embed_ptr->getMode() != INTERP_MODE_AUTO)
        embed_ptr->pause();

    return 0;
}

bool InterpEmbed::pyResetInterp(void)
{
    /* uninitialize the python interpreter, this will destroy all interpreters*/
    Py_Finalize();
    /* prameter 0 means initialize the python interpreter and skip sighandler
       if it's 1 some error will happen when ctrl-c the controller process */
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

void InterpEmbed::pyRunFile(void)
{
    LogProducer::info("interpembed", "program %s will open", main_path_name_.c_str());
    // this is the interpreter control thread
    FILE *fp = fopen(main_path_name_.c_str(), "r");
    if(NULL == fp)
    {
        LogProducer::error("interpembed", "program %s open failed", main_path_name_.c_str());
        base_space::ErrorQueue::instance().push(INTERPRETER_ERROR_PROG_NOT_EXIST);
        return;
    }
    LogProducer::info("interpembed", "start run program %s", main_path_name_.c_str());
    PyRun_SimpleFileEx(fp, main_path_name_.c_str(), 1); // 1:return with close file, 0:return but not close file
}

bool InterpEmbed::pySetTrace(void)
{
    Py_buffer buf;

    /* NOTE: the buffer needn't be released as its object is NULL. */
    if (PyBuffer_FillInfo(&buf, NULL, (char *)this, sizeof(*this), 0, PyBUF_CONTIG) == -1)
    {
        return false;
        LogProducer::error("interpembed", "fill object to buffer failed");
    }

    /* build InterpEmbed object to PyObject and pass to tracer */
    trace_obj_ = PyMemoryView_FromBuffer(&buf);
    if(trace_obj_ == NULL)
    {
        LogProducer::error("interpembed", "set program[%ld] tracer failed", id_);
        return false;
    }

    /* set the trace function.this function will call while ervery line execute */
    PyEval_SetTrace(tracer, trace_obj_);

    return true;
}

bool InterpEmbed::inThreadInit(void)
{
    exec_sem_ptr_ = new SemHelp(0, 1); // init value 1
    exit_sem_ptr_ = new SemHelp(0, 1); // init value 1
    if(exec_sem_ptr_ == NULL || exit_sem_ptr_ == NULL)
    {
        LogProducer::error("interpembed","semphore alocate failed");
        return false;
    }
    self_ts_ = PyThreadState_Get();  // python interpreter
    pyid_    = PyThreadState_GetID(self_ts_);// python interpreter id
    thread_id_ = pthread_self();
    return true;
}

int InterpEmbed::hold(void)
{
    /* semphore has been hold by another this will block */
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
    if(!inThreadInit())
    {
        LogProducer::info("interpembed", "sub-interpreter thread init failed");
        return;
    }

    if(!pySetTrace())
    {
        LogProducer::info("interpembed", "tracer set failed");
        return;
    }
    exit_sem_ptr_->take();

    LogProducer::info("interpembed", "start program %s", main_prog_.c_str());

    InterpEmbed::pyRunFile();
    
    exit_sem_ptr_->give();

    LogProducer::info("interpembed", "program %s exit", main_prog_.c_str());


    curr_state_ = INTERP_STATE_IDLE;
}

void InterpEmbed::pyThreadFunc(void)
{
    PyGILState_STATE gstate;
    PyThreadState *substate, *mainstate;

    gstate = PyGILState_Ensure();

    mainstate = PyThreadState_Get();

    PyThreadState_Swap(NULL); // now thread state is null

    substate = Py_NewInterpreter(); // now thread state is 'substate'
    if (substate == NULL)
    {
        PyThreadState_Swap(mainstate);
        LogProducer::info("interpembed", "sub-interpreter creation failed");
        return;
    }

    if(!inThreadInit())
    {
        LogProducer::info("interpembed", "sub-interpreter thread init failed");
        return;
    }

    if(!pySetTrace())
    {
        LogProducer::info("interpembed", "tracer set failed");
        return;
    }

    exit_sem_ptr_->take();

    LogProducer::info("interpembed", "start program %s", main_prog_.c_str());

    InterpEmbed::pyRunFile();

    Py_EndInterpreter(substate);
    PyThreadState_Swap(mainstate);
    PyGILState_Release(gstate);

    exit_sem_ptr_->give();

    LogProducer::info("interpembed", "program %s exit", main_prog_.c_str());

    curr_state_ = INTERP_STATE_IDLE;
}

static void* interp_prog_thread_func(void* arg)
{
    char module_name[32];
    uint32_t isr_ptr = 0;
    log_space::LogProducer log_manager;
    InterpEmbed* interp_embed = static_cast<InterpEmbed*>(arg);
    // log initialize
    sprintf(module_name, "interpembed_%ld", interp_embed->getId());
    log_manager.init(module_name, &isr_ptr);
    LogProducer::warn(module_name, "interpreter program thread TID is %ld", syscall(SYS_gettid));
    interp_embed->progThreadFunc();
    std::cout<<"interp_prog_thread_func " << interp_embed->getId() << " exit"<<std::endl;

	return NULL;
}

static void py_thread_func(void* arg)
{
    char module_name[32];
    uint32_t isr_ptr = 0;
    log_space::LogProducer log_manager;
    InterpEmbed* interp_embed = static_cast<InterpEmbed*>(arg);
    // log initialize
    sprintf(module_name, "interpembed_%ld", interp_embed->getId());
    log_manager.init(module_name, &isr_ptr);
    LogProducer::warn(module_name, "interpreter program thread TID is %ld", syscall(SYS_gettid));
    interp_embed->pyThreadFunc();
    std::cout<<"py_thread_func exit"<<std::endl;

	return NULL;
}

bool InterpEmbed::pyStartThread(void)
{
    // only in idle state
    if(curr_state_ != INTERP_STATE_IDLE)
    {
        LogProducer::error("interpembed", "start program %s failed not in IDLE state", main_prog_);
        return false;
    }

    is_exit_ = is_abort_ = is_paused_ = false;
    unsigned long ident;
    ident = PyThread_start_new_thread(py_thread_func, (void*)this);
    if (ident == PYTHREAD_INVALID_THREAD_ID)
    {
        return false;
    }

    curr_state_ = INTERP_STATE_RUNNING;
    return true;
}

ErrorCode InterpEmbed::start(void)
{
    // only in idle state
    if(curr_state_ != INTERP_STATE_IDLE)
    {
        LogProducer::error("interpembed", "start program %s failed not in IDLE state", main_prog_);
        return INTERPRETER_ERROR_INVALID_STATE;
    }

    is_exit_ = is_abort_ = is_paused_ = false;

    if(!prog_thread_.run(interp_prog_thread_func, this, config_.progThreadPriority()))
    {
        return INTERPRETER_ERROR_START_THREAD_FAILED;
    }

    curr_state_ = INTERP_STATE_RUNNING;
    return 0;
}

ErrorCode InterpEmbed::pause(void)
{
    if(curr_state_ != INTERP_STATE_RUNNING)
    {
        LogProducer::warn("interpembed", "pause program %d not in RUNNING state, ignored", id_);
        return 0;
    }

    LogProducer::info("interpembed", "try pause program %d", id_);

    if(hold() != 0) // block in here
    {
        LogProducer::error("interpembed", "program %d paused failed while holding", id_);
        return INTERPRETER_ERROR_HOLD_FAILED;
    }

    is_paused_ = true;
    curr_state_ = INTERP_STATE_PAUSE;

    LogProducer::info("interpembed", "program %d paused", id_);

    return 0;
}

ErrorCode InterpEmbed::resume(void)
{
    if(curr_state_ != INTERP_STATE_PAUSE)
    {
        LogProducer::warn("interpembed", "resume program %d not in PAUSE state, ignored", id_);
        return 0;
    }

    LogProducer::info("interpembed", "try resume program %d", id_);

    if(release() != 0)
    {
        LogProducer::error("interpembed", "program %d resume failed while releasing", id_);
        return INTERPRETER_ERROR_RELEASE_FAILED;
    }
    is_paused_ = false;
    curr_state_ = INTERP_STATE_RUNNING;

    LogProducer::info("interpembed", "program %d resumed", id_);

    return 0;
}

ErrorCode InterpEmbed::abort(void)
{
    if(curr_state_ == INTERP_STATE_IDLE)
    {
        LogProducer::warn("interpembed", "abort program %d in IDLE state, ignored", id_);
        return 0;
    }

    LogProducer::info("interpembed", "try abort program %d", id_);

    is_abort_ = true;
    if(curr_state_ == INTERP_STATE_PAUSE)
        release();
    
    waitAborted();
    curr_state_ = INTERP_STATE_IDLE;

    LogProducer::info("interpembed", "program %d aborted", id_);

    return 0;
}

ErrorCode InterpEmbed::forward(void)
{
    if(curr_state_ == INTERP_STATE_RUNNING)
    {
        LogProducer::error("interpembed", "forward program %d failed in RUNNING state", id_);
        return INTERPRETER_ERROR_INVALID_STATE;
    }

    LogProducer::info("interpembed", "try forward program %d", id_);

    curr_mode_ = INTERP_MODE_STEP;

    LogProducer::info("interpembed", "program %d mode change to STEP", id_);

    return 0;
}

ErrorCode InterpEmbed::backward(void)
{
    if(curr_state_ == INTERP_STATE_RUNNING)
    {
        LogProducer::error("interpembed", "backward program %d failed in RUNNING state", id_);
        return INTERPRETER_ERROR_INVALID_STATE;
    }

    LogProducer::info("interpembed", "try forward program %d", id_);

    curr_mode_ = INTERP_MODE_JUMP;

    LogProducer::info("interpembed", "program %d mode change to JUMP", id_);

    return 0;
}

ErrorCode InterpEmbed::jump(int line)
{
    if(curr_state_ == INTERP_STATE_RUNNING)
    {
        LogProducer::error("interpembed", "forward program %d failed in RUNNING state", id_);
        return INTERPRETER_ERROR_INVALID_STATE;
    }

    LogProducer::info("interpembed", "try jump program %d", id_);

    curr_mode_ = INTERP_MODE_JUMP;

    LogProducer::info("interpembed", "program %d mode change to JUMP", id_);

    return 0;
}

void InterpEmbed::waitAborted(void)
{
    exit_sem_ptr_->take();
    exit_sem_ptr_->give();
}

