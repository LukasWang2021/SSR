#define PY_SSIZE_T_CLEAN
#include "Python.h"

#include "common_error_code.h"
#include "interpreter_embed.h"
#include "log_manager_producer.h"
#include "interpreter_control.h"
#include "error_queue.h"

using namespace log_space;

InterpEmbed::InterpEmbed(/* args */)
{
}

InterpEmbed::~InterpEmbed()
{
}

static int tracer(PyObject *obj, PyFrameObject *frame, int what, PyObject *arg)
{
    // state sem take
    InterpCtrl::instance().hold();

    // // run registered trace callback(s)
    if(!InterpCtrl::instance().runSyncCallback())
    {
        LogProducer::error("interpembed","sync-call failed going to pause");
        InterpCtrl::instance().pause();
    }

    // PyCodeObject *code = PyFrame_GetCode(frame);
    // int line = PyFrame_GetLineNumber(frame);
    // const char *file = PyUnicode_AsUTF8(code->co_filename);
    // if(strstr(file, "<frozen") == NULL)
    //     LogProducer::debug("interpembed", "executing program(%s) at line[%d]", file, line);
    // printf("##########executing program(%s) at line[%d]\n", file, line);

    // if step state sem not give
    if(InterpCtrl::instance().getMode() != INTERP_AUTO)
        return 0;

    InterpCtrl::instance().release();
    
    return 0;
}

bool InterpEmbed::pyResetInterp(void)
{
    // uninitialize the python interpreter
    Py_Finalize();
    // initialize the python interpreter and skip sighandler
    Py_InitializeEx(0);
    // set the trace function.this function will call while ervery line execute
    PyEval_SetTrace(tracer, &trace_obj_);

    return true;
}

bool InterpEmbed::pyUpdatePath(void)
{
    // the python module search path compute for system enviroment
    wchar_t *curr_path = Py_GetPath();

    PyObject *py_curr_path = PyUnicode_FromWideChar(curr_path, wcslen(curr_path));
    LogProducer::info("interpembed", "path before:%s", PyUnicode_AsUTF8(py_curr_path));

    // read configuration file
    if(!config_.loadConfig())
    {
        LogProducer::error("interpembed", "load config file failed");
        return false;
    }

    // get specified module path from config file
    PyObject *py_mod_path = PyUnicode_FromString(config_.getModulePath().c_str());
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
    PyRun_SimpleFileEx(fp, file.c_str(), 1);
}
