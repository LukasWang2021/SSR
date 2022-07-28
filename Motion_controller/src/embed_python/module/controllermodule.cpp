#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "interpreter_control.h"

/* Do not use this module in user program
 * This module can only use in another python process
 */

static PyObject *controller_Pause(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int id = 0;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);
    
    ret = InterpCtrl::instance().pause();

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *controller_Abort(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int id = 0;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);
    
    ret = InterpCtrl::instance().abort();

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject* controller_ThreadRun(PyObject *self, PyObject *args)
{
    char *file;
    int in_real = 0;
    ErrorCode ret = 0;
    if (!PyArg_ParseTuple(args, "si:thread run file", &file, &in_real))
        return NULL;

    ret = InterpCtrl::instance().startNewFile(file, in_real);

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject* controller_Delay(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    double delay_time = 0;
    if (!PyArg_ParseTuple(args, "d", &delay_time))
        return NULL;

    ret = InterpCtrl::instance().delay(delay_time);
    
    return PyLong_FromUnsignedLongLong(ret);
} 

static PyMethodDef controllerMethods[] = {
    {"Pause",      controller_Pause,     METH_VARARGS, "pause the running user program."},
    {"Abort",      controller_Abort,     METH_VARARGS, "abort the running user program."},
    {"ThreadRun",  controller_ThreadRun, METH_VARARGS, "run in another thread."},
    {"Delay",      controller_Delay,     METH_VARARGS, "delay for seconds."},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

static struct PyModuleDef controllermodule = {
    PyModuleDef_HEAD_INIT,
    "controller",   /* name of module */
    NULL, /* module documentation, may be NULL */
    -1,       /* size of per-interpreter state of the module,
                 or -1 if the module keeps state in global variables. */
    controllerMethods
};

PyMODINIT_FUNC PyInit_controller(void)
{
    PyObject *m;

    m = PyModule_Create(&controllermodule);
    if (m == NULL)
        return NULL;

    return m;
}


