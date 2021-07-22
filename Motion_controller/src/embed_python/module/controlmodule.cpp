#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "interpreter_control.h"

/* Do not use this module in user program
 * This module can only use in another python process
 */

static PyObject *ControlModuleError;

static PyObject *control_Pause(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int id = 0;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);
    
    ret = InterpCtrl::instance().pause();

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *control_Abort(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int id = 0;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);
    
    ret = InterpCtrl::instance().abort();

    return PyLong_FromUnsignedLongLong(ret);
}

static PyMethodDef controlMethods[] = {
    {"Pause",      control_Pause,    METH_VARARGS, "pause the running user program."},
    {"Abort",      control_Abort,    METH_VARARGS, "abort the running user program."},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

static struct PyModuleDef controlmodule = {
    PyModuleDef_HEAD_INIT,
    "control",   /* name of module */
    NULL, /* module documentation, may be NULL */
    -1,       /* size of per-interpreter state of the module,
                 or -1 if the module keeps state in global variables. */
    controlMethods
};

PyMODINIT_FUNC PyInit_control(void)
{
    PyObject *m;

    m = PyModule_Create(&controlmodule);
    if (m == NULL)
        return NULL;

    ControlModuleError = PyErr_NewException("ControlModule.error", NULL, NULL);
    Py_XINCREF(ControlModuleError);
    if (PyModule_AddObject(m, "error", ControlModuleError) < 0)
    {
        Py_XDECREF(ControlModuleError);
        Py_CLEAR(ControlModuleError);
        Py_DECREF(m);
        return NULL;
    }

    return m;
}


