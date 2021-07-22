#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "interpreter_register.h"

static PyObject *register_GetRR(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int id = 0;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);


    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *register_SetRR(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int id = 0;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);


    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *register_GetRR(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int id = 0;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);


    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *register_SetRR(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int id = 0;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);
    

    return PyLong_FromUnsignedLongLong(ret);
}
static PyObject *register_GetMR(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int id = 0;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);


    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *register_SetMR(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int id = 0;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);
    

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *register_GetSR(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int id = 0;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);


    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *register_SetSR(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int id = 0;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);
    

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *register_GetPR(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int id = 0;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);


    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *register_SetPR(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int id = 0;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);
    

    return PyLong_FromUnsignedLongLong(ret);
}

static PyMethodDef registerMethods[] = {
    {"SetRR",      register_SetRR,    METH_VARARGS, "set real reg value."},
    {"GetRR",      register_GetRR,    METH_VARARGS, "get real reg value."},
    {"SetMR",      register_SetMR,    METH_VARARGS, "set motion reg value."},
    {"GetMR",      register_GetMR,    METH_VARARGS, "get motion reg value."},
    {"SetSR",      register_SetSR,    METH_VARARGS, "set string reg value."},
    {"GetSR",      register_GetSR,    METH_VARARGS, "get string reg value."},
    {"SetPR",      register_SetPR,    METH_VARARGS, "set pos reg value."},
    {"GetPR",      register_GetPR,    METH_VARARGS, "get pos reg value."},

    {NULL, NULL, 0, NULL}        /* Sentinel */
};

static struct PyModuleDef registermodule = {
    PyModuleDef_HEAD_INIT,
    "register",   /* name of module */
    NULL, /* module documentation, may be NULL */
    -1,       /* size of per-interpreter state of the module,
                 or -1 if the module keeps state in global variables. */
    registerMethods
};

PyMODINIT_FUNC PyInit_register(void)
{
    PyObject *m;

    m = PyModule_Create(&registermodule);
    if (m == NULL)
        return NULL;

    return m;
}


