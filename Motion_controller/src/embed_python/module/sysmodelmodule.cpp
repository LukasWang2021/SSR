#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "interpreter_sysmodel.h"

static PyObject *SysmodelModuleError;

static PyObject *sysmodel_SetParam(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    uint32_t param_part = 0;
    uint32_t param_index = 0;
    int32_t param_value = 0;

    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "IIi", &param_part, &param_index, &param_value))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);

    ret = InterpSysModel_SetParam(param_part, param_index, param_value);

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *sysmodel_GetParam(PyObject *self, PyObject *args)
{
    uint32_t param_part = 0;
    uint32_t param_index = 0;
    int32_t param_value = 0;

    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "IIi", &param_part, &param_index))
        return PyLong_FromUnsignedLongLong(0);

    if(InterpSysModel_GetParam(param_part, param_index, &param_value))
        return PyLong_FromUnsignedLongLong(0);

    return PyLong_FromUnsignedLongLong(param_value);
}

static PyObject *sysmodel_SaveParam(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    uint32_t param_part = 0;

    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "I", &param_part))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);

    ret = InterpSysModel_SaveParam(param_part);

    return PyLong_FromUnsignedLongLong(ret);
}

static PyMethodDef sysmodelMethods[] = {
    {"SetParam",         sysmodel_SetParam,   METH_VARARGS, "set system model parameter."},
    {"GetParam",         sysmodel_GetParam,   METH_VARARGS, "get system model parameter."},
    {"SaveParam",        sysmodel_SaveParam,  METH_VARARGS, "save system model parameter."},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

static struct PyModuleDef sysmodelmodule = {
    PyModuleDef_HEAD_INIT,
    "sysmodel",   /* name of module */
    NULL, /* module documentation, may be NULL */
    -1,       /* size of per-interpreter state of the module,
                 or -1 if the module keeps state in global variables. */
    sysmodelMethods
};

PyMODINIT_FUNC PyInit_sysmodel(void)
{
    PyObject *m;

    m = PyModule_Create(&sysmodelmodule);
    if (m == NULL)
        return NULL;

    SysmodelModuleError = PyErr_NewException("SysmodelModule.error", NULL, NULL);
    Py_XINCREF(SysmodelModuleError);
    if (PyModule_AddObject(m, "error", SysmodelModuleError) < 0)
    {
        Py_XDECREF(SysmodelModuleError);
        Py_CLEAR(SysmodelModuleError);
        Py_DECREF(m);
        return NULL;
    }

    return m;
}


