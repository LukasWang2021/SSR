#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "interpreter_device.h"

/* Virtual device module.
 * Like DI,DO,RI,RO,etc.
 * reference:https://docs.python.org/3/c-api/typeobj.html
 */

static PyObject *DeviceModuleError;

static PyObject *device_SetDOBit(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int id = 0, val = 0;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "ii", &id, &val))
        return PyLong_FromUnsignedLongLong(0);
    
    ret = InterpDevice_SetDOBit(id, val);
    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *device_GetDOBit(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int id = 0;
    uint8_t return_val = 0;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLong(0);
    ret = InterpDevice_GetDOBit(id, return_val);
    if(ret!=0)
        return PyLong_FromUnsignedLongLong(0);
    return PyLong_FromUnsignedLong(return_val);//DO 引脚状态(通常为0或1)
}

static PyObject *device_GetDIBit(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int id = 0;
    uint8_t return_val = 0;
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLong(0);
    ret = InterpDevice_GetDIBit(id, return_val);
    if(ret!=0)
        return PyLong_FromUnsignedLongLong(0);
    return PyLong_FromUnsignedLong(return_val);//DI pin status
}

static PyObject *device_ForceValue(PyObject *self, PyObject *args)
{
    int id = 0;
    double ft[6] = {0};

    if (!PyArg_ParseTuple(args, "i", &id))
        return NULL;

    if(InterpDevice_GetForceValue(id, ft) != 0) 
        return NULL;

    return NULL;
}

static PyMethodDef deviceMethods[] = {
    {"GetDO",      device_GetDOBit,    METH_VARARGS, "read DO value of the index."},
    {"SetDO",      device_SetDOBit,    METH_VARARGS, "write DO value of the index."},
    {"GetDI",      device_GetDIBit,    METH_VARARGS, "read DI value of the index."},
    {"ForceValue", device_ForceValue,  METH_VARARGS, "read force sensor value with the index."},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

static struct PyModuleDef devicemodule = {
    PyModuleDef_HEAD_INIT,
    "device",   /* name of module */
    NULL, /* module documentation, may be NULL */
    -1,       /* size of per-interpreter state of the module,
                 or -1 if the module keeps state in global variables. */
    deviceMethods
};

PyMODINIT_FUNC PyInit_device(void)
{
    PyObject *m;

    m = PyModule_Create(&devicemodule);
    if (m == NULL)
        return NULL;

    DeviceModuleError = PyErr_NewException("DeviceModule.error", NULL, NULL);
    Py_XINCREF(DeviceModuleError);
    if (PyModule_AddObject(m, "error", DeviceModuleError) < 0)
    {
        Py_XDECREF(DeviceModuleError);
        Py_CLEAR(DeviceModuleError);
        Py_DECREF(m);
        return NULL;
    }

    return m;
}


