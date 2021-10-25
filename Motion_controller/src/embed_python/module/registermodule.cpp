#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "interpreter_register.h"


static PyObject *register_GetRR(PyObject *self, PyObject *args)
{
    int id = 0;
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);
    RegValue value;
    InterpReg_GetRR(id, &value);
    return Py_BuildValue("d", value.rr);
    //return PyFloat_FromDouble(value.rr);
}

static PyObject *register_SetRR(PyObject *self, PyObject *args)
{
    double rrvalue = 0;
    int id = 0;   
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "id", &id, &rrvalue))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);
    RegValue value;
    value.rr = rrvalue;
    ErrorCode ret = InterpReg_SetRR(id, &value);
    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *register_GetMR(PyObject *self, PyObject *args)
{
    int id = 0;
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);
    RegValue value;
    InterpReg_GetMR(id, &value);
    return Py_BuildValue("i", value.mr);
    //return PyLong_AsLong(value->rr);
}

static PyObject *register_SetMR(PyObject *self, PyObject *args)
{
    int mrvalue = 0;
    int id = 0;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "ii", &id, &mrvalue))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);
    RegValue value;
    value.mr = mrvalue;
    ErrorCode ret = InterpReg_SetMR(id, &value);
    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *register_GetSR(PyObject *self, PyObject *args)
{
    int id = 0;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    //printf("---------------|register_GetSR|-----------\n");
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);
    RegValue value;
    InterpReg_GetSR(id, &value);
    //printf("===============|register_SetSR result: SR[%d]=%s|===============\n",id,value.sr);
    return Py_BuildValue("s", value.sr);
    //return PyString_AsString(value.sr);
}

static PyObject *register_SetSR(PyObject *self, PyObject *args)
{
    char *srvalue;
    int id = 0;
    //printf("---------------|registe_SetSR|-----------\n");
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "is", &id, &srvalue))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);
    RegValue value;
    memcpy(value.sr, srvalue, sizeof(value.sr));
    ErrorCode ret = InterpReg_SetSR(id, &value);
    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *register_GetPR(PyObject *self, PyObject *args)
{
    int id = 0;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "i", &id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);
    RegValue value;
    InterpReg_GetPR(id, &value);
    return Py_BuildValue("iiiiiiiiiiiiiiddddddddd",value.pr.coord, value.pr.posture[0], value.pr.posture[1], value.pr.posture[2], value.pr.posture[3], 
    value.pr.turn[0], value.pr.turn[1], value.pr.turn[2], value.pr.turn[3], value.pr.turn[4], value.pr.turn[5], value.pr.turn[6], value.pr.turn[7], value.pr.turn[8],
    value.pr.pos[0], value.pr.pos[1], value.pr.pos[2], value.pr.pos[3], value.pr.pos[4], value.pr.pos[5], value.pr.pos[6], value.pr.pos[7], value.pr.pos[8]);    
}

static PyObject *register_SetPR(PyObject *self, PyObject *args)
{
    int id = 0;
    RegValue value;
    Py_buffer buffer;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "iw*", &id, &buffer))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);
    memcpy(&value.pr, buffer.buf, sizeof(PrInfo));   
    ErrorCode ret = InterpReg_SetPR(id, &value);
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
    if(!InterpReg_Init())
    {
        //printf("interpreg.\n");
        return NULL;
    }

    m = PyModule_Create(&registermodule);
    if (m == NULL)
        return NULL;

    return m;
}


