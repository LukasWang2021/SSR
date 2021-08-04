#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "interpreter_group.h"
#include "interpreter_control.h"

// static PyObject *GroupModuleError;

static MoveTrajInfo g_traj;

static PyObject *group_MoveJ(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    Py_buffer buffer;

    // parameters (PostureInfo,double,int,double)(target,velocity,smooth type,smooth value)
    if (!PyArg_ParseTuple(args, "w*did", &buffer, &g_traj.vel, &g_traj.smooth_type, &g_traj.smooth_value))
            return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);

    memcpy(&g_traj.tgt, buffer.buf, sizeof(PostureInfo));
    // call robot system interface
    ret = InterpGroup_MoveJoint(0, &g_traj);
    // free Py_buffer
    PyBuffer_Release(&buffer);

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *group_MoveJwithAcc(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    Py_buffer buffer;

    // parameters (PostureInfo,double,int,double,double)(target,velocity,smooth type,smooth value,acc)
    if (!PyArg_ParseTuple(args, "w*didd", &buffer, &g_traj.vel, &g_traj.smooth_type, &g_traj.smooth_value, &g_traj.acc))
            return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);

    memcpy(&g_traj.tgt, buffer.buf, sizeof(PostureInfo));
    // call robot system interface
    ret = InterpGroup_MoveJoint(0, &g_traj);
    // free Py_buffer
    PyBuffer_Release(&buffer);

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *group_MoveJwithOffset(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int offset_type = -1, offset_id = -1;
    Py_buffer buffer;

    // parameters (PostureInfo,double,int,double,int)(target,velocity,smooth type,smooth value,acc)
    if (!PyArg_ParseTuple(args, "w*didii", &buffer, &g_traj.vel, &g_traj.smooth_type, &g_traj.smooth_value, &offset_type, &offset_id))
            return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);

    if(offset_type == 0) g_traj.tf_id = offset_id;
    if(offset_type == 1) g_traj.uf_id = offset_id;
    memcpy(&g_traj.tgt, buffer.buf, sizeof(PostureInfo));
    // call robot system interface
    ret = InterpGroup_MoveJoint(0, &g_traj);
    // free Py_buffer
    PyBuffer_Release(&buffer);

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *group_MoveJwithAccOffset(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int offset_type = -1, offset_id = -1;
    Py_buffer buffer;

    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "w*diddii", &buffer, &g_traj.vel, &g_traj.smooth_type, &g_traj.smooth_value, &g_traj.acc, &offset_type, &offset_id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);

    if(offset_type == 0) g_traj.tf_id = offset_id;
    if(offset_type == 1) g_traj.uf_id = offset_id;
    memcpy(&g_traj.tgt, buffer.buf, sizeof(PostureInfo));
    // call robot system interface
    ret = InterpGroup_MoveJoint(0, &g_traj);
    // free Py_buffer
    PyBuffer_Release(&buffer);

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *group_MoveL(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    Py_buffer buffer;

    // parameters (PostureInfo,double,int,double)(target,velocity,smooth type,smooth value)
    if (!PyArg_ParseTuple(args, "w*did", &buffer, &g_traj.vel, &g_traj.smooth_type, &g_traj.smooth_value))
            return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);

    memcpy(&g_traj.tgt, buffer.buf, sizeof(PostureInfo));
    // call robot system interface
    ret = InterpGroup_MoveLiner(0, &g_traj);
    // free Py_buffer
    PyBuffer_Release(&buffer);

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *group_MoveLwithAcc(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    Py_buffer buffer;
    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "w*didd", &buffer, &g_traj.vel, &g_traj.smooth_type, &g_traj.smooth_value, &g_traj.acc))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);

    memcpy(&g_traj.tgt, buffer.buf, sizeof(PostureInfo));
    // call robot system interface
    ret = InterpGroup_MoveLiner(0, &g_traj);
    // free Py_buffer
    PyBuffer_Release(&buffer);

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *group_MoveLwithOffset(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int offset_type = -1, offset_id = -1;
    Py_buffer buffer;

    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "w*didii", &buffer, &g_traj.vel, &g_traj.smooth_type, &g_traj.smooth_value, &offset_type, &offset_id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);

    if(offset_type == 0) g_traj.tf_id = offset_id;
    if(offset_type == 1) g_traj.uf_id = offset_id;
    memcpy(&g_traj.tgt, buffer.buf, sizeof(PostureInfo));
    // call robot system interface
    ret = InterpGroup_MoveLiner(0, &g_traj);
    // free Py_buffer
    PyBuffer_Release(&buffer);

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *group_MoveLwithAccOffset(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int offset_type = -1, offset_id = -1;
    Py_buffer buffer;

    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "w*diddii", &buffer, &g_traj.vel, &g_traj.smooth_type, &g_traj.smooth_value, &g_traj.acc, &offset_type, &offset_id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);

    if(offset_type == 0) g_traj.tf_id = offset_id;
    if(offset_type == 1) g_traj.uf_id = offset_id;
    memcpy(&g_traj.tgt, buffer.buf, sizeof(PostureInfo));
    // call robot system interface
    ret = InterpGroup_MoveLiner(0, &g_traj);
    // free Py_buffer
    PyBuffer_Release(&buffer);

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *group_MoveC(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    Py_buffer tgt_buffer;
    Py_buffer aux_buffer;

    // parameters (PostureInfo,double,int,double)(target,velocity,smooth type,smooth value)
    if (!PyArg_ParseTuple(args, "w*w*did", &aux_buffer, &tgt_buffer, &g_traj.vel, &g_traj.smooth_type, &g_traj.smooth_value))
            return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);

    memcpy(&g_traj.aux, aux_buffer.buf, sizeof(PostureInfo));
    memcpy(&g_traj.tgt, tgt_buffer.buf, sizeof(PostureInfo));
    // call robot system interface
    ret = InterpGroup_MoveCircl(0, &g_traj);
    // free Py_buffer
    PyBuffer_Release(&tgt_buffer);
    PyBuffer_Release(&aux_buffer);

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *group_MoveCwithAcc(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    Py_buffer tgt_buffer;
    Py_buffer aux_buffer;

    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "w*w*didd", &aux_buffer, &tgt_buffer, &g_traj.vel, &g_traj.smooth_type, &g_traj.smooth_value, &g_traj.acc))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);

    memcpy(&g_traj.aux, aux_buffer.buf, sizeof(PostureInfo));
    memcpy(&g_traj.tgt, tgt_buffer.buf, sizeof(PostureInfo));
    // call robot system interface
    ret = InterpGroup_MoveCircl(0, &g_traj);
    // free Py_buffer
    PyBuffer_Release(&tgt_buffer);
    PyBuffer_Release(&aux_buffer);

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *group_MoveCwithOffset(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int offset_type = -1, offset_id = -1;
    Py_buffer tgt_buffer;
    Py_buffer aux_buffer;

    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "w*w*didid", &aux_buffer, &tgt_buffer, &g_traj.vel, &g_traj.smooth_type, &g_traj.smooth_value, &offset_type, &offset_id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);

    if(offset_type == 0) g_traj.tf_id = offset_id;
    if(offset_type == 1) g_traj.uf_id = offset_id;
    memcpy(&g_traj.aux, aux_buffer.buf, sizeof(PostureInfo));
    memcpy(&g_traj.tgt, tgt_buffer.buf, sizeof(PostureInfo));
    // call robot system interface
    ret = InterpGroup_MoveCircl(0, &g_traj);
    // free Py_buffer
    PyBuffer_Release(&tgt_buffer);
    PyBuffer_Release(&aux_buffer);

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *group_MoveCwithAccOffset(PyObject *self, PyObject *args)
{
    ErrorCode ret = 0;
    int offset_type = -1, offset_id = -1;
    Py_buffer tgt_buffer;
    Py_buffer aux_buffer;

    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "w*w*diddid", &aux_buffer, &tgt_buffer, &g_traj.vel, &g_traj.smooth_type, &g_traj.smooth_value, &g_traj.acc, &offset_type, &offset_id))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);

    if(offset_type == 0) g_traj.tf_id = offset_id;
    if(offset_type == 1) g_traj.uf_id = offset_id;
    memcpy(&g_traj.aux, aux_buffer.buf, sizeof(PostureInfo));
    memcpy(&g_traj.tgt, tgt_buffer.buf, sizeof(PostureInfo));
    // call robot system interface
    ret = InterpGroup_MoveCircl(0, &g_traj);
    // free Py_buffer
    PyBuffer_Release(&tgt_buffer);
    PyBuffer_Release(&aux_buffer);

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *group_SetOAC(PyObject *self, PyObject *args)
{
    double val = 0;
    ErrorCode ret = 0;

    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "d", &val))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);
    
    ret = InterpGroup_SetOAC(0, val);

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *group_SetOVC(PyObject *self, PyObject *args)
{
    double val = 0;
    ErrorCode ret = 0;

    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "d", &val))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);
    
    ret = InterpGroup_SetOVC(0, val);

    return PyLong_FromUnsignedLongLong(ret);
}

static PyObject *group_SetPLD(PyObject *self, PyObject *args)
{
    int val = 0;
    ErrorCode ret = 0;

    /*see https://docs.python.org/3/c-api/arg.html#arg-parsing for PyArg_ParseTuple details*/
    if (!PyArg_ParseTuple(args, "i", &val))
        return PyLong_FromUnsignedLongLong(INTERPRETER_ERROR_MOD_INVALID_ARG);
    
    ret = InterpGroup_SetPLD(0, val);

    return PyLong_FromUnsignedLongLong(ret);
}

static PyMethodDef groupMethods[] = {
    {"MoveJ",                group_MoveJ,    METH_VARARGS, "move joint."},
    {"MoveJwithAcc",         group_MoveJwithAcc,    METH_VARARGS, "move joint."},
    {"MoveJwithOffset",      group_MoveJwithOffset,    METH_VARARGS, "move joint."},
    {"MoveJwithAccOffset",   group_MoveJwithAccOffset,    METH_VARARGS, "move joint."},
    {"MoveL",                group_MoveL,    METH_VARARGS, "move liner."},
    {"MoveLwithAcc",         group_MoveLwithAcc,    METH_VARARGS, "move liner."},
    {"MoveLwithOffset",      group_MoveLwithOffset,    METH_VARARGS, "move liner."},
    {"MoveLwithAccOffset",   group_MoveLwithAccOffset,    METH_VARARGS, "move liner."},
    {"MoveC",                group_MoveC,    METH_VARARGS, "move circular."},
    {"MoveCwithAcc",         group_MoveCwithAcc,    METH_VARARGS, "move circular."},
    {"MoveCwithOffset",      group_MoveCwithOffset,    METH_VARARGS, "move circular."},
    {"MoveCwithAccOffset",   group_MoveCwithAccOffset,    METH_VARARGS, "move circular."},
    {"SetOAC",               group_SetOAC,   METH_VARARGS, "set global acceleration."},
    {"SetOVC",               group_SetOVC,   METH_VARARGS, "set global velocity."},
    {"SetPLD",               group_SetPLD,   METH_VARARGS, "set global payload."},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

static struct PyModuleDef groupmodule = {
    PyModuleDef_HEAD_INIT,
    "group",   /* name of module */
    NULL, /* module documentation, may be NULL */
    -1,       /* size of per-interpreter state of the module,
                 or -1 if the module keeps state in global variables. */
    groupMethods
};

PyMODINIT_FUNC PyInit_group(void)
{
    PyObject *m;

    m = PyModule_Create(&groupmodule);
    if (m == NULL)
        return NULL;

    return m;
}


