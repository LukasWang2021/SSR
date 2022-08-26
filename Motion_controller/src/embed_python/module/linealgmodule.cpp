#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include "interpreter_alg.h"

/* linear alg module.
 * mathmatics of linear algebra.
 * reference:https://docs.python.org/3/c-api/typeobj.html
 */

static PyObject *LinealgModuleError;


static PyObject *linealg_Inverse(PyObject *self, PyObject *args)
{
    PyObject *bufobj = NULL; // recieve an array object
    Py_buffer view;

    if (!PyArg_ParseTuple(args, "O", bufobj))
            return NULL;

    /* Attempt to extract buffer information from it */
    if (PyObject_GetBuffer(bufobj, &view, PyBUF_ANY_CONTIGUOUS | PyBUF_FORMAT) == -1)
    {
        return NULL;
    }
    if (view.ndim <= 1 || *(view.shape) != *(view.strides))
    {
        char err_str[128];
        sprintf(err_str, "Invalid array shape[%d,%d]", (int)view.shape[0], (int)view.strides[0]);
        PyErr_SetString(PyExc_TypeError, err_str);
        PyBuffer_Release(&view);
        return NULL;
    }
    /* Check the type of items in the array */
    if (strcmp(view.format, "d") != 0)
    {
        PyErr_SetString(PyExc_TypeError, "Expected an array of doubles");
        PyBuffer_Release(&view);
        return NULL;
    }

    double *p_inv = (double *)malloc(sizeof(double) * view.ndim * view.ndim);
    if(p_inv == NULL) return NULL;

    if(InterpLinealg_Inverse((double *)(view.buf), view.ndim, p_inv) != 0)
        return NULL;

    PyObject *ret_list = PyList_New(0);
    PyObject *list_item = PyList_New(view.ndim);
    if(ret_list == NULL || list_item ==NULL)
    {
        PyErr_SetString(PyExc_OSError, "Memory alocate failed");
        return NULL;
    }

    for(int i = 0; i < view.ndim; ++i)
    {
        for(int j = 0; j < view.ndim; ++j)
        {
            PyList_SET_ITEM(list_item, j, PyFloat_FromDouble(p_inv[i * view.ndim + j]));
        }
        if(PyList_Append(ret_list, list_item) < 0)
        {
            return NULL;
        }
    }
    Py_DECREF(list_item);
    free(p_inv);
    return ret_list;
}

static PyObject *linealg_Eigens(PyObject *self, PyObject *args)
{
    PyObject *bufobj = NULL; // recieve an array object
    Py_buffer view;

    if (!PyArg_ParseTuple(args, "O", bufobj))
            return NULL;

    /* Attempt to extract buffer information from it */
    if (PyObject_GetBuffer(bufobj, &view, PyBUF_ANY_CONTIGUOUS | PyBUF_FORMAT) == -1)
    {
        return NULL;
    }
    if (view.ndim != 1 || *(view.shape) != *(view.strides))
    {
        char err_str[128];
        sprintf(err_str, "Invalid array shape[%d,%d]", (int)view.shape[0], (int)view.strides[0]);
        PyErr_SetString(PyExc_TypeError, err_str);
        PyBuffer_Release(&view);
        return NULL;
    }
    /* Check the type of items in the array */
    if (strcmp(view.format, "d") != 0)
    {
        PyErr_SetString(PyExc_TypeError, "Expected an array of doubles");
        PyBuffer_Release(&view);
        return NULL;
    }

    double eig_val = 0;
    double *eig_vec = (double *)malloc(sizeof(double) * view.ndim);

    if(InterpLinealg_Eigens((double *)(view.buf), view.ndim, eig_vec, &eig_val) != 0)
        return NULL;

    PyObject *ret_list = PyList_New(0);
    PyObject *list_item = PyList_New(view.ndim);
    if(ret_list == NULL || list_item ==NULL)
    {
        PyErr_SetString(PyExc_OSError, "Memory alocate failed");
        return NULL;
    }

    for(int i = 0; i < view.ndim; ++i)
    {
        PyList_SET_ITEM(list_item, i, PyFloat_FromDouble(eig_vec[i]));
    }

    if(PyList_Append(ret_list, PyFloat_FromDouble(eig_val)) < 0)
    {
        return NULL;
    }
    Py_DECREF(list_item);
    return ret_list;
}

static PyMethodDef linealgMethods[] = {
    {"inv",         linealg_Inverse,  METH_VARARGS, "matrix inverse."},
    {"eigens",      linealg_Eigens,   METH_VARARGS, "get the eigenvalue and eigenvector."},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

static struct PyModuleDef linealgmodule = {
    PyModuleDef_HEAD_INIT,
    "linealg",   /* name of module */
    NULL, /* module documentation, may be NULL */
    -1,       /* size of per-interpreter state of the module,
                 or -1 if the module keeps state in global variables. */
    linealgMethods
};

PyMODINIT_FUNC PyInit_linealg(void)
{
    PyObject *m;

    m = PyModule_Create(&linealgmodule);
    if (m == NULL)
        return NULL;

    LinealgModuleError = PyErr_NewException("LinealgModule.error", NULL, NULL);
    Py_XINCREF(LinealgModuleError);
    if (PyModule_AddObject(m, "error", LinealgModuleError) < 0)
    {
        Py_XDECREF(LinealgModuleError);
        Py_CLEAR(LinealgModuleError);
        Py_DECREF(m);
        return NULL;
    }

    return m;
}


