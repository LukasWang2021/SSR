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
    PyObject *list;

    if (!PyArg_ParseTuple(args, "O", &list))
        return NULL;

    Py_ssize_t len_r = PyList_GET_SIZE(list);
    if(len_r <= 1)
    {
        PyErr_SetString(PyExc_TypeError, "Invalid matrix dimension");
        return NULL;
    }

    Py_ssize_t len_c = 0;
    double *p_mat = (double *)malloc(sizeof(double) * len_r * len_r);
    if(p_mat == NULL)
    {
        PyErr_SetString(PyExc_OSError, "Memoey allocate failed for current matrix");
        return NULL;
    }

    for(Py_ssize_t i = 0; i < len_r; ++i)
    {
        PyObject *v = PyList_GET_ITEM(list, i);
        len_c = PyList_GET_SIZE(v);
        if(len_c != len_r)
        {
            PyErr_SetString(PyExc_TypeError, "Invalid matrix shape");
            return NULL;
        }
        for(Py_ssize_t j = 0; j < len_c; ++j)
        {
            p_mat[i * len_r + j] = PyFloat_AS_DOUBLE(PyList_GET_ITEM(v, j));
            // printf("#%lf ", p_mat[i * len_r + j]);
        }
        // printf("\n");
    }

    double *p_inv = (double *)malloc(sizeof(double) * len_r * len_r);
    if(p_inv == NULL)
    {
        PyErr_SetString(PyExc_OSError, "Memoey allocate failed for inverse matrix");
        return NULL;
    }

    if(InterpLinealg_Inverse(p_mat, len_r, p_inv) != 0)
    {
        PyErr_SetString(PyExc_OSError, "Calculate matrix inverse failed");
        return NULL;
    }

    PyObject *ret_list = PyList_New(0);
    if(ret_list == NULL)
    {
        PyErr_SetString(PyExc_OSError, "Memory alocate failed for result list");
        return NULL;
    }

    for(int i = 0; i < len_r; ++i)
    {
        PyObject *list_item = PyList_New(len_c);
        if(ret_list == NULL)
        {
            PyErr_SetString(PyExc_OSError, "Memory alocate failed for result list item");
            return NULL;
        }
        for(int j = 0; j < len_c; ++j)
        {
            // printf("@%lf ", p_inv[i * len_r + j]);
            PyList_SET_ITEM(list_item, j, PyFloat_FromDouble(p_inv[i * len_r + j]));
        }
        // printf("\n");
        if(PyList_Append(ret_list, list_item) < 0)
        {
            PyErr_SetString(PyExc_OSError, "Append result list failed");
            Py_DECREF(list_item);
            return NULL;
        }
        Py_DECREF(list_item);
    }

    free(p_inv);
    free(p_mat);
    return ret_list;
}

static PyObject *linealg_Eigens(PyObject *self, PyObject *args)
{
    return NULL;
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


