/* This module provide the interface to option
 * python enviroment or call python interface
 */

#ifndef INTERPRETER_EMBED_H
#define INTERPRETER_EMBED_H

#define PY_SSIZE_T_CLEAN
#include "Python.h"
#include "string"
#include "interpreter_config.h"


class InterpEmbed
{
private:
    Py_tracefunc tracer_;         /*tracer function for python eval trace*/
    InterpConfig config_;        /*configuration file*/
    PyObject trace_obj_;

public:
    InterpEmbed(/* args */);
    ~InterpEmbed();

    bool pyResetInterp(void); /*reset the python interpreter*/
    
    bool pyUpdatePath(void); /*update the python module search path specified by config file*/

    void pyRunFile(const std::string& file); /*run the python script 'file'.
    The file is search from the path specified by configuration file 'program_path' item*/
    void pyRunString(const std::string& str); /*run the python script 'str'*/
};


#endif

