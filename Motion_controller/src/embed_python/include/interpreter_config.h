#ifndef INTERPRETER_CONFIG_H
#define INTERPRETER_CONFIG_H

#include "yaml_help.h"

class InterpConfig
{
private:
    base_space::YamlHelp yaml_help_;
    /*The configuration file path*/
    std::string file_path_;
    /*Tell interpreter control where to find the program file*/
    std::string prog_path_;
    /*The robot system module path.This path will add to the python runtime.*/
    std::string module_path_;
    // thread priority
    int interp_thread_priority_;
public:
    InterpConfig(/* args */);
    ~InterpConfig();

    bool loadConfig(void);
    
    std::string getProgPath(void);
    std::string getModulePath(void);
    int getThreadPriority(void);
};




#endif
