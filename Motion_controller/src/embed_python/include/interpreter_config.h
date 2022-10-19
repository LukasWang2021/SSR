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
    int prog_thread_priority_;
    int state_thread_priority_;
    int state_thread_cycle_time_;
public:
    InterpConfig(/* args */);
    ~InterpConfig();

    bool loadConfig(void);

    std::string getProgPath(void);
    std::string getModulePath(void);
    int progThreadPriority(void);
    int stateThreadPriority(void);
    int stateCycleTime(void);
};




#endif
