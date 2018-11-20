/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       io_simulation.h
Author:     Feng.Wu
Create:     29-Oct-2018
Modify:     29-Oct-2018
Summary:    to store the simulation values.
**********************************************/
#ifndef IO_SIMULATION_H
#define IO_SIMULATION_H

#include<stdio.h>
#include<string.h>
#include<string>
#include<vector>
#include<map>
#include "io_mapping_cJSON.h"
#include "io_mapping_param.h"
#include "common_log.h"
#include "base_datatype.h"
#include "base_device.h"
#include "error_code.h"

#define IO_MAPPING_LOAD_SIM_FILE_FAILED (unsigned long long int)0x00100002008F03FE   /*failed to load simused status json files*/

using namespace std;

namespace fst_ctrl
{

typedef struct _IOStatusJsonSim
{
    char simUsed[8];
    char simValue[8];
    int  userPort;
}IOStatusJsonSim;

class IoSimulation
{
public:
    IoSimulation(fst_log::Logger *logger, IoMappingParam *param);
    ~IoSimulation();

    ErrorCode init();

    //------------------------------------------------------------
    // Function:    updateSimFile
    // Summary: read again the io status json files.
    // In:      None.
    // Out:     None.
    // Return:  ErrorCode
    //------------------------------------------------------------
    ErrorCode updateSimFile(void);

    //------------------------------------------------------------
    // Function:    isSimUsed
    // Summary: check if the user port is in the sim status.
    // In:      userString  ->the user defined IO such as DI[1], RI[2]
    // Out:     value       ->high or low.
    // Return:  true  -> the port is in sim status.
    //          false -> not.
    //------------------------------------------------------------
    bool isSimUsed(const char * userString, bool & value);

    //------------------------------------------------------------
    // Function:    setSimValue
    // Summary: set the user port value.
    // In:      userString  ->the user defined IO such as DI[1], RI[1]
    //          value       ->high or low.
    // Out:     None.
    // Return:  true  -> set success.
    //          false -> the port is not in sim status.
    //------------------------------------------------------------
    bool setSimValue(const char * userString, bool value);


private:

    void loadProgramsPath(void);
    char * getProgramsPath(void);
    int generateIOInfo(IOStatusJsonSim &objInfo, const char * strIOType);
    int parseIOObject(cJSON *jsonIObject, const char * strIOType);
    int parseIO(cJSON *jsonDI, const char * strIOType);
    int parseIOMap(char * data, const char * strIOType);
    int printIOMapper(void);
    int appendSingleIOMapper(char *filename, const char * strIOType);
    vector<string> split(string str,string pattern);

    IoMappingParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    map<string, bool> io_sim_;
    std::string files_manager_data_path_ = "";

};

}

#endif

