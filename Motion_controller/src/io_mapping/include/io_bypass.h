/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       io_bypass.h
Author:     Feng.Wu
Create:     30-Jan-2019
Modify:     30-Jan-2019
Summary:    to handle ui bypass values.
**********************************************/
#ifndef IO_BYPASS_H
#define IO_BYPASS_H

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


using namespace std;

namespace fst_ctrl
{

typedef struct _IOStatusJsonBypass
{
    char useBypass[8];
    char bypassValue[8];
    int  userPort;
}IOStatusJsonBypass;

class IoBypass
{
public:
    IoBypass(fst_log::Logger *logger, IoMappingParam *param);
    ~IoBypass();

    ErrorCode init();

    //------------------------------------------------------------
    // Function:    updateBypassFile
    // Summary: read again the io status json files.
    // In:      None.
    // Out:     None.
    // Return:  ErrorCode
    //------------------------------------------------------------
    ErrorCode updateBypassFile(void);

    //------------------------------------------------------------
    // Function:    isBypassUsed
    // Summary: check if the user port is in the Bypass status.
    // In:      userString  ->the user defined IO such as DI[1], RI[2]
    // Out:     value       ->high or low.
    // Return:  true  -> the port is in Bypass status.
    //          false -> not.
    //------------------------------------------------------------
    bool isBypassUsed(const char * userString, bool & value);

    //------------------------------------------------------------
    // Function:    setBypassValue
    // Summary: set the user port value.
    // In:      userString  ->the user defined IO such as DI[1], RI[1]
    //          yes_or_no   ->whether to set bypass
    // Out:     None.
    // Return:  true  -> set success.
    //          false -> the port is not in Bypass status.
    //------------------------------------------------------------
    bool setBypass(const char * userString, bool yes_or_no);


private:

    void loadProgramsPath(void);
    char * getProgramsPath(void);
    int generateIOInfo(IOStatusJsonBypass &objInfo, const char * strIOType);
    int parseIOObject(cJSON *jsonIObject, const char * strIOType);
    int parseIO(cJSON *jsonDI, const char * strIOType);
    int parseIOMap(char * data, const char * strIOType);
    int printIOMapper(void);
    int appendSingleIOMapper(char *filename, const char * strIOType);
    vector<string> split(string str,string pattern);

    IoMappingParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    map<string, bool> io_bypass_;
    map<string, bool> value_map_;
    std::string files_manager_data_path_ = "";

};

}

#endif

