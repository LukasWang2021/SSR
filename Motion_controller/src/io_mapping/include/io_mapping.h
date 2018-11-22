#ifndef IO_MAPPING_H
#define IO_MAPPING_H

#include<stdio.h>
#include<stdlib.h>
#include<memory.h>
#include<string.h>
#include<string>
#include<vector>
#include<map>
#include<algorithm>
#include "io_mapping_cJSON.h"
#include "io_mapping_param.h"
#include "common_log.h"
#include "base_datatype.h"
#include "base_device.h"
#include "io_simulation.h"
#include "fst_io_device.h"
#include "error_code.h"


using namespace std;

namespace fst_ctrl
{
	
typedef struct _IOMapJsonInfo
{
	int  from;
	int  index;
	char module[128];
	int  to;
}IOMapJsonInfo;

class IoMapping
{
public:
    IoMapping();
    ~IoMapping();

	//------------------------------------------------------------
	// Function:    init
	// Summary:
	// In:      None.
	// Out:     None.
	// Return:  1  -> success.
	//          -  -> failed.
	//------------------------------------------------------------
	ErrorCode init(fst_hal::FstIoDevice* io_device_ptr);

	//------------------------------------------------------------
	// Function:    updateMappingFile
	// Summary: read again the io mapping json files.
	// In:      None.
	// Out:     None.
	// Return:  1  -> success.
	//          -1 -> failed.
	//------------------------------------------------------------
	ErrorCode updateMappingFile(void);

	ErrorCode updateSimFile(void);

	//------------------------------------------------------------
	// Function:    getDIByBit
	// Summary: get the value to the user port.
	// In:      user_port  -> the port defined by the user.
	// Out      value      -> 1 = ON, 0 = OFF.
	// Return:  ErrorCode   -> error codes.
	//------------------------------------------------------------
    ErrorCode getDIByBit(uint32_t user_port, uint8_t &value);

	//------------------------------------------------------------
	// Function:    setDIByBit
	// Summary: Set the value to the the port.
	// In:      user_port  -> the port defined by the user.
	//          value      -> 1 = ON, 0 = OFF.
	// Out:     None.
	// Return:  ErrorCode   -> error codes.
	//------------------------------------------------------------
    ErrorCode setDIByBit(uint32_t user_port, uint8_t value);

	//------------------------------------------------------------
	// Function:    getDOByBit
	// Summary: get the value to the user port.
	// In:      user_port  -> the port defined by the user.
	// Out:     value      -> 1 = ON, 0 = OFF.
	// Return:  ErrorCode   -> error codes.
	//------------------------------------------------------------
    ErrorCode getDOByBit(uint32_t user_port, uint8_t &value);

	//------------------------------------------------------------
	// Function:    setDOByBit
	// Summary: Set the output to the the port.
	// In:      user_port  -> the port defined by the user.
	//          value      -> 1 = ON, 0 = OFF.
	// Out:     None.
	// Return:  ErrorCode   -> error codes.
	//------------------------------------------------------------
    ErrorCode setDOByBit(uint32_t user_port, uint8_t value);

	//------------------------------------------------------------
	// Function:    getRIByBit
	// Summary: get the value to the user port.
	// In:      user_port  -> the port defined by the user.
	// Out      value      -> 1 = ON, 0 = OFF.
	// Return:  ErrorCode   -> error codes.
	//------------------------------------------------------------
	ErrorCode getRIByBit(uint32_t user_port, uint8_t &value);

	//------------------------------------------------------------
	// Function:    setRIByBit
	// Summary: Set the value to the the port.
	// In:      user_port  -> the port defined by the user.
	//          value      -> 1 = ON, 0 = OFF.
	// Out:     None.
	// Return:  ErrorCode   -> error codes.
	//------------------------------------------------------------
	ErrorCode setRIByBit(uint32_t user_port, uint8_t value);

	//------------------------------------------------------------
	// Function:    getROByBit
	// Summary: get the value to the user port.
	// In:      user_port  -> the port defined by the user.
	// Out:     value      -> 1 = ON, 0 = OFF.
	// Return:  ErrorCode   -> error codes.
	//------------------------------------------------------------
	ErrorCode getROByBit(uint32_t user_port, uint8_t &value);

	//------------------------------------------------------------
	// Function:    setROByBit
	// Summary: Set the output to the the port.
	// In:      user_port  -> the port defined by the user.
	//          value      -> 1 = ON, 0 = OFF.
	// Out:     None.
	// Return:  ErrorCode   -> error codes.
	//------------------------------------------------------------
	ErrorCode setROByBit(uint32_t user_port, uint8_t value);

	//------------------------------------------------------------
	// Function:    getIOPhysicsID
	// Summary: get the mapping id according to the user port.
	//          the physics id is used in fst_io_device.
	// In:      key  -> the user defined IO such as DI[1], DO[2].
	// Out:     None.
	// Return:  uint32_t   -> the physics id.
	//------------------------------------------------------------
    uint32_t getIOPhysicsID(string key) {return io_mapper_[key];}

private:
	void loadProgramsPath(void);
	char * getProgramsPath(void);
    bool generateIOInfo(IOMapJsonInfo &objInfo, const char * strIOType);
	bool parseIOObject(cJSON *jsonIObject, const char * strIOType);
	bool parseIO(cJSON *jsonDI, const char * strIOType);
	bool parseIOMap(char * data, const char * strIOType);
	bool printIOMapper(void);
	bool appendSingleIOMapper(char *filename, const char * strIOType);
	vector<string> split(string str,string pattern);

    IoMappingParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    IoSimulation* sim_ptr_;
    fst_hal::FstIoDevice* io_dev_ptr_;
    map<string, uint32_t> io_mapper_; // to do uint32_t tobe PhysicsID
	std::string files_manager_data_path_ = "";
};

}

#endif


