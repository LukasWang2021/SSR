/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       fst_io_device.h
Author:     Feng.Wu
Create:     14-Oct-2018
Modify:     25-Oct-2018
Summary:    dealing with IO module
**********************************************/

#ifndef FST_IO_DEVICE_H
#define FST_IO_DEVICE_H

#include <vector>
#include <thread>
#include <mutex>
#include <string>
#include <map>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "base_device.h"
#include "common_log.h"
#include "error_code.h"
#include "fst_io_mem.h"
#include "fst_io_device_param.h"
#include "fst_io_manager.h"

#define IO_DEVICE_LOAD_PARAM_FAILED 123

//depressed soon
typedef struct _IOPortInfo
{
	uint32_t    msg_id;
	uint32_t    dev_id;
	int         port_type;
	int         port_index;
	int         bytes_len;
}IOPortInfo;


namespace fst_hal
{
	
class FstIoDevice : public BaseDevice
{
public:
    FstIoDevice(int address);
    ~FstIoDevice();

    virtual bool init();

	//------------------------------------------------------------
	// Function:    getIODevices
	// Summary: get the info of all the device.
	// In:      None
	// Out:     None
	// Return:  the vector of all io devices.
	//------------------------------------------------------------
	std::vector<fst_hal::IODeviceInfo> getIODeviceList(void);

	ErrorCode getDevInfoByID(uint8_t address, IODeviceInfo &info);

    //------------------------------------------------------------
    // Function:    getDevicePortValues
    // Summary: get the information of each device.
    // In:      address -> the address is from io board.
    // Out:     values  -> the information of each device.
    // Return:  ErrorCode   -> error codes.
    //------------------------------------------------------------
 	ErrorCode getDevicePortValues(uint8_t address, fst_hal::IODevicePortValues &values);

	//------------------------------------------------------------
	// Function:    setDIOByBit
	// Summary: Set the output to the specified port.
	// In:      physics_id -> the specified physical port.
	//          value      -> 1 = ON, 0 = OFF.
	// Out:     None.
	// Return:  ErrorCode   -> error codes.
	//------------------------------------------------------------
    ErrorCode setDIOByBit(uint32_t physics_id, uint8_t value);

	//------------------------------------------------------------
	// Function:    getDIOByBit
	// Summary: get the value of the specified port.
	// In:      physics_id -> the specified physical port.
	// Out:     value      -> 1 = ON, 0 = OFF.
	// Return:  ErrorCode   -> error codes.
	//------------------------------------------------------------
    ErrorCode getDIOByBit(uint32_t physics_id, uint8_t &value);


	//------------------------------------------------------------
	// Function:    getDevicesNum
	// Summary: get the number of devices without freshening.
	// In:      None
	// Out:     None
	// Return:  int -> the total number of io devices.
	//------------------------------------------------------------
	int getIODevNum(void);

	//------------------------------------------------------------
	// Function:    refreshIODevNum
	// Summary: refresh the IO device slots.
	// In:      None
	// Out:     None
	// Return:  int -> the total number of io devices.
	//------------------------------------------------------------
	int refreshIODevNum(void);
    
	ErrorCode setDO(int physics_id, unsigned char value){return 0;} //depressed soon.
	ErrorCode getDIO(int physics_id, uint8_t *buffer, int buf_len, int& io_bytes_len){return 0;}//depressed soon
	ErrorCode getDIO(IOPortInfo *io_info, uint8_t *buffer, int buf_len){return 0;}//depressed soon
	ErrorCode checkIO(const char *path, IOPortInfo* io_info){return 0;}//depressed soon
	

private:
    FstIoDeviceParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    FstIoDevice();
    int io_num_;
};

}

#endif

