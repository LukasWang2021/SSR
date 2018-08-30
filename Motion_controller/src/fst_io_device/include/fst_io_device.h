#ifndef FST_IO_DEVICE_H
#define FST_IO_DEVICE_H

#include "base_device.h"
#include "fst_io_device_param.h"
#include "common_log.h"
#include "error_code.h"
#include <vector>
#include <thread>
#include <mutex>
#include <string>
#include <map>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "IOboard.h"

#include "io_interface.h"

namespace fst_hal
{
	
typedef unsigned long long int U64;
	
	// IO Macro and structure begin
#ifndef IO_BASE_ADDRESS
#define IO_BASE_ADDRESS (100000)
#endif
	
#ifndef IO_MAX_NUM
#define IO_MAX_NUM      (1000)
#endif
	
#ifndef IO_INPUT
#define IO_INPUT 0
#endif
	
#ifndef IO_OUTPUT
#define IO_OUTPUT 1
#endif
	
#ifndef IO_DATAFRAME_MAX
#define IO_DATAFRAME_MAX 5
#endif

class FstIoDevice : public BaseDevice
{
public:
    FstIoDevice(int address);
    ~FstIoDevice();

    virtual bool init();
	
public:
    //------------------------------------------------------------
    // Function:    getDevicesNum
    // Summary: get the number of devices
    // In:      None
    // Out:     None
    // Return:  int -> the total number of io devices.
    //------------------------------------------------------------
    int getDevicesNum(void);

	vector<fst_io_manager::IODeviceInfo> getIODevices();

    //------------------------------------------------------------
    // Function:    getDeviceInfo
    // Summary: get the information of each device.
    // In:      index -> the sequence number of the device.
    // Out:     info  -> the information of each device.
    // Return:  U64   -> error codes.
    //------------------------------------------------------------
    U64 getDeviceInfo(unsigned int index, fst_io_manager::IODeviceInfo &info);

	
	 /**
	  * @brief: set DO 
	  *
	  * @param path: path of set DO
	  * @param value: value to set
	  *
	  * @return: 0 if success 
	  */
	 U64 setDO(const char *path, char value);
	
	 /**
	  * @brief: set DO 
	  *
	  * @param msg_id: id of this DO
	  * @param value: value to set
	  *
	  * @return: 0 if success 
	  */
	 U64 setDO(int msg_id, unsigned char value);
	
	 /**
	  * @brief: set DO 
	  *
	  * @param io_info: port infomation of DO
	  * @param value: value to set
	  *
	  * @return: 0 if success 
	  */
	 U64 setDO(IOPortInfo *io_info, char value);
	
	 /**
	  * @brief:get DI and DO 
	  *
	  * @param path: path to get
	  * @param buffer: buffer to store value 
	  * @param buf_len: buffer len
	  * @param io_bytes_len: actual length
	  *
	  * @return: 0 if success 
	  */
	 U64 getDIO(const char *path, unsigned char *buffer, int buf_len, int& io_bytes_len);
	
	/**
	  * @brief:get DI and DO 
	  *
	  * @param msg_id: id to get
	  * @param buffer: buffer to store value 
	  * @param buf_len: buffer len
	  * @param io_bytes_len: actual length
	  *
	  * @return: 0 if success 
	  */
	 U64 getDIO(int msg_id, uint8_t *buffer, int buf_len, int& io_bytes_len);
	  /**
	  * @brief:get DI and DO 
	  *
	  * @param io_info: infomation of DI or DO
	  * @param buffer: buffer to store value 
	  * @param buf_len: buffer len
	  *
	  * @return: 0 if success 
	  */
	 U64 getDIO(IOPortInfo *io_info, uint8_t *buffer, int buf_len);
	 
	 /**
	  * @brief: check valid of this DI or DO 
	  *
	  * @param path: path of this DI or DO
	  * @param io_info: io infomation 
	  *
	  * @return: true if it is valid 
	  */
	 U64 checkIO(const char *path, IOPortInfo* io_info);
	
	 /**
	  * @brief: get index of this IO device
	  *
	  * @param dev_address: address of device
	  *
	  * @return: index of IO device 
	  */
	 int getIODevIndex(int dev_address);
	
	 /**
	  * @brief: update IO error 
	  */
	 U64 updateIOError();
	 

    // for the convert between parameter id and device id.
    static const int ID_DIFF = 100000; 
    static const int MULTIPLIER = 1000; 

    // the total number of device on RS485.
    static const int RS_DEV_NUM = 4;

    // the faulty tolerance times.
    static const int FAULT_TOL = 20;

    // the thread cycle.
    static const int LOOP_CYCLE = 1000;
	
private:
    FstIoDeviceParam* param_ptr_;
    fst_log::Logger* log_ptr_;

    FstIoDevice();

	
private:

    // the flags to read and write FPGA.
    uint8_t seq_;

    // the object to operate on the configuration file.
    fst_parameter::ParamGroup param_;

};

}

#endif

