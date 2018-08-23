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

namespace fst_hal
{
	
typedef unsigned long long int U64;
#define PARSE_IO_PATH_FAILED                    (unsigned long long int)0x0001000400670009   /*cant use current path to set IO*/
#define INVALID_PATH_FROM_TP                    (unsigned long long int)0x0001000200830004   /*tp sent invalid path*/
	
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

enum IODeviceType
{
    UNKNOWN = 0,
    DIGITAL_INPUT_DEVICE = 1,
    DIGITAL_OUTPUT_DEVICE = 2,
    DIGITAL_IN_OUT_DEVICE = 3,
    ANALOG_INPUT_DEVICE = 4,
};

// This is output info.
struct IODeviceInfo
{
    std::string path;
    unsigned int id;
    std::string communication_type;
    int device_number;
    IODeviceType device_type;
    unsigned int input;
    unsigned int output;
};


// Local table
struct IOTable
{
    IODeviceData data;
    IODeviceInfo info;
    unsigned char output[IO_DATAFRAME_MAX]; // store the output request from tp.
};

enum ThreadStatus
{
    INIT_STATUS = 1,
    RUNNING_STATUS = 2,
};

enum ThreadError
{
    THREAD_SUCCESS = 0,
    THREAD_LOAD_IO_CONFIG_FAIL = 1,
    THREAD_GET_IO_FAIL = 2,
    THREAD_IO_DEVICE_CHANGED = 3,
};


class FstIoDevice : public BaseDevice
{
public:
    FstIoDevice(int address);
    ~FstIoDevice();

    virtual bool init();

    void getDeviceInfo();
    int getInputByteSize();
    int getOutputByteSize();

    std::vector<uint8_t> getAllInput();
    std::vector<uint8_t> getAllOutput();
    bool getInputByByte(int byte_offset, uint8_t& data);
    bool getOutputByByte(int byte_offset, uint8_t& data);
    bool getInputByBit(int bit_offset, bool data);
    bool getOutputByBit(int bit_offset, bool data);
    
    bool setAllInput(std::vector<uint8_t>& input);
    bool setAllOutput(std::vector<uint8_t>& output);
    bool setInputByByte(int byte_offset, uint8_t data);
    bool setOutputByByte(int byte_offset, uint8_t data);
    bool setInputByBit(int bit_offset, uint8_t data);
    bool setOutputByBit(int bit_offset, uint8_t data);
	
public:
    //------------------------------------------------------------
    // Function:    getDevicesNum
    // Summary: get the number of devices
    // In:      None
    // Out:     None
    // Return:  int -> the total number of io devices.
    //------------------------------------------------------------
    int getDevicesNum(void);

    //------------------------------------------------------------
    // Function:    getDeviceInfo
    // Summary: get the information of each device.
    // In:      index -> the sequence number of the device.
    // Out:     info  -> the information of each device.
    // Return:  U64   -> error codes.
    //------------------------------------------------------------
    U64 getDeviceInfo(unsigned int index, IODeviceInfo &info);

    //------------------------------------------------------------
    // Function:    getModuleValue
    // Summary: get the status value of one port on one device.
    // In:      id         -> the parameter id according to the parameter path.
    //          port_type  -> IO_INPUT or IO_output.
    //          port_seq   -> the sequence number of the ports.
    // Out:     port_value -> the port status.
    // Return:  U64        -> error codes.
    //------------------------------------------------------------
    U64 getModuleValue(unsigned int id, int port_type, unsigned int port_seq, unsigned char &port_value);

    //------------------------------------------------------------
    // Function:    getModuleValues
    // Summary: get the status values of all ports on one device
    // In:      id  -> the parameter id according to the parameter path.
    //          len -> the size of the memory to store port values.
    // Out:     ptr -> the address of the memoryto store port values.
    //          num -> the total number of the ports.
    // Return:  U64 -> error codes.
    //------------------------------------------------------------
    U64 getModuleValues(unsigned int id, int len, unsigned char *ptr, int &num);

    //------------------------------------------------------------
    // Function:    setModuleValue
    // Summary: set the status value of one port on one device.
    // In:      id         -> the parameter id according to the parameter path.
    //          port_seq   -> the sequence number of the port.
    //          port_value -> the status value of the port.
    // Out:     None.
    // Return:  U64        -> error codes.
    //------------------------------------------------------------
    U64 setModuleValue(unsigned int id, unsigned int port_seq, unsigned char port_value);

    //------------------------------------------------------------
    // Function:    getThreadError
    // Summary: get the error status of the io thread.
    // In:      None.
    // Out:     None.
    // Return:  U64 -> error codes.
    //------------------------------------------------------------
    U64 getIOError(void);

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
    std::vector<uint8_t> input_;
    std::vector<uint8_t> output_;
    std::thread update_thread_;
    boost::mutex mutex_;  // data protection

    FstIoDevice();
    void updateThreadFunc();    // data exchange 

	
private:

    // the flags to read and write FPGA.
    uint8_t seq_;

    // the counter for the error times of reading FPGA.
    int read_counter_;

    // the counter for the error times of writing FPGA.
    int write_counter_;

    // to record the last error when thread runs.
    ThreadError last_error_;

    // to indicate the thread has error once. Lock it when used.
    ThreadError thread_error_;

    // the status of the io thread.
    ThreadStatus thread_status_;

    // The buffer to store all the reading io data. Lock it when used.
    std::vector<IOTable> io_r_;

    // the map to store the configuration file.
    std::map<uint8_t, std::string> name_map_;
    std::map<uint8_t, int> input_map_;
    std::map<uint8_t, int> output_map_;
    std::map<uint8_t, IODeviceType> type_map_;

    // the map to store the error codes.
    std::map<int, U64> error_map_;

    // the object to operate on the configuration file.
    fst_parameter::ParamGroup param_;

    // the thread object.
    boost::thread io_thread;
    
    //------------------------------------------------------------
    // Function:    searchParamId
    // Summary: find the index of table according to the parameter id.
    // In:      id  -> the parameter id according to the parameter path.
    //          io  -> the table to store all the io data.
    // Out:     None.
    // Return:  int -> the index of the table.
    //          -1  -> can't find.
    //------------------------------------------------------------
    int searchParamId(unsigned int id, std::vector<IOTable> &io);

    //------------------------------------------------------------
    // Function:    startThread
    // Summary: start a thread.
    // In:      None.
    // Out:     None.
    // Return:  None.
    //------------------------------------------------------------
    void startThread(void);

    //------------------------------------------------------------
    // Function:    runThread
    // Summary: main function of io thread.
    // In:      None.
    // Out:     None.
    // Return:  None.
    //------------------------------------------------------------
    void runThread(void);

    //------------------------------------------------------------
    // Function:    initDeviceData
    // Summary: read io and load configuration file when the thread is starting up.
    // In:      None.
    // Out:     None.
    // Return:  None.
    //------------------------------------------------------------
    void initDevicesData(void);

    //------------------------------------------------------------
    // Function:    updateDeviceData
    // Summary: refreshen the io data.
    // In:      None.
    // Out:     None.
    // Return:  U64 -> error codes.
    //------------------------------------------------------------
    U64 updateDevicesData(void);

    //------------------------------------------------------------
    // Function:    setThreadError
    // Summary: set the thread_error_.
    // In:      None.
    // Out:     None.
    // Return:  None.
    //------------------------------------------------------------
    void setThreadError(void);

    //------------------------------------------------------------
    // Function:    initIOTableVar
    // Summary: initialize the variable of IOTable structure.
    // In:      None.
    // Out:     io -> variable.
    // Return:  None.
    //------------------------------------------------------------
    void initIOTableVar(IOTable &io);

    //------------------------------------------------------------
    // Function:    implementConfigFile
    // Summary: try to load configuration files to fill the complete information.
    // In:      None.
    // Out:     io -> fill io.info.
    // Return:  None.
    //------------------------------------------------------------
    void implementConfigFile(IOTable &io);

    //------------------------------------------------------------
    // Function:    loadConfigFile
    // Summary: try to load configuration files to fill the complete information.
    // In:      model -> the model type of the device.
    // Out:     None.
    // Return:  true  -> load file successfully.
    //          false -> load file failed.
    //------------------------------------------------------------
    bool loadConfigFile(int model);

    //------------------------------------------------------------
    // Function:    getPathOfDevice
    // Summary: create the parameter path.
    // In:      type          -> the communication type.
    //          device_number -> the id in device.
    // Out:     None.
    // Return:  std::string   -> the parameter path.
    //------------------------------------------------------------
    std::string getPathOfDevice(std::string type, int device_number);

    //------------------------------------------------------------
    // Function:    getDeviceDataBy485
    // Summary: get io data by RS485 and check data.
    // In:      None.
    // Out:     data -> io data from RS485.
    // Return:  U64  -> error codes.
    //------------------------------------------------------------
    U64 getDeviceDataBy485(IODeviceData &data);

    //------------------------------------------------------------
    // Function:    readWriteBy485Driver
    // Summary: interact with FPGA to get data.
    // In:      None.
    // Out:     data  -> io data from RS485.
    // Return:  true  -> get data successfully.
    //          false -> get data failed.
    //------------------------------------------------------------
    bool readWriteBy485Driver(IODeviceData &data);

/*    // simulate driver function
    int ioSetIdSeq(uint8_t idseq);
    int ioGetSeq(uint8_t *seq);
    int ioWriteDownload(struct IODeviceData *idd);
    int ioReadUpload(struct IODeviceData *idd);
*/
};

}

#endif

