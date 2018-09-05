#ifndef MODBUS_MANAGER
#define MODBUS_MANAGER

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "parameter_manager/parameter_manager_param_group.h"
#include <string>
using namespace std;
#include "tcp_client.h"
#include "tcp_server.h"

#include "io_manager.h"

namespace fst_modbus
{
typedef unsigned long long int U64;

enum
{
    START_MODE_CLIENT = 0,
    START_MODE_SERVER = 1,
};

enum DeviceType
{
    UNKNOWN = 0,
    DIGITAL_INPUT_DEVICE = 1,
    DIGITAL_OUTPUT_DEVICE = 2,
    DIGITAL_IN_OUT_DEVICE = 3,
    ANALOG_INPUT_DEVICE = 4,
};

class ModbusManager
{
public:
    ModbusManager();
    ~ModbusManager(){}

	int getStartMode() { return START_MODE_CLIENT; }
	string getConfigIP() { return string("192.168.1.1"); }
	int getConfigPort() { return 1025; }

	
    //------------------------------------------------------------
    // Function:    init
    // Summary: Initialize
    // In:      None
    // Out:     None
    // Return:  0 -> success.
    //          ERROR_CODE -> failed.
    //------------------------------------------------------------
    U64 init(int fake = 0);
	
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
    U64 getDeviceInfo(unsigned int index, fst_io_manager::IODeviceInfo &info);

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

    // the thread cycle.
    static const int LOOP_CYCLE = 1000;

private:
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
	
private:
    fst_modbus::ModbusTCPClient * tcp_client;
    fst_modbus::ModbusTCPServer * tcp_server;
	
    // param to load & save
    int log_level_;
    // the thread object.
    boost::thread io_thread;
};

}

#endif


