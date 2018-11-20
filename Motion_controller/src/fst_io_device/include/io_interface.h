/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       io_interface.h
Author:     Feng.Wu WeiWang
Create:     12-Jun-2017
Modify:     25-Oct-2018
Summary:    dealing with IO module
**********************************************/

#ifndef TP_INTERFACE_IO_INTERFACE_H_
#define TP_INTERFACE_IO_INTERFACE_H_

#ifndef WIN32
#include <atomic>
#include <vector>
#include "fst_io_manager.h"
#endif
#include "common_log.h"
#include "fst_io_device_param.h"

/*//depressed soon
typedef struct _IOPortInfo
{
    uint32_t    msg_id;
    uint32_t    dev_id;
    int         port_type;
    int         port_index;
    int         bytes_len;
}IOPortInfo;
*/

namespace fst_hal
{

    class IOInterface {
    public:
        IOInterface(fst_log::Logger *logger, fst_hal::FstIoDeviceParam *param);

        ~IOInterface();

        static IOInterface *instance(fst_log::Logger *logger, fst_hal::FstIoDeviceParam *param);

        ErrorCode initial(){return 0;}

        //------------------------------------------------------------
        // Function:    getDevicesNum
        // Summary: get the number of devices without freshening.
        // In:      None
        // Out:     None
        // Return:  int -> the total number of io devices.
        //------------------------------------------------------------
        int getIODevNum(){return 0;}

        //------------------------------------------------------------
        // Function:    refreshIODevNum
        // Summary: refresh the IO device slots.
        // In:      None
        // Out:     None
        // Return:  int -> the total number of io devices.
        //------------------------------------------------------------
        int refreshIODevNum(){return 0;}

        //------------------------------------------------------------
        // Function:    getIODevices
        // Summary: get the info of all the device.
        // In:      None
        // Out:     None
        // Return:  the vector of all io devices.
        //------------------------------------------------------------
        std::vector <fst_hal::IODeviceInfo> getIODeviceList(){std::vector<fst_hal::IODeviceInfo> v;return v;}

        //------------------------------------------------------------
        // Function:    getDeviceInfo
        // Summary: get the information of each device.
        // In:      index -> the sequence number of the device.
        // Out:     info  -> the information of each device.
        // Return:  ErrorCode   -> error codes.
        //------------------------------------------------------------
        ErrorCode getDevicePortValues(uint8_t address, fst_hal::IODevicePortValues &values){return 0;}

        //------------------------------------------------------------
        // Function:    setDIOByBit
        // Summary: Set the output to the specified port.
        // In:      physics_id -> the specified physical port.
        //          value      -> 1 = ON, 0 = OFF.
        // Out:     None.
        // Return:  ErrorCode   -> error codes.
        //------------------------------------------------------------
        ErrorCode setDIOByBit(uint32_t physics_id, uint8_t value){return 0;}

        //------------------------------------------------------------
        // Function:    getDIOByBit
        // Summary: get the value of the specified port.
        // In:      physics_id -> the specified physical port.
        // Out:     value      -> 1 = ON, 0 = OFF.
        // Return:  ErrorCode   -> error codes.
        //------------------------------------------------------------
        ErrorCode getDIOByBit(uint32_t physics_id, uint8_t &value){return 0;}


        ErrorCode getDIO(int physics_id, uint8_t *buffer, int buf_len, int &io_bytes_len) { return 0; }//depressed


        fst_hal::IODeviceInfo *getDevInfoPtr() { return dev_info_; }

    private:
        IOInterface();

        fst_hal::IOManager *io_manager_;
        std::atomic<fst_hal::IODeviceInfo *> dev_info_; // delete soon

        std::atomic_int io_num_;    //number of IO board
        bool is_virtual_;


        fst_log::Logger *log_ptr_;
        fst_hal::FstIoDeviceParam *param_ptr_;
    };

}

#endif
