#include "fst_io_device.h"
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <boost/date_time/posix_time/posix_time.hpp>

using namespace std;


using namespace fst_hal;

FstIoDevice::FstIoDevice(int address):
    BaseDevice(address, fst_hal::DEVICE_TYPE_FST_IO),
    log_ptr_(NULL),
    param_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new FstIoDeviceParam();
	
    seq_ = 1;
}

FstIoDevice::~FstIoDevice()
{

}


bool FstIoDevice::init()
{
	// IOInterface::instance would call initial
	int iIONum = IOInterface::instance()->getIODevNum();
    return SUCCESS;
}

//------------------------------------------------------------
// Function:    getDevicesNum
// Summary: get the number of devices
// In:      None
// Out:     None
// Return:  int -> the total number of io devices.
//------------------------------------------------------------
int FstIoDevice::getDevicesNum(void)
{
	return IOInterface::instance()->getIODevNum();
}

vector<fst_io_manager::IODeviceInfo> FstIoDevice::getIODevices()
{
    return IOInterface::instance()->getIODevices();
}

//------------------------------------------------------------
// Function:    getDeviceInfo
// Summary: get the information of each device.
// In:      index -> the sequence number of the device.
// Out:     info  -> the information of each device.
// Return:  U64   -> error codes.
//------------------------------------------------------------
U64 FstIoDevice::getDeviceInfo(unsigned int index, fst_io_manager::IODeviceInfo &info)
{
    return IOInterface::instance()->getDeviceInfo(index, info);
}

U64 FstIoDevice::setDO(const char *path, char value)
{
	return IOInterface::instance()->setDO(path, value);
}

U64 FstIoDevice::setDO(int msg_id, unsigned char value)
{
	return IOInterface::instance()->setDO(msg_id, value);
}

U64 FstIoDevice::setDO(IOPortInfo *io_info, char value)
{
	return IOInterface::instance()->setDO(io_info, value);
}

U64 FstIoDevice::getDIO(const char *path, unsigned char *buffer, int buf_len, int& io_bytes_len)
{
	return IOInterface::instance()->getDIO(path, buffer, buf_len, io_bytes_len);
}

U64 FstIoDevice::getDIO(int msg_id, uint8_t *buffer, int buf_len, int& io_bytes_len)
{
	return IOInterface::instance()->getDIO(msg_id, buffer, buf_len, io_bytes_len);
}

U64 FstIoDevice::getDIO(IOPortInfo *io_info, uint8_t *buffer, int buf_len)
{
	return IOInterface::instance()->getDIO(io_info, buffer, buf_len);
}

U64 FstIoDevice::checkIO(const char *path, IOPortInfo* io_info)
{
	return IOInterface::instance()->checkIO(path, io_info);
}

U64 FstIoDevice::updateIOError()
{
	return IOInterface::instance()->updateIOError();
}




