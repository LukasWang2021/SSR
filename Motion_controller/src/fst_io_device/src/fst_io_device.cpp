#include "fst_io_device.h"


using namespace fst_hal;

FstIoDevice::FstIoDevice(int address):
    BaseDevice(address, fst_hal::DEVICE_TYPE_FST_IO)
{
   
}

FstIoDevice::~FstIoDevice()
{

}

bool FstIoDevice::init()
{
    return true;
}

void FstIoDevice::getDeviceInfo()
{
    
}

int FstIoDevice::getInputByteSize()
{
    return 0;
}

int FstIoDevice::getOutputByteSize()
{
    return 0;
}

std::vector<uint8_t> FstIoDevice::getAllInput()
{
    return input_;
}

std::vector<uint8_t> FstIoDevice::getAllOutput()
{
    return output_;
}

bool FstIoDevice::getInputByByte(int byte_offset, uint8_t& data)
{
    return true;
}

bool FstIoDevice::getOutputByByte(int byte_offset, uint8_t& data)
{
    return true;
}

bool FstIoDevice::getInputByBit(int bit_offset, bool data)
{
    return true;
}

bool FstIoDevice::getOutputByBit(int bit_offset, bool data)
{
    return true;
}

bool FstIoDevice::setAllInput(std::vector<uint8_t>& input)
{
    return true;
}

bool FstIoDevice::setAllOutput(std::vector<uint8_t>& output)
{
    return true;
}

bool FstIoDevice::setInputByByte(int byte_offset, uint8_t data)
{
    return true;
}

bool FstIoDevice::setOutputByByte(int byte_offset, uint8_t data)
{
    return true;
}

bool FstIoDevice::setInputByBit(int bit_offset, uint8_t data)
{
    return true;
}

bool FstIoDevice::setOutputByBit(int bit_offset, uint8_t data)
{
    return true;
}

FstIoDevice::FstIoDevice():
    BaseDevice(0, fst_hal::DEVICE_TYPE_INVALID)
{

}

void FstIoDevice::updateThreadFunc()
{

}

