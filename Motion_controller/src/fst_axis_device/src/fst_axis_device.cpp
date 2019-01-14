#include "fst_axis_device.h"


using namespace fst_hal;


FstAxisDevice::FstAxisDevice(int address):
    BaseDevice(address, fst_hal::DEVICE_TYPE_FST_AXIS),
    log_ptr_(NULL),
    param_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new FstAxisDeviceParam();
}

FstAxisDevice::~FstAxisDevice()
{

}

bool FstAxisDevice::init()
{
    return true;
}

bool FstAxisDevice::downloadParameters()
{
    return true;
}

bool FstAxisDevice::setEnable()
{
    return true;
}

bool FstAxisDevice::setDisable()
{
    return true;
}

bool FstAxisDevice::setHalt()
{
    return true;
}

bool FstAxisDevice::setQuickStop()
{
    return true;
}

bool FstAxisDevice::resetFault()
{
    return true;
}

bool FstAxisDevice::setTmpZeroPosition()
{
    return true;
}

bool FstAxisDevice::setZeroPosition()
{
    return true;
}

bool FstAxisDevice::isTargetBufferFull()
{
    return true;
}

bool FstAxisDevice::pushbackTarget(unsigned int time_stamp, double position, double velocity, double torque)
{
    return true;
}

bool FstAxisDevice::sendTarget()
{
    return true;
}

FstAxisDeviceTx FstAxisDevice::getLastTarget()
{
    FstAxisDeviceTx tx;
    return tx;
}

FstAxisDeviceRx FstAxisDevice::getFeedBack()
{
    FstAxisDeviceRx rx;
    return rx;
}

FstAxisDeviceStatus FstAxisDevice::getStatus()
{
    return FST_AXIS_DEVICE_ERROR;
}

int FstAxisDevice::convertAppUnitToEncoderPulse(double data)
{
    return 0;
}

double FstAxisDevice::convertEncoderPulseToAppUnit(int encoder_pulse)
{
    return 0;
}


