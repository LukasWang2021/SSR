#include "interpreter_device.h"
using namespace log_space;

hal_space::BaseDevice *io_dev;
hal_space::Io1000*  io1000_dev;
bool InterpDevice_Init(hal_space::BaseDevice *dev)
{
    io_dev =  dev; 
    io1000_dev = (hal_space::Io1000*)io_dev;
    return true;
}
/*
函数功能: 读取引脚电平状态
参数: 
    offset--序号  范围:[0~65535]
    value--读值指针
返回值:错误码
*/
ErrorCode InterpDevice_GetDIBit(uint32_t offset, uint8_t &value)
{
    ErrorCode ret = io1000_dev->readDiBit(offset,value);
    if(ret != 0)
    {
        LogProducer::error("interpreter","InterpDevice_GetDIBit fail. ");
    }
    return ret;
}
ErrorCode InterpDevice_GetDOBit(uint32_t offset, uint8_t &value)
{
    ErrorCode ret = io1000_dev->readDoBit(offset,value);
    if(ret != 0)
    {
        LogProducer::error("interpreter","InterpDevice_GetDOBit fail. ");
    }
    return ret;
}

ErrorCode InterpDevice_SetDOBit(uint32_t offset, uint8_t value)
{
    ErrorCode ret = io1000_dev->writeDoBit(offset,value);
    if(ret != 0)
    {
        LogProducer::error("interpreter","InterpDevice_SetDOBit fail. ");
    }
    return ret;
}

ErrorCode InterpDevice_GetForceValue(uint32_t id, double value[6])
{
    return 0;
}

