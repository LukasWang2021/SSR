#ifndef VIRTUAL_AXIS_DEVICE_H
#define VIRTUAL_AXIS_DEVICE_H


#include "base_device.h"
#include "virtual_axis_device_param.h"
#include "common_log.h"

namespace fst_hal
{
// in current phase, we use servo status to replace axis status, although they are not quite the same.
typedef enum
{
    VIRTUAL_AXIS_DEVICE_INIT = 0,
    VIRTUAL_AXIS_DEVICE_DISABLE = 1,
    VIRTUAL_AXIS_DEVICE_ENABLE = 2,
}VirtualAxisDeviceStatus;

class VirtualAxisDevice : public BaseDevice
{
public:
    VirtualAxisDevice(int address);
    ~VirtualAxisDevice();

    virtual bool init();

    bool setEnable();
    bool setDisable();
    
    void setZeroPosition();    
    void setTargetPosition(double position);
    double getTargetPosition();  
    double getFeedBack();
    VirtualAxisDeviceStatus getStatus();

private:
    VirtualAxisDeviceParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    VirtualAxisDeviceStatus status_;
    double target_position_;
    double feedback_position_;    
};

}

#endif


