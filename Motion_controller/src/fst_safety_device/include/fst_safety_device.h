#ifndef FST_SAFETY_DEVICE_H
#define FST_SAFETY_DEVICE_H

#include "base_device.h"
#include "fst_safety_device_param.h"
#include "common_log.h"
#include <thread>
#include <mutex>

namespace fst_hal
{
class FstSafetyDevice : public BaseDevice
{
public:
    FstSafetyDevice(int address);
    ~FstSafetyDevice();

    virtual bool init();

    //various API to control or monitor...

    
private:
    FstSafetyDeviceParam param_;
    fst_log::Logger log_;
    int input[8];   //maybe 8 bytes?
    int output[8];  //maybe 8 bytes?

    std::thread update_thread_;
    std::mutex mutex_;  // data protection

    FstSafetyDevice();
    void updateThreadFunc();    // data exchange 
};

}

#endif

