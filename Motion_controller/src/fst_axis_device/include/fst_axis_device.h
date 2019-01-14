#ifndef FST_AXIS_DEVICE_H
#define FST_AXIS_DEVICE_H


#include "base_device.h"
#include "fst_axis_device_param.h"
#include "common_log.h"
#include <vector>

namespace fst_hal
{
// in current phase, we use servo status to replace axis status, although they are not quite the same.
typedef enum
{
    FST_AXIS_DEVICE_INIT = 10,
    FST_AXIS_DEVICE_READY = 1,
    FST_AXIS_DEVICE_RUNNING = 2,
    FST_AXIS_DEVICE_ERROR = 3,
    FST_AXIS_DEVICE_WAIT_SERVOREADY = 4,
    FST_AXIS_DEVICE_WAIT_SERVODOWN = 5,
}FstAxisDeviceStatus;

typedef struct
{
    unsigned int time_stamp;
    double target_position;
    double target_velocity;
    double target_torque;
}FstAxisDeviceTx;

typedef struct
{
    double feedback_position;
    double feedback_velocity;
    double feedback_torque;
}FstAxisDeviceRx;

class FstAxisDevice : public BaseDevice
{
public:
    FstAxisDevice(int address);
    ~FstAxisDevice();

    virtual bool init();

    bool downloadParameters();

    bool setEnable();
    bool setDisable();
    bool setHalt();
    bool setQuickStop();
    bool resetFault();
    
    bool setTmpZeroPosition();
    bool setZeroPosition();
    
    bool isTargetBufferFull();
    bool pushbackTarget(unsigned int time_stamp, double position, double velocity, double torque);
    bool sendTarget();
    FstAxisDeviceTx getLastTarget();  
    FstAxisDeviceRx getFeedBack();
    FstAxisDeviceStatus getStatus();

    // the following two functions might not be used at the moment, this is done by core1 now
    int convertAppUnitToEncoderPulse(double data);
    double convertEncoderPulseToAppUnit(int encoder_pulse);

private:
    FstAxisDeviceParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    std::vector<FstAxisDeviceTx> tx_data_;
    FstAxisDeviceRx rx_data_;
    int rt_tx_buffer_size_;

    // database used
    // motor/servo/encoder/chain/app database, not realize them currently
    
};

}

#endif


