#ifndef FST_IO_DEVICE_H
#define FST_IO_DEVICE_H

#include "base_device.h"
#include "fst_io_device_param.h"
#include "common_log.h"
#include <vector>
#include <thread>
#include <mutex>

namespace fst_hal
{
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
private:
    FstIoDeviceParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    std::vector<uint8_t> input_;
    std::vector<uint8_t> output_;
    std::thread update_thread_;
    std::mutex mutex_;  // data protection

    FstIoDevice();
    void updateThreadFunc();    // data exchange 
};

}

#endif

