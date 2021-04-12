#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <string.h> 
#include <climits>
#include <map>
#include "io_1000.h"

using namespace log_space;
using namespace hal_space;
using namespace std;

static std::map<uint32_t, uint32_t> g_di_map = {
    {1, 3}, {2, 1}, {3, 7}, {4, 5}, {5, 11}, {6, 9}, {7, 15}, {8, 13},
    {9, 2}, {10 , 0}, {11, 6}, {12, 4}, {13, 10}, {14, 8}, {15, 14}, {16, 12},
    {17, 19}, {18, 17}, {19, 23}, {20, 21}, {21, 27}, {22, 25}, {23, 31}, {24, 29},
    {25, 18}, {26, 16}, {27, 22}, {28, 20}, {29, 26}, {30, 24}, {31, 30}, {32, 28},
    {33, 35}, {34, 33}, {35, 39}, {36, 37}, {37, 43}, {38, 41}, {39, 47}, {40, 45},
    {41, 34}, {42, 32}, {43, 38}, {44, 36}, {45, 42}, {46, 40}, {47, 46}, {48, 44},
    {49, 55}, {50, 54}, {51, 53}, {52, 52}, {53, 51}, {54, 50}, {55, 48}, {56, 49}
};
static std::map<uint32_t, uint32_t> g_do_map = {
    {1, 32}, {2, 33}, {3, 34}, {4, 35}, {5, 36}, {6, 37}, {7, 38}, {8, 39},
    {9, 24}, {10 , 25}, {11, 26}, {12, 27}, {13, 28}, {14, 29}, {15, 30}, {16, 31},
    {17, 9}, {18, 11}, {19, 13}, {20, 15}, {21, 0}, {22, 2}, {23, 4}, {24, 6},
    {25, 17}, {26, 19}, {27, 1}, {28, 3}, {29, 5}, {30, 7}, {31, 21}, {32, 23},
    {33, 8}, {34, 10}, {35, 12}, {36, 14}, {37, 16}, {38, 18}, {39, 20}, {40, 22}
};

Io1000::Io1000(void):
    BaseDevice(hal_space::DEVICE_TYPE_DIO),
    is_real_(false),
    pre_err_io_(SUCCESS),
    pre_err_stepper_(SUCCESS)
{
}

Io1000::~Io1000(void)
{
    closeDevice(device_);
    closeDevice(stepper_dev_);
}

bool Io1000::init(bool is_real)
{
    is_real_ = is_real;
    if(!openDevice("/dev/fst_shmem", IO_BASE_ADDRESS, IO_TOTAL_BYTE_SIZE, device_))
    {
        return false;
    } 
    if (!openDevice("/dev/fst_shmem", STEPPER_DI_BASE_ADDRESS, IO_TOTAL_BYTE_SIZE, stepper_dev_))
    {
        return false;
    }

    //io_board
    io_ptr_ = (IOReg_t *)device_.device_ptr;
    io_status_ptr_ = (uint32_t *)(device_.device_ptr + IO_STATUS_OFFSET);

    //stepper io
    stepper_ptr_ = (uint32_t *)(stepper_dev_.device_ptr + STEPPER_DI_OFFSET);
    stepper_status_ptr_ = (uint32_t *)(stepper_dev_.device_ptr + IO_STATUS_OFFSET);

    return true;
}

ErrorCode Io1000::writeDoBit(uint32_t offset, uint8_t value)
{ 
    if (offset > IO_DO_MAX_SIZE || 0 == offset) 
    {
        LogProducer::error("io_1000", "writeDoBit: invalid offset for DO[%d]", offset);
        return IO_DIGITAL_INVALID_OFFSET;
    }

    // find the bit offset in IO register according the user offset.
    map<uint32_t, uint32_t>::iterator iter = g_do_map.find(offset);
    uint32_t shift = 0;
    if (iter != g_do_map.end())
        shift = iter->second;
    else
        return IO_DIGITAL_INVALID_OFFSET;

    uint32_t op_bit = 0x01;
    uint32_t section_bit = CHAR_BIT * sizeof(uint32_t);  
    if (value == 0)
    {
        if (shift < section_bit)
        {
            io_ptr_->do_reset = op_bit << shift;
        }
        else
        {
            io_ptr_->do2_reset = op_bit << (shift - section_bit);
        }
    }
    else
    {
        if (shift < section_bit)
        {
            io_ptr_->do_set = op_bit << shift;
        }
        else
        {
            io_ptr_->do2_set = op_bit << (shift - section_bit);
        }
    }

    return SUCCESS;
}

ErrorCode Io1000::readDiBit(uint32_t offset, uint8_t &value)
{
    if (offset > IO_DI_MAX_SIZE || 0 == offset)
    {
        LogProducer::error("io_1000", "readDiBit: invalid offset for DI[%d]", offset);
        return IO_DIGITAL_INVALID_OFFSET;
    }
    else if (offset >= STEPPER_DI_SEQ_START)
    {
        return readStepperDiBit(offset - STEPPER_DI_SEQ_START, value);
    }

    // find the bit offset in IO register according the user offset.
    map<uint32_t, uint32_t>::iterator iter = g_di_map.find(offset);
    uint32_t shift = 0;
    if (iter != g_di_map.end())
        shift = iter->second;
    else
        return IO_DIGITAL_INVALID_OFFSET;

    uint32_t op_bit = 0x1;
    uint32_t section_bit = CHAR_BIT * sizeof(uint32_t);
    if (shift < section_bit)
    {
        value = (io_ptr_->di_read >> shift) & op_bit;
    }
    else
    {
        value = (io_ptr_->di2_read >> (shift - section_bit)) & op_bit;
    }

    return SUCCESS;
}

ErrorCode Io1000::readDoBit(uint32_t offset, uint8_t &value)
{
    if (offset > IO_DO_MAX_SIZE || 0 == offset)
    {
        LogProducer::error("io_1000", "readDoBit: invalid offset for DO[%d]", offset);
        return IO_DIGITAL_INVALID_OFFSET;
    }

    // find the bit offset in IO register according the user offset.
    map<uint32_t, uint32_t>::iterator iter = g_do_map.find(offset);
    uint32_t shift = 0;
    if (iter != g_do_map.end())
        shift = iter->second;
    else
        return IO_DIGITAL_INVALID_OFFSET;
    
    uint32_t op_bit = 0x1;
    uint32_t section_bit = CHAR_BIT * sizeof(uint32_t);
    if (shift < section_bit)
    {
        value = (io_ptr_->do_read >> shift) & op_bit;
    }
    else
    {
        value = (io_ptr_->do2_read >> (shift - section_bit)) & op_bit;
    }

    return SUCCESS;
}

ErrorCode Io1000::readDiAll(uint32_t &value_lower, uint32_t &value_upper)
{
    value_lower = io_ptr_->di_read;
    value_upper = io_ptr_->di2_read;
    return SUCCESS;
}

ErrorCode Io1000::readDoAll(uint32_t &value_lower, uint32_t &value_upper)
{
    value_lower = io_ptr_->do_read;
    value_upper = io_ptr_->do2_read;
    return SUCCESS;
}

ErrorCode Io1000::readStepperDiBit(uint32_t offset, uint8_t &value)
{
    value = ((*stepper_ptr_) >> offset) & 0x1;
    return SUCCESS;
}

ErrorCode Io1000::updateStatus(void)
{
    if (!is_real_)
        return SUCCESS;
    //io board
    ErrorCode current_err = SUCCESS;
    Io1000State_u status;
    status.all = *io_status_ptr_;
    if (status.bit.comm_err)
    {
        current_err = IO_DIGITAL_DEV_DISABLE;
    }
    if (pre_err_io_ != current_err)
    {
        pre_err_io_ = current_err;
        if (current_err != SUCCESS)
        {
            LogProducer::error("io_1000", "updateStatus: debug io_board state: 0x%x", status.all);
        }
        return current_err;
    }

    //stepper board
    current_err = SUCCESS;
    status.all = *stepper_status_ptr_;
    if (status.bit.comm_err)
    {
        current_err = IO_STEPPER_DIGITAL_DEV_DISABLE;
    }
    if (pre_err_stepper_ != current_err)
    {
        pre_err_stepper_ = current_err;
        if (current_err != SUCCESS)
        {
            LogProducer::error("io_1000", "updateStatus: debug stepper_board state: 0x%x", status.all);
        }
        return current_err;
    }

    return SUCCESS;
}

bool Io1000::openDevice(std::string device_path, uint32_t base_address, size_t byte_size, Device_t& device)
{
    int device_fd = open(device_path.c_str(), O_RDWR);
    device.device_ptr = (char *)mmap(NULL, byte_size, PROT_READ|PROT_WRITE, MAP_SHARED, device_fd, base_address);
    if (device.device_ptr == MAP_FAILED) 
    {
        close(device_fd);
        device.device_ptr = NULL;
        device.base_address = 0;
        device.byte_size = 0;        
        return false;
    }
    else
    {
        close(device_fd);
        device.base_address = base_address;
        device.byte_size = byte_size;
        return true;
    }
}

void Io1000::closeDevice(Device_t& device)
{
    if(device.device_ptr != NULL)
    {
        munmap(device.device_ptr, device.byte_size);
        device.device_ptr = NULL;
        device.base_address = 0;
        device.byte_size = 0;
    }
}


