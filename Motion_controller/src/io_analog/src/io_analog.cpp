#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <string.h> 
#include <climits>
#include "io_analog.h"

using namespace log_space;
using namespace hal_space;
using namespace std;


IoAnalog::IoAnalog(void):
    BaseDevice(hal_space::DEVICE_TYPE_AIO),
    is_real_(false),
    pre_err_(SUCCESS)
{
}

IoAnalog::~IoAnalog(void)
{
    closeDevice(device_);
}

bool IoAnalog::init(bool is_real)
{
    is_real_ = is_real;
    if(!openDevice("/dev/fst_shmem", IO_AIO_BASE_ADDRESS, IO_AIO_TOTAL_BYTE_SIZE, device_))
    {
        return false;
    } 

    if (NULL == device_.device_ptr) return false;

    reg_ptr_ = (IOAnalogReg_t *)(device_.device_ptr + IO_AIO_REG_OFFSET);
    read_ptr_ = (IOAnalogMapReadReg_t *)(device_.device_ptr + IO_AIO_MAP_READ_OFFSET);

    //temp modify for FPGA. todo deleted.
    int32_t* temp_ptr = (int32_t *)(device_.device_ptr + 0x30);
    *temp_ptr = 0x19003C04;

    resetError();
    return true;
}

ErrorCode IoAnalog::writeAO(uint32_t board_id, uint32_t offset, int16_t value)
{
    if (offset >= IO_AIO_MAX_SIZE)
    {
        LogProducer::error("io_analog", "writeAO: invalid offset for AO[%d]", offset);
        return IO_ANALOG_INVALID_OFFSET;
    }
    IOAnalogCmdReg_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.aio_val = value;
    cmd.addr = offset;
    cmd.board_id = board_id;
    cmd.is_write = 1;
    cmd.op_start = 1;

    memcpy(&(reg_ptr_->cmd), &cmd, sizeof(cmd));
    return SUCCESS;
}

ErrorCode IoAnalog::readAIO(uint32_t board_id, uint32_t offset, int16_t &value)
{
    if (offset < IO_AIO_MAX_SIZE)
    {
        value = read_ptr_->aio[offset];
        return SUCCESS;
    }
    
    IOAnalogCmdReg_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.addr = offset;
    cmd.board_id = board_id;
    cmd.is_write = 0;
    cmd.op_start = 1;
    memcpy(&(reg_ptr_->cmd), &cmd, sizeof(cmd));

    while (reg_ptr_->cmd.op_start != 0)
    {
        usleep(200);
    }
    value = reg_ptr_->cmd.aio_val;
    return SUCCESS;
}

ErrorCode IoAnalog::updateStatus(void)
{
    if (!is_real_)
        return SUCCESS;
     
    ErrorCode current_err = SUCCESS;
    if (0 == reg_ptr_->state.connection && reg_ptr_->state.lost_connection)
        current_err = IO_ANALOG_DEV_LOST;    
    else if (reg_ptr_->state.line_error)
        current_err = IO_ANALOG_DEV_LINE_ERR;
    else if (reg_ptr_->state.crc_error)
        current_err = IO_ANALOG_DEV_CRC_ERR;
    else if (reg_ptr_->state.bad_frame_error)
        current_err = IO_ANALOG_DEV_FRAME_ERR;
    
    if (pre_err_ != current_err)
    {
        pre_err_ = current_err;
        if (current_err != SUCCESS)
        {
            LogProducer::error("io_analog", "updateStatus: debug analog io state: 0x%x", reg_ptr_->state);
        }
        return current_err;
    }
    return SUCCESS;
}

void IoAnalog::resetError(void)
{
    int reset_val = 0xFFFF;
    memcpy(&(reg_ptr_->state), &reset_val, sizeof(reset_val));
}

bool IoAnalog::openDevice(std::string device_path, uint32_t base_address, size_t byte_size, Device_t& device)
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

void IoAnalog::closeDevice(Device_t& device)
{
    if(device.device_ptr != NULL)
    {
        munmap(device.device_ptr, device.byte_size);
        device.device_ptr = NULL;
        device.base_address = 0;
        device.byte_size = 0;
    }
}


