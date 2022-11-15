
#include <string.h>
#include <iostream>
#include <unistd.h>
#include "fio_cmd.h"
#include "fio_device.h"
#include <stdio.h>
#include "log_manager_producer.h"
#include "common_file_path.h"

using namespace std;
using namespace hal_space;
using namespace log_space;
using namespace base_space;

FioDevice::FioDevice():
    BaseDevice(DEVICE_TYPE_FIO)
{

}

FioDevice::~FioDevice()
{
    closeDevice(fio_device_);
}

bool FioDevice::init(bool is_real)
{
    is_real_ = is_real;

    if(is_real == false)
        return true;

    fio_device_ = openDevice(FIO_DEVICE_PATH, GrindCmdRegsAddr, FIO_CH1_SIZE);
    if(fio_device_ == NULL)
    {
        LogProducer::error("FioDevice", "open fio device failed");
        return false;
    }
    YamlHelp yaml_help;
    int32_t serial_timeout = 0;
    if(!yaml_help.loadParamFile(string(COMPONENT_PARAM_FILE_DIR) + "fio_config.yaml")
    || !yaml_help.getParam("serial_time_out", serial_timeout))
    {
        return false;
    }

    fio_hw_ptr_ = (FioHw * )fio_device_->device_ptr;
    // hardware unit is 10ns
    fio_hw_ptr_->status.time_out_base = (uint32_t)serial_timeout * 100000;
    memset(&(fio_hw_ptr_->int_status), 0, sizeof(FioIntStatus));

    return true;

}

uint16_t u16_byte_reverse(uint16_t da)
{
	uint16_t t1=0, t2=0;
	t1 = (da & 0x00FF) << 8;
	t2 = (da & 0xFF00) >> 8;
	return t1+t2;
}

uint32_t u32_byte_reverse(uint32_t da)
{
	uint32_t b1=0, b2=0, b3=0, b4=0;
	b1 = (da & 0x000000FF) << 24;
	b2 = (da & 0x0000FF00) << 8;
	b3 = (da & 0x00FF0000) >> 8;
	b4 = (da & 0xFF000000) >> 24;
	return b1+b2+b3+b4;
}

ErrorCode FioDevice::rplResult(uint32_t status)
{
/*
    Status              	Status对应值          说明
    REPLY_OK                    100             返回正确
    REPLY_CMD_LOADED    	    101             正在处理命令
    REPLY_CHKERR            	1	            checksum错误
    REPLY_INVALID_CMD           2	            错误命令
    REPLY_INVALID_TYPE          3	            type值错误
    REPLY_INVALID_VALUE         4	            value值错误
    REPLY_EEPROM_LOCKED	        5	            eeprom锁定
    REPLY_CMD_NOT_AVAILABLE	    6	            命令不能使用
    REPLY_CMD_LOAD_ERROR	    7	            命令执行出错
    REPLY_WRITE_PROTECTED   	8	            写保护
    REPLY_MAX_EXCEEDED       	9	            最大执行
    REPLY_DOWNLOAD_NOT_POSSIBLE	10              不能执行
    REPLY_CHIP_READ_FAILED  	11          	芯片读取错误
    REPLY_DELAYED	            128	            延迟
*/
    switch ((status & 0xFF00) >> 8)
    {
    case 100: return SUCCESS;
    case 101: return FIO_DEVICE_BUSY;
    case 1:   return FIO_CMD_CRC_ERR;
    case 2:   return FIO_CMD_INVALID;
    case 3:   return FIO_CMD_TYPE_ERR;
    case 4:   return FIO_CMD_VALUE_ERR;
    case 5:   return FIO_EEPROM_LOCKED;
    case 6:   return FIO_CMD_NOT_AVAILABLE;
    case 7:   return FIO_CMD_EXEC_FAILED;
    case 8:   return FIO_REG_WRITE_PROTECTED;
    case 9:   return FIO_MAX_EXCEEDED;
    case 10:  return FIO_DOWNLOAD_NOT_POSSIBLE;
    case 11:  return FIO_CHIP_READ_FAILED;
    case 128: return FIO_GET_ID_FAILED;
    default:  break;
    }
    return FIO_UNKNOWN;
}

ErrorCode FioDevice::sendCmdRcvRpl(uint32_t cmd, uint32_t cmd_val, uint32_t *rpl_val)
{
    if(!is_real_)
    {
        LogProducer::warn("FioDevice", "fio device is not exist");
        return SUCCESS;
    }

    fio_mutex_.lock();

    int32_t retry_cnt = 0; // max try for 3 times
    uint32_t rpl_status = 0;
    ErrorCode err_ret = SUCCESS;

    do
    {
        err_ret = SUCCESS;
        if(!fioSendCmdPack(cmd, cmd_val))
        {
            err_ret = FIO_DEVICE_BUSY;
            goto RETRY;
        }

        if(!fioRecvRplPack(&rpl_status, rpl_val))
        {
            err_ret = FIO_DEVICE_NO_RPL;
        }
RETRY:
        if(retry_cnt > 0)
        {
            LogProducer::warn("FioDevice", "fio device cmd[0x%X] tried for %d time(s)", cmd, retry_cnt);
        }
    } while((err_ret != SUCCESS || rplResult(rpl_status) == FIO_CMD_CRC_ERR) && retry_cnt++ < 3);

    fio_mutex_.unlock();

    return err_ret ? err_ret : rplResult(rpl_status);
}

bool FioDevice::fioSendCmdPack(uint32_t cmd, uint32_t val)
{
    int wait_cnt = 0;

    while(wait_cnt++ < 100)
    {
        if(fio_hw_ptr_->status.cmd_trans_valid == 1)
        {
            usleep(1000);// if cmd channel is busy wait about 100ms
            continue;
        }
        fio_hw_ptr_->cmd_regs.status = 1;
        fio_hw_ptr_->cmd_regs.tmcl_cmd = u16_byte_reverse((uint16_t)cmd);
        fio_hw_ptr_->cmd_regs.motor = 0;
        fio_hw_ptr_->cmd_regs.value = u32_byte_reverse(val);
        //printf("cmd=%x, value=%x\n", fio_hw_ptr_->cmd_regs.tmcl_cmd,fio_hw_ptr_->cmd_regs.value);
        return true;
    }

    LogProducer::error("FioDevice", "fio device cmd channel is busy, send cmd failed");
    return false;
}

bool FioDevice::fioRecvRplPack(uint32_t *status, uint32_t *val)
{
    int wait_cnt = 0;
    uint32_t t_pktId = 0;
    uint32_t t_opcode = 0;
    uint32_t t_status = 0;

    while(wait_cnt++ < 500) // ensure the cmd excuted max about 500ms
    {
        if(fio_hw_ptr_->int_status.rx_interrupt == 0)
        {
            usleep(1000);
            continue;
        }
        t_pktId  = fio_hw_ptr_->back_regs.packet_id << 16;
        t_status = fio_hw_ptr_->back_regs.status << 8;
        t_opcode= fio_hw_ptr_->back_regs.opcode;
        *status =  t_pktId + t_opcode + t_status;
        *val = u32_byte_reverse(fio_hw_ptr_->back_regs.value);
    
        // LogProducer::error("FioDevice", "back_reg=%llx, id=%llx, status=%llx, opcode=%llx, pktid_status_cmd=[%llx], value=%llx", 
        // fio_hw_ptr_->back_regs, t_pktId, t_status, t_opcode, *status,*val);

        fio_hw_ptr_->int_status.rx_interrupt = 0;
        fio_hw_ptr_->int_status.timeout = 0;
        return true;
    }
    LogProducer::error("FioDevice", "fio device reply channel is empty, recieve data timeout");
    return false;
}

ErrorCode FioDevice::updateStatus(void)
{
    if(!is_real_) return SUCCESS;

    ErrorCode err = sendCmdRcvRpl(READ_ERROR_STATE, 0, &(fio_status_.all));
     // mcu handle recieve for 700us and others options all may coast about 1ms
    if(err) return err; else usleep(1000);

    err = sendCmdRcvRpl(GET_ACTUAL_SPEED, 0, &(fio_topic_.grind_speed));

    return err;
}






