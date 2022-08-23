
#include <string.h>
#include <iostream>
#include <unistd.h>
#include "fio_cmd.h"
#include "fio_device.h"
#include <stdio.h>

using namespace std;
using namespace hal_space;

FioDevice::FioDevice():BaseDevice(DEVICE_TYPE_FOC)
{
    //init(1);
}

FioDevice::~FioDevice()
{
    closeDevice(fio_device_);
}

bool FioDevice::init(bool is_real)
{
    is_real_ = is_real;
    fio_device_ = openDevice(FOC_DEVICE_PATH, GrindCmdRegsAddr, FOC_CH1_SIZE);
    if(fio_device_ == NULL)
    {
        return false;
    }
    else
    {
        fio_hw_ptr = (FioHw * )fio_device_->device_ptr;
        fio_hw_ptr->status.time_out_base = 0xFFFF;
        fio_hw_ptr->int_status.timeout = 0xFFFF;
        fio_hw_ptr->status.cmd_trans_valid = 0;
        return true;
    }
}


//默认大端存储, 字节序逆转后得到小端存储形式 
uint16_t u16_byte_reverse(uint16_t da)
{
	uint16_t t1=0,t2=0;
	t1 = (da&0x00FF)<<8;
	t2 = (da&0xFF00)>>8;
	return t1+t2;
}
uint32_t u32_byte_reverse(uint32_t da)
{
	uint32_t b1=0,b2=0,b3=0,b4=0;
	b1 = (da&0x000000FF)<<24;
	b2 = (da&0x0000FF00)<<8;
	b3 = (da&0x00FF0000)>>8;
	b4 = (da&0xFF000000)>>24;
	return b1+b2+b3+b4;
}


int FioDevice::FioSendCmdPack(uint32_t cmd, uint32_t val)
{
    int wait_cnt=0;
	while( fio_hw_ptr->status.cmd_trans_valid==1 && wait_cnt < 1000)
    {
        usleep(1000);
        wait_cnt++;
    }
    printf("send_wait->%d\n",wait_cnt);
    if(wait_cnt < 1000)
    {
        fio_hw_ptr->cmd_regs.status = 1;
        fio_hw_ptr->cmd_regs.tmcl_cmd = u16_byte_reverse((uint16_t)cmd);
        fio_hw_ptr->cmd_regs.motor = 0;
        fio_hw_ptr->cmd_regs.value = u32_byte_reverse(val);
        //printf("cmd=%x, value=%x\n", fio_hw_ptr->cmd_regs.tmcl_cmd,fio_hw_ptr->cmd_regs.value);
        return 0;
    }
    return 1;//err
}

int FioDevice::FioRecvReplyPack(uint32_t *pktid_status_cmd, uint32_t *val)
{
    int wait_cnt=0;
    uint32_t t_pktId;
    uint32_t t_opcode;
    uint32_t t_status;
    while (fio_hw_ptr->int_status.rx_interrupt == 0 && wait_cnt<1000)//没收到数据
    {
        usleep(1000);
        wait_cnt++;
    }
    /*
    printf("recv_wait->%d\n",wait_cnt);
    printf("cmd_reg=%llx, int_status=%llx \nint_status.rx_int=%llx, int_status.timeout=%llx\n",fio_hw_ptr->cmd_regs,fio_hw_ptr->int_status,\
        fio_hw_ptr->int_status.rx_interrupt,fio_hw_ptr->int_status.timeout);
    */
    fio_hw_ptr->int_status.rx_interrupt = 0;//重置标记为没收到数据
    if(wait_cnt < 1000)
    {
        t_pktId  = fio_hw_ptr->back_regs.packet_id<<16;
        t_status = fio_hw_ptr->back_regs.status<<8;
        t_opcode= fio_hw_ptr->back_regs.opcode;
        *pktid_status_cmd =  t_pktId + t_opcode + t_status;
        *val = u32_byte_reverse(fio_hw_ptr->back_regs.value);
        /*printf("back_reg=%llx, id=%llx, status=%llx, opcode=%llx, pktid_status_cmd=[%llx], value=%llx\n",fio_hw_ptr->back_regs,\
                t_pktId,t_status,t_opcode, *pktid_status_cmd,*val);
        */
        return 0;
    }
    return 1;
}



bool FioDevice::getIsReal()
{
    return is_real_;
}
void FioDevice::heartBeatBreak()
{
    is_real_ = false;
}


void FioDevice::FioHeartBeatLoopQuery()
{
    int res = 0;
    static int err_cnt=0;
    uint32_t rep_dat1, rep_dat2;
    if(is_real_)
    {
        FioSendCmdPack(38657, 0); //38657-->查询板卡状态-错误信息
        if(res == 0)
        {
            res  = FioRecvReplyPack(&rep_dat1,&rep_dat2);
            if(res==0)
            {
                err_cnt = 0;
                printf("FioHeartBeatLoopQuery  dat1=0x%x, dat2=0x%x\n", rep_dat1, rep_dat2);
            }
            else
            {
                printf("FioHeartBeatLoopQuery  FioRecvReplyPack error");
                err_cnt++;
                if(err_cnt > 100)
                {
                    heartBeatBreak();
                }
            }
        }
    }
}

ErrorCode FioDevice::updateStatus(void)
{

    return SUCCESS;
}






