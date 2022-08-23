#ifndef _FIO_DEVICE_H
#define _FIO_DEVICE_H

#define MMAP_BASE_ADDR 0xA00C0000
#define GrindCmdRegsAddr  MMAP_BASE_ADDR
#define GrindBkRegsOffset 0x008
#define GrindTxStatusOffset 0x028
#define GrindIntStatusOffset 0x030

#define TorqueSensorRegsAddr  (MMAP_BASE_ADDR+0x100)
#define TorqueSensorBkRegsOffset 0x130

#define FOC_CH1_SIZE       56   
#define FOC_CH2_SIZE       392       
#define FOC_DEVICE_PATH    "/dev/mem"

#define FOC_STATUS_OFFSET  0x28


#include "base_device.h"

//#pragma pack(1)
namespace hal_space
{

typedef struct{
    uint32_t status:8;
    uint32_t tmcl_cmd:16;
    uint32_t motor:8;
    uint32_t value;//小端模式
}FioCmdRegs;

typedef struct{
    uint32_t packet_id:16;
    uint32_t status:8;
    uint32_t opcode:8;
    uint32_t value;//小端模式
}FioBkRegs;

typedef	struct{
    uint32_t  err_pkt;//错误的包个数,包括crc校验与遗失
    uint32_t  all_pkt;//收到的所有包个数
}FioRxPktNum;//接收数据帧状态

typedef	struct{
    uint32_t  rx_err_bytes;//错误字节数
    uint32_t  rev;//保留
}FioRxErrorNum;//接收错误字节数

typedef	struct{
    uint32_t  rx_lost_bytes;//丢失字节数
    uint32_t  rev;//保留
}FioRxLostNum;//接收遗失字节数

typedef	struct{
    uint32_t  time_out_base;//接收超时时间,软件设定超时时间,读写
    uint32_t  time_out:16;//接收超时标识寄存器.   发送后开启超时计数,若TimeOutBase*10ns未收到返回包,该位为1只读
    uint32_t  cmd_trans_valid:16;//发送有效寄存器 1:正在发送 0:发送完毕 只读
}FioStatus;//状态寄存器

typedef	struct{
    uint64_t  timeout:1;
    uint64_t  rx_interrupt:1;//1为中断标识,收到数据包 读写
    uint64_t  rev:62;//预留
}FioIntStatus;

typedef	struct{
    uint64_t  timeout_enable:1;//1开启,0关闭, 默认0 读写
    uint64_t  rx_interrupt_enable:1;//1开启,0关闭, 默认0 读写
    uint64_t  rev:62;//预留
}FioIntEn;


typedef	struct {
    FioCmdRegs	    cmd_regs;//8字节命令寄存器
    FioBkRegs       back_regs;//返回数据寄存器,接收到的数据帧
    FioRxPktNum     rx_packet_num;//接收数据帧状态
    FioRxErrorNum 	rx_err_byte_num;//接收错误字节数
    FioRxLostNum    rx_lost_num; //接收遗失字节数
    FioStatus	    status; //第[0]位：超时标志，第[1]位：接收标志，对intStatus 某位写入0值，清中断
    FioIntStatus    int_status;
    FioIntEn 	    int_en;;//中断使能，第[0]位：超时中断使能， 1使能，超出timeOutBase时间为收到则中断；第[1]位：接收中断使能， 1使能，收到数据后产生中断
}FioHw;


/**
 * @brief FielManager can be used to read or write a text file.
 */
class FioDevice : public BaseDevice
{
public:
    /**
     * @brief Constructor of the class.
     */
    FioDevice();
    /**
     * @brief Destructor of the class. 
     */ 
    ~FioDevice();
    virtual bool init(bool is_real);
public:
    virtual ErrorCode updateStatus();
    int FioSendCmdPack(uint32_t cmd, uint32_t val);
    int FioRecvReplyPack(uint32_t *pktid_status_cmd, uint32_t *val);
    bool getIsReal();
    void heartBeatBreak();
    void FioHeartBeatLoopQuery();
private:
    Device_t *fio_device_;/**< Stores the information of the share memory.*/
    bool is_real_;            /**< True indicates operating on the real device while false means no checking device.*/
    FioHw * fio_hw_ptr;
};

}

#endif

