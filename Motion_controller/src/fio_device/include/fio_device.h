#ifndef _FIO_DEVICE_H
#define _FIO_DEVICE_H

#define MMAP_BASE_ADDR 0xA00C0000
#define GrindCmdRegsAddr  MMAP_BASE_ADDR
#define GrindBkRegsOffset 0x008
#define GrindTxStatusOffset 0x028
#define GrindIntStatusOffset 0x030

#define TorqueSensorRegsAddr  (MMAP_BASE_ADDR+0x100)
#define TorqueSensorBkRegsOffset 0x130

#define FIO_CH1_SIZE       56   
#define FIO_CH2_SIZE       392       
#define FIO_DEVICE_PATH    "/dev/mem"

#define FIO_STATUS_OFFSET  0x28


#include "base_device.h"
#include "common_error_code.h"
#include <mutex>

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

/*
    读取错误状态，type=1。返回值VALUE
    bit 0 MCU出错.LDO 3.3v 和LDO 5V 
    bit 1 TMC4671板子出错
    bit 2 保留
    bit 3 保留
    bit 4 保留
    bit 5 保留
    bit 6 MCU电压低，未实现
    bit 7 TMC4671电压低 ，未实现
    bit 8 DRV8302电压低，未实现
    bit 9 MCU电压高，未实现
    bit 10 TMC4671电压高
    bit 11 DRV8302电压高
    bit 12 TMC4671芯片报错
    bit 13 DRV8302报错
    bit 14 ADS8688报错
    bit 15  0 电机停止  1 电机运行。
    bit 16  0电机无错误，1 电机出错
    bit 17  帧间隔时间超时
    bit 18  0 踏板未踩下 1 踏板踩下 
    其他：保留
*/
typedef struct
{
    uint32_t mcu_ldo_err:1; // adc power supply
    uint32_t tmc_err:1;     // TMC4671(grind controller)
    uint32_t reserved_1:4;  // reserved
    uint32_t mcu_low_vol:1; // mcu low voltage
    uint32_t tmc_low_vol:1; // TMC4671 low voltage
    uint32_t drv_low_vol:1; // DRV8302(MOS driver) low voltage
    uint32_t mcu_high_vol:1; // mcu high voltage
    uint32_t tmc_high_vol:1; // TMC4671 high voltage
    uint32_t drv_high_vol:1; // DRV8302 high voltage
    uint32_t tmc_ret_err:1;  // MCU cmd to TMC return error
    uint32_t drv_ret_err:1;  // DRV8302 return error
    uint32_t ads_ret_err:1;  // ADS8688(force sensor ADC)
    uint32_t grind_state:1;  // the grind motor state, 0:stopped, 1:running
    uint32_t grind_error:1;  // the grind motor error or not, 0:ok, 1:error
    uint32_t heartbeat_loss:1;  // the master side always ask about the salve state every 100ms 
    // if slave side do not recive the cmd more than 300ms it will report error
    uint32_t footboard_state:1;  // the master side always ask about the salve state every 100ms 
    uint32_t reserved_2:13;  // reserved
}FioStatus_b;

typedef union
{
	uint32_t all;                      /**< Operated attribute by double word.*/
	FioStatus_b bit;                   /**< Operated attribute by bit.*/
}FioStatus_u;

typedef struct
{
	uint32_t grind_speed; // grinder's speed
    // ...
}FioTopicVal_t;


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
    bool isReal() { return is_real_; }
    // void heartBeatBreak();
    // void FioHeartBeatLoopQuery();
    ErrorCode sendCmdRcvRpl(uint32_t cmd, uint32_t cmd_val, uint32_t *rpl_val);
    FioStatus_u getStatus(void) { return fio_status_; }
    FioTopicVal_t getTopicVal(void) { return fio_topic_; }

private:
    bool fioSendCmdPack(uint32_t cmd, uint32_t val);
    bool fioRecvRplPack(uint32_t *status, uint32_t *val);
    ErrorCode rplResult(uint32_t status);

private:
    Device_t *fio_device_;    /**< Stores the information of the share memory.*/
    bool is_real_;            /**< True indicates operating on the real device while false means no checking device.*/
    FioHw * fio_hw_ptr_;
    FioStatus_u fio_status_;
    FioTopicVal_t fio_topic_;
    std::mutex fio_mutex_;
};

}

#endif

