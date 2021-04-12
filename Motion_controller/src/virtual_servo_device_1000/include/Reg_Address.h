/*
 *Reg_Address.h
 *
 *Created on: 2019年7月15日
 *Author: qichao.wang
 */

 #ifndef REG_ADDRESS_H_
 #define REG_ADDRESS_H_

 /*--------------------------------------fpga配置，包含电流环等--------------------------------*/
 #define FPGA_CON_BASE        0xC0000000
 #define FPGA_MODE_BASE       0xC0000008
 #define FPGA_CURRENT_OFFSET  0x100
 #define FPGA_PARA_OFFSET     0x04
 #define MAX_CURRENT_NUM      0x07
 #define FPGA_RAM_OFFSET      0x2000
 #define FPGA_AXIS_OFFSET    0x10000

 #define FPGA_AXIS_RAM_OFFSET 0x100
 /*--------------------------------------编码器读获取---------------------------------------*/
 /*轴1编码器数据*/
 #define ENCODE_READ_BASE     0xC0010044 //同步编码器采样
 /*编码器多圈信息*/
 #define ENCODE_RAP_BASE 0xC0010004

 #define ENACODE_TOTAL_BASE   0xC0010060
 /*速度差值*/
 #define SP_CNT_ERR           0xC001003C
 /*清编码器错误*/
 #define REST_FPGA          0xC001004C

 #define ENCODE_FILTER_REG    0xC0010050     //编码器滤波，周期设置使能。

 #define CURRENT_FILTER_CON      0xC0110004

 #define ANGLE_UNIT 13107
 /*-------------------------------------DA观测-------------------------------------------*/
 /*WATCH window*/
 #define WATCH_CON_REG1          0xC010007C
 #define IV_WATCHWINDOW       0xC0002044//3轴
 #define IU_WATCHWINDOW       0xC0002040//3轴
 #define IQ_WATCHWINDOW 0xC0002074
 #define ID_WATCHWINDOW 0xC1014020
 #define IU_SOU_WATCHWINDOW   0xC0002004

 #define ALPHA_BASE           0xC0002078
 #define BETA_BASE            0xC000207C

 #define DA_CFG_COMP          0xC0100004
 /*----------------------------------------测试功能---------------------------------------------*/
 #define DA_WATCH
 #undef DEBUG_MO
 #undef TEST_PERIOD

 /*---------------------------------------安全板地址读写------------------------------------------*/
 #define SAFE_BASE_ADD 0xC00A0000
 #define SAFE_READ_OFFSET 0x14
 #define SAFE_ERR_CODE    0x18
 #define SAFE_WRITE_OFFSET 0x8
 #define SAFE_CON_OFFSET 0x1C
 #define SAFETY_RESET_ADD 0xC00A0020
 #define SAFETY_ERR_CODE  0xC00A0048

 #define TIME_SCALE 5000  //时间离散因子
 #define TIME_FACTOR 0.0002

 /*--------------------------弱磁alpha,bltea-------------------------------------*/
 #define WATCH_DATA_DEFINE    0xC1014088
 #define ENC_ERR_CHECK        0xC0010030

 /*20190315*/
 #define PHASE_CURRENT_ERR    0xC000204C

 #define SPEED_FILTER_ADD     0xC0010054
 #define SPEED_FILTER_DATA    0xC0010058
 #define MUL_RAP_INIT_DATA    0xC001005C
 #define MUL_RAP_CAL_DATA     0xC0010060

 /*电机状态*/
 #define MOTOR_STATUS0xC00B0004
 #define FEED_DOG            0xC00B0060
 #define VD_WATCHWINDOW0xC00B0008

 #define DEAD_ZONE_COMPEN    0xC0000038
 /*-------------------------fpga dma硬件DMA初始化----------------------------------*/
 #define ADDR_BASE_ADC 0xC0110000
 #define ADDR_BASE_DMA 0xC00F0000
 #define ADDR_BASE_ACR 0xC0000000
 #define ADDR_BASE_ENC 0xC0010000
 /*硬DMA偏移值*/
 #define SOURCE_OFFSET   0x10000
 #define DES_OFFSET      0x2000
 /*硬DMA源址基址*/
 #define ADC_SOU_OFFSET   0x08
 #define ENC_SOU_OFFSET   0x44
 /*硬DMA连接电流环基址*/
 #define ADC_ACR_OFFSET   0x04
 #define ENC_ACR_OFFSET   0x0C
 /*---建立DMA链表寄存器常量-------*/
 #define INDEX_OFFSET 0x20
 #define LINK_ADD_OFFSET 0x800
 #define LINK_SOFT_OFFSET 0x400
 #define SOFT_INDEX_OFFSET 0x10

 #define ACR_FACTOR 256
 /*----dma的连续寄存器配置，目标，源等----*/
 #define SOU_REG_OFFSET 0x4
 #define DES_REG_OFFSET 0x8
 #define LINK_REG_OFFSET 0xc
 #define DMA_REG_OFFSET  0x8
 /*---------信号链表单个-----*/
 #define SIGNLE_CMD 0x20
 
 #endif /* REG_ADDRESS_H_ */

