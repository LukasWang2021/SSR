/*
 *Servo_Fpga.h
 *
 *Created on: 2019年7月15日
 *Author: qichao.wang
 */

 #ifndef SERVO_FPGA_H_
 #define SERVO_FPGA_H_

 #include "Servo_General.h"
 #include "Reg_Address.h"

 #ifdef V3_VERSION

 #define CORE_LIGNT_CON_BASE 0xff709000 //BANK B
 #define CORE_LIGHT_DAT_BASE 0xff709004
 #define LED_CONFIG 0x8000000
 #define LED_MASK 0x7ffffff
 #define LED_SHIFT 27

 #else

 #define CORE_LIGNT_CON_BASE 0xff70a000 //BANK C
 #define CORE_LIGHT_DAT_BASE 0xff70a004
 #define LED_CONFIG 0x01
 #define LED_MASK 0xfffffffe
 #define LED_SHIFT 0x0

 #endif

 /*读地址数据*/
 #define READ_HPS(addr,data) {data = *(volatile unsigned int *)(addr);}
 /*写地址数据*/
 #define WRITE_HPS(addr,data) {*(volatile unsigned int *)(addr) = data;}

 #define AXIS_REG_CFG(reg_info,base_add,axis_num) {reg_info = base_add + axis_num*FPGA_AXIS_OFFSET;}

 #define AXIS_RAM_CFG(reg_info,axis_num,var_id) {reg_info = FPGA_CON_BASE \
                                                +FPGA_RAM_OFFSET        \
                                                +(axis_num*FPGA_CURRENT_OFFSET) \
                                                +(var_id*FPGA_PARA_OFFSET);}

namespace virtual_servo_device{

 typedef struct
 {
    uint32_t enc_reg;
    int32_t mult_rap_reg;
    int32_t vel_reg;
    int32_t encode_err_ram;
    int32_t reset_fpga_ram;
    int32_t iq_ram;
    int32_t id_ram;
    int32_t iq_dump_ram;
    int32_t cur_sample_ram;
    int32_t cur_ki_ram;
    int32_t cur_kp_ram;
    int32_t motor_angle_ram;
    int32_t polar_angle_ram;
    int32_t polar_nums_ram;
    int32_t iq_inter_ram;
    int32_t id_inter_ram;
    int32_t dead_comp_ram;
    int32_t encode_filter_reg;
    int32_t current_filter_reg;
    int32_t iq_fback_ram;
    int32_t vq_out;
    int32_t vd_out;
    uint32_t current_loop_sa_reg;


 }SERVO_FPGA_REG_CFG_STRUCT;
 /*寄存器参数枚举*/
 typedef enum
 {
    ENC_REG,
    MUL_RAMP_REG,
    VEL_REG,
    ENCODE_ERR_REG,
    RESET_FPGA_REG,
    IQ_RAM,
    ID_RAM,
    IQ_DUMP_RAM,
    CUR_SAMPLE_RAM,
    CUR_KI_RAM,
    CUR_KP_RAM,
    MOTOR_ANGLE_RAM,
    POLAR_ALIGN_RAM,
    POLAR_NUM_RAM,
    IQ_INTER_RAM,
    ID_INTER_RAM,
    DEAD_COMP_REG,
    ENC_FILTER_REG,
    CURRENT_FILTER_REG,
    IQ_FBACK_RAM,
    VQ_FBACK_RAM,
    VD_FBACK_RAM,
    PHA_SATURE_RAM,
    FPGA_CFG_MAX = PHA_SATURE_RAM + 1


 }SERVO_FPGA_REG_CFG_ENUM;

 /*RAM参数枚举*/
 typedef enum
 {
    PHASE_CURR_OFFSET = 0X0,
    CURR_SAMPLE,
    CURR_FACTOR,
    ENCODE_SAMP,
    STUDY_THETA,
    POLAR_ANG_COM,
    POLAR_NUMS,
    IQ_CURRENT,
    ID_CURRENT,
    CURR_LOOP_KP,
    CURR_LOOP_KI,
    CURR_LOOP_KD,
    CURR_OUT_LIMIT,
    CURR_DECOUP,
    VOL_OUT_VQ,
    VOL_OUT_VD,
    IU_SOURCE,
    IV_SOURCE,
    THEAT_SOURCE,
    RESEVED_VAR,
    PWM_AB_DUTY,
    PWM_C_DUTY,
    PID_IQ_INT,
    PID_ID_INT,
    VQ_OUT_IPARK,
    VD_OUT_IPARK,
    CURR_ALPHA,
    CURR_BELTA,
    FEEDBAK_ID,
    FEEDBAK_IQ,
    FEEDBAK_VQ,
    FEEDBAK_VD,
    CURRENT_LOOP_SAT,
    FPGA_RAM_MAX = CURRENT_LOOP_SAT + 1


 }SERVO_FPGA_RAM_ITEM_ENUM;

 typedef struct
 {
    uint32_t sour_data;
    uint32_t rap_count;


 }SERVO_FPGA_SOURCE_ENCODE_STRUCT;

 typedef struct
 {
    int64_t encode_total;


 }SERVO_FPGA_ENCODE_BACK_STRUCT;


 typedef struct
 {
    int32_t vel_act;

 }SERVO_FPGA_VEL_BACK_STRUCT;

 typedef struct
 {
    int32_t iq_out;
    int32_t id_out;


 }SERVO_FPGA_IQID_STRUCT;

 typedef enum
 {
    FPGA_CLEAR,
    FPGA_RESET


 }SERVO_FPGA_RESET_CMD_ENUM;

 void Servo_Fpga_Init(void);

 void Servo_Fpga_Get_MeaPtr(SERVO_FPGA_SOURCE_ENCODE_STRUCT **sn_data,SERVO_AXIS_ENUM axis_id);

 void Servo_Fpga_Get_IqdPtr(SERVO_FPGA_IQID_STRUCT **current_data,SERVO_AXIS_ENUM axis_id);

 void Servo_Fpga_p_WriteFPGA(SERVO_AXIS_ENUM axis_id);

 void Servo_Fpga_p_ReadFPGA(SERVO_AXIS_ENUM axis_id);

 void Servo_Fpga_s_ResetFpga(SERVO_AXIS_ENUM axis_id);

 int32_t Servo_Fpga_Read(SERVO_AXIS_ENUM axis_id,
                                                SERVO_FPGA_REG_CFG_ENUM map_id);

 uint32_t Servo_Fpga_Gen_Read(uint32_t gen_addr);

 void Servo_Fpga_Gen_Write(uint32_t gen_addr,
                                                          int32_t input_data);

 void Servo_Fpga_Write(SERVO_AXIS_ENUM axis_id,
                                                    SERVO_FPGA_REG_CFG_ENUM map_id
                                                   ,int32_t out2fpga);

 void Servo_Read_p_FPGA(SERVO_AXIS_ENUM axis_id);

 void Servo_Write_p_FPGA(SERVO_AXIS_ENUM axis_id);

 void Servo_Fpga_Get_IqdPtr(SERVO_FPGA_IQID_STRUCT **iqd_data
                                                     ,SERVO_AXIS_ENUM axis_id);

 void Servo_Fpga_Get_VelPtr(SERVO_FPGA_VEL_BACK_STRUCT **vel_ptr
                                                      ,SERVO_AXIS_ENUM axis_id);

 void Servo_Fpga_Get_EncPtr(SERVO_FPGA_ENCODE_BACK_STRUCT **encode_ptr
                                                      ,SERVO_AXIS_ENUM axis_id);

 void Servo_Fpga_Heartbeat(void);
 void Servo_Fpga_Reset(SERVO_AXIS_ENUM axis_id);
 void Servo_Fpga_FeedDog(int32_t input_data);
 }

 #endif /* SERVO_FPGA_H_ */

