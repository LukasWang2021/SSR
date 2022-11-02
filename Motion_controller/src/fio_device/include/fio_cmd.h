#ifndef _FIO_CMD_H
#define _FIO_CMD_H

typedef enum 
{
    SET_GRIND_TURN_RIGHT           = 0x0100, // 设置FIO参数-转子向右运动
    SET_GRIND_TURN_LEFT            = 0x0200, // 设置FIO参数-转子向左运动
    SET_GRIND_STOP                 = 0x0300, // 设置FIO参数-转子停止
    SET_GRIND_MAX_SPEED            = 0x0504, // 设置FIO参数-最大速度限制值
    SET_GRIND_ACC                  = 0x050B, // 设置FIO参数-硬件加速度
    SET_RAMP_SWITCH                = 0x050C, // 设置FIO参数-软件RAMP开关
    SET_RAMP_VALUE                 = 0x050D, // 设置FIO参数-软件RAMP的值
    GET_GRIND_MAX_SPEED            = 0x0604, // 查询FIO参数-最大速度设置值
    GET_GRIND_ACC                  = 0x060B, // 查询FIO参数-加速度设置值
    GET_RAMP_SWITCH                = 0x060C, // 查询FIO参数-RAMP开启状态
    GET_RAMP_VALUE                 = 0x060D, // 查询FIO参数-RAMP设置值
    GET_TARGET_CURRENT             = 0x06AB, // 查询FIO参数-目标电流
    GET_TARGET_SPEED               = 0x06AD, // 查询FIO参数-目标速度
    GET_ACTUAL_CURRENT             = 0x06B0, // 查询FIO参数-实际电流
    GET_ACTUAL_SPEED               = 0x06B2, // 查询FIO参数-实际速度
    GET_3V3_VOLTAGE                = 0x0F00, // 查询板卡-3.3V电压
    GET_LDO5V_VOLTAGE              = 0x0F01, // 查询板卡-LDO5V电压
    GET_PWM5V_VOLTAGE              = 0x0F02, // 查询板卡-PWM5V电压
    GET_VM_VOLTAGE                 = 0x0F05, // 查询板卡-母线VM电压
    SET_48V_ENABLE                 = 0x4000, // 设置48V使能
    READ_EEPROM_ID                 = 0x4100, // 读取EEPROM内的ID
    WRITE_EEPROM_ID                = 0x4200, // 将ID写入EEPROM
    READ_VERSION                   = 0x8800, // 读取版本号
    WRITE_TMC4671_REG              = 0x9200, // 0x9200 + OFFSET[0~125] 将VALUE写入TMC4671的寄存器[0-125]
    READ_TMC4671_REG               = 0x9400, // 0x9400 + OFFSET[0~125] 读取TMC4671的寄存器[0-125]
    READ_ERROR_STATE               = 0x9701, // 读取错误状态
    REBOOT_TMC4671                 = 0x9800, // 重启板卡部件TMC4671
    REBOOT_DRV8302                 = 0x9801, // 重启板卡部件通道DRV8302
    REBOOT_TMC4671_DRV8302         = 0x9802, // 重启板卡部件通道TMC4671和DRV8302
    REBOOT_ADS8688_SELFCHECK       = 0x9803, // 重启板卡部件ADS8688  做启动自检
    REBOOT_TMC4671_DRV8302_ADS8688 = 0x9804, // 重启板卡部件TMC4671，DRV8302，ADS8688
    REBOOT_FOC_MCU                 = 0xF200, // 重启MCU软件-REBOOT控制器
    SET_TORQUESENSOR_LP_PARAMETER  = 0xC800, // 设置力矩传感器低通滤波参数。
    SET_TORQUESENSOR_SELFCHECK     = 0xC900, // 设置力矩传感器自检
    READ_SELFCHECK_RESULT          = 0xC901, // 读取自检结果
    SET_TORQUESENSOR_ENABLE_UPLOAD = 0xCA00, // 设置力矩传感器启动上传数据
    SET_BREATHING_LAMP_SWITCH      = 0xCB00, // 设置呼吸灯开关
    SET_BREATHING_LAMP_COLOR       = 0xCB01, // 设置呼吸灯颜色
    SET_BREATHING_LAMP_FREQUENCY   = 0xCB02, // 设置呼吸灯频率
}FioCmd_e;


#endif







