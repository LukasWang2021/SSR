/*************************************************************************
	> File Name: motion_controller_offset_calibrator.h
	> Author: 
	> Mail: 
	> Created Time: 2016年12月12日 星期一 14时30分56秒
 ************************************************************************/

#ifndef _MOTION_CONTROLLER_OFFSET_CALIBRATOR_H
#define _MOTION_CONTROLLER_OFFSET_CALIBRATOR_H

#include <string>
#include <vector>
#include <motion_control_datatype.h>
#include <error_code.h>
#include <common_log.h>
#include <motion_control_core_interface.h>
#include <parameter_manager/parameter_manager_param_group.h>
#include <nvram.h>

namespace fst_mc
{

enum CalibrateState
{
    MOTION_NORMAL,          // 正常运行模式
    MOTION_LIMITED,         // 受限运行模式，允许关节空间的低速手动示教
    MOTION_FORBIDDEN,       // 禁止运行模式
};

enum OffsetState
{
    OFFSET_NORMAL,          // 零位正常
    OFFSET_DEVIATE,         // 零位偏移
    OFFSET_LOST,            // 零位丢失
};

enum OffsetMask
{
    OFFSET_UNMASK,          // 未屏蔽零位错误，零位校验时对该轴零位进行检查
    OFFSET_MASKED,          // 屏蔽零位错误，零位校验时不对该轴进行检查
};

static const int NEED_SAVE = 1;
static const int UNNEED_SAVE = 0;


class Calibrator
{

//***************************************************************************************************************************//
  public:
    Calibrator();
    ~Calibrator(void);

    //------------------------------------------------------------------------------
    // 方法：  initCalibrator
    // 摘要：  对零位校验模块进行初始配置；
    // 输入：  joint_num - 机器人轴数
    //        pcore - 与裸核进行通信的句柄
    //        plog - 日志操作句柄
    //        path - 零位记录文件所在目录
    // 输出：  None
    // 返回：  错误代码，详见报警代码表
    //------------------------------------------------------------------------------
    ErrorCode initCalibrator(size_t joint_num, BareCoreInterface *pcore, fst_log::Logger *plog);

    //------------------------------------------------------------------------------
    // 方法：  checkOffset
    // 摘要：  在当前位置进行一次零位校验，注意只有在机器人处于静止状态下才能进行零位校验；
    // 输入：  None
    // 输出：  cali_stat - 零位校验结果的汇总，指示机器人现在处于正常运行模式、受限运行模式或者禁止运行模式
    //        offset_stat - 每个轴的零位校验结果
    // 返回：  错误代码，详见报警代码表
    //------------------------------------------------------------------------------
    ErrorCode checkOffset(CalibrateState &cali_stat, OffsetState (&offset_stat)[NUM_OF_JOINT]);

    //------------------------------------------------------------------------------
    // 方法：  calibrateOffset
    // 摘要：  在当前位置对所有轴进行一次零位标定，注意只有在机器人处于静止状态下才能进行零位标定；
    //        标定的新零位需要调用saveOffset借口进行保存，否则重启后本次标定的零位将会丢失；
    // 输入：  None
    // 输出：  None
    // 返回：  错误代码，详见报警代码表
    //------------------------------------------------------------------------------
    ErrorCode calibrateOffset(void);

    //------------------------------------------------------------------------------
    // 方法：  calibrateOffset
    // 摘要：  在当前位置对某个轴进行一次零位标定，注意只有在机器人处于静止状态下才能进行零位标定；
    //        标定的新零位需要调用saveOffset借口进行保存，否则重启后本次标定的零位将会丢失；
    // 输入：  index - 需要标定的轴标号，0 ~ NUM_OF_JOINT
    // 输出：  None
    // 返回：  错误代码，详见报警代码表
    //------------------------------------------------------------------------------
    ErrorCode calibrateOffset(size_t index);

    //------------------------------------------------------------------------------
    // 方法：  calibrateOffset
    // 摘要：  在当前位置对某几个轴进行一次零位标定，注意只有在机器人处于静止状态下才能进行零位标定；
    //        标定的新零位需要调用saveOffset借口进行保存，否则重启后本次标定的零位将会丢失；
    // 输入：  pindex - 需要标定的轴标号数组地址
    //        length - 需要标定的轴标号数组长度
    // 输出：  None
    // 返回：  错误代码，详见报警代码表
    //------------------------------------------------------------------------------
    ErrorCode calibrateOffset(const size_t *pindex, size_t length);

    //------------------------------------------------------------------------------
    // 方法：  saveOffset
    // 摘要：  将标定的新零位保存到配置文件中；
    // 输入：  None
    // 输出：  None
    // 返回：  错误代码，详见报警代码表
    //------------------------------------------------------------------------------
    ErrorCode saveOffset(void);

    //------------------------------------------------------------------------------
    // 方法：  saveJoint
    // 摘要：  将当前的关节位置保存到零位记录文件中，将在下次零位校验时使用；
    // 输入：  None
    // 输出：  None
    // 返回：  错误代码，详见报警代码表
    //------------------------------------------------------------------------------
    ErrorCode saveJoint(void);

    //------------------------------------------------------------------------------
    // 方法：  maskOffsetLostError
    // 摘要：  屏蔽已然存在的零位错误，零位错误被屏蔽后机器人能够进入engage状态，但处于受限运行模式，
    //        直到所有发生零位错误的轴都已被标定；
    // 输入：  None
    // 输出：  None
    // 返回：  错误代码，详见报警代码表
    //------------------------------------------------------------------------------
    ErrorCode maskOffsetLostError(void);

    //------------------------------------------------------------------------------
    // 方法：  setOffsetState
    // 摘要：  当某些轴发生零位偏移时，允许用户对这些轴进行设置，将其状态归为零位正常或者零位丢失；
    // 输入：  index - 需要设置的轴标号，0 ~ NUM_OF_JOINT
    //        stat - 将该轴的零位状态设置为指定的状态
    // 输出：  None
    // 返回：  错误代码，详见报警代码表
    //------------------------------------------------------------------------------
    ErrorCode setOffsetState(size_t index, OffsetState stat);

    //------------------------------------------------------------------------------
    // 方法：  setOffset
    // 摘要：  允许用户直接设置零位值，被改变的零位值需要调用saveOffset借口才会被写入配置文件中；
    // 输入：  index - 需要设置零位的轴标号，0 ~ NUM_OF_JOINT
    //        offset - 将该轴的零位设置为给定的值
    // 输出：  None
    // 返回：  None
    //------------------------------------------------------------------------------
    void setOffset(size_t index, double offset);

    //------------------------------------------------------------------------------
    // 方法：  setOffset
    // 摘要：  一次性设置所有轴的零位值，被改变的零位值需要调用saveOffset借口才会被写入配置文件中；
    // 输入：  offset - 需要设置的所有轴的零位数值，注意确保数组长度不小于实际轴数
    // 输出：  None
    // 返回：  None
    //------------------------------------------------------------------------------
    void setOffset(const double *offset);

    //------------------------------------------------------------------------------
    // 方法：  getOffset
    // 摘要：  获取所有轴的零位值；
    // 输入：  None
    // 输出：  offset - 获取到的零位数值，注意确保数组长度不小于实际轴数
    // 返回：  None
    //------------------------------------------------------------------------------
    void getOffset(double *offset);

    //------------------------------------------------------------------------------
    // 方法：  getOffsetMask
    // 摘要：  获取所有轴的零位错误屏蔽状态；
    // 输入：  None
    // 输出：  mask - 获取到的零位错误屏蔽状态，注意确保数组长度不小于实际轴数
    // 返回：  None
    //------------------------------------------------------------------------------
    void getOffsetMask(OffsetMask *mask);

    //------------------------------------------------------------------------------
    // 方法：  getCalibrateState
    // 摘要：  获取机器人当前的许可运动状态，正常运行模式、受限运行模式或者禁止运行模式；
    // 输入：  None
    // 输出：  None
    // 返回：  当前的许可运动状态
    //------------------------------------------------------------------------------
    CalibrateState getCalibrateState(void);

    //------------------------------------------------------------------------------
    // 方法：  isReferenceAvailable
    // 摘要：  获取快速标定是否可用的标志，如果之前标记过参考点，可通过快速标定接口进行标定；
    // 输入：  None
    // 输出：  None
    // 返回：  true - 快速标定可用， false - 快速标定不可用
    //------------------------------------------------------------------------------
    bool isReferenceAvailable(void);

    //------------------------------------------------------------------------------
    // 方法：  deleteReference
    // 摘要：  删除用于快速标定的记录参考点；
    // 输入：  None
    // 输出：  None
    // 返回：  错误代码，详见报警代码表
    //------------------------------------------------------------------------------
    ErrorCode deleteReference(void);

    //------------------------------------------------------------------------------
    // 方法：  saveReference
    // 摘要：  将当前位置记录为快速标定参考点，标定过参考点的机器人可通过快速标定对零位进行标定；
    // 输入：  None
    // 输出：  None
    // 返回：  错误代码，详见报警代码表
    //------------------------------------------------------------------------------
    ErrorCode saveReference(void);

    //------------------------------------------------------------------------------
    // 方法：  fastCalibrate
    // 摘要：  快速标定所有轴，注意标定完成后需要调用saveOffset进行保存；
    // 输入：  None
    // 输出：  None
    // 返回：  错误代码，详见报警代码表
    //------------------------------------------------------------------------------
    ErrorCode fastCalibrate(void);

    //------------------------------------------------------------------------------
    // 方法：  fastCalibrate
    // 摘要：  对某个轴进行快速标定，注意标定完成后需要调用saveOffset进行保存；
    // 输入：  index - 需要进行快速标定的轴标号
    // 输出：  None
    // 返回：  错误代码，详见报警代码表
    //------------------------------------------------------------------------------
    ErrorCode fastCalibrate(size_t index);

    //------------------------------------------------------------------------------
    // 方法：  fastCalibrate
    // 摘要：  对某几个轴进行快速标定，注意标定完成后需要调用saveOffset进行保存；
    // 输入：  pindex - 需要进行快速标定的轴标号数组
    //        length - 轴标号数组长度
    // 输出：  None
    // 返回：  错误代码，详见报警代码表
    //------------------------------------------------------------------------------
    ErrorCode fastCalibrate(const size_t *pindex, size_t length);

//***************************************************************************************************************************//
  private:
    void checkOffset(basic_alg::Joint curr_jnt, basic_alg::Joint last_jnt, OffsetState *offset_stat);
    double calculateOffset(double current_offset, double current_joint, double target_joint);
    double calculateOffsetEasy(double gear_ratio, double ref_offset, unsigned int ref_encoder, unsigned int cur_encoder);
    ErrorCode sendConfigData(int id, const std::vector<int> &data);
    ErrorCode sendConfigData(int id, const std::vector<double> &data);
    ErrorCode sendOffsetToBareCore(void);
    ErrorCode getOffsetFromBareCore(std::vector<double> &data);
    ErrorCode writeOffsetJoint(const basic_alg::Joint &joint);
    ErrorCode writeOffsetMask(const OffsetMask (&mask)[NUM_OF_JOINT]);
    ErrorCode writeOffsetState(const OffsetState (&state)[NUM_OF_JOINT]);
    ErrorCode writeNvRam(const void *array, uint32_t addr, uint32_t length);
    ErrorCode readOffsetJoint(basic_alg::Joint &joint);
    ErrorCode readOffsetMask(OffsetMask (&mask)[NUM_OF_JOINT]);
    ErrorCode readOffsetState(OffsetState (&state)[NUM_OF_JOINT]);
    ErrorCode readNvRam(void *array, uint32_t addr, uint32_t length);

    inline char* printDBLine(const int *data, char *buffer, size_t length);
    inline char* printDBLine(const double *data, char *buffer, size_t length);

//***************************************************************************************************************************//
    CalibrateState              current_state_;
    fst_parameter::ParamGroup   coupling_param_;
    fst_parameter::ParamGroup   gear_ratio_param_;
    fst_parameter::ParamGroup   offset_param_;
    fst_parameter::ParamGroup   robot_recorder_;

    Nvram       *nvram_ptr_;

    double      normal_threshold_[NUM_OF_JOINT];
    double      lost_threshold_[NUM_OF_JOINT];
    OffsetMask  offset_mask_[NUM_OF_JOINT];
    OffsetState offset_stat_[NUM_OF_JOINT];
    double      zero_offset_[NUM_OF_JOINT];
    int         offset_need_save_[NUM_OF_JOINT];

    size_t joint_num_;
    BareCoreInterface *bare_core_ptr_;
    fst_log::Logger *log_ptr_;
};


}

#endif
