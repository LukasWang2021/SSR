/*************************************************************************
	> File Name: motion_control_offset_calibrator.cpp
	> Author: 
	> Mail: 
	> Created Time: 2016年12月12日 星期一 14时30分05秒
 ************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <math.h>
#include <motion_control_offset_calibrator.h>
#include <common_file_path.h>

#define NVRAM_ZERO_OFFSET_START_ADDR    0x1EF00
#define NVRAM_ZERO_OFFSET_DATA_SIZE     0x100
#define NVRAM_ZERO_OFFSET_MAGIC_NUM     0x5A6F95B7
#define OFFSET_MAGIC_ADDR   (NVRAM_ZERO_OFFSET_START_ADDR + 0x00)
#define OFFSET_STATE_ADDR   (NVRAM_ZERO_OFFSET_START_ADDR + 0x10)
#define OFFSET_MASK_ADDR    (NVRAM_ZERO_OFFSET_START_ADDR + 0x20)
#define OFFSET_JOINT_ADDR   (NVRAM_ZERO_OFFSET_START_ADDR + 0x30)

using namespace std;
using namespace basic_alg;
using namespace log_space;

namespace group_space
{

Calibrator::Calibrator(void)
{
    joint_num_ = 0;
    bare_core_ptr_ = NULL;
    current_state_ = MOTION_FORBIDDEN;
    memset(b_log_flag_, false, sizeof(b_log_flag_));
    memset(i_com_flag_, NORMAL, NUM_OF_JOINT * sizeof(int));
    memset(normal_threshold_, 0, NUM_OF_JOINT * sizeof(double));
    memset(lost_threshold_, 0, NUM_OF_JOINT * sizeof(double));
    memset(zero_offset_, 0, NUM_OF_JOINT * sizeof(double));
    memset(offset_mask_, 0, NUM_OF_JOINT * sizeof(OffsetMask));
    memset(offset_stat_, 0, NUM_OF_JOINT * sizeof(OffsetState));
}

Calibrator::~Calibrator(void)
{
}

//------------------------------------------------------------------------------
// 方法：  initCalibrator
// 摘要：  初始化标定模块，未被初始化过的标定模块不能用于标定，此时调用标定模块任何方法得到的结果
//        都是不可预知的，所以确保在标定模块开始工作之前调用此方法进行初始化配置；
//        主要包括加载配置文件和记录文件，并向裸核发送零位配置；
//------------------------------------------------------------------------------
ErrorCode Calibrator::initCalibrator(size_t joint_num, BareCoreInterface *pcore)
{
    // 输入参数检查
    if (joint_num > 0 && joint_num <= NUM_OF_JOINT && pcore)
    {
        joint_num_ = joint_num;
        bare_core_ptr_ = pcore;
    }
    else
    {
        LogProducer::error("mc_calib","initCalibrator: invalid parameter, joint-number = %d, bare-core-ptr = %p", joint_num, pcore);
        return MC_FAIL_IN_INIT;
    }

    vector<int> stat;
    vector<double> data;
    char buffer[LOG_TEXT_SIZE];
    LogProducer::info("mc_calib","Initializing offset calibrator, number-of-joint = %d.", joint_num_);
    //初始化操作Nvram，获取零点状态，零点错误屏蔽标志
    if (!nvram_handler_.init())
    {
        LogProducer::error("mc_calib","Fail to init NvRam.");
        return MC_FAIL_IN_INIT;
    }

    uint32_t magic_number = 0;
    nvram_handler_.readNvram(OFFSET_MAGIC_ADDR, (uint8_t*)&magic_number, sizeof(magic_number));

    if (magic_number != NVRAM_ZERO_OFFSET_MAGIC_NUM)
    {
        LogProducer::warn("mc_calib","Offset data in nvram is invalid, rebuild offset data");
        uint8_t stat[NUM_OF_JOINT];
        uint8_t mask[NUM_OF_JOINT];
        double joint[NUM_OF_JOINT];

        for (uint32_t i = 0; i < NUM_OF_JOINT; i++)
        {
            stat[i] = static_cast<uint8_t>(OFFSET_LOST);
            mask[i] = static_cast<uint8_t>(OFFSET_UNMASK);
            joint[i] = 0;
        }
        
        magic_number = NVRAM_ZERO_OFFSET_MAGIC_NUM;
        nvram_handler_.writeNvram(OFFSET_STATE_ADDR, stat, sizeof(stat));
        nvram_handler_.writeNvram(OFFSET_MASK_ADDR, mask, sizeof(mask));
        nvram_handler_.writeNvram(OFFSET_JOINT_ADDR, (uint8_t*)joint, sizeof(joint));
        nvram_handler_.writeNvram(OFFSET_MAGIC_ADDR, (uint8_t*)&magic_number, sizeof(magic_number));
    }

    ErrorCode err = readOffsetState(offset_stat_);
    if (err != SUCCESS)
    {
        LogProducer::error("mc_calib","Fail to read offset state, code = 0x%llx", err);
        return err;
    }
    LogProducer::info("mc_calib","Offset state: %s", printDBLine((int*)offset_stat_, buffer, LOG_TEXT_SIZE));

    err = readOffsetMask(offset_mask_);
    if (err != SUCCESS)
    {
        LogProducer::error("mc_calib","Fail to read offset mask, code = 0x%llx", err);
        return err;
    }
    LogProducer::info("mc_calib","Offset mask: %s", printDBLine((int*)offset_mask_, buffer, LOG_TEXT_SIZE));

    Joint offset_joint;
    err = readOffsetJoint(offset_joint);
    if (err != SUCCESS)
    {
        LogProducer::error("mc_calib","Fail to read offset joint, code = 0x%llx", err);
        return err;
    }
    LogProducer::info("mc_calib","Offset joint: %s", printDBLine(&offset_joint[0], buffer, LOG_TEXT_SIZE));

    // 加载零位偏移门限值和零位丢失门限值，并检查数据是否合法
    string config_file = AXIS_GROUP_DIR;
    base_space::YamlHelp params;
    if (!params.loadParamFile(config_file + "base_group.yaml"))
    {
        LogProducer::error("mc_calib","Fail to load thresholds config file(base_group.yaml)");
        return MC_LOAD_PARAM_FAILED;
    }

    data.clear();
    if (!params.getParam("calibrator/normal_offset_threshold", data))
    {
        LogProducer::error("mc_calib","Fail to read threshold from config file");
        return MC_LOAD_PARAM_FAILED;
    }
    if (data.size() != joint_num_)
    {
        LogProducer::error("mc_calib","Invalid array size of normal threshold, except %d but get %d", joint_num_, data.size());
        return INVALID_PARAMETER;
    }    
    LogProducer::info("mc_calib","Threshold-normal: %s", printDBLine(&data[0], buffer, LOG_TEXT_SIZE));
    for (size_t i = 0; i < joint_num_; i++)
        normal_threshold_[i] = data[i];

    data.clear();
    params.getParam("calibrator/lost_offset_threshold", data);
    if (data.size() != joint_num_)
    {
        LogProducer::error("mc_calib","Invalid array size of lost threshold, except %d but get %d", joint_num_, data.size());
        return INVALID_PARAMETER;
    }
    LogProducer::info("mc_calib","Threshold-lost: %s", printDBLine(&data[0], buffer, LOG_TEXT_SIZE));
    for (size_t i = 0; i < joint_num_; i++)
        lost_threshold_[i] = data[i];

    return SUCCESS;
}

//------------------------------------------------------------------------------
// 方法：  checkOffset
// 摘要：  调用checkOffset进行一次零位校验，给出各轴的零位校验结果和机器人是否允许运动的标志；
//        注意只有在机器人处于完全静止的状态下才能进行零位校验，否则校验结果不具有任何意义；
//------------------------------------------------------------------------------
ErrorCode Calibrator::checkOffset(CalibrateState &cali_stat, OffsetState (&offset_stat)[NUM_OF_JOINT])
{
    ServoState servo_state;
    Joint cur_jnt, old_jnt;
    uint32_t encoder_state[NUM_OF_JOINT];
    char buffer[LOG_TEXT_SIZE];
    LogProducer::info("mc_calib","Check zero offset.");

    // 获取NvRam中各关节的位置
    ErrorCode err = readOffsetJoint(old_jnt);
    if (err != SUCCESS)
    {
        LogProducer::error("mc_calib","Fail to get offset joint from NvRam, code = 0x%llx.", err);
        current_state_ = MOTION_FORBIDDEN;
        cali_stat = current_state_;
        memcpy(offset_stat, offset_stat_, sizeof(offset_stat));
        return err;
    }

    OffsetState nvram_state[NUM_OF_JOINT];
    err = readOffsetState(nvram_state);
    if (err != SUCCESS)
    {
        LogProducer::error("mc_calib","Fail to get offset state from NvRam, code = 0x%llx.", err);
        current_state_ = MOTION_FORBIDDEN;
        cali_stat = current_state_;
        memcpy(offset_stat, offset_stat_, sizeof(offset_stat));
        return err;
    }

    // 获取当前各关节的位置
    if (!bare_core_ptr_->getLatestJoint(cur_jnt, encoder_state, servo_state))
    {
        LogProducer::error("mc_calib","Fail to get current joint from share mem."); 
        current_state_ = MOTION_FORBIDDEN;
        cali_stat = current_state_;
        memcpy(offset_stat, offset_stat_, sizeof(offset_stat));
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }

    LogProducer::info("mc_calib","Curr-joint: %s", printDBLine(&cur_jnt[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_calib","Last-joint: %s", printDBLine(&old_jnt[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_calib","Mask-flags: %s", printDBLine((int*)offset_mask_, buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_calib","Nvram-state: %s", printDBLine((int*)nvram_state, buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_calib","encoder_state: %s", printDBLine((int*)encoder_state, buffer, LOG_TEXT_SIZE));

    // 当前各关节位置和记录文件中各关节位置进行比对
    OffsetState state[NUM_OF_JOINT];
    checkOffsetStates(cur_jnt, old_jnt, state, encoder_state);
    LogProducer::info("mc_calib","New-state: %s", printDBLine((int*)state, buffer, LOG_TEXT_SIZE));

    // 比对结果比记录文件中的状态标志更严重，则更新记录文件中的标志
    bool recorder_need_update = false;
    for (size_t i = 0; i < joint_num_; i++)
    {
        if (state[i] == OFFSET_INVALID)
        {
            offset_stat_[i] = state[i];
        }
        else if (offset_mask_[i] == OFFSET_UNMASK && state[i] > offset_stat_[i])
        {
            recorder_need_update = true;
            offset_stat_[i] = state[i];
            nvram_state[i] = state[i];
        }
        else if (offset_stat_[i] == OFFSET_INVALID)
        {
            offset_stat_[i] = nvram_state[i];
        }
        else
        {
            state[i] = offset_stat_[i];
        }
    }
    for (size_t i = joint_num_; i < NUM_OF_JOINT; i++)
    {
        offset_stat_[i] = OFFSET_LOST;
    }
    LogProducer::info("mc_calib","Offset-state: %s", printDBLine((int*)offset_stat_, buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_calib","Nvram-state: %s", printDBLine((int*)nvram_state, buffer, LOG_TEXT_SIZE));

    if (recorder_need_update)
    {
        // 更新记录文件中的零位标志
        LogProducer::info("mc_calib","Update offset state into recorder.");
        err = writeOffsetState(nvram_state);
        if (err != SUCCESS)
        {
            LogProducer::error("mc_calib","Failed, code = 0x%llx", err);
            current_state_ = MOTION_FORBIDDEN;
            cali_stat = current_state_;
            memcpy(offset_stat, offset_stat_, sizeof(offset_stat));
            return err;
        }
    }

    checkCalibrateState();
    cali_stat = current_state_;
    memcpy(offset_stat, offset_stat_, sizeof(offset_stat));
    LogProducer::info("mc_calib","Done, calibrate motion-state is %d", current_state_);
    return SUCCESS;
}

//------------------------------------------------------------------------------
// 方法：  checkOffsetStates
// 摘要：  对各个轴的当前读数和记录值进行比对，根据偏移门限和丢失门限给出比对结果；
//------------------------------------------------------------------------------
void Calibrator::checkOffsetStates(Joint curr_jnt, Joint last_jnt, OffsetState *offset_stat, const uint32_t (&encoder_state)[NUM_OF_JOINT])
{
    for (size_t i = 0; i < joint_num_; i++)
    {
        LogProducer::info("mc_calib","Joint %d: encoder battery alarm bit %d, encoder communication alarm bit: %d", i, encoder_state[i] & 0x1, (encoder_state[i] >> 4) & 0x1);
        
        if (((encoder_state[i] >> 4) & 0x1) != 0)
        {
            offset_stat[i] = OFFSET_INVALID;
        }
        else
        {
            if (((encoder_state[i]) & 0x1) != 0)
            {
                offset_stat[i] = OFFSET_LOST;
            }
            else
            {
                if (fabs(curr_jnt[i] - last_jnt[i]) > lost_threshold_[i])
                {
                    offset_stat[i] = OFFSET_LOST;
                }
                else if (fabs(curr_jnt[i] - last_jnt[i]) > normal_threshold_[i])
                {
                    offset_stat[i] = OFFSET_DEVIATE;
                }
                else
                {
                    offset_stat[i] = OFFSET_NORMAL;
                }
            }
        }
    }
}

void Calibrator::checkCalibrateState(void)
{
    //LogProducer::info("mc_calib","Check Calibrate State.");   
    bool limited = false;
    bool forbidden = false;

    for (size_t i = 0; i < joint_num_; i++)
    {
        if (offset_mask_[i] == OFFSET_UNMASK)
        {
            if (offset_stat_[i] != OFFSET_NORMAL)
            {
                forbidden = true;
            }
        }
        else
        {
            limited = true;
        }
    }

    current_state_ = forbidden ? MOTION_FORBIDDEN : (limited ? MOTION_LIMITED : MOTION_NORMAL);
    //LogProducer::info("mc_calib","Done check Calibrate State, calibrate motion-state is %d", current_state_);   
}

//------------------------------------------------------------------------------
// 方法：  updateOffset
// 摘要：  将给定的零偏下发到裸核并写入配置文件,同时更新相关的状态；
//------------------------------------------------------------------------------
ErrorCode Calibrator::updateOffset(uint32_t index, double offset)
{
    LogProducer::info("mc_calib","Update zero offset to %.6f of joint %d", offset, index);
    Joint cur_joint, old_joint;
    ServoState  state;
    uint32_t encoder_state[NUM_OF_JOINT];
    //检查编码器状态，是否可更新零位
    if (!bare_core_ptr_->getLatestJoint(cur_joint, encoder_state, state))
    {
        LogProducer::error("mc_calib","Fail to get current joint from core1.");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }
    if (((encoder_state[index] >> 4) & 0x1) != 0)
    {
        LogProducer::info("mc_calib","Lost communication to encoder of axis %d, can not update offset", index);
        return MC_COMMUNICATION_WITH_ENCODER_FAIL;
    }

    // 更新裸核零位
    ErrorCode err = bare_core_ptr_->setOffsetPositions(index, offset);
    if (err != SUCCESS)
    {
        LogProducer::error("mc_calib","Cannot update offset in core1, code = 0x%llx", err);
        return err;
    }

    OffsetMask  offset_mask[NUM_OF_JOINT];
    OffsetState offset_stat[NUM_OF_JOINT];

    // 获取当前轴位置
    if (!bare_core_ptr_->getLatestJoint(cur_joint, encoder_state, state))
    {
        LogProducer::error("mc_calib","Fail to get current joint from core1.");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }
    char buffer[LOG_TEXT_SIZE];
    LogProducer::info("mc_calib","Current joint: %s", printDBLine(&cur_joint[0], buffer, LOG_TEXT_SIZE));

    err = readOffsetJoint(old_joint);
    if (err != SUCCESS)
    {
        LogProducer::error("mc_calib","Fail to get last joint from nvram.");
        return err;
    }

    for (size_t i = 0; i < joint_num_; i++)
    {
        offset_mask[i] = offset_mask_[i];
        offset_stat[i] = offset_stat_[i];
    }
    for (size_t i = joint_num_; i < NUM_OF_JOINT; i++)
    {
        offset_mask[i] = OFFSET_UNMASK;
        offset_stat[i] = OFFSET_LOST;
    }
    //清除当前轴的零位丢失标志和错误屏蔽标志
    offset_stat[index] = OFFSET_NORMAL;
    offset_mask[index] = OFFSET_UNMASK;
    old_joint[index] = cur_joint[index];

    LogProducer::info("mc_calib","  flag = %s", printDBLine((int*)offset_stat, buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_calib","  mask = %s", printDBLine((int*)offset_mask, buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_calib","  joint = %s", printDBLine(&old_joint[0], buffer, LOG_TEXT_SIZE));
    //更新存储的关节位置，零位丢失标志，错误屏蔽标志。
    if (writeOffsetJoint(old_joint) != SUCCESS || writeOffsetState(offset_stat) != SUCCESS || writeOffsetMask(offset_mask) != SUCCESS)
    {
        LogProducer::error("mc_calib","Fail to update recorder in nvram.");
        return MC_OPERATE_NVRAM_FAILED;
    }

    zero_offset_[index] = offset;
    offset_mask_[index] = OFFSET_UNMASK;
    offset_stat_[index] = OFFSET_NORMAL;
    return SUCCESS;
}

//------------------------------------------------------------------------------
// 方法：  saveJoint
// 摘要：  将当前的关节读数写入记录文件；
//------------------------------------------------------------------------------
ErrorCode Calibrator::saveJoint(void)
{
    Joint cur_jnt;
    ServoState state;
    memset(&cur_jnt, 0, sizeof(cur_jnt));
    Joint read_nvram_jnt;
    memset(&read_nvram_jnt, 0, sizeof(read_nvram_jnt));
    uint32_t encoder_state[NUM_OF_JOINT];

    ErrorCode err = readOffsetJoint(read_nvram_jnt);
    if (err != SUCCESS)
    {
        LogProducer::error("mc_calib","Fail to get offset joint from NvRam, code = 0x%llx.", err);
        return err;
    }
    //获取所有编码器状态
    if (!bare_core_ptr_->getLatestJoint(cur_jnt, encoder_state, state))
    {
        LogProducer::error("mc_calib","Fail to get current joint from share memory");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }

    for (size_t i = 0; i < joint_num_; i++)
    {
        // if ((encoder_state[i] & 0x1) != 0)
        // {
            // LogProducer::error("mc_calib","encoder battery unnormal, axis: %d", i);
            // offset_stat_[i] = OFFSET_LOST;
        // }

        if (((encoder_state[i] >> 4) & 0x1) != 0)
        {
            // LogProducer::error("mc_calib","encoder communication unnormal, axis: %d", i);
            offset_stat_[i] = OFFSET_INVALID;
        }
    }
    //检查编码器状态
    for (size_t i = 0; i < joint_num_; ++i)
    {
        //编码器电量不足
        if (((encoder_state[i]) & 0x1) != 0)
        {
            if (!b_log_flag_[i])
            {
                LogProducer::info("mc_calib","encoder battery unnormal axis: %d", i);
                b_log_flag_[i] = true;
            }   
            cur_jnt[i] = read_nvram_jnt[i];
        }
        else//编码器电量恢复
        {
            if (b_log_flag_[i])
            {
                LogProducer::info("mc_calib","encoder battery recovery axis: %d", i);
                b_log_flag_[i] = false;
            }   
        }
        //编码器无通信
        if (((encoder_state[i] >> 4) & 0x1) != 0)
        {
            if (i_com_flag_[i] == NORMAL)
            {
                LogProducer::info("mc_calib","encoder communication unnormal axis: %d", i);
            }           
            i_com_flag_[i] = UNNORMAL;
            cur_jnt[i] = read_nvram_jnt[i];
        }//编码器通信恢复
        else if (i_com_flag_[i] == UNNORMAL)
        {
            LogProducer::info("mc_calib","encoder communication recovery axis: %d", i);
            i_com_flag_[i] = RECOVERY;
        }
    }

    //如果恢复通信，则checkoffset
    bool b_check_flag = false;
    for (size_t i = 0; i < joint_num_; ++i)
    {
        if (i_com_flag_[i] == RECOVERY)
        {
            i_com_flag_[i] = NORMAL;
            b_check_flag = true;
        }
    }
    if (b_check_flag)
    {
        CalibrateState calibrate_stat;
        OffsetState offset_stat[NUM_OF_JOINT];
        return checkOffset(calibrate_stat, offset_stat);
    }

    //如果持续无通信，维持当前关节位置
    for (size_t i = 0; i < joint_num_; i++)
    {
        if (offset_stat_[i] != OFFSET_NORMAL)
        {
            // LogProducer::error("mc_calib","offset unormal axis: %d",i);
            cur_jnt[i] = read_nvram_jnt[i];
        }
    }
    err = writeOffsetJoint(cur_jnt);
    if (err != SUCCESS)
    {
        LogProducer::error("mc_calib","Fail to record joint, code = 0x%llx", err);
        return err;
    }
    //char buffer[LOG_TEXT_SIZE];
    //LogProducer::info("mc_calib","Save current joint into record file: %s", printDBLine(&cur_jnt.j1_, buffer, LOG_TEXT_SIZE));
    return SUCCESS;
}

//------------------------------------------------------------------------------
// 方法：  maskOffsetLostError
// 摘要：  屏蔽零位错误，如果有轴零位丢失，该方法将设置此轴的屏蔽标志，之后将不对此轴进行零位校验，
//        且机器人只能进入受限运行模式；
//------------------------------------------------------------------------------
ErrorCode Calibrator::maskOffsetLostError(void)
{
    char buffer[LOG_TEXT_SIZE];
    OffsetMask mask[NUM_OF_JOINT];

    LogProducer::info("mc_calib","Mask all lost-offset errors.");
    LogProducer::info("mc_calib","Mask flag: %s", printDBLine((int*)offset_mask_, buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_calib","Offset state: %s", printDBLine((int*)offset_stat_, buffer, LOG_TEXT_SIZE));

    bool need_save = false;

    for (size_t i = 0; i < joint_num_; i++)
    {
        mask[i] = offset_mask_[i];

        if ((offset_stat_[i] == OFFSET_LOST) && (offset_mask_[i] == OFFSET_UNMASK))
        {
            LogProducer::info("mc_calib","index = %d, lost its offset, we'll try to mask this error.", i);
            mask[i] = OFFSET_MASKED;
            need_save = true;
        }
    }
    for (size_t i = joint_num_; i < NUM_OF_JOINT; i++)
    {
        mask[i] = OFFSET_UNMASK;
    }

    // 至少有一个轴需要屏蔽零位校验错误,否则直接返回
    if (!need_save)
    {
        LogProducer::info("mc_calib","No lost-offset error need mask");
        return SUCCESS;
    }

    ErrorCode err = writeOffsetMask(mask);
    if (err != SUCCESS)
    {
        LogProducer::error("mc_calib","Fail to save mask flags into recorder, code = 0x%llx", err);
        return err;
    }

    for (size_t i = 0; i < joint_num_; i++)
    {
        if (offset_stat_[i] == OFFSET_LOST)
        {
            offset_mask_[i] = mask[i];
        }
    }

    LogProducer::info("mc_calib","All lost-offset errors masked.");
    return SUCCESS;
}

//------------------------------------------------------------------------------
// 方法：  setOffsetState
// 摘要：  当某些轴发生零位偏移时，允许用户对这些轴进行设置，将其状态归为零位正常或者零位丢失；
//------------------------------------------------------------------------------
ErrorCode Calibrator::setOffsetState(size_t index, OffsetState stat)
{
    char buffer[LOG_TEXT_SIZE];
    LogProducer::info("mc_calib","Set offset state, index = %d, state = %d", index, stat);

    if (index >= joint_num_)
    {
        LogProducer::error("mc_calib","Invalid index: %d", index);
        return INVALID_AXIS_ID;
    }

    Joint old_joint;
    bool update_joint = false;
    OffsetState stats[NUM_OF_JOINT];
    memcpy(stats, offset_stat_, sizeof(stats));
    LogProducer::info("mc_calib","Offset state: %s", printDBLine((int*)stats, buffer, LOG_TEXT_SIZE));
    stats[index] = stat;
    LogProducer::info("mc_calib","  changed to: %s", printDBLine((int*)stats, buffer, LOG_TEXT_SIZE));

    if (stat == OFFSET_NORMAL && offset_stat_[index] != OFFSET_NORMAL)
    {
        update_joint = true;
    }

    Joint cur_joint;
    ServoState servo_state;
    uint32_t encoder_state[NUM_OF_JOINT];
    ErrorCode err = readOffsetJoint(old_joint);
    if (err != SUCCESS)
    {
        LogProducer::error("mc_calib","Fail to get last joint from recorder, code = 0x%llx", err);
        return err;
    }

    if (!bare_core_ptr_->getLatestJoint(cur_joint, encoder_state, servo_state))
    {
        LogProducer::error("mc_calib","Fail to get current joint from bare core.");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }

    if (((encoder_state[index] >> 4) & 1) != 0)
    {
        LogProducer::info("mc_calib","Lost communication to encoder of axis %d, can not set offset state", index);
        return MC_COMMUNICATION_WITH_ENCODER_FAIL;
    }

    if (update_joint)
    {
        old_joint[index] = cur_joint[index];
    }

    if (writeOffsetState(stats) != SUCCESS || writeOffsetJoint(old_joint) != SUCCESS)
    {
        LogProducer::error("mc_calib","Fail to set offset flag into recorder");
        return MC_OPERATE_NVRAM_FAILED;
    }

    offset_stat_[index] = stats[index];
    LogProducer::info("mc_calib","Success.");
    return SUCCESS;
}

//------------------------------------------------------------------------------
// 方法：  setOffset
// 摘要：  允许用户直接设置某个轴的零位值；
//------------------------------------------------------------------------------
ErrorCode Calibrator::setOffset(size_t index, double offset)
{
    if (index < joint_num_)
    {
        updateOffset(index, offset);
        return SUCCESS;
    }
    else
    {
        LogProducer::error("mc_calib","Given index out of range, joint-number is %d", joint_num_);
        return INVALID_AXIS_ID;
    }
}

//------------------------------------------------------------------------------
// 方法：  setOffset
// 摘要：  允许用户直接设置所有轴的零位值；
//        注意确保传入的offset数组长度不小于有效轴数
//------------------------------------------------------------------------------
ErrorCode Calibrator::setOffset(const double *offset)
{
    char buf[LOG_TEXT_SIZE];
    LogProducer::info("mc_calib","Set offset of all joints, new-offset: %s", printDBLine(offset, buf, LOG_TEXT_SIZE));
    
    for (size_t i = 0; i < joint_num_; i++)
    {
        updateOffset(i, offset[i]);
    }

    return SUCCESS;
}

//------------------------------------------------------------------------------
// 方法：  getOffset
// 摘要：  获取所有轴的零位值，注意确保数组长度不小于实际轴数；
//------------------------------------------------------------------------------
void Calibrator::getOffset(double *offset)
{
    memcpy(offset, zero_offset_, joint_num_ * sizeof(double));
}

//------------------------------------------------------------------------------
// 方法：  getOffsetState
// 摘要：  获取所有轴的零位状态，注意确保数组长度不小于实际轴数；
//------------------------------------------------------------------------------
void Calibrator::getOffsetState(OffsetState *state)
{
    for (size_t i = 0; i < joint_num_; i++)
    {
        state[i] = offset_stat_[i];
    }
}

//------------------------------------------------------------------------------
// 方法：  getOffsetMask
// 摘要：  获取所有轴的错误屏蔽标志，注意确保数组长度不小于实际轴数；
//------------------------------------------------------------------------------
void Calibrator::getOffsetMask(OffsetMask *mask)
{
    for (size_t i = 0; i < joint_num_; i++)
    {
        mask[i] = offset_mask_[i];
    }
}

//------------------------------------------------------------------------------
// 方法：  getCalibrateState
// 摘要：  获取允许的运行状态，所有轴零位正常处于正常运行模式，某些轴零位错误被屏蔽处于受限模式，
//        某些轴零位异常且没有被屏蔽则处于禁止运行模式；
//------------------------------------------------------------------------------
CalibrateState Calibrator::getCalibrateState(void)
{
    checkCalibrateState();
    return current_state_;
}


ErrorCode Calibrator::resetEncoderMultiTurnValue()
{
    LogProducer::info("mc_calib","reset encoder multi-turn value of all joints.");

    ErrorCode result = bare_core_ptr_->resetEncoderError();
    if (result != SUCCESS)
    {
        LogProducer::error("mc_calib","Fail to reset encoder errors.");
        return result;
    }

    usleep(250 * 1000);

    LogProducer::info("mc_calib","Zero loss flag set");
    for (size_t i = 0; i < joint_num_; i++)
    {
        offset_stat_[i] = OFFSET_LOST;
    }

    result = writeOffsetState(offset_stat_);

    return result;
}


ErrorCode Calibrator::readOffsetState(OffsetState (&state)[NUM_OF_JOINT])
{
    uint8_t data[NUM_OF_JOINT];

    if (!nvram_handler_.readNvram(OFFSET_STATE_ADDR, data, sizeof(data)))
    {
        return MC_OPERATE_NVRAM_FAILED;
    }

    for (size_t i = 0; i < NUM_OF_JOINT; i++)
    {
        state[i] = static_cast<OffsetState>(data[i]);
    }

    return SUCCESS;
}

ErrorCode Calibrator::readOffsetMask(OffsetMask (&mask)[NUM_OF_JOINT])
{
    uint8_t data[NUM_OF_JOINT];

    if (!nvram_handler_.readNvram(OFFSET_MASK_ADDR, data, sizeof(data)))
    {
        return MC_OPERATE_NVRAM_FAILED;
    }

    for (size_t i = 0; i < NUM_OF_JOINT; i++)
    {
        mask[i] = static_cast<OffsetMask>(data[i]);
    }

    return SUCCESS;
}

ErrorCode Calibrator::readOffsetJoint(Joint &joint)
{
    double data[NUM_OF_JOINT];

    if (!nvram_handler_.readNvram(OFFSET_JOINT_ADDR, (uint8_t*)data, sizeof(data)))
    {
        return MC_OPERATE_NVRAM_FAILED;
    }
    
    memcpy(&joint, data, sizeof(joint));
    return SUCCESS;
}

ErrorCode Calibrator::writeOffsetState(const OffsetState (&state)[NUM_OF_JOINT])
{
    uint8_t data[NUM_OF_JOINT];

    for (size_t i = 0; i < NUM_OF_JOINT; i++)
    {
        data[i] = static_cast<uint8_t>(state[i]);
    }

    return nvram_handler_.writeNvram(OFFSET_STATE_ADDR, data, sizeof(data)) ? SUCCESS : MC_OPERATE_NVRAM_FAILED;
}

ErrorCode Calibrator::writeOffsetMask(const OffsetMask (&mask)[NUM_OF_JOINT])
{
    uint8_t data[NUM_OF_JOINT];

    for (size_t i = 0; i < NUM_OF_JOINT; i++)
    {
        data[i] = static_cast<uint8_t>(mask[i]);
    }

    return nvram_handler_.writeNvram(OFFSET_MASK_ADDR, data, sizeof(data)) ? SUCCESS : MC_OPERATE_NVRAM_FAILED;
}

ErrorCode Calibrator::writeOffsetJoint(const Joint &joint)
{
    return nvram_handler_.writeNvram(OFFSET_JOINT_ADDR, (const uint8_t*)&joint, sizeof(Joint)) ? SUCCESS : MC_OPERATE_NVRAM_FAILED;
}

//------------------------------------------------------------------------------
// 方法：  printDBLine
// 摘要：  根据实际轴数格式化输出数据组；
//------------------------------------------------------------------------------
char* Calibrator::printDBLine(const int *data, char *buffer, size_t length)
{
    int len = 0;
    
    for (size_t i = 0; i < joint_num_; i++)
    {
        len += snprintf(buffer + len, length - len, "%d ", data[i]);
    }

    if (len > 0)
    {
        len --;
    }

    buffer[len] = '\0';
    return buffer;
}

//------------------------------------------------------------------------------
// 方法：  printDBLine
// 摘要：  根据实际轴数格式化输出数据组；
//------------------------------------------------------------------------------
char* Calibrator::printDBLine(const double *data, char *buffer, size_t length)
{
    int len = 0;
    
    for (size_t i = 0; i < joint_num_; i++)
    {
        len += snprintf(buffer + len, length - len, "%.6f ", data[i]);
    }

    if (len > 0)
    {
        len --;
    }

    buffer[len] = '\0';
    return buffer;
}

}
