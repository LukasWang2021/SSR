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
#include <boost/filesystem.hpp>
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
    
    /*// 下发关节数量到裸核,SSR不用
    LogProducer::info("mc_calib","ID of activated-motor-number: 0x%x", 0x0140);
    LogProducer::info("mc_calib","Send activated-motor-number: %d", joint_num_);
    vector<int> data_of_joint_num;
    data_of_joint_num.push_back(joint_num_);
    int id;
    ErrorCode result = sendConfigData(0x0140, data_of_joint_num);

    if (result != SUCCESS)
    {
        LogProducer::error("mc_calib","Fail to send gear-ratio to barecore, code = 0x%llx", result);
        return result;
    }
    
    usleep(50 * 1000);
    
    // 加载传动比配置文件，并检查数据
    // 检查无误后将数据下发到裸核
    data.clear();
    gear_ratio_param_.loadParamFile(config_file + "group_gear_ratio.yaml");

    if (!gear_ratio_param_.loadParamFile(config_file + "group_gear_ratio.yaml"))
    {
        LogProducer::error("mc_calib","Loading offset param file(group_gear_ratio.yaml) failed");
        return MC_LOAD_PARAM_FAILED;
    }

    if (gear_ratio_param_.getParam("gear_ratio/id", id) && gear_ratio_param_.getParam("gear_ratio/data", data))
    {
        LogProducer::info("mc_calib","ID of gear ratio: 0x%x", id);
        LogProducer::info("mc_calib","Send gear ratio: %s", printDBLine(&data[0], buffer, LOG_TEXT_SIZE));
        result = sendConfigData(id, data);

        if (result == SUCCESS)
        {
            LogProducer::info("mc_calib","Success.");
            usleep(50 * 1000);
        }
        else
        {
            LogProducer::error("mc_calib","Fail to send gear-ratio to barecore, code = 0x%llx", result);
            return result;
        }
    }
    else
    {
        LogProducer::error("mc_calib","Fail to read gear ratio config file, err=0x%llx", result);
        return MC_LOAD_PARAM_FAILED;
    }

    // 加载耦合系数配置文件，并检查数据
    // 检查无误后将数据下发到裸核
    data.clear();
    coupling_param_.loadParamFile(config_file + "group_coupling_coeff.yaml");

    if (!coupling_param_.loadParamFile(config_file + "group_coupling_coeff.yaml"))
    {
        LogProducer::error("mc_calib","Loading coupling coefficient param file(group_coupling_coeff.yaml) failed");
        return MC_LOAD_PARAM_FAILED;
    }

    if (coupling_param_.getParam("coupling_coeff/id", id) && coupling_param_.getParam("coupling_coeff/data", data))
    {
        LogProducer::info("mc_calib","ID of coupling coefficient: 0x%x", id);
        LogProducer::info("mc_calib","Send coupling coefficient:");

        for (size_t i = 0; i < joint_num; i++)
        {
            LogProducer::info("mc_calib","  axis-%d: %s", i, printDBLine(&data[0] + 8 * i, buffer, LOG_TEXT_SIZE));
        }
        
        result = sendConfigData(id, data);

        if (result == SUCCESS)
        {
            LogProducer::info("mc_calib","Success.");
            usleep(50 * 1000);
        }
        else
        {
            LogProducer::error("mc_calib","Fail to send coupling coefficient to barecore, code = 0x%llx", result);
            return result;
        }
    }
    else
    {
        LogProducer::error("mc_calib","Fail to read coupling coefficient config file");
        return MC_LOAD_PARAM_FAILED;
    }

    // 加载零位配置文件，并检查数据
    // 检查无误后将数据下发到裸核，注意只有裸核处于DISABLE状态零位才能生效
    data.clear();
    offset_param_.loadParamFile(config_file + "group_offset.yaml");

    if (!offset_param_.loadParamFile(config_file + "group_offset.yaml"))
    {
        LogProducer::error("mc_calib","Loading offset param file(group_offset.yaml) failed");
        return MC_LOAD_PARAM_FAILED;
    }

    if (offset_param_.getParam("zero_offset/id", id) && offset_param_.getParam("zero_offset/data", data))
    {
        LogProducer::info("mc_calib","ID of offset: 0x%x", id);
        LogProducer::info("mc_calib","Send offset: %s", printDBLine(&data[0], buffer, LOG_TEXT_SIZE));
        result = sendConfigData(id, data);

        if (result == SUCCESS)
        {
            Joint cur_jnt;
            ServoState servo_state;
            uint32_t encoder_state[NUM_OF_JOINT];
            char buffer[LOG_TEXT_SIZE];
            usleep(256 * 1000);
            bare_core_ptr_->getLatestJoint(cur_jnt, encoder_state, servo_state);
            memcpy(zero_offset_, &data[0], joint_num_ * sizeof(data[0]));
            LogProducer::info("mc_calib","Current joint: %s", printDBLine(&cur_jnt[0], buffer, LOG_TEXT_SIZE));
            usleep(50 * 1000);
        }
        else
        {
            LogProducer::error("mc_calib","Fail to send offset to barecore, code = 0x%llx", result);
            return result;
        }
    }
    else
    {
        LogProducer::error("mc_calib","Fail to read zero offset config file");
        return MC_LOAD_PARAM_FAILED;
    }
    */
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
    // for(size_t i = 0; i < joint_num_; i++)
    // {
    //     LogProducer::info("mc_calib","encoder_state 0 bit, axis: %d, %d", i,encoder_state[i]&1);
    //     LogProducer::info("mc_calib","encoder_state 4 bit, axis: %d, %d", i,(encoder_state[i]>>4)&1);
    // }

    // 当前各关节位置和记录文件中各关节位置进行比对
    OffsetState state[NUM_OF_JOINT];
    checkOffset(cur_jnt, old_jnt, state, encoder_state);
    // LogProducer::info("mc_calib","Curr-state: %s", printDBLine((int*)state, buffer, LOG_TEXT_SIZE));

    // for (size_t i = 0; i < joint_num_; i++)
    // {
    //     OffsetState tmp_state = (encoder_err[i] == -1 || encoder_err[i] == -3) ? OFFSET_LOST : OFFSET_NORMAL;
    //     state[i] = (state[i] >= tmp_state) ? state[i] : tmp_state;
    // }

    // LogProducer::info("mc_calib","Encoder-err: %s", printDBLine(&encoder_err[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_calib","New-state: %s", printDBLine((int*)state, buffer, LOG_TEXT_SIZE));

    bool recorder_need_update = false;

    // 比对结果比记录文件中的状态标志更严重，则更新记录文件中的标志
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

    // for (size_t i = joint_num_; i < NUM_OF_JOINT; i++)
    // {
    //     state[i] = OFFSET_LOST;
    // }

    // LogProducer::info("mc_calib","Offset-state: %s", printDBLine((int*)state, buffer, LOG_TEXT_SIZE));

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

    // bool limited = false;
    // bool forbidden = false;

    // // 综合各轴校验结果和零位错误屏蔽标志，给出允许运行标志
    //         // 函数？？？
    // for (size_t i = 0; i < joint_num_; i++)
    // {
    //     if (offset_mask_[i] == OFFSET_UNMASK)
    //     {
    //         if (state[i] != OFFSET_NORMAL /*&& state[i] != OFFSET_INVALID*/)
    //         {
    //             forbidden = true;
    //         }
    //     }
    //     else
    //     {
    //         limited = true;
    //     }
    // }

    // current_state_ = forbidden ? MOTION_FORBIDDEN : (limited ? MOTION_LIMITED : MOTION_NORMAL);
    checkCalibrateState();
    cali_stat = current_state_;
    memcpy(offset_stat, offset_stat_, sizeof(offset_stat));
    LogProducer::info("mc_calib","Done, calibrate motion-state is %d", current_state_);
    return SUCCESS;
}

//------------------------------------------------------------------------------
// 方法：  checkOffset
// 摘要：  对各个轴的当前读数和记录值进行比对，根据偏移门限和丢失门限给出比对结果；
//------------------------------------------------------------------------------
void Calibrator::checkOffset(Joint curr_jnt, Joint last_jnt, OffsetState *offset_stat, const uint32_t (&encoder_state)[NUM_OF_JOINT])
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
// 方法：  calibrateOffset
// 摘要：  在当前位置，对所有轴进行零位标定；
//        注意只有在机器人处于完全静止的状态下才能进行零位标定，否则标定结果不具有任何意义；
//------------------------------------------------------------------------------
ErrorCode Calibrator::calibrateOffset(double *new_offset)
{
    LogProducer::info("mc_calib","Calibrate offset of all joints.");
    size_t indexs[NUM_OF_JOINT] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    return calibrateOffset(indexs, joint_num_, new_offset);
}

//------------------------------------------------------------------------------
// 方法：  calibrateOffset
// 摘要：  在当前位置，对某一个轴进行零位标定；
//        注意只有在机器人处于完全静止的状态下才能进行零位标定，否则标定结果不具有任何意义；
//------------------------------------------------------------------------------
ErrorCode Calibrator::calibrateOffset(size_t index, double *new_offset)
{
    LogProducer::info("mc_calib","Calibrate offset of joint %d", index);
    return calibrateOffset(&index, 1, new_offset);
}

//------------------------------------------------------------------------------
// 方法：  calibrateOffset
// 摘要：  在当前位置，对某几个轴进行零位标定；
//        注意只有在机器人处于完全静止的状态下才能进行零位标定，否则标定结果不具有任何意义；
//------------------------------------------------------------------------------
ErrorCode Calibrator::calibrateOffset(const size_t *pindex, uint32_t length, double *new_offset)
{
    char buf[64];
    string str = "Calibrate offset of joint:";

    // 检查需要标定的轴标号
    for (uint32_t i = 0; i < length; i++)
    {
        snprintf(buf, sizeof(buf), " %d", pindex[i]);
        str += buf;

        if (pindex[i] >= joint_num_)
        {
            LogProducer::error("mc_calib",str.c_str());
            return INVALID_AXIS_ID;
        }
    }

    LogProducer::info("mc_calib",str.c_str());
    Joint cur_joint;
    ServoState servo_state;
    vector<double> cur_offset;
    uint32_t encoder_state[NUM_OF_JOINT];
    vector<int> cur_encoder;
    cur_encoder.resize(joint_num_);

    if (!bare_core_ptr_->getLatestJoint(cur_joint, encoder_state, servo_state) || !bare_core_ptr_->getEncoder(cur_encoder) || getOffsetFromBareCore(cur_offset) != SUCCESS)
    {
        LogProducer::error("mc_calib","Fail to get current offset, joint or encoder state from Core1");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }

    for (uint32_t i = 0; i < joint_num_; i++)
    {
        LogProducer::info("mc_calib","Encoder %d: roll is 0x%x, pulse is 0x%x", i, (cur_encoder[i] >> 16) & 0xFFFF, cur_encoder[i] & 0xFFFF);
    }

    for (uint32_t i = 0; i < joint_num_; i++)
    {
        new_offset[i] = zero_offset_[i];
    }

    for (uint32_t i = joint_num_; i < NUM_OF_JOINT; i++)
    {
        new_offset[i] = 0;
    }

    char buffer[LOG_TEXT_SIZE];
    LogProducer::info("mc_calib","Current-offset: %s", printDBLine(&cur_joint[0], buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_calib","Current-joint:  %s", printDBLine(&cur_offset[0], buffer, LOG_TEXT_SIZE));

    // 计算新的零位值，暂存直到saveOffset方法被调用，新的零位值生效并被写入配置文件中
    for (uint32_t i = 0; i < length; i++)
    {
        if (((encoder_state[pindex[i]] >> 4) & 0x1) != 0)
        {
            LogProducer::info("mc_calib","Lost communication to encoder of axis %d, can not calculate new offset", pindex[i]);
        }
        else
        {
            new_offset[pindex[i]] = calculateOffset(cur_offset[pindex[i]], cur_joint[pindex[i]], 0);
        }
    }

    LogProducer::info("mc_calib","New-offset: %s", printDBLine(new_offset, buffer, LOG_TEXT_SIZE));
    return SUCCESS;
}

//------------------------------------------------------------------------------
// 方法：  calculateOffset
// 摘要：  通过在当前的零位条件下的关节读数和在新零位条件下的目标读数计算零位；
//        公式：编码器读数 = （旧读数 + 旧零位） * 转换系数 = （新读数 + 新零位） * 转换系数
//------------------------------------------------------------------------------
double Calibrator::calculateOffset(double current_offset, double current_joint, double target_joint)
{
    double new_offset = current_joint + current_offset - target_joint;
    LogProducer::info("mc_calib","Current-offset=%.6f, current-joint=%.6f, target-joint=%.6f, new-offset=%.6f", current_offset, current_joint, target_joint, new_offset);
    return new_offset;
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

    // 更新配置文件
    vector<double> data;
    LogProducer::info("mc_calib","Update config file ...");

    if (!offset_param_.getParam("zero_offset/data", data))
    {
        LogProducer::error("mc_calib","Fail to get offset from config file;");
        return MC_LOAD_PARAM_FAILED;
    }

    char buffer[LOG_TEXT_SIZE];
    LogProducer::info("mc_calib","Old offset: %s", printDBLine(&data[0], buffer, LOG_TEXT_SIZE));
    data[index] = offset;
    LogProducer::info("mc_calib","New offset: %s", printDBLine(&data[0], buffer, LOG_TEXT_SIZE));

    if (!offset_param_.setParam("zero_offset/data", data) || !offset_param_.dumpParamFile(AXIS_GROUP_DIR"group_offset.yaml"))
    {
        LogProducer::error("mc_calib","Fail to save offset");
        return MC_LOAD_PARAM_FAILED;
    }

    // 更新裸核零位
    LogProducer::info("mc_calib","Offset saved, update offset in core1 ...");
    ErrorCode err = sendOffsetToBareCore();

    if (err != SUCCESS)
    {
        LogProducer::error("mc_calib","Cannot update offset in core1, code = 0x%llx", err);
        return err;
    }

    // 此处usleep为了等待裸核的零位生效，考虑是否有更好的实现方式
    usleep(200 * 1000);
    LogProducer::info("mc_calib","Core1 offset updated, update recorder ...");

    // Joint cur_joint, old_joint;
    // ServoState  state;
    // uint32_t encoder_state[NUM_OF_JOINT];

    OffsetMask  offset_mask[NUM_OF_JOINT];
    OffsetState offset_stat[NUM_OF_JOINT];

    // 更新记录文件，清除零位丢失标志和错误屏蔽标志
    if (!bare_core_ptr_->getLatestJoint(cur_joint, encoder_state, state))
    {
        LogProducer::error("mc_calib","Fail to get current joint from core1.");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }

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

    offset_stat[index] = OFFSET_NORMAL;
    offset_mask[index] = OFFSET_UNMASK;
    old_joint[index] = cur_joint[index];

    LogProducer::info("mc_calib","  flag = %s", printDBLine((int*)offset_stat, buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_calib","  mask = %s", printDBLine((int*)offset_mask, buffer, LOG_TEXT_SIZE));
    LogProducer::info("mc_calib","  joint = %s", printDBLine(&old_joint[0], buffer, LOG_TEXT_SIZE));

    if (writeOffsetJoint(old_joint) != SUCCESS || writeOffsetState(offset_stat) != SUCCESS || writeOffsetMask(offset_mask) != SUCCESS)
    {
        LogProducer::error("mc_calib","Fail to update recorder in nvram.");
        return MC_OPERATE_NVRAM_FAILED;
    }

    zero_offset_[index] = offset;
    offset_mask_[index] = OFFSET_UNMASK;
    offset_stat_[index] = OFFSET_NORMAL;
    LogProducer::info("mc_calib","Success!");
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

    if (!bare_core_ptr_->getLatestJoint(cur_jnt, encoder_state, state))
    {
        LogProducer::error("mc_calib","Fail to get current joint from share memory");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }

    for (size_t i = 0; i < joint_num_; i++)
    {
        // LogProducer::info("mc_calib","saveJoint encoder_state , axis: %d, %d", i, encoder_state[i]);
        // LogProducer::info("mc_calib","saveJoint encoder_state 0 bit, axis: %d, %d", i, encoder_state[i]&1);
        // LogProducer::info("mc_calib","saveJoint encoder_state 4 bit, axis: %d, %d", i, (encoder_state[i]>>4)&1);
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

    for (size_t i = 0; i < joint_num_; ++i)
    {
        if (((encoder_state[i]) & 0x1) != 0)
        {
            if (!b_log_flag_[i])
            {
                LogProducer::info("mc_calib","encoder battery unnormal axis: %d", i);
                b_log_flag_[i] = true;
            }   
            cur_jnt[i] = read_nvram_jnt[i];
        }
        else
        {
            if (b_log_flag_[i])
            {
                LogProducer::info("mc_calib","encoder battery recovery axis: %d", i);
                b_log_flag_[i] = false;
            }   
        }

        if (((encoder_state[i] >> 4) & 0x1) != 0)
        {
            if (i_com_flag_[i] == NORMAL)
            {
                LogProducer::info("mc_calib","encoder communication unnormal axis: %d", i);
            }           
            i_com_flag_[i] = UNNORMAL;
            cur_jnt[i] = read_nvram_jnt[i];
        }
        else if (i_com_flag_[i] == UNNORMAL)
        {
            LogProducer::info("mc_calib","encoder communication recovery axis: %d", i);
            i_com_flag_[i] = RECOVERY;
        }
    }

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

    // LogProducer::info("mc_calib","write Offset Joint success");

    //char buffer[LOG_TEXT_SIZE];
    //LogProducer::info("mc_calib","Save current joint into record file: %s", printDBLine(&cur_jnt.j1_, buffer, LOG_TEXT_SIZE));
    //LogProducer::info("mc_calib","Success.");
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

    LogProducer::info("mc_calib","Saving mask flags ...");
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

//------------------------------------------------------------------------------
// 方法：  isReferenceAvailable
// 摘要：  获取快速标定是否可用的标志，如果之前标记过参考点，可通过快速标定接口进行标定；
//------------------------------------------------------------------------------
bool Calibrator::isReferenceAvailable(void)
{
    base_space::YamlHelp params;

    if (params.loadParamFile(AXIS_GROUP_DIR"arm_group_offset_reference.yaml"))
    {
        bool is_available = false;

        if (params.getParam("reference_available", is_available))
        {
            LogProducer::info("mc_calib","Reference point is %s", (is_available ? "available" : "inavailable"));
            return is_available;
        }
        else
        {
            LogProducer::error("mc_calib","Fail to get item from config file");
            return false;
        }
    }
    else
    {
        LogProducer::error("mc_calib","Fail to load config file(arm_group_offset_reference.yaml)");
        return false;
    }
}

//------------------------------------------------------------------------------
// 方法：  deleteReference
// 摘要：  删除用于快速标定的记录参考点；
//------------------------------------------------------------------------------
ErrorCode Calibrator::deleteReference(void)
{
    LogProducer::info("mc_calib","Delete reference point.");
    base_space::YamlHelp params;

    if (params.loadParamFile(AXIS_GROUP_DIR"arm_group_offset_reference.yaml"))
    {
        bool is_available = false;

        if (params.getParam("reference_available", is_available))
        {
            if (is_available)
            {
                is_available = false;

                if (params.setParam("reference_available", is_available) && params.dumpParamFile(AXIS_GROUP_DIR"arm_group_offset_reference.yaml"))
                {
                    LogProducer::info("mc_calib","Inactive reference point success.");
                    return SUCCESS;
                }
                else
                {
                    LogProducer::error("mc_calib","Error while modifying reference point");
                    return MC_LOAD_PARAM_FAILED;
                }
            }
            else
            {
                LogProducer::info("mc_calib","Reference point is inavailable, nothing to do.");
                return SUCCESS;
            }
        }
        else
        {
            LogProducer::error("mc_calib","Fail to get item from config file");
            return MC_LOAD_PARAM_FAILED;
        }
    }
    else
    {
        LogProducer::error("mc_calib","Fail to load config file(arm_group_offset_reference.yaml)");
        return MC_LOAD_PARAM_FAILED;
    }
}

//------------------------------------------------------------------------------
// 方法：  saveReference
// 摘要：  将当前位置记录为快速标定参考点，标定过参考点的机器人可通过快速标定对零位进行标定；
//------------------------------------------------------------------------------
ErrorCode Calibrator::saveReference(void)
{
    Joint joint;
    ServoState state;
    uint32_t encoder_state[NUM_OF_JOINT];
    char buffer[LOG_TEXT_SIZE];

    LogProducer::info("mc_calib","Save referenece point");
    base_space::YamlHelp params;

    if (!params.loadParamFile(AXIS_GROUP_DIR"arm_group_offset_reference.yaml"))
    {
        LogProducer::error("mc_calib","Error while load reference file(arm_group_offset_reference.yaml)");
        return MC_LOAD_PARAM_FAILED;
    }

    if (!bare_core_ptr_->getLatestJoint(joint, encoder_state, state))
    {
        LogProducer::error("mc_calib","Fail to get current joint from bare core");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }

    vector<double> ref_joint(&joint[0], &joint[0] + joint_num_);
    LogProducer::info("mc_calib","  joint:  %s", printDBLine(&ref_joint[0], buffer, LOG_TEXT_SIZE));

    vector<double> ref_offset;
    ErrorCode err = getOffsetFromBareCore(ref_offset);

    if (err != SUCCESS)
    {
        LogProducer::error("mc_calib","Fail to get current offset from bare core, code = 0x%llx", err);
        return err;
    }

    LogProducer::info("mc_calib","  offset: %s", printDBLine(&ref_offset[0], buffer, LOG_TEXT_SIZE));

    vector<int> ref_encoder;
    ref_encoder.resize(joint_num_);

    if(!bare_core_ptr_->getEncoder(ref_encoder))
    {
        LogProducer::error("mc_calib","Fail to get current encoder from bare core");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }

    LogProducer::info("mc_calib","  encoder: %s", printDBLine(&ref_encoder[0], buffer, LOG_TEXT_SIZE));

    if (!params.setParam("reference_joint", ref_joint) || !params.setParam("reference_offset", ref_offset) ||
        !params.setParam("reference_encoder", ref_encoder) || !params.setParam("reference_available", true) ||
        !params.dumpParamFile(AXIS_GROUP_DIR"arm_group_offset_reference.yaml"))
    {
        LogProducer::error("mc_calib","Error while saving reference point");
        return MC_SET_PARAM_FAILED;
    }

    LogProducer::info("mc_calib","Success.");
    return SUCCESS;
}

//------------------------------------------------------------------------------
// 方法：  fastCalibrate
// 摘要：  快速标定所有轴，注意标定完成后需要调用saveOffset进行保存；
//------------------------------------------------------------------------------
ErrorCode Calibrator::fastCalibrate(void)
{
    LogProducer::info("mc_calib","Easy calibrate of all joints.");
    size_t indexs[NUM_OF_JOINT] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    return fastCalibrate(indexs, joint_num_);
}

//------------------------------------------------------------------------------
// 方法：  fastCalibrate
// 摘要：  对某个轴进行快速标定，注意标定完成后需要调用saveOffset进行保存；
//------------------------------------------------------------------------------
ErrorCode Calibrator::fastCalibrate(size_t index)
{
    LogProducer::info("mc_calib","Fsst calibrate index = %d", index);
    return fastCalibrate(&index, 1);
}

//------------------------------------------------------------------------------
// 方法：  fastCalibrate
// 摘要：  对某几个轴进行快速标定，注意标定完成后需要调用saveOffset进行保存；
//------------------------------------------------------------------------------
ErrorCode Calibrator::fastCalibrate(const size_t *pindex, size_t length)
{
    LogProducer::info("mc_calib","Fast calibrate offset");
    return INVALID_PARAMETER;//NOT USED.

    for (size_t i = 0; i < length; i++)
    {
        if (pindex[i] < joint_num_)
        {
            LogProducer::info("mc_calib","  joint index = %d", pindex[i]);
        }
        else
        {
            LogProducer::error("mc_calib","  joint index = %d, index out of range", pindex[i]);
            return INVALID_AXIS_ID;
        }
    }

    vector<int>     ref_encoder;
    vector<double>  ref_offset;
    vector<double>  gear_ratio;

    base_space::YamlHelp params;
    base_space::YamlHelp jtac;

    if (params.loadParamFile(AXIS_GROUP_DIR"arm_group_offset_reference.yaml") && 
        jtac.loadParamFile("share/configuration/machine/jtac.yaml"))
    {
        if (params.getParam("reference_offset", ref_offset) &&
            params.getParam("reference_encoder", ref_encoder) &&
            jtac.getParam("gear_ratio/data", gear_ratio))
        {
            if (ref_offset.size() == joint_num_ && ref_encoder.size() == joint_num_ && gear_ratio.size() == joint_num_)
            {
                char buffer[LOG_TEXT_SIZE];
                vector<int> cur_encoder; cur_encoder.resize(joint_num_);

                LogProducer::info("mc_calib","Reference offset: %s", printDBLine(&ref_offset[0], buffer, LOG_TEXT_SIZE));
                LogProducer::info("mc_calib","Reference encoder: %s", printDBLine(&ref_encoder[0], buffer, LOG_TEXT_SIZE));
                LogProducer::info("mc_calib","Gear ratio: %s", printDBLine(&gear_ratio[0], buffer, LOG_TEXT_SIZE));

                if (bare_core_ptr_->getEncoder(cur_encoder))
                {
                    size_t index;
                    LogProducer::info("mc_calib","Current encoder: %s", printDBLine(&cur_encoder[0], buffer, LOG_TEXT_SIZE));

                    Joint cur_joint;
                    ServoState  state;
                    uint32_t encoder_state[NUM_OF_JOINT];
                    if (!bare_core_ptr_->getLatestJoint(cur_joint, encoder_state, state))
                    {
                        LogProducer::error("mc_calib","Fail to get current joint from core1.");
                        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
                    }

                    for (size_t i = 0; i < length; i++)
                    {
                        index = pindex[i];
                        if(((encoder_state[index]>>4)&1) != 0)
                        {
                            LogProducer::info("mc_calib","encoder communication unnormal,axis: %d, do not fast calculate & need save Offset",index+1);
                            //offset_need_save_[index] = UNNEED_SAVE;
                        }
                        else
                        {
                            zero_offset_[index] = calculateOffsetEasy(gear_ratio[index], ref_offset[index], ref_encoder[index], cur_encoder[index]);
                            //offset_need_save_[index] = NEED_SAVE;
                        }
                    }

                    LogProducer::info("mc_calib","New offset: %s", printDBLine(zero_offset_, buffer, LOG_TEXT_SIZE));
                    return SUCCESS;
                }
                else
                {
                    LogProducer::error("mc_calib","Fail to get current encoders.");
                    return MC_COMMUNICATION_WITH_BARECORE_FAIL;
                }
            }
            else
            {
                if (ref_offset.size() != joint_num_)
                    LogProducer::error("mc_calib","Invalid array size of reference offset, size is %d but %d wanted.", ref_offset.size(), joint_num_);
                else if (ref_encoder.size() != joint_num_)
                    LogProducer::error("mc_calib","Invalid array size of reference encoder, size is %d but %d wanted.", ref_encoder.size(), joint_num_);
                else
                    LogProducer::error("mc_calib","Invalid array size of gear ratio, size is %d but %d wanted.", gear_ratio.size(), joint_num_);

                return INVALID_PARAMETER;
            }
        }
        else
        {
            return MC_LOAD_PARAM_FAILED;
        }
    }
    else
    {
        return MC_LOAD_PARAM_FAILED;
    }
}

ErrorCode Calibrator::resetEncoderMultiTurnValue()
{
    LogProducer::info("mc_calib","reset encoder multi-turn value of all joints.");

    if (!bare_core_ptr_->resetEncoderError())
    {
        LogProducer::error("mc_calib","Fail to reset encoder errors.");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }

    usleep(250 * 1000);

    LogProducer::info("mc_calib","Zero loss flag set");
    for (size_t i = 0; i < joint_num_; i++)
    {
        offset_stat_[i] = OFFSET_LOST;
    }

    ErrorCode err = writeOffsetState(offset_stat_);

    return err;
}

//------------------------------------------------------------------------------
// 方法：  calculateOffsetEasy
// 摘要：  根据记录中的参考点和当前编码器读数计算新零位；
//------------------------------------------------------------------------------
double Calibrator::calculateOffsetEasy(double gear_ratio, double ref_offset,
                                       unsigned int ref_encoder, unsigned int cur_encoder)
{
    LogProducer::info("mc_calib","Reference-offset = %.6f, reference-encoder = 0x%x, current-encoder = 0x%x, gear-ratio = %.6f",
             ref_offset, ref_encoder, cur_encoder, gear_ratio);

    double new_offset;
    unsigned int cur_rolls = (cur_encoder >> 16) & 0xFFFF;
    unsigned int ref_rolls = (ref_encoder >> 16) & 0xFFFF;

    if (cur_rolls > ref_rolls)
    {
        new_offset = ref_offset + PI * 2 * (cur_rolls - ref_rolls) / gear_ratio;
    }
    else if (cur_rolls < ref_rolls)
    {
        new_offset = ref_offset - PI * 2 * (ref_rolls - cur_rolls) / gear_ratio;
    }
    else
    {
        new_offset = ref_offset;
    }

    LogProducer::info("mc_calib","Reference-rolls = 0x%x, current-rolls = 0x%x, new-offset = %.6f", ref_rolls, cur_rolls, new_offset);
    return new_offset;
}

//------------------------------------------------------------------------------
// 方法：  sendConfigData
// 摘要：  使用service形式向裸核发送参数id和数据；
//------------------------------------------------------------------------------
ErrorCode Calibrator::sendConfigData(int id, const vector<double> &data)
{
    return bare_core_ptr_->setConfigData(id, data) ? SUCCESS : MC_COMMUNICATION_WITH_BARECORE_FAIL;
}

//------------------------------------------------------------------------------
// 方法：  sendConfigData
// 摘要：  使用service形式向裸核发送参数id和数据；
//------------------------------------------------------------------------------
ErrorCode Calibrator::sendConfigData(int id, const vector<int> &data)
{
    return bare_core_ptr_->setConfigData(id, data) ? SUCCESS : MC_COMMUNICATION_WITH_BARECORE_FAIL;
}

//------------------------------------------------------------------------------
// 方法：  sendOffsetToBareCore
// 摘要：  从配置文件中提取零位参数，并使用service形式向裸核发送这些参数；
//------------------------------------------------------------------------------
ErrorCode Calibrator::sendOffsetToBareCore(void)
{
    int id;
    vector<double> data;

    if (offset_param_.getParam("zero_offset/id", id) && offset_param_.getParam("zero_offset/data", data))
    {
        return sendConfigData(id, data);
    }
    else
    {
        return MC_LOAD_PARAM_FAILED;
    }
}

//------------------------------------------------------------------------------
// 方法：  getOffsetFromBareCore
// 摘要：  通过service形式，从裸核获取零位；
//------------------------------------------------------------------------------
ErrorCode Calibrator::getOffsetFromBareCore(vector<double> &data)
{
    int id;

    if (offset_param_.getParam("zero_offset/id", id) && offset_param_.getParam("zero_offset/data", data))
    {
        return bare_core_ptr_->getConfigData(id, data) ? SUCCESS : MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }
    else
    {
        data.clear();
        return MC_LOAD_PARAM_FAILED;
    }
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
