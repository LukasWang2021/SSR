/*************************************************************************
	> File Name: motion_control_offset_calibrator.cpp
	> Author: 
	> Mail: 
	> Created Time: 2016年12月12日 星期一 14时30分05秒
 ************************************************************************/

#include <fstream>
#include <math.h>
#include <boost/filesystem.hpp>
#include <motion_control_offset_calibrator.h>
#include <common_file_path.h>

#define OFFSET_STATE_ADDR       (ZERO_BLK + 0x00)
#define OFFSET_MASK_ADDR        (ZERO_BLK + 0x10)
#define OFFSET_JOINT_ADDR       (ZERO_BLK + 0x20)

using namespace std;
using namespace basic_alg;
using namespace fst_parameter;

namespace fst_mc
{

Calibrator::Calibrator(void)
{
    log_ptr_ = NULL;
    joint_num_ = 0;
    nvram_ptr_ = NULL;
    bare_core_ptr_ = NULL;
    current_state_ = MOTION_FORBIDDEN;
    b_check_flag_ = false;
    memset(i_com_flag_, NORMAL, NUM_OF_JOINT * sizeof(int));
    memset(offset_need_save_, 0, NUM_OF_JOINT * sizeof(int));
    memset(normal_threshold_, 0, NUM_OF_JOINT * sizeof(double));
    memset(lost_threshold_, 0, NUM_OF_JOINT * sizeof(double));
    memset(zero_offset_, 0, NUM_OF_JOINT * sizeof(double));
    memset(offset_mask_, 0, NUM_OF_JOINT * sizeof(OffsetMask));
    memset(offset_stat_, 0, NUM_OF_JOINT * sizeof(OffsetState));
}

Calibrator::~Calibrator(void)
{
    if (nvram_ptr_ != NULL)
    {
        delete nvram_ptr_;
        nvram_ptr_ = NULL;
    }
}

//------------------------------------------------------------------------------
// 方法：  initCalibrator
// 摘要：  初始化标定模块，未被初始化过的标定模块不能用于标定，此时调用标定模块任何方法得到的结果
//        都是不可预知的，所以确保在标定模块开始工作之前调用此方法进行初始化配置；
//        主要包括加载配置文件和记录文件，并向裸核发送零位配置；
//------------------------------------------------------------------------------
ErrorCode Calibrator::initCalibrator(size_t joint_num, BareCoreInterface *pcore, fst_log::Logger *plog)
{
    // 输入参数检查
    if (joint_num > 0 && joint_num <= NUM_OF_JOINT && plog && pcore)
    {
        log_ptr_ = plog;
        joint_num_ = joint_num;
        bare_core_ptr_ = pcore;
    }
    else
    {
        FST_INFO("initCalibrator: invalid parameter, joint-number = %d, bare-core-ptr = %p, log-ptr = %p", joint_num, pcore, plog);
        return MC_FAIL_IN_INIT;
    }

    int id;
    ErrorCode result;
    vector<int> stat;
    vector<double> data;
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Initializing offset calibrator, number-of-joint = %d.", joint_num_);
    nvram_ptr_ = new Nvram(0x100);

    if (nvram_ptr_ == NULL)
    {
        FST_INFO("Fail to create nvram.");
        return MC_INTERNAL_FAULT;
    }

    ErrorCode err = nvram_ptr_->openNvram();

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to open NvRam, code = 0x%llx.", err);
        return err;
    }

    err = nvram_ptr_->isNvramReady();

    if (err != SUCCESS)
    {
        FST_ERROR("NvRam is not ready, code = 0x%llx.", err);
        return err;
    }

    err = readOffsetState(offset_stat_);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to read offset state, code = 0x%llx", err);
        return err;
    }

    FST_INFO("Offset state: %s", printDBLine((int*)offset_stat_, buffer, LOG_TEXT_SIZE));

    err = readOffsetMask(offset_mask_);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to read offset mask, code = 0x%llx", err);
        return err;
    }

    FST_INFO("Offset mask: %s", printDBLine((int*)offset_mask_, buffer, LOG_TEXT_SIZE));

    Joint offset_joint;
    err = readOffsetJoint(offset_joint);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to read offset joint, code = 0x%llx", err);
        return err;
    }

    FST_INFO("Offset joint: %s", printDBLine(&offset_joint[0], buffer, LOG_TEXT_SIZE));

    // 加载零位偏移门限值和零位丢失门限值，并检查数据是否合法
    string config_file = AXIS_GROUP_DIR;
    ParamGroup params(config_file + "base_group.yaml");

    if (params.getLastError() != SUCCESS)
    {
        result = params.getLastError();
        FST_ERROR("Fail to load thresholds config file, err=0x%llx", result);
        return result;
    }

    data.clear();
    params.getParam("calibrator/normal_offset_threshold", data);

    if (params.getLastError() != SUCCESS)
    {
        result = params.getLastError();
        FST_ERROR("Fail to read threshold from config file, err=0x%llx", result);
        return result;
    }

    if (data.size() != joint_num_)
    {
        FST_ERROR("Invalid array size of normal threshold, except %d but get %d", joint_num_, data.size());
        return INVALID_PARAMETER;
    }
    
    FST_INFO("Threshold-normal: %s", printDBLine(&data[0], buffer, LOG_TEXT_SIZE));

    for (size_t i = 0; i < joint_num_; i++)
        normal_threshold_[i] = data[i];

    data.clear();
    params.getParam("calibrator/lost_offset_threshold", data);

    if (data.size() != joint_num_)
    {
        FST_ERROR("Invalid array size of lost threshold, except %d but get %d", joint_num_, data.size());
        return INVALID_PARAMETER;
    }

    FST_INFO("Threshold-lost: %s", printDBLine(&data[0], buffer, LOG_TEXT_SIZE));

    for (size_t i = 0; i < joint_num_; i++)
        lost_threshold_[i] = data[i];
    
    // 下发关节数量到裸核
    FST_INFO("ID of activated-motor-number: 0x%x", 0x0140);
    FST_INFO("Send activated-motor-number: %d", joint_num_);
    vector<int> data_of_joint_num;
    data_of_joint_num.push_back(joint_num_);
    result = sendConfigData(0x0140, data_of_joint_num);

    if (result != SUCCESS)
    {
        FST_ERROR("Fail to send gear-ratio to barecore, code = 0x%llx", result);
        return result;
    }
    
    FST_INFO("Success.");
    usleep(50 * 1000);
    
    // 加载传动比配置文件，并检查数据
    // 检查无误后将数据下发到裸核
    data.clear();
    gear_ratio_param_.loadParamFile(config_file + "group_gear_ratio.yaml");
    result = gear_ratio_param_.getLastError();

    if (result != SUCCESS)
    {
        FST_ERROR("Loading offset param file failed, err=0x%llx", result);
        return result;
    }

    if (gear_ratio_param_.getParam("gear_ratio/id", id) && gear_ratio_param_.getParam("gear_ratio/data", data))
    {
        FST_INFO("ID of gear ratio: 0x%x", id);
        FST_INFO("Send gear ratio: %s", printDBLine(&data[0], buffer, LOG_TEXT_SIZE));
        result = sendConfigData(id, data);

        if (result == SUCCESS)
        {
            FST_INFO("Success.");
            usleep(50 * 1000);
        }
        else
        {
            FST_ERROR("Fail to send gear-ratio to barecore, code = 0x%llx", result);
            return result;
        }
    }
    else
    {
        result = gear_ratio_param_.getLastError();
        FST_ERROR("Fail to read gear ratio config file, err=0x%llx", result);
        return result;
    }

    // 加载耦合系数配置文件，并检查数据
    // 检查无误后将数据下发到裸核
    data.clear();
    coupling_param_.loadParamFile(config_file + "group_coupling_coeff.yaml");
    result = coupling_param_.getLastError();

    if (result != SUCCESS)
    {
        FST_ERROR("Loading coupling coefficient param file failed, err=0x%llx", result);
        return result;
    }

    if (coupling_param_.getParam("coupling_coeff/id", id) && coupling_param_.getParam("coupling_coeff/data", data))
    {
        FST_INFO("ID of coupling coefficient: 0x%x", id);
        FST_INFO("Send coupling coefficient:");

        for (size_t i = 0; i < joint_num; i++)
        {
            FST_INFO("  axis-%d: %s", i, printDBLine(&data[0] + 8 * i, buffer, LOG_TEXT_SIZE));
        }
        
        result = sendConfigData(id, data);

        if (result == SUCCESS)
        {
            FST_INFO("Success.");
            usleep(50 * 1000);
        }
        else
        {
            FST_ERROR("Fail to send coupling coefficient to barecore, code = 0x%llx", result);
            return result;
        }
    }
    else
    {
        result = coupling_param_.getLastError();
        FST_ERROR("Fail to read coupling coefficient config file, err=0x%llx", result);
        return result;
    }

    // 加载零位配置文件，并检查数据
    // 检查无误后将数据下发到裸核，注意只有裸核处于DISABLE状态零位才能生效
    data.clear();
    offset_param_.loadParamFile(config_file + "group_offset.yaml");
    result = offset_param_.getLastError();

    if (result != SUCCESS)
    {
        FST_ERROR("Loading offset param file failed, err=0x%llx", result);
        return result;
    }

    if (offset_param_.getParam("zero_offset/id", id) && offset_param_.getParam("zero_offset/data", data))
    {
        FST_INFO("ID of offset: 0x%x", id);
        FST_INFO("Send offset: %s", printDBLine(&data[0], buffer, LOG_TEXT_SIZE));
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
            FST_INFO("Current joint: %s", printDBLine(&cur_jnt[0], buffer, LOG_TEXT_SIZE));
            FST_INFO("Success.");
            usleep(50 * 1000);
        }
        else
        {
            FST_ERROR("Fail to send offset to barecore, code = 0x%llx", result);
            return result;
        }
    }
    else
    {
        result = offset_param_.getLastError();
        FST_ERROR("Fail to read zero offset config file, err=0x%llx", result);
        return result;
    }
    
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
    // vector<int> encoder_err;
    // encoder_err.resize(joint_num_);
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Check zero offset.");

    // // 获取编码器错误标志
    // if (!bare_core_ptr_->getEncoderError(encoder_err))
    // {
    //     current_state_ = MOTION_LIMITED;
    //     cali_stat = current_state_;
    //     memcpy(offset_stat, offset_stat_, sizeof(offset_stat));
    //     FST_ERROR("Fail to get encoder error from bare core, code = 0x%llx", MC_COMMUNICATION_WITH_BARECORE_FAIL);
    //     return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    // }

    // 获取NvRam中各关节的位置
    ErrorCode err = readOffsetJoint(old_jnt);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to get offset joint from NvRam, code = 0x%llx.", err);
        current_state_ = MOTION_FORBIDDEN;
        cali_stat = current_state_;
        memcpy(offset_stat, offset_stat_, sizeof(offset_stat));
        return err;
    }

    // 获取当前各关节的位置
    if (!bare_core_ptr_->getLatestJoint(cur_jnt, encoder_state, servo_state))
    {
        FST_ERROR("Fail to get current joint from share mem."); 
        current_state_ = MOTION_FORBIDDEN;
        cali_stat = current_state_;
        memcpy(offset_stat, offset_stat_, sizeof(offset_stat));
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }

    FST_INFO("Curr-joint: %s", printDBLine(&cur_jnt[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Last-joint: %s", printDBLine(&old_jnt[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Mask-flags: %s", printDBLine((int*)offset_mask_, buffer, LOG_TEXT_SIZE));
    FST_INFO("Last-state: %s", printDBLine((int*)offset_stat_, buffer, LOG_TEXT_SIZE));
    FST_INFO("encoder_state: %s", printDBLine((int*)encoder_state, buffer, LOG_TEXT_SIZE));
    // for(size_t i = 0; i < joint_num_; i++)
    // {
    //     FST_INFO("encoder_state 0 bit, axis: %d, %d", i+1,encoder_state[i]&1);
    //     FST_INFO("encoder_state 4 bit, axis: %d, %d", i+1,(encoder_state[i]>>4)&1);
    // }

    // 当前各关节位置和记录文件中各关节位置进行比对
    OffsetState state[NUM_OF_JOINT];
    checkOffset(cur_jnt, old_jnt, state, encoder_state);
    // FST_INFO("Curr-state: %s", printDBLine((int*)state, buffer, LOG_TEXT_SIZE));

    // for (size_t i = 0; i < joint_num_; i++)
    // {
    //     OffsetState tmp_state = (encoder_err[i] == -1 || encoder_err[i] == -3) ? OFFSET_LOST : OFFSET_NORMAL;
    //     state[i] = (state[i] >= tmp_state) ? state[i] : tmp_state;
    // }

    // FST_INFO("Encoder-err: %s", printDBLine(&encoder_err[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("New-state: %s", printDBLine((int*)state, buffer, LOG_TEXT_SIZE));

    // bool recorder_need_update = false;

    // 比对结果比记录文件中的状态标志更严重，则更新记录文件中的标志
    for (size_t i = 0; i < joint_num_; i++)
    {
        if (offset_mask_[i] == OFFSET_UNMASK /*&& state[i] > offset_stat_[i]*/)
        {
            // recorder_need_update = true;
            offset_stat_[i] = state[i];
        }
        // else
        // {
        //     state[i] = offset_stat_[i];
        // }
    }

    for (size_t i = joint_num_; i < NUM_OF_JOINT; i++)
    {
        offset_stat_[i] = OFFSET_LOST;
    }

    FST_INFO("Offset-state: %s", printDBLine((int*)offset_stat_, buffer, LOG_TEXT_SIZE));

    // for (size_t i = joint_num_; i < NUM_OF_JOINT; i++)
    // {
    //     state[i] = OFFSET_LOST;
    // }

    // FST_INFO("Offset-state: %s", printDBLine((int*)state, buffer, LOG_TEXT_SIZE));

    // if (recorder_need_update)
    // {
    //     // 更新记录文件中的零位标志
    //     FST_INFO("Update offset state into recorder.");
    //     err = writeOffsetState(state);

    //     if (err != SUCCESS)
    //     {
    //         FST_ERROR("Failed, code = 0x%llx", err);
    //         current_state_ = MOTION_FORBIDDEN;
    //         cali_stat = current_state_;
    //         memcpy(offset_stat, offset_stat_, sizeof(offset_stat));
    //         return err;
    //     }

    //     for (size_t i = 0; i < joint_num_; i++)
    //     {
    //         offset_stat_[i] = state[i];
    //     }
    // }

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
    FST_INFO("Done, calibrate motion-state is %d", current_state_);
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
        FST_INFO("encoder_state 0 bit, axis: %d, state: %d", i+1,encoder_state[i]&1);
        FST_INFO("encoder_state 4 bit, axis: %d, state: %d", i+1,(encoder_state[i]>>4)&1);
        if(((encoder_state[i]>>4)&1) != 0)
        {
            FST_INFO("checkOffset encoder communication unnormal, axis: %d", i+1);
            offset_stat[i] = OFFSET_INVALID;
        }
        else
        {
            if(((encoder_state[i])&1) != 0)
            {
                FST_INFO("checkOffset encoder battery unnormal, axis: %d", i+1);
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
    //FST_INFO("Check Calibrate State.");   
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
    //FST_INFO("Done check Calibrate State, calibrate motion-state is %d", current_state_);   
}

//------------------------------------------------------------------------------
// 方法：  calibrateOffset
// 摘要：  在当前位置，对所有轴进行零位标定；
//        注意只有在机器人处于完全静止的状态下才能进行零位标定，否则标定结果不具有任何意义；
//------------------------------------------------------------------------------
ErrorCode Calibrator::calibrateOffset(void)
{
    FST_INFO("Calibrate offset of all joints.");
    size_t indexs[NUM_OF_JOINT] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    return calibrateOffset(indexs, joint_num_);
}

//------------------------------------------------------------------------------
// 方法：  calibrateOffset
// 摘要：  在当前位置，对某一个轴进行零位标定；
//        注意只有在机器人处于完全静止的状态下才能进行零位标定，否则标定结果不具有任何意义；
//------------------------------------------------------------------------------
ErrorCode Calibrator::calibrateOffset(size_t index)
{
    FST_INFO("Calibrate offset of joint index=%d", index);
    return calibrateOffset(&index, 1);
}

//------------------------------------------------------------------------------
// 方法：  calibrateOffset
// 摘要：  在当前位置，对某几个轴进行零位标定；
//        注意只有在机器人处于完全静止的状态下才能进行零位标定，否则标定结果不具有任何意义；
//------------------------------------------------------------------------------
ErrorCode Calibrator::calibrateOffset(const size_t *pindex, size_t length)
{
    FST_INFO("Calibrate offset of:");

    // 检查需要标定的轴标号
    for (size_t i = 0; i < length; i++)
    {
        if (pindex[i] < joint_num_)
        {
            FST_INFO("  joint index=%d", pindex[i]);
            FST_INFO("Reset encoder error flags and rolls ...");
            if (!bare_core_ptr_->resetEncoderError(pindex[i]))
            {
                FST_ERROR("Fail to reset encoder errors.");
                return MC_COMMUNICATION_WITH_BARECORE_FAIL;
            }
        }
        else
        {
            FST_ERROR("  joint index=%d, index out of range", pindex[i]);
            return INVALID_PARAMETER;
        }
    }

    usleep(250 * 1000);

    Joint cur_joint;
    ServoState servo_state;
    vector<double> cur_offset;
    uint32_t encoder_state[NUM_OF_JOINT];
    char buffer[LOG_TEXT_SIZE];

    if (bare_core_ptr_->getLatestJoint(cur_joint, encoder_state, servo_state) && getOffsetFromBareCore(cur_offset) == SUCCESS)
    {
        vector<int> cur_encoder;
        cur_encoder.resize(joint_num_);

        if(bare_core_ptr_->getEncoder(cur_encoder))
        {
            FST_INFO("Current encoder: %s", printDBLine(&cur_encoder[0], buffer, LOG_TEXT_SIZE));

            for (size_t i = 0; i < joint_num_; i++)
            {
                int roll = (cur_encoder[i] >> 16) & 0xFFFF;
                int pulse = cur_encoder[i] & 0xFFFF;
                FST_INFO("Encoder %d: roll is 0x%x, pulse is 0x%x", i, roll, pulse);
            }
        }
        else
        {
            FST_WARN("Fail to get current encoder from bare core");
        }

        FST_INFO("Current-offset: %s", printDBLine(&cur_joint[0], buffer, LOG_TEXT_SIZE));
        FST_INFO("Current-joint:  %s", printDBLine(&cur_offset[0], buffer, LOG_TEXT_SIZE));

        size_t index;

        // 计算新的零位值，暂存直到saveOffset方法被调用，新的零位值生效并被写入配置文件中
        for (size_t i = 0; i < length; i++)
        {
            index = pindex[i];
            if(((encoder_state[index]>>4)&1) != 0)
            {
                FST_INFO("encoder communication unnormal,axis: %d, do not calculate & need save Offset",index+1);
                offset_need_save_[index] = UNNEED_SAVE;
            }
            else
            {
                zero_offset_[index] = calculateOffset(cur_offset[index], cur_joint[index], 0);
                offset_need_save_[index] = NEED_SAVE;
            }
        }

        FST_INFO("New-offset: %s", printDBLine(&zero_offset_[0], buffer, LOG_TEXT_SIZE));
        // 由于已经清了多圈值和编码器电池错误，标定和保存必须在一个操作之内完成
        return saveOffset();
    }
    else
    {
        FST_ERROR("Fail to get current offset or current joint from Core1");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }
}

//------------------------------------------------------------------------------
// 方法：  calculateOffset
// 摘要：  通过在当前的零位条件下的关节读数和在新零位条件下的目标读数计算零位；
//        公式：编码器读数 = （旧读数 + 旧零位） * 转换系数 = （新读数 + 新零位） * 转换系数
//------------------------------------------------------------------------------
double Calibrator::calculateOffset(double current_offset, double current_joint, double target_joint)
{
    double new_offset = current_joint + current_offset - target_joint;
    FST_INFO("Current-offset=%.6f, current-joint=%.6f, target-joint=%.6f, new-offset=%.6f", current_offset, current_joint, target_joint, new_offset);
    return new_offset;
}

//------------------------------------------------------------------------------
// 方法：  saveOffset
// 摘要：  将之前计算出并暂存的零位值下发到裸核并写入配置文件；
//------------------------------------------------------------------------------
ErrorCode Calibrator::saveOffset(void)
{
    FST_INFO("Save zero offset into config file ...");
    //vector<double> data(zero_offset_, zero_offset_ + NUM_OF_JOINT);
    Joint cur_joint, old_joint;
    ServoState  state;
    uint32_t encoder_state[NUM_OF_JOINT];
    if (!bare_core_ptr_->getLatestJoint(cur_joint, encoder_state, state))
    {
        FST_ERROR("Fail to get current joint from core1.");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }

    for (size_t i = 0; i < joint_num_; i++)
    {
        if(((encoder_state[i]>>4)&1) != 0)
        {
            FST_INFO("encoder communication unnormal, save Offset set UNNEED_SAVE, axis: %d",i+1);
            offset_need_save_[i] = UNNEED_SAVE;
        }
    }

    bool need_save = false;

    for (size_t i = 0; i < joint_num_; i++)
    {
        if (offset_need_save_[i] == NEED_SAVE)
        {
            need_save = true;
            break;
        }
    }

    // 至少有一个轴的零位尚未生效和保存才需要更新零位文件，否则直接返回
    if (!need_save)
    {
        FST_WARN("None offset need to save.");
        return SUCCESS;
    }

    // 更新配置文件
    vector<double> data;

    if (!offset_param_.getParam("zero_offset/data", data))
    {
        FST_ERROR("Fail to get offset from config file;");
        return offset_param_.getLastError();
    }

    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Old offset: %s", printDBLine(&data[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("New offset: %s", printDBLine(&zero_offset_[0], buffer, LOG_TEXT_SIZE));
    FST_INFO("Need save: %s", printDBLine(offset_need_save_, buffer, LOG_TEXT_SIZE));

    for (size_t i = 0; i < joint_num_; i++)
    {
        if(((encoder_state[i]>>4)&1) != 0)
        {
             FST_INFO("encoder communication unnormal, do not update zero_offset Param, axis: %d",i+1);
        }
        else
        {
            if (offset_need_save_[i] == NEED_SAVE)
            data[i] = zero_offset_[i];
        }
    }

    if (!offset_param_.setParam("zero_offset/data", data) || !offset_param_.dumpParamFile())
    {
        FST_ERROR("Fail to save offset, err=0x%llx", offset_param_.getLastError());
        return offset_param_.getLastError();
    }

    // 更新裸核零位
    FST_INFO("Offset saved, update offset in core1 ...");
    ErrorCode err = sendOffsetToBareCore();

    if (err != SUCCESS)
    {
        FST_ERROR("Cannot update offset in core1, code = 0x%llx", err);
        return err;
    }

    // 此处usleep为了等待裸核的零位生效，考虑是否有更好的实现方式
    usleep(200 * 1000);
    FST_INFO("Core1 offset updated, update recorder ...");

    // Joint cur_joint, old_joint;
    // ServoState  state;
    // uint32_t encoder_state[NUM_OF_JOINT];

    OffsetMask  offset_mask[NUM_OF_JOINT];
    OffsetState offset_stat[NUM_OF_JOINT];

    // 更新记录文件，清除零位丢失标志和错误屏蔽标志
    if (!bare_core_ptr_->getLatestJoint(cur_joint, encoder_state, state))
    {
        FST_ERROR("Fail to get current joint from core1.");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }

    FST_INFO("Current joint: %s", printDBLine(&cur_joint[0], buffer, LOG_TEXT_SIZE));

    err = readOffsetJoint(old_joint);
    
    if (err != SUCCESS)
    {
        FST_ERROR("Fail to get last joint from nvram.");
        return err;
    }

    for (size_t i = 0; i < joint_num_; i++)
    {
        offset_mask[i] = offset_mask_[i];
        offset_stat[i] = offset_stat_[i];

        // 对于更新了零位的轴，同时更新其记录关节读数、零位状态标志和错误屏蔽标志
        if (offset_need_save_[i] == NEED_SAVE)
        {
            old_joint[i] = cur_joint[i];
            offset_stat[i] = OFFSET_NORMAL;
            offset_mask[i] = OFFSET_UNMASK;
        }
    }

    for (size_t i = joint_num_; i < NUM_OF_JOINT; i++)
    {
        offset_mask[i] = OFFSET_UNMASK;
        offset_stat[i] = OFFSET_LOST;
    }

    FST_INFO("  flag = %s", printDBLine((int*)offset_stat, buffer, LOG_TEXT_SIZE));
    FST_INFO("  mask = %s", printDBLine((int*)offset_mask, buffer, LOG_TEXT_SIZE));
    FST_INFO("  joint = %s", printDBLine(&old_joint[0], buffer, LOG_TEXT_SIZE));

    if (writeOffsetJoint(old_joint) != SUCCESS || writeOffsetState(offset_stat) != SUCCESS || writeOffsetMask(offset_mask) != SUCCESS)
    {
        FST_ERROR("Fail to update recorder.");
        return FST_NVRAM_R_TIMEOUT_F;
    }

    /*
    FST_INFO("Reset encoder error flags ...");

    if (!bare_core_ptr_->resetEncoderError())
    {
        FST_ERROR("Fail to reset encoder errors.");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }
    */

    for (size_t i = 0; i < joint_num_; i++)
    {
        if (offset_need_save_[i] == NEED_SAVE)
        {
            offset_need_save_[i] = UNNEED_SAVE;
            offset_mask_[i] = offset_mask[i];
            offset_stat_[i] = offset_stat[i];
        }
    }

    FST_INFO("Success!");
    return SUCCESS;
}

//------------------------------------------------------------------------------
// 方法：  saveJoint
// 摘要：  将当前的关节读数写入记录文件，注意只有在机器人完全静止后才可调用；
//------------------------------------------------------------------------------
ErrorCode Calibrator::saveJoint(void)
{
    Joint cur_jnt;
    ServoState state;
    memset(&cur_jnt, 0, sizeof(Joint));
    Joint read_nvram_jnt;
    memset(&read_nvram_jnt, 0, sizeof(Joint));
    uint32_t encoder_state[NUM_OF_JOINT];

    ErrorCode err = readOffsetJoint(read_nvram_jnt);;
    if (err != SUCCESS)
    {
        FST_ERROR("Fail to get offset joint from NvRam, code = 0x%llx.", err);
        return err;
    }

    if (!bare_core_ptr_->getLatestJoint(cur_jnt, encoder_state, state))
    {
        FST_ERROR("Fail to get current joint from share memory");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }

    for (size_t i = 0; i < joint_num_; i++)
    {
        // FST_INFO("saveJoint encoder_state , axis: %d, %d", i+1,encoder_state[i]);
        // FST_INFO("saveJoint encoder_state 0 bit, axis: %d, %d", i+1,encoder_state[i]&1);
        // FST_INFO("saveJoint encoder_state 4 bit, axis: %d, %d", i+1,(encoder_state[i]>>4)&1);
        if(((encoder_state[i])&1) != 0)
        {
            // FST_ERROR("encoder battery unnormal, axis: %d", i+1);
            offset_stat_[i] = OFFSET_LOST;
        }

        if(((encoder_state[i]>>4)&1) != 0)
        {
            // FST_ERROR("encoder communication unnormal, axis: %d", i+1);
            offset_stat_[i] = OFFSET_INVALID;
        }
    }

    for(size_t i = 0; i < joint_num_; ++i)
    {
        if(((encoder_state[i])&1) != 0)
        {
            FST_INFO("encoder battery unnormal axis: %d", i+1);
            cur_jnt[i] = read_nvram_jnt[i];
        }
        if(((encoder_state[i]>>4)&1) != 0)
        {
            FST_INFO("encoder communication unnormal axis: %d", i+1);
            i_com_flag_[i] = UNNORMAL;
            cur_jnt[i] = read_nvram_jnt[i];
        }
        else if(i_com_flag_[i] == UNNORMAL)
        {
            FST_INFO("encoder communication recovery axis: %d", i+1);
            i_com_flag_[i] = RECOVERY;
        }
    }

    for(size_t i = 0; i < joint_num_; ++i)
    {
        if(i_com_flag_[i] == RECOVERY)
        {
            i_com_flag_[i] = NORMAL;
            b_check_flag_ = true;
        }
    }

    if(b_check_flag_)
    {
        b_check_flag_ = false;
        CalibrateState calibrate_stat;
        OffsetState offset_stat[NUM_OF_JOINT];
        return checkOffset(calibrate_stat, offset_stat);  
    }

    for (size_t i = 0; i < joint_num_; i++)
    {
        if(offset_stat_[i] != OFFSET_NORMAL)
        {
            FST_ERROR("offset unormal axis: %d",i+1);
            cur_jnt[i] = read_nvram_jnt[i];
        }
    }
    
    err = writeOffsetJoint(cur_jnt);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to record joint, code = 0x%llx", err);
        return err;
    }

    // FST_INFO("write Offset Joint success");

    //char buffer[LOG_TEXT_SIZE];
    //FST_INFO("Save current joint into record file: %s", printDBLine(&cur_jnt.j1_, buffer, LOG_TEXT_SIZE));
    //FST_INFO("Success.");
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
    OffsetMask  mask[NUM_OF_JOINT];

    FST_INFO("Mask all lost-offset errors.");
    FST_INFO("  mask flag: %s", printDBLine((int*)offset_mask_, buffer, LOG_TEXT_SIZE));
    FST_INFO("  offset state: %s", printDBLine((int*)offset_stat_, buffer, LOG_TEXT_SIZE));

    bool need_save = false;

    for (size_t i = 0; i < joint_num_; i++)
    {
        mask[i] = offset_mask_[i];

        if (offset_stat_[i] == OFFSET_LOST)
        {
            FST_INFO("    index = %d, lost its offset, we'll try to mask this error.", i);
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
        FST_INFO("No lost-offset error need mask");
        return SUCCESS;
    }

    FST_INFO("Saving mask flags ...");
    ErrorCode err = writeOffsetMask(mask);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to save mask flags into recorder, code = 0x%llx", err);
        return err;
    }

    for (size_t i = 0; i < joint_num_; i++)
    {
        if (offset_stat_[i] == OFFSET_LOST)
        {
            offset_mask_[i] = mask[i];
        }
    }

    FST_INFO("All lost-offset errors masked.");
    return SUCCESS;
}

//------------------------------------------------------------------------------
// 方法：  setOffsetState
// 摘要：  当某些轴发生零位偏移时，允许用户对这些轴进行设置，将其状态归为零位正常或者零位丢失；
//------------------------------------------------------------------------------
ErrorCode Calibrator::setOffsetState(size_t index, OffsetState stat)
{
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Set offset state, index = %d, state = %d", index, stat);

    if (index >= joint_num_)
    {
        FST_ERROR("Invalid index.");
        return INVALID_PARAMETER;
    }

    Joint old_joint;
    bool update_joint = false;
    OffsetState stats[NUM_OF_JOINT];
    memcpy(stats, offset_stat_, sizeof(stats));
    FST_INFO("Offset state: %s", printDBLine((int*)stats, buffer, LOG_TEXT_SIZE));
    stats[index] = stat;
    FST_INFO("  changed to: %s", printDBLine((int*)stats, buffer, LOG_TEXT_SIZE));

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
        FST_ERROR("Fail to get last joint from recorder, code = 0x%llx", err);
        return err;
    }

    if (!bare_core_ptr_->getLatestJoint(cur_joint, encoder_state, servo_state))
    {
        FST_ERROR("Fail to get current joint from bare core.");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }

    if(((encoder_state[index]>>4)&1) != 0)
    {
        FST_INFO("encoder communication unnormal, do not set Offset State.");
        return ZERO_OFFSET_INVALID;
    }

    if (update_joint)
    {
        old_joint[index] = cur_joint[index];
    }

    if (writeOffsetState(stats) != SUCCESS || writeOffsetJoint(old_joint) != SUCCESS)
    {
        FST_ERROR("Fail to set offset flag into recorder");
        return FST_NVRAM_R_TIMEOUT_F;
    }

    offset_stat_[index] = stats[index];
    FST_INFO("Success.");
    return SUCCESS;
}

//------------------------------------------------------------------------------
// 方法：  setOffset
// 摘要：  允许用户直接设置某个轴的零位值，被改变的零位值需要调用saveOffset借口才会被写入配置文件中；
//------------------------------------------------------------------------------
ErrorCode Calibrator::setOffset(size_t index, double offset)
{
    FST_INFO("Set offset, index = %d, offset = %.6f", index, offset);

    if (index >= joint_num_)
    {
        FST_ERROR("Given index out of range, joint-number is %d", joint_num_);
        return INVALID_PARAMETER;
    }
    
    FST_INFO("Old-offset: %.6f, new-offset: %.6f", zero_offset_[index], offset);

    Joint cur_joint;
    ServoState  state;
    uint32_t encoder_state[NUM_OF_JOINT];
    if (!bare_core_ptr_->getLatestJoint(cur_joint, encoder_state, state))
    {
        FST_ERROR("Fail to get current joint from core1.");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }

    if(((encoder_state[index]>>4)&1) != 0)
    {
        FST_ERROR("encoder communication unnormal, axis: %d ,setOffset set UNNEED_SAVE",index);
        offset_need_save_[index] = UNNEED_SAVE;
        return ZERO_OFFSET_INVALID;
    }
    else
    {
        zero_offset_[index] = offset;
        offset_need_save_[index] = NEED_SAVE;
    }

    FST_INFO("Done.");
    return SUCCESS;
}

//------------------------------------------------------------------------------
// 方法：  setOffset
// 摘要：  允许用户直接设置所有轴的零位值，被改变的零位值需要调用saveOffset借口才会被写入配置文件中；
//        注意确保传入的offset数组长度不小于有效轴数
//------------------------------------------------------------------------------
ErrorCode Calibrator::setOffset(const double *offset)
{
    char buf[LOG_TEXT_SIZE];
    FST_INFO("Set offset, new-offset: %s", printDBLine(offset, buf, LOG_TEXT_SIZE));

    Joint cur_joint;
    ServoState  state;
    uint32_t encoder_state[NUM_OF_JOINT];
    if (!bare_core_ptr_->getLatestJoint(cur_joint, encoder_state, state))
    {
        FST_ERROR("Fail to get current joint from core1.");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }
    
    for (size_t i = 0; i < joint_num_; i++)
    {
        if(((encoder_state[i]>>4)&1) != 0)
        {
            FST_ERROR("encoder communication unnormal, axis: %d ,setOffset set UNNEED_SAVE",i);
            offset_need_save_[i] = UNNEED_SAVE;
            return ZERO_OFFSET_INVALID;
        }
        else
        {
            zero_offset_[i] = offset[i];
            offset_need_save_[i] = NEED_SAVE;
        }
    }

    return SUCCESS;
}

//------------------------------------------------------------------------------
// 方法：  getOffset
// 摘要：  获取所有轴的零位值，注意确保数组长度不小于实际轴数；
//------------------------------------------------------------------------------
void Calibrator::getOffset(double *offset)
{
    bool need_save = false;
    char buffer[LOG_TEXT_SIZE];

    for (size_t i = 0; i < joint_num_; i++)
    {
        offset[i] = zero_offset_[i];
        need_save = need_save || (offset_need_save_[i] == NEED_SAVE);
    }

    FST_INFO("Get latest offset: %s", printDBLine(offset, buffer, LOG_TEXT_SIZE));

    if (need_save)
    {
        FST_WARN("One or more offset have not been saved.");
    }
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

    /*
    for (size_t i = joint_num_; i < NUM_OF_JOINT; i++)
    {
        mask[i] = OFFSET_UNMASK;
    }
    */
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
    ParamGroup params(AXIS_GROUP_DIR"arm_group_offset_reference.yaml");

    if (params.getLastError() == SUCCESS)
    {
        bool is_available = false;

        if (params.getParam("reference_available", is_available))
        {
            FST_INFO("Reference point is %s", (is_available ? "available" : "inavailable"));
            return is_available;
        }
        else
        {
            FST_ERROR("Fail to get item from config file, error code = 0x%llx", params.getLastError());
            return false;
        }
    }
    else
    {
        FST_ERROR("Fail to load config file, error code = 0x%llx", params.getLastError());
        return false;
    }
}

//------------------------------------------------------------------------------
// 方法：  deleteReference
// 摘要：  删除用于快速标定的记录参考点；
//------------------------------------------------------------------------------
ErrorCode Calibrator::deleteReference(void)
{
    FST_INFO("Delete reference point.");
    ParamGroup params(AXIS_GROUP_DIR"arm_group_offset_reference.yaml");

    if (params.getLastError() == SUCCESS)
    {
        bool is_available = false;

        if (params.getParam("reference_available", is_available))
        {
            if (is_available)
            {
                is_available = false;

                if (params.setParam("reference_available", is_available) && params.dumpParamFile())
                {
                    FST_INFO("Inactive reference point success.");
                    return SUCCESS;
                }
                else
                {
                    FST_ERROR("Error while modifying reference point, err=0x%llx", params.getLastError());
                    return params.getLastError();
                }
            }
            else
            {
                FST_INFO("Reference point is inavailable, nothing to do.");
                return SUCCESS;
            }
        }
        else
        {
            FST_ERROR("Fail to get item from config file, error code = 0x%llx", params.getLastError());
            return params.getLastError();
        }
    }
    else
    {
        FST_ERROR("Fail to load config file, error code = 0x%llx", params.getLastError());
        return params.getLastError();
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

    FST_INFO("Save referenece point");
    ParamGroup params(AXIS_GROUP_DIR"arm_group_offset_reference.yaml");

    if (params.getLastError() != SUCCESS)
    {
        FST_ERROR("Error while load reference file, err=0x%llx", params.getLastError());
        return params.getLastError();
    }

    if (!bare_core_ptr_->getLatestJoint(joint, encoder_state, state))
    {
        FST_ERROR("Fail to get current joint from bare core");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }

    vector<double> ref_joint(&joint[0], &joint[0] + joint_num_);
    FST_INFO("  joint:  %s", printDBLine(&ref_joint[0], buffer, LOG_TEXT_SIZE));

    vector<double> ref_offset;
    ErrorCode err = getOffsetFromBareCore(ref_offset);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to get current offset from bare core, code = 0x%llx", err);
        return err;
    }

    FST_INFO("  offset: %s", printDBLine(&ref_offset[0], buffer, LOG_TEXT_SIZE));

    vector<int> ref_encoder;
    ref_encoder.resize(joint_num_);

    if(!bare_core_ptr_->getEncoder(ref_encoder))
    {
        FST_ERROR("Fail to get current encoder from bare core");
        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
    }

    FST_INFO("  encoder: %s", printDBLine(&ref_encoder[0], buffer, LOG_TEXT_SIZE));

    if (!params.setParam("reference_joint", ref_joint) || !params.setParam("reference_offset", ref_offset) ||
        !params.setParam("reference_encoder", ref_encoder) || !params.setParam("reference_available", true) ||
        !params.dumpParamFile())
    {
        FST_ERROR("Error while saving reference point, err=0x%llx", params.getLastError());
        return params.getLastError();
    }

    FST_INFO("Success.");
    return SUCCESS;
}

//------------------------------------------------------------------------------
// 方法：  fastCalibrate
// 摘要：  快速标定所有轴，注意标定完成后需要调用saveOffset进行保存；
//------------------------------------------------------------------------------
ErrorCode Calibrator::fastCalibrate(void)
{
    FST_INFO("Easy calibrate of all joints.");
    size_t indexs[NUM_OF_JOINT] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    return fastCalibrate(indexs, joint_num_);
}

//------------------------------------------------------------------------------
// 方法：  fastCalibrate
// 摘要：  对某个轴进行快速标定，注意标定完成后需要调用saveOffset进行保存；
//------------------------------------------------------------------------------
ErrorCode Calibrator::fastCalibrate(size_t index)
{
    FST_INFO("Fsst calibrate index = %d", index);
    return fastCalibrate(&index, 1);
}

//------------------------------------------------------------------------------
// 方法：  fastCalibrate
// 摘要：  对某几个轴进行快速标定，注意标定完成后需要调用saveOffset进行保存；
//------------------------------------------------------------------------------
ErrorCode Calibrator::fastCalibrate(const size_t *pindex, size_t length)
{
    FST_INFO("Fsst calibrate offset of:");

    for (size_t i = 0; i < length; i++)
    {
        if (pindex[i] < joint_num_)
        {
            FST_INFO("  joint index = %d", pindex[i]);
        }
        else
        {
            FST_ERROR("  joint index = %d, index out of range", pindex[i]);
            return INVALID_PARAMETER;
        }
    }

    vector<int>     ref_encoder;
    vector<double>  ref_offset;
    vector<double>  gear_ratio;

    ParamGroup params(AXIS_GROUP_DIR"arm_group_offset_reference.yaml");
    ParamGroup jtac("share/configuration/machine/jtac.yaml");

    if (params.getLastError() == SUCCESS && jtac.getLastError() == SUCCESS)
    {
        if (params.getParam("reference_offset", ref_offset) &&
            params.getParam("reference_encoder", ref_encoder) &&
            jtac.getParam("gear_ratio/data", gear_ratio))
        {
            if (ref_offset.size() == joint_num_ && ref_encoder.size() == joint_num_ && gear_ratio.size() == joint_num_)
            {
                char buffer[LOG_TEXT_SIZE];
                vector<int> cur_encoder; cur_encoder.resize(joint_num_);

                FST_INFO("Reference offset: %s", printDBLine(&ref_offset[0], buffer, LOG_TEXT_SIZE));
                FST_INFO("Reference encoder: %s", printDBLine(&ref_encoder[0], buffer, LOG_TEXT_SIZE));
                FST_INFO("Gear ratio: %s", printDBLine(&gear_ratio[0], buffer, LOG_TEXT_SIZE));

                if (bare_core_ptr_->getEncoder(cur_encoder))
                {
                    size_t index;
                    FST_INFO("Current encoder: %s", printDBLine(&cur_encoder[0], buffer, LOG_TEXT_SIZE));

                    Joint cur_joint;
                    ServoState  state;
                    uint32_t encoder_state[NUM_OF_JOINT];
                    if (!bare_core_ptr_->getLatestJoint(cur_joint, encoder_state, state))
                    {
                        FST_ERROR("Fail to get current joint from core1.");
                        return MC_COMMUNICATION_WITH_BARECORE_FAIL;
                    }

                    for (size_t i = 0; i < length; i++)
                    {
                        index = pindex[i];
                        if(((encoder_state[index]>>4)&1) != 0)
                        {
                            FST_INFO("encoder communication unnormal,axis: %d, do not fast calculate & need save Offset",index+1);
                            offset_need_save_[index] = UNNEED_SAVE;
                        }
                        else
                        {
                            zero_offset_[index] = calculateOffsetEasy(gear_ratio[index], ref_offset[index], ref_encoder[index], cur_encoder[index]);
                            offset_need_save_[index] = NEED_SAVE;
                        }
                    }

                    FST_INFO("New offset: %s", printDBLine(zero_offset_, buffer, LOG_TEXT_SIZE));
                    return SUCCESS;
                }
                else
                {
                    FST_ERROR("Fail to get current encoders.");
                    return MC_COMMUNICATION_WITH_BARECORE_FAIL;
                }
            }
            else
            {
                if (ref_offset.size() != joint_num_)
                    FST_ERROR("Invalid array size of reference offset, size is %d but %d wanted.", ref_offset.size(), joint_num_);
                else if (ref_encoder.size() != joint_num_)
                    FST_ERROR("Invalid array size of reference encoder, size is %d but %d wanted.", ref_encoder.size(), joint_num_);
                else
                    FST_ERROR("Invalid array size of gear ratio, size is %d but %d wanted.", gear_ratio.size(), joint_num_);

                return INVALID_PARAMETER;
            }
        }
        else
        {
            if (params.getLastError() != SUCCESS)
            {
                FST_ERROR("Fail to get reference point from config file");
                return params.getLastError();
            }
            else
            {
                FST_ERROR("Fail to get gear ratio from config file");
                return jtac.getLastError();
            }
        }
    }
    else
    {
        if (params.getLastError() != SUCCESS)
        {
            FST_ERROR("Fail to open reference file");
            return params.getLastError();
        }
        else {
            FST_ERROR("Fail to open jtac config file");
            return jtac.getLastError();
        }
    }
}

//------------------------------------------------------------------------------
// 方法：  calculateOffsetEasy
// 摘要：  根据记录中的参考点和当前编码器读数计算新零位；
//------------------------------------------------------------------------------
double Calibrator::calculateOffsetEasy(double gear_ratio, double ref_offset,
                                       unsigned int ref_encoder, unsigned int cur_encoder)
{
    FST_INFO("Reference-offset = %.6f, reference-encoder = 0x%x, current-encoder = 0x%x, gear-ratio = %.6f",
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

    FST_INFO("Reference-rolls = 0x%x, current-rolls = 0x%x, new-offset = %.6f", ref_rolls, cur_rolls, new_offset);
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
        return offset_param_.getLastError();
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
        return offset_param_.getLastError();
    }
}


ErrorCode Calibrator::readNvRam(void *array, uint32_t addr, uint32_t length)
{
    uint8_t nvram_data1[ZERO_BLK_LEN];
    uint8_t nvram_data2[ZERO_BLK_LEN];

    ErrorCode err = nvram_ptr_->read(nvram_data1, NVRAM_BLK_1_BASE + addr, length + 1);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to read BLK1 in NvRam, code = 0x%llx.", err);
        return err;
    }

    err = nvram_ptr_->read(nvram_data2, NVRAM_BLK_2_BASE + addr, length + 1);

    if (err != SUCCESS)
    {
        FST_ERROR("Fail to read BLK2 in NvRam, code = 0x%llx.", err);
        return err;
    }

    uint8_t checksum1 = nvram_ptr_->checkSumOfData(nvram_data1, length + 1);
    uint8_t checksum2 = nvram_ptr_->checkSumOfData(nvram_data2, length + 1);

    if (checksum1 != 0 && checksum2 != 0)
    {
        FST_ERROR("Data in NvRam is lost.");
        return MC_NVRAM_DATA_INVALID;
    }

    if (checksum1 != 0)
    {
        FST_WARN("Data in blank-1 is lost, rebuild from blank-2.");
        nvram_ptr_->write(nvram_data2, NVRAM_BLK_1_BASE + addr, length + 1);
        memcpy(nvram_data1, nvram_data2, length + 1);
        usleep(200 * 1000);
    }
    else if (checksum2 != 0)
    {
        FST_WARN("Data in blank-2 is lost, rebuild from blank-1.");
        nvram_ptr_->write(nvram_data1, NVRAM_BLK_2_BASE + addr, length + 1);
        memcpy(nvram_data2, nvram_data1, length + 1);
        usleep(200 * 1000);
    }

    bool data_is_same = true;

    for (size_t i = 0; i < length; i++)
    {
        if (nvram_data1[i] != nvram_data2[i])
        {
            data_is_same = false;
            break;
        }
    }

    if (!data_is_same)
    {
        FST_WARN("Data in blank-1 different with data in blank-2, rebuild bland-2");
        nvram_ptr_->write(nvram_data1, NVRAM_BLK_2_BASE + addr, length + 1);
        memcpy(nvram_data2, nvram_data1, length + 1);
        usleep(200 * 1000);
    }

    memcpy(array, nvram_data1, length);
    return SUCCESS;
}

ErrorCode Calibrator::readOffsetState(OffsetState (&state)[NUM_OF_JOINT])
{
    uint8_t data[NUM_OF_JOINT];
    ErrorCode err = readNvRam(data, OFFSET_STATE_ADDR, NUM_OF_JOINT);

    if (err == SUCCESS)
    {
        for (size_t i = 0; i < NUM_OF_JOINT; i++)
        {
            state[i] = OffsetState(data[i]);
        }
    }
    else if (err == MC_NVRAM_DATA_INVALID)
    {
        FST_WARN("Offset state in NvRam is lost, reset state in NvRam.");
        memset(data, OFFSET_LOST, NUM_OF_JOINT);
        writeNvRam(data, OFFSET_STATE_ADDR, NUM_OF_JOINT);

        for (size_t i = 0; i < NUM_OF_JOINT; i++)
        {
            state[i] = OFFSET_LOST;
        }
    }
    else
    {

        return err;
    }

    return SUCCESS;
}

ErrorCode Calibrator::readOffsetMask(OffsetMask (&mask)[NUM_OF_JOINT])
{
    uint8_t data[NUM_OF_JOINT];
    ErrorCode err = readNvRam(data, OFFSET_MASK_ADDR, NUM_OF_JOINT);

    if (err == SUCCESS)
    {
        for (size_t i = 0; i < NUM_OF_JOINT; i++)
        {
            mask[i] = OffsetMask(data[i]);
        }
    }
    else if (err == MC_NVRAM_DATA_INVALID)
    {
        FST_WARN("Offset mask in NvRam is lost, reset mask in NvRam.");
        memset(data, OFFSET_UNMASK, NUM_OF_JOINT);
        writeNvRam(data, OFFSET_MASK_ADDR, NUM_OF_JOINT);

        for (size_t i = 0; i < NUM_OF_JOINT; i++)
        {
            mask[i] = OFFSET_UNMASK;
        }
    }
    else
    {
        return err;
    }

    return SUCCESS;
}

ErrorCode Calibrator::readOffsetJoint(Joint &joint)
{
    uint8_t data[sizeof(joint)];
    ErrorCode err = readNvRam(data, OFFSET_JOINT_ADDR, sizeof(data));

    if (err == SUCCESS)
    {
        memcpy(&joint, data, sizeof(joint));
    }
    else if (err == MC_NVRAM_DATA_INVALID)
    {
        FST_WARN("Offset joint in NvRam is lost, reset joint in NvRam.");
        memset(data, 0, sizeof(data));
        memset(&joint, 0, sizeof(joint));
        writeNvRam(data, OFFSET_JOINT_ADDR, sizeof(data));
    }
    else
    {
        return err;
    }

    return SUCCESS;
}

ErrorCode Calibrator::writeNvRam(const void *array, uint32_t addr, uint32_t length)
{
    uint8_t data[ZERO_BLK_LEN];
    memcpy(data, array, length);
    data[length] = nvram_ptr_->checkSumOfData(data, length);
    //FST_INFO("writeNvRam: data = %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9]);
    nvram_ptr_->write(data, addr, length + 1);
    usleep(30 * 1000);
    //FST_INFO("write 0x%x, length=%d", NVRAM_BLK_1_BASE + addr, length + 1);

    //memset(data, 0, sizeof(data));
    //nvram_ptr_->read(data, NVRAM_BLK_1_BASE + addr, length + 1);
    //FST_INFO("readNvRam: data = %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9]);
    //memset(data, 0, sizeof(data));
    //nvram_ptr_->read(data, NVRAM_BLK_2_BASE + addr, length + 1);
    //FST_INFO("readNvRam: data = %d, %d, %d, %d, %d, %d, %d, %d, %d, %d", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9]);

    return SUCCESS;
}

ErrorCode Calibrator::writeOffsetState(const OffsetState (&state)[NUM_OF_JOINT])
{
    uint8_t data[NUM_OF_JOINT];

    for (size_t i = 0; i < NUM_OF_JOINT; i++)
    {
        data[i] = state[i];
    }

    return writeNvRam(data, OFFSET_STATE_ADDR, NUM_OF_JOINT);
}

ErrorCode Calibrator::writeOffsetMask(const OffsetMask (&mask)[NUM_OF_JOINT])
{
    uint8_t data[NUM_OF_JOINT];

    for (size_t i = 0; i < NUM_OF_JOINT; i++)
    {
        data[i] = mask[i];
    }

    return writeNvRam(data, OFFSET_MASK_ADDR, NUM_OF_JOINT);
}

ErrorCode Calibrator::writeOffsetJoint(const Joint &joint)
{
    return writeNvRam(&joint, OFFSET_JOINT_ADDR, sizeof(Joint));
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
