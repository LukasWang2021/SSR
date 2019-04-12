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


using namespace std;
using namespace basic_alg;
using namespace fst_parameter;

namespace fst_mc
{

Calibrator::Calibrator(void)
{
    log_ptr_ = NULL;
    joint_num_ = 0;
    bare_core_ptr_ = NULL;
    current_state_ = MOTION_FORBIDDEN;
    memset(offset_need_save_, 0, NUM_OF_JOINT * sizeof(int));
    memset(normal_threshold_, 0, NUM_OF_JOINT * sizeof(double));
    memset(lost_threshold_, 0, NUM_OF_JOINT * sizeof(double));
    memset(zero_offset_, 0, NUM_OF_JOINT * sizeof(double));
    memset(offset_mask_, 0, NUM_OF_JOINT * sizeof(OffsetMask));
    memset(offset_stat_, 0, NUM_OF_JOINT * sizeof(OffsetState));
}

Calibrator::~Calibrator(void)
{}

//------------------------------------------------------------------------------
// 方法：  initCalibrator
// 摘要：  初始化标定模块，未被初始化过的标定模块不能用于标定，此时调用标定模块任何方法得到的结果
//        都是不可预知的，所以确保在标定模块开始工作之前调用此方法进行初始化配置；
//        主要包括加载配置文件和记录文件，并向裸核发送零位配置；
//------------------------------------------------------------------------------
ErrorCode Calibrator::initCalibrator(size_t joint_num, BareCoreInterface *pcore, fst_log::Logger *plog, const string &path)
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
        return MOTION_INTERNAL_FAULT;
    }

    int id;
    ErrorCode result;
    vector<int> stat;
    vector<double> data;
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Initializing offset calibrator, number-of-joint = %d.", joint_num_);

    // 检查path指定的目录是否存在，不存在则创建之
    boost::filesystem::path pa(path);

    if (!boost::filesystem::is_directory(pa))
    {
        FST_WARN("Directory: %s not found", path.c_str());
        if (boost::filesystem::create_directory(pa))
        {
            FST_INFO("Directory: %s created", path.c_str());
        }
        else
        {
            FST_ERROR("Failed to create directory: %s", path.c_str());
            return MOTION_INTERNAL_FAULT;
        }
    }

    // 检查零位记录文件是否存在，不存在则从模板创建一个记录文件，模板在'share/motion_controller/config'
    string record_file =  path + "robot_recorder.yaml";
    boost::filesystem::path fi(record_file);

    if (!boost::filesystem::exists(fi))
    {
        FST_WARN("Record file: %s not found", record_file.c_str());
        if (buildRecordFile(record_file) == SUCCESS)
        {
            FST_INFO("Build record file from template");
        }
        else
        {
            FST_ERROR("Fail to build record file");
            return MOTION_INTERNAL_FAULT;
        }
    }

    // 加载记录文件并检查数据是否合法
    if (!robot_recorder_.loadParamFile(record_file.c_str()))
    {
        result = robot_recorder_.getLastError();
        FST_ERROR("Fail to load record file, err=0x%llx", result);
        return result;
    }

    stat.clear();
    if (!robot_recorder_.getParam("mask", stat))
    {
        result = robot_recorder_.getLastError();
        FST_ERROR("Fail to read joint-mask from record file, err=0x%llx", result);
        return result;
    }
    if (stat.size() != joint_num_)
    {
        FST_ERROR("Invalid array size of joint-mask, except %d but get %d", joint_num_, stat.size());
        return INVALID_PARAMETER;
    }
    FST_INFO("Recorded-Masks: %s", printDBLine(&stat[0], buffer, LOG_TEXT_SIZE));

    for (size_t i = 0; i < joint_num_; i++)
        offset_mask_[i] = OffsetMask(stat[i]);

    stat.clear();
    if (!robot_recorder_.getParam("state", stat))
    {
        result = robot_recorder_.getLastError();
        FST_ERROR("Fail to read joint-flag from record file, err=0x%llx", result);
        return result;
    }
    if (stat.size() != joint_num_)
    {
        FST_ERROR("Invalid array size of joint-flag, except %d but get %d", joint_num_, stat.size());
        return INVALID_PARAMETER;
    }
    FST_INFO("Recorded-Flags: %s", printDBLine(&stat[0], buffer, LOG_TEXT_SIZE));

    for (size_t i = 0; i < joint_num_; i++)
        offset_stat_[i] = OffsetState(stat[i]);

    // 加载零位偏移门限值和零位丢失门限值，并检查数据是否合法
    string config_file = AXIS_GROUP_DIR;
    ParamGroup params(config_file + "base_group.yaml");

    if (params.getLastError() == SUCCESS)
    {
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
    }
    else
    {
        result = params.getLastError();
        FST_ERROR("Fail to load thresholds config file, err=0x%llx", result);
        return result;
    }

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

    if (result == SUCCESS)
    {
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
    }
    else
    {
        FST_ERROR("Loading offset param file failed, err=0x%llx", result);
        return result;
    }

    // 加载耦合系数配置文件，并检查数据
    // 检查无误后将数据下发到裸核
    data.clear();
    coupling_param_.loadParamFile(config_file + "group_coupling_coeff.yaml");
    result = coupling_param_.getLastError();

    if (result == SUCCESS)
    {
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
    }
    else
    {
        FST_ERROR("Loading coupling coefficient param file failed, err=0x%llx", result);
        return result;
    }

    // 加载零位配置文件，并检查数据
    // 检查无误后将数据下发到裸核，注意只有裸核处于DISABLE状态零位才能生效
    data.clear();
    offset_param_.loadParamFile(config_file + "group_offset.yaml");
    result = offset_param_.getLastError();

    if (result == SUCCESS)
    {
        if (offset_param_.getParam("zero_offset/id", id) && offset_param_.getParam("zero_offset/data", data))
        {
            FST_INFO("ID of offset: 0x%x", id);
            FST_INFO("Send offset: %s", printDBLine(&data[0], buffer, LOG_TEXT_SIZE));
            result = sendConfigData(id, data);

            if (result == SUCCESS)
            {
                Joint cur_jnt;
                ServoState servo_state;
                char buffer[LOG_TEXT_SIZE];
                usleep(256 * 1000);
                bare_core_ptr_->getLatestJoint(cur_jnt, servo_state);
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
    }
    else
    {
        FST_ERROR("Loading offset param file failed, err=0x%llx", result);
        return result;
    }

    // 进行一次零位检测
    /*
    CalibrateState calibrate_stat;
    OffsetState offset_stat[NUM_OF_JOINT];
    checkOffset(&calibrate_stat, offset_stat);
    */

    return SUCCESS;
}

//------------------------------------------------------------------------------
// 方法：  buildRecordFile
// 摘要：  如果零位记录文件不存在，从模板生成一个零位记录文件放到初始化时指定的目录之中，正常情况下
//        仅在MC程序第一次在控制器上运行时需要生成零位记录文件；
//------------------------------------------------------------------------------
ErrorCode Calibrator::buildRecordFile(const string &file)
{
    char buf[256] = {0};
    readlink("/proc/self/exe", buf, sizeof(buf));
    boost::filesystem::path executable(buf);
    string temp = executable.parent_path().parent_path().parent_path().string() + "/share/motion_controller/config/robot_recorder.yaml";
    FST_INFO("Rebuild %s from %s", file.c_str(), temp.c_str());
    std::ifstream  in(temp.c_str());
    std::ofstream out(file.c_str());

    if (in.is_open() && out.is_open())
    {
        out << in.rdbuf();
        in.close();
        out.close();
        return SUCCESS;
    }
    else
    {
        return FAIL_OPENNING_FILE;
    }
}

//------------------------------------------------------------------------------
// 方法：  checkOffset
// 摘要：  调用checkOffset进行一次零位校验，给出各轴的零位校验结果和机器人是否允许运动的标志；
//        注意只有在机器人处于完全静止的状态下才能进行零位校验，否则校验结果不具有任何意义；
//------------------------------------------------------------------------------
ErrorCode Calibrator::checkOffset(CalibrateState &cali_stat, OffsetState (&offset_stat)[NUM_OF_JOINT])
{
    ServoState servo_state;
    vector<double> joint;
    Joint cur_jnt, old_jnt;
    char buffer[LOG_TEXT_SIZE];

    FST_INFO("Check zero offset.");

    // 获取当前各关节的位置
    if (bare_core_ptr_->getLatestJoint(cur_jnt, servo_state))
    {
        FST_INFO("Curr-joint: %s", printDBLine(&cur_jnt[0], buffer, LOG_TEXT_SIZE));

        // 获取记录文件中各关节的位置
        if (robot_recorder_.getParam("joint", joint))
        {
            if (joint.size() == joint_num_)
            {
                FST_INFO("Last-joint: %s", printDBLine(&joint[0], buffer, LOG_TEXT_SIZE));
                FST_INFO("Mask-flags: %s", printDBLine((int*)offset_mask_, buffer, LOG_TEXT_SIZE));
                FST_INFO("Last-state: %s", printDBLine((int*)offset_stat_, buffer, LOG_TEXT_SIZE));

                // 当前各关节位置和记录文件中各关节位置进行比对
                OffsetState state[NUM_OF_JOINT];
                memcpy(&old_jnt, &joint[0], sizeof(Joint));
                checkOffset(cur_jnt, old_jnt, state);

                FST_INFO("Curr-state: %s", printDBLine((int*)state, buffer, LOG_TEXT_SIZE));

                bool recorder_need_update = false;

                // 比对结果比记录文件中的状态标志更严重，则更新记录文件中的标志
                for (size_t i = 0; i < joint_num_; i++)
                {
                    if (offset_mask_[i] == OFFSET_UNMASK && state[i] > offset_stat_[i])
                    {
                        recorder_need_update = true;
                    }
                    else
                    {
                        state[i] = offset_stat_[i];
                    }
                }

                for (size_t i = joint_num_; i < NUM_OF_JOINT; i++)
                {
                    state[i] = OFFSET_LOST;
                }

                FST_INFO("Offset-state: %s", printDBLine((int*)state, buffer, LOG_TEXT_SIZE));

                if (recorder_need_update)
                {
                    // 更新记录文件中的零位标志
                    FST_INFO("Update offset state into recorder.");

                    vector<int> stat;

                    for (size_t i = 0; i < joint_num_; i++)
                    {
                        stat.push_back(int(state[i]));
                    }

                    if (robot_recorder_.setParam("state", stat) && robot_recorder_.dumpParamFile())
                    {
                        for (size_t i = 0; i < joint_num_; i++)
                        {
                            offset_stat_[i] = state[i];
                        }
                        FST_INFO("Success.");
                    }
                    else
                    {
                        FST_ERROR("Failed, err=0x%llx", robot_recorder_.getLastError());
                        current_state_ = MOTION_FORBIDDEN;
                        cali_stat = current_state_;
                        memcpy(offset_stat, offset_stat_, sizeof(offset_stat));
                        return robot_recorder_.getLastError();
                    }
                }

                bool limited = false;
                bool forbidden = false;

                // 综合各轴校验结果和零位错误屏蔽标志，给出允许运行标志
                        // 函数？？？
                for (size_t i = 0; i < joint_num_; i++)
                {
                    if (offset_mask_[i] == OFFSET_UNMASK)
                    {
                        if (state[i] != OFFSET_NORMAL)
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
                cali_stat = current_state_;
                memcpy(offset_stat, offset_stat_, sizeof(offset_stat));
                FST_INFO("Done, calibrate motion-state is %d", current_state_);
                return SUCCESS;
            }
            else
            {
                FST_ERROR("Invalid array size of joint in recorder, expect %d but get %d", joint_num_, joint.size());
                current_state_ = MOTION_FORBIDDEN;
                cali_stat = current_state_;
                memcpy(offset_stat, offset_stat_, sizeof(offset_stat));
                return INVALID_PARAMETER;
            }
        }
        else
        {
            FST_ERROR("Fail to get joint from recorder.");
            current_state_ = MOTION_FORBIDDEN;
            cali_stat = current_state_;
            memcpy(offset_stat, offset_stat_, sizeof(offset_stat));
            return robot_recorder_.getLastError();
        }
    }
    else
    {
        FST_ERROR("Fail to get current joint from share mem.");
        current_state_ = MOTION_FORBIDDEN;
        cali_stat = current_state_;
        memcpy(offset_stat, offset_stat_, sizeof(offset_stat));
        return BARE_CORE_TIMEOUT;
    }
}

//------------------------------------------------------------------------------
// 方法：  checkOffset
// 摘要：  对各个轴的当前读数和记录值进行比对，根据偏移门限和丢失门限给出比对结果；
//------------------------------------------------------------------------------
void Calibrator::checkOffset(Joint curr_jnt, Joint last_jnt, OffsetState *offset_stat)
{
    for (size_t i = 0; i < joint_num_; i++)
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
    /*
    if (index < joint_num_)
    {
        Joint cur_jnt;
        ServoState servo_state;
        vector<double> old_offset_data;

        if (bare_core_ptr_->getLatestJoint(cur_jnt, servo_state) && getOffsetFromBareCore(old_offset_data) == SUCCESS)
        {
            zero_offset_[index] = calculateOffset(old_offset_data[index], cur_jnt[index], 0);
            offset_need_save_[index] = NEED_SAVE;
            FST_INFO("Done.");
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Fail to get current offset or current joint from Core1");
            return BARE_CORE_TIMEOUT;
        }
    }
    else
    {
        FST_ERROR("Index out of range.");
        return INVALID_PARAMETER;
    }
    */
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
        }
        else
        {
            FST_ERROR("  joint index=%d, index out of range", pindex[i]);
            return INVALID_PARAMETER;
        }
    }

    Joint cur_joint;
    ServoState servo_state;
    vector<double> cur_offset;
    char buffer[LOG_TEXT_SIZE];

    if (bare_core_ptr_->getLatestJoint(cur_joint, servo_state) && getOffsetFromBareCore(cur_offset) == SUCCESS)
    {
        FST_INFO("Current-offset: %s", printDBLine(&cur_joint[0], buffer, LOG_TEXT_SIZE));
        FST_INFO("Current-joint:  %s", printDBLine(&cur_offset[0], buffer, LOG_TEXT_SIZE));

        size_t index;

        // 计算新的零位值，暂存直到saveOffset方法被调用，新的零位值生效并被写入配置文件中
        for (size_t i = 0; i < length; i++)
        {
            index = pindex[i];
            zero_offset_[index] = calculateOffset(cur_offset[index], cur_joint[index], 0);
            offset_need_save_[index] = NEED_SAVE;
        }

        FST_INFO("New-offset: %s", printDBLine(&zero_offset_[0], buffer, LOG_TEXT_SIZE));
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to get current offset or current joint from Core1");
        return BARE_CORE_TIMEOUT;
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
    FST_INFO("Current-offset=%.6f, current-joint=%.6f, target-joint=%.6f, new-offset=%.6f",
             current_offset, current_joint, target_joint, new_offset);
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
    bool need_save = false;

    for (size_t i = 0; i < joint_num_; i++)
    {
        if (offset_need_save_[i] == NEED_SAVE)
        {
            need_save = true;
            break;
        }
    }

    // 至少有一个轴的零位尚未生效和保存
    if (need_save)
    {
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
            if (offset_need_save_[i] == NEED_SAVE)
                data[i] = zero_offset_[i];
        }

        if (offset_param_.setParam("zero_offset/data", data) && offset_param_.dumpParamFile())
        {
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

            Joint joint;
            ServoState state;
            vector<double> jnt;

            // 更新记录文件，清除零位丢失标志和错误屏蔽标志
            if (!bare_core_ptr_->getLatestJoint(joint, state))
            {
                FST_ERROR("Fail to get current joint from core1.");
                return BARE_CORE_TIMEOUT;
            }

            FST_INFO("Current joint: %s", printDBLine(&joint[0], buffer, LOG_TEXT_SIZE));

            if (!robot_recorder_.getParam("joint", jnt))
            {
                FST_ERROR("Fail to get last joint from recorder.");
                return robot_recorder_.getLastError();
            }

            if (jnt.size() != joint_num_)
            {
                FST_ERROR("Wrong array size of last joint in recorder.");
                return INVALID_PARAMETER;
            }

            vector<int> flag;   flag.resize(joint_num_);
            vector<int> mask;   mask.resize(joint_num_);

            for (size_t i = 0; i < joint_num_; i++)
            {
                mask[i] = offset_mask_[i];
                flag[i] = offset_stat_[i];

                // 对于更新了零位的轴，同时更新其记录关节读数、零位状态标志和错误屏蔽标志
                if (offset_need_save_[i] == NEED_SAVE)
                {
                    jnt[i] = joint[i];
                    flag[i] = OFFSET_NORMAL;
                    mask[i] = OFFSET_UNMASK;
                }
            }

            time_t time_now = time(NULL);
            tm *local = localtime(&time_now);
            char buf[128];
            memset(buf, 0, sizeof(buf));
            strftime(buf, 64, "%Y-%m-%d %H:%M:%S", local);
            string temp(buf);

            FST_INFO("  time = %s", buf);
            FST_INFO("  flag = %s", printDBLine(&flag[0], buffer, LOG_TEXT_SIZE));
            FST_INFO("  mask = %s", printDBLine(&mask[0], buffer, LOG_TEXT_SIZE));
            FST_INFO("  joint = %s", printDBLine(&jnt[0], buffer, LOG_TEXT_SIZE));

            if (robot_recorder_.setParam("time", temp) &&
                robot_recorder_.setParam("joint", jnt) &&
                robot_recorder_.setParam("state", flag) &&
                robot_recorder_.setParam("mask", mask) &&
                robot_recorder_.dumpParamFile())
            {
                for (size_t i = 0; i < joint_num_; i++)
                {
                    if (offset_need_save_[i] == NEED_SAVE)
                    {
                        offset_need_save_[i] = UNNEED_SAVE;
                        offset_mask_[i] = OffsetMask (mask[i]);
                        offset_stat_[i] = OffsetState(flag[i]);
                    }
                }

                FST_INFO("Success!");
                return SUCCESS;
            }
            else
            {
                FST_ERROR("Fail to update recorder.");
                return robot_recorder_.getLastError();
            }
        }
        else
        {
            FST_ERROR("Fail to save offset, err=0x%llx", offset_param_.getLastError());
            return offset_param_.getLastError();
        }
    }
    else
    {
        FST_WARN("None offset need to save.");
        return SUCCESS;
    }
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
    FST_INFO("Save current joint into record file.");

    if (bare_core_ptr_->getLatestJoint(cur_jnt, state))
    {
        ErrorCode  err = saveGivenJoint(cur_jnt);

        if (err != SUCCESS)
        {
            FST_ERROR("Fail to record joint, code = 0x%llx", err);
            return err;
        }

        FST_INFO("Success.");
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Fail to get current joint from share memory");
        return BARE_CORE_TIMEOUT;
    }
}

//------------------------------------------------------------------------------
// 方法：  saveJoint
// 摘要：  将给定的关节读数写入记录文件；
//------------------------------------------------------------------------------
ErrorCode Calibrator::saveGivenJoint(const Joint &joint)
{
    vector<double> data(&joint[0], &joint[0] + joint_num_);
    time_t time_now = time(NULL);
    tm *local = localtime(&time_now);
    char buffer[LOG_TEXT_SIZE];
    memset(buffer, 0, LOG_TEXT_SIZE);
    strftime(buffer, 64, "%Y-%m-%d %H:%M:%S", local);
    string temp(buffer);

    FST_INFO("Save joint: %s", printDBLine(&joint[0], buffer, LOG_TEXT_SIZE));

    if (!robot_recorder_.setParam("time", temp) || !robot_recorder_.setParam("joint", data) || !robot_recorder_.dumpParamFile())
    {
        FST_ERROR("Fail to record time and joint into file, code = 0x%llx.", robot_recorder_.getLastError());
        return robot_recorder_.getLastError();
    }

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
    vector<int> mask;   mask.resize(joint_num_);

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

    if (need_save)
    {
        FST_INFO("Saving mask flags ...");

        for (size_t i = joint_num_; i < joint_num_; i++)
        {
            mask[i] = OFFSET_UNMASK;
        }

        if (robot_recorder_.setParam("mask", mask) && robot_recorder_.dumpParamFile())
        {
            for (size_t i = 0; i < joint_num_; i++)
            {
                if (offset_stat_[i] == OFFSET_LOST)
                {
                    offset_mask_[i] = OffsetMask(mask[i]);
                }
            }

            FST_INFO("All lost-offset errors masked.");
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Fail to save mask flags into recorder, err=0x%llx", robot_recorder_.getLastError());
            return robot_recorder_.getLastError();
        }
    }
    else
    {
        FST_INFO("No lost-offset error need mask");
        return SUCCESS;
    }
}

//------------------------------------------------------------------------------
// 方法：  setOffsetState
// 摘要：  当某些轴发生零位偏移时，允许用户对这些轴进行设置，将其状态归为零位正常或者零位丢失；
//------------------------------------------------------------------------------
ErrorCode Calibrator::setOffsetState(size_t index, OffsetState stat)
{
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Set offset state, index = %d, state = %d", index, stat);

    if (index < joint_num_)
    {
        bool update_joint = false;
        vector<int>     flag;   flag.resize(joint_num_);
        vector<double>  joint;  joint.resize(joint_num_);

        for (size_t i = 0; i < joint_num_; i++)
        {
            flag[i] = int(offset_stat_[i]);
        }

        FST_INFO("Offset state: %s", printDBLine(&flag[0], buffer, LOG_TEXT_SIZE));
        flag[index] = int(stat);
        FST_INFO("  changed to: %s", printDBLine(&flag[0], buffer, LOG_TEXT_SIZE));

        if (stat == OFFSET_NORMAL && offset_stat_[index] != OFFSET_NORMAL)
        {
            update_joint = true;
        }

        Joint cur_jnt;
        ServoState servo_state;

        if (!robot_recorder_.getParam("joint", joint))
        {
            FST_ERROR("Fail to get last joint from recorder, err=0x%llx", robot_recorder_.getLastError());
            return robot_recorder_.getLastError();
        }

        if (!bare_core_ptr_->getLatestJoint(cur_jnt, servo_state))
        {
            FST_ERROR("Fail to get current joint from bare core.");
            return BARE_CORE_TIMEOUT;
        }

        if (update_joint)
        {
            joint[index] = cur_jnt[index];
        }

        if (robot_recorder_.setParam("state", flag) && robot_recorder_.setParam("joint", joint) && robot_recorder_.dumpParamFile())
        {
            offset_stat_[index] = OffsetState(flag[index]);

            FST_INFO("Success.");
            return SUCCESS;
        }
        else
        {
            FST_ERROR("Fail to set offset flag into recorder, err=0x%llx", robot_recorder_.getLastError());
            return robot_recorder_.getLastError();
        }
    }
    else
    {
        FST_ERROR("Invalid index.");
        return INVALID_PARAMETER;
    }
}

//------------------------------------------------------------------------------
// 方法：  setOffset
// 摘要：  允许用户直接设置某个轴的零位值，被改变的零位值需要调用saveOffset借口才会被写入配置文件中；
//------------------------------------------------------------------------------
void Calibrator::setOffset(size_t index, double offset)
{
    FST_INFO("Set offset, index = %d, offset = %.6f", index, offset);
    
    if (index < joint_num_)
    {
        zero_offset_[index] = offset;
        offset_need_save_[index] = NEED_SAVE;
    }
}

//------------------------------------------------------------------------------
// 方法：  setOffset
// 摘要：  允许用户直接设置所有轴的零位值，被改变的零位值需要调用saveOffset借口才会被写入配置文件中；
//        注意确保传入的offset数组长度不小于有效轴数
//------------------------------------------------------------------------------
void Calibrator::setOffset(const double *offset)
{
    char buf[LOG_TEXT_SIZE];
    FST_INFO("Set offset: = %s", printDBLine(offset, buf, LOG_TEXT_SIZE));
    
    for (size_t i = 0; i < joint_num_; i++)
    {
        zero_offset_[i] = offset[i];
        offset_need_save_[i] = NEED_SAVE;
    }
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
    char buffer[LOG_TEXT_SIZE];

    FST_INFO("Save referenece point");
    ParamGroup params(AXIS_GROUP_DIR"arm_group_offset_reference.yaml");

    if (params.getLastError() != SUCCESS)
    {
        FST_ERROR("Error while load reference file, err=0x%llx", params.getLastError());
        return params.getLastError();
    }

    if (!bare_core_ptr_->getLatestJoint(joint, state))
    {
        FST_ERROR("Fail to get current joint from bare core");
        return BARE_CORE_TIMEOUT;
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
        return BARE_CORE_TIMEOUT;
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

    /*
    if (index < joint_num_)
    {
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
                if (ref_offset.size() == NUM_OF_JOINT && ref_encoder.size() == NUM_OF_JOINT && gear_ratio.size() == NUM_OF_JOINT)
                {
                    FST_INFO("Reference-offset=%.6f, reference-encoder=0x%x, gear-ratio=%.6f",
                             ref_offset[index], ref_encoder[index], gear_ratio[index]);

                    vector<int> cur_encoder; cur_encoder.resize(joint_num_);

                    if (bare_core_ptr_->getEncoder(cur_encoder))
                    {
                        FST_INFO("Current-encoder=0x%x", cur_encoder[index]);
                        zero_offset_[index] = calculateOffsetEasy(gear_ratio[index], ref_offset[index], ref_encoder[index], cur_encoder[index]);
                        offset_need_save_[index] = NEED_SAVE;
                        FST_INFO("New-offset=%.6f", zero_offset_[index]);
                        return SUCCESS;
                    }
                    else
                    {
                        FST_ERROR("Fail to get current encoders from bare core.");
                        return BARE_CORE_TIMEOUT;
                    }
                }
                else
                {
                    if (ref_offset.size() != NUM_OF_JOINT)
                        FST_ERROR("Invalid array size of reference offset, size is %d but %d wanted.", ref_offset.size(), NUM_OF_JOINT);
                    else if (ref_encoder.size() != NUM_OF_JOINT)
                        FST_ERROR("Invalid array size of reference encoder, size is %d but %d wanted.", ref_encoder.size(), NUM_OF_JOINT);
                    else
                        FST_ERROR("Invalid array size of gear ratio, size is %d but %d wanted.", gear_ratio.size(), NUM_OF_JOINT);

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
    else
    {
        FST_ERROR("Invalid index=%d, index should less than %d", index, joint_num_);
        return INVALID_PARAMETER;
    }
     */
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

                    for (size_t i = 0; i < length; i++)
                    {
                        index = pindex[i];
                        zero_offset_[index] = calculateOffsetEasy(gear_ratio[index], ref_offset[index], ref_encoder[index], cur_encoder[index]);
                        offset_need_save_[index] = NEED_SAVE;
                    }

                    FST_INFO("New offset: %s", printDBLine(zero_offset_, buffer, LOG_TEXT_SIZE));
                    return SUCCESS;
                }
                else
                {
                    FST_ERROR("Fail to get current encoders.");
                    return BARE_CORE_TIMEOUT;
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
    return bare_core_ptr_->setConfigData(id, data) ? SUCCESS : BARE_CORE_TIMEOUT;
}

//------------------------------------------------------------------------------
// 方法：  sendConfigData
// 摘要：  使用service形式向裸核发送参数id和数据；
//------------------------------------------------------------------------------
ErrorCode Calibrator::sendConfigData(int id, const vector<int> &data)
{
    return bare_core_ptr_->setConfigData(id, data) ? SUCCESS : BARE_CORE_TIMEOUT;
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
        return bare_core_ptr_->getConfigData(id, data) ? SUCCESS : BARE_CORE_TIMEOUT;
    }
    else
    {
        data.clear();
        return offset_param_.getLastError();
    }
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
