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
#include <motion_control_error_code.h>


using namespace std;

namespace fst_mc
{

Calibrator::Calibrator(size_t joint_num, BareCoreInterface *pcore, fst_log::Logger *plog)
{
    log_ptr_ = plog;
    joint_num_ = joint_num;
    bare_core_ptr_ = pcore;

    current_state_ = MOTION_FORBIDDEN;
    memset(normal_threshold_, 0, NUM_OF_JOINT * sizeof(double));
    memset(lost_threshold_, 0, NUM_OF_JOINT * sizeof(double));
    memset(zero_offset_, 0, NUM_OF_JOINT * sizeof(double));
    memset(offset_mask_, 0, NUM_OF_JOINT * sizeof(OffsetMask));
    memset(offset_stat_, 0, NUM_OF_JOINT * sizeof(OffsetState));
    memset(offset_need_save_, 0, NUM_OF_JOINT * sizeof(int));
}

Calibrator::~Calibrator(void)
{}

ErrorCode Calibrator::initCalibrator(const string &path)
{
    int id;
    ErrorCode result;
    vector<int> stat;
    vector<double> data;
    char buffer[LOG_TEXT_SIZE];

    FST_INFO("Initializing offset calibrator, number-of-joint = %d ...", joint_num_);

    // if directory given by 'path' is not found, create it
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

    // if record file is not found, re-create it from template in 'share/motion_controller/config'
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

    // load record file
    if (!robot_recorder_.loadParamFile(record_file.c_str()))
    {
        result = robot_recorder_.getLastError();
        FST_ERROR("Fail to load record file, err=0x%llx", result);
        return result;
    }

    // check whether data arrays in record file are valid
    stat.clear();
    if (!robot_recorder_.getParam("mask", stat))
    {
        result = robot_recorder_.getLastError();
        FST_ERROR("Fail to read joint-mask from record file, err=0x%llx", result);
        return result;
    }
    if (stat.size() != NUM_OF_JOINT)
    {
        FST_ERROR("Invalid array size of joint-mask, except %d but get %d", NUM_OF_JOINT, stat.size());
        return false;
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
    if (stat.size() != NUM_OF_JOINT)
    {
        FST_ERROR("Invalid array size of joint-flag, except %d but get %d", NUM_OF_JOINT, stat.size());
        return INVALID_PARAMETER;
    }
    FST_INFO("Recorded-Flags: %s", printDBLine(&stat[0], buffer, LOG_TEXT_SIZE));

    for (size_t i = 0; i < joint_num_; i++)
        offset_stat_[i] = OffsetState(stat[i]);

    // load thresholds used in checking offset
    fst_parameter::ParamGroup params("share/configuration/configurable/motion_plan.yaml");
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
        if (data.size() != NUM_OF_JOINT)
        {
            FST_ERROR("Invalid array size of normal threshold, except %d but get %d", NUM_OF_JOINT, data.size());
            return INVALID_PARAMETER;
        }
        FST_INFO("Threshold-normal: %s", printDBLine(&data[0], buffer, LOG_TEXT_SIZE));

        for (size_t i = 0; i < joint_num_; i++)
            normal_threshold_[i] = data[i];

        data.clear();
        params.getParam("calibrator/lost_offset_threshold", data);
        if (data.size() != NUM_OF_JOINT)
        {
            FST_ERROR("Invalid array size of lost threshold, except %d but get %d", NUM_OF_JOINT, data.size());
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

    // load offset config file
    offset_param_.loadParamFile("share/configuration/machine/offset.yaml");
    result = offset_param_.getLastError();
    if (result == SUCCESS)
    {
        data.clear();
        if (!offset_param_.getParam("zero_offset/id", id) || !offset_param_.getParam("zero_offset/data", data))
        {
            result = params.getLastError();
            FST_ERROR("Fail to read zero offset config file, err=0x%llx", result);
            return result;
        }
        if (data.size() != NUM_OF_JOINT)
        {
            FST_ERROR("Invalid array size of zero offset, except %d but get %d", NUM_OF_JOINT, data.size());
            return result;
        }

        FST_INFO("ID of offset: 0x%x", id);
        FST_INFO("Encoder-Zero-Offset: %s", printDBLine(&data[0], buffer, LOG_TEXT_SIZE));

        for (size_t i = 0; i < joint_num_; i++)
            zero_offset_[i] = data[i];
    }
    else
    {
        FST_ERROR("Loading offset param file failed, err=0x%llx", result);
        return result;
    }

    // set zero offset into core1, offset in core1 should be setted before engage
    FST_INFO("Downloading zero offset ...");
    if (sendJtacParam("zero_offset"))
    {
        FST_INFO("Success!");
        usleep(256 * 1000);
    }
    else
    {
        FST_ERROR("Failed, cannot set zero offset to Core1.");
        return BARE_CORE_TIMEOUT;
    }

    ////////////////////////////////
    // checkZeroOffset 
    ////////////////////////////////
    //CalibrateState calibrate_stat;
    //OffsetState offset_stat[NUM_OF_JOINT];
    //checkOffset(&calibrate_stat, offset_stat);

    return SUCCESS;
}


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




    ErrorCode Calibrator::checkOffset(CalibrateState *cali_stat, OffsetState *offset_stat)
{
    ServoState servo_state;
    vector<double> joint;
    Joint cur_jnt, old_jnt;
    char buffer[LOG_TEXT_SIZE];

    FST_INFO("Check zero offset.");

    if (bare_core_ptr_->getLatestJoint(cur_jnt, servo_state))
    {
        FST_INFO("Curr-joint: %s", printDBLine(&cur_jnt[0], buffer, LOG_TEXT_SIZE));

        if (robot_recorder_.getParam("joint", joint))
        {
            if (joint.size() == NUM_OF_JOINT)
            {
                FST_INFO("Last-joint: %s", printDBLine(&joint[0], buffer, LOG_TEXT_SIZE));
                FST_INFO("Mask-flags: %s", printDBLine((int*)offset_mask_, buffer, LOG_TEXT_SIZE));
                FST_INFO("Last-state: %s", printDBLine((int*)offset_stat_, buffer, LOG_TEXT_SIZE));

                OffsetState state[NUM_OF_JOINT];
                memcpy(&old_jnt, &joint[0], sizeof(Joint));
                checkOffset(cur_jnt, old_jnt, state);

                FST_INFO("Curr-state: %s", printDBLine((int*)state, buffer, LOG_TEXT_SIZE));

                bool recorder_need_update = false;

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
                    FST_INFO("Update offset state into recorder.");

                    vector<int> stat; stat.resize(NUM_OF_JOINT);

                    for (size_t i = 0; i < NUM_OF_JOINT; i++)
                    {
                        stat[i] = int(state[i]);
                    }

                    if (robot_recorder_.setParam("state", stat) && robot_recorder_.dumpParamFile())
                    {
                        for (size_t i = 0; i < joint_num_; i++)
                        {
                            offset_stat_[i] = state[i];
                            offset_stat[i] = state[i];
                        }
                        FST_INFO("Success.");
                    }
                    else
                    {
                        FST_ERROR("Failed, err=0x%llx", robot_recorder_.getLastError());
                        current_state_ = MOTION_FORBIDDEN;
                        *cali_stat = current_state_;
                        return robot_recorder_.getLastError();
                    }
                }

                bool limited = false;
                bool forbidden = false;

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
                *cali_stat = current_state_;
                FST_INFO("Done, calibrate motion-state is %d", current_state_);
                return SUCCESS;
            }
            else
            {
                FST_ERROR("Invalid array size of joint in recorder, expect %d but get %d", joint_num_, joint.size());
                current_state_ = MOTION_FORBIDDEN;
                *cali_stat = current_state_;
                return INVALID_PARAMETER;
            }
        }
        else
        {
            FST_ERROR("Fail to get joint from recorder.");
            current_state_ = MOTION_FORBIDDEN;
            *cali_stat = current_state_;
            return robot_recorder_.getLastError();
        }
    }
    else
    {
        FST_ERROR("Fail to get current joint from share mem.");
        current_state_ = MOTION_FORBIDDEN;
        *cali_stat = current_state_;
        return BARE_CORE_TIMEOUT;
    }
}


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


ErrorCode Calibrator::calibrateOffset(void)
{
    FST_INFO("Calibrate offset of all joints.");
    size_t indexs[NUM_OF_JOINT] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    return calibrateOffset(indexs, joint_num_);
}


ErrorCode Calibrator::calibrateOffset(size_t index)
{
    FST_INFO("Calibrate offset of joint index=%d", index);

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
            return true;
        }
        else
        {
            FST_ERROR("Fail to get current offset or current joint from Core1");
            return false;
        }
    }
    else
    {
        FST_ERROR("Index out of range.");
        return false;
    }
}


ErrorCode Calibrator::calibrateOffset(const size_t *pindex, size_t length)
{
    FST_INFO("Calibrate offset of:");

    for (size_t i = 0; i < length; i++)
    {
        if (pindex[i] < NUM_OF_JOINT)
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


double Calibrator::calculateOffset(double current_offset, double current_joint, double target_joint)
{
    double new_offset = current_joint + current_offset - target_joint;
    FST_INFO("Current-offset=%.6f, current-joint=%.6f, target-joint=%.6f, new-offset=%.6f",
             current_offset, current_joint, target_joint, new_offset);
    return new_offset;
}


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

    if (need_save)
    {
        vector<double> data; data.resize(NUM_OF_JOINT);

        if (!offset_param_.getParam("zero_offset/data", data))
        {
            FST_ERROR("Fail to get offset from config file;");
            return offset_param_.getLastError();
        }

        if (data.size() != NUM_OF_JOINT)
        {
            FST_ERROR("Wrong array size of offset in config file.");
            return  INVALID_PARAMETER;
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
            FST_INFO("Offset saved, update offset in core1 ...");

            if (!sendConfigData("zero_offset/"))
            {
                FST_ERROR("Cannot update offset in core1");
                return BARE_CORE_TIMEOUT;
            }

            usleep(200 * 1000);
            FST_INFO("Core1 offset updated, update recorder ...");
            vector<double> jnt;

            if (!robot_recorder_.getParam("joint", jnt))
            {
                FST_ERROR("Fail to get last joint from recorder.");
                return robot_recorder_.getLastError();
            }

            if (jnt.size() != NUM_OF_JOINT)
            {
                FST_ERROR("Wrong array size of last joint in recorder.");
                return INVALID_PARAMETER;
            }

            Joint joint;
            ServoState state;
            if (!bare_core_ptr_->getLatestJoint(joint, state))
            {
                FST_ERROR("Fail to get current joint from core1.");
                return BARE_CORE_TIMEOUT;
            }

            vector<int> flag;   flag.resize(NUM_OF_JOINT);
            vector<int> mask;   mask.resize(NUM_OF_JOINT);

            for (size_t i = 0; i < joint_num_; i++)
            {
                mask[i] = offset_mask_[i];
                flag[i] = offset_stat_[i];

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

                FST_INFO("SUccess!");
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


ErrorCode Calibrator::saveJoint(void)
{
    FST_INFO("Save current joint into record file.");
    Joint cur_jnt;
    ServoState state;

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


ErrorCode Calibrator::saveGivenJoint(const Joint &joint)
{
    vector<double> data(&joint[0], &joint[0] + NUM_OF_JOINT);
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


ErrorCode Calibrator::maskOffsetLostError(void)
{
    char buffer[LOG_TEXT_SIZE];
    vector<int> mask;   mask.resize(NUM_OF_JOINT);

    FST_INFO("Mask all lost-offset errors.");
    FST_INFO("     mask flag: %s", printDBLine((int*)offset_mask_, buffer, LOG_TEXT_SIZE));
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

        for (size_t i = joint_num_; i < NUM_OF_JOINT; i++)
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


ErrorCode Calibrator::setOffsetState(size_t index, OffsetState stat)
{
    char buffer[LOG_TEXT_SIZE];
    FST_INFO("Set offset state, index = %d", index);

    if (index < joint_num_)
    {
        vector<int> flag;   flag.resize(NUM_OF_JOINT);

        for (int i = 0; i < NUM_OF_JOINT; i++)
        {
            flag[i] = int(offset_stat_[i]);
        }

        FST_INFO("Offset state: %s", printDBLine(&flag[0], buffer, LOG_TEXT_SIZE));
        flag[index] = stat;
        FST_INFO("  changed to: %s", printDBLine(&flag[0], buffer, LOG_TEXT_SIZE));

        if (robot_recorder_.setParam("state", flag) && robot_recorder_.dumpParamFile())
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


CalibrateState Calibrator::getCalibrateState(void)
{
    return current_state_;
}


ErrorCode Calibrator::saveReference(void)
{
    Joint joint;
    ServoState state;
    char buffer[LOG_TEXT_SIZE];

    FST_INFO("Save referenece point");
    fst_parameter::ParamGroup params("share/configuration/configurable/offset_reference.yaml");

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

    vector<double> ref_joint(&joint[0], &joint[0] + NUM_OF_JOINT);
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
        !params.setParam("reference_encoder", ref_encoder) || !params.dumpParamFile())
    {
        FST_ERROR("Error while saving reference point, err=0x%llx", params.getLastError());
        return params.getLastError();
    }

    FST_INFO("Success.");
    return SUCCESS;
}


ErrorCode Calibrator::fastCalibrate(void)
{
    FST_INFO("Easy calibrate of all joints.");
    size_t indexs[NUM_OF_JOINT] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    return fastCalibrate(indexs, joint_num_);
}


ErrorCode Calibrator::fastCalibrate(size_t index)
{
    FST_INFO("Fsst calibrate index = %d", index);

    if (index < joint_num_)
    {
        vector<int>     ref_encoder;
        vector<double>  ref_offset;
        vector<double>  gear_ratio;

        fst_parameter::ParamGroup params("share/configuration/configurable/offset_reference.yaml");
        fst_parameter::ParamGroup jtac("share/configuration/machine/jtac.yaml");

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
}


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
            return false;
        }
    }

    vector<int>     ref_encoder;
    vector<double>  ref_offset;
    vector<double>  gear_ratio;

    fst_parameter::ParamGroup params("share/configuration/configurable/offset_reference.yaml");
    fst_parameter::ParamGroup jtac("share/configuration/machine/jtac.yaml");

    if (params.getLastError() == SUCCESS && jtac.getLastError() == SUCCESS)
    {
        if (params.getParam("reference_offset", ref_offset) &&
            params.getParam("reference_encoder", ref_encoder) &&
            jtac.getParam("gear_ratio/data", gear_ratio))
        {
            if (ref_offset.size() == NUM_OF_JOINT && ref_encoder.size() == NUM_OF_JOINT && gear_ratio.size() == NUM_OF_JOINT)
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
                    return true;
                }
                else
                {
                    FST_ERROR("Fail to get current encoders.");
                    return false;
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


ErrorCode Calibrator::sendJtacParam(const std::string &param)
{
    ErrorCode  err = SUCCESS;

    if (param == "zero_offset")
    {
        FST_INFO("Send zero offset to bare core.");
        err = sendConfigData("zero_offset/");
    }
    else if (param == "gear_ratio")
    {
        FST_INFO("Send gear ratio to bare core.");
        err = sendConfigData("gear_ratio/");
    }
    else if (param == "coupling_coefficient")
    {
        FST_INFO("Send coupling coefficient to bare core.");
        err = sendConfigData("coupling_coefficient/");
    }
    else if (param == "trajectory_delay")
    {
        FST_INFO("Send trajectory delay to bare core.");
        err = sendConfigData("trajectory_delay/");
    }
    else if (param == "all")
    {
        if (err == SUCCESS)
        {
            FST_INFO("Send zero offset to bare core.");
            err = sendConfigData("zero_offset/");
        }

        if (err == SUCCESS)
        {
            FST_INFO("Send gear ratio to bare core.");
            err = sendConfigData("gear_ratio/");
        }

        if (err == SUCCESS)
        {
            FST_INFO("Send coupling coefficient to bare core.");
            err = sendConfigData("coupling_coefficient/");
        }

        if (err == SUCCESS)
        {
            FST_INFO("Send trajectory delay to bare core.");
            err = sendConfigData("trajectory_delay/");
        }
    }
    else
    {
        FST_ERROR("Invalid parameter name.");
        err = INVALID_PARAMETER;
    }

    if (err == SUCCESS)
    {
        FST_INFO("Done.");
        return SUCCESS;
    }
    else
    {
        FST_ERROR("Abort.");
        return err;
    }
}


ErrorCode Calibrator::sendConfigData(const string &path)
{
    int id;
    vector<double> data;

    if (offset_param_.getParam(path + "id", id) && offset_param_.getParam(path + "data", data))
    {
        return bare_core_ptr_->setConfigData(id, data) ? SUCCESS : BARE_CORE_TIMEOUT;
    }
    else
    {
        return offset_param_.getLastError();
    }
}


ErrorCode Calibrator::getOffsetFromBareCore(vector<double> &data)
{
    int id;
    data.resize(NUM_OF_JOINT);

    if (offset_param_.getParam("zero_offset/id", id))
    {
        return bare_core_ptr_->getConfigData(id, data) ? SUCCESS : BARE_CORE_TIMEOUT;
    }
    else
    {
        data.clear();
        return offset_param_.getLastError();
    }
}


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
