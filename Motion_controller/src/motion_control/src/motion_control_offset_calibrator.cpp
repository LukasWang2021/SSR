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
    memset(offset_need_save_, 0, NUM_OF_JOINT * sizeof(bool));
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
        if (buildRecordFile(record_file))
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
    if (stat.size() < joint_num_)
    {
        FST_ERROR("Invalid array size of joint-mask, except %d but get %d", joint_num_, stat.size());
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
    if (stat.size() < joint_num_)
    {
        FST_ERROR("Invalid array size of joint-flag, except %d but get %d", joint_num_, stat.size());
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
        if (data.size() < joint_num_)
        {
            FST_ERROR("Invalid array size of normal threshold, except %d but get %d", joint_num_, data.size());
            return INVALID_PARAMETER;
        }
        FST_INFO("Threshold-normal: %s", printDBLine(&data[0], buffer, LOG_TEXT_SIZE));

        for (size_t i = 0; i < joint_num_; i++)
            normal_threshold_[i] = data[i];

        data.clear();
        params.getParam("calibrator/lost_offset_threshold", data);
        if (data.size() < joint_num_)
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
        if (data.size() < joint_num_)
        {
            FST_ERROR("Invalid array size of zero offset, except %d but get %d", joint_num_, data.size());
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
            if (joint.size() >= joint_num_)
            {
                FST_INFO("Last-joint: %s", printDBLine(&joint[0], buffer, LOG_TEXT_SIZE);
                FST_INFO("Mask-flags: %s", printDBLine(offset_mask_, buffer, LOG_TEXT_SIZE);
                FST_INFO("Last-state: %s", printDBLine(offset_stat_, buffer, LOG_TEXT_SIZE);

                OffsetState state[NUM_OF_JOINT];
                memcpy(&old_jnt, &joint[0], sizeof(Joint));
                checkOffset(cur_jnt, old_jnt, state);

                FST_INFO("Curr-state: %s", printDBLine(state, buffer, LOG_TEXT_SIZE);

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

                FST_INFO("Offset-state: %s", printDBLine(state, buffer, LOG_TEXT_SIZE);

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

        if (bare_core_ptr_->getLatestJoint(cur_jnt) && getOffsetFromBareCore(old_offset_data, servo_state))
        {
            zero_offset_[index] = calculateOffset(old_offset_data[index], cur_jnt[index], 0);
            offset_need_save_[index] = true;
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


bool Calibrator::calibrateOffset(const size_t *pindex, size_t length)
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
            return false;
        }
    }

    Joint cur_joint;
    ServoState servo_state;
    vector<double> cur_offset;

    if (ShareMem::instance()->getLatestJoint(cur_joint, servo_state) && getOffsetFromBareCore(cur_offset))
    {
        FST_INFO("Current-offset: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                 cur_joint[0], cur_joint[1], cur_joint[2], cur_joint[3], cur_joint[4],
                 cur_joint[5], cur_joint[6], cur_joint[7], cur_joint[8]);
        FST_INFO("Current-joint:  %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                 cur_offset[0], cur_offset[1], cur_offset[2], cur_offset[3], cur_offset[4],
                 cur_offset[5], cur_offset[6], cur_offset[7], cur_offset[8]);

        size_t index;

        for (size_t i = 0; i < length; i++)
        {
            index = pindex[i];
            zero_offset_[index] = calculateOffset(cur_offset[index], cur_joint[index], 0);
            offset_need_save_[index] = true;
        }

        FST_INFO("New-offset: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                 zero_offset_[0], zero_offset_[1], zero_offset_[2], zero_offset_[3], zero_offset_[4],
                 zero_offset_[5], zero_offset_[6], zero_offset_[7], zero_offset_[8]);
        return true;
    }
    else
    {
        FST_ERROR("Fail to get current offset or current joint from Core1");
        return false;
    }
}


bool Calibrator::saveOffset(void)
{
    FST_INFO("Save zero offset into config file ...");
    //vector<double> data(zero_offset_, zero_offset_ + NUM_OF_JOINT);
    bool need_save = offset_need_save_[0] || offset_need_save_[1] || offset_need_save_[2] ||
                     offset_need_save_[3] || offset_need_save_[4] || offset_need_save_[5] ||
                     offset_need_save_[6] || offset_need_save_[7] || offset_need_save_[8];

    if (need_save)
    {
        vector<double> data; data.resize(NUM_OF_JOINT);
        if (!offset_param_.getParam("zero_offset/data", data))
        {
            FST_ERROR("Fail to get offset from config file;");
            return false;
        }
        if (data.size() != NUM_OF_JOINT)
        {
            FST_ERROR("Wrong array size of offset in config file.");
        }

        FST_INFO("Old offset: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                 data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
        FST_INFO("New offset: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                 zero_offset_[0], zero_offset_[1], zero_offset_[2], zero_offset_[3], zero_offset_[4],
                 zero_offset_[5], zero_offset_[6], zero_offset_[7], zero_offset_[8]);
        FST_INFO("Need save: %d, %d, %d, %d, %d, %d, %d, %d, %d",
                 offset_need_save_[0], offset_need_save_[1], offset_need_save_[2], offset_need_save_[3], offset_need_save_[4],
                 offset_need_save_[5], offset_need_save_[6], offset_need_save_[7], offset_need_save_[8]);

        for (size_t i = 0; i < NUM_OF_JOINT; i++)
        {
            if (offset_need_save_[i])
                data[i] = zero_offset_[i];
        }

        if (offset_param_.setParam("zero_offset/data", data) && offset_param_.dumpParamFile())
        {
            FST_INFO("Offset saved, update offset in core1 ...");

            if (!sendConfigData("zero_offset/"))
            {
                FST_ERROR("Cannot update offset in core1");
                return false;
            }

            usleep(200 * 1000);
            FST_INFO("Core1 offset updated, update recorder ...");
            vector<double> jnt;

            if (!robot_recorder_.getParam("joint", jnt))
            {
                FST_ERROR("Fail to get last joint from recorder.");
                rcs::Error::instance()->add(robot_recorder_.getLastError());
                return false;
            }

            if (jnt.size() != NUM_OF_JOINT)
            {
                FST_ERROR("Wrong array size of last joint in recorder.");
                return false;
            }

            Joint joint;
            if (!ShareMem::instance()->getLatestJoint(joint))
            {
                FST_ERROR("Fail to get current joint from core1.");
                return false;
            }

            vector<int> flag;   flag.resize(NUM_OF_JOINT);
            vector<int> mask;   mask.resize(NUM_OF_JOINT);

            for (int i = 0; i < NUM_OF_JOINT; i++)
            {
                mask[i] = offset_mask_[i];
                flag[i] = offset_stat_[i];

                if (offset_need_save_[i])
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
            FST_INFO("  flag = %d %d %d %d %d %d %d %d %d",
                     flag[0], flag[1], flag[2], flag[3], flag[4], flag[5], flag[6], flag[7], flag[8]);
            FST_INFO("  mask = %d %d %d %d %d %d %d %d %d",
                     mask[0], mask[1], mask[2], mask[3], mask[4], mask[5], mask[6], mask[7], mask[8]);
            FST_INFO("  joint = %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                     jnt[0], jnt[1], jnt[2], jnt[3], jnt[4], jnt[5], jnt[6], jnt[7], jnt[8]);


            if (robot_recorder_.setParam("time", temp) &&
                robot_recorder_.setParam("joint", jnt) &&
                robot_recorder_.setParam("state", flag) &&
                robot_recorder_.setParam("mask", mask) &&
                robot_recorder_.dumpParamFile())
            {
                for (int i = 0; i < NUM_OF_JOINT; i++)
                {
                    if (offset_need_save_[i])
                    {
                        offset_need_save_[i] = false;
                        offset_mask_[i] = mask[i];
                        offset_stat_[i] = flag[i];
                    }
                }

                FST_INFO("SUccess!");
                return true;
            }
            else
            {
                FST_ERROR("Fail to update recorder.");
                rcs::Error::instance()->add(robot_recorder_.getLastError());
                return false;
            }
        }
        else
        {
            FST_ERROR("Fail to save offset, err=0x%llx", offset_param_.getLastError());
            rcs::Error::instance()->add(offset_param_.getLastError());
            return false;
        }
    }
    else
    {
        FST_WARN("None offset need to save.");
        return true;
    }
}


bool Calibrator::saveJoint(void)
{
    FST_INFO("Save current joint into record file.");
    Joint cur_jnt;

    if (ShareMem::instance()->getLatestJoint(cur_jnt))
    {
        if (saveGivenJoint(cur_jnt))
        {
            FST_INFO("Success.");
            return true;
        }
        else
        {
            FST_ERROR("Fail to record joint.");
            return false;
        }
    }
    else
    {
        FST_ERROR("Fail to get current joint from share memory");
        return false;
    }
}


bool Calibrator::saveGivenJoint(const Joint &joint)
{
    vector<double> data(&joint[0], &joint[0] + sizeof(joint) / sizeof(double));
    return saveGivenJoint(data);
}


bool Calibrator::saveGivenJoint(const vector<double> &joint)
{
    time_t time_now = time(NULL);
    tm *local = localtime(&time_now);
    char buf[128];
    memset(buf, 0, sizeof(buf));
    strftime(buf, 64, "%Y-%m-%d %H:%M:%S", local);
    string temp(buf);

    if (robot_recorder_.setParam("time", temp) && robot_recorder_.setParam("joint", joint) && robot_recorder_.dumpParamFile())
    {
        FST_INFO("Record-Time : %s", buf);
        FST_INFO("Record-Joint: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                 joint[0], joint[1],joint[2],joint[3],joint[4],joint[5],joint[6],joint[7],joint[8]);
        return true;
    }
    else
    {
        FST_ERROR("Fail to record time and joint into file.");
        rcs::Error::instance()->add(robot_recorder_.getLastError());
        return false;
    }
}



bool Calibrator::maskOffsetLost(void)
{
    vector<int> mask;   mask.resize(NUM_OF_JOINT);
    FST_INFO("Mask all lost-offset errors.");
    FST_INFO("     mask flag: %d %d %d %d %d %d %d %d %d",
             offset_mask_[0], offset_mask_[1], offset_mask_[2], offset_mask_[3], offset_mask_[4],
             offset_mask_[5], offset_mask_[6], offset_mask_[7], offset_mask_[8]);
    FST_INFO("  offset state: %d %d %d %d %d %d %d %d %d",
             offset_stat_[0], offset_stat_[1], offset_stat_[2], offset_stat_[3], offset_stat_[4],
             offset_stat_[5], offset_stat_[6], offset_stat_[7], offset_stat_[8]);

    bool need_save = false;

    for (int i = 0; i < NUM_OF_JOINT; i++)
    {
        mask[i] = offset_mask_[i];

        if (offset_stat_[i] == OFFSET_LOST)
        {
            FST_INFO("    index=%d, lost its offset, we'll try to mask this error.", i);
            mask[i] = OFFSET_MASKED;
            need_save = true;
        }
    }

    if (need_save)
    {
        FST_INFO("Saving mask flags ...");

        if (robot_recorder_.setParam("mask", mask) && robot_recorder_.dumpParam())
        {
            for (int i = 0; i < NUM_OF_JOINT; i++)
            {
                if (offset_stat_[i] == OFFSET_LOST)
                {
                    offset_mask_[i] = mask[i];
                }
            }

            FST_INFO("Done.");
        }
        else
        {
            FST_ERROR("Fail to save mask flags into recorder, err=0x%llx", robot_recorder_.getLastError());
            rcs::Error::instance()->add(robot_recorder_.getLastError());
            return false;
        }
    }

    FST_INFO("All lost-offset errors masked.");
    return true;
}


bool Calibrator::setOffsetState(size_t index, OffsetState stat)
{
    FST_INFO("Set offset state, index=%d", index);

    if (index < NUM_OF_JOINT)
    {
        vector<int> flag;   flag.resize(NUM_OF_JOINT);

        for (int i = 0; i < NUM_OF_JOINT; i++)
        {
            flag[i] = int(offset_stat_[i]);
        }

        FST_INFO("Offset state: %d %d %d %d %d %d %d %d %d",
                 flag[0], flag[1], flag[2], flag[3], flag[4], flag[5], flag[6], flag[7], flag[8]);

        flag[index] = stat;

        FST_INFO("  changed to: %d %d %d %d %d %d %d %d %d",
                 flag[0], flag[1], flag[2], flag[3], flag[4], flag[5], flag[6], flag[7], flag[8]);

        if (robot_recorder_.setParam("state", flag) && robot_recorder_.dumpParam())
        {
            offset_stat_[index] = flag[index];
            FST_INFO("Success.");
            return true;
        }
        else
        {
            FST_ERROR("Fail to set offset flag into recorder, err=0x%llx", robot_recorder_.getLastError());
            rcs::Error::instance()->add(robot_recorder_.getLastError());
            return false;
        }
    }
    else
    {
        FST_ERROR("Invalid index.");
        return false;
    }
}


void Calibrator::getOffset(double *offset)
{
    bool need_save = false;
    FST_INFO("Get recent offset.");

    for (int i = 0; i < NUM_OF_JOINT; i++)
    {
        offset[i] = zero_offset_[i];
        need_save = need_save || offset_need_save_[i];
    }

    FST_INFO("  %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
             offset[0], offset[1], offset[2], offset[3], offset[4], offset[5], offset[6], offset[7], offset[8]);

    if (need_save)
    {
        FST_WARN("One or more offset have not been saved.");
    }

    FST_INFO("Done.");
}


CalibrateState Calibrator::getCalibrateState(void)
{
    return current_state_;
}


double Calibrator::calculateOffset(double current_offset, double current_joint, double target_joint)
{
    double new_offset = current_joint + current_offset - target_joint;
    FST_INFO("Current-offset=%.6f, current-joint=%.6f, target-joint=%.6f, new-offset=%.6f",
             current_offset, current_joint, target_joint, new_offset);
    return new_offset;
}


double Calibrator::calculateOffsetEasy(double gear_ratio, double ref_offset,
                                       unsigned int ref_encoder, unsigned int cur_encoder)
{
    FST_INFO("Reference-offset=%.6f, reference-encoder=0x%x, current-encoder=0x%x, gear-ratio=%.6f",
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

    FST_INFO("Reference-rolls=0x%x, current-rolls=0x%x, new-offset=%.6f", ref_rolls, cur_rolls, new_offset);
    return new_offset;
}


bool Calibrator::saveReference(void)
{
    Joint joint;
    vector<int>     idata;
    vector<double>  fdata;

    FST_INFO("Save referenece point");
    fst_parameter::ParamGroup params("share/configuration/configurable/offset_reference.yaml");
    if (params.getLastError() == SUCCESS)
    {
        ShareMem::instance()->getLatestJoint(joint);
        fdata.insert(fdata.begin(), &joint[0], &joint[0] + NUM_OF_JOINT);
        params.setParam("reference_joint", fdata);
        FST_INFO("  joint: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                 fdata[0], fdata[1], fdata[2], fdata[3], fdata[4], fdata[5], fdata[6], fdata[7], fdata[8]);

        getOffsetFromBareCore(fdata);
        params.setParam("reference_offset", fdata);
        FST_INFO("  offset: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                 fdata[0], fdata[1], fdata[2], fdata[3], fdata[4], fdata[5], fdata[6], fdata[7], fdata[8]);

        ServiceParam::instance()->getEncoder(idata);
        params.setParam("reference_encoder", idata);
        FST_INFO("  encoder: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
                 idata[0], idata[1], idata[2], idata[3], idata[4], idata[5], idata[6], idata[7], idata[8]);

        if (params.dumpParamFile())
        {
            FST_INFO("Success.");
            return true;
        }
        else
        {
            FST_ERROR("Error while saving reference point, err=0x%llx", params.getLastError());
            return false;
        }
    }
    else
    {
        FST_ERROR("Error while load reference file, err=0x%llx", params.getLastError());
        return false;
    }
}


bool Calibrator::fastCalibrate(void)
{
    FST_INFO("Easy calibrate of all joints.");
    size_t indexs[NUM_OF_JOINT] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    return fastCalibrate(indexs, NUM_OF_JOINT);
}


bool Calibrator::fastCalibrate(size_t index)
{
    FST_INFO("Fsst calibrate index = %d", index);

    if (index < NUM_OF_JOINT)
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

                    vector<int> cur_encoder; cur_encoder.resize(NUM_OF_JOINT);
                    if (ServiceParam::instance()->getEncoder(cur_encoder) && cur_encoder.size() == NUM_OF_JOINT)
                    {
                        FST_INFO("Current-encoder=0x%x", cur_encoder[index]);
                        zero_offset_[index] = calculateOffsetEasy(gear_ratio[index], ref_offset[index], ref_encoder[index], cur_encoder[index]);
                        offset_need_save_[index] = true;
                        FST_INFO("New-offset=%.6f", zero_offset_[index]);
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

                    return false;
                }
            }
            else
            {
                if (params.getLastError() != SUCCESS)
                    FST_ERROR("Fail to get reference point from config file");
                else
                    FST_ERROR("Fail to get gear ratio from config file");

                return false;
            }
        }
        else
        {
            if (params.getLastError() != SUCCESS)
                FST_ERROR("Fail to open reference file");
            else
                FST_ERROR("Fail to open jtac config file");

            return false;
        }
    }
    else
    {
        FST_ERROR("Invalid index=%d, index should less than %d", index, NUM_OF_JOINT);
        return false;
    }
}


bool Calibrator::fastCalibrate(const size_t *pindex, size_t length)
{
    FST_INFO("Fsst calibrate offset of:");

    for (size_t i = 0; i < length; i++)
    {
        if (pindex[i] < NUM_OF_JOINT)
        {
            FST_INFO("  joint index=%d", pindex[i]);
        }
        else
        {
            FST_ERROR("  joint index=%d, index out of range", pindex[i]);
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
                FST_INFO("Reference offset: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                         ref_offset[0], ref_offset[1], ref_offset[2], ref_offset[3], ref_offset[4],
                         ref_offset[5], ref_offset[6], ref_offset[7], ref_offset[8]);
                FST_INFO("Reference encoder: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
                         ref_encoder[0], ref_encoder[1], ref_encoder[2], ref_encoder[3], ref_encoder[4],
                         ref_encoder[5], ref_encoder[6], ref_encoder[7], ref_encoder[8]);
                FST_INFO("Gear ratio: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                         gear_ratio[0], gear_ratio[1], gear_ratio[2], gear_ratio[3], gear_ratio[4],
                         gear_ratio[5], gear_ratio[6], gear_ratio[7], gear_ratio[8]);

                vector<int> cur_encoder; cur_encoder.resize(NUM_OF_JOINT);
                if (ServiceParam::instance()->getEncoder(cur_encoder) && cur_encoder.size() == NUM_OF_JOINT)
                {
                    FST_INFO("Current encoder: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x",
                             cur_encoder[0], cur_encoder[1], cur_encoder[2], cur_encoder[3], cur_encoder[4],
                             cur_encoder[5], cur_encoder[6], cur_encoder[7], cur_encoder[8]);

                    size_t index;
                    for (size_t i = 0; i < length; i++)
                    {
                        index = pindex[i];
                        zero_offset_[index] = calculateOffsetEasy(gear_ratio[index], ref_offset[index], ref_encoder[index], cur_encoder[index]);
                        offset_need_save_[index] = true;
                    }

                    FST_INFO("New offset: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f",
                             zero_offset_[0], zero_offset_[1], zero_offset_[2], zero_offset_[3], zero_offset_[4],
                             zero_offset_[5], zero_offset_[6], zero_offset_[7], zero_offset_[8]);
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

                return false;
            }
        }
        else
        {
            if (params.getLastError() != SUCCESS)
                FST_ERROR("Fail to get reference point from config file");
            else
                FST_ERROR("Fail to get gear ratio from config file");

            return false;
        }
    }
    else
    {
        if (params.getLastError() != SUCCESS)
            FST_ERROR("Fail to open reference file");
        else
            FST_ERROR("Fail to open jtac config file");

        return false;
    }

}


bool Calibrator::sendJtacParam(const std::string &param)
{
    if (param == "zero_offset")
    {
        return sendConfigData("zero_offset/");
    }
    else if (param == "gear_ratio")
    {
        return sendConfigData("gear_ratio/");
    }
    else if (param == "coupling_coefficient")
    {
        return sendConfigData("coupling_coefficient/");
    }
    else if (param == "trajectory_delay")
    {
        return sendConfigData("trajectory_delay/");
    }
    else if (param == "all")
    {
        bool result = true;
        result = result && sendConfigData("zero_offset/");
        result = result && sendConfigData("gear_ratio/");
        result = result && sendConfigData("coupling_coefficient/");
        result = result && sendConfigData("trajectory_delay/");
        return result;
    }
    else
    {
        FST_ERROR("Invalid parameter name");
        //rcs::Error:instance()->add(INVALID_PARAMETER);
        return false;
    }
}


bool Calibrator::sendConfigData(const string &path)
{
    int id;
    vector<double> data;

    if (offset_param_.getParam(path + "id", id) && offset_param_.getParam(path + "data", data))
    {
        return ServiceParam::instance()->setConfigData(id, data);
    }
    else
    {
        rcs::Error::instance()->add(offset_param_.getLastError());
        return false;
    }
}


bool Calibrator::getOffsetFromBareCore(vector<double> &data)
{
    int id;
    data.resize(NUM_OF_JOINT);

    if (offset_param_.getParam("zero_offset/id", id))
    {
        return ServiceParam::instance()->getConfigData(id, data);
    }
    else
    {
        data.clear();
        rcs::Error::instance()->add(offset_param_.getLastError());
        return false;
    }
}


bool Calibrator::buildRecordFile(const string &file)
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
        return true;
    }
    else
    {
        return false;
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
