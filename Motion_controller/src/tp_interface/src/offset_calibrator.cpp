/*************************************************************************
	> File Name: offset_calibrator.cpp
	> Author: 
	> Mail: 
	> Created Time: 2016年12月12日 星期一 14时30分05秒
 ************************************************************************/

#include <offset_calibrator.h>
#include <error_code.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <math.h>
#include "share_mem.h"
#include "service_parameter.h"
#include "error_monitor.h"

using std::string;
using std::vector;


Calibrator::Calibrator()
{
    is_using_temp_zero_offset_ = false;
    if (!initCalibrator())
    {
        rcs::Error::instance()->add(INIT_CONTROLLER_FAILED);
    }
}

Calibrator::~Calibrator(void)
{
    return;
}

const unsigned int& Calibrator::getCurrentState(void)
{
    return current_state_;
}

/*const ErrorCode& Calibrator::getLastError(void)*/
//{
    //return last_error_;
/*}*/

bool Calibrator::initCalibrator(const string &path)
{
    FST_INFO("Initializing offset calibrator ...");
    offset_param_.loadParamFile("share/configuration/machine/offset.yaml");
    U64 result = offset_param_.getLastError();
    if (result != SUCCESS) {
        rcs::Error::instance()->add(result);
        return false;
    }

    // if directory given by 'path' is not found, create it
    boost::filesystem::path pa(path);
    if (!boost::filesystem::is_directory(pa)) {
        FST_WARN("  Directory: %s not found", path.c_str());
        if (boost::filesystem::create_directory(pa)) {
            FST_INFO("  Directory: %s created", path.c_str());
        }
        else {
            FST_ERROR("  Failed to create directory: %s", path.c_str());
            return false;
        }
    }

    // if record file is not found, re-create it from template in 'share/motion_controller/config'
    string record_file =  path + "robot_recorder.yaml";
    boost::filesystem::path fi(record_file);
    if (!boost::filesystem::exists(fi)) {
        FST_WARN("  File: %s not found", record_file.c_str());
        if (buildRecorderFromTemplate(record_file)) {
            FST_INFO("  Record file build from template");
        }
        else {
            FST_ERROR("  Fail to build record file");
            return false;
        }
    }
    if (!robot_recorder_.loadParamFile(record_file.c_str())) {
        result = robot_recorder_.getLastError();
        if (result != SUCCESS) {
            rcs::Error::instance()->add(result);
            return false;
        }
    }
    vector<double> data;
    if (!robot_recorder_.getParam("last_joint", data)) {
        result = robot_recorder_.getLastError();
        if (result != SUCCESS) {
            rcs::Error::instance()->add(result);
            return false;
        }
    }
    if (data.size() < 9) {
        FST_ERROR("invalid parameter of last joint");
        return false;
    }
    if (!robot_recorder_.getParam("flag", data)) {
        result = robot_recorder_.getLastError();
        if (result != SUCCESS) {
            rcs::Error::instance()->add(result);
            return false;
        }
    }
    if (data.size() < 9) {
        FST_ERROR("invalid parameter of flag");
        return false;
    }

    fst_parameter::ParamGroup params("share/configuration/configurable/motion_plan.yaml");
    if (params.getLastError() == SUCCESS) {
        params.getParam("calibrator/normal_offset_threshold", offset_normal_threshold_);
        params.getParam("calibrator/lost_offset_threshold", offset_lost_threshold_);
        if (params.getLastError() != SUCCESS) {
            rcs::Error::instance()->add(params.getLastError());
            return false;
        } 
        if (offset_normal_threshold_.size() < 9 || offset_lost_threshold_.size() < 9) {
            FST_ERROR("invalid parameter of offset_normal_threshold_");
            return false;
        }
    }
    else {
        rcs::Error::instance()->add(params.getLastError());
        FST_ERROR("Fail to load offset thresholds");
        return false;
    }


    FST_INFO("Downloading JTAC parameters ...");
    if (transmitJtacParam("all")) {
        FST_INFO("Success!");
        usleep(256 * 1000);
    }

    ////////////////////////////////
    // checkZeroOffset 
    //===========================
    unsigned int calibrate_result;
    checkZeroOffset(calibrate_result);

    return true;
}

bool Calibrator::calibrateZeroOffset(unsigned int &calibrate_result)
{
  
    FST_INFO("Calibrating new zero offset ...");
    if (isUsingTempZeroOffset()) {
        if (setZeroOffset()) {
            FST_INFO("Done!");
            FST_INFO("Reviewing the new offset ...");
            unsigned int bitmap;
            if (checkZeroOffset(bitmap)) {
                FST_INFO("Success!");
                return true;
            }
            else {
                FST_ERROR("It seems that the new offset doesn't work.");
                return false;
            }
        }
        else {
            return false;
        }
    }
    else {
        FST_ERROR("Cannot calibrate new offset, set temp offset first.");
        rcs::Error::instance()->add(INVALID_ACTION_IN_CURRENT_STATE);
        return false;
    }
}

bool Calibrator::buildRecorderFromTemplate(const string &file)
{
    char buf[256] = {0};
    int length = readlink("/proc/self/exe", buf, sizeof(buf));
    boost::filesystem::path executable(buf);
    string temp = executable.parent_path().parent_path().parent_path().string() + "/share/motion_controller/config/robot_recorder.yaml";
    std::ifstream  in(temp.c_str());
    std::ofstream out(file.c_str());
    if (!in.is_open() || !out.is_open()) {
        return false;
    }

    out << in.rdbuf();
    in.close();
    out.close();
    return true;
}



bool Calibrator::getZeroOffsetFromBareCore(vector<double> &data)
{
    int id;
    data.resize(8);
    if (offset_param_.getParam("zero_offset/id", id)) {
        return ServiceParam::instance()->getConfigData(id, data);
    }
    else {
        data.clear();
        rcs::Error::instance()->add(offset_param_.getLastError());
        return false;
    }
}

bool Calibrator::setTempZeroOffset(void)
{
    Joint joint;
    memset(&joint, 0, sizeof(joint));
    return setTempZeroOffsetImpl(joint);
}

bool Calibrator::setZeroOffset(void)
{
    Joint joint;
    memset(&joint, 0, sizeof(joint));
    return setZeroOffsetImpl(joint);
}

bool Calibrator::isUsingTempZeroOffset(void)
{
    return is_using_temp_zero_offset_;
}

bool Calibrator::setTempZeroOffsetImpl(const Joint &target_joint)
{
    Joint cur_jnt;
    vector<double> old_offset_data;

    if (ShareMem::instance()->getLatestJoint(cur_jnt)
    && getZeroOffsetFromBareCore(old_offset_data)) 
    {
        temp_zero_offset_.resize(8);
        temp_zero_offset_[0] = cur_jnt.j1 + old_offset_data[0] - target_joint.j1;
        temp_zero_offset_[1] = cur_jnt.j2 + old_offset_data[1] - target_joint.j2;
        temp_zero_offset_[2] = cur_jnt.j3 + old_offset_data[2] - target_joint.j3;
        temp_zero_offset_[3] = cur_jnt.j4 + old_offset_data[3] - target_joint.j4;
        temp_zero_offset_[4] = cur_jnt.j5 + old_offset_data[4] - target_joint.j5;
        temp_zero_offset_[5] = cur_jnt.j6 + old_offset_data[5] - target_joint.j6;
        temp_zero_offset_[6] = old_offset_data[6];
        temp_zero_offset_[7] = old_offset_data[7];

        FST_INFO("old offset=%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", 
                 old_offset_data[0], old_offset_data[1], old_offset_data[2],
                 old_offset_data[3], old_offset_data[4], old_offset_data[5],
                 old_offset_data[6], old_offset_data[7]);
        FST_INFO("old joint=%lf, %lf, %lf, %lf, %lf, %lf",
                 cur_jnt.j1, cur_jnt.j2, cur_jnt.j3, cur_jnt.j4
                 , cur_jnt.j5, cur_jnt.j6);
        FST_INFO("target joint=%lf, %lf, %lf, %lf, %lf, %lf",
                 target_joint.j1, target_joint.j2, target_joint.j3,
                 target_joint.j4, target_joint.j5, target_joint.j6);
        FST_INFO("new offset=%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", 
                 temp_zero_offset_[0], temp_zero_offset_[1], temp_zero_offset_[2],
                 temp_zero_offset_[3], temp_zero_offset_[4], temp_zero_offset_[5],
                 temp_zero_offset_[6], temp_zero_offset_[7]);

        int id;
        offset_param_.getParam("zero_offset/id", id);
        if (ServiceParam::instance()->setConfigData(id, temp_zero_offset_)) {
            is_using_temp_zero_offset_ = true;
            temp_robot_recorder_.clear();
            temp_robot_recorder_.push_back(target_joint.j1);
            temp_robot_recorder_.push_back(target_joint.j2);
            temp_robot_recorder_.push_back(target_joint.j3);
            temp_robot_recorder_.push_back(target_joint.j4);
            temp_robot_recorder_.push_back(target_joint.j5);
            temp_robot_recorder_.push_back(target_joint.j6);
            temp_robot_recorder_.push_back(0.0);
            temp_robot_recorder_.push_back(0.0);
            return true;
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }
}

bool Calibrator::setZeroOffsetImpl(const Joint &target_joint)
{
    Joint cur_jnt;
    vector<double> old_offset_data;
    vector<double> new_offset_data;
    if (ShareMem::instance()->getLatestJoint(cur_jnt)
    && getZeroOffsetFromBareCore(old_offset_data)) 
    {
        new_offset_data.resize(8);
        new_offset_data[0] = cur_jnt.j1 + old_offset_data[0] - target_joint.j1;
        new_offset_data[1] = cur_jnt.j2 + old_offset_data[1] - target_joint.j2;
        new_offset_data[2] = cur_jnt.j3 + old_offset_data[2] - target_joint.j3;
        new_offset_data[3] = cur_jnt.j4 + old_offset_data[3] - target_joint.j4;
        new_offset_data[4] = cur_jnt.j5 + old_offset_data[4] - target_joint.j5;
        new_offset_data[5] = cur_jnt.j6 + old_offset_data[5] - target_joint.j6;
        new_offset_data[6] = old_offset_data[6];
        new_offset_data[7] = old_offset_data[7];

        FST_INFO("old offset=%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", 
                 old_offset_data[0], old_offset_data[1], old_offset_data[2],
                 old_offset_data[3], old_offset_data[4], old_offset_data[5],
                     old_offset_data[6], old_offset_data[7]);
        FST_INFO("old joint=%lf, %lf, %lf, %lf, %lf, %lf",
                 cur_jnt.j1, cur_jnt.j2, cur_jnt.j3, cur_jnt.j4
                 , cur_jnt.j5, cur_jnt.j6);
        FST_INFO("target joint=%lf, %lf, %lf, %lf, %lf, %lf",
                 target_joint.j1, target_joint.j2, target_joint.j3,
                 target_joint.j4, target_joint.j5, target_joint.j6);
        FST_INFO("new offset=%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", 
                 new_offset_data[0], new_offset_data[1], new_offset_data[2],
                 new_offset_data[3], new_offset_data[4], new_offset_data[5],
                 new_offset_data[6], new_offset_data[7]);
        
        int id;
        offset_param_.getParam("zero_offset/id", id);
        if (ServiceParam::instance()->setConfigData(id, new_offset_data)) {
            vector<int> flag(9, OFFSET_NORMAL);
            vector<double> recorder((double*)(&target_joint), (double*)(&target_joint) + 6);
            recorder.push_back(0.0);
            recorder.push_back(0.0);
            recorder.push_back(0.0);
            
            time_t time_now = time(NULL);
            tm *local = localtime(&time_now);
            char buf[128];
            memset(buf, 0, sizeof(buf));
            strftime(buf, 64, "%Y-%m-%d %H:%M:%S", local);
            string temp(buf);

            if (offset_param_.setParam("zero_offset/data", new_offset_data) && 
                offset_param_.dumpParamFile() &&
                robot_recorder_.setParam("last_time", temp) &&
                robot_recorder_.setParam("last_joint", recorder) &&
                robot_recorder_.setParam("flag", flag) &&
                robot_recorder_.dumpParamFile())
            {
                is_using_temp_zero_offset_ = false;
                return true;
            }
            else {
                if (offset_param_.getLastError() != SUCCESS)
                    rcs::Error::instance()->add(offset_param_.getLastError());
                else
                    rcs::Error::instance()->add(robot_recorder_.getLastError());
                return false;
            }
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }
}

bool Calibrator::reviewCurrentJoint(unsigned int &bitmap)
{
    bool result = true;
    Joint cur_jnt;
    vector<double> last_joint;
    FST_INFO("sfdsssfddddd");
    if (ShareMem::instance()->getLatestJoint(cur_jnt)) {
        double *position = (double*)&cur_jnt;
        // if using normal zero-offset, compare robot_recorder with current joint
        if (!is_using_temp_zero_offset_) {
            vector<int> flag;
            if (robot_recorder_.getParam("flag", flag) && robot_recorder_.getParam("last_joint", last_joint)) {
                bitmap = OFFSET_NORMAL;
                for (int loop = 0; loop < 6; ++loop) {
                    bitmap |= flag[loop] << loop * 4;
                }
                if (bitmap == OFFSET_NORMAL) {
                    for (int loop = 0; loop < 6; ++loop) {
                        if (fabs(position[loop] - last_joint[loop]) > offset_lost_threshold_[loop]) {
                            bitmap |= OFFSET_LOST << loop * 4;
                            flag[loop] = OFFSET_LOST;
                        }
                        else if (fabs(position[loop] - last_joint[loop]) > offset_normal_threshold_[loop]) {
                            bitmap |= OFFSET_DEVIATE << loop * 4;
                            flag[loop] = OFFSET_DEVIATE;
                        }
                    }
                    if (bitmap != OFFSET_NORMAL) {
                        if (!robot_recorder_.setParam("flag", flag) || !robot_recorder_.dumpParamFile()) {
                            rcs::Error::instance()->add(robot_recorder_.getLastError());
                            result = false;
                        }
                    }
                }
            }
            else {
                rcs::Error::instance()->add(robot_recorder_.getLastError());
                result = false;
            }
        }
        // if using temp zero-offset, compare temp_robot_recorder with current joint
        else {
            bitmap = OFFSET_NORMAL;
            for (int loop = 0; loop < 6; ++loop) {
                if (fabs(position[loop] - temp_robot_recorder_[loop]) > offset_lost_threshold_[loop])
                    bitmap |= OFFSET_LOST << loop * 4;
                else if (fabs(position[loop] - temp_robot_recorder_[loop]) > offset_normal_threshold_[loop])
                    bitmap |= OFFSET_DEVIATE << loop * 4;
            }
        }
    }
    else {
        result = false;
    }

    if (result && !is_using_temp_zero_offset_ && bitmap == OFFSET_NORMAL) {
        current_state_ = CALIBRATED;
    }
    else {
        if (result && !is_using_temp_zero_offset_) {
            current_state_ = NEED_CALIBRATE;
        }
        if (result) {
            if (!is_using_temp_zero_offset_) {
                FST_INFO("Expected joint <-> Current joint");
                FST_INFO(" %lf\t\t%lf", last_joint[0], cur_jnt.j1);
                FST_INFO(" %lf\t\t%lf", last_joint[1], cur_jnt.j2);
                FST_INFO(" %lf\t\t%lf", last_joint[2], cur_jnt.j3);
                FST_INFO(" %lf\t\t%lf", last_joint[3], cur_jnt.j4);
                FST_INFO(" %lf\t\t%lf", last_joint[4], cur_jnt.j5);
                FST_INFO(" %lf\t\t%lf", last_joint[5], cur_jnt.j6);
            }
            else {
                FST_INFO("Expected joint <-> Current joint");
                FST_INFO(" %lf\t\t%lf", temp_robot_recorder_[0], cur_jnt.j1);
                FST_INFO(" %lf\t\t%lf", temp_robot_recorder_[1], cur_jnt.j2);
                FST_INFO(" %lf\t\t%lf", temp_robot_recorder_[2], cur_jnt.j3);
                FST_INFO(" %lf\t\t%lf", temp_robot_recorder_[3], cur_jnt.j4);
                FST_INFO(" %lf\t\t%lf", temp_robot_recorder_[4], cur_jnt.j5);
                FST_INFO(" %lf\t\t%lf", temp_robot_recorder_[5], cur_jnt.j6);
            }
        }
    }
    return result;
}

bool Calibrator::recordCurrentJoint(void) {
    Joint cur_jnt;
    if (ShareMem::instance()->getLatestJoint(cur_jnt)) {
        vector<double> data((double*)(&cur_jnt), (double*)(&cur_jnt) + sizeof(cur_jnt) / sizeof(double));
        FST_INFO("size:%d", data.size());
        return recordGivenJointImpl(data);
    }
    else {
        return false;
    }
}
bool Calibrator::recordGivenJoint(const Joint &joint)
{
    vector<double> data((double*)(&joint), (double*)(&joint) + sizeof(joint) / sizeof(double));
    return recordGivenJointImpl(data);
}

bool Calibrator::recordGivenJointImpl(vector<double> &joint)
{
    if (joint.size() == 6) {joint.push_back(0.0); joint.push_back(0.0); joint.push_back(0.0);}
    if (joint.size() == 9) {
        if (!is_using_temp_zero_offset_) {
            return recordJointToRobotRecorder(joint);
        }
        else {
            return recordJointToTempRecorder(joint);
        }
    }
    else {
        FST_ERROR("invalid parameter of given joint. size:%d", joint.size());
        //rcs::Error::instance()->add(INVALID_PARAMETER);
        return false;
    }
}

bool Calibrator::recordJointToRobotRecorder(const vector<double> &joint)
{
    time_t time_now = time(NULL);
    tm *local = localtime(&time_now);
    char buf[128];
    memset(buf, 0, sizeof(buf));
    strftime(buf, 64, "%Y-%m-%d %H:%M:%S", local);
    string temp(buf);

    if (robot_recorder_.setParam("last_time", temp) && robot_recorder_.setParam("last_joint", joint)) {
        if (robot_recorder_.dumpParamFile()) {
            return true;
        }
        else {
            rcs::Error::instance()->add(robot_recorder_.getLastError());
            return false;
        }
    }
    else {
        rcs::Error::instance()->add(robot_recorder_.getLastError());
        return false;
    }
}

bool Calibrator::recordJointToTempRecorder(const vector<double> &joint)
{
    temp_robot_recorder_.clear();
    temp_robot_recorder_.assign(joint.begin(), joint.end());
    return true;
}

bool Calibrator::transmitJtacParam(const std::string &param) {
    if (param == "zero_offset") {
        return sendConfigData("zero_offset/");
    }
    else if (param == "gear_ratio") {
        return sendConfigData("gear_ratio/");
    }
    else if (param == "coupling_coefficient") {
        return sendConfigData("coupling_coefficient/");
    }
    else if (param == "trajectory_delay") {
        return sendConfigData("trajectory_delay/");
    }
    else if (param == "all") {
        bool result = sendConfigData("zero_offset/");
        //result = result && sendConfigData("gear_ratio/");
        //result = result && sendConfigData("coupling_coefficient/");
        //result = result && sendConfigData("trajectory_delay/");
        return result;
    }
    else {
        FST_ERROR("invalid parameter name");
        //rcs::Error:instance()->add(INVALID_PARAMETER);
        return false;    
    }
}

bool Calibrator::sendConfigData(const string &path) {
    int id;
    vector<double> data;

    if (offset_param_.getParam(path + "id", id) && 
        offset_param_.getParam(path + "data", data)) {
        return ServiceParam::instance()->setConfigData(id, data);
    }
    else {
        rcs::Error::instance()->add(offset_param_.getLastError());
        return false;
    }
    
}


bool Calibrator::checkZeroOffset(unsigned int &calibrate_result)
{
    FST_INFO("Reviewing current joint and joint in recorder ...");
    unsigned int result;
    if (!reviewCurrentJoint(result)) 
        return false;
    calibrate_result = result;
    if (result == OFFSET_NORMAL) {
        FST_INFO("Success!");
    }
    else {
        if ((result & OFFSET_LOST_MASK) != 0) {
            rcs::Error::instance()->add(ZERO_OFFSET_LOST);
            FST_ERROR("Fault: zero offset lost, need calibration. ");
            return false;
        }
        else if ((result & OFFSET_DEVIATE_MASK) != 0) {
            rcs::Error::instance()->add(ZERO_OFFSET_DEVIATE);
            FST_ERROR("Fault: zero offset deviated, need calibration.");
            return false;
        }
        else {
            //rcs::Error::instance()->add(MOTION_INTERNAL_FAULT);
            FST_ERROR("check offset internal fault");
            return false;
        }
    }

    return true;
}
