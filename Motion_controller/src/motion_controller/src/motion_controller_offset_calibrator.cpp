/*************************************************************************
	> File Name: motion_controller_offset_calibration.cpp
	> Author: 
	> Mail: 
	> Created Time: 2016年12月12日 星期一 14时30分05秒
 ************************************************************************/

#include <iostream>
#include <motion_controller/motion_controller_offset_calibrator.h>
#include <motion_controller/motion_controller_error_code.h>
#include <middleware_to_mem/middleware_to_sharedmem.h>
#include <struct_to_mem/struct_service_request.h>
#include <struct_to_mem/struct_service_request.h>


using std::string;
using std::vector;

static const unsigned int UNDEFINED         = 0x5555;
static const unsigned int INITIALIZED       = 0x5556;


static const unsigned char READ_BY_ADDR     = 0x14;
static const unsigned char READ_BY_ID       = 0x1D;
static const unsigned char WRITE_BY_ADDR    = 0x24;
static const unsigned char WRITE_BY_ID      = 0x2D;

// static fst_log::Logger private_log;

namespace fst_controller {

Calibrator::Calibrator(fst_log::Logger &inh_log):log(inh_log)
{
    current_state_ = UNDEFINED;
    last_error_ = SUCCESS;
    mem_handle_ = -1;
    is_using_temp_zero_offset_ = false;
}

Calibrator::~Calibrator(void)
{
    return;
}

const unsigned int& Calibrator::getCurrentState(void)
{
    return current_state_;
}

const ErrorCode& Calibrator::getLastError(void)
{
    return last_error_;
}

bool Calibrator::initCalibrator(const string &path)
{
    if (current_state_ != UNDEFINED) {last_error_ = CALIBRATION_FAIL_IN_INIT; return false;}

    string temp_str = path + "jtac.yaml";
    jtac_param_.loadParamFile(temp_str);
    if (jtac_param_.getLastError() != SUCCESS) {
        last_error_ = jtac_param_.getLastError();
        return false;
    }
    temp_str = path + "robot_recorder.yaml";
    robot_recorder_.loadParamFile(temp_str);
    if (robot_recorder_.getLastError() != SUCCESS) {
        last_error_ = robot_recorder_.getLastError();
        return false;
    }

    fst_parameter::ParamGroup params("share/motion_controller/config/motion_controller.yaml");
    if (params.getLastError() == SUCCESS) {
        params.getParam("calibrator/normal_offset_threshold", offset_normal_threshold_);
        params.getParam("calibrator/lost_offset_threshold", offset_lost_threshold_);
        if (params.getLastError() != SUCCESS) {
            last_error_ = params.getLastError();
            return false;
        } 
        if (offset_normal_threshold_.size() != 8 || offset_lost_threshold_.size() != 8) {
            last_error_ = INVALID_PARAMETER;
            return false;
        }
    }
    else {
        last_error_ = params.getLastError();
        log.error("Fail to load offset thresholds, error code=0x%llx", last_error_);
        return false;
    }

    mem_handle_ = openMem(MEM_CORE);
    if (mem_handle_ == -1) {
        last_error_ = MOTION_FAIL_IN_INIT;
        return false;
    }

    if (comm_interface_.createChannel(COMM_REQ, COMM_IPC, "servo_param") != SUCCESS) {
        last_error_ = MOTION_FAIL_IN_INIT;
        return false;
    }

    current_state_ = INITIALIZED;
    return true;
}

bool Calibrator::getCurrentJoint(FeedbackJointState &fbjs)
{
    if (current_state_ < INITIALIZED) {last_error_ = NEED_INITIALIZATION; return false;}

    int cnt = 0;
    while (cnt < 10) {
        ++cnt;
        if (true == readWriteSharedMem(mem_handle_, &fbjs, "FeedbackJointState", MEM_READ)) {
            return true;
        }
        usleep(500);
    }

    last_error_ = FAIL_GET_FEEDBACK_JOINT;
    return false;
}

bool Calibrator::getZeroOffsetFromBareCore(vector<double> &data)
{
    int id;
    data.resize(8);
    if (jtac_param_.getParam("zero_offset/id", id)) {
        return readConfigDataImpl(id, data);
    }
    else {
        data.clear();
        last_error_ = jtac_param_.getLastError();
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
    if (current_state_ < INITIALIZED) {last_error_ = NEED_INITIALIZATION; return false;}

    FeedbackJointState fbjs;
    vector<double> old_offset_data;

    if (getCurrentJoint(fbjs) && getZeroOffsetFromBareCore(old_offset_data)) {
        temp_zero_offset_.resize(8);
        temp_zero_offset_[0] = fbjs.position[0] + old_offset_data[0] - target_joint.j1;
        temp_zero_offset_[1] = fbjs.position[1] + old_offset_data[1] - target_joint.j2;
        temp_zero_offset_[2] = fbjs.position[2] + old_offset_data[2] - target_joint.j3;
        temp_zero_offset_[3] = fbjs.position[3] + old_offset_data[3] - target_joint.j4;
        temp_zero_offset_[4] = fbjs.position[4] + old_offset_data[4] - target_joint.j5;
        temp_zero_offset_[5] = fbjs.position[5] + old_offset_data[5] - target_joint.j6;
        temp_zero_offset_[6] = old_offset_data[6];
        temp_zero_offset_[7] = old_offset_data[7];

        log.info("old offset=%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", 
                 old_offset_data[0], old_offset_data[1], old_offset_data[2],
                 old_offset_data[3], old_offset_data[4], old_offset_data[5],
                 old_offset_data[6], old_offset_data[7]);
        log.info("old joint=%lf, %lf, %lf, %lf, %lf, %lf",
                 fbjs.position[0], fbjs.position[1], fbjs.position[2],
                 fbjs.position[3], fbjs.position[4], fbjs.position[5]);
        log.info("target joint=%lf, %lf, %lf, %lf, %lf, %lf",
                 target_joint.j1, target_joint.j2, target_joint.j3,
                 target_joint.j4, target_joint.j5, target_joint.j6);
        log.info("new offset=%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", 
                 temp_zero_offset_[0], temp_zero_offset_[1], temp_zero_offset_[2],
                 temp_zero_offset_[3], temp_zero_offset_[4], temp_zero_offset_[5],
                 temp_zero_offset_[6], temp_zero_offset_[7]);

        int id;
        jtac_param_.getParam("zero_offset/id", id);
        if (sendConfigDataImpl(id, temp_zero_offset_)) {
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
    if (current_state_ < INITIALIZED) {last_error_ = NEED_INITIALIZATION; return false;}

    FeedbackJointState fbjs;
    vector<double> old_offset_data;
    vector<double> new_offset_data;
    if (getCurrentJoint(fbjs) && getZeroOffsetFromBareCore(old_offset_data)) {
        new_offset_data.resize(8);
        new_offset_data[0] = fbjs.position[0] + old_offset_data[0] - target_joint.j1;
        new_offset_data[1] = fbjs.position[1] + old_offset_data[1] - target_joint.j2;
        new_offset_data[2] = fbjs.position[2] + old_offset_data[2] - target_joint.j3;
        new_offset_data[3] = fbjs.position[3] + old_offset_data[3] - target_joint.j4;
        new_offset_data[4] = fbjs.position[4] + old_offset_data[4] - target_joint.j5;
        new_offset_data[5] = fbjs.position[5] + old_offset_data[5] - target_joint.j6;
        new_offset_data[6] = old_offset_data[6];
        new_offset_data[7] = old_offset_data[7];

        log.info("old offset=%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", 
                 old_offset_data[0], old_offset_data[1], old_offset_data[2],
                 old_offset_data[3], old_offset_data[4], old_offset_data[5],
                     old_offset_data[6], old_offset_data[7]);
        log.info("old joint=%lf, %lf, %lf, %lf, %lf, %lf",
                 fbjs.position[0], fbjs.position[1], fbjs.position[2],
                 fbjs.position[3], fbjs.position[4], fbjs.position[5]);
        log.info("target joint=%lf, %lf, %lf, %lf, %lf, %lf",
                 target_joint.j1, target_joint.j2, target_joint.j3,
                 target_joint.j4, target_joint.j5, target_joint.j6);
        log.info("new offset=%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", 
                 new_offset_data[0], new_offset_data[1], new_offset_data[2],
                 new_offset_data[3], new_offset_data[4], new_offset_data[5],
                 new_offset_data[6], new_offset_data[7]);
        
        int id;
        jtac_param_.getParam("zero_offset/id", id);
        if (sendConfigDataImpl(id, new_offset_data)) {
            vector<double> recorder((double*)(&target_joint), (double*)(&target_joint) + 6);
            recorder.push_back(0.0);
            recorder.push_back(0.0);
            
            time_t time_now = time(NULL);
            tm *local = localtime(&time_now);
            char buf[128];
            memset(buf, 0, sizeof(buf));
            strftime(buf, 64, "%Y-%m-%d %H:%M:%S", local);
            string temp(buf);

            if (jtac_param_.setParam("zero_offset/data", new_offset_data) && 
                jtac_param_.dumpParamFile() &&
                robot_recorder_.setParam("last_time", temp) &&
                robot_recorder_.setParam("last_joint", recorder) &&
                robot_recorder_.dumpParamFile())
            {
                is_using_temp_zero_offset_ = false;
                return true;
            }
            else {
                if (jtac_param_.getLastError() != SUCCESS)
                    last_error_ = jtac_param_.getLastError();
                else
                    last_error_ = robot_recorder_.getLastError();
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
    if (current_state_ < INITIALIZED) {last_error_ = NEED_INITIALIZATION; return false;}
    
    bool result = true;;
    FeedbackJointState fbjs;
    vector<double> last_joint;
    if (getCurrentJoint(fbjs)) {
        // if using normal zero-offset, compare robot_recorder with current joint
        if (!is_using_temp_zero_offset_) {
            if (robot_recorder_.getParam("last_joint", last_joint)) {
                bitmap = OFFSET_NORMAL;
                for (int loop = 0; loop < 6; ++loop) {
                    if (fabs(fbjs.position[loop] - last_joint[loop]) > offset_lost_threshold_[loop])
                        bitmap |= OFFSET_LOST << loop * 4;
                    else if (fabs(fbjs.position[loop] - last_joint[loop]) > offset_normal_threshold_[loop])
                        bitmap |= OFFSET_DEVIATE << loop * 4;
                }
            }
            else {
                last_error_ = robot_recorder_.getLastError();
                result = false;
            }
        }
        // if using temp zero-offset, compare temp_robot_recorder with current joint
        else {
            bitmap = OFFSET_NORMAL;
            for (int loop = 0; loop < 6; ++loop) {
                if (fabs(fbjs.position[loop] - temp_robot_recorder_[loop]) > offset_lost_threshold_[loop])
                    bitmap |= OFFSET_LOST << loop * 4;
                else if (fabs(fbjs.position[loop] - temp_robot_recorder_[loop]) > offset_normal_threshold_[loop])
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
                log.info("Expected joint <-> Current joint");
                log.info(" %lf\t\t%lf", last_joint[0], fbjs.position[0]);
                log.info(" %lf\t\t%lf", last_joint[1], fbjs.position[1]);
                log.info(" %lf\t\t%lf", last_joint[2], fbjs.position[2]);
                log.info(" %lf\t\t%lf", last_joint[3], fbjs.position[3]);
                log.info(" %lf\t\t%lf", last_joint[4], fbjs.position[4]);
                log.info(" %lf\t\t%lf", last_joint[5], fbjs.position[5]);
            }
            else {
                log.info("Expected joint <-> Current joint");
                log.info(" %lf\t\t%lf", temp_robot_recorder_[0], fbjs.position[0]);
                log.info(" %lf\t\t%lf", temp_robot_recorder_[1], fbjs.position[1]);
                log.info(" %lf\t\t%lf", temp_robot_recorder_[2], fbjs.position[2]);
                log.info(" %lf\t\t%lf", temp_robot_recorder_[3], fbjs.position[3]);
                log.info(" %lf\t\t%lf", temp_robot_recorder_[4], fbjs.position[4]);
                log.info(" %lf\t\t%lf", temp_robot_recorder_[5], fbjs.position[5]);
            }
        }
    }
    return result;
}

bool Calibrator::recordCurrentJoint(void) {
    if (current_state_ < INITIALIZED) {last_error_ = NEED_INITIALIZATION; return false;}

    FeedbackJointState fbjs;
    if (getCurrentJoint(fbjs)) {
        vector<double> data(fbjs.position, fbjs.position + sizeof(fbjs.position) / sizeof(double));
        return recordGivenJointImpl(data);
    }
    else {
        return false;
    }
}

bool Calibrator::recordGivenJoint(const Joint &joint)
{
    if (current_state_ < INITIALIZED) {last_error_ = NEED_INITIALIZATION; return false;}

    vector<double> data((double*)(&joint), (double*)(&joint) + sizeof(joint) / sizeof(double));
    return recordGivenJointImpl(data);
}

bool Calibrator::recordGivenJointImpl(vector<double> &joint)
{
    if (joint.size() == 6) {joint.push_back(0.0); joint.push_back(0.0);}
    if (joint.size() == 8) {
        if (!is_using_temp_zero_offset_) {
            return recordJointToRobotRecorder(joint);
        }
        else {
            return recordJointToTempRecorder(joint);
        }
    }
    else {
        last_error_ = INVALID_PARAMETER;
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
            last_error_ = robot_recorder_.getLastError();
            return false;
        }
    }
    else {
        last_error_ = robot_recorder_.getLastError();
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
    if (current_state_ < INITIALIZED) {last_error_ = NEED_INITIALIZATION; return false;}

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
        bool result = true;
        result = result && sendConfigData("zero_offset/");
        result = result && sendConfigData("gear_ratio/");
        result = result && sendConfigData("coupling_coefficient/");
        result = result && sendConfigData("trajectory_delay/");
        return result;
    }
    else {
        last_error_ = INVALID_PARAMETER;
        return false;    
    }
}

bool Calibrator::sendConfigData(const string &path) {
    if (current_state_ < INITIALIZED) {last_error_ = NEED_INITIALIZATION; return false;}

    int id;
    vector<double> data;

    if (jtac_param_.getParam(path + "id", id) && 
        jtac_param_.getParam(path + "data", data)) {
        return sendConfigDataImpl(id, data);
    }
    else {
        last_error_ = jtac_param_.getLastError();
        return false;
    }
    
}

bool Calibrator::sendConfigDataImpl(int id, const vector<double> &data) {
    ServiceRequest service_request;
    service_request.req_id = WRITE_BY_ID;
    int len = data.size();
    memcpy(&service_request.req_buff[0], (char*)&id, sizeof(id));
    // log.info("id=0x%x", *((int*)&service_request.req_buff[0]));
    memcpy(&service_request.req_buff[4], (char*)&len, sizeof(len));
    // log.info("len=%d", *((int*)&service_request.req_buff[4]));
    memcpy(&service_request.req_buff[8], (char*)&data[0], len * sizeof(double));
    // for (int cnt = 0; cnt < 8; ++cnt)
    //     log.info("data[%d]=%lf", cnt, *((double*)&service_request.req_buff[8 + 8 * cnt]));
    //if (clientSendRequest(mem_handle_, &service_request) == true)
    if (comm_interface_.send(&service_request, sizeof(service_request), COMM_DONTWAIT) == SUCCESS) {
        log.info("Request service ID=0x%x, sub ID=0x%x, data length=%d sended", service_request.req_id, id, len);
    }
    else {
        last_error_ = IPC_COMMUNICATION_ERROR;
        return false;
    }

    unsigned int loop_cnt = 0;
    ServiceResponse service_response;
    while (loop_cnt < 10) {
        usleep(5 * 1000);
        if (comm_interface_.recv(&service_response, sizeof(service_response), COMM_DONTWAIT) == SUCCESS) {
            break;
        }
        loop_cnt++;
    }

    if (loop_cnt < 10) {
        log.info("Response ID=0x%x received", service_response.res_id);
        usleep(10 * 1000);
        return true;
    }
    else {
        last_error_ = IPC_COMMUNICATION_ERROR;
        log.error("Request ID=0x%x get no response within %d ms, aborted.", service_request.req_id, loop_cnt * 5);
        return false;
    }
}

bool Calibrator::readConfigDataImpl(int id, vector<double> &data) {
    int length = data.size();
    
    ServiceRequest service_request;
    service_request.req_id = READ_BY_ID;
    
    memcpy(&service_request.req_buff[0], (char*)&id, sizeof(id));
    // log.info("id=0x%x", *((int*)&service_request.req_buff[0]));
    memcpy(&service_request.req_buff[4], (char*)&length, sizeof(length));
    // log.info("len=%d", *((int*)&service_request.req_buff[4]));
    if (comm_interface_.send(&service_request, sizeof(service_request), COMM_DONTWAIT) == SUCCESS)
        log.info("Request service ID=0x%x, sub ID=0x%x, data length=%d sended", service_request.req_id, id, length);
    else {
        last_error_ = IPC_COMMUNICATION_ERROR;
        return false;
    }
    
    unsigned int loop_cnt = 0;
    ServiceResponse service_response;
    while (loop_cnt < 10) {
        usleep(5 * 1000);
        if (comm_interface_.recv(&service_response, sizeof(service_response), COMM_DONTWAIT) == SUCCESS) {
            break;
        }
        loop_cnt++;
    }

    if (loop_cnt < 10) {
        log.info("Response ID=0x%x received", service_response.res_id);
        if (*((int*)(&service_response.res_buff[0])) == id && *((int*)(&service_response.res_buff[4])) == length) {
            for (int i = 0; i < length; ++i) {
                data[i] = *((double*)(&service_response.res_buff[8 + i * sizeof(double)]));
                // log.info("data[%d]=%lf", i, data[i]);
            }
            return true;
        }
        else {
            last_error_ = IPC_COMMUNICATION_ERROR;
            log.error("id or length mismatch");
            return false;
        }
    }
    else {
        last_error_ = IPC_COMMUNICATION_ERROR;
        log.error("Request ID=0x%x get no response within %d ms, aborted.", service_request.req_id, loop_cnt * 5);
        return false;
    }
}

}


