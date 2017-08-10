/*************************************************************************
	> File Name: motion_controller_offset_calibration.cpp
	> Author: 
	> Mail: 
	> Created Time: 2016年12月12日 星期一 14时30分05秒
 ************************************************************************/

#include <iostream>
#include <motion_controller/motion_controller_offset_calibration.h>
#include <motion_controller/motion_controller_error_code.h>
#include <parameter_manager/parameter_manager_param_group.h>
#include <parameter_manager/parameter_manager_error_code.h>
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
/*
Calibrator::Calibrator(void):log(private_log) {
    current_state_ = UNDEFINED;
    last_error_ = SUCCESS;
    mem_handle_ = -1;
}
*/
Calibrator::Calibrator(fst_log::Logger &inh_log):log(inh_log) {
    current_state_ = UNDEFINED;
    last_error_ = SUCCESS;
    mem_handle_ = -1;
}

Calibrator::~Calibrator(void) {
    return;
}

const unsigned int& Calibrator::getCurrentState(void) {
    return current_state_;
}

const ErrorCode& Calibrator::getLastError(void) {
    return last_error_;
}

bool Calibrator::initCalibrator(const string &jtac, const string &record) {
    if (current_state_ != UNDEFINED) {
        last_error_ = CALIBRATION_FAIL_IN_INIT;
        return false;
    }

    fst_parameter::ParamGroup param(jtac);
    if (param.getLastError() == SUCCESS) {
        jtac_param_file_ = jtac;
        if (!param.uploadParam()) {
            last_error_ = param.getLastError();
            return false;
        }
    }
    else {
        last_error_ = param.getLastError();
        return false;
    }

    param.loadParamFile(record);
    if (param.getLastError() == SUCCESS) {
        record_file_ = record;
        if (!param.uploadParam()) {
            last_error_ = param.getLastError();
            return false;
        }
    }
    else {
        last_error_ = param.getLastError();
        return false;
    }

    mem_handle_ = openMem(MEM_CORE);
    if (mem_handle_ == -1) {
        last_error_ = MOTION_FAIL_IN_INIT;
        return false;
    }

    if (comm_interface_.createChannel(IPC_REQ, "servo_param") != SUCCESS) {
        last_error_ = MOTION_FAIL_IN_INIT;
        return false;
    }

    current_state_ = INITIALIZED;
    return true;
}

bool Calibrator::reloadJTACParam(void) {
    fst_parameter::ParamGroup param(jtac_param_file_);
    if (param.getLastError() == SUCCESS) {
        if (!param.uploadParam()) {
            last_error_ = param.getLastError();
            return false;
        }
        else {
            return true;
        }
    }
    else {
        last_error_ = param.getLastError();
        return false;
    }
}

bool Calibrator::getCurrentJoint(FeedbackJointState &fbjs) {
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

bool Calibrator::getZeroOffsetFromBareCore(std::vector<double> &data) {
    return readConfigData("/fst_param/jtac/zero_offset", data);
}

bool Calibrator::setTemporaryZeroOffset(void) {
    FeedbackJointState fbjs;
    vector<double> old_offset_data;
    vector<double> new_offset_data;

    if (getCurrentJoint(fbjs) && getZeroOffsetFromBareCore(old_offset_data)) {
        if (old_offset_data.size() == 8) {
            new_offset_data.resize(8);
            new_offset_data[0] = fbjs.position[0] + old_offset_data[0];
            new_offset_data[1] = fbjs.position[1] + old_offset_data[1];
            new_offset_data[2] = fbjs.position[2] + old_offset_data[2];
            new_offset_data[3] = fbjs.position[3] + old_offset_data[3];
            new_offset_data[4] = fbjs.position[4] + old_offset_data[4];
            new_offset_data[5] = fbjs.position[5] + old_offset_data[5];
            new_offset_data[6] = old_offset_data[6];
            new_offset_data[7] = old_offset_data[7];
            
            log.info("old offset=%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", 
                     old_offset_data[0], old_offset_data[1], old_offset_data[2],
                     old_offset_data[3], old_offset_data[4], old_offset_data[5],
                     old_offset_data[6], old_offset_data[7]);
            log.info("joint=%lf, %lf, %lf, %lf, %lf, %lf",
                     fbjs.position[0], fbjs.position[1], fbjs.position[2],
                     fbjs.position[3], fbjs.position[4], fbjs.position[5]);
            log.info("new offset=%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", 
                     new_offset_data[0], new_offset_data[1], new_offset_data[2],
                     new_offset_data[3], new_offset_data[4], new_offset_data[5],
                     new_offset_data[6], new_offset_data[7]);
            
            int id;
            fst_parameter::ParamGroup::getRemoteParamImpl("/fst_param/jtac/zero_offset/id", id);
            
            if (sendConfigData(id, new_offset_data)) {
                current_state_ = NEED_CALIBRATE;
                return true;
            }
            else {
                return false;
            }
        }
    }
    return false;
}

bool Calibrator::recordZeroOffset(void) {
    if (current_state_ < INITIALIZED) {
        last_error_ = NEED_INITIALIZATION;
        return false;
    }
    if (current_state_ == CALIBRATED) {
        last_error_ = 0x66666666;
        return false;
    }

    FeedbackJointState fbjs;
    if (!getCurrentJoint(fbjs)) {
        return false;
    }

    std::vector<double> data(fbjs.position, fbjs.position + 6);
    data.push_back(0.0);
    data.push_back(0.0);
    
    fst_parameter::ParamGroup param(jtac_param_file_);
    if (param.getLastError() != SUCCESS) {
        last_error_ = param.getLastError();
        return false;
    }

    vector<double> old_offset_data;
    vector<double> new_offset_data;
    getZeroOffsetFromBareCore(old_offset_data);
    if (old_offset_data.size() == 8) {
        new_offset_data.resize(8);
        new_offset_data[0] = data[0] + old_offset_data[0];
        new_offset_data[1] = data[1] + old_offset_data[1];
        new_offset_data[2] = data[2] + old_offset_data[2];
        new_offset_data[3] = data[3] + old_offset_data[3];
        new_offset_data[4] = data[4] + old_offset_data[4];
        new_offset_data[5] = data[5] + old_offset_data[5];
        new_offset_data[6] = data[6] + old_offset_data[6];
        new_offset_data[7] = data[7] + old_offset_data[7];
        log.info("old offset=%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", 
                 old_offset_data[0], old_offset_data[1], old_offset_data[2],
                 old_offset_data[3], old_offset_data[4], old_offset_data[5],
                     old_offset_data[6], old_offset_data[7]);
        log.info("joint=%lf, %lf, %lf, %lf, %lf, %lf",
                 data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        log.info("new offset=%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", 
                 new_offset_data[0], new_offset_data[1], new_offset_data[2],
                 new_offset_data[3], new_offset_data[4], new_offset_data[5],
                 new_offset_data[6], new_offset_data[7]);
            
    }
    else {
        last_error_ = INVALID_PARAMETER;
        return false;
    }

    if (param.setParam("zero_offset/data", new_offset_data)) {
        if (param.dumpParamFile(jtac_param_file_)) {
            return true;
        }
        else {
            last_error_ = param.getLastError();
            return false;
        }
    }
    else {
        last_error_ = param.getLastError();
        return false;
    }
}

bool Calibrator::reviewCalibratedJoint(unsigned int &bitmap) {
    if (current_state_ < INITIALIZED) {
        last_error_ = NEED_INITIALIZATION;
        return false;
    }
    
    FeedbackJointState fbjs;
    if (!getCurrentJoint(fbjs)) {
        return false;
    }
    
    vector<double> normal_offset_threshold;
    vector<double> lost_offset_threshold;

    bool ret = true;
    ret = ret && fst_parameter::ParamGroup::getRemoteParamImpl(
                    "/fst_param/motion_controller/calibrator/normal_offset_threshold", normal_offset_threshold);
    ret = ret && fst_parameter::ParamGroup::getRemoteParamImpl(
                    "/fst_param/motion_controller/calibrator/lost_offset_threshold", lost_offset_threshold);

    if (ret == true && normal_offset_threshold.size() >= 6 && lost_offset_threshold.size() >= 6)
    {
        bitmap = OFFSET_NORMAL;
        for (int loop = 0; loop < 6; ++loop) {
            if (fabs(fbjs.position[loop]) > lost_offset_threshold[loop]) {
                bitmap |= OFFSET_LOST << loop * 4;
            }
            else if (fabs(fbjs.position[loop]) > normal_offset_threshold[loop]) {
                bitmap |= OFFSET_DEVIATE << loop * 4;
            }
            else {
                bitmap |= OFFSET_NORMAL << loop * 4;
            }
        }
        if (bitmap == OFFSET_NORMAL) {
            current_state_ = CALIBRATED;
        }
        else {
            current_state_ = NEED_CALIBRATE;
            log.info("Expected  Current");
            log.info("  0.0      %lf", fbjs.position[0]);
            log.info("  0.0      %lf", fbjs.position[1]);
            log.info("  0.0      %lf", fbjs.position[2]);
            log.info("  0.0      %lf", fbjs.position[3]);
            log.info("  0.0      %lf", fbjs.position[4]);
            log.info("  0.0      %lf", fbjs.position[5]);
        }
        return true;
    }
    else {
        last_error_ = INVALID_PARAMETER;
        return false;
    }
}

bool Calibrator::reviewLastJoint(unsigned int &bitmap) {
    if (current_state_ < INITIALIZED) {
        last_error_ = NEED_INITIALIZATION;
        return false;
    }
    
    FeedbackJointState fbjs;
    if (!getCurrentJoint(fbjs)) {
        return false;
    }
    
    vector<double> normal_offset_threshold;
    vector<double> lost_offset_threshold;
    vector<double> last_joint;
    bool ret = true;
    ret = ret && fst_parameter::ParamGroup::getRemoteParamImpl(
                    "/fst_param/motion_controller/calibrator/normal_offset_threshold", normal_offset_threshold);
    ret = ret && fst_parameter::ParamGroup::getRemoteParamImpl(
                    "/fst_param/motion_controller/calibrator/lost_offset_threshold", lost_offset_threshold);
    ret = ret && fst_parameter::ParamGroup::getRemoteParamImpl(
                    "/fst_param/robot_recorder/last_joint", last_joint);
    if (ret == true && normal_offset_threshold.size() >= 6 && lost_offset_threshold.size() >= 6 && last_joint.size() >= 6)
    {
        bitmap = OFFSET_NORMAL;
        for (int loop = 0; loop < 6; ++loop) {
            if (fabs(fbjs.position[loop] - last_joint[loop]) > lost_offset_threshold[loop]) {
                bitmap |= OFFSET_LOST << loop * 4;
            }
            else if (fabs(fbjs.position[loop] - last_joint[loop]) > normal_offset_threshold[loop]) {
                bitmap |= OFFSET_DEVIATE << loop * 4;
            }
            else {
                bitmap |= OFFSET_NORMAL << loop * 4;
            }
        }
        if (bitmap == OFFSET_NORMAL) {
            current_state_ = CALIBRATED;
        }
        else {
            current_state_ = NEED_CALIBRATE;
            log.info("Expected  Current");
            log.info(" %lf    %lf", last_joint[0], fbjs.position[0]);
            log.info(" %lf    %lf", last_joint[1], fbjs.position[1]);
            log.info(" %lf    %lf", last_joint[2], fbjs.position[2]);
            log.info(" %lf    %lf", last_joint[3], fbjs.position[3]);
            log.info(" %lf    %lf", last_joint[4], fbjs.position[4]);
            log.info(" %lf    %lf", last_joint[5], fbjs.position[5]);
        }
        return true;
    }
    else {
        last_error_ = INVALID_PARAMETER;
        return false;
    }
}

bool Calibrator::recordLastJoint(void) {
    FeedbackJointState fbjs;
    if (!getCurrentJoint(fbjs)) {
        return false;
    }
    JointValues joint;
    joint.j1 = fbjs.position[0];
    joint.j2 = fbjs.position[1];
    joint.j3 = fbjs.position[2];
    joint.j4 = fbjs.position[3];
    joint.j5 = fbjs.position[4];
    joint.j6 = fbjs.position[5];
    return recordLastJoint(joint);
}

bool Calibrator::recordLastJoint(const JointValues &joint) {
    fst_parameter::ParamGroup recorder(record_file_);
    if (recorder.getLastError() != SUCCESS) {
        last_error_ = recorder.getLastError();
        return false;
    }

    time_t time_now = time(NULL);
    tm *local = localtime(&time_now);
    char buf[128];
    memset(buf, 0, sizeof(buf));
    strftime(buf, 64, "%Y-%m-%d %H:%M:%S", local);
    string temp(buf);
    vector<double> data;
    data.push_back(joint.j1); data.push_back(joint.j2); data.push_back(joint.j3);
    data.push_back(joint.j4); data.push_back(joint.j5); data.push_back(joint.j6);
    data.push_back(0.0); data.push_back(0.0);
    recorder.setParam("last_time", temp);
    recorder.setParam("last_joint", data);
    recorder.dumpParamFile(record_file_);
    if (recorder.getLastError() != SUCCESS) {
        last_error_ = recorder.getLastError();
        return false;
    }
    return true;
}

bool Calibrator::transmitJtacParam(void) {
    if (current_state_ < INITIALIZED) {
        last_error_ = NEED_INITIALIZATION;
        return false;
    }
    
    if (sendConfigData("/fst_param/jtac/zero_offset") &&
        sendConfigData("/fst_param/jtac/transmission_ratio") &&
        sendConfigData("/fst_param/jtac/coupling_coefficient") &&
        sendConfigData("/fst_param/jtac/trajectory_delay")) {
        return true;
    }
    else {
        last_error_ = 0x66666666;
        return false;
    }
}

bool Calibrator::sendConfigData(const string &path) {
    int id;
    vector<double> data;
    fst_parameter::ParamGroup::getRemoteParamImpl(path + "/id", id);
    fst_parameter::ParamGroup::getRemoteParamImpl(path + "/data", data);
    
    return sendConfigData(id, data);
}

bool Calibrator::sendConfigData(int id, const vector<double> &data) {
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
    if (comm_interface_.send(&service_request, sizeof(service_request), IPC_DONTWAIT) == SUCCESS) {
        log.info("Request ID=0x%x sended", service_request.req_id);
    }
    else {
        return false;
    }
    usleep(50 * 1000);
    ServiceResponse service_response;
    if (comm_interface_.recv(&service_response, sizeof(service_response), IPC_DONTWAIT) == SUCCESS) {
        log.info("Response ID=0x%x received", service_response.res_id);
        usleep(100 * 1000);
        return true;
    }
    else {
        log.error("No response");
        return false;
    }
}

bool Calibrator::readConfigData(const string &path, vector<double> &data) {
    int id;
    data.clear();
    fst_parameter::ParamGroup::getRemoteParamImpl(path + "/id", id);
    fst_parameter::ParamGroup::getRemoteParamImpl(path + "/data", data);
    int length = data.size();
    
    ServiceRequest service_request;
    service_request.req_id = READ_BY_ID;
    
    memcpy(&service_request.req_buff[0], (char*)&id, sizeof(id));
    // log.info("id=0x%x", *((int*)&service_request.req_buff[0]));
    memcpy(&service_request.req_buff[4], (char*)&length, sizeof(length));
    // log.info("len=%d", *((int*)&service_request.req_buff[4]));
    if (comm_interface_.send(&service_request, sizeof(service_request), IPC_DONTWAIT) == SUCCESS)
        log.info("Request ID=0x%x sended", service_request.req_id);
    usleep(100 * 1000);
    ServiceResponse service_response;
    if (comm_interface_.recv(&service_response, sizeof(service_response), IPC_DONTWAIT) == SUCCESS) {
        log.info("Response ID=0x%x received", service_response.res_id);
        if (*((int*)(&service_response.res_buff[0])) == id && *((int*)(&service_response.res_buff[4])) == length) {
            for (int i = 0; i < length; ++i) {
                data[i] = *((double*)(&service_response.res_buff[8 + i * sizeof(double)]));
                // log.info("data[%d]=%lf", i, data[i]);
            }
            return true;
        }
        else {
            log.error("id or length mismatch");
        }
    }
    else {
        log.error("no response");
    }
    return false;
}

}


