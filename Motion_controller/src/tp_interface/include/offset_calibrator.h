/*************************************************************************
	> File Name: motion_controller_offset_calibration.h
	> Author: 
	> Mail: 
	> Created Time: 2016年12月12日 星期一 14时30分56秒
 ************************************************************************/

#ifndef _MOTION_CONTROLLER_OFFSET_CALIBRATION_H
#define _MOTION_CONTROLLER_OFFSET_CALIBRATION_H

#include <string>
#include <vector>
#include <fst_datatype.h>
#include <log_manager/log_manager_logger.h>
#include <parameter_manager/parameter_manager_param_group.h>
#include <parameter_manager/parameter_manager_error_code.h>
#include "common.h"

namespace fst_controller {
static const unsigned int UNCALIBRATED      = 0x5A54;
static const unsigned int NEED_CALIBRATE    = 0x5A55;
static const unsigned int CALIBRATED        = 0x5A56;

static const unsigned int OFFSET_NORMAL     = 0x0;
static const unsigned int OFFSET_DEVIATE    = 0x1;
static const unsigned int OFFSET_LOST       = 0x2;

static const unsigned int OFFSET_DEVIATE_MASK   = 0x111111;
static const unsigned int OFFSET_LOST_MASK      = 0x222222;


class Calibrator {


  public:
    Calibrator();
    ~Calibrator(void);
    
    bool initCalibrator(const std::string &path = "/opt/fst_controller/runtime/");
    bool calibrateZeroOffset(unsigned int &calibrate_result);
    const unsigned int& getCurrentState(void);
   // const ErrorCode& getLastError(void);
    bool sendConfigData(const std::string &path);
    bool transmitJtacParam(const std::string &param = "all");
    bool setTempZeroOffset(void);
    bool setZeroOffset(void);
    bool recordCurrentJoint(void);
    bool recordGivenJoint(const Joint &joint);
    bool reviewCurrentJoint(unsigned int &bitmap);
    bool isUsingTempZeroOffset(void);

  protected:
    bool getZeroOffsetFromBareCore(std::vector<double> &data);
    bool recordGivenJointImpl(std::vector<double> &joint);
    bool recordJointToRobotRecorder(const std::vector<double> &joint);
    bool recordJointToTempRecorder(const std::vector<double> &joint);
    bool setTempZeroOffsetImpl(const Joint &target_joint);
    bool setZeroOffsetImpl(const Joint &target_joint);
    
  private:
    bool is_using_temp_zero_offset_;
    bool buildRecorderFromTemplate(const std::string &file);
    std::vector<double> temp_zero_offset_;
    std::vector<double> temp_robot_recorder_;

    fst_parameter::ParamGroup offset_param_;
    fst_parameter::ParamGroup robot_recorder_;
    std::vector<double> offset_normal_threshold_;
    std::vector<double> offset_lost_threshold_;
    unsigned int current_state_;
   // U64 last_error_;
    bool checkZeroOffset(unsigned int &calibrate_result);
};


}

#endif
