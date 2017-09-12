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
#include <trajplan/fst_datatype.h>
#include <comm_interface/comm_interface.h>
#include <struct_to_mem/struct_feedback_joint_states.h>
#include <log_manager/log_manager_logger.h>
#include <parameter_manager/parameter_manager_param_group.h>
#include <parameter_manager/parameter_manager_error_code.h>

typedef int MemoryHandle;
namespace fst_controller {
static const unsigned int NEED_CALIBRATE    = 0x5A55;
static const unsigned int CALIBRATED        = 0x5A56;

static const unsigned int OFFSET_NORMAL     = 0x0;
static const unsigned int OFFSET_DEVIATE    = 0x1;
static const unsigned int OFFSET_LOST       = 0x2;

static const unsigned int OFFSET_DEVIATE_MASK   = 0x111111;
static const unsigned int OFFSET_LOST_MASK      = 0x222222;
/*
static const unsigned int J1_OFFSET_NORMAL  = 0x000000;
static const unsigned int J1_OFFSET_DEVIATE = 0x000001;
static const unsigned int J1_OFFSET_LOST    = 0x000002;
static const unsigned int J2_OFFSET_NORMAL  = 0x000000;
static const unsigned int J2_OFFSET_DEVIATE = 0x000010;
static const unsigned int J2_OFFSET_LOST    = 0x000020;
static const unsigned int J3_OFFSET_NORMAL  = 0x000000;
static const unsigned int J3_OFFSET_DEVIATE = 0x000100;
static const unsigned int J3_OFFSET_LOST    = 0x000200;
static const unsigned int J4_OFFSET_NORMAL  = 0x000000;
static const unsigned int J4_OFFSET_DEVIATE = 0x001000;
static const unsigned int J4_OFFSET_LOST    = 0x002000;
static const unsigned int J5_OFFSET_NORMAL  = 0x000000;
static const unsigned int J5_OFFSET_DEVIATE = 0x010000;
static const unsigned int J5_OFFSET_LOST    = 0x020000;
static const unsigned int J6_OFFSET_NORMAL  = 0x000000;
static const unsigned int J6_OFFSET_DEVIATE = 0x100000;
static const unsigned int J6_OFFSET_LOST    = 0x200000;
*/

class Calibrator {


  public:
    // Calibrator(void);
    Calibrator(fst_log::Logger &inh_log);
    ~Calibrator(void);
    
    bool initCalibrator(const std::string &path = "config/");
    const unsigned int& getCurrentState(void);
    const ErrorCode& getLastError(void);
    bool getCurrentJoint(FeedbackJointState &fbjs);
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
    bool sendConfigDataImpl(int id, const std::vector<double> &data);
    bool readConfigDataImpl(int id, std::vector<double> &data);
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
    ErrorCode last_error_;
    unsigned int current_state_;
    MemoryHandle mem_handle_;
    fst_comm_interface::CommInterface comm_interface_;
    fst_log::Logger &log;
};


}

#endif
