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
#include <motion_controller/fst_datatype.h>
#include <comm_interface/comm_interface.h>
#include <struct_to_mem/struct_feedback_joint_states.h>


typedef int MemoryHandle;
typedef unsigned long long int ErrorCode;
namespace fst_controller {
static const unsigned int NEED_CALIBRATE    = 0x5A55;
static const unsigned int CALIBRATED        = 0x5A56;

static const unsigned int OFFSET_NORMAL     = 0;
static const unsigned int OFFSET_DEVIATE    = 1;
static const unsigned int OFFSET_LOST       = 2;

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
    Calibrator(void);
    ~Calibrator(void);
    
    const unsigned int& getCurrentState(void);
    const ErrorCode& getLastError(void);
    bool initCalibrator(const std::string &jtac = "share/motion_controller/config/jtac.yaml",
                        const std::string &record = "share/motion_controller/config/robot_recorder.yaml");
    bool reloadJTACParam(void);
    bool getCurrentJoint(FeedbackJointState &fbjs);
    bool transmitJtacParam(void);
    bool getZeroOffsetFromBareCore(std::vector<double> &data);
    bool setTemporaryZeroOffset(void);
    bool recordZeroOffset(void);
    bool recordZeroOffset(const std::vector<double> &data);
    bool reviewCalibratedJoint(unsigned int &bitmap);
    bool reviewLastJoint(unsigned int &bitmap);
    bool recordLastJoint(void);
    bool recordLastJoint(const JointValues &joint);

  protected:
    bool sendConfigData(const std::string &path);
    bool sendConfigData(int id, const std::vector<double> &data);
    bool readConfigData(const std::string &path, std::vector<double> &data);
    
  private:
    std::string jtac_param_file_;
    std::string record_file_;
    ErrorCode last_error_;
    unsigned int current_state_;
    MemoryHandle mem_handle_;
    fst_comm_interface::CommInterface comm_interface_;
};


}

#endif
