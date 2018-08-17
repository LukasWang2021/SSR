/*************************************************************************
	> File Name: motion_controller_offset_calibrator.h
	> Author: 
	> Mail: 
	> Created Time: 2016年12月12日 星期一 14时30分56秒
 ************************************************************************/

#ifndef _MOTION_CONTROLLER_OFFSET_CALIBRATOR_H
#define _MOTION_CONTROLLER_OFFSET_CALIBRATOR_H

#include <string>
#include <vector>
#include <base_datatype.h>
#include <common_log.h>
#include <motion_control_core_interface.h>
#include <parameter_manager/parameter_manager_param_group.h>
#include <parameter_manager/parameter_manager_error_code.h>

namespace fst_mc
{

enum CalibrateState
{
    MOTION_NORMAL,
    MOTION_LIMITED,
    MOTION_FORBIDDEN,
};

enum OffsetState
{
    OFFSET_NORMAL,
    OFFSET_DEVIATE,
    OFFSET_LOST,
};

enum OffsetMask
{
    OFFSET_UNMASK,
    OFFSET_MASKED,
};

static const int NEED_SAVE = 1;
static const int UNNEED_SAVE = 0;


class Calibrator
{
  public:
    Calibrator(size_t joint_num, BareCoreInterface *bare_core, fst_log::Logger *plog);
    ~Calibrator(void);
    
    ErrorCode initCalibrator(const std::string &path = "/opt/fst_controller/runtime/");
    ErrorCode checkOffset(CalibrateState *cali_stat, OffsetState *offset_stat);


    ErrorCode calibrateOffset(void);
    ErrorCode calibrateOffset(size_t index);
    ErrorCode calibrateOffset(const size_t *pindex, size_t length);


    ErrorCode saveOffset(void);
    ErrorCode saveJoint(void);


    ErrorCode maskOffsetLostError(void);
    ErrorCode setOffsetState(size_t index, OffsetState stat);


    void getOffset(double *offset);
    CalibrateState getCalibrateState(void);
    ErrorCode sendJtacParam(const std::string &param = "all");

    ErrorCode saveReference(void);
    ErrorCode fastCalibrate(void);
    ErrorCode fastCalibrate(size_t index);
    ErrorCode fastCalibrate(const size_t *pindex, size_t length);

    
  private:
    void checkOffset(Joint curr_jnt, Joint last_jnt, OffsetState *offset_stat);
    double calculateOffset(double current_offset, double current_joint, double target_joint);
    double calculateOffsetEasy(double gear_ratio, double ref_offset, unsigned int ref_encoder, unsigned int cur_encoder);
    ErrorCode saveGivenJoint(const Joint &joint);
    ErrorCode sendConfigData(const std::string &path);
    ErrorCode buildRecordFile(const std::string &file);
    ErrorCode getOffsetFromBareCore(std::vector<double> &data);

    inline char* printDBLine(const int *data, char *buffer, size_t length);
    inline char* printDBLine(const double *data, char *buffer, size_t length);


    CalibrateState              current_state_;
    fst_parameter::ParamGroup   offset_param_;
    fst_parameter::ParamGroup   robot_recorder_;


    double      normal_threshold_[NUM_OF_JOINT];
    double      lost_threshold_[NUM_OF_JOINT];
    OffsetMask  offset_mask_[NUM_OF_JOINT];
    OffsetState offset_stat_[NUM_OF_JOINT];
    double      zero_offset_[NUM_OF_JOINT];
    int         offset_need_save_[NUM_OF_JOINT];

    size_t joint_num_;
    BareCoreInterface *bare_core_ptr_;
    fst_log::Logger *log_ptr_;
};


}

#endif
