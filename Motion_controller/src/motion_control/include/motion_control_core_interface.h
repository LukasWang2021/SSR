/*************************************************************************
	> File Name: motion_control_core_interface.h
	> Author: 
	> Mail: 
	> Created Time: 2018年08月08日 星期三 15时35分55秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_CORE_INTERFACE_H
#define _MOTION_CONTROL_CORE_INTERFACE_H

#include <vector>
//TODO#include <comm_interface/core_interface.h>
//TODO#include <struct_to_mem/struct_service_request.h>
//TODO#include <struct_to_mem/struct_service_response.h>
//TODO#include <service_actions/response_actions.h>
//TODO#include <comm_interface/comm_interface.h>
#include <motion_control_datatype.h>
#include <common_error_code.h>
#include <common_enum.h>

namespace fst_mc
{
#define JC_POINT_NUM 10
#define START_POINT 1
#define END_POINT 2
#define MID_POINT 0
typedef struct 
{
    double angle[6];
    double omega[6];
    double alpha[6];
    double inertia[6];
    int point_position;
}Points;
typedef struct 
{
    Points points[JC_POINT_NUM];
    int total_points;
}JointCommand;

struct PointCache
{
    bool is_empty;
    JointCommand cache;
    PointProperty property;
};

class BareCoreInterface
{
  public:
    BareCoreInterface();
    ~BareCoreInterface();

    bool initInterface(uint32_t joint_num);
    
    bool sendPoint(void);
    bool isPointCacheEmpty(void);
    bool clearPointCache(void);
    bool fillPointCache(TrajectoryPoint *points, size_t length, PointProperty proerty);

    bool getLatestJoint(basic_alg::Joint &joint, uint32_t (&encoder_state)[NUM_OF_JOINT], ServoState &state);

    bool resetBareCore(void);
    bool stopBareCore(void);

    bool setConfigData(int id, const std::vector<int> &data);
    bool setConfigData(int id, const std::vector<double> &data);
    bool getConfigData(int id, std::vector<double> &data);
    bool getEncoder(std::vector<int> &data);
    bool getEncoderError(std::vector<int> &data);
    bool resetEncoderError(size_t index);
    bool resetEncoderError(void);
    bool getControlPosition(double *data, size_t len);
    bool setOmegaFilter(uint32_t filter_half_length, double *weights, size_t weights_size);
    bool readVersion(char *buffer, size_t size);
    bool downloadServoParam(uint32_t addr, const char *data, uint32_t length);

  private:
    //TODObool sendRequest(fst_comm_interface::CommInterface &comm, const ServiceRequest &req);
    //TODObool recvResponse(fst_comm_interface::CommInterface &comm, ServiceResponse &res);
    uint32_t joint_num_;
    PointCache  point_cache_;

    //TODOfst_core_interface::CoreInterface   core_interface_;
    //TODOfst_comm_interface::CommInterface   command_interface_;
    //fst_comm_interface::CommInterface   jtac_param_interface_;
};









}

#endif
