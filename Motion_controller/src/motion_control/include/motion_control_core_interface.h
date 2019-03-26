/*************************************************************************
	> File Name: motion_control_core_interface.h
	> Author: 
	> Mail: 
	> Created Time: 2018年08月08日 星期三 15时35分55秒
 ************************************************************************/

#ifndef _MOTION_CONTROL_CORE_INTERFACE_H
#define _MOTION_CONTROL_CORE_INTERFACE_H

#include <vector>
#include <comm_interface/core_interface.h>
#include <struct_to_mem/struct_service_request.h>
#include <struct_to_mem/struct_service_response.h>
#include <service_actions/response_actions.h>
#include <comm_interface/comm_interface.h>
#include <motion_control_datatype.h>
#include <error_code.h>
#include <common_enum.h>

namespace fst_mc
{

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

    bool initInterface(void);
    
    bool sendPoint(void);
    bool isPointCacheEmpty(void);
    bool clearPointCache(void);
    bool fillPointCache(TrajectoryPoint *points, size_t length, PointProperty proerty);

    bool getLatestJoint(basic_alg::Joint &joint, ServoState &state);

    bool resetBareCore(void);
    bool stopBareCore(void);

    bool setConfigData(int id, const std::vector<int> &data);
    bool setConfigData(int id, const std::vector<double> &data);
    bool getConfigData(int id, std::vector<double> &data);
    bool getEncoder(std::vector<int> &data);
    bool getControlPosition(double *data, size_t len);
    bool readVersion(char *buffer, size_t size);

  private:
    bool sendRequest(fst_comm_interface::CommInterface &comm, const ServiceRequest &req);
    bool recvResponse(fst_comm_interface::CommInterface &comm, ServiceResponse &res);
    PointCache  point_cache_;

    fst_core_interface::CoreInterface   core_interface_;
    fst_comm_interface::CommInterface   command_interface_;
    //fst_comm_interface::CommInterface   jtac_param_interface_;
};









}

#endif
