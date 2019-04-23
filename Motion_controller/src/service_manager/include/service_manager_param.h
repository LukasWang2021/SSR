#ifndef SERVICE_MANAGER_PARAM_H
#define SERVICE_MANAGER_PARAM_H


#include "parameter_manager/parameter_manager_param_group.h"

namespace fst_service_manager
{
class ServiceManagerParam
{
public:
    ServiceManagerParam();
    ~ServiceManagerParam();

    bool loadParam();
    bool saveParam();

    // param to load & save
    int log_level_;
    int cycle_time_;    // thread cycle time, ms
    int max_barecore_timeout_count_;
    int heartbeat_with_barecore_count_;
    int max_send_resp_count_;


private:
    fst_parameter::ParamGroup yaml_help_;
    std::string file_path_;
};

}


#endif

