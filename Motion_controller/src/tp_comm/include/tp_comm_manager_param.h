#ifndef TP_COMM_MANAGER_PARAM
#define TP_COMM_MANAGER_PARAM

#include "parameter_manager/parameter_manager_param_group.h"
#include <string>
using namespace std;

namespace fst_comm
{
class TpCommManagerParam
{
public:
    TpCommManagerParam();
    ~TpCommManagerParam(){}

    bool loadParam();
    bool saveParam();

    // param to load & save
    int log_level_;

    int cycle_time_;    //ms
    int recv_buffer_size_;
    int send_buffer_size_;
    int rpc_list_max_size_;

private:
    fst_parameter::ParamGroup yaml_help_;
    string file_path_;
};
}

#endif


