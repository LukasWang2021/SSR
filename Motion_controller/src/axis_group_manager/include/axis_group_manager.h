#ifndef AXIS_GROUP_MANAGER_H
#define AXIS_GROUP_MANAGER_H


#include "base_axis_group.h"
#include "axis_group_manager_param.h"
#include "common_log.h"
#include "xml_help.h"
#include <map>


namespace fst_mc
{

typedef struct
{
    int dummy;
}AxisGroupConfig;

class AxisGroupManager
{
public:
    AxisGroupManager();
    ~AxisGroupManager();

    bool init();    
    bool addAxisGroup(int axis_group_index, fst_mc::AxisGroupConfig device_config);
    bool removeAxisGroup(int axis_group_index);
    BaseAxisGroup* getAxisGroupByIndex(int axis_group_index);   

private:
    AxisGroupManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    fst_base::XmlHelp xml_help_;
    std::map<int, BaseAxisGroup*> axis_group_map_;  // map for axis_group_index & BaseAxisGroup*
};

}

#endif


