#ifndef BASE_AXIS_GROUP_H
#define BASE_AXIS_GROUP_H


#include "axis_group_model.h"
#include "base_device.h"
#include <vector>

namespace fst_mc
{

typedef struct
{
    int device_index;
    fst_hal::BaseDevice* device_ptr;
}AxisInfo;

class BaseAxisGroup
{
public:
    BaseAxisGroup();
    ~BaseAxisGroup();

    virtual bool init() = 0;
    bool addAxisToGroup(int device_index, fst_hal::BaseDevice* device_ptr);
    bool removeAxisFromGroup(int device_index);
    bool unGroupAllAxes();
    bool bindModelToGroup(AxisGroupModel& model);
    bool unbindModelToGroup();
    bool checkValid();
    bool isValid();
    fst_hal::BaseDevice* getAxisByPos(int pos);
    AxisGroupModel* getModel();
    
private:
    std::vector<AxisInfo> axis_list_;
    AxisGroupModel model_;
    bool is_valid_;
};

}

#endif

