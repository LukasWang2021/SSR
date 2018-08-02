#ifndef FST_AXIS_GROUP_H
#define FST_AXIS_GROUP_H


#include "common_log.h"
#include "base_axis_group.h"
#include "fst_axis_group_param.h"


namespace fst_mc
{
class FstAxisGroup : public BaseAxisGroup
{
public:
    FstAxisGroup();
    ~FstAxisGroup();

    bool init();
    
    bool setEnable();
    bool setDisable();
    bool setHalt();
    bool setQuickStop();
    bool setHome();
    bool setPause();
    bool setContinue();
    bool resetFault();
    bool setPosition(/*something*/);

    bool setCoordinate(int coordinate_index);
    bool setDynamicCoordinate(int coordinate_index);
    bool startDynamicCoordinate(int index);
    bool stopDynamicCoordinate(int index);  
    bool setTool(int tool_index);

    bool moveJAbsolute(/*something*/);
    bool moveJRelative(/*something*/);
    bool moveLAbsolute(/*something*/);
    bool moveLRelative(/*something*/);
    bool moveCAbsolute(/*something*/);
    bool moveCRelative(/*something*/);
    
private:
    FstAxisGroupParam* param_ptr_;
    fst_log::Logger* log_ptr_;
}; 


}

#endif

