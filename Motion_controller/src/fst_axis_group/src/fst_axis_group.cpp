#include "fst_axis_group.h"

using namespace fst_mc;
using namespace fst_hal;


FstAxisGroup::FstAxisGroup():
    BaseAxisGroup()
{

}

FstAxisGroup::~FstAxisGroup()
{

}

bool FstAxisGroup::init()
{
    return true;
}

bool FstAxisGroup::setEnable()
{
    return true;
}

bool FstAxisGroup::setDisable()
{
    return true;
}

bool FstAxisGroup::setHalt()
{
    return true;
}

bool FstAxisGroup::setQuickStop()
{
    return true;
}

bool FstAxisGroup::setHome()
{
    return true;
}

bool FstAxisGroup::setPause()
{
    return true;
}

bool FstAxisGroup::setContinue()
{
    return true;
}

bool FstAxisGroup::resetFault()
{
    return true;
}

bool FstAxisGroup::setPosition(/*something*/)
{
    return true;
}

bool FstAxisGroup::setCoordinate(int coordinate_index)
{
    return true;
}

bool FstAxisGroup::setDynamicCoordinate(int coordinate_index)
{
    return true;
}

bool FstAxisGroup::startDynamicCoordinate(int index)
{
    return true;
}

bool FstAxisGroup::stopDynamicCoordinate(int index)
{
    return true;
}

bool FstAxisGroup::setTool(int tool_index)
{
    return true;
}

bool FstAxisGroup::moveJAbsolute(/*something*/)
{
    return true;
}

bool FstAxisGroup::moveJRelative(/*something*/)
{
    return true;
}

bool FstAxisGroup::moveLAbsolute(/*something*/)
{
    return true;
}

bool FstAxisGroup::moveLRelative(/*something*/)
{
    return true;
}

bool FstAxisGroup::moveCAbsolute(/*something*/)
{
    return true;
}

bool FstAxisGroup::moveCRelative(/*something*/)
{
    return true;
}


