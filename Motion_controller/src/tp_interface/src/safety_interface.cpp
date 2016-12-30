/**
 * @file safety_interface.cpp
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-12-05
 */

#include "safety/safety.h"
#include "safety_interface.h"
#include "common/common.h"
#include <boost/thread/mutex.hpp>


SafetyInterface::SafetyInterface()
{
    valid_flag_ = false;
#ifdef CROSS_PLATFORM
    openSafety();
#endif
}
SafetyInterface::~SafetyInterface()
{
#ifdef CROSS_PLATFORM 
    closeSafety();
#endif
}

char SafetyInterface::getDigitalValue(int frame_id, int index, U64 &err)
{
    if (isSafetyValid() == false)
    {
        return 0;
    }
	char frame_data = 0;
#ifdef CROSS_PLATFORM
    frame_data = getSafety(frame_id, &err);
#endif 
    return ((frame_data >> index) & 1);
}
U64 SafetyInterface::setDigitalOutput(int frame_id, int index, char data)
{
    if (isSafetyValid() == false)
    {
        return 0;
    }
    U64 err;
	char frame_data = 0;
#ifdef CROSS_PLATFORM
    frame_data = getSafety(frame_id, &err);
#endif
    if(err != FST_SUCCESS)
    {
        return err;
    }

    frame_data = ((frame_data & (~(1 << index))) | ((data & 1) << index));
#ifdef CROSS_PLATFORM
    return setSafety(frame_data, frame_id);
#else
	return FST_SUCCESS;
#endif
}

char SafetyInterface::getDIStatusPause(U64 &err)
{
    return getDigitalValue(SAFETY_INPUT_SECONDFRAME, 7, err);
}
char SafetyInterface::getDIStatusBrake3(U64 &err)
{
    return getDigitalValue(SAFETY_INPUT_SECONDFRAME, 2, err);
}
char SafetyInterface::getDIStatusBrake2(U64 &err)
{
    return getDigitalValue(SAFETY_INPUT_SECONDFRAME, 1, err);
}
char SafetyInterface::getDIStatusBrake1(U64 &err)
{
    return getDigitalValue(SAFETY_INPUT_SECONDFRAME, 0, err);
}
char SafetyInterface::getDIDeadmanPanic(U64 &err)
{
    return getDigitalValue(SAFETY_INPUT_THIRDFRAME, 0, err);
}
char SafetyInterface::getDIDeadmanNormal(U64 &err)
{
    return getDigitalValue(SAFETY_INPUT_THIRDFRAME, 1, err);
}

char SafetyInterface::getDITPManual(U64 &err)
{
    return getDigitalValue(SAFETY_INPUT_THIRDFRAME, 2, err);
}
char SafetyInterface::getDITPLimitedManual(U64 &err)
{
    return getDigitalValue(SAFETY_INPUT_THIRDFRAME, 3, err);
}
char SafetyInterface::getDITPAuto(U64 &err)
{
    return getDigitalValue(SAFETY_INPUT_THIRDFRAME, 4, err);
}

char SafetyInterface::getDIStatusEStop(U64 &err)
{
    return getDigitalValue(SAFETY_INPUT_THIRDFRAME, 5, err);
}
char SafetyInterface::getDIStatusLimitedEStop(U64 &err)
{
    return getDigitalValue(SAFETY_INPUT_THIRDFRAME, 6, err);
}
char SafetyInterface::getDIStatusExtEStop(U64 &err)
{
    return getDigitalValue(SAFETY_INPUT_THIRDFRAME, 7, err);
}
char SafetyInterface::getDOStatusWarn(U64 &err)
{
    return getDigitalValue(SAFETY_OUTPUT_SECONDFRAME, 0, err);
}
U64 SafetyInterface::setDOStatusWarn(char data)
{
    return setDigitalOutput(SAFETY_OUTPUT_SECONDFRAME, 0, data);
}

U64 SafetyInterface::setSafetyHeartBeat()
{
    if (isSafetyValid() == false)
    {
        return FST_SUCCESS;
    }
    return autorunSafetyData();
}

bool SafetyInterface::isSafetyValid()
{
   boost::mutex			mutex_; 
   boost::mutex::scoped_lock lock(mutex_);

   return valid_flag_;
}

