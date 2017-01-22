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
    valid_flag_ = true;
    memset((char*)&din_frm1_, 0, sizeof(din_frm1_));
    memset((char*)&din_frm2_, 0, sizeof(din_frm2_));
    memset((char*)&dout_frm1_, 0, sizeof(dout_frm1_));
    memset((char*)&dout_frm2_, 0, sizeof(dout_frm2_));
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

U32 SafetyInterface::getDIFrm1()
{
    boost::mutex::scoped_lock lock(mutex_);
    return *(U32*)&din_frm1_;
}
U32 SafetyInterface::getDIFrm2()
{
    boost::mutex::scoped_lock lock(mutex_);
    return *(U32*)&din_frm2_;
}
U32 SafetyInterface::getDOFrm1()
{
    boost::mutex::scoped_lock lock(mutex_);
    return *(U32*)&dout_frm1_;
}
U32 SafetyInterface::getDOFrm2()
{
    boost::mutex::scoped_lock lock(mutex_);
    return *(U32*)&dout_frm2_;
}


char SafetyInterface::getDIDec()
{
    boost::mutex::scoped_lock lock(mutex_);
    return din_frm1_.byte4.decelerate;
}
char SafetyInterface::getDIOutage1()
{
    boost::mutex::scoped_lock lock(mutex_);
    return din_frm1_.byte4.outage1;
}
char SafetyInterface::getDIOutage0()
{
    boost::mutex::scoped_lock lock(mutex_);
    return din_frm1_.byte4.outage0;
}

char SafetyInterface::getDIBrake3()
{
    boost::mutex::scoped_lock lock(mutex_);
    return din_frm1_.byte4.brake3;
}
char SafetyInterface::getDIBrake2()
{
    boost::mutex::scoped_lock lock(mutex_);
    return din_frm1_.byte4.brake2;
}
char SafetyInterface::getDIBrake1()
{
    boost::mutex::scoped_lock lock(mutex_);
    return din_frm1_.byte4.brake1;
}
char SafetyInterface::getDIDeadmanPanic()
{
    boost::mutex::scoped_lock lock(mutex_);
    return din_frm2_.byte5.deadman_panic;
}
char SafetyInterface::getDIDeadmanNormal()
{
    boost::mutex::scoped_lock lock(mutex_);
    return din_frm2_.byte5.deadman_normal;
}

char SafetyInterface::getDITPManual()
{
    boost::mutex::scoped_lock lock(mutex_);
    return din_frm2_.byte5.manual;
}
char SafetyInterface::getDITPLimitedManual()
{
    boost::mutex::scoped_lock lock(mutex_);
    return din_frm2_.byte5.lmt_manual;
}
char SafetyInterface::getDITPAuto()
{
    boost::mutex::scoped_lock lock(mutex_);
    return din_frm2_.byte5.automatic;
}

char SafetyInterface::getDIEStop()
{
    boost::mutex::scoped_lock lock(mutex_);
    return din_frm2_.byte5.estop;
}
char SafetyInterface::getDILimitedStop()
{
    boost::mutex::scoped_lock lock(mutex_);
    return din_frm2_.byte5.lmt_stop;
}
char SafetyInterface::getDIExtEStop()
{
    boost::mutex::scoped_lock lock(mutex_);
    return din_frm2_.byte5.ext_estop;
}
char SafetyInterface::getDIHWReset()
{
    boost::mutex::scoped_lock lock(mutex_);
    return din_frm2_.byte6.hw_reset;
}
char SafetyInterface::getDIServoAlarm()
{
    boost::mutex::scoped_lock lock(mutex_);
    return din_frm2_.byte6.hw_reset;
}
char SafetyInterface::getDIContactorFeadback()
{
    boost::mutex::scoped_lock lock(mutex_);
    return din_frm2_.byte6.hw_reset;
}
char SafetyInterface::getDISafetyDoorStop()
{
    boost::mutex::scoped_lock lock(mutex_);
    return din_frm2_.byte6.hw_reset;
}
char SafetyInterface::getDOSWAlarm()
{
    boost::mutex::scoped_lock lock(mutex_);
    return dout_frm1_.byte4.sw_alarm;
}
U64 SafetyInterface::setDOSWAlarm(char data)
{
    if (isSafetyValid() == false)
    {
        return FST_SUCCESS;
    }
    boost::mutex::scoped_lock lock(mutex_);
    SafetyBoardDOFrm1 out = dout_frm1_;
    out.byte4.sw_alarm = data;
    return setSafety(*(int*)&out, SAFETY_OUTPUT_FIRSTFRAME);
}

U64 SafetyInterface::setBrakeOn()
{
    if (isSafetyValid() == false)
    {
        return FST_SUCCESS;
    }
    boost::mutex::scoped_lock lock(mutex_);
    SafetyBoardDOFrm1 out = dout_frm1_;
    out.byte4.disble_brake = 1;

}

U64 SafetyInterface::setBrakeOff()
{
    if (isSafetyValid() == false)
    {
        return FST_SUCCESS;
    }
    boost::mutex::scoped_lock lock(mutex_);
    SafetyBoardDOFrm1 out = dout_frm1_;
    out.byte4.disble_brake = 0;

}


char SafetyInterface::getDOSafetyStopConf()
{
    boost::mutex::scoped_lock lock(mutex_);
    return dout_frm2_.byte5.safety_stop_config;
}
U64 SafetyInterface::setDOSafetyStopConf(char data)
{
    if (isSafetyValid() == false)
    {
        return FST_SUCCESS;
    }
    boost::mutex::scoped_lock lock(mutex_);
    SafetyBoardDOFrm2 out = dout_frm2_;
    out.byte5.safety_stop_config = data;
    return setSafety(*(int*)&out, SAFETY_OUTPUT_SECONDFRAME);
}

char SafetyInterface::getDOExtEStopConf()
{
    boost::mutex::scoped_lock lock(mutex_);
    return dout_frm2_.byte5.ext_estop_config;
}
U64 SafetyInterface::setDOExtEStopConf(char data)
{
    if (isSafetyValid() == false)
    {
        return FST_SUCCESS;
    }
    boost::mutex::scoped_lock lock(mutex_);
    SafetyBoardDOFrm2 out = dout_frm2_;
    out.byte5.ext_estop_config = data;
    return setSafety(*(int*)&out, SAFETY_OUTPUT_SECONDFRAME);
}

char SafetyInterface::getDOLmtStopConf()
{
    boost::mutex::scoped_lock lock(mutex_);
    return dout_frm2_.byte5.lmt_stop_config;
}
U64 SafetyInterface::setDOLmtStopConf(char data)
{
    if (isSafetyValid() == false)
    {
        return FST_SUCCESS;
    }
    boost::mutex::scoped_lock lock(mutex_);
    SafetyBoardDOFrm2 out = dout_frm2_;
    out.byte5.sw_reset = data;
    setSafety(*(int*)&out, SAFETY_OUTPUT_SECONDFRAME);
}

U64 SafetyInterface::softwareReset()
{
    if (isSafetyValid() == false)
    {
        return FST_SUCCESS;
    }
    boost::mutex::scoped_lock lock(mutex_);
    SafetyBoardDOFrm2 out = dout_frm2_;
    lock.unlock();
    out.byte5.sw_reset = 1;
    setSafety(*(int*)&out, SAFETY_OUTPUT_SECONDFRAME);
    usleep(300*1000); //6 cycles of safety board
    out.byte5.sw_reset = 0;
    return setSafety(*(int*)&out, SAFETY_OUTPUT_SECONDFRAME);
}



U64 SafetyInterface::setSafetyHeartBeat()
{
    if (isSafetyValid() == false)
    {
        return FST_SUCCESS;
    }
    U64 result = autorunSafetyData(); 
    if (result == FST_SUCCESS)
    {
        boost::mutex::scoped_lock lock(mutex_);
        U32 data = getSafety(SAFETY_INPUT_FIRSTFRAME, &result);
        //din_frm1_ = *(SafetyBoardDIFrm1*)&data;
        memcpy((char*)&din_frm1_, (char*)&data, sizeof(U32));
        data = getSafety(SAFETY_INPUT_SECONDFRAME, &result);
        //din_frm2_ = *(SafetyBoardDIFrm2*)&data;
        memcpy((char*)&din_frm2_, (char*)&data, sizeof(U32));
        /*data = getSafety(SAFETY_OUTPUT_FIRSTFRAME, &result);*/
        ////dout_frm1_ = *(SafetyBoardDOFrm1*)&data;
        //memcpy((char*)&dout_frm1_, (char*)&data, sizeof(U32));
        //data = getSafety(SAFETY_OUTPUT_SECONDFRAME, &result);
       //// dout_frm2_ = *(SafetyBoardDOFrm2*)&data;
        /*memcpy((char*)&dout_frm2_, (char*)&data, sizeof(U32));*/
    }
    return result;
}

bool SafetyInterface::isSafetyValid()
{
   boost::mutex::scoped_lock lock(mutex_);

   return valid_flag_;
}

