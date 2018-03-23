/**
 * @file safety_interface.cpp
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-12-05
 */

#include "safety/safety.h"
#include "safety_interface.h"


SafetyInterface::SafetyInterface()
{
    valid_flag_ = false;
    memset((char*)&din_frm2_, 0, sizeof(din_frm2_));
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

U32 SafetyInterface::getDIFrm2()
{
    return *(U32*)&din_frm2_;
}
U32 SafetyInterface::getDOFrm2()
{
    return *(U32*)&dout_frm2_;
}

bool SafetyInterface::isDIFrmChanged()
{
    static U32 pre_frame_data;
    U32 frm_data = getDIFrm2();
    if (pre_frame_data == frm_data)
    {
        return false;
    }
    
    pre_frame_data = frm_data;
    return true;
}

char SafetyInterface::getDIDec()
{
    return din_frm2_.load().byte6.decelerate;
}
char SafetyInterface::getDIOutage1()
{
    return din_frm2_.load().byte5.outage1;
}
char SafetyInterface::getDIOutage0()
{
    return din_frm2_.load().byte5.outage0;
}

char SafetyInterface::getDIBrake3()
{
    return din_frm2_.load().byte5.brake3;
}
char SafetyInterface::getDIBrake2()
{
    return din_frm2_.load().byte5.brake2;
}
char SafetyInterface::getDIBrake1()
{
    if (isSafetyValid() == false)
    {
        return 0;
    }
    return din_frm2_.load().byte5.brake1;
}
char SafetyInterface::getDIDeadmanPanic()
{
    return din_frm2_.load().byte7.deadman_panic;
}
char SafetyInterface::getDIDeadmanNormal()
{
    return din_frm2_.load().byte7.deadman_normal;
}

char SafetyInterface::getDITPManual()
{
    return din_frm2_.load().byte6.manual;
}
char SafetyInterface::getDITPLimitedManual()
{
    return din_frm2_.load().byte6.lmt_manual;
}
char SafetyInterface::getDITPAuto()
{
    return din_frm2_.load().byte6.automatic;
}

char SafetyInterface::getDITPEStop()
{
    return din_frm2_.load().byte7.tp_estop;
}
char SafetyInterface::getDILimitedStop()
{
    return din_frm2_.load().byte7.lmt_stop;
}
char SafetyInterface::getDIExtEStop()
{
    return din_frm2_.load().byte7.ext_estop;
}
char SafetyInterface::getDICore1Reset()
{
    return din_frm2_.load().byte6.core1_reset;
}
char SafetyInterface::getDICore1Alarm()
{
    return din_frm2_.load().byte8.core1_alarm;
}
char SafetyInterface::getDIAlarm()
{
    return din_frm2_.load().byte8.alarming;
}
char SafetyInterface::getDISafetyDoorStop()
{
    return din_frm2_.load().byte7.safetydoor_stop;
}
char SafetyInterface::getDOType0Stop()
{
    return dout_frm2_.load().byte5.core0_sw0;
}
U64 SafetyInterface::setDOType0Stop(char data)
{
    if (isSafetyValid() == false)
    {
        return TPI_SUCCESS;
    }
    SafetyBoardDOFrm2 out = dout_frm2_;
    out.byte5.core0_sw0 = data;
    return setSafety(*(int*)&out, SAFETY_OUTPUT_SECONDFRAME);
}

char SafetyInterface::getDOType1Stop()
{
    return dout_frm2_.load().byte5.core0_sw1;
}
U64 SafetyInterface::setDOType1Stop(char data)
{
    if (isSafetyValid() == false)
    {
        return TPI_SUCCESS;
    }
    SafetyBoardDOFrm2 out = dout_frm2_;
    out.byte5.core0_sw1 = data;
    return setSafety(*(int*)&out, SAFETY_OUTPUT_SECONDFRAME);
}

char SafetyInterface::getDOType2Stop()
{
    return dout_frm2_.load().byte5.core0_sw2;
}
U64 SafetyInterface::setDOType2Stop(char data)
{
    if (isSafetyValid() == false)
    {
        return TPI_SUCCESS;
    }
    SafetyBoardDOFrm2 out = dout_frm2_;
    out.byte5.core0_sw2 = data;
    return setSafety(*(int*)&out, SAFETY_OUTPUT_SECONDFRAME);
}

char SafetyInterface::getDOSafetyStopConf()
{
    return dout_frm2_.load().byte5.safedoor_stop_config;
}
U64 SafetyInterface::setDOSafetyStopConf(char data)
{
    if (isSafetyValid() == false)
    {
        return TPI_SUCCESS;
    }
    SafetyBoardDOFrm2 out = dout_frm2_;
    out.byte5.safedoor_stop_config = data;
    return setSafety(*(int*)&out, SAFETY_OUTPUT_SECONDFRAME);
}

char SafetyInterface::getDOExtEStopConf()
{
    return dout_frm2_.load().byte5.ext_estop_config;
}
U64 SafetyInterface::setDOExtEStopConf(char data)
{
    if (isSafetyValid() == false)
    {
        return TPI_SUCCESS;
    }
    SafetyBoardDOFrm2 out = dout_frm2_;
    out.byte5.ext_estop_config = data;
    return setSafety(*(int*)&out, SAFETY_OUTPUT_SECONDFRAME);
}

char SafetyInterface::getDOLmtStopConf()
{
    return dout_frm2_.load().byte5.lmt_stop_config;
}
U64 SafetyInterface::setDOLmtStopConf(char data)
{
    if (isSafetyValid() == false)
    {
        return TPI_SUCCESS;
    }
    SafetyBoardDOFrm2 out = dout_frm2_;
    out.byte5.lmt_stop_config = data;
    return setSafety(*(int*)&out, SAFETY_OUTPUT_SECONDFRAME);
}

void SafetyInterface::reset()
{
    setDOType0Stop(0);
    setDOType1Stop(0);
    setDOType2Stop(0);
}


U64 SafetyInterface::setSafetyHeartBeat()
{
    if (isSafetyValid() == false)
    {
        return TPI_SUCCESS;
    }
    U64 result = autorunSafetyData(); 
    if (result == TPI_SUCCESS)
    {
        //U32 data = getSafety(SAFETY_INPUT_FIRSTFRAME, &result);
        //din_frm1_ = *(SafetyBoardDIFrm1*)&data;
        //memcpy((char*)&din_frm1_, (char*)&data, sizeof(U32));
        U32 data = getSafety(SAFETY_INPUT_SECONDFRAME, &result);
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
   return valid_flag_;
}

bool SafetyInterface::isSafetyAlarm()
{
    static char pre_alarm = 1;

    char alarm = getDIAlarm();    
    if (alarm != pre_alarm)
    {
        FST_INFO("cur alarm:%d, pre_alarm:%d",alarm, pre_alarm);
    }

    if ((pre_alarm == 0) && (alarm == 1))
    {
        pre_alarm = alarm;
        return true;
    }

    pre_alarm = alarm;
    return false;
}
