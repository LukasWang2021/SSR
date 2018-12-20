
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <memory.h>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "safety.h"
#include "fst_safety_device.h"
#include "error_monitor.h"

#define CROSS_PLATFORM


using namespace fst_hal;

/**
 * @brief : get local ip address
 *
 * @return : the ip address in the form of string
 */
static std::string getLocalIP()
{
	int fd;
    struct ifreq ifr;

    char iface[] = "eth0";
     
    fd = socket(AF_INET, SOCK_DGRAM, 0);
 
    //Type of address to retrieve - IPv4 IP address
    ifr.ifr_addr.sa_family = AF_INET;
 
    //Copy the interface name in the ifreq structure
    strncpy(ifr.ifr_name , iface , IFNAMSIZ-1);
 
    ioctl(fd, SIOCGIFADDR, &ifr);
 
    close(fd);
 
    //display result
    //printf("%s - %s\n" , iface , inet_ntoa(( (struct sockaddr_in *)&ifr.ifr_addr )->sin_addr) );
	std::string ret = inet_ntoa(( (struct sockaddr_in *)&ifr.ifr_addr )->sin_addr);
	return ret;
}


FstSafetyDevice::FstSafetyDevice(int address):
    BaseDevice(address, fst_hal::DEVICE_TYPE_FST_SAFETY),
    log_ptr_(NULL),
    param_ptr_(NULL)
{
    log_ptr_ = new fst_log::Logger();
    param_ptr_ = new FstSafetyDeviceParam();
}

FstSafetyDevice::~FstSafetyDevice()
{
#ifdef CROSS_PLATFORM 
    closeSafety();
#endif

    if(log_ptr_ != NULL)
    {
        delete log_ptr_;
        log_ptr_ = NULL;
    }

    if(param_ptr_ != NULL)
    {
        delete param_ptr_;
        param_ptr_ = NULL;
    }
}

bool FstSafetyDevice::init()
{
    int iRet = 0 ;
    
    // enable safty
    //valid_flag_ = true;
    
    // disable safty
    // 
    std::string str_addr = getLocalIP();
	if(str_addr.substr(0,3) == "192")
	{
    	FST_INFO("Use Fake Safety");
        valid_flag_ = false;
	}
    else
	{
    	FST_INFO("Use True Safety");
	    valid_flag_ = true;
	}
    
    setValid(true);
    memset((char*)&din_frm2_, 0, sizeof(din_frm2_));
    memset((char*)&dout_frm2_, 0, sizeof(dout_frm2_));
#ifdef CROSS_PLATFORM
    iRet = openSafety();
#endif
	if(iRet != 0)
	{
        setValid(false);
        FST_ERROR("init FstSafetyDevice failed");
	}
    
    startThread();
    return true;
}

//------------------------------------------------------------
// Function:    startThread
// Summary: start a thread.
// In:      None.
// Out:     None.
// Return:  None.
//------------------------------------------------------------
void FstSafetyDevice::startThread(void)
{
    safety_thread = boost::thread(boost::bind(&FstSafetyDevice::runThread, this));
    //io_thread.timed_join(boost::posix_time::milliseconds(100)); // check the thread running or not.
}

//------------------------------------------------------------
// Function:    runThread
// Summary: main function of io thread.
// In:      None.
// Out:     None.
// Return:  None.
//------------------------------------------------------------
void FstSafetyDevice::runThread(void)
{
    try
    {   FST_INFO("safety thread running...");
        while (true)
        {
            setSafetyHeartBeat();

            // set interruption point.
            boost::this_thread::sleep(boost::posix_time::microseconds(LOOP_CYCLE));
        }
    }
    catch (boost::thread_interrupted &)
    {
        std::cout<<"~Stop safety_thread Thread Safely.~"<<std::endl;
    }
}

FstSafetyDevice::FstSafetyDevice():
    BaseDevice(0, fst_hal::DEVICE_TYPE_INVALID)
{

}

void FstSafetyDevice::updateThreadFunc()
{

}


U32 FstSafetyDevice::getDIFrm2()
{
    return *(U32*)&din_frm2_;
}
U32 FstSafetyDevice::getDOFrm2()
{
    return *(U32*)&dout_frm2_;
}

bool FstSafetyDevice::isDIFrmChanged()
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

// char FstSafetyDevice::getDIDec()
// {
//    return din_frm2_.load().byte6.slowdown;
// }
char FstSafetyDevice::getDIOutage1()
{
    return din_frm2_.load().byte5.outage1;
}
char FstSafetyDevice::getDIOutage0()
{
    return din_frm2_.load().byte5.outage0;
}

char FstSafetyDevice::getDIBrake3()
{
    return din_frm2_.load().byte5.brake3;
}
char FstSafetyDevice::getDIBrake2()
{
    return din_frm2_.load().byte5.brake2;
}
char FstSafetyDevice::getDIBrake1()
{
    if (isSafetyValid() == false)
    {
        // FST_INFO("getDIBrake1 Error \n");
        return 0;
    }
    return din_frm2_.load().byte5.brake1;
}
char FstSafetyDevice::getDIDeadmanPanic()
{
    return din_frm2_.load().byte7.deadman_panic;
}
char FstSafetyDevice::getDIDeadmanNormal()
{
    return din_frm2_.load().byte7.deadman_normal;
}

char FstSafetyDevice::getDITPManual()
{
    return din_frm2_.load().byte6.usermode_man;
}
char FstSafetyDevice::getDITPLimitedManual()
{
    return din_frm2_.load().byte6.usermode_limit;
}
char FstSafetyDevice::getDITPAuto()
{
    return din_frm2_.load().byte6.usermode_auto;
}
//
//qianjin add for user mode
int FstSafetyDevice::getDITPUserMode()
{
    int val;

    val = 0;
    if( din_frm2_.load().byte6.usermode_auto) val |= 0x1;
    if( din_frm2_.load().byte6.usermode_man) val |= 0x2;
    if( din_frm2_.load().byte6.usermode_limit) val |= 0x4;
	
//	FST_INFO("getDITPUserMode: safety_interface_ :: din_frm2_: %08X", *(U32*)&din_frm2_);
//	FST_INFO("getDITPUserMode: safety_interface_ :: val: %d", val);

    if(val == 0x1) return 1;
    if(val == 0x2) return 3;
    if(val == 0x4) return 2;
    return 0;
}

char FstSafetyDevice::getDITPEStop()
{
    return din_frm2_.load().byte7.tp_estop;
}
char FstSafetyDevice::getDILimitedStop()
{
    return din_frm2_.load().byte7.lmt_stop;
}
char FstSafetyDevice::getDIExtEStop()
{
    return din_frm2_.load().byte7.ext_estop;
}
char FstSafetyDevice::getDICore1Reset()
{
    return din_frm2_.load().byte6.core1_reset;
}
char FstSafetyDevice::getDICore1Alarm()
{
    return din_frm2_.load().byte8.core1_alarm;
}
char FstSafetyDevice::getDIAlarm()
{
    return din_frm2_.load().byte8.alarming;
}
char FstSafetyDevice::getDISafetyDoorStop()
{
    return din_frm2_.load().byte7.safetydoor_stop;
}
char FstSafetyDevice::getDOType0Stop()
{
    return dout_frm2_.load().byte5.core0_sw0;
}
U64 FstSafetyDevice::setDOType0Stop(char data)
{
    if (isSafetyValid() == false)
    {
        // FST_INFO("setDOType0Stop Error set %c\n", data);
        return SUCCESS;
    }
    SafetyBoardDOFrm2 out = dout_frm2_;
    out.byte5.core0_sw0 = data;
        FST_INFO("setSafety set 0x%X\n", *(int*)&out);
    return setSafety(*(int*)&out, SAFETY_OUTPUT_SECONDFRAME);
}

char FstSafetyDevice::getDOType1Stop()
{
    return dout_frm2_.load().byte5.core0_sw1;
}
U64 FstSafetyDevice::setDOType1Stop(char data)
{
    if (isSafetyValid() == false)
    {
        // FST_INFO("setDOType1Stop Error set %c\n", data);
        return SUCCESS;
    }
    SafetyBoardDOFrm2 out = dout_frm2_;
    out.byte5.core0_sw1 = data;
        FST_INFO("setSafety set 0x%X\n", *(int*)&out);
    return setSafety(*(int*)&out, SAFETY_OUTPUT_SECONDFRAME);
}

char FstSafetyDevice::getDOType2Stop()
{
    return dout_frm2_.load().byte5.core0_sw2;
}
U64 FstSafetyDevice::setDOType2Stop(char data)
{
    if (isSafetyValid() == false)
    {
        // FST_INFO("setDOType2Stop Error set %c\n", data);
        return SUCCESS;
    }
    SafetyBoardDOFrm2 out = dout_frm2_;
    out.byte5.core0_sw2 = data;
        FST_INFO("setDOType1Stop set 0x%X\n", *(int*)&out);
    return setSafety(*(int*)&out, SAFETY_OUTPUT_SECONDFRAME);
}

char FstSafetyDevice::getDOSafetyStopConf()
{
    return dout_frm2_.load().byte5.safedoor_stop_config;
}
U64 FstSafetyDevice::setDOSafetyStopConf(char data)
{
    if (isSafetyValid() == false)
    {
        // FST_INFO("setDOSafetyStopConf Error set %c\n", data);
        return SUCCESS;
    }
    SafetyBoardDOFrm2 out = dout_frm2_;
    out.byte5.safedoor_stop_config = data;
        FST_INFO("setDOSafetyStopConf set 0x%X\n", *(int*)&out);
    return setSafety(*(int*)&out, SAFETY_OUTPUT_SECONDFRAME);
}

char FstSafetyDevice::getDOExtEStopConf()
{
    return dout_frm2_.load().byte5.ext_estop_config;
}
U64 FstSafetyDevice::setDOExtEStopConf(char data)
{
    if (isSafetyValid() == false)
    {
        // FST_INFO("setDOExtEStopConf Error set %c\n", data);
        return SUCCESS;
    }
    SafetyBoardDOFrm2 out = dout_frm2_;
    out.byte5.ext_estop_config = data;
        FST_INFO("setSafety set 0x%X\n", *(int*)&out);
    return setSafety(*(int*)&out, SAFETY_OUTPUT_SECONDFRAME);
}

char FstSafetyDevice::getDOLmtStopConf()
{
    return dout_frm2_.load().byte5.lmt_stop_config;
}
U64 FstSafetyDevice::setDOLmtStopConf(char data)
{
    if (isSafetyValid() == false)
    {
        // FST_INFO("setDOLmtStopConf Error set %c\n", data);
        return SUCCESS;
    }
    SafetyBoardDOFrm2 out = dout_frm2_;
    out.byte5.lmt_stop_config = data;
        FST_INFO("setSafety set 0x%X\n", *(int*)&out);
    return setSafety(*(int*)&out, SAFETY_OUTPUT_SECONDFRAME);
}

void FstSafetyDevice::reset()
{
    setDOType0Stop(0);
    setDOType1Stop(0);
    setDOType2Stop(0);
}


U64 FstSafetyDevice::setSafetyHeartBeat()
{
    if (isSafetyValid() == false)
    {
        // FST_INFO("setSafetyHeartBeat Error \n");
        return SUCCESS;
    }
    U64 result = autorunSafetyData(); 
    if (result == SUCCESS)
    {
        //U32 data = getSafety(SAFETY_INPUT_FIRSTFRAME, &result);
        //din_frm1_ = *(SafetyBoardDIFrm1*)&data;
        //memcpy((char*)&din_frm1_, (char*)&data, sizeof(U32));
        U32 data = getSafety(SAFETY_INPUT_SECONDFRAME, &result);
        // FST_INFO("getSafety return 0x%X", data);
        // printf("getSafety set 0x%X", data);
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

bool FstSafetyDevice::isSafetyValid()
{
   return valid_flag_;
}

bool FstSafetyDevice::isSafetyAlarm()
{
    static char pre_alarm = 1;

    char alarm = getDIAlarm();    
    if (alarm != pre_alarm)
    {
        FST_INFO("cur alarm:%d, pre_alarm:%d\n",alarm, pre_alarm);
    }

    if ((pre_alarm == 0) && (alarm == 1))
    {
        pre_alarm = alarm;
        return true;
    }

    pre_alarm = alarm;
    return false;
}


