#include "touch_play.h"
#include "rpc_help.h"
#include "touch_test.h"
#include "touch_interface.h"
#include "err_proc.h"
#include <HD/hd.h>
#include <HL/hl.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <mutex>
#include <assert.h>
#include <math.h>
#include <corecrt_math_defines.h>

typedef struct
{
    HDdouble tf[16];
    hduVector3Dd position;
    hduVector3Dd velocity;
    hduVector3Dd angle_joint;
	int key_value;
}HdFeedBackInfo_t;

static HHD hHD;
static HHLRC hHLRC;
static HDErrorInfo error;
static int calibrationStyle;
static int key_num;
static HdFeedBackInfo_t hd_fdbck_info;
static std::mutex mutex_key_value;

static HLuint effect;
static char *err_buff;
static int force_feedback_enable_ = 0;

HDCallbackCode HDCALLBACK UpdateCalibrationCallback(void *pUserData);
HDCallbackCode HDCALLBACK CalibrationStatusCallback(void *pUserData);
HDCallbackCode HDCALLBACK DevicePositionCallback(void *pUserData);
HDCallbackCode HDCALLBACK ButtonStatusCallback(void *pUserData);

static HDenum GetCalibrationStatus();
static int CheckCalibration(HDenum calibrationStyle);
static void PrintDevicePosition(char *buff, int size);
static int DeviceKeyValue_Get(void);
/******************************************************************************
 Servo loop thread callback.  Computes a force effect.
******************************************************************************/
void HLCALLBACK computeForceCB(HDdouble force[3], HLcache* cache, void* userdata)
{
	double force_tmp[6];
	int size = 6*sizeof(double);
	// Get the time delta since the last update.
	HDdouble instRate;
	hdGetDoublev(HD_INSTANTANEOUS_UPDATE_RATE, &instRate);
	HDdouble deltaT = 1.0 / instRate;

	// use rpc get force
	if (force_feedback_enable_ && (rpc_help_getForce(force_tmp, size) == 0))
	{
		//printf("force_val: %lf %lf %lf\n", force_tmp[0], force_tmp[1], force_tmp[2]);
		//force transformation
		force[0] = force_tmp[0];
		force[1] = force_tmp[1];
		force[2] = force_tmp[2];
	}
	else {
		force[0] = 0.0;
		force[1] = 0.0;
		force[2] = 0.0;
	}
}

/******************************************************************************
 Servo loop thread callback called when the effect is stopped.
******************************************************************************/
void HLCALLBACK stopEffectCB(HLcache* cache, void* userdata)
{
	//fprintf(stdout, "Custom effect stopped\n");
}


int DeviceInit(void)
{
	hHD = hdInitDevice(HD_DEFAULT_DEVICE);
	if (HD_DEVICE_ERROR(error = hdGetError())) 
    {
		PushErrInfo(ERR_TOUCH, "Call HD failed! (hdInitDevice)");
        return -1;
    }
	//pthread_mutex_init(&mutex_key_value,NULL);

	hdMakeCurrentDevice(hHD);

	hHLRC = hlCreateContext(hHD);
	hlMakeCurrent(hHLRC);

	hlDisable(HL_USE_GL_MODELVIEW);

	effect = hlGenEffects(1);

	hlBeginFrame();

	hlCallback(HL_EFFECT_COMPUTE_FORCE, (HLcallbackProc)computeForceCB, NULL);

	hlStartEffect(HL_EFFECT_CALLBACK, effect);

	hlEndFrame();

	return 0;
}

int DeviceCalib(void)
{
	int supportedCalibrationStyles;
	
	//printf("Calibration\n");
	
	/* Choose a calibration style.  Some devices may support multiple types of 
	  calibration.	In that case, prefer auto calibration over inkwell 
	  calibration, and prefer inkwell calibration over reset encoders. */
	hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
	if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
	{
	   calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
	}
	if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
	{
	   calibrationStyle = HD_CALIBRATION_INKWELL;
	}
	if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
	{
	   calibrationStyle = HD_CALIBRATION_AUTO;
	}

	/* Some haptic devices only support manual encoder calibration via a
	  hardware reset. This requires that the endpoint be placed at a known
	  physical location when the reset is commanded. For the PHANTOM haptic
	  devices, this means positioning the device so that all links are
	  orthogonal. Also, this reset is typically performed before the servoloop
	  is running, and only technically needs to be performed once after each
	  time the device is plugged in. */
	if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET)
	{
	   //printf("Please prepare for manual calibration by\n");
	   //printf("placing the device at its reset position.\n\n");
	   //printf("Press any key to continue...\n");

	   //getch();

	   hdUpdateCalibration(calibrationStyle);
	   if (hdCheckCalibration() == HD_CALIBRATION_OK)
	   {
		   //printf("Calibration complete.\n\n");
	   }
	   if (HD_DEVICE_ERROR(error = hdGetError()))
	   {
		   PushErrInfo(ERR_TOUCH, "Call HD failed! (hdUpdateCalibration)");
		   return -1;			
	   }
	}
	return 0;
}

void DeviceCommConfig(int freq)
{
	hdSetSchedulerRate(freq);
}

int DeviceStart(void)
{
	int key_temp;
	hdStartScheduler();
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		PushErrInfo(ERR_TOUCH, "Call HD failed! (hdStartScheduler)");
		return -1;			 
	}

	/* Some haptic devices are calibrated when the gimbal is placed into
	   the device inkwell and updateCalibration is called.	This form of
	   calibration is always performed after the servoloop has started 
	   running. */
	if (calibrationStyle  == HD_CALIBRATION_INKWELL)
	{
		if (GetCalibrationStatus() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
		{
			//printf("Please place the device into the inkwell ");
			//printf("for calibration.\n\n");
		}
	}
	/*printf("Press any key to start.\n\n");
	while(1)
    {
		hdScheduleSynchronous(ButtonStatusCallback, &key_temp,
	        HD_DEFAULT_SCHEDULER_PRIORITY);	
		printf("key_num:%d\n",key_num);	
		if(key_temp ==1)
		{
			break;	
		}
		break;
    } */
    /* Loop until key press. */
	return 0;
}


int DevicePullData(char *buff, int size)
{
	if (CheckCalibration(calibrationStyle)<0)
	//if(0)
	{ 
		PushErrInfo(ERR_TOUCH, "Call HD failed! (CheckCalibration)");
        return -1;
    }else{
		PrintDevicePosition(buff,size);
		return 0;
    }
	return 0;
}

int DeviceStop(void)
{
    hdStopScheduler();
	return 0;
}

int DeviceDisable(void)
{
	hlBeginFrame();
	hlStopEffect(effect);
	hlEndFrame();

	hlDeleteEffects(effect, 1);

	hdDisableDevice(hHD);
	//pthread_mutex_destroy(&mutex_key_value);
	return 0;
}

int DeviceKeyValue_Get(void)
{
	hdScheduleSynchronous(ButtonStatusCallback, &key_num,
				HD_DEFAULT_SCHEDULER_PRIORITY); 
	return key_num;
}

/******************************************************************************
 Begin Scheduler callbacks
 */

HDCallbackCode HDCALLBACK UpdateCalibrationCallback(void *pUserData)
{
    HDenum *calibrationStyle = (HDenum*) pUserData;

    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE)
    {
        hdUpdateCalibration(*calibrationStyle);
    }

    return HD_CALLBACK_DONE;
}

HDCallbackCode HDCALLBACK CalibrationStatusCallback(void *pUserData)
{
    HDenum *pStatus = (HDenum *) pUserData;

    hdBeginFrame(hdGetCurrentDevice());
    *pStatus = hdCheckCalibration();
    hdEndFrame(hdGetCurrentDevice());

    return HD_CALLBACK_DONE;
}

HDCallbackCode HDCALLBACK ButtonStatusCallback(void *pUserData)
{
    int *pButton = (int*)(pUserData);
    hdBeginFrame(hdGetCurrentDevice());
    hdGetIntegerv(HD_CURRENT_BUTTONS,pButton);
    hdEndFrame(hdGetCurrentDevice());

    return HD_CALLBACK_DONE;
}

HDCallbackCode HDCALLBACK DeviceCallback(void *pUserData)
{
    HdFeedBackInfo_t *pfdbck= (HdFeedBackInfo_t *) pUserData;
    hdBeginFrame(hdGetCurrentDevice());
    hdGetDoublev(HD_CURRENT_POSITION, pfdbck->position);
    hdGetDoublev(HD_CURRENT_VELOCITY, pfdbck->velocity);
    hdGetDoublev(HD_CURRENT_TRANSFORM, pfdbck->tf);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, pfdbck->angle_joint);
	hdGetIntegerv(HD_CURRENT_BUTTONS,&pfdbck->key_value);
    hdEndFrame(hdGetCurrentDevice());

    return HD_CALLBACK_DONE;
}

HDenum GetCalibrationStatus()
{
    HDenum status;
    hdScheduleSynchronous(CalibrationStatusCallback, &status,
                          HD_DEFAULT_SCHEDULER_PRIORITY);
    return status;
}

int CheckCalibration(HDenum calibrationStyle)
{
    HDenum status = GetCalibrationStatus();
    
    if (status == HD_CALIBRATION_OK)
    {
        return 0;
    }
    else if (status == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
    {
        printf("Calibration requires manual input...\n");
        return -1;
    }
    else if (status == HD_CALIBRATION_NEEDS_UPDATE)
    {
        hdScheduleSynchronous(UpdateCalibrationCallback, &calibrationStyle,
            HD_DEFAULT_SCHEDULER_PRIORITY);

        if (HD_DEVICE_ERROR(hdGetError()))
        {
            printf("\nFailed to update calibration.\n\n");
             return -1;
        }
        else
        {
            printf("\nCalibration updated successfully.\n\n");
            return 0;
        }
    }
    else
    {
        assert(!"Unknown calibration status");
        return -1;
    }
	return 0;
}

void PrintDevicePosition(char *buff, int size)
{
    hdScheduleSynchronous(DeviceCallback, &hd_fdbck_info,
        HD_MAX_SCHEDULER_PRIORITY);
#if 0
    snprintf(buff,size,"%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf%c%lf\n",hd_fdbck_info.position[0],',',hd_fdbck_info.position[1],',',hd_fdbck_info.position[2],',',hd_fdbck_info.velocity[0],',',hd_fdbck_info.velocity[1],',',hd_fdbck_info.velocity[2],',',    
		hd_fdbck_info.angle_joint[0],',',hd_fdbck_info.angle_joint[1],',',hd_fdbck_info.angle_joint[2],',',
		hd_fdbck_info.tf[0],',',hd_fdbck_info.tf[1],',',hd_fdbck_info.tf[2],',',hd_fdbck_info.tf[3],',',hd_fdbck_info.tf[4],',',hd_fdbck_info.tf[5],',',hd_fdbck_info.tf[6],',',hd_fdbck_info.tf[7],',',hd_fdbck_info.tf[8],',',hd_fdbck_info.tf[9],',',hd_fdbck_info.tf[10],',',hd_fdbck_info.tf[11],',',hd_fdbck_info.tf[12],',',hd_fdbck_info.tf[13],',',hd_fdbck_info.tf[14],',',hd_fdbck_info.tf[15]);
#else
	memcpy(buff, &hd_fdbck_info.tf[0], size);
	
	if (mutex_key_value.try_lock())
	{
		key_num = hd_fdbck_info.key_value;
		mutex_key_value.unlock();
	}
	
#endif
}

static int cnt[KEY_NUMS] = { 0,0 };
static int key_hist[KEY_NUMS] = { 0,0 };
static KeyState_e key_state[KEY_NUMS] = { NOT_PRESS,NOT_PRESS };
static KeyState_e key_state_hist[KEY_NUMS] = { NOT_PRESS,NOT_PRESS };

int DeviceKeyState_Get(int which_key)
{
	int key = 0;
	if ((which_key != KEY_A) && (which_key != KEY_B))
	{
		return -1;
	}
	key = key_num;
	key = (key>>which_key)&0x1;
	switch(key_state_hist[which_key])
	{
	case NOT_PRESS:
		if(key_hist[which_key]==0 && key==1)
		{
			key_state[which_key] = TRIGGER;
		}
		break;
	case TRIGGER:
		if(key_hist[which_key]==1 && key == 1)
		{
			if(cnt[which_key] < KEY_PRESSED_KEEP_CNT*3)// 1.5s
			{
				cnt[which_key]++;
			}else{
				cnt[which_key] = 0;
				key_state[which_key] = LONG_PRESS;
			}
		}else if(key == 0){
			cnt[which_key] = 0;
			key_state[which_key] = RELEASE;
		}
		break;
	case RELEASE: 		
		if(cnt[which_key] < KEY_PRESSED_KEEP_CNT)
		{
			cnt[which_key]++;
			if(key_hist[which_key]==0 && key == 1)
			{
				cnt[which_key] = 0;
				key_state[which_key] = DOUBLE_CLICK_CHECK;
			}
		}else{
			cnt[which_key] = 0;
			key_state[which_key] = SINGLE_CLICK;
		}
		break;
	case SINGLE_CLICK:
		key_state[which_key] = NOT_PRESS;
		break;
	case LONG_PRESS:
		if(key_hist[which_key] == 1 && key == 0)
		{
			key_state[which_key] = NOT_PRESS;
		}
		break;
	case DOUBLE_CLICK_CHECK:
	    if(cnt[which_key] < KEY_PRESSED_KEEP_CNT)
		{	
			cnt[which_key]++;
			if(key_hist[which_key] == 1 && key == 0)
			{
				cnt[which_key] = 0;
				key_state[which_key] = T_DOUBLE_CLICK;
			}
		}else{
			cnt[which_key] = 0;
			key_state[which_key] = LONG_PRESS;
		}
		break;
	case T_DOUBLE_CLICK:
		key_state[which_key] = NOT_PRESS;
		break;
	}
	
	if(key_state_hist[which_key]^key_state[which_key])
	{
		//printf("key_state: %d\n",key_state);
		switch(key_state[which_key])
		{
		case SINGLE_CLICK:
			printf("key_%d: single clicked!\n",which_key);
			TouchCallbackRun(TOUCH_EVENT_KEY_SINGLE_PRESS, (void*)&which_key);
			break;
		case T_DOUBLE_CLICK:
			printf("key_%d: double clicked!\n",which_key);
			TouchCallbackRun(TOUCH_EVENT_KEY_DOUBLE_PRESS, (void*)&which_key);
			break;
		case LONG_PRESS:
			printf("key_%d: long pressed!\n",which_key);
			TouchCallbackRun(TOUCH_EVENT_KEY_LONG_PRESS, (void*)&which_key);
			break;
		}
	}
	key_hist[which_key] = key;
	key_state_hist[which_key] = key_state[which_key];
	
	return (int)key_state[which_key];
}

void DeviceKeyState_Reset(void)
{
	int i = 0;
	for (i = 0; i < KEY_NUMS; i++)
	{
		cnt[i] = 0;
		key_hist[i] = 0;
		key_state[i] = NOT_PRESS;
		key_state_hist[i] = NOT_PRESS;
	}
}

//char* DeviceErrInfoGet()
//{
//	return err_buff;
//}

void DeviceForceFeedCtrl(int open_ctrl)
{
	force_feedback_enable_ = open_ctrl;
}