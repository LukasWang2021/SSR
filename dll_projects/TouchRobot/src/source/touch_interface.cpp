#include <Windows.h>
#include "touch_interface.h"
#include "touch_test.h"
#include "general_params.h"
#include "err_proc.h"
#include "data_proc.h"
#include "touch_play.h"

typedef struct {
	int readable;
	double tf_mtx[16];
}Touch_Tmtx_View_t;

static Touch_Callbak_Manager_t callback_fcn[TOUCH_EVENT_MAX];
static Touch_Tmtx_View_t tf_view;

int TouchTransfromationMatrixGet(double* tf_mtx_ptr)
{
	tf_view.readable = 0;
	memcpy(tf_view.tf_mtx, tf_mtx_ptr, 16*sizeof(double));
	tf_view.readable = 1;
	return 0;
}
/********************************************************************************************/
int TouchEnable(void)
{
	TouchTest::getInstance().ServiceEnable();
	do
	{
		Sleep(100);
		if (TouchTest::getInstance().ServiceGetState() == PRE_WORK_STAT)
		{
			return 0;
		}
		else {
			//printf("sm state val: %d\n", TouchTest::getInstance().ServiceGetState());
		}
	} while (0);

	return -1;
}

int TouchDisable(void)
{
	TouchTest::getInstance().ServiceDisable();
	do
	{
		Sleep(50);
		if (TouchTest::getInstance().ServiceGetState() == INIT_STAT)
		{
			return 0;
		}
	} while (0);

	return -1;
}

int TouchSetFault(void)
{
	TouchTest::getInstance().ServiceSetFault();
	do
	{
		Sleep(50);
		if (TouchTest::getInstance().ServiceGetState() == ERR_STAT)
		{
			return 0;
		}
	} while (0);

	return -1;
}

int TouchResetFault(void)
{
	TouchTest::getInstance().ServiceResetFault();
	do
	{
		Sleep(50);
		if (TouchTest::getInstance().ServiceGetState() == INIT_STAT)
		{
			ErrInfoReset();
			return 0;
		}
	} while (0);
	
	return -1;
}

int TouchCallbackRegister(int index, TOUCH_CALLBACK fcn, void* usr)
{
	if (index < 0 || index >TOUCH_EVENT_MAX)
		return -1;
	callback_fcn[index].fct = fcn;
	callback_fcn[index].param = usr;
	return 0;
}

int TouchStateGet(void)
{
	return TouchTest::getInstance().ServiceGetState();
}

int TouchTransfromationMatrixPull(double* tf_mtx_ptr, int size)
{
	int cnt = 100;
	if (tf_mtx_ptr == NULL || size < sizeof(double) * 16)
		return -1;
	do 
	{
		if (tf_view.readable == 1)
		{
			memcpy(tf_mtx_ptr, tf_view.tf_mtx, 16 * sizeof(double));
			break;
		}
		cnt--;
	} while (cnt > 0);

	if (cnt == 0)
		return -1;

	return 0;
}

int TouchXyzAbcPull(double* point, int size)
{
	double temp_tmx[16];
	int ret;
	RotationMatrix rmx;
	Vector3 euler;

	if (size < 6 * sizeof(double))
		return -1;

	ret = TouchTransfromationMatrixPull(temp_tmx, sizeof(double) * 16);
	if (ret < 0)
		return -1;

	point[0] = temp_tmx[12];
	point[1] = temp_tmx[13];
	point[2] = temp_tmx[14];
	/*
	%touch基座标到机械臂的基座标坐标转换。
			  [0  0  1  0;
               1  0  0  0;
               0  1  0  0;
               0  0  0  1]
	%touch末端朝向坐标变换
			  [ 0  -1   1   0;
               -1   0   0   0;
                0   0  -1   0;
                0   0   0   1]
	[a11 a12 a13 a14;    temp_tmx[0] temp_tmx[4] temp_tmx[8] temp_tmx[12]
	 a21 a22 a23 a24;	 temp_tmx[1] temp_tmx[5] temp_tmx[9] temp_tmx[13]
	 a31 a32 a33 a34;    temp_tmx[2] temp_tmx[6] temp_tmx[10] temp_tmx[14]
	 0   0   0   1 ];    temp_tmx[3] temp_tmx[7] temp_tmx[11] temp_tmx[15]

	 [-a32  -a31  -a33  a34;
	  -a12  -a11  -a13  a14;
	  -a22  -a21  -a23  a24;
	  0     0     0        1  ]
	*/
	rmx.matrix_[0][0] = -temp_tmx[6];
	rmx.matrix_[0][1] = -temp_tmx[2];
	rmx.matrix_[0][2] = - temp_tmx[10];

	rmx.matrix_[1][0] = -temp_tmx[4];
	rmx.matrix_[1][1] = -temp_tmx[0];
	rmx.matrix_[1][2] = - temp_tmx[8];

	rmx.matrix_[2][0] = -temp_tmx[5];
	rmx.matrix_[2][1] = -temp_tmx[1];
	rmx.matrix_[2][2] = - temp_tmx[9];

	RotationMatrix_to_Euler(&rmx, &euler);

	point[3] = euler.x;
	point[4] = euler.y;
	point[5] = euler.z;
	return 0;
}

int TouchCallbackRun(int index, void* param)
{
	int ret = 0;
	if (index < 0 || index >TOUCH_EVENT_MAX)
		return -1;
	if (callback_fcn[index].fct != NULL)
	{
		ret = callback_fcn[index].fct(callback_fcn[index].param, param);
	}
	return ret;
}

static char buff[1024];
char* TouchTestLoopBack(const char* str, int size)
{
	memset(buff,0,1024);
	memcpy_s(buff, 1024, str, size);
	return buff;
}

void TouchGetVersion(char* str, int size)
{
	memcpy_s(str,size, _TOUCH_DLL_VERSION_, strlen(_TOUCH_DLL_VERSION_)+1);
}

int TouchGetErrInfo(char* str, int size)
{
	if (TouchTest::getInstance().ServiceGetState() == ERR_STAT)
	{
		GetErrInfo(str, size);
		return 0;
	}
	return -1;
}

void TouchForceFeedCtrl(int open_flag)
{
	DeviceForceFeedCtrl(open_flag);
}