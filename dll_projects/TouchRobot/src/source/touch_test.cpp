#ifdef _WIN_PLAT
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <time.h>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <Windows.h>
#include <iostream>
#include <mmsystem.h>

#include "touch_test.h"
#include "rpc_help.h"
#include "touch_play.h"
#include "data_proc.h"
#include "err_proc.h"
#include "touch_interface.h"

#pragma comment(lib, "ws2_32.lib")//连接库（这种连接库的方式与在编程设置中链接库的功能是一致的）
#pragma comment(lib,"winmm.lib")

//定义时钟分辨率，以ms为单位
#define TIMER_ACCURACY		1				

struct tick_calc_t{
	int id;// 1-read, 2-write
	LARGE_INTEGER tp1;
	LARGE_INTEGER tp2;
	LARGE_INTEGER tp3;
};

void SleepSelectUS(SOCKET s, int64_t usec)
{
	struct timeval tv;
	fd_set dummy;
	FD_ZERO(&dummy);
	FD_SET(s, &dummy);
	tv.tv_sec = usec / 1000000L;
	tv.tv_usec = usec % 1000000L;
	select(0, 0, 0, &dummy, &tv);
	DWORD err = GetLastError();
	if (err != 0)
		printf("Error : %d", err);
}

////参数一表示 需要等待的时间 微秒为单位
//int UsSleep(int us)
//{
//	//储存计数的联合
//	LARGE_INTEGER fre;
//	//获取硬件支持的高精度计数器的频率
//	if (QueryPerformanceFrequency(&fre))
//	{
//		LARGE_INTEGER run, priv, curr, res;
//		run.QuadPart = fre.QuadPart * us / 1000000;//转换为微妙级
//		//获取高精度计数器数值
//		QueryPerformanceCounter(&priv);
//		do
//		{
//			QueryPerformanceCounter(&curr);
//		} while (curr.QuadPart - priv.QuadPart < run.QuadPart);
//		curr.QuadPart -= priv.QuadPart;
//		int nres = (curr.QuadPart * 1000000 / fre.QuadPart);//实际使用微秒时间
//		return nres;
//	}
//	return -1;//
//}

void TimerTask_RpcSend(void)
{
	TouchTest::getInstance().ServiceWriteController();
}

TouchTest::TouchTest(void)
{
	is_enable_ = false;
	is_exit_ = false;
	is_error_ = false;

	is_touch_init_ = false;
	is_rpc_init_ = false;
	is_config_init_ = false;
	is_timer_init_ = false;

	work_state_ = INIT_STAT;

	relay_buff_info_.head_index_offset = 0;
	relay_buff_info_.tail_index_offset = relay_buff_info_.head_index_offset;
	relay_buff_info_.element_length = ONLINETRJ_FRAME_SIZE;
	relay_buff_info_.buff_length = RELAY_BUFF_SIZE;

	if (is_config_init_ == false && TouchCfgInit() == true)
	{
		is_config_init_ = true;
		touch_read_thread_ = std::thread(&TouchTest::ServiceReadTouch, this);// cancel when init touch device failed
		touch_read_thread_.detach();

		if (CtrlCommInit() == true)
		{
			is_rpc_init_ = true;
		}
		else {
			is_error_ = true;
		}

		if (is_timer_init_ == false && InitHighTimer())
		{
			is_timer_init_ = true;
		}
		else {
			is_error_ = true;
		}
	}
	else {
		is_error_ = true;
	}

	statemachine_thread_ = std::thread(&TouchTest::ServiceStatemachine, this);// keep background

#ifdef _TICK_RECORD_	
	tick_node_ = circle_dlist_init();
	tick_record_thread_ = std::thread(&TouchTest::ServiceTickRecord, this);
#endif

	
}


TouchTest::~TouchTest()
{
	is_exit_ = 1;
	
	if (is_timer_init_ == true)
		FreeHighTimer();
	statemachine_thread_.join();

#ifdef _TICK_RECORD_
	tick_mutex_.unlock();
	circle_dlist_end(tick_node_);
#endif
}


void TouchTest::ServiceEnable(void)
{
	is_enable_ = true;
}

void TouchTest::ServiceDisable(void)
{
	is_enable_ = false;
}

void TouchTest::ServiceResetFault(void)
{
	is_error_ = false;
}

void TouchTest::ServiceSetFault(void)
{
	is_error_ = true;
}
 int TouchTest::ServiceGetState(void)
{
	return work_state_;
}
bool TouchTest::TouchCfgInit(void)
{
	FILE* fp = NULL;
	char buff[1024] = { 0 };
	char* temp[4] = { NULL };
	char* p = NULL;
	int i = 0;
	char* err_buff;
	int* ptr = (int*)&touch_params_.freq_touch;

	GetErrPtr(&err_buff, ERR_CONFIG);
	fp = fopen("ssr_touch_params.csv", "r");
	if (fp == NULL)
	{
		PushErrInfo(ERR_CONFIG, "Err: can't open config file!");
		return false;
	}
	while ((fgets(buff, 1024, fp)) != NULL)
	{
		i = 0;
		p = strtok(buff, ",");

		while (p)
		{
			temp[i] = p;
			++i;
			p = strtok(NULL, ",");
		}

		if (temp[0] != NULL &&  atoi(temp[0]) == 11)
		{
			touch_params_.controller_ip = (char*)malloc(32);
			memcpy_s(touch_params_.controller_ip, 32, temp[2], strlen(temp[2]));
			if (touch_params_.controller_ip != NULL && temp[2] != NULL)
			{
				touch_params_.controller_ip[strlen(temp[2])] = '\0';
			}
			else {
				PushErrInfo(ERR_CONFIG, "Err: decode config file failed!");
				return false;
			}
			//printf("%s\n", touch_params_.controller_ip);
			ptr = &touch_params_.online_trj_flag;
		}
		else {
			*ptr++ = atoi(temp[2]);
		}

	}
	fclose(fp);

	/*printf("load params: %d %d %d %d %d %d %d %s\n",
		touch_params_.freq_touch, touch_params_.filter_enum_xyz,
		touch_params_.filter_enum_abc, touch_params_.filter_fd_xyz,
		touch_params_.filter_enum_abc, touch_params_.cycle_time_controller,
		touch_params_.point_nums, touch_params_.controller_ip);*/

	return true;
}

bool TouchTest::TouchDevInit(void)
{
	if (touch_params_.online_trj_flag == 1)
	{
		if (DeviceInit() < 0)
		{
			goto device_init_failed;
		}

		if (DeviceCalib() < 0)
		{
			goto device_calib_failed;
		}

		DeviceCommConfig(touch_params_.freq_touch);

		if (DeviceStart() < 0)
		{
			goto device_start_failed;
		}
	}
	else
	{
		/*open offline file*/
		FILE* fp_trj = fopen(OFFLINE_FILE_PATH, "r");
		if (fp_trj == NULL)
		{
			goto open_offline_trjfile_failed;
		}
	
		fclose(fp_trj);
	}
	return true;

open_offline_trjfile_failed:
device_start_failed:
device_calib_failed:
device_init_failed:
	DeviceDisable();
	return false;
}


bool TouchTest::CtrlCommInit(void)
{
#ifdef _RPC_OPEN_
#ifndef _MAKE_DLL_
	if (rpc_help_comm_init(touch_params_.controller_ip) < 0)
	{
		goto rpc_call_failed;
	}
#endif
	if (rpc_help_setMode(4) < 0)
	{
		goto rpc_call_failed;
	}
	return true;
#else
	return true;
#endif

rpc_call_failed:
	return false;
}

// 释放定时器
void TouchTest::FreeHighTimer()
{
	if (g_mmTimerId == 0)
		return;

	timeKillEvent(g_mmTimerId);
	timeEndPeriod(g_wAccuracy);
}

// 初始化高精度定时器
bool TouchTest::InitHighTimer()
{
	TIMECAPS	tc;
	//利用函数timeGetDeVCaps取出系统分辨率的取值范围，如果无错则继续； 
	if (timeGetDevCaps(&tc, sizeof(TIMECAPS)) == TIMERR_NOERROR)
	{
		//分辨率的值不能超出系统的取值范围
		g_wAccuracy = min(max(tc.wPeriodMin, TIMER_ACCURACY), tc.wPeriodMax);

		//调用timeBeginPeriod函数设置定时器的分辨率 
		timeBeginPeriod(g_wAccuracy);

		// 设定10毫秒定时器
		g_mmTimerId = timeSetEvent(touch_params_.cycle_time_controller, 0, (LPTIMECALLBACK)TimerTask_RpcSend, NULL, TIME_PERIODIC);
		if (g_mmTimerId == 0)
		{
			char buff[ERR_INFO_BYTES];
			sprintf_s(buff, ERR_INFO_BYTES, "Err: timer init fialed: %d(timeSetEvent)", GetLastError());
			PushErrInfo(ERR_TIMER, buff);
			return false;
		}

		return true;
	}

	return FALSE;
}


void TouchTest::ServiceStatemachine(void)
{
	TouchState_e work_state_hist = INIT_STAT;

	int key_st_a = NOT_PRESS;
	int key_st_b = NOT_PRESS;
	int which_key = KEY_A;

	printf("Sm Thread Start!\n");

	LARGE_INTEGER tick_start, tick_end, tick_elapsed;
	LARGE_INTEGER tick_fre;
	double timeSpend = 0.0;
	
	WORD wVersionRequested = MAKEWORD(1, 0);
	WSADATA wsaData;
	WSAStartup(wVersionRequested, &wsaData);

	SOCKET s = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

	QueryPerformanceFrequency(&tick_fre);
	SetThreadDescription(GetCurrentThread(), L"write_thread");
	while (!is_exit_)
	{
		QueryPerformanceCounter(&tick_start);
		
		if (is_enable_ == false)
		{
			work_state_ = INIT_STAT;

		}else{

			if (is_error_ == true)
			{
				sig_read_start_ = 0;
				sig_write_start_ = 0;
				work_state_ = ERR_STAT;
			}

			key_st_a = DeviceKeyState_Get(KEY_A);
			key_st_b = DeviceKeyState_Get(KEY_B);

			switch (work_state_)
			{
			case INIT_STAT:

				sig_read_start_ = 0;
				sig_write_start_ = 0;
				work_state_ = IDEL_STAT;
							
				break;

			case IDEL_STAT:

				if (is_rpc_init_ == false 
						|| is_timer_init_ == false)
				{
					work_state_ = ERR_STAT;
				}

				if (is_touch_init_ == true 
						&& is_rpc_init_ == true 
						&& is_timer_init_ == true )
				{
					work_state_ = PRE_WORK_STAT;
				}
				break;

			case PRE_WORK_STAT:
			
				if (key_st_a == LONG_PRESS)
				{
					//emit start read signal
					sig_read_start_ = 1;
					//printf(">>> send sig: read start!\n");

					work_state_ = WORK_STAT;
				}
				break;

			case WORK_STAT:
				
				if (key_st_a == NOT_PRESS)
				{
					//emit stop send signal
					sig_read_start_ = 0;
					//printf(">>> send sig: read stop!\n");

				work_state_ = IDEL_STAT;
				TouchCallbackRun(TOUCH_EVENT_KEY_LONG_PRESS_RELEASE, (void*)&which_key);
			}
			break;

			case ERR_STAT:

				if (key_st_a == T_DOUBLE_CLICK)		// back door for reseting fault, not feasible in DLL
				{	
#ifdef _RPC_OPEN_
					if (CtrlCommInit() == true)
					{
						is_rpc_init_ = 1;
						is_error_ = false;
						work_state_ = INIT_STAT;
					}
					else {
						is_error_ = true;
					}
#else 
					is_error_ = false;
					is_rpc_init_ = 1;
#endif
				}

				if (is_error_ == false)
				{

					if (is_config_init_ == false && TouchCfgInit() == true)
					{
						is_config_init_ = true;
					}

					if(is_config_init_ == true)
					{
						if (is_touch_init_ == false)
						{
							touch_read_thread_ = std::thread(&TouchTest::ServiceReadTouch, this);// cancel when init touch device failed
							touch_read_thread_.detach();
						}

						if (is_rpc_init_ == false && CtrlCommInit() == true)
						{
							is_rpc_init_ = true;
						}
						
						if (is_timer_init_ == false && InitHighTimer())
						{
							is_timer_init_ = true;
						}
						
					}

					if (is_config_init_ == true && is_rpc_init_ == true && is_timer_init_ == true)
					{
						work_state_ = INIT_STAT;
					}
					else {
						is_error_ = true;
					}
					
				}

				break;
			default:
				break;
			}

			if (work_state_ ^ work_state_hist)
			{
				printf("work state: %d\n", work_state_);
			}

			work_state_hist = work_state_;

		}
		// sleep 8 ms
		SleepSelectUS(s, 8000);
	}
	return;
}

void TouchTest::ServiceReadTouch(void)
{
	int first_pack = 1;
	int  sig_read_start_now = 0;
	int  sig_read_start_hist = 0;

	LARGE_INTEGER tick_read, tick_read_hist, tick_elapsed;
	LARGE_INTEGER tick_fre;
	int tick_flag = 1;
 
	WORD wVersionRequested = MAKEWORD(1, 0);
	WSADATA wsaData;
	WSAStartup(wVersionRequested, &wsaData);
	SOCKET s = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

	QueryPerformanceFrequency(&tick_fre);
	//SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);

	if (TouchDevInit() == true)
	{
		is_touch_init_ = true;
		fflush(stdout);
	}
	else {
		is_error_ = 1;
		return;
	}

	Data_Proc_Init(&touch_params_);

	SetThreadDescription(GetCurrentThread(), L"read_thread");
	while (!is_exit_)
	{
		if (is_enable_ == false)	// wait enable flag: do nothing
		{
			first_pack = 1;
			sig_read_start_hist = 0;
			buffer_clear(&relay_buff_info_);
			DeviceKeyState_Reset();
			Data_Filter(NULL, 1);
			SleepSelectUS(s, 10000);
		}
		else {

			if (is_error_ == true)	// error state: do nothing
			{
				first_pack = 1;
				sig_read_start_hist = 0;
				buffer_clear(&relay_buff_info_);
#ifndef _MAKE_DLL_
				online_trj_frame_t buff = { 0 };

				if (DevicePullData((char*)&buff.tf[0], ONLINETRJ_FRAME_TF_SIZE) < 0)
				{
					is_error_ = 1;
					continue;
				}
				else {
					TouchTransfromationMatrixGet(&buff.tf[0]);
				}
#else
				DeviceKeyState_Reset();
				break;					//exit read thread!
#endif
			}
			else {

				if (work_state_ == WORK_STAT || work_state_ == PRE_WORK_STAT)// work state: pull data from touch device then push to relay buffer
				{
					online_trj_frame_t buff = { 0 };

					if (DevicePullData((char*)&buff.tf[0], ONLINETRJ_FRAME_TF_SIZE) < 0)
					{
						//printf("err: device pull data failed!\n");
						is_error_ = 1;
						continue;
					}
					Data_Filter(&buff.tf[0], 0);
					TouchTransfromationMatrixGet(&buff.tf[0]);
					sig_read_start_now = sig_read_start_;
					if (sig_read_start_hist == 0 && sig_read_start_now == 1)		// frame head: head frame
					{
						buff.head = 0;
						first_pack = 1;
						goto push_touch_data;

					}
					else if (sig_read_start_hist == 1 && sig_read_start_now == 1) {	// frame head: middle frame

						buff.head = 1;
						if (first_pack == 1 && (get_circle_buff_occupied(&relay_buff_info_) >= 5))
						{
							sig_write_start_ = 1;
							printf("2 >>> send sig: write start!\n");
							first_pack = 0;
						}
						goto push_touch_data;

					}
					else if (sig_read_start_hist == 1 && sig_read_start_now == 0) {	// frame head: tail frame

						buff.head = 2;
						sig_write_start_ = 0;
						goto push_touch_data;

					}
					else {
						goto not_push_data;
					}

				push_touch_data:
					// 判断缓冲区是否满，满出错
					if (is_buff_full(&relay_buff_info_))
					{
						PushErrInfo(ERR_READ_THREAD, "Err: read buff is full!");
						is_error_ = 1;
					}
					// 将touch数据压入环形缓冲区
					push_circle_buff_item(&relay_buff_info_, relay_buff, (char*)&buff.head);
#ifdef _TICK_RECORD_
					QueryPerformanceCounter(&tick_read);
					if (tick_flag == 1)
					{
						tick_flag = 0;

					}
					else {

						//tick_elapsed.QuadPart = tick_read.QuadPart - tick_read_hist.QuadPart;
						//printf("===>>> read_tick: %lf\n", (tick_elapsed.QuadPart * 1000.0) / tick_fre.QuadPart);
						tick_calc_t* tick_ptr = (tick_calc_t*)malloc(sizeof(tick_calc_t));
						tick_ptr->id = 1;
						tick_ptr->tp1 = tick_read;
						tick_ptr->tp2 = tick_read_hist;
						tick_mutex_.lock();
						circle_dlist_tail_insert(tick_node_, (char*)tick_ptr);
						tick_mutex_.unlock();
					}
					tick_read_hist = tick_read;
#endif
				not_push_data:
					sig_read_start_hist = sig_read_start_now;


				}
				else {
					sig_read_start_hist = 0;
					Data_Filter(NULL, 1);
					SleepSelectUS(s, 10000);
				}
			}

		}
	}

	DeviceStop();
	DeviceDisable();

	//printf("Read Touch Thread End!\n");
	fflush(stdout);
	//return;
}

void TouchTest::ServiceWriteController(void)
{
	//状态机用
	static int sig_write_start_now = 0;
	static int sig_write_start_hist = 0;
	static int eat_left_buff = 0;

	static online_trj_frame_t frame_get[MAX_TF_NUMS] = { 0 };
	static online_trj_send_t frame_send = { 0 };
	//数据补偿
	static double comp_ms = 0.0;
	static int pull_nums = 0;
	int comp_nums = 0;

	// 计算耗时用
	static LARGE_INTEGER tick_start, tick_end, tick_start_hist,tick_fre;
	LARGE_INTEGER tick_start_rpc, tick_end_rpc;
	double timeSpend;
	tick_calc_t* tick_ptr;
	int ret = 0;

	int* pdat = NULL;

	QueryPerformanceFrequency(&tick_fre);

	if (!is_exit_)
	{
		if (is_enable_ == false)
		{
			sig_write_start_hist = 0;
			eat_left_buff = 0;
			comp_ms = 0.0;
		}
		else {
			//10ms周期任务开始
			if (is_error_ == true)
			{
				sig_write_start_hist = 0;
				eat_left_buff = 0;
				comp_ms = 0.0;
			}
			else {

#ifdef _DELAY_COMP_	
				if (comp_ms > 2.0)// 达到数据补偿临界
				{
					comp_nums = (comp_ms / 2);

					if ((comp_nums + touch_params_.cycle_time_controller / 2) > MAX_TF_NUMS)
					{
						comp_nums = MAX_TF_NUMS - touch_params_.cycle_time_controller / 2;
					}
					comp_ms -= (comp_nums * 2.0);
				}
				else {
					comp_nums = 0;
				}
#else
				comp_nums = 0;
#endif
				frame_send.send_nums = comp_nums + touch_params_.point_nums;

				sig_write_start_now = sig_write_start_;
				if (sig_write_start_now == 1)
				{
					eat_left_buff = 0;
					goto pull_online_trj;
				}
				else if ((sig_write_start_hist == 1) && (sig_write_start_now == 0)) {

					eat_left_buff = 1;
					goto pull_online_trj;
				}
				else if ((sig_write_start_hist == 0) && (sig_write_start_now == 0)) {

					if (eat_left_buff == 1)
					{
						goto pull_online_trj;
					}
					else {
						goto send_nothing;
					}
				}

			pull_online_trj:
				if (is_buff_empty(&relay_buff_info_))
				{
					printf("buff is empty!\n");
					if (eat_left_buff == 1)
					{
						eat_left_buff = 0;// no data to send
						goto send_nothing;
					}
					else {
						goto send_online_trj;
					}
				}

				pull_nums = pull_circle_buff_all(&relay_buff_info_, relay_buff, (char*)(&(frame_get[0].head)), frame_send.send_nums);

				if (pull_nums == 0)
				{
					printf("err: pull no data!\n");
				}

				for (int i = 0; i < frame_send.send_nums; i++)
				{
#if 1
					if (i < pull_nums)
					{
						frame_send.state[i] = frame_get[i].head;
						memcpy(&frame_send.tf_mtx[i][0], &frame_get[i].tf[0], ONLINETRJ_FRAME_TF_NUM * sizeof(double));
					}
					else {
						frame_send.state[i] = frame_send.state[i - 1];
						memcpy(&frame_send.tf_mtx[i][0], &frame_send.tf_mtx[i - 1][0], ONLINETRJ_FRAME_TF_NUM * sizeof(double));
					}
#else
					//for test
					pdat = (int*)&frame_send.tf_mtx[i][0];
					for (int j = 0; j < 32; j++)
					{
						pdat[j] = i*0xF0+j;
					}
#endif
				}

			send_online_trj:

#ifdef _RPC_OPEN_
				
				QueryPerformanceCounter(&tick_start_rpc);
				if (rpc_help_sendOnlineTrajectory(&frame_send.tf_mtx[0][0], &frame_send.state[0], frame_send.send_nums) < 0)
				{
					is_error_ = 1;
				}
				QueryPerformanceCounter(&tick_end_rpc);

				/*printf("period: %lf === rpc: %lf === pull_nums:%d ===== head:%d %d \n", 
							(tick_end_rpc.QuadPart - tick_start_hist.QuadPart) * 1000.0 / tick_fre.QuadPart,
							(tick_end_rpc.QuadPart - tick_start_rpc.QuadPart) * 1000.0 / tick_fre.QuadPart,
							pull_nums, 
							frame_send.state[0], frame_send.state[frame_send.send_nums - 1]);*/
#ifdef _TICK_RECORD_
				tick_ptr = (tick_calc_t*)malloc(sizeof(tick_calc_t));
				tick_ptr->id = 2;
				tick_ptr->tp1 = tick_start_rpc;
				tick_ptr->tp2 = tick_end_rpc;
				tick_ptr->tp3 = tick_start_hist;
				tick_mutex_.lock();
				circle_dlist_tail_insert(tick_node_, (char*)tick_ptr);
				tick_mutex_.unlock();
#endif				

#else
				QueryPerformanceCounter(&tick_end_rpc);
				printf("comp_ms:%lf === send_nums:%d ===== head:%d %d \n",
					comp_ms, frame_send.send_nums, frame_send.state[0], frame_send.state[frame_send.send_nums - 1]);
#endif		

			send_nothing:
				sig_write_start_hist = sig_write_start_now;
				tick_start_hist.QuadPart = tick_end_rpc.QuadPart;
#if _DELAY_COMP_
				QueryPerformanceCounter(&tick_end);
				timeSpend = (tick_end.QuadPart - tick_start.QuadPart) * 1000.0 / tick_fre.QuadPart;
				if (timeSpend > 10.0)
				{
					comp_ms += (timeSpend - 10.0);
				}
#endif
			}
		}
	}
}

void TouchTest::ServiceTickRecord(void)
{
	tick_calc_t* tick_ptr[5] = { NULL };
	FILE* fp = NULL;
	LARGE_INTEGER tick_fre;
	LARGE_INTEGER tick_hist_write = {0};
	int i = 0, cnt = 0;

	WORD wVersionRequested = MAKEWORD(1, 0);
	WSADATA wsaData;
	WSAStartup(wVersionRequested, &wsaData);
	SOCKET s = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);

	fp = fopen("tick_record.dat", "w+");

	if (fp == NULL)
	{
		return;
	}

	QueryPerformanceFrequency(&tick_fre);
	while (1)
	{
		if (tick_mutex_.try_lock())
		{
			for (i = 0; i < 6; i++)
			{
				if ((tick_ptr[i] = (tick_calc_t*)circle_dlist_head_eat(tick_node_)) == NULL)
					break;
			}
			tick_mutex_.unlock();
		}
		cnt = i - 1;
		while(cnt>=0)
		{
			switch (tick_ptr[cnt]->id)
			{
			case 1:
				fprintf(fp, "read: %lf\n", (tick_ptr[cnt]->tp1.QuadPart - tick_ptr[cnt]->tp2.QuadPart) * 1000.0 / tick_fre.QuadPart);
				break;
			case 2:
				fprintf(fp, "write: %lf, %lf\n", (tick_ptr[cnt]->tp2.QuadPart - tick_ptr[cnt]->tp1.QuadPart) * 1000.0 / tick_fre.QuadPart,
					(tick_ptr[cnt]->tp2.QuadPart - tick_ptr[cnt]->tp3.QuadPart) * 1000.0 / tick_fre.QuadPart);
				//tick_hist_write = tick_ptr[cnt]->tp1;
				break;
			default:
				break;
			}
			cnt--;
		}
		SleepSelectUS(s, 10000);
	}
}