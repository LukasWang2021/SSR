#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/mman.h> //mmap head file
#include <sys/shm.h>
#include <sched.h>
#include <assert.h>
#include <time.h>
#include <pthread.h>
#include "rpc_help.h"
#include "touch_play.h"
#include "mt_timer.h"
#include "log_help.h"
#include "buffer_manager.h"

#define TIME_TEST
#define TOUCH_TEST	 // in: touch
//#define RPC_TEST   // out: controller
/******************************** typedef ***********************************/

#define RELAY_BUFF_SIZE (ONLINETRJ_FRAME_TF_SIZE*1024)
#define LOG_BUFF_SIZE (1024*1024)
#define OFFLINE_FILE_PATH "offline_trj.csv"

typedef struct{
	int freq_touch; 			           //touch的采样频率 2ms
	int cycle_time_controller;             //和controller的通信周期 10ms
	int online_trj_flag;		           //在线轨迹flag
	int point_nums;
}ssr_touch_params_t;

typedef enum{
	INIT_STAT,
	IDEL_STAT,
	WORK_STAT,
	ERR_STAT,
}ssr_touch_sm_e;
	
/***************************  static variable *******************************/
static ssr_touch_sm_e work_state = INIT_STAT;
static pthread_mutex_t mutex_work_st;
	
/*log map ptr*/
static buffer_info_t *map_ctrl;
static char *map_msg;

/*config*/
static ssr_touch_params_t ssr_params;
static FILE *fp_trj;

/*buffer: share memory*/
static buffer_info_t relay_buff_info;
static char relay_buff[RELAY_BUFF_SIZE];

static pthread_t tid_sm;
static pthread_t tid_write;

TIMER_CREATE(controller);
static int timer;

// test time of task 
static FILE *fp_tt1 = NULL;
static FILE *fp_tt2 = NULL;

/***************************  static function   *****************************/
static int ssr_touch_init_log();
static int ssr_touch_init_config();
static int ssr_touch_init_device(void);
static void ssr_touch_sigint_handle(int num);
static void ssr_touch_end_log();
static void ssr_touch_rpc_init(void);
static void ssr_touch_rpc_end(void);

static void print_delta_time1(FILE *fpp);
static void print_delta_time2(FILE *fpp);

static void *task_read_touch(void * data);
static void *task_write_controller(void * data);
static void *task_ssr_touch_sm(void* data);

static int ssr_touch_init_log()
{
	int fd_ctrl,fd_msg;

	fd_ctrl = open("/tmp/f1",O_CREAT|O_RDWR|O_TRUNC,00777);	
	if(fd_ctrl<0)
	{
		printf("open /tmp/f1 error\n");
		goto open_ctl_error;
	}

	fd_msg = open("/tmp/f2",O_CREAT|O_RDWR|O_TRUNC,00777);
	if(fd_msg<0)
	{
        printf("open /tmp/f2 error\n");
        goto open_msg_error;
	}

	lseek(fd_ctrl,sizeof(buffer_info_t),SEEK_SET);
	write(fd_ctrl,"1",1);
	lseek(fd_msg,LOG_BUFF_SIZE,SEEK_SET);
	write(fd_msg,"1",1);

	map_ctrl = (buffer_info_t*) mmap( NULL,sizeof(buffer_info_t),PROT_READ|PROT_WRITE,MAP_SHARED,fd_ctrl,0 );
	if(map_ctrl==NULL)
	{
	    printf("mmap map_ctrl error\n ");
	    goto mmap_ctl_error;
	}

	map_msg = (char*) mmap( NULL,LOG_FILE_LEN,PROT_READ|PROT_WRITE,MAP_SHARED,fd_msg,0 );
	if(map_msg==NULL)
	{
        printf("mmap map_msg error\n");
        goto mmap_msg_error;
	}
	close(fd_ctrl);
	close(fd_msg);

	LogHelp_Init(map_ctrl,map_msg);
	return 0;

mmap_msg_error:
	munmap(map_ctrl,LOG_FILE_LEN);
mmap_ctl_error:
    close(fd_msg);
open_msg_error:
	close(fd_ctrl);
open_ctl_error:
	return -1;
}


static int ssr_touch_init_config(void)
{
/*step 1: init params*/
	
    ssr_params.freq_touch = 500;
    ssr_params.cycle_time_controller = 10;
    ssr_params.online_trj_flag = 1;
	ssr_params.point_nums = 5;
	
	Log_info("load params: %d %d %d\n",ssr_params.freq_touch,ssr_params.cycle_time_controller,ssr_params.online_trj_flag);
    
	relay_buff_info.head_index_offset = 0;
	relay_buff_info.tail_index_offset = relay_buff_info.head_index_offset;
	relay_buff_info.element_length = ONLINETRJ_FRAME_TF_SIZE;
	relay_buff_info.buff_length = RELAY_BUFF_SIZE;
	pthread_mutex_init(&relay_buff_info.mutex,NULL);
	pthread_mutex_init(&mutex_work_st,NULL);
	return 0;
}

static int ssr_touch_init_device(void)
{
/*step 2: init touch device or open offline file*/
	if(ssr_params.online_trj_flag == 1)
	{
		if(DeviceInit()<0)
		{
			goto device_init_failed;
		}
		
		if(DeviceCalib()<0)
		{
			goto device_calib_failed;
		}
		
		DeviceCommConfig(500);
		
		if(DeviceStart()<0)
		{
			goto device_start_failed;
		}
		Log_info("touch device init succeed!\n");
	}
	else
	{
		/*open offline file*/
		fp_trj = fopen(OFFLINE_FILE_PATH,"r");
		if(fp_trj == NULL)
		{
			goto open_offline_trjfile_failed;
		}else{
			Log_info("open offline trj succeed!\n");
		}
	}
	return 0;
device_init_failed:
device_calib_failed:
device_start_failed:
open_offline_trjfile_failed:
	return -1;
}


static void ssr_touch_sigint_handle(int num)
{	
#ifdef TOUCH_TEST
	DeviceStop();
	DeviceDisable();
#endif
	TIMER_DEL(controller, timer);
    TIMER_CLEAR(controller);
    TIMER_DEINIT(controller);
    Log_info("Interrupt Signal.");	
	usleep(2000);
#ifdef RPC_TEST
	ssr_touch_rpc_end();
#endif
	ssr_touch_end_log();
	pthread_mutex_destroy(&relay_buff_info.mutex);
	pthread_cancel(tid_sm);
   	exit(1);	
}
static void print_delta_time1(FILE *fpp)
{
	struct timespec tp;
	static struct timespec tp_h;
	static int first = 1;
	
	clock_gettime(CLOCK_MONOTONIC,&tp);
	if(!first)
	{
		fprintf(fpp,"%d\n",(int)((tp.tv_sec- tp_h.tv_sec)*1000*1000+(tp.tv_nsec - tp_h.tv_nsec)/1000));
	}else{
		first = 0;
	}
	
	tp_h.tv_sec = tp.tv_sec;
	tp_h.tv_nsec = tp.tv_nsec;
}

static void print_delta_time2(FILE *fpp)
{
	struct timespec tp;
	static struct timespec tp_h;
	static int first = 1;
	
	clock_gettime(CLOCK_MONOTONIC,&tp);
	if(!first)
	{
		fprintf(fpp,"%d\n",(int)((tp.tv_sec- tp_h.tv_sec)*1000*1000+(tp.tv_nsec - tp_h.tv_nsec)/1000));
		
	}else{
		first = 0;
	}
	
	tp_h.tv_sec = tp.tv_sec;
	tp_h.tv_nsec = tp.tv_nsec;
}
static void time_out_warning()
{
	struct timespec tp;
	static struct timespec tp_h;
	static int first = 1;
	
	clock_gettime(CLOCK_MONOTONIC,&tp);
	if(!first)
	{
		//fprintf(fpp,"%d\n",(int)((tp.tv_sec- tp_h.tv_sec)*1000*1000+(tp.tv_nsec - tp_h.tv_nsec)/1000));
		if(((tp.tv_sec- tp_h.tv_sec)*1000*1000+(tp.tv_nsec - tp_h.tv_nsec)/1000)>10000)
		{
			printf("over time 10ms!\n");
		}
	}else{
		first = 0;
	}
	
	tp_h.tv_sec = tp.tv_sec;
	tp_h.tv_nsec = tp.tv_nsec;
}
static void ssr_touch_end_log()
{
	munmap( map_ctrl, sizeof(buffer_info_t) );
	munmap( map_msg, LOG_BUFF_SIZE);
}

static void ssr_touch_init(void)
{

	if(ssr_touch_init_config()<0)
	{
		Log_warn("load config file failed!\n");
		exit(-1);
	}

	if(ssr_touch_init_device()<0)
	{
		/*init device failed*/
		Log_warn("init device failed*!\n");
		exit(-1);
	}
	else
	{
		Log_info("init device succeed*!\n");
	}

}
	
static void ssr_touch_rpc_init()
{
	if(rpc_help_comm_init("192.168.1.100")<0)
	{
		Log_warn("err %s\n",rpc_help_err());
		return;
		//exit(0);
	}

	if(rpc_help_setMode(4)<0)
	{
		Log_warn("err %s\n",rpc_help_err());
		return;
	}
}

static void ssr_touch_rpc_end()
{
	if(rpc_help_comm_end()<0)
	{
		Log_warn("err %s\n",rpc_help_err());
		return;
	}
}

/*--------------------------------------------------------------------
 * 函数名称:static void *task_read_touch(void * data)
 * 函数输入:
 * 函数输出:
 * 函数备注:从touch获取数据，2ms一次
 * ------------------------------------------------------------------*/

static void *task_read_touch(void * data)
{
	static ssr_touch_sm_e workst_read = INIT_STAT;
	printf("read touch start\n");
#ifdef RPC_TEST
	FILE *fp = fopen("hddata.csv","r");
	if(fp==NULL)
	{
		printf("open read data failed!\n");
	}
#endif
	while(1)
    {
       	char buff[ONLINETRJ_FRAME_TF_SIZE];
		
#ifdef TOUCH_TEST
		if(DevicePullData(buff, ONLINETRJ_FRAME_TF_SIZE)<0)
#else
		if(fgets(buff, ONLINETRJ_FRAME_TF_SIZE, fp)==NULL)
#endif
		{
			pthread_mutex_trylock(&mutex_work_st);
			work_state = ERR_STAT;
			pthread_mutex_unlock(&mutex_work_st);
			break;

		}else{
			
			if(pthread_mutex_trylock(&mutex_work_st)==0)
			{
				workst_read = work_state;
				pthread_mutex_unlock(&mutex_work_st);
				if(work_state==WORK_STAT)
				{
					if(buffer_lock(&relay_buff_info)==0)	//pthread_lock
			        {
				        if(is_buff_full(&relay_buff_info))
				        {
				            printf("buff is full!\n");
				            buffer_unlock(&relay_buff_info);
							pthread_yield();
							continue;
				        }
				        push_circle_buff_item(&relay_buff_info,relay_buff,buff);
						print_delta_time1(fp_tt1);
				        buffer_unlock(&relay_buff_info); 
			        }
				}
			}
		}

#ifdef RPC_TEST
		usleep(2000);
#endif
    }
#ifdef RPC_TEST	
	fclose(fp);
	usleep(100000);
#endif
	printf("read task end!\n");
	return NULL;

}

/*--------------------------------------------------------------------
 * 函数名称:static void task_write_controller(void * data)
 * 函数输入:
 * 函数输出:
 * 函数备注:向controller发数据，10ms执行一次
 * ------------------------------------------------------------------*/

static void *task_write_controller(void * data)
{
	static ssr_touch_sm_e workst_read = INIT_STAT;
	static ssr_touch_sm_e workst_hist_read = INIT_STAT;
	static char buff[ONLINETRJ_FRAME_TF_SIZE];// 16*sizeof(double)//16*8
	static online_trj_frame_t frame_send = {0};
	int pull_nums = 0 ;
	int pull_nums_read = 0;
	
	if(pthread_mutex_trylock(&mutex_work_st)==0)
	{
		workst_read = work_state;
		pthread_mutex_unlock(&mutex_work_st);
		
		if(workst_read == WORK_STAT && workst_hist_read == INIT_STAT)
		{
			/*first pressed*/
	        frame_send.head = 0.0;
			printf("Start! frame head: %lf\n",frame_send.head);
			goto pull_frame_data;
			
		}else if(workst_read == WORK_STAT && workst_hist_read == WORK_STAT){

			/*keep pressed*/
			frame_send.head = 1.0;	
			printf("Keep! frame head: %lf\n",frame_send.head);
			goto pull_frame_data;

		}else if(workst_read == INIT_STAT && workst_hist_read == WORK_STAT){
		
			/*release*/
			frame_send.head = 2.0;	
			printf("Release! frame head: %lf\n",frame_send.head);
			goto pull_frame_data;

		}else{
			frame_send.head = 0.0;
			workst_hist_read = workst_read;
			/*wait sm change*/
			return;
		}
		
	}
pull_frame_data:
	workst_hist_read = workst_read;
	if(buffer_lock(&relay_buff_info)==0)	//pthread_lock
    {
        if(is_buff_empty(&relay_buff_info))
        {
            //printf("buff is empty!\n");
            buffer_unlock(&relay_buff_info);
			pthread_yield();
			return;
        }
        pull_nums = pull_circle_buff_all(&relay_buff_info,relay_buff, (char*)&frame_send.tf[0][0], ONLINETRJ_FRAME_POINT_NUM);
		buffer_unlock(&relay_buff_info); 
		pull_nums_read = pull_nums;
		if(pull_nums==0)
		{
			printf("pull no data!\n");
			Log_warn("pull no data!\n");
			/*use old tf ?*/
			return;
		}else{
			
			while(pull_nums < ONLINETRJ_FRAME_POINT_NUM)
			{
				memcpy(&frame_send.tf[pull_nums][0], &frame_send.tf[pull_nums-1][0], ONLINETRJ_FRAME_TF_NUM*sizeof(double));
				pull_nums++;
			}
			Log_info("frame head: %f pull nums: %d\n\t%f\t%f\t%f\t%f\n"
									 "\t%f\t%f\t%f\t%f\n"
									 "\t%f\t%f\t%f\t%f\n"
									 "\t%f\t%f\t%f\t%f\n",frame_send.head,pull_nums_read,
									 frame_send.tf[4][0],frame_send.tf[4][4],frame_send.tf[4][8],frame_send.tf[4][12],
									 frame_send.tf[4][1],frame_send.tf[4][5],frame_send.tf[4][9],frame_send.tf[4][13],
									 frame_send.tf[4][2],frame_send.tf[4][6],frame_send.tf[4][10],frame_send.tf[4][14],
									 frame_send.tf[4][3],frame_send.tf[4][7],frame_send.tf[4][11],frame_send.tf[4][15]
									 );
			print_delta_time2(fp_tt2);
		}
    }
#ifdef RPC_TEST 
	rpc_help_sendOnlineTrajectory(&frame_send.head, ONLINETRJ_FRAME_NUM);
#endif

	
}

/*--------------------------------------------------------------------
 * 函数名称:static void *task_ssr_touch_sm(void* data)
 * 函数输入:
 * 函数输出:
 * 函数备注:状态机，受控于按键行为
 * ------------------------------------------------------------------*/
 
static void *task_ssr_touch_sm(void* data)
{
	ssr_touch_sm_e work_state_hist = INIT_STAT;
	KeyState_e key_st_a = NOT_PRESS;
	KeyState_e key_st_b = NOT_PRESS;
	
	printf("ssr touch sm task run 1!\n");

	printf("ssr touch sm task run 2\n");
	while(1)
	{
		key_st_a = DeviceKeyState_Get(KEY_A);
		key_st_b = DeviceKeyState_Get(KEY_B);
		
		pthread_mutex_lock(&mutex_work_st);	
		switch(work_state)
		{
		case INIT_STAT:
			if(key_st_a == LONG_PRESS)
			{
				work_state = WORK_STAT;
			}
			break;
		case IDEL_STAT:
			
			break;
		case WORK_STAT:
			if(key_st_a == NOT_PRESS)
			{
				work_state = INIT_STAT;
			}
			break;
		case ERR_STAT:
			if(key_st_a == DOUBLE_CLICK)
			{				
				work_state = INIT_STAT;
			}
			break;
		default:
			break;
		}
	
		if(work_state^work_state_hist)
		{
			//Log_info("State","change state from %s to %s!\n",sm_str[work_state_hist].c_str(),sm_str[work_state].c_str());
			printf("work state: %d\n",work_state);
		}
		
		work_state_hist = work_state;
		pthread_mutex_unlock(&mutex_work_st);
		usleep(1000);
	}
}


/******************************************************************************************
*											main
*******************************************************************************************/

int main(void)
{
    struct itimerspec itimespec;
	
	pthread_attr_t attr;
	struct sched_param param;
	int maxpri, count; 
	
	ssr_touch_init_log();
	
	#ifdef RPC_TEST
		ssr_touch_rpc_init();
	#endif

	ssr_touch_init_config();//params init; relay buff init

	signal(SIGINT, ssr_touch_sigint_handle);
	signal(SIGTERM, ssr_touch_sigint_handle);
	
	#ifdef TOUCH_TEST
		ssr_touch_init();
	#endif

	#ifdef TIME_TEST
		fp_tt1 = fopen("tt_send.csv","w+");
		fp_tt2 = fopen("tt_read.csv","w+");
	#endif

	
/* timer setup: add controller communication task */
	TIMER_INIT(controller, 10);
    itimespec.it_value.tv_sec = 0;
    itimespec.it_value.tv_nsec = 100000000;
    itimespec.it_interval.tv_sec = 0;
    itimespec.it_interval.tv_nsec = 10000000;//10ms
    timer = TIMER_ADD(controller, &itimespec, -1, task_write_controller, NULL, NULL);


/* state machine: add sm task, change sm by key_value */	
	pthread_create(&tid_sm, NULL, task_ssr_touch_sm, NULL);    



    maxpri = sched_get_priority_max(SCHED_RR);
     if(maxpri == -1) { 
  	  	perror("sched_get_priority_max() failed"); 
    	return -1; 
  	}
  	printf("max priority of SCHED_RR is %d\n", maxpri);
	param.sched_priority = 50; 
	if (sched_setscheduler(getpid(), SCHED_RR, &param) == -1) { 
		perror("sched_setscheduler() failed"); 
		return -1; 
	}

/* get touch data: main task */
 	task_read_touch(NULL);



	#ifdef RPC_TEST
		ssr_touch_rpc_end();
	#endif
	
	ssr_touch_end_log();
	
	pthread_mutex_destroy(&relay_buff_info.mutex);
	pthread_mutex_destroy(&mutex_work_st);
	printf("program end!\n");
	pthread_join(tid_sm,NULL);
	pthread_attr_destroy(&attr);
	#ifdef TOUCH_TEST
		DeviceStop();
		usleep(5000);
		DeviceDisable();
		printf("device disable!\n");
	#endif
	
	return 0;
}




