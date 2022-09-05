#ifdef _WIN_PLAT
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <stdio.h>
#include "rpc_help.h"
#include "rpc_interface.h"
#include "sub_interface.h"
#include "touch_interface.h"
#include "touch_test.h"
#include "err_proc.h"

static uint64_t err_code;
static int err_buff_size;

static char *err_buff;

int rpc_help_comm_init(char *ip)
{
	GetErrPtr(&err_buff, ERR_RPC);
#ifdef _RPC_OPEN_
#ifndef _MAKE_DLL_
	err_code = c_initRpc(ip);
	if(err_code)
	{
		strcpy_s(err_buff, ERR_INFO_BYTES, "Init RPC Failed!");
		return -1;
	}
	//err_code = c_deleteTopic();
	err_code = c_addTopic();
	if (err_code)
	{
		//c_getErrorCodeDetail(err_code,err_buff,&err_buff_size);
		//strcpy_s(err_buff, ERR_INFO_BYTES, "Call RPC failed! (c_addTopic)");
		PushErrInfo(ERR_RPC, "Call RPC failed! (c_addTopic)");
		return -1;
	}

	err_code = c_initSub(ip);
	if (err_code)
	{
		//strcpy_s(err_buff, ERR_INFO_BYTES, "Init Sub failed! (c_initSub)");
		PushErrInfo(ERR_RPC, "Init Sub failed! (c_initSub)");
		return -1;
	}
#endif
#endif
	return 0;
}

int rpc_help_comm_end()
{
#ifndef _MAKE_DLL_
	err_code = c_deleteTopic();
	if(err_code)
	{
		//c_getErrorCodeDetail(err_code,err_buff,&err_buff_size);
		strcpy(err_buff, "Call RPC failed! (c_deleteTopic)");
		return -1;	
	}
#endif
	return 0;
}

//char* rpc_help_err()
//{
//	printf("err size: %d\n",err_buff_size);
//	if(err_buff_size<ERR_BUFF_SIZE)
//		err_buff[err_buff_size] = '\0';
//	else
//		err_buff[ERR_BUFF_SIZE-1] = '\0';
//	return err_buff;
//}


int rpc_help_setMode(uint32_t mode)
{
	char buff[1024];
	uint32_t tmp_mode;
	err_code = c_setWorkMode(mode);
	if(err_code)
	{
		//c_getErrorCodeDetail(err_code,err_buff,&err_buff_size);
		sprintf_s(err_buff, ERR_INFO_BYTES, "Call RPC failed! (c_setWorkMode: 0x%x)",err_code);
		PushErrInfo(ERR_RPC, err_buff);
		return -1;
	}
	do
	{
		Sleep(100);
		err_code = c_getWorkMode(&tmp_mode);
		if(err_code)
		{
			//c_getErrorCodeDetail(err_code,err_buff,&err_buff_size);
			//strcpy_s(err_buff, ERR_INFO_BYTES, "Call RPC failed! (c_getWorkMode)");
			PushErrInfo(ERR_RPC, "Call RPC failed! (c_getWorkMode)");
			return -1;
		}
		if(tmp_mode == mode)
		{
			break;
		}
		else {
			//strcpy_s(err_buff, ERR_INFO_BYTES, "Call RPC failed! (c_setWorkMode)");
			PushErrInfo(ERR_RPC, "Call RPC failed! (c_setWorkMode)");
			return -1;
		}
			
	}while(1);
		
	return 0;
	
}

int rpc_help_sendOnlineTrajectory(double traj[], int state[], int cnt)
{	
	err_code = c_sendOnlineTrajectory(traj, state, cnt);
	if(err_code)
	{
		char buff[ERR_INFO_BYTES];
		c_getErrorCodeDetail(err_code,err_buff,&err_buff_size);
		
		sprintf_s(buff, ERR_INFO_BYTES, "Call RPC failed, err_code:  0x%llx! (c_sendOnlineTrajectory)", err_code);
		PushErrInfo(ERR_RPC, buff);
		return -1;
	}else{
		return 0;
	}
	return 0;
}


int rpc_help_getForce(double *force, int32_t size)
{
	if (c_getTorqueFeedBack(force, size) != 0)
	{
		//strcpy_s(err_buff, ERR_INFO_BYTES, "Call Sub failed! (c_getTorqueFeedBack)");
		PushErrInfo(ERR_RPC, "Call Sub failed! (c_getTorqueFeedBack)");
		return -1;
	}

	return 0;
}