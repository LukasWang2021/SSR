/**
 * @file proto_parse.h
 * @brief 
 * @author Wang Wei
 * @version 1.0.0
 * @date 2016-08-21
 */
#include <stdio.h>
#include "base_types_hash.h"
#include "common.h"
#include "lib_controller/fst_datatype.h"
#include "nn_socket.h"
#include "json_parse.h"
#include "robot_motion.h"
#include "proto_define.h"
#include "watchdog.h"
#include "ip_address.h"

#define MIN_ROBOT_PUBLISH_TIME	(10) //ms
#define MAX_BUFFER_SIZE	1024

class Proto_Parse
{
public:
	int hash_size;
	int cmd_bytes_written;
	int state_bytes_written;
	NN_Socket* nn_socket;
	Json_Parse *json_parse;
	Robot_Motion robot_motion;
	bool wdt_start_flag;
	Watchdog wdt;
	uint8_t sending_buffer[MAX_BUFFER_SIZE];
	uint8_t publish_buffer[MAX_BUFFER_SIZE];

	int state_publish_tm;
	int mode_publish_tm;
	int joint_publish_tm;
	int coord_publish_tm;

	Proto_Parse()
	{
		state_publish_tm = 0;
		mode_publish_tm = 0;
		joint_publish_tm = 0;
		coord_publish_tm = 0;

		wdt_start_flag = false;

		string str_addr = Get_Local_IP();

		hash_size = get_hash_size();
		NN_Sock_Type type = NN_SOCK_WS;
		nn_socket = new NN_Socket(str_addr, type, type, COMMAND_PORT, STATE_PORT);
		if(nn_socket == NULL)
		{
			printf("construct socket server failed\n");
			exit(-1);
		}
		
		if(nn_socket->NN_Socket_Init() < 0)
			printf("error: socket init");
		
		json_parse = new Json_Parse(FILE_API_PATH);
		if(json_parse == NULL)
		{
			printf("construct json parser failed\n");
			exit(-1);
		}
		
		json_parse->Parse_API_Json(); //store all the infomation in the API file	
	}
 
	~Proto_Parse()
	{
		delete nn_socket;
		delete json_parse;
	}

	void Check_Manual_Wdt();
	ErrorCode Parse_Buffer(uint8_t *buffer, int count);
	bool Ret_Status(string path, BaseTypes_StatusCode status_code);
	bool Ret_Previous_Command_Id(int previous_command_id);
	bool Ret_Current_Command_Id(int current_command_id);

	template<typename T> 
	bool Ret_Param_Msg(string path, T param);
	template<typename T>
	bool Pub_Param_Msg(string path, T param);

	bool Pub_Cur_State();
	bool Pub_Cur_Mode();
	bool Pub_Cur_Joint();
	bool Pub_Cur_Coord();


private:
	ErrorCode Parse_Motion_Program(uint8_t *buffer, int count);
	bool compare_int(const unsigned char *cmp_a, uint8_t *cmp_b)
	{
		int *p_a = (int*)cmp_a;
		int *p_b = (int*)cmp_b;
		for(int i = 0; i < get_hash_size()/sizeof(int); i++)
		{
			if(p_a[i] != p_b[i])
			{
				//printf("a:%d, b:%d, %d\n", cmp_a[i], cmp_b[i],i);
				return false;
			}
		}

		return true;
	}
};
