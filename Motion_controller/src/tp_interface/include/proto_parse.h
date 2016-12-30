/**
 * @file proto_parse.h
 * @brief 
 * @author Wang Wei
 * @version 1.0.0
 * @date 2016-08-21
 */
#ifndef TP_INTERFACE_PROTO_PARSE_H_
#define TP_INTERFACE_PROTO_PARSE_H_

#include <stdio.h>
#include "base_types_hash.h"
#include "robot.h"
#include "motion_controller/fst_datatype.h"
#include "nn_socket.h"
#include "json_parse.h"
#include "robot_motion.h"
#include "proto_define.h"
#include "watchdog.h"
#include "ip_address.h"
#include "ros_basic.h"

#define FORCE_PUBLISH_EXPIRE_TIME   (1000) //ms
#define MANUAL_EXPIRE_TIME			(300) //ms

#define ERROR_NUM               (10)

typedef struct _PublishUpdate
{
	int relative_time; //the unit is 10ms right now
	int update_cnt;
    bool update_flag;
}PublishUpdate;

class ProtoParse
{
  public:
	bool wdt_start_flag_;
	long manual_start_time_;	
    RosBasic *ros_basic_;
	
	ProtoParse(RosBasic *ros_basic);
	~ProtoParse();

	/**
	 * @brief: get pointer of RobotMotion object
	 *
	 * @return: the object 
	 */
	RobotMotion *getRobotMotionPtr();
	/**
	 * @brief: the main process of parser 
	 */
	void StartParser();
	/**
	 * @brief: check if watchdog started in manual mode
	 */
	void checkManualWdt();
	/**
	 * @brief: updateParams
	 */
    void updateParams();
	/**
	 * @brief: publish controller parameters
	 *
	 * @return:true if success 
	 */	
	void pubParamList();

    /**
     * @brief: store err_code 
     *
     * @param err_code: input
     */
    void storeErrorCode(U64 err_code);


  private:
	Watchdog	wdt_;			//the Watchdog object	
	NNSocket	*nn_socket_;	//socket object pointer
	JsonParse	*json_parse_;	//
	RobotMotion robot_motion_;	//
	int			hash_size_;		//size of hash bytes	

	map<int, PublishUpdate>	param_pub_map_; //the map from parameter id to there publish time 
    ThreadsafeQueue<U64>    error_queue_;
	boost::mutex	        mutex_;		//mutex lock

	/**
	 * @brief: parse the struct of set message
	 *
	 * @param field_buffer: input==>the buffer pointer
	 * @param field_size: input==>the length of buffer
	 */
	void parseParamSetMsg(const uint8_t *field_buffer, int field_size);
	/**
	 * @brief: parse the struct of command message
	 *
	 * @param field_buffer: input==>the buffer pointer
	 * @param field_size: input==>the length of buffer
	 *
	 * @return:true if success 
	 */
	void parseParamCmdMsg(const uint8_t *field_buffer, int field_size);
	/**
	 * @brief: parse the struct of get message
	 *
	 * @param field_buffer: input==>the buffer pointer
	 * @param field_size: input==>the length of buffer
	 *
	 * @return:true if success 
	 */
	void parseParamGetMsg(const uint8_t *field_buffer, int field_size);
	/**
	 * @brief: parse the buffer input 
	 *
	 * @param buffer: input==>buffer to store decoded data
	 * @param count: input==>the buffer size
	 */
	void parseBuffer(const uint8_t *buffer, int count);
	/**
	 * @brief: return param_list_msg to TP 
	 *
	 * @param param_list_msg: input==>parameter list store in API.txt
	 *
	 * @return: true if success 
	 */
	bool retParamListMsg(BaseTypes_ParameterListMsg param_list_msg);
	/**
	 * @brief: return status to TP
	 *
	 * @param: status_code: input==>status code
	 *
	 * @return: true if success
	 */
	bool retStatus(const char *path, int id, BaseTypes_StatusCode status_code);
	/**
	 * @brief :return previous_command_id_ to TP
	 *
	 * @param id: input 
	 *
	 * @return: true if Success
	 */
	bool retPreviousCommandId(int id);
	/**
	 * @brief: return current_command_id_ to TP
	 *
	 * @param id: input
	 *
	 * @return: true if Success
	 */
	bool retCurrentCommandId(int id);
	/**
	 * @brief :return parameter to TP with a template name T
	 *
	 * @tparam T: input ==>the template
	 * @param param: input
	 */
	template<typename T> 
	void retParamMsg(const char *path, int id, T param);
	/**
	 * @brief: set state publish time
	 *
	 * @param ms: input==> time in the form of millisecond
	 *
	 * @return 
	 */
	bool pubParamMsg(int id, const char *param);
	/**
	 * @brief: publish a parameter by the id
	 *
	 * @param id:input==>the uniq id of the parameter
	 */
	void pubParamByID(int id);

    /**
     * @brief 
     *
     * @param id
     * @param flag
     */
    void setUpdateFlagByID(int id, bool flag);
	/**
	 * @brief: parse the motion program 
	 *
	 * @param buffer: input==>the buffer to parse
	 * @param count: input==>the count of buffer
	 *
	 * @return: true if success
	 */
	bool parseMotionProgram(const uint8_t *buffer, int count);
    /**
	 * @brief
	 *
	 * @param motion_command: input==>the command to parse
	 * @param cmd_instruction: output==>store the result of parsing
	 *
	 * @return true if Success
	 */
	bool parseMotionCommand(motion_spec_MotionCommand motion_command, CommandInstruction &cmd_instruction);
	/**
	 * @brief: compare two strings in the form of int
	 *
	 * @param cmp_a: input
	 * @param cmp_b: input
	 *
	 * @return: true if they are the same 
	 */
	bool compareInt(const unsigned char *cmp_a, const uint8_t *cmp_b);

	template<typename T>
	bool checkPathAndID(T msg, uint32_t &id);

	/**
	 * @brief: set logic mode  
	 *
	 * @param id: input==>the parameter id
	 * @param mode_cmd: input==>the mode command
	 */
	void setProtoLogicMode(int id, RobotModeCmd mode_cmd);
	
	/**
	 * @brief: set logic state 
	 *
	 * @param id: input==>the parameter id
	 * @param state_cmd: input==>the state command
	 */
	void setProtoLogicState(int id, RobotStateCmd state_cmd);
	/**
	 * @brief: set joints trajectory 
	 *
	 * @param id: input==>the parameter id
	 * @param state_cmd: input==>joints pointer
	 */
	void setProtoJiontTraj(int id, string joints);
	/**
	 * @brief: set coordinates trajectory 
	 *
	 * @param id: input==>the parameter id
	 * @param state_cmd: input==>coordinates pointer
	 */
	void setProtoToolCoord(int id, string coordinates);

	/**
	 * @brief: add a parameter to publish list
	 *
	 * @param path: input==>the path of parameter
	 * @param id: input==>the parameter id
	 * @param has_freq: input==> if has frequency
	 * @param update_freq: input==> the frequency value
	 */
	void addPubParameter(const char *path, int id, bool has_freq, int update_freq);

	/**
	 * @brief: remove a paramet from the publish list 
	 *
	 * @param path: input==>the path of parameter
	 * @param  id: input==>the parameter id
	 */
	void removePubParameter(const char *path, int id);

	
	/**
	 * @brief: remove all publish list 
	 */
	void removeAllPubParams();

    /**
     * @brief: return error status 
     */
    void retErrorStatus();
	
};

#endif
