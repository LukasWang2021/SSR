/**
 * @file proto_parse.h
 * @brief: change some of protocol 
 * @author Wang Wei
 * @version 2.0.1
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

#define ERROR_NUM                   (10)

#define MIN_IO_BASE_ID              (100000)

typedef struct _PublishUpdate
{
    int basic_freq;
    int max_freq;
	int count;
    bool update_flag;
    uint8_t* buffer;
    int buf_len;
}PublishUpdate;

class ProtoParse
{
  public:
    RosBasic *ros_basic_;
	
	ProtoParse(RosBasic *ros_basic);
	~ProtoParse();

	/**
	 * @brief: get pointer of RobotMotion object
	 *
	 * @return: the object 
	 */
	RobotMotion *getRobotMotionPtr();

    JsonParse* getJsonParserPtr();
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
	NNSocket	*nn_socket_;	//socket object pointer
	JsonParse	*json_parse_;	//
	RobotMotion robot_motion_;	//
	int			hash_size_;		//size of hash bytes	

	map<int, PublishUpdate>	param_pub_map_; //the map from parameter id to there publish time 
    ThreadSafeList<U64>     error_list_;
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
	 * @brief: parse the struct of overwrite message
	 *
	 * @param field_buffer: input==>the buffer pointer
	 * @param field_size: input==>the length of buffer
	 *
	 * @return:true if success 
	 */
    void parseParamOverwriteMsg(const uint8_t *field_buffer, int field_size);
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
     * @param param_info: input
     * @param status_code: input
     */
    void retStatus(BaseTypes_ParamInfo *param_info, BaseTypes_StatusCode status_code);
    void retStatus(BaseTypes_StatusCode status_code);
	/**
     * @brief 
     *
     * @param param_info: input
     * @param params: input
     */
    template<typename T> 
    void retParamMsg(BaseTypes_ParamInfo* param_info, T params);	

    /**
     * @brief: return IO message 
     *
     * @param param_msg: input
     */
    void retIOMsg(BaseTypes_ParameterMsg *param_msg);
    /**
	 * @brief: publish parameter 
     * 
	 * @param param: input
     * @param len: input
	 *
	 * @return 
	 */
    template<typename T>
	bool pubParamMsg(int id, T param, int len);
	/**
	 * @brief: publish a parameter by the id
	 *
	 * @param id:input==>the unique id of the parameter
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

	template<typename T>
	bool checkPathAndID(T msg, uint32_t &id);

	/**
	 * @brief: set logic mode  
	 *
	 * @param mode_cmd: input==>the mode command
	 */
	void setProtoLogicMode(RobotModeCmd mode_cmd);
	
	/**
	 * @brief: set logic state 
	 *
	 * @param state_cmd: input==>the state command
	 */
	void setProtoLogicState(RobotStateCmd state_cmd);
	/**
	 * @brief: set joints trajectory 
	 *
	 * @param state_cmd: input==>joints pointer
	 */
	void setProtoJiontTraj(const uint8_t *buffer, int count);
	/**
	 * @brief: set coordinates trajectory 
	 *
	 * @param state_cmd: input==>coordinates pointer
	 */
	void setProtoToolCoord(const uint8_t *buffer, int count);

	/**
	 * @brief: add a parameter to publish list
	 *
	 * @param id: input==>the parameter id
	 * @param has_freq: input==> if has frequency
	 * @param update_freq: input==> the frequency value
     * @param min_freq: input==> the min frequency value
     * @param buf_len: input==> length of publish buffer
	 */
	bool addPubParameter(int id, int update_freq, int min_freq, int buf_len);

	/**
	 * @brief: remove a paramet from the publish list 
	 *
	 * @param  id: input==>the parameter id
	 */
	void removePubParameter(int id);
	
	/**
	 * @brief: remove all publish list 
	 */
	void removeAllPubParams();

    /**
     * @brief: compare two strings in the form of int
     *
     * @param cmp_a: input
     * @param cmp_b: input
     *
     * @return: true if they are the same 
     */
    bool compareInt(const unsigned char *cmp_a, const uint8_t *cmp_b);

    /**
     * @brief: return error status 
     */
    void retErrorStatus();

    /**
     * @brief: clear error flag 
     */
    void clearUpdatedError();	

    /**
     * @brief: 
     *
     * @param buffer: input==>the buffer to parse
	 * @param count: input==>the count of buffer
     *
     * @return : 0 if success
     */
    U64 parseToolFrame(const uint8_t *buffer, int count);
     /**
     * @brief: 
     *
     * @param buffer: input==>the buffer to parse
	 * @param count: input==>the count of buffer
     *
     * @return : 0 if success
     */
    U64 parseUserFrame(const uint8_t *buffer, int count);

    /**
     * @brief 
     *
     * @param buffer: input==>the buffer to parse
	 * @param count: input==>the count of buffer
     *
     * @return : 0 if success
     */
    U64 parseJointConstraint(const uint8_t *buffer, int count);

    /**
     * @brief 
     *
     * @param buffer: input==>the buffer to parse
	 * @param count: input==>the count of buffer
     *
     * @return : 0 if success
     */
    U64 parseDHGroup(const uint8_t *buffer, int count);


    /**
     * @brief 
     *
     * @return: true if success 
     */
    bool retDeviceInfo(BaseTypes_ParamInfo* param_info);

};

#endif
