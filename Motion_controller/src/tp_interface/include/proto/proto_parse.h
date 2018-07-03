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
#include <vector>
#include "base_types_hash.h"
#include "motionSL.pb.h"
#include "proto_define.h"
#include "instruction.h"
#include "version.pb.h"
#include "frameSL.pb.h"
#include "registerSL.pb.h"

class ProtoParse 
{
  public:
	ProtoParse();
	~ProtoParse();

    static ProtoParse* instance();

    BaseTypes_ParamInfo* getIOInfo();
    BaseTypes_ParamInfo* getRegInfo();

    BaseTypes_ParamInfo* getInfoByID(uint32_t id);

    void decDefault(const uint8_t *in_buf, int in_len, void *out_buf);
    void encDefault(const uint8_t *in_buf, int in_len, void *out_buf);
   // void decMotionProgram(const uint8_t *in_buf, int in_len, void *out_buf);
    void decSoftConstraint(const uint8_t *in_buf, int in_len, void *out_buf);
    void decManualCmd(const uint8_t *in_buf, int in_len, void *out_buf);
    void decTeachTarget(const uint8_t *in_buf, int in_len, void *out_buf);
    int checkPath(char *path);

    void encIOInfo(const uint8_t *in_buf, int in_len, void *out_buf);
    void encSoftConstraint(const uint8_t *in_buf, int in_len, void *out_buf);
    void encDHParameters(const uint8_t *in_buf, int in_len, void *out_buf);
    void encHardConstraint(const uint8_t *in_buf, int in_len, void *out_buf);

    void decFrame(const uint8_t *in_buf, int in_len, void *out_buf);
    void encFrame(const uint8_t *in_buf, int in_len, void *out_buf);
    void decActivateFrame(const uint8_t *in_buf, int in_len, void *out_buf);
    void encActivateFrame(const uint8_t *in_buf, int in_len, void *out_buf);
    void decValidSimpleFrame(const uint8_t *in_buf, int in_len, void *out_buf);
    void encValidSimpleFrame(const uint8_t *in_buf, int in_len, void *out_buf);

    void encString(const uint8_t *in_buf, int in_len, void *out_buf);

    void encPoseRegister(const uint8_t *in_buf, int in_len, void *out_buf);
    void decPoseRegister(const uint8_t *in_buf, int in_len, void *out_buf);
    void encNumberRegister(const uint8_t *in_buf, int in_len, void *out_buf);
    void decNumberRegister(const uint8_t *in_buf, int in_len, void *out_buf);
    void encSR(const uint8_t *in_buf, int in_len, void *out_buf);
    void decSR(const uint8_t *in_buf, int in_len, void *out_buf);
    void encMR(const uint8_t *in_buf, int in_len, void *out_buf);
    void decMR(const uint8_t *in_buf, int in_len, void *out_buf);

    void encGlobalAcc(const uint8_t *in_buf, int in_len, void *out_buf);
	void decGlobalAcc(const uint8_t *in_buf, int in_len, void *out_buf);

    void encValidSimpleReg(const uint8_t *in_buf, int in_len, void *out_buf);

    //void encManualCmd(const uint8_t *in_buf, int in_len, void *out_buf){}

    bool decParamSetMsg(const uint8_t *in_buf, int in_len, BaseTypes_ParameterSetMsg &param_set_msg);

    bool decParamGetMsg(const uint8_t *in_buf, int in_len, BaseTypes_ParameterGetMsg &param_get_msg);
    bool decParamCmdMsg(const uint8_t *in_buf, int in_len, BaseTypes_ParameterCmdMsg &param_cmd_msg);

    bool encStatus(BaseTypes_StatusCode status_code, BaseTypes_ParamInfo *param_info, uint8_t* buf_out, int buf_len, int& bytes_written);


    bool encParamMsg(const BaseTypes_ParameterMsg &param_msg, uint8_t *buf_out, int buf_len, int& bytes_written);
    bool encPubGroupMsg(motion_spec_SignalGroup *sig_group, uint8_t *buf_out, int buf_len, int& bytes_written);

    bool encParamListMsg(uint8_t *buf_out, int buf_len, int& bytes_written);

    void encVersionInfo(const uint8_t *in_buf, int in_len, void *out_buf);
  private:
    int hash_size_;
    std::vector<CommandInstruction> inst_list_;
	/**
	 * @brief: parse the motion program 
	 *
	 * @param buffer: input==>the buffer to parse
	 * @param count: input==>the count of buffer
	 *
	 * @return: true if success:
	 */
	//bool decMotionProgram(const uint8_t *buffer, int count);
    /**
	 * @brief
	 *
	 * @param motion_command: input==>the command to parse
	 * @param cmd_instruction: output==>store the result of parsing
	 *
	 * @return true if Success
	 */
//	bool decMotionCommand(motion_spec_MotionCommand motion_command, CommandInstruction &cmd_instruction);
    
};

#endif
