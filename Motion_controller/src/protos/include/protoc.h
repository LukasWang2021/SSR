#ifndef _PROTOC_H_
#define _PROTOC_H_

#include "protocal/comm.pb.h"
#include "protocal/publish.pb.h"
#include "protocal/request_header.pb.h"
#include "protocal/response_header.pb.h"

#include "request/request_base.pb.h"
#include "request/request_publish_topic.pb.h"
#include "request/request_register_pr.pb.h"
#include "request/request_register_r.pb.h"
#include "request/request_register_mr.pb.h"
#include "request/request_register_sr.pb.h"
#include "request/request_tool_frame.pb.h"
#include "request/request_user_frame.pb.h"

#include "message_type/base.pb.h"
#include "message_type/publish_table.pb.h"
#include "message_type/publish_topic.pb.h"
#include "message_type/rpc_table.pb.h"
#include "message_type/joint_list.pb.h"
#include "message_type/pose_euler.pb.h"
#include "message_type/register_mr.pb.h"
#include "message_type/register_pr.pb.h"
#include "message_type/register_r.pb.h"
#include "message_type/register_sr.pb.h"
#include "message_type/tool_frame.pb.h"
#include "message_type/user_frame.pb.h"


#include "response/response_base.pb.h"
#include "response/response_publish_table.pb.h"
#include "response/response_rpc_table.pb.h"
#include "response/response_register_pr.pb.h"
#include "response/response_register_r.pb.h"
#include "response/response_register_mr.pb.h"
#include "response/response_register_sr.pb.h"
#include "response/response_tool_frame.pb.h"
#include "response/response_user_frame.pb.h"
#endif