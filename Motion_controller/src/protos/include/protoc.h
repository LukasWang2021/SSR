#ifndef _PROTOC_H_
#define _PROTOC_H_

#include "protocal/comm.pb.h"
#include "protocal/publish.pb.h"
#include "protocal/request_header.pb.h"
#include "protocal/response_header.pb.h"

#include "request/request_base.pb.h"
#include "request/request_publish_topic.pb.h"
#include "request/request_tool_frame.pb.h"
#include "request/request_coord_user.pb.h"

#include "request/request_reg_hr.pb.h"
#include "request/request_reg_mr.pb.h"
#include "request/request_reg_pr.pb.h"
#include "request/request_reg_r.pb.h"
#include "request/request_reg_sr.pb.h"
#include "request/request_int32_doublelist.pb.h"
#include "request/request_int32list_doublelist.pb.h"
#include "request/request_int32_int32list.pb.h"
#include "request/request_int32_jointlimit.pb.h"

#include "message_type/base.pb.h"
#include "message_type/publish_table.pb.h"
#include "message_type/publish_topic.pb.h"
#include "message_type/rpc_table.pb.h"
#include "message_type/pose_euler.pb.h"
#include "message_type/joint_list.pb.h"
#include "message_type/tool_frame.pb.h"
#include "message_type/coord_user.pb.h"

#include "message_type/device_info.pb.h"
#include "message_type/int32_int32list.pb.h"
#include "message_type/int32_doublelist.pb.h"
#include "message_type/joint_limit.pb.h"
#include "message_type/string_int32.pb.h"

#include "message_type/reg_hr.pb.h"
#include "message_type/reg_mr.pb.h"
#include "message_type/reg_pr.pb.h"
#include "message_type/reg_r.pb.h"
#include "message_type/reg_sr.pb.h"
#include "message_type/reg_base_summary.pb.h"

#include "response/response_base.pb.h"
#include "response/response_publish_table.pb.h"
#include "response/response_rpc_table.pb.h"
#include "response/response_tool_frame.pb.h"
#include "response/response_coord_user.pb.h"

#include "response/response_reg_base_summary.pb.h"
#include "response/response_reg_hr.pb.h"
#include "response/response_reg_mr.pb.h"
#include "response/response_reg_r.pb.h"
#include "response/response_reg_pr.pb.h"
#include "response/response_reg_sr.pb.h"

#include "response/response_bool_int32.pb.h"
#include "response/response_bool_int32list.pb.h"
#include "response/response_device_info.pb.h"
#include "response/response_uint64_doublelist.pb.h"
#include "response/response_uint64_jointlimit.pb.h"

#include "response/response_uint64_int32.pb.h"
#include "response/response_uint64_int32list.pb.h"
#include "response/response_uint64_double.pb.h"


#endif
