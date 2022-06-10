#ifndef _PROTOC_H_
#define _PROTOC_H_

#include "protocol/comm.pb.h"
#include "protocol/publish.pb.h"
#include "protocol/request_header.pb.h"
#include "protocol/response_header.pb.h"
#include "protocol/event_header.pb.h"

#include "request/request_base.pb.h"
#include "request/request_int32_bool.pb.h"
#include "request/request_int32_bool_coord_type_doublelist.pb.h"
#include "request/request_int32_coord_type.pb.h"
#include "request/request_int32_coord_type_doublelist.pb.h"
#include "request/request_int32_double.pb.h"
#include "request/request_int32_doublelist.pb.h"
#include "request/request_int32_int32list.pb.h"
#include "request/request_int32_uint32.pb.h"
#include "request/request_int32_uint32list.pb.h"
#include "request/request_int32list_core_comm_state.pb.h"
#include "request/request_int32list_double.pb.h"
#include "request/request_int32list_doublelist.pb.h"
#include "request/request_int32list_int32list.pb.h"
#include "request/request_int32list_int64.pb.h"
#include "request/request_int32_string.pb.h"
#include "request/request_publish_topic.pb.h"
#include "request/request_string_bytes.pb.h"
#include "request/request_stringlist.pb.h"
#include "request/request_work_mode.pb.h"
#include "request/request_int32list_uint32.pb.h"
#include "request/request_tool_frame.pb.h"
#include "request/request_coord_user.pb.h"
#include "request/request_int32_uftf_pose_and_posture.pb.h"
#include "request/request_int32_jointlimit.pb.h"
#include "request/request_reg_pr.pb.h"
#include "request/request_reg_mr.pb.h"
#include "request/request_reg_hr.pb.h"
#include "request/request_reg_sr.pb.h"
#include "request/request_reg_r.pb.h"
#include "request/request_string_double.pb.h"

#include "message_type/base.pb.h"
#include "message_type/axis_info.pb.h"
#include "message_type/axis_status.pb.h"
#include "message_type/coord_type.pb.h"
#include "message_type/core_comm_state.pb.h"
#include "message_type/group_status.pb.h"
#include "message_type/int32_doublelist.pb.h"
#include "message_type/int32_int32list.pb.h"
#include "message_type/param_detail_list.pb.h"
#include "message_type/publish_table.pb.h"
#include "message_type/publish_topic.pb.h"
#include "message_type/rpc_table.pb.h"
#include "message_type/string_int32.pb.h"
#include "message_type/string_list.pb.h"
#include "message_type/work_mode.pb.h"
#include "message_type/int64_int32list.pb.h"
#include "message_type/axis_feedback_list.pb.h"
#include "message_type/servo1000_servo_feedback_list.pb.h"
#include "message_type/scara_a_feedback_list.pb.h"
#include "message_type/servo_feedback_list.pb.h"
#include "message_type/tool_frame.pb.h"
#include "message_type/coord_user.pb.h"
#include "message_type/posture.pb.h"
#include "message_type/pose_and_posture.pb.h"
#include "message_type/uftf_pose_and_posture.pb.h"
#include "message_type/joint_limit.pb.h"
#include "message_type/reg_pr.pb.h"
#include "message_type/reg_mr.pb.h"
#include "message_type/reg_hr.pb.h"
#include "message_type/reg_sr.pb.h"
#include "message_type/reg_r.pb.h"
#include "message_type/string_double.pb.h"

#include "response/response_base.pb.h"
#include "response/response_bool_int32.pb.h"
#include "response/response_bool_int32list.pb.h"
#include "response/response_publish_table.pb.h"
#include "response/response_rpc_table.pb.h"
#include "response/response_uint64_axis_info.pb.h"
#include "response/response_uint64_axis_status.pb.h"
#include "response/response_uint64_bool.pb.h"
#include "response/response_uint64_bytes.pb.h"
#include "response/response_uint64_core_comm_state.pb.h"
#include "response/response_uint64_double.pb.h"
#include "response/response_uint64_doublelist.pb.h"
#include "response/response_uint64_group_status_bool.pb.h"
#include "response/response_uint64_int32.pb.h"
#include "response/response_uint64_int32list.pb.h"
#include "response/response_uint64_uint32.pb.h"
#include "response/response_uint64_param_detail_list.pb.h"
#include "response/response_uint64_string.pb.h"
#include "response/response_uint64_uint32list.pb.h"
#include "response/response_uint64_uint64.pb.h"
#include "response/response_uint64_uint64list.pb.h"
#include "response/response_uint64_work_mode.pb.h"
#include "response/response_uint64_int32_uint32.pb.h"
#include "response/response_tool_frame.pb.h"
#include "response/response_coord_user.pb.h"
#include "response/response_uint64_posture.pb.h"
#include "response/response_uint64_pose_and_posture.pb.h"
#include "response/response_uint64_jointlimit.pb.h"
#include "response/response_reg_pr.pb.h"
#include "response/response_reg_mr.pb.h"
#include "response/response_reg_hr.pb.h"
#include "response/response_reg_sr.pb.h"
#include "response/response_reg_r.pb.h"
#include "response/response_reg_base_summary.pb.h"

#include "event/event_base.pb.h"

#endif
