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
#include "request/request_file.pb.h"

#include "request/request_reg_hr.pb.h"
#include "request/request_reg_mr.pb.h"
#include "request/request_reg_pr.pb.h"
#include "request/request_reg_r.pb.h"
#include "request/request_reg_sr.pb.h"
#include "request/request_int32_doublelist.pb.h"
#include "request/request_int32list_doublelist.pb.h"
#include "request/request_int32_int32list.pb.h"
#include "request/request_int32_jointlimit.pb.h"
#include "request/request_int32_double.pb.h"
#include "request/request_int32list_double.pb.h"
#include "request/request_stringlist.pb.h"
#include "request/request_int32_bool.pb.h"
#include "request/request_string_bytes.pb.h"

#include "request/request_modbus_start_mode.pb.h"
#include "request/request_int32_modbus_reg_info.pb.h"
#include "request/request_int32_modbus_status_info.pb.h"
#include "request/request_int32_modbus_function_addr_info.pb.h"
#include "request/request_modbus_server_all_function_addr_info.pb.h"
#include "request/request_modbus_server_start_info.pb.h"
#include "request/request_modbus_client_start_info.pb.h"
#include "request/request_int32_modbus_all_function_addr_info.pb.h"
#include "request/request_int32_modbus_client_start_info.pb.h"

#include "request/request_int32_param_info.pb.h"

#include "message_type/base.pb.h"
#include "message_type/publish_table.pb.h"
#include "message_type/publish_topic.pb.h"
#include "message_type/rpc_table.pb.h"
#include "message_type/joint_list.pb.h"
#include "message_type/tool_frame.pb.h"
#include "message_type/coord_user.pb.h"
#include "message_type/axis_group_info.pb.h"

#include "message_type/device_info.pb.h"
#include "message_type/int32_int32list.pb.h"
#include "message_type/int32_doublelist.pb.h"
#include "message_type/joint_limit.pb.h"
#include "message_type/joint_limit_with_unit.pb.h"
#include "message_type/string_int32.pb.h"
#include "message_type/string_list.pb.h"
#include "message_type/file.pb.h"

#include "message_type/reg_hr.pb.h"
#include "message_type/reg_mr.pb.h"
#include "message_type/reg_pr.pb.h"
#include "message_type/reg_r.pb.h"
#include "message_type/reg_sr.pb.h"
#include "message_type/reg_base_summary.pb.h"
#include "message_type/reg_type.pb.h"
#include "message_type/reg_pr_value.pb.h"
#include "message_type/reg_hr_value.pb.h"
#include "message_type/reg_mr_value.pb.h"
#include "message_type/reg_sr_value.pb.h"
#include "message_type/reg_r_value.pb.h"

#include "message_type/io_type.pb.h"
#include "message_type/io_device_info.pb.h"
#include "message_type/io_board_status_list.pb.h"
#include "message_type/device_version.pb.h"

#include "message_type/timeval.pb.h"

#include "message_type/modbus_start_mode.pb.h"
#include "message_type/modbus_function.pb.h"
#include "message_type/modbus_server_config.pb.h"
#include "message_type/modbus_server_start_info.pb.h"
#include "message_type/modbus_reg_info.pb.h"
#include "message_type/modbus_status_info.pb.h"
#include "message_type/modbus_client_config_params.pb.h"
#include "message_type/modbus_client_start_info.pb.h"
#include "message_type/modbus_client_summary_start_info_list.pb.h"
#include "message_type/modbus_client_ctrl_status.pb.h"

#include "message_type/param_info.pb.h"

#include "response/response_base.pb.h"
#include "response/response_publish_table.pb.h"
#include "response/response_rpc_table.pb.h"
#include "response/response_tool_frame.pb.h"
#include "response/response_coord_user.pb.h"
#include "response/response_axis_group_info.pb.h"
#include "response/response_joint_limit_with_unit.pb.h"

#include "response/response_reg_base_summary.pb.h"
#include "response/response_reg_hr.pb.h"
#include "response/response_reg_mr.pb.h"
#include "response/response_reg_r.pb.h"
#include "response/response_reg_pr.pb.h"
#include "response/response_reg_sr.pb.h"

#include "response/response_bool_int32.pb.h"
#include "response/response_bool_int32list.pb.h"
#include "response/response_device_info.pb.h"
#include "response/response_io_device_info.pb.h"
#include "response/response_io_device_info_list.pb.h"
#include "response/response_uint64_doublelist.pb.h"
#include "response/response_uint64_jointlimit.pb.h"

#include "response/response_uint64_bytes.pb.h"
#include "response/response_uint64_int32.pb.h"
#include "response/response_uint64_int32list.pb.h"
#include "response/response_uint64_double.pb.h"
#include "response/response_uint64_bool.pb.h"
#include "response/response_uint64_string.pb.h"
#include "response/response_uint64_uint64list.pb.h"
#include "response/response_file.pb.h"
#include "response/response_uint64_device_version.pb.h"

#include "response/response_modbus_start_mode.pb.h"
#include "response/response_uint64_modbus_all_funtion_addr_info.pb.h"
#include "response/response_uint64_modbus_server_config_params.pb.h"
#include "response/response_uint64_modbus_server_start_info.pb.h"
#include "response/response_uint64_modbus_status_info.pb.h"
#include "response/response_uint64_modbus_reg_info.pb.h"
#include "response/response_uint64_modbus_client_config_params.pb.h"
#include "response/response_uint64_modbus_client_start_info.pb.h"
#include "response/response_uint64_modbus_client_summary_start_info_list.pb.h"

#include "response/response_uint64_param_info_list.pb.h"
#endif
