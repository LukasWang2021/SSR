/**
 * @file proto_define.h
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-08-29
 */
#ifndef TP_INTERFACE_PROTO_DEFINE_H_
#define TP_INTERFACE_PROTO_DEFINE_H_


#define PATH_LOGIC_MODE		("root/Logic/mode")
#define PATH_LOGIC_STATE	("root/Logic/state")
#define PATH_LOGIC_MODECMD	("root/Logic/modeCommand")
#define PATH_LOGIC_STATECMD	("root/Logic/stateCommand")

#define PATH_CTL_ACTUAL_TOOL_COORD		("root/Control/actualToolCoordinates")
#define PATH_CTL_ACTUAL_JOINT_POS		("root/Control/actualJointPositionsFiltered")
#define PATH_CTL_HOSTIN_TOOL_TRAJ		("root/Control/hostInToolTrajectory")
#define PATH_CTL_HOSTIN_JOINT_TRAJ		("root/Control/hostInJointTrajectory")
#define PATH_CTL_KINEMATICS_TOOL_COORD	("root/Control/forwardKinematics/toolCoordinates")

#define PATH_INTERP_PRE_CMD_ID			("root/MotionInterpreter/previous_command_id")
#define PATH_INTERP_CUR_CMD_ID			("root/MotionInterpreter/current_command_id")
#define PATH_INTERP_MOTION_PROGRAM		("root/MotionInterpreter/motion_program")

#define PATH_INPUT_IO1		("io/system/dInUser1")
#define PATH_INPUT_IO2		("io/system/dInUser2")
#define PATH_INPUT_IO3		("io/system/dInUser3")
#define PATH_INPUT_IO4		("io/system/dInUser4")

#define PATH_OUTPUT_IO1		("io/system/dOutUser1")
#define PATH_OUTPUT_IO2		("io/system/dOutUser2")
#define PATH_OUTPUT_IO3		("io/system/dOutUser3")
#define PATH_OUTPUT_IO4		("io/system/dOutUser4")

#define PATH_CTL_COMMAND    ("root/Control/fst/ControlCommand")
#define PATH_ERROR_WARNINGS ("root/Logic/activeWarnings")

#define PATH_ETHERCAT_SMNTIO_IN3    ("root/Ethercat_IO/SomanetIO/dInUser1")
#define PATH_ETHERCAT_SMNTIO_IN4    ("root/Ethercat_IO/SomanetIO/dInUser2")
#define PATH_ETHERCAT_SMNTIO_IN3    ("root/Ethercat_IO/SomanetIO/dInUser3")
#define PATH_ETHERCAT_SMNTIO_IN4    ("root/Ethercat_IO/SomanetIO/dInUser4")
#define PATH_ETHERCAT_SMNTIO_IN3    ("root/Ethercat_IO/SomanetIO/dInUser5")
#define PATH_ETHERCAT_SMNTIO_IN4    ("root/Ethercat_IO/SomanetIO/dInUser6")
#define PATH_ETHERCAT_SMNTIO_IN3    ("root/Ethercat_IO/SomanetIO/dInUser7")
#define PATH_ETHERCAT_SMNTIO_IN4    ("root/Ethercat_IO/SomanetIO/dInUser8")


#define PATH_CTL_SYSTEM_SHUTDOWN    ("root/Control/system/shutdown")


#define	FK_TOOL_COORD_ID		(4581)
#define HOSTIN_JOINT_TRAJ_ID	(4632)
#define HOSTIN_TOOL_COORD_ID	(4633)
#define ACTUAL_JOINT_POS_ID		(4667)
#define ACTUAL_TOOL_COORD_ID	(4670)

#define DIN_ESTOP_NOT_ACTIVE_ID	(4694)
#define DIN_DEADMAN_ENABLED_ID	(4695)
#define DIN_MANUAL_MODE_ID		(4696)
#define DIN_AUTO_MODE_ID		(4697)
#define DIN_INTERNAL_SPARE_ID	(4698)
#define DIN_EXTERN_ENABLE_ID	(4699)
#define DIN_START_ID			(4700)
#define DIN_ACKNOWLEDGE_ID		(4701)
#define DIN_USER1_ID			(4702)
#define DIN_USER2_ID			(4703)
#define DIN_USER3_ID			(4704)
#define DIN_USER4_ID			(4705)
#define DOUT_ESTOP				(4706)
#define DOUT_RESET_ESTOP_ID		(4707)
#define DOUT_ENABLE_DRIVERS_ID	(4708)
#define DOUT_SLOW_SPEED_ID		(4709)
#define	DOUT_INTERNAL_SPARE_ID	(4710)
#define DOUT_SYSTEM_RUNNING_ID	(4711)
#define DOUT_NO_ERROR_ACTIVE_ID	(4712)
#define DOUT_PROGRAM_RUNNING_ID	(4713)
#define DOUT_USER1_ID			(4714)
#define DOUT_USER2_ID			(4715)
#define DOUT_USER3_ID			(4716)
#define DOUT_USER4_ID			(4717)

#define LOGIC_MODE_ID			(4737)
#define MODE_COMMAND_ID			(4738)
#define LOGIC_STATE_ID			(4739)
#define STATE_COMMAND_ID		(4740)
#define ID_CURRENT_COMMAND_ID	(4747)
#define ID_PREVIOUS_COMMAND_ID	(4748)

#define CTL_COMMAND_ID          (9001)
#define ERROR_WARNINGS_ID       (5107)

#define ETHERCAT_SMNT_DIN1_ID   (363)
#define ETHERCAT_SMNT_DIN2_ID   (364)
#define ETHERCAT_SMNT_DIN3_ID   (365)
#define ETHERCAT_SMNT_DIN4_ID   (366)

#define SAFETY_INPUT_FRAME1_ID  (1000)
#define SAFETY_INPUT_FRAME2_ID  (1001)



/**
 * @brief 
 *
 * @param obj: field object
 * @param field: field to find 
 * @param field_buffer:recieved buffer from TP
 * @param field_size: size of the field
 * 
 */
#define PARSE_FIELD(obj, field, field_buffer, field_size, ret)\
		do\
		{\
			pb_istream_t stream = {0};\
			stream = pb_istream_from_buffer(field_buffer, field_size);\
			ret = pb_decode(&stream, field##_fields, &obj);\
		}\
		while (0)
/**
 * @brief 
 *
 * @param obj: field object
 * @param field: field to find 
 * @param field_buffer:buffer to send to TP
 * @param buffer_size: size of the buffer
 * @param bytes_written: bytes that have been written
 *
 */

#define SET_FIELD(obj, field, field_buffer, buffer_size, hash_size_, bytes, ret)\
		do\
		{\
			memcpy(field_buffer, get_hash<field>(), hash_size_);\
			pb_ostream_t stream = {0};\
			stream = pb_ostream_from_buffer(field_buffer+hash_size_, buffer_size-hash_size_);\
			ret = pb_encode(&stream, field##_fields, &obj);\
			bytes = stream.bytes_written+hash_size_;\
		}\
		while (0)


#define HASH_CMP(field, buffer)	compareInt(get_hash<BaseTypes_##field>(), buffer)

#endif
