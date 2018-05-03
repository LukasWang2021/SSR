/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.7-dev at Sat Apr 28 10:40:38 2018. */

#include "base_types.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

const uint32_t BaseTypes_ParameterCmdMsg_update_frq_devider_default = 30u;
const uint32_t BaseTypes_ParameterCmdMsg_minimum_frq_devider_default = 2000u;
const uint32_t BaseTypes_CreatePlotMsg_frq_devider_default = 100u;


const pb_field_t BaseTypes_Point_fields[4] = {
    PB_FIELD(  1, DOUBLE  , REQUIRED, STATIC  , FIRST, BaseTypes_Point, x, x, 0),
    PB_FIELD(  2, DOUBLE  , REQUIRED, STATIC  , OTHER, BaseTypes_Point, y, x, 0),
    PB_FIELD(  3, DOUBLE  , REQUIRED, STATIC  , OTHER, BaseTypes_Point, z, y, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_Euler_fields[4] = {
    PB_FIELD(  1, DOUBLE  , REQUIRED, STATIC  , FIRST, BaseTypes_Euler, a, a, 0),
    PB_FIELD(  2, DOUBLE  , REQUIRED, STATIC  , OTHER, BaseTypes_Euler, b, a, 0),
    PB_FIELD(  3, DOUBLE  , REQUIRED, STATIC  , OTHER, BaseTypes_Euler, c, b, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_Joint_fields[10] = {
    PB_FIELD(  1, DOUBLE  , REQUIRED, STATIC  , FIRST, BaseTypes_Joint, j1, j1, 0),
    PB_FIELD(  2, DOUBLE  , REQUIRED, STATIC  , OTHER, BaseTypes_Joint, j2, j1, 0),
    PB_FIELD(  3, DOUBLE  , REQUIRED, STATIC  , OTHER, BaseTypes_Joint, j3, j2, 0),
    PB_FIELD(  4, DOUBLE  , REQUIRED, STATIC  , OTHER, BaseTypes_Joint, j4, j3, 0),
    PB_FIELD(  5, DOUBLE  , REQUIRED, STATIC  , OTHER, BaseTypes_Joint, j5, j4, 0),
    PB_FIELD(  6, DOUBLE  , REQUIRED, STATIC  , OTHER, BaseTypes_Joint, j6, j5, 0),
    PB_FIELD(  7, DOUBLE  , REQUIRED, STATIC  , OTHER, BaseTypes_Joint, j7, j6, 0),
    PB_FIELD(  8, DOUBLE  , REQUIRED, STATIC  , OTHER, BaseTypes_Joint, j8, j7, 0),
    PB_FIELD(  9, DOUBLE  , REQUIRED, STATIC  , OTHER, BaseTypes_Joint, j9, j8, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_RegMap_fields[4] = {
    PB_FIELD(  1, UENUM   , REQUIRED, STATIC  , FIRST, BaseTypes_RegMap, type, type, 0),
    PB_FIELD(  2, INT32   , REQUIRED, STATIC  , OTHER, BaseTypes_RegMap, index, type, 0),
    PB_FIELD(  3, BYTES   , REQUIRED, CALLBACK, OTHER, BaseTypes_RegMap, value, index, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_PoseEuler_fields[3] = {
    PB_FIELD(  1, MESSAGE , REQUIRED, STATIC  , FIRST, BaseTypes_PoseEuler, position, position, &BaseTypes_Point_fields),
    PB_FIELD(  2, MESSAGE , REQUIRED, STATIC  , OTHER, BaseTypes_PoseEuler, orientation, position, &BaseTypes_Euler_fields),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_PRShmi_fields[6] = {
    PB_FIELD(  1, MESSAGE , REQUIRED, STATIC  , FIRST, BaseTypes_PRShmi, pose, pose, &BaseTypes_PoseEuler_fields),
    PB_FIELD(  2, MESSAGE , REQUIRED, STATIC  , OTHER, BaseTypes_PRShmi, joint, pose, &BaseTypes_Joint_fields),
    PB_FIELD(  3, INT32   , REQUIRED, STATIC  , OTHER, BaseTypes_PRShmi, type, joint, 0),
    PB_FIELD(  4, INT32   , REQUIRED, STATIC  , OTHER, BaseTypes_PRShmi, id, type, 0),
    PB_FIELD(  5, BYTES   , REQUIRED, CALLBACK, OTHER, BaseTypes_PRShmi, comment, id, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_SRShmi_fields[4] = {
    PB_FIELD(  1, BYTES   , REQUIRED, CALLBACK, FIRST, BaseTypes_SRShmi, value, value, 0),
    PB_FIELD(  2, INT32   , REQUIRED, STATIC  , OTHER, BaseTypes_SRShmi, id, value, 0),
    PB_FIELD(  3, BYTES   , REQUIRED, CALLBACK, OTHER, BaseTypes_SRShmi, comment, id, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_RShmi_fields[4] = {
    PB_FIELD(  1, DOUBLE  , REQUIRED, STATIC  , FIRST, BaseTypes_RShmi, value, value, 0),
    PB_FIELD(  2, INT32   , REQUIRED, STATIC  , OTHER, BaseTypes_RShmi, id, value, 0),
    PB_FIELD(  3, BYTES   , REQUIRED, CALLBACK, OTHER, BaseTypes_RShmi, comment, id, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_MRShmi_fields[4] = {
    PB_FIELD(  1, INT32   , REQUIRED, STATIC  , FIRST, BaseTypes_MRShmi, value, value, 0),
    PB_FIELD(  2, INT32   , REQUIRED, STATIC  , OTHER, BaseTypes_MRShmi, id, value, 0),
    PB_FIELD(  3, BYTES   , REQUIRED, CALLBACK, OTHER, BaseTypes_MRShmi, comment, id, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_TFShmi_fields[4] = {
    PB_FIELD(  1, MESSAGE , REQUIRED, STATIC  , FIRST, BaseTypes_TFShmi, c, c, &BaseTypes_PoseEuler_fields),
    PB_FIELD(  2, INT32   , REQUIRED, STATIC  , OTHER, BaseTypes_TFShmi, id, c, 0),
    PB_FIELD(  3, BYTES   , REQUIRED, CALLBACK, OTHER, BaseTypes_TFShmi, comment, id, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_UFShmi_fields[4] = {
    PB_FIELD(  1, MESSAGE , REQUIRED, STATIC  , FIRST, BaseTypes_UFShmi, c, c, &BaseTypes_PoseEuler_fields),
    PB_FIELD(  2, INT32   , REQUIRED, STATIC  , OTHER, BaseTypes_UFShmi, id, c, 0),
    PB_FIELD(  3, BYTES   , REQUIRED, CALLBACK, OTHER, BaseTypes_UFShmi, comment, id, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_ParamInfo_fields[11] = {
    PB_FIELD(  1, STRING  , REQUIRED, STATIC  , FIRST, BaseTypes_ParamInfo, path, path, 0),
    PB_FIELD(  2, UINT32  , REQUIRED, STATIC  , OTHER, BaseTypes_ParamInfo, id, path, 0),
    PB_FIELD(  3, BOOL    , REQUIRED, STATIC  , OTHER, BaseTypes_ParamInfo, overwrite_active, id, 0),
    PB_FIELD(  4, UINT32  , REQUIRED, STATIC  , OTHER, BaseTypes_ParamInfo, data_type, overwrite_active, 0),
    PB_FIELD(  5, UINT32  , REQUIRED, STATIC  , OTHER, BaseTypes_ParamInfo, data_size, data_type, 0),
    PB_FIELD(  6, UINT32  , REQUIRED, STATIC  , OTHER, BaseTypes_ParamInfo, number_of_elements, data_size, 0),
    PB_FIELD(  7, UENUM   , REQUIRED, STATIC  , OTHER, BaseTypes_ParamInfo, param_type, number_of_elements, 0),
    PB_FIELD(  8, UENUM   , REQUIRED, STATIC  , OTHER, BaseTypes_ParamInfo, permission, param_type, 0),
    PB_FIELD(  9, UENUM   , REQUIRED, STATIC  , OTHER, BaseTypes_ParamInfo, user_level, permission, 0),
    PB_FIELD( 10, UENUM   , REQUIRED, STATIC  , OTHER, BaseTypes_ParamInfo, unit, user_level, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_Header_fields[3] = {
    PB_FIELD(  1, UINT32  , REQUIRED, STATIC  , FIRST, BaseTypes_Header, frameCounter, frameCounter, 0),
    PB_FIELD(  2, UINT64  , REQUIRED, STATIC  , OTHER, BaseTypes_Header, timestamp, frameCounter, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_ParameterMsg_fields[4] = {
    PB_FIELD(  1, MESSAGE , OPTIONAL, STATIC  , FIRST, BaseTypes_ParameterMsg, header, header, &BaseTypes_Header_fields),
    PB_FIELD(  2, MESSAGE , OPTIONAL, STATIC  , OTHER, BaseTypes_ParameterMsg, info, header, &BaseTypes_ParamInfo_fields),
    PB_FIELD(  3, BYTES   , REQUIRED, STATIC  , OTHER, BaseTypes_ParameterMsg, param, info, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_ParameterListMsg_fields[3] = {
    PB_FIELD(  1, MESSAGE , OPTIONAL, STATIC  , FIRST, BaseTypes_ParameterListMsg, header, header, &BaseTypes_Header_fields),
    PB_FIELD(  2, MESSAGE , REPEATED, CALLBACK, OTHER, BaseTypes_ParameterListMsg, params, header, &BaseTypes_ParameterMsg_fields),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_ParameterGetMsg_fields[5] = {
    PB_FIELD(  1, MESSAGE , OPTIONAL, STATIC  , FIRST, BaseTypes_ParameterGetMsg, header, header, &BaseTypes_Header_fields),
    PB_FIELD(  2, STRING  , OPTIONAL, STATIC  , OTHER, BaseTypes_ParameterGetMsg, path, header, 0),
    PB_FIELD(  3, UINT32  , OPTIONAL, STATIC  , OTHER, BaseTypes_ParameterGetMsg, id, path, 0),
    PB_FIELD(  4, BYTES   , OPTIONAL, STATIC  , OTHER, BaseTypes_ParameterGetMsg, param, id, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_ParameterGetListMsg_fields[3] = {
    PB_FIELD(  1, MESSAGE , OPTIONAL, STATIC  , FIRST, BaseTypes_ParameterGetListMsg, header, header, &BaseTypes_Header_fields),
    PB_FIELD(  2, MESSAGE , REPEATED, STATIC  , OTHER, BaseTypes_ParameterGetListMsg, params, header, &BaseTypes_ParameterGetMsg_fields),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_ParameterSetMsg_fields[6] = {
    PB_FIELD(  1, MESSAGE , OPTIONAL, STATIC  , FIRST, BaseTypes_ParameterSetMsg, header, header, &BaseTypes_Header_fields),
    PB_FIELD(  2, STRING  , OPTIONAL, STATIC  , OTHER, BaseTypes_ParameterSetMsg, path, header, 0),
    PB_FIELD(  3, UINT32  , OPTIONAL, STATIC  , OTHER, BaseTypes_ParameterSetMsg, id, path, 0),
    PB_FIELD(  4, UINT32  , REQUIRED, STATIC  , OTHER, BaseTypes_ParameterSetMsg, number_of_elements, id, 0),
    PB_FIELD(  5, BYTES   , REQUIRED, STATIC  , OTHER, BaseTypes_ParameterSetMsg, param, number_of_elements, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_ParameterSetListMsg_fields[3] = {
    PB_FIELD(  1, MESSAGE , OPTIONAL, STATIC  , FIRST, BaseTypes_ParameterSetListMsg, header, header, &BaseTypes_Header_fields),
    PB_FIELD(  2, MESSAGE , REPEATED, STATIC  , OTHER, BaseTypes_ParameterSetListMsg, params, header, &BaseTypes_ParameterSetMsg_fields),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_ParameterOverwriteMsg_fields[6] = {
    PB_FIELD(  1, MESSAGE , OPTIONAL, STATIC  , FIRST, BaseTypes_ParameterOverwriteMsg, header, header, &BaseTypes_Header_fields),
    PB_FIELD(  2, STRING  , OPTIONAL, STATIC  , OTHER, BaseTypes_ParameterOverwriteMsg, path, header, 0),
    PB_FIELD(  3, UINT32  , OPTIONAL, STATIC  , OTHER, BaseTypes_ParameterOverwriteMsg, id, path, 0),
    PB_FIELD(  4, UINT32  , REQUIRED, STATIC  , OTHER, BaseTypes_ParameterOverwriteMsg, number_of_elements, id, 0),
    PB_FIELD(  5, BYTES   , REQUIRED, STATIC  , OTHER, BaseTypes_ParameterOverwriteMsg, param, number_of_elements, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_ParameterCmdMsg_fields[7] = {
    PB_FIELD(  1, MESSAGE , OPTIONAL, STATIC  , FIRST, BaseTypes_ParameterCmdMsg, header, header, &BaseTypes_Header_fields),
    PB_FIELD(  2, STRING  , OPTIONAL, STATIC  , OTHER, BaseTypes_ParameterCmdMsg, path, header, 0),
    PB_FIELD(  3, UINT32  , OPTIONAL, STATIC  , OTHER, BaseTypes_ParameterCmdMsg, id, path, 0),
    PB_FIELD(  4, UENUM   , REQUIRED, STATIC  , OTHER, BaseTypes_ParameterCmdMsg, cmd, id, 0),
    PB_FIELD(  5, UINT32  , OPTIONAL, STATIC  , OTHER, BaseTypes_ParameterCmdMsg, update_frq_devider, cmd, &BaseTypes_ParameterCmdMsg_update_frq_devider_default),
    PB_FIELD(  6, UINT32  , OPTIONAL, STATIC  , OTHER, BaseTypes_ParameterCmdMsg, minimum_frq_devider, update_frq_devider, &BaseTypes_ParameterCmdMsg_minimum_frq_devider_default),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_ParameterCmdListMsg_fields[3] = {
    PB_FIELD(  1, MESSAGE , OPTIONAL, STATIC  , FIRST, BaseTypes_ParameterCmdListMsg, header, header, &BaseTypes_Header_fields),
    PB_FIELD(  2, MESSAGE , REPEATED, STATIC  , OTHER, BaseTypes_ParameterCmdListMsg, cmds, header, &BaseTypes_ParameterCmdMsg_fields),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_StatusMsg_fields[4] = {
    PB_FIELD(  1, MESSAGE , OPTIONAL, STATIC  , FIRST, BaseTypes_StatusMsg, header, header, &BaseTypes_Header_fields),
    PB_FIELD(  2, MESSAGE , OPTIONAL, STATIC  , OTHER, BaseTypes_StatusMsg, info, header, &BaseTypes_ParamInfo_fields),
    PB_FIELD(  3, UENUM   , REQUIRED, STATIC  , OTHER, BaseTypes_StatusMsg, status, info, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_ConsoleCmdMsg_fields[3] = {
    PB_FIELD(  1, MESSAGE , OPTIONAL, STATIC  , FIRST, BaseTypes_ConsoleCmdMsg, header, header, &BaseTypes_Header_fields),
    PB_FIELD(  2, STRING  , REQUIRED, STATIC  , OTHER, BaseTypes_ConsoleCmdMsg, val, header, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_ConsoleCmdListMsg_fields[3] = {
    PB_FIELD(  1, MESSAGE , OPTIONAL, STATIC  , FIRST, BaseTypes_ConsoleCmdListMsg, header, header, &BaseTypes_Header_fields),
    PB_FIELD(  2, MESSAGE , REPEATED, STATIC  , OTHER, BaseTypes_ConsoleCmdListMsg, cmds, header, &BaseTypes_ConsoleCmdMsg_fields),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_LogMsg_fields[3] = {
    PB_FIELD(  1, MESSAGE , OPTIONAL, STATIC  , FIRST, BaseTypes_LogMsg, header, header, &BaseTypes_Header_fields),
    PB_FIELD(  2, STRING  , REQUIRED, STATIC  , OTHER, BaseTypes_LogMsg, val, header, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_LogListMsg_fields[3] = {
    PB_FIELD(  1, MESSAGE , OPTIONAL, STATIC  , FIRST, BaseTypes_LogListMsg, header, header, &BaseTypes_Header_fields),
    PB_FIELD(  2, MESSAGE , REPEATED, STATIC  , OTHER, BaseTypes_LogListMsg, logs, header, &BaseTypes_LogMsg_fields),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_CreatePlotMsg_fields[4] = {
    PB_FIELD(  1, UINT32  , REQUIRED, STATIC  , FIRST, BaseTypes_CreatePlotMsg, id, id, 0),
    PB_FIELD(  2, UINT32  , REQUIRED, STATIC  , OTHER, BaseTypes_CreatePlotMsg, frq_devider, id, &BaseTypes_CreatePlotMsg_frq_devider_default),
    PB_FIELD(  3, STRING  , REPEATED, STATIC  , OTHER, BaseTypes_CreatePlotMsg, paths, frq_devider, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_RemovePlotMsg_fields[2] = {
    PB_FIELD(  1, UINT32  , REQUIRED, STATIC  , FIRST, BaseTypes_RemovePlotMsg, id, id, 0),
    PB_LAST_FIELD
};

const pb_field_t BaseTypes_CommonString_fields[2] = {
    PB_FIELD(  1, STRING  , REQUIRED, STATIC  , FIRST, BaseTypes_CommonString, data, data, 0),
    PB_LAST_FIELD
};


/* Check that field information fits in pb_field_t */
#if !defined(PB_FIELD_32BIT)
/* If you get an error here, it means that you need to define PB_FIELD_32BIT
 * compile-time option. You can do that in pb.h or on compiler command line.
 * 
 * The reason you need to do this is that some of your messages contain tag
 * numbers or field sizes that are larger than what can fit in 8 or 16 bit
 * field descriptors.
 */
PB_STATIC_ASSERT((pb_membersize(BaseTypes_PoseEuler, position) < 65536 && pb_membersize(BaseTypes_PoseEuler, orientation) < 65536 && pb_membersize(BaseTypes_PRShmi, pose) < 65536 && pb_membersize(BaseTypes_PRShmi, joint) < 65536 && pb_membersize(BaseTypes_TFShmi, c) < 65536 && pb_membersize(BaseTypes_UFShmi, c) < 65536 && pb_membersize(BaseTypes_ParameterMsg, header) < 65536 && pb_membersize(BaseTypes_ParameterMsg, info) < 65536 && pb_membersize(BaseTypes_ParameterListMsg, header) < 65536 && pb_membersize(BaseTypes_ParameterListMsg, params) < 65536 && pb_membersize(BaseTypes_ParameterGetMsg, header) < 65536 && pb_membersize(BaseTypes_ParameterGetListMsg, header) < 65536 && pb_membersize(BaseTypes_ParameterGetListMsg, params[0]) < 65536 && pb_membersize(BaseTypes_ParameterSetMsg, header) < 65536 && pb_membersize(BaseTypes_ParameterSetListMsg, header) < 65536 && pb_membersize(BaseTypes_ParameterSetListMsg, params[0]) < 65536 && pb_membersize(BaseTypes_ParameterOverwriteMsg, header) < 65536 && pb_membersize(BaseTypes_ParameterCmdMsg, header) < 65536 && pb_membersize(BaseTypes_ParameterCmdListMsg, header) < 65536 && pb_membersize(BaseTypes_ParameterCmdListMsg, cmds[0]) < 65536 && pb_membersize(BaseTypes_StatusMsg, header) < 65536 && pb_membersize(BaseTypes_StatusMsg, info) < 65536 && pb_membersize(BaseTypes_ConsoleCmdMsg, header) < 65536 && pb_membersize(BaseTypes_ConsoleCmdListMsg, header) < 65536 && pb_membersize(BaseTypes_ConsoleCmdListMsg, cmds[0]) < 65536 && pb_membersize(BaseTypes_LogMsg, header) < 65536 && pb_membersize(BaseTypes_LogListMsg, header) < 65536 && pb_membersize(BaseTypes_LogListMsg, logs[0]) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_BaseTypes_Point_BaseTypes_Euler_BaseTypes_Joint_BaseTypes_RegMap_BaseTypes_PoseEuler_BaseTypes_PRShmi_BaseTypes_SRShmi_BaseTypes_RShmi_BaseTypes_MRShmi_BaseTypes_TFShmi_BaseTypes_UFShmi_BaseTypes_ParamInfo_BaseTypes_Header_BaseTypes_ParameterMsg_BaseTypes_ParameterListMsg_BaseTypes_ParameterGetMsg_BaseTypes_ParameterGetListMsg_BaseTypes_ParameterSetMsg_BaseTypes_ParameterSetListMsg_BaseTypes_ParameterOverwriteMsg_BaseTypes_ParameterCmdMsg_BaseTypes_ParameterCmdListMsg_BaseTypes_StatusMsg_BaseTypes_ConsoleCmdMsg_BaseTypes_ConsoleCmdListMsg_BaseTypes_LogMsg_BaseTypes_LogListMsg_BaseTypes_CreatePlotMsg_BaseTypes_RemovePlotMsg_BaseTypes_CommonString)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
#error Field descriptor for BaseTypes_ParameterMsg.param is too large. Define PB_FIELD_16BIT to fix this.
#endif


/* On some platforms (such as AVR), double is really float.
 * These are not directly supported by nanopb, but see example_avr_double.
 * To get rid of this error, remove any double fields from your .proto.
 */
PB_STATIC_ASSERT(sizeof(double) == 8, DOUBLE_MUST_BE_8_BYTES)

/* @@protoc_insertion_point(eof) */