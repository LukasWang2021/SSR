/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.7-dev at Tue Aug 22 09:06:29 2017. */

#include "motionSL.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t motion_spec_Pose_fields[2] = {
    PB_FIELD(  1, DOUBLE  , REPEATED, STATIC  , FIRST, motion_spec_Pose, coordinates, coordinates, 0),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_ToolTipOffset_fields[7] = {
    PB_FIELD(  1, DOUBLE  , REQUIRED, STATIC  , FIRST, motion_spec_ToolTipOffset, offsetX, offsetX, 0),
    PB_FIELD(  2, DOUBLE  , REQUIRED, STATIC  , OTHER, motion_spec_ToolTipOffset, offsetY, offsetX, 0),
    PB_FIELD(  3, DOUBLE  , REQUIRED, STATIC  , OTHER, motion_spec_ToolTipOffset, offsetZ, offsetY, 0),
    PB_FIELD(  4, DOUBLE  , REQUIRED, STATIC  , OTHER, motion_spec_ToolTipOffset, offsetA, offsetZ, 0),
    PB_FIELD(  5, DOUBLE  , REQUIRED, STATIC  , OTHER, motion_spec_ToolTipOffset, offsetB, offsetA, 0),
    PB_FIELD(  6, DOUBLE  , REQUIRED, STATIC  , OTHER, motion_spec_ToolTipOffset, offsetC, offsetB, 0),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_WayPoint_fields[7] = {
    PB_FIELD(  1, MESSAGE , REQUIRED, STATIC  , FIRST, motion_spec_WayPoint, pose, pose, &motion_spec_Pose_fields),
    PB_FIELD(  2, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_WayPoint, blendingAcc, pose, 0),
    PB_FIELD(  3, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_WayPoint, blendInDistance, blendingAcc, 0),
    PB_FIELD(  4, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_WayPoint, blendTime, blendInDistance, 0),
    PB_FIELD(  5, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_WayPoint, segmentVelocity, blendTime, 0),
    PB_FIELD(  6, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_WayPoint, smoothPercent, segmentVelocity, 0),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_MoveL_fields[6] = {
    PB_FIELD(  1, DOUBLE  , REQUIRED, STATIC  , FIRST, motion_spec_MoveL, vMax, vMax, 0),
    PB_FIELD(  2, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_MoveL, aMax, vMax, 0),
    PB_FIELD(  3, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_MoveL, omegaMax, aMax, 0),
    PB_FIELD(  4, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_MoveL, alfaMax, omegaMax, 0),
    PB_FIELD(  5, MESSAGE , REPEATED, STATIC  , OTHER, motion_spec_MoveL, waypoints, alfaMax, &motion_spec_WayPoint_fields),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_MoveC_fields[7] = {
    PB_FIELD(  1, DOUBLE  , REQUIRED, STATIC  , FIRST, motion_spec_MoveC, vMax, vMax, 0),
    PB_FIELD(  2, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_MoveC, aMax, vMax, 0),
    PB_FIELD(  3, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_MoveC, jMax, aMax, 0),
    PB_FIELD(  4, MESSAGE , REQUIRED, STATIC  , OTHER, motion_spec_MoveC, pose1, jMax, &motion_spec_Pose_fields),
    PB_FIELD(  5, MESSAGE , REQUIRED, STATIC  , OTHER, motion_spec_MoveC, pose2, pose1, &motion_spec_Pose_fields),
    PB_FIELD(  6, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_MoveC, smoothPercent, pose2, 0),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_MoveJ_fields[7] = {
    PB_FIELD(  1, DOUBLE  , REPEATED, STATIC  , FIRST, motion_spec_MoveJ, targetJointCoordinates, targetJointCoordinates, 0),
    PB_FIELD(  2, DOUBLE  , REQUIRED, STATIC  , OTHER, motion_spec_MoveJ, vMax, targetJointCoordinates, 0),
    PB_FIELD(  3, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_MoveJ, aMax, vMax, 0),
    PB_FIELD(  4, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_MoveJ, jMax, aMax, 0),
    PB_FIELD(  5, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_MoveJ, smoothDistance, jMax, 0),
    PB_FIELD(  6, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_MoveJ, smoothPercent, smoothDistance, 0),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_TeachPose_fields[5] = {
    PB_FIELD(  1, MESSAGE , REQUIRED, STATIC  , FIRST, motion_spec_TeachPose, pose, pose, &motion_spec_Pose_fields),
    PB_FIELD(  2, BOOL    , OPTIONAL, STATIC  , OTHER, motion_spec_TeachPose, step, pose, 0),
    PB_FIELD(  3, BOOL    , OPTIONAL, STATIC  , OTHER, motion_spec_TeachPose, last, step, 0),
    PB_FIELD(  4, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_TeachPose, velocity, last, 0),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_userFrame_fields[7] = {
    PB_FIELD(  1, DOUBLE  , REQUIRED, STATIC  , FIRST, motion_spec_userFrame, X, X, 0),
    PB_FIELD(  2, DOUBLE  , REQUIRED, STATIC  , OTHER, motion_spec_userFrame, Y, X, 0),
    PB_FIELD(  3, DOUBLE  , REQUIRED, STATIC  , OTHER, motion_spec_userFrame, Z, Y, 0),
    PB_FIELD(  4, DOUBLE  , REQUIRED, STATIC  , OTHER, motion_spec_userFrame, A, Z, 0),
    PB_FIELD(  5, DOUBLE  , REQUIRED, STATIC  , OTHER, motion_spec_userFrame, B, A, 0),
    PB_FIELD(  6, DOUBLE  , REQUIRED, STATIC  , OTHER, motion_spec_userFrame, C, B, 0),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_toolFrame_fields[7] = {
    PB_FIELD(  1, DOUBLE  , REQUIRED, STATIC  , FIRST, motion_spec_toolFrame, X, X, 0),
    PB_FIELD(  2, DOUBLE  , REQUIRED, STATIC  , OTHER, motion_spec_toolFrame, Y, X, 0),
    PB_FIELD(  3, DOUBLE  , REQUIRED, STATIC  , OTHER, motion_spec_toolFrame, Z, Y, 0),
    PB_FIELD(  4, DOUBLE  , REQUIRED, STATIC  , OTHER, motion_spec_toolFrame, A, Z, 0),
    PB_FIELD(  5, DOUBLE  , REQUIRED, STATIC  , OTHER, motion_spec_toolFrame, B, A, 0),
    PB_FIELD(  6, DOUBLE  , REQUIRED, STATIC  , OTHER, motion_spec_toolFrame, C, B, 0),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_Set_fields[4] = {
    PB_FIELD(  1, STRING  , REQUIRED, STATIC  , FIRST, motion_spec_Set, path, path, 0),
    PB_FIELD(  2, BOOL    , REQUIRED, STATIC  , OTHER, motion_spec_Set, value, path, 0),
    PB_FIELD(  3, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_Set, time_sec, value, 0),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_Get_fields[2] = {
    PB_FIELD(  1, STRING  , REQUIRED, CALLBACK, FIRST, motion_spec_Get, path, path, 0),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_Wait_fields[5] = {
    PB_FIELD(  1, STRING  , OPTIONAL, STATIC  , FIRST, motion_spec_Wait, path, path, 0),
    PB_FIELD(  2, BOOL    , OPTIONAL, STATIC  , OTHER, motion_spec_Wait, value, path, 0),
    PB_FIELD(  3, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_Wait, timeout, value, 0),
    PB_FIELD(  4, UINT32  , OPTIONAL, STATIC  , OTHER, motion_spec_Wait, error_code, timeout, 0),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_MotionCommand_fields[4] = {
    PB_FIELD(  1, UINT32  , REQUIRED, STATIC  , FIRST, motion_spec_MotionCommand, id, id, 0),
    PB_FIELD(  2, UENUM   , REQUIRED, STATIC  , OTHER, motion_spec_MotionCommand, commandtype, id, 0),
    PB_FIELD(  3, BYTES   , REQUIRED, STATIC  , OTHER, motion_spec_MotionCommand, commandarguments, commandtype, 0),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_MotionProgram_fields[4] = {
    PB_FIELD(  1, STRING  , REQUIRED, STATIC  , FIRST, motion_spec_MotionProgram, name, name, 0),
    PB_FIELD(  2, UINT32  , REQUIRED, STATIC  , OTHER, motion_spec_MotionProgram, id, name, 0),
    PB_FIELD(  3, MESSAGE , REPEATED, STATIC  , OTHER, motion_spec_MotionProgram, commandlist, id, &motion_spec_MotionCommand_fields),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_DeviceList_fields[2] = {
    PB_FIELD(  1, MESSAGE , REPEATED, STATIC  , FIRST, motion_spec_DeviceList, dev_info, dev_info, &motion_spec_DeviceInfo_fields),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_DeviceInfo_fields[6] = {
    PB_FIELD(  1, STRING  , REQUIRED, STATIC  , FIRST, motion_spec_DeviceInfo, communication_type, communication_type, 0),
    PB_FIELD(  2, UINT32  , REQUIRED, STATIC  , OTHER, motion_spec_DeviceInfo, device_number, communication_type, 0),
    PB_FIELD(  3, UENUM   , REQUIRED, STATIC  , OTHER, motion_spec_DeviceInfo, device_type, device_number, 0),
    PB_FIELD(  4, UINT32  , REQUIRED, STATIC  , OTHER, motion_spec_DeviceInfo, input, device_type, 0),
    PB_FIELD(  5, UINT32  , REQUIRED, STATIC  , OTHER, motion_spec_DeviceInfo, output, input, 0),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_JointLimit_fields[6] = {
    PB_FIELD(  1, DOUBLE  , OPTIONAL, STATIC  , FIRST, motion_spec_JointLimit, zero, zero, 0),
    PB_FIELD(  2, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_JointLimit, upper, zero, 0),
    PB_FIELD(  3, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_JointLimit, lower, upper, 0),
    PB_FIELD(  4, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_JointLimit, max_omega, lower, 0),
    PB_FIELD(  5, DOUBLE  , OPTIONAL, STATIC  , OTHER, motion_spec_JointLimit, max_alpha, max_omega, 0),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_JointConstraint_fields[2] = {
    PB_FIELD(  1, MESSAGE , REPEATED, STATIC  , FIRST, motion_spec_JointConstraint, jnt_lmt, jnt_lmt, &motion_spec_JointLimit_fields),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_CoordinateOffset_fields[5] = {
    PB_FIELD(  1, DOUBLE  , REQUIRED, STATIC  , FIRST, motion_spec_CoordinateOffset, alpha, alpha, 0),
    PB_FIELD(  2, DOUBLE  , REQUIRED, STATIC  , OTHER, motion_spec_CoordinateOffset, a, alpha, 0),
    PB_FIELD(  3, DOUBLE  , REQUIRED, STATIC  , OTHER, motion_spec_CoordinateOffset, d, a, 0),
    PB_FIELD(  4, DOUBLE  , REQUIRED, STATIC  , OTHER, motion_spec_CoordinateOffset, theta, d, 0),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_DHGroup_fields[2] = {
    PB_FIELD(  1, MESSAGE , REPEATED, STATIC  , FIRST, motion_spec_DHGroup, coord_offset, coord_offset, &motion_spec_CoordinateOffset_fields),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_Signal_fields[3] = {
    PB_FIELD(  1, UINT32  , REQUIRED, STATIC  , FIRST, motion_spec_Signal, id, id, 0),
    PB_FIELD(  2, BYTES   , REQUIRED, STATIC  , OTHER, motion_spec_Signal, param, id, 0),
    PB_LAST_FIELD
};

const pb_field_t motion_spec_SignalGroup_fields[2] = {
    PB_FIELD(  1, MESSAGE , REPEATED, STATIC  , FIRST, motion_spec_SignalGroup, sig_param, sig_param, &motion_spec_Signal_fields),
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
PB_STATIC_ASSERT((pb_membersize(motion_spec_WayPoint, pose) < 65536 && pb_membersize(motion_spec_MoveL, waypoints[0]) < 65536 && pb_membersize(motion_spec_MoveC, pose1) < 65536 && pb_membersize(motion_spec_MoveC, pose2) < 65536 && pb_membersize(motion_spec_TeachPose, pose) < 65536 && pb_membersize(motion_spec_MotionProgram, commandlist[0]) < 65536 && pb_membersize(motion_spec_DeviceList, dev_info[0]) < 65536 && pb_membersize(motion_spec_JointConstraint, jnt_lmt[0]) < 65536 && pb_membersize(motion_spec_DHGroup, coord_offset[0]) < 65536 && pb_membersize(motion_spec_SignalGroup, sig_param[0]) < 65536), YOU_MUST_DEFINE_PB_FIELD_32BIT_FOR_MESSAGES_motion_spec_Pose_motion_spec_ToolTipOffset_motion_spec_WayPoint_motion_spec_MoveL_motion_spec_MoveC_motion_spec_MoveJ_motion_spec_TeachPose_motion_spec_userFrame_motion_spec_toolFrame_motion_spec_Set_motion_spec_Get_motion_spec_Wait_motion_spec_MotionCommand_motion_spec_MotionProgram_motion_spec_DeviceList_motion_spec_DeviceInfo_motion_spec_JointLimit_motion_spec_JointConstraint_motion_spec_CoordinateOffset_motion_spec_DHGroup_motion_spec_Signal_motion_spec_SignalGroup)
#endif

#if !defined(PB_FIELD_16BIT) && !defined(PB_FIELD_32BIT)
#error Field descriptor for motion_spec_Signal.param is too large. Define PB_FIELD_16BIT to fix this.
#endif


/* On some platforms (such as AVR), double is really float.
 * These are not directly supported by nanopb, but see example_avr_double.
 * To get rid of this error, remove any double fields from your .proto.
 */
PB_STATIC_ASSERT(sizeof(double) == 8, DOUBLE_MUST_BE_8_BYTES)

/* @@protoc_insertion_point(eof) */
