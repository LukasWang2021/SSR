/* This module provide the interface to control the axis group.
 * In this case it is a robot arm in PUMA structure.
 */


#ifndef INTERPRETER_GROUP_H
#define INTERPRETER_GROUP_H

#include "motion_control.h"
#include "common_error_code.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int coord; /* the posture is in joint or cartesian coodinate*/
    // posture
    // [0] arm;    // 1: right arm, -1:left arm
    // [1] elbow;  // 1: elbow above wrist, -1:elbow below wrist
    // [2] wrist;  // 1: wrist down, -1: wrist up
    // [3] flip;   // 0: not flip wrist, 1: flip wrist
    int posture[4];
    // turn
    int turn[9];
    // pose
    double pos[9]; /* the posture value of j1~j9/x-y-z-a-b-c */
}PostureInfo;

typedef struct
{
    double vel;
    bool is_swift;
    int smooth_type;
    double smooth_value;
    double acc;
    int uf_id;
    int tf_id;
    PostureInfo aux; // auxiliar point for move circular
    PostureInfo tgt; // the target position
}MoveTrajInfo;

bool InterpGroup_Init(group_space::MotionControl **group_ptr);
/*move functions*/
ErrorCode InterpGroup_MoveJoint(int gid, MoveTrajInfo *traj);
ErrorCode InterpGroup_MoveLiner(int gid, MoveTrajInfo *traj);
ErrorCode InterpGroup_MoveCircl(int gid, MoveTrajInfo *traj);
/*parameters set for move functions*/
ErrorCode InterpGroup_SetOVC(int gid, double val);
ErrorCode InterpGroup_SetOAC(int gid, double val);
ErrorCode InterpGroup_SetPLD(int gid, int val);
ErrorCode InterpGroup_SetUF(int gid, int val);
ErrorCode InterpGroup_SetTF(int gid, int val);

#ifdef __cplusplus
}
#endif

#endif