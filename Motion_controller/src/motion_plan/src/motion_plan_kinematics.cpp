/*************************************************************************
	> File Name: motion_plan_kinematics.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年02月01日 星期四 09时25分48秒
 ************************************************************************/

#include <string.h>
#include <math.h>
#include <motion_plan_kinematics.h>
#include <motion_plan_variable.h>
#include <motion_plan_basic_function.h>
#include <motion_plan_reuse.h>

using namespace std;
using namespace fst_controller;

namespace fst_algorithm
{

void reviseJoint(double &jnt, double ref, double upper_limit, double lower_limit)
{
    jnt = jnt - round((jnt - ref) / 2 / PI) * PI * 2;
    if      (jnt > upper_limit)  jnt = jnt - ceil((jnt - upper_limit) / 2 / PI) * PI * 2;
    else if (jnt < lower_limit)  jnt = jnt - ceil((jnt - lower_limit) / 2 / PI) * PI * 2;
}

void forwardKinematics(const Joint &jnt, Pose &pose)
{
    Matrix mat;
    mat.identityMatrix();

    mat.transFromDH(g_dh_mat[0], jnt.j1);
    mat.transFromDH(g_dh_mat[1], jnt.j2);
    mat.transFromDH(g_dh_mat[2], jnt.j3);
    mat.transFromDH(g_dh_mat[3], jnt.j4);
    mat.transFromDH(g_dh_mat[4], jnt.j5);
    mat.transFromDH(g_dh_mat[5], jnt.j6);

    mat.leftMultiply(g_user_frame_inverse).rightMultiply(g_tool_frame).toPose(pose);
}

void forwardKinematics(const Joint &jnt, PoseEuler &pose)
{
    Matrix mat;
    mat.identityMatrix();

    mat.transFromDH(g_dh_mat[0], jnt.j1);
    mat.transFromDH(g_dh_mat[1], jnt.j2);
    mat.transFromDH(g_dh_mat[2], jnt.j3);
    mat.transFromDH(g_dh_mat[3], jnt.j4);
    mat.transFromDH(g_dh_mat[4], jnt.j5);
    mat.transFromDH(g_dh_mat[5], jnt.j6);

    mat.leftMultiply(g_user_frame_inverse).rightMultiply(g_tool_frame).toPoseEuler(pose);
}

/*
PoseEuler forwardKinematics(const Joint &jnt)
{
    Matrix mat;
    mat.identityMatrix();

    mat.transFromDH(g_dh_mat[0], jnt.j1);
    mat.transFromDH(g_dh_mat[1], jnt.j2);
    mat.transFromDH(g_dh_mat[2], jnt.j3);
    mat.transFromDH(g_dh_mat[3], jnt.j4);
    mat.transFromDH(g_dh_mat[4], jnt.j5);
    mat.transFromDH(g_dh_mat[5], jnt.j6);

    mat.leftMultiply(g_user_frame_inverse);
    mat.rightMultiply(g_tool_frame);
    
    return toPoseEuler();
}
*/

Pose forwardKinematics(const Joint &jnt)
{
    Matrix mat;
    mat.identityMatrix();

    mat.transFromDH(g_dh_mat[0], jnt.j1);
    mat.transFromDH(g_dh_mat[1], jnt.j2);
    mat.transFromDH(g_dh_mat[2], jnt.j3);
    mat.transFromDH(g_dh_mat[3], jnt.j4);
    mat.transFromDH(g_dh_mat[4], jnt.j5);
    mat.transFromDH(g_dh_mat[5], jnt.j6);

    return mat.leftMultiply(g_user_frame_inverse).rightMultiply(g_tool_frame).toPose();
}

ErrorCode inverseKinematics(const Pose &pose, const Joint &ref, Joint &res)
{
    Matrix wrist = Matrix(pose).leftMultiply(g_user_frame).rightMultiply(g_tool_frame_inverse)\
                    .rightMultiply(Matrix(0, 0, -g_dh_mat[5][2]));

    double &nx = wrist[0][0], &ny = wrist[1][0], &nz = wrist[2][0];
    double &ox = wrist[0][1], &oy = wrist[1][1], &oz = wrist[2][1];
    double &ax = wrist[0][2], &ay = wrist[1][2], &az = wrist[2][2];
    double &px = wrist[0][3], &py = wrist[1][3], &pz = wrist[2][3];

    double j1, j2, j3, j4, j5, j6;
    double joint_res, joint_ref;
    double upper_limit, lower_limit;

    double mn1 = px * px + py * py;
    double mn2 = g_dh_mat[2][2] * g_dh_mat[2][2];
   
    if (mn2 > mn1) {
        FST_ERROR("IK fail solving theta1.");
        FST_ERROR("  px = %lf, py = %lf, DH[2][2] = %lf", px, py, g_dh_mat[2][2]);
        return IK_OUT_OF_WORKSPACE;
    }

    double t1_part1 = atan2(g_dh_mat[2][2], (mn1 - mn2) / 2);
    double t1_part2 = atan2(py, px);
    double t1_part3 = atan2(g_dh_mat[2][2], (mn2 - mn1) / 2);

    double angle1 = t1_part1 + t1_part2 - g_dh_mat[0][3];
    double angle2 = t1_part3 + t1_part2 - g_dh_mat[0][3];

    
    joint_ref   = ref.j1;
    upper_limit = g_soft_constraint.j1.upper;
    lower_limit = g_soft_constraint.j1.lower;

    reviseJoint(angle1, ref.j1, upper_limit, lower_limit);
    reviseJoint(angle2, ref.j1, upper_limit, lower_limit);

    if (fabs(angle1 - ref.j1) < fabs(angle2 - ref.j1)) {
        res.j1 = angle1;
        j1 = t1_part1 + t1_part2;
    }
    else {
        res.j1 = angle2;
        j1 = t1_part3 + t1_part2;
    }

    double c1 = cos(j1);
    double s1 = sin(j1);
    
    double k1 = pz - g_dh_mat[0][2];
    double k2 = px * c1 + py * s1 - g_dh_mat[1][1];
    double mp1 = g_dh_mat[3][1] * g_dh_mat[3][1] + g_dh_mat[3][2] * g_dh_mat[3][2];
    double mp6 = k1 * k1 + k2 * k2;
    double k = (mp6 - g_dh_mat[2][1] * g_dh_mat[2][1] - mp1) / g_dh_mat[2][1] / 2;
    double mp2 = k * k;
    
    if (mp2 > mp1) {
        FST_ERROR("IK fail solving j3.");
        FST_ERROR("  j1 = %lf, px = %lf, py = %lf, pz = %lf", j1, px, py, pz);
        FST_ERROR("  DH[0][2] = %lf, DH[1][1] = %lf, DH[3][1] = %lf, DH[3][2] = %lf, DH[2][1] = %lf",
                    g_dh_mat[0][2], g_dh_mat[1][1], g_dh_mat[3][1], g_dh_mat[3][2], g_dh_mat[2][1]);
        FST_ERROR("  k1 = %lf, k2=%lf, mp1 = %lf, mp6 = %lf, mp2 = %lf", k1, k2, mp1, mp6, mp2);
        return IK_OUT_OF_WORKSPACE;
    }

    double mp7 = sqrt(mp1 - mp2);
    double mp3 = atan2(k, mp7);
    double mp4 = atan2(g_dh_mat[3][1], g_dh_mat[3][2]);
    double mp5 = atan2(k, -mp7);

    angle1      = mp3 - mp4 - g_dh_mat[2][3];
    angle2      = mp5 - mp4 - g_dh_mat[2][3];
    joint_ref   = ref.j3;
    upper_limit = g_soft_constraint.j3.upper;
    lower_limit = g_soft_constraint.j3.lower;
    
    reviseJoint(angle1, ref.j3, upper_limit, lower_limit);
    reviseJoint(angle2, ref.j3, upper_limit, lower_limit);

    if (fabs(angle1 - ref.j3) < fabs(angle2 - ref.j3)) {
        res.j3 = angle1;
        j3 = mp3 - mp4;
    }
    else {
        res.j3 = angle2;
        j3 = mp5 - mp4;
    }

    double c3 = cos(j3);
    double s3 = sin(j3);

    double b1 = g_dh_mat[3][1] + c3 * g_dh_mat[2][1];
    double b2 = g_dh_mat[3][2] + s3 * g_dh_mat[2][1];
    double s23 = (k1 * b1 + k2 * b2) / mp6;
    double c23 = (k2 * b1 - k1 * b2) / mp6;

    j2 = atan2(s23, c23) - j3;
    double c2 = cos(j2);
    double s2 = sin(j2);

    j2 -= g_dh_mat[1][3];
    joint_ref   = ref.j2;
    upper_limit = g_soft_constraint.j2.upper;
    lower_limit = g_soft_constraint.j2.lower;
    
    reviseJoint(j2, ref.j2, upper_limit, lower_limit);
    res.j2 = j2;

    double c5 = c1 * s23 * ax + s1 * s23 * ay - c23 * az;
    
    if (c5 < (1 - MINIMUM_E6) && c5 > (MINIMUM_E6 - 1)) {
        angle1 = atan2(s1 * ax - c1 * ay, c1 * c23 * ax + c23 * s1 * ay + s23 * az);
        angle2 = (angle1 > 0) ? (angle1 - PI) : (angle1 + PI);
    }
    else {
        angle1 = 0;
        angle2 = 0;
    }
    
    double tmp1 = angle1 - g_dh_mat[3][3];
    double tmp2 = angle2 - g_dh_mat[3][3];
    joint_ref   = ref.j4;
    upper_limit = g_soft_constraint.j4.upper;
    lower_limit = g_soft_constraint.j4.lower;
    
    reviseJoint(tmp1, ref.j4, upper_limit, lower_limit);
    reviseJoint(tmp2, ref.j4, upper_limit, lower_limit);

    if (fabs(tmp1 - ref.j4) < fabs(tmp2 - ref.j4)) {
        j4 = angle1;
    }
    else {
        j4 = angle2;
    }

    double c4 = cos(j4);
    double s4 = sin(j4);
    double s5 = (c1 * c23 * c4 + s1 * s4) * ax + (s1 * c2 * c3 * c4 - s1 * s2 * s3 * c4 - c1 * s4) * ay + s23 * c4 * az;

    j5 = atan2(s5, c5);

    double cf_x = (c4 * s1 - c1 * c23 * s4);
    double cf_y = c1 * c4 + c23 * s1 * s4;
    double cf_z = s23 * s4;
                    
    double s6 = cf_x * nx - cf_y * ny - cf_z * nz;
    double c6 = cf_x * ox - cf_y * oy - cf_z * oz;
    
    j6 = atan2(s6, c6);

    if (fabs(s5) < MINIMUM_E6) {
        double j46_tmp = j4 + j6;
        double j46_ref = ref.j4 + g_dh_mat[3][3] + ref.j6 + g_dh_mat[5][3];
        
        j46_tmp -= round((j46_tmp - j46_ref) / 2 / PI) * PI * 2;
        
        j4 = (ref.j4 + g_dh_mat[3][3] - ref.j5 - g_dh_mat[5][3] + j46_tmp) / 2;
        j6 = j46_tmp - j4;
    }

    res.j4 = j4 - g_dh_mat[3][3];
    res.j5 = j5 - g_dh_mat[4][3];
    res.j6 = j6 - g_dh_mat[5][3];

    reviseJoint(res.j4, ref.j4, g_soft_constraint.j4.upper, g_soft_constraint.j4.lower);
    reviseJoint(res.j5, ref.j5, g_soft_constraint.j5.upper, g_soft_constraint.j5.lower);
    reviseJoint(res.j6, ref.j6, g_soft_constraint.j6.upper, g_soft_constraint.j6.lower);

    res.j7 = 0;
    res.j8 = 0;
    res.j9 = 0;
    
    return SUCCESS;
}

ErrorCode inverseKinematics(const PoseEuler &pose, const Joint &ref, Joint &res)
{
    return inverseKinematics(PoseEuler2Pose(pose), ref, res);
}

ErrorCode inverseKinematics(const Pose &pose, const Angle *ref, Angle *res)
{
    return inverseKinematics(pose, *((Joint*)ref), *((Joint*)res));
}

ErrorCode chainIK(const fst_controller::Pose &pose, fst_controller::Joint &ref, fst_controller::Angle *res)
{
    ErrorCode err = inverseKinematics(pose, ref, *((Joint*)res));
    
    if (err == SUCCESS) {
        memcpy(&ref, res, NUM_OF_JOINT * sizeof(Angle));
    }

    return err;
}

}

