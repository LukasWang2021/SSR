/*************************************************************************
	> File Name: arm_kinematics.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年08月28日 星期二 13时29分43秒
 ************************************************************************/

#include <math.h>
#include <string.h>
#include <arm_kinematics.h>

using namespace basic_alg;



namespace fst_mc
{

void inline reviseJoint(double &jnt, double ref, double upper_limit, double lower_limit)
{
    jnt = jnt - round((jnt - ref) / 2 / PI) * PI * 2;
    if      (jnt > upper_limit)  jnt = jnt - ceil((jnt - upper_limit) / 2 / PI) * PI * 2;
    else if (jnt < lower_limit)  jnt = jnt - ceil((jnt - lower_limit) / 2 / PI) * PI * 2;
}


void ArmKinematics::initKinematics(double (&dh_matrix)[NUM_OF_JOINT][4])
{
    for (size_t i = 0; i < JOINT_OF_ARM; i++)
    {
        dh_matrix_[i][0] = dh_matrix[i][0];
        dh_matrix_[i][1] = dh_matrix[i][1];
        dh_matrix_[i][2] = dh_matrix[i][2];
        dh_matrix_[i][3] = dh_matrix[i][3];
    }
}


void ArmKinematics::forwardKinematicsInBase(const Joint &joint, Pose &pose)
{
    Matrix matrix;
    forwardKinematics(joint, matrix);
    matrix.rightMultiply(tool_frame_).toPose(pose);
}


void ArmKinematics::forwardKinematicsInBase(const Joint &joint, PoseEuler &pose)
{
    Matrix matrix;
    forwardKinematics(joint, matrix);
    matrix.rightMultiply(tool_frame_).toPoseEuler(pose);
}


void ArmKinematics::forwardKinematicsInUser(const Joint &joint, Pose &pose)
{
    Matrix matrix;
    forwardKinematics(joint, matrix);
    matrix.leftMultiply(inverse_user_frame_).rightMultiply(tool_frame_).toPose(pose);
}


void ArmKinematics::forwardKinematicsInUser(const Joint &joint, PoseEuler &pose)
{
    Matrix matrix;
    forwardKinematics(joint, matrix);
    matrix.leftMultiply(inverse_user_frame_).rightMultiply(tool_frame_).toPoseEuler(pose);
}


void ArmKinematics::forwardKinematicsInWorld(const Joint &joint, Pose &pose)
{
    Matrix matrix;
    forwardKinematics(joint, matrix);
    matrix.leftMultiply(inverse_world_frame_).rightMultiply(tool_frame_).toPose(pose);
}


void ArmKinematics::forwardKinematicsInWorld(const Joint &joint, PoseEuler &pose)
{
    Matrix matrix;
    forwardKinematics(joint, matrix);
    matrix.leftMultiply(inverse_world_frame_).rightMultiply(tool_frame_).toPoseEuler(pose);
}


void ArmKinematics::forwardKinematics(const Joint &joint, Matrix &matrix)
{
    matrix.eye();
    matrix.transFromDH(dh_matrix_[0], joint.j1);
    matrix.transFromDH(dh_matrix_[1], joint.j2);
    matrix.transFromDH(dh_matrix_[2], joint.j3);
    matrix.transFromDH(dh_matrix_[3], joint.j4);
    matrix.transFromDH(dh_matrix_[4], joint.j5);
    matrix.transFromDH(dh_matrix_[5], joint.j6);
}







ErrorCode ArmKinematics::inverseKinematicsInBase(const Pose &pose, const Joint &ref, Joint &res)
{
    Matrix matrix = Matrix(pose).rightMultiply(inverse_tool_frame_);
    return inverseKinematics(matrix, ref, res);
}


ErrorCode ArmKinematics::inverseKinematicsInBase(const PoseEuler &pose, const Joint &ref, Joint &res)
{
    Matrix matrix = Matrix(pose).rightMultiply(inverse_tool_frame_);
    return inverseKinematics(matrix, ref, res);
}


ErrorCode ArmKinematics::inverseKinematics(const Matrix &matrix, const Joint &ref, Joint &res)
{
    Matrix wrist(matrix);
    wrist.rightMultiply(Matrix(0, 0, -dh_matrix_[5][2]));

    double &nx = wrist[0][0], &ny = wrist[1][0], &nz = wrist[2][0];
    double &ox = wrist[0][1], &oy = wrist[1][1], &oz = wrist[2][1];
    double &ax = wrist[0][2], &ay = wrist[1][2], &az = wrist[2][2];
    double &px = wrist[0][3], &py = wrist[1][3], &pz = wrist[2][3];

    double j1, j2, j3, j4, j5, j6;
    double joint_ref;

    double mn1 = px * px + py * py;
    double mn2 = dh_matrix_[2][2] * dh_matrix_[2][2];

    if (mn2 > mn1)
    {
        //FST_ERROR("IK fail solving theta1.");
        //FST_ERROR("  px = %lf, py = %lf, DH[2][2] = %lf", px, py, dh_matrix_[2][2]);
        return IK_OUT_OF_WORKSPACE;
    }

    double t1_part1 = atan2(dh_matrix_[2][2], (mn1 - mn2) / 2);
    double t1_part2 = atan2(py, px);
    double t1_part3 = atan2(dh_matrix_[2][2], (mn2 - mn1) / 2);

    double angle1 = t1_part1 + t1_part2 - dh_matrix_[0][3];
    double angle2 = t1_part3 + t1_part2 - dh_matrix_[0][3];

    joint_ref   = ref.j1;

    reviseJoint(angle1, ref.j1, PI, -PI);
    reviseJoint(angle2, ref.j1, PI, -PI);

    if (fabs(angle1 - ref.j1) < fabs(angle2 - ref.j1))
    {
        res.j1 = angle1;
        j1 = t1_part1 + t1_part2;
    }
    else
    {
        res.j1 = angle2;
        j1 = t1_part3 + t1_part2;
    }

    double c1 = cos(j1);
    double s1 = sin(j1);

    double k1 = pz - dh_matrix_[0][2];
    double k2 = px * c1 + py * s1 - dh_matrix_[1][1];
    double mp1 = dh_matrix_[3][1] * dh_matrix_[3][1] + dh_matrix_[3][2] * dh_matrix_[3][2];
    double mp6 = k1 * k1 + k2 * k2;
    double k = (mp6 - dh_matrix_[2][1] * dh_matrix_[2][1] - mp1) / dh_matrix_[2][1] / 2;
    double mp2 = k * k;

    if (mp2 > mp1)
    {
        //FST_ERROR("IK fail solving j3.");
        //FST_ERROR("  j1 = %lf, px = %lf, py = %lf, pz = %lf", j1, px, py, pz);
        //FST_ERROR("  DH[0][2] = %lf, DH[1][1] = %lf, DH[3][1] = %lf, DH[3][2] = %lf, DH[2][1] = %lf",
        //          dh_matrix_[0][2], dh_matrix_[1][1], dh_matrix_[3][1], dh_matrix_[3][2], dh_matrix_[2][1]);
        //FST_ERROR("  k1 = %lf, k2=%lf, mp1 = %lf, mp6 = %lf, mp2 = %lf", k1, k2, mp1, mp6, mp2);
        return IK_OUT_OF_WORKSPACE;
    }

    double mp7 = sqrt(mp1 - mp2);
    double mp3 = atan2(k, mp7);
    double mp4 = atan2(dh_matrix_[3][1], dh_matrix_[3][2]);
    double mp5 = atan2(k, -mp7);

    angle1      = mp3 - mp4 - dh_matrix_[2][3];
    angle2      = mp5 - mp4 - dh_matrix_[2][3];
    joint_ref   = ref.j3;

    reviseJoint(angle1, ref.j3, PI, -PI);
    reviseJoint(angle2, ref.j3, PI, -PI);

    if (fabs(angle1 - ref.j3) < fabs(angle2 - ref.j3))
    {
        res.j3 = angle1;
        j3 = mp3 - mp4;
    }
    else
    {
        res.j3 = angle2;
        j3 = mp5 - mp4;
    }

    double c3 = cos(j3);
    double s3 = sin(j3);

    double b1 = dh_matrix_[3][1] + c3 * dh_matrix_[2][1];
    double b2 = dh_matrix_[3][2] + s3 * dh_matrix_[2][1];
    double s23 = (k1 * b1 + k2 * b2) / mp6;
    double c23 = (k2 * b1 - k1 * b2) / mp6;

    j2 = atan2(s23, c23) - j3;
    double c2 = cos(j2);
    double s2 = sin(j2);

    j2 -= dh_matrix_[1][3];
    joint_ref   = ref.j2;

    reviseJoint(j2, ref.j2, PI, -PI);
    res.j2 = j2;

    double c5 = c1 * s23 * ax + s1 * s23 * ay - c23 * az;

    if (c5 < (1 - MINIMUM_E6) && c5 > (MINIMUM_E6 - 1))
    {
        angle1 = atan2(s1 * ax - c1 * ay, c1 * c23 * ax + c23 * s1 * ay + s23 * az);
        angle2 = (angle1 > 0) ? (angle1 - PI) : (angle1 + PI);
    }
    else
    {
        angle1 = 0;
        angle2 = 0;
    }

    double tmp1 = angle1 - dh_matrix_[3][3];
    double tmp2 = angle2 - dh_matrix_[3][3];
    joint_ref   = ref.j4;

    reviseJoint(tmp1, ref.j4, PI, -PI);
    reviseJoint(tmp2, ref.j4, PI, -PI);

    j4 = fabs(tmp1 - ref.j4) < fabs(tmp2 - ref.j4) ? angle1 : angle2;

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

    if (fabs(s5) < MINIMUM_E6)
    {
        double j46_tmp = j4 + j6;
        double j46_ref = ref.j4 + dh_matrix_[3][3] + ref.j6 + dh_matrix_[5][3];

        j46_tmp -= round((j46_tmp - j46_ref) / 2 / PI) * PI * 2;

        j4 = (ref.j4 + dh_matrix_[3][3] - ref.j5 - dh_matrix_[5][3] + j46_tmp) / 2;
        j6 = j46_tmp - j4;
    }

    res.j4 = j4 - dh_matrix_[3][3];
    res.j5 = j5 - dh_matrix_[4][3];
    res.j6 = j6 - dh_matrix_[5][3];

    reviseJoint(res.j4, ref.j4, PI, -PI);
    reviseJoint(res.j5, ref.j5, PI, -PI);
    reviseJoint(res.j6, ref.j6, 2 * PI, -2 * PI);

    res.j7 = 0;
    res.j8 = 0;
    res.j9 = 0;

    return SUCCESS;
}


ErrorCode ArmKinematics::inverseKinematics(const Matrix &matrix, Joint (&solutions)[8], size_t length)
{
    Matrix wrist(matrix);
    wrist.rightMultiply(Matrix(0, 0, -dh_matrix_[5][2]));

    double &nx = wrist[0][0], &ny = wrist[1][0], &nz = wrist[2][0];
    double &ox = wrist[0][1], &oy = wrist[1][1], &oz = wrist[2][1];
    double &ax = wrist[0][2], &ay = wrist[1][2], &az = wrist[2][2];
    double &px = wrist[0][3], &py = wrist[1][3], &pz = wrist[2][3];

    double mn1 = px * px + py * py;
    double mn2 = dh_matrix_[2][2] * dh_matrix_[2][2];

    if (mn2 > mn1)
    {
        //FST_ERROR("IK fail solving theta1.");
        //FST_ERROR("  px = %lf, py = %lf, DH[2][2] = %lf", px, py, dh_matrix_[2][2]);
        return IK_OUT_OF_WORKSPACE;
    }

    double t1_part1 = atan2(dh_matrix_[2][2], (mn1 - mn2) / 2);
    double t1_part2 = atan2(py, px);
    double t1_part3 = atan2(dh_matrix_[2][2], (mn2 - mn1) / 2);

    double angle1 = t1_part1 + t1_part2 - dh_matrix_[0][3];
    double angle2 = t1_part3 + t1_part2 - dh_matrix_[0][3];
    //reviseJoint(angle1, 0, PI, -PI);
    //reviseJoint(angle2, 0, PI, -PI);

    return SUCCESS;
}

}

