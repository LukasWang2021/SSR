#include "segment_alg.h"
#include <iostream>

using namespace std;
using namespace fst_mc;

ComplexAxisGroupModel model;
double stack[12000];
SegmentAlgParam segment_alg_param;


inline double getVector3Norm(double* vector)
{
    return sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
}

inline double getVector4Norm(double* vector)
{
    return sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2] + vector[3]*vector[3]);
}

void updateTransMatrix44(double* rot_vector, double* trans_vector)
{
    stack[S_TmpDouble_1] = sin(rot_vector[0]);  // sin(A)
    stack[S_TmpDouble_2] = cos(rot_vector[0]);  // cos(A)
    stack[S_TmpDouble_3] = sin(rot_vector[1]);  // sin(B)
    stack[S_TmpDouble_4] = cos(rot_vector[1]);  // cos(B)
    stack[S_TmpDouble_5] = sin(rot_vector[2]);  // sin(C)
    stack[S_TmpDouble_6] = cos(rot_vector[2]);  // cos(C)

    stack[S_TransMatrix] = stack[S_TmpDouble_4]*stack[S_TmpDouble_6];
    stack[S_TransMatrix + 1] = stack[S_TmpDouble_6]*stack[S_TmpDouble_1]*stack[S_TmpDouble_3]-stack[S_TmpDouble_2]*stack[S_TmpDouble_5];
    stack[S_TransMatrix + 2] = stack[S_TmpDouble_2]*stack[S_TmpDouble_6]*stack[S_TmpDouble_3]+stack[S_TmpDouble_1]*stack[S_TmpDouble_5];
    stack[S_TransMatrix + 3] = trans_vector[0];
    stack[S_TransMatrix + 4] = stack[S_TmpDouble_4]*stack[S_TmpDouble_5];
    stack[S_TransMatrix + 5] = stack[S_TmpDouble_2]*stack[S_TmpDouble_6]+stack[S_TmpDouble_1]*stack[S_TmpDouble_3]*stack[S_TmpDouble_5];
    stack[S_TransMatrix + 6] = -stack[S_TmpDouble_6]*stack[S_TmpDouble_1]+stack[S_TmpDouble_2]*stack[S_TmpDouble_3]*stack[S_TmpDouble_5];
    stack[S_TransMatrix + 7] = trans_vector[1];
    stack[S_TransMatrix + 8] = -stack[S_TmpDouble_3];
    stack[S_TransMatrix + 9] = stack[S_TmpDouble_4]*stack[S_TmpDouble_1];
    stack[S_TransMatrix + 10] = stack[S_TmpDouble_2]*stack[S_TmpDouble_4];
    stack[S_TransMatrix + 11] = trans_vector[2];
}

void updateHomoTransMatrix44(int target_joint_index, double target_joint_q)
{
    // standard dh
    //                T =    [    ct  -st*ca  st*sa   L.a*ct
    //                            st  ct*ca   -ct*sa  L.a*st
    //                            0   sa      ca      d
    //                            0   0       0       1];

    stack[S_TmpDouble_1] = stack[S_RealTheta + target_joint_index] + target_joint_q;        // compute real q
    stack[S_TmpDouble_2] = sin(stack[S_TmpDouble_1]);   // sin(theta)
    stack[S_TmpDouble_3] = cos(stack[S_TmpDouble_1]);   // cos(theta)
    stack[S_TmpDouble_4] = sin(stack[S_RealAlpha + target_joint_index]); // sin(alpha)
    stack[S_TmpDouble_5] = cos(stack[S_RealAlpha + target_joint_index]); // cos(alpha)
    stack[S_TmpDouble_6] = stack[S_RealA + target_joint_index];     // a

    stack[S_HomoTransMatrix] = stack[S_TmpDouble_3];
    stack[S_HomoTransMatrix + 1] = -stack[S_TmpDouble_2] * stack[S_TmpDouble_5];
    stack[S_HomoTransMatrix + 2] = stack[S_TmpDouble_2] * stack[S_TmpDouble_4];
    stack[S_HomoTransMatrix + 3] = stack[S_TmpDouble_6] * stack[S_TmpDouble_3];
    stack[S_HomoTransMatrix + 4] = stack[S_TmpDouble_2];
    stack[S_HomoTransMatrix + 5] = stack[S_TmpDouble_3] * stack[S_TmpDouble_5];
    stack[S_HomoTransMatrix + 6] = -stack[S_TmpDouble_3] * stack[S_TmpDouble_4];
    stack[S_HomoTransMatrix + 7] = stack[S_TmpDouble_6] * stack[S_TmpDouble_2];
    stack[S_HomoTransMatrix + 9] = stack[S_TmpDouble_4];
    stack[S_HomoTransMatrix + 10] = stack[S_TmpDouble_5];
    stack[S_HomoTransMatrix + 11] = stack[S_RealD + target_joint_index];
}

void getHomoTransMatrix44(int target_joint_index, double target_joint_q, double* r)
{
    // standard dh
    //                T =    [    ct  -st*ca  st*sa   L.a*ct
    //                            st  ct*ca   -ct*sa  L.a*st
    //                            0   sa      ca      d
    //                            0   0       0       1];

    stack[S_TmpDouble_1] = stack[S_RealTheta + target_joint_index] + target_joint_q;        // compute real q
    stack[S_TmpDouble_2] = sin(stack[S_TmpDouble_1]);   // sin(theta)
    stack[S_TmpDouble_3] = cos(stack[S_TmpDouble_1]);   // cos(theta)
    stack[S_TmpDouble_4] = sin(stack[S_RealAlpha + target_joint_index]); // sin(alpha)
    stack[S_TmpDouble_5] = cos(stack[S_RealAlpha + target_joint_index]); // cos(alpha)
    stack[S_TmpDouble_6] = stack[S_RealA + target_joint_index];     // a

    r[0] = stack[S_TmpDouble_3];
    r[1] = -stack[S_TmpDouble_2] * stack[S_TmpDouble_5];
    r[2] = stack[S_TmpDouble_2] * stack[S_TmpDouble_4];
    r[3] = stack[S_TmpDouble_6] * stack[S_TmpDouble_3];
    r[4] = stack[S_TmpDouble_2];
    r[5] = stack[S_TmpDouble_3] * stack[S_TmpDouble_5];
    r[6] = -stack[S_TmpDouble_3] * stack[S_TmpDouble_4];
    r[7] = stack[S_TmpDouble_6] * stack[S_TmpDouble_2];
    r[9] = stack[S_TmpDouble_4];
    r[10] = stack[S_TmpDouble_5];
    r[11] = stack[S_RealD + target_joint_index];
}


void getMatrix44MultiMatrix44(double* a, double* b, double* r)
{
    r[0] = a[0]*b[0] + a[1]*b[4] + a[2]*b[8] + a[3]*b[12];
    r[1] = a[0]*b[1] + a[1]*b[5] + a[2]*b[9] + a[3]*b[13];
    r[2] = a[0]*b[2] + a[1]*b[6] + a[2]*b[10] + a[3]*b[14];
    r[3] = a[0]*b[3] + a[1]*b[7] + a[2]*b[11] + a[3]*b[15];

    r[4] = a[4]*b[0] + a[5]*b[4] + a[6]*b[8] + a[7]*b[12];
    r[5] = a[4]*b[1] + a[5]*b[5] + a[6]*b[9] + a[7]*b[13];
    r[6] = a[4]*b[2] + a[5]*b[6] + a[6]*b[10] + a[7]*b[14];
    r[7] = a[4]*b[3] + a[5]*b[7] + a[6]*b[11] + a[7]*b[15];   

    r[8] = a[8]*b[0] + a[9]*b[4] + a[10]*b[8] + a[11]*b[12];
    r[9] = a[8]*b[1] + a[9]*b[5] + a[10]*b[9] + a[11]*b[13];
    r[10] = a[8]*b[2] + a[9]*b[6] + a[10]*b[10] + a[11]*b[14];
    r[11] = a[8]*b[3] + a[9]*b[7] + a[10]*b[11] + a[11]*b[15];  

    r[12] = a[12]*b[0] + a[13]*b[4] + a[14]*b[8] + a[15]*b[12];
    r[13] = a[12]*b[1] + a[13]*b[5] + a[14]*b[9] + a[15]*b[13];
    r[14] = a[12]*b[2] + a[13]*b[6] + a[14]*b[10] + a[15]*b[14];
    r[15] = a[12]*b[3] + a[13]*b[7] + a[14]*b[11] + a[15]*b[15];       
}

void getVector3CrossProduct(double* a, double* b, double* r)
{
    r[0] = a[1]*b[2] - a[2]*b[1];
    r[1] = a[2]*b[0] - a[0]*b[2];
    r[2] = a[0]*b[1] - a[1]*b[0];
}


void getMatrix33Transpose(double* matrix, double* matrix_t)
{
    matrix_t[0] = matrix[0]; matrix_t[1] = matrix[3]; matrix_t[2] = matrix[6];
    matrix_t[3] = matrix[1]; matrix_t[4] = matrix[4]; matrix_t[5] = matrix[7];
    matrix_t[6] = matrix[2]; matrix_t[7] = matrix[5]; matrix_t[8] = matrix[8];
}

void getMatrix33MultiVector3(double* matrix, double* vector, double* vector_r)
{
    vector_r[0] = matrix[0]*vector[0] + matrix[1]*vector[1] + matrix[2]*vector[2];
    vector_r[1] = matrix[3]*vector[0] + matrix[4]*vector[1] + matrix[5]*vector[2];
    vector_r[2] = matrix[6]*vector[0] + matrix[7]*vector[1] + matrix[8]*vector[2];
}

void getMatrix66MultiVector6(double* matrix, double* vector, double* vector_r)
{
    vector_r[0] = matrix[0]*vector[0] + matrix[1]*vector[1] + matrix[2]*vector[2] + matrix[3]*vector[3] + matrix[4]*vector[4] + matrix[5]*vector[5];
    vector_r[1] = matrix[6]*vector[0] + matrix[7]*vector[1] + matrix[8]*vector[2] + matrix[9]*vector[3] + matrix[10]*vector[4] + matrix[11]*vector[5];
    vector_r[2] = matrix[12]*vector[0] + matrix[13]*vector[1] + matrix[14]*vector[2] + matrix[15]*vector[3] + matrix[16]*vector[4] + matrix[17]*vector[5];
    vector_r[3] = matrix[18]*vector[0] + matrix[19]*vector[1] + matrix[20]*vector[2] + matrix[21]*vector[3] + matrix[22]*vector[4] + matrix[23]*vector[5];
    vector_r[4] = matrix[24]*vector[0] + matrix[25]*vector[1] + matrix[26]*vector[2] + matrix[27]*vector[3] + matrix[28]*vector[4] + matrix[29]*vector[5];
    vector_r[5] = matrix[30]*vector[0] + matrix[31]*vector[1] + matrix[32]*vector[2] + matrix[33]*vector[3] + matrix[34]*vector[4] + matrix[35]*vector[5];
}



void getRotationMatrix33FromHomoTransMatrix44(double* homo_trans, double* rotation)
{
    rotation[0] = homo_trans[0];
    rotation[1] = homo_trans[1];
    rotation[2] = homo_trans[2];
    rotation[3] = homo_trans[4];
    rotation[4] = homo_trans[5];
    rotation[5] = homo_trans[6];
    rotation[6] = homo_trans[8];
    rotation[7] = homo_trans[9];
    rotation[8] = homo_trans[10];
}

double getBaseFunction(int i, int k, double u)
{
    int base_index = S_BSplineNodeVector + i;
    if(k == 0)
    {
        if(u >= stack[base_index]
            && u <= stack[base_index+1])
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        stack[S_TmpDouble_1] = stack[base_index+k] - stack[base_index];
        stack[S_TmpDouble_2] = stack[base_index+k+1] - stack[base_index+1];
        if(stack[S_TmpDouble_1] == 0)
        {
            stack[S_TmpDouble_1] = 1;
        }
        if(stack[S_TmpDouble_2] == 0)
        {
            stack[S_TmpDouble_2] = 1;
        }   
        return (u - stack[base_index]) / stack[S_TmpDouble_1] * getBaseFunction(i, k-1, u)
               + (stack[base_index+k+1] - u) / stack[S_TmpDouble_2] * getBaseFunction(i+1, k-1, u);
    }
}

void updateTransitionBSpLineResult(int k, double* start_pos, double* mid_pos, double* end_pos, int result_count)
{
    //S_TmpDouble_1, S_TmpDouble_2 are used in getBaseFunction(), reserve them here
    stack[S_TmpDouble_3] = 1.0 / (result_count + 1);  // interpolation step
    stack[S_TmpDouble_4] = stack[S_TmpDouble_3];   // sum of interpolation step
    
    for(int i = 0; i < result_count; ++i)
    {
        stack[S_BaseFunctionNik0] = getBaseFunction(0, k, stack[S_TmpDouble_4]);
        stack[S_BaseFunctionNik1] = getBaseFunction(1, k, stack[S_TmpDouble_4]);
        stack[S_BaseFunctionNik2] = getBaseFunction(2, k, stack[S_TmpDouble_4]);

        stack[S_BSpLineResultXBase + i] = start_pos[0] * stack[S_BaseFunctionNik0]
                                        + mid_pos[0] * stack[S_BaseFunctionNik1]
                                        + end_pos[0] * stack[S_BaseFunctionNik2];
        stack[S_BSpLineResultYBase + i] = start_pos[1] * stack[S_BaseFunctionNik0]
                                        + mid_pos[1] * stack[S_BaseFunctionNik1]
                                        + end_pos[1] * stack[S_BaseFunctionNik2];
        stack[S_BSpLineResultZBase + i] = start_pos[2] * stack[S_BaseFunctionNik0]
                                        + mid_pos[2] * stack[S_BaseFunctionNik1]
                                        + end_pos[2] * stack[S_BaseFunctionNik2];
        stack[S_TmpDouble_4] += stack[S_TmpDouble_3];
    }
}

double getQuaternsIntersectionAngle(double* quatern1, double* quatern2)
{
    stack[S_TmpDouble_1] = quatern1[0] * quatern2[0] + quatern1[1] * quatern2[1] + quatern1[2] * quatern2[2] + quatern1[3] * quatern2[3];
    if(stack[S_TmpDouble_1] < 0)
    {
        quatern1[0] = -quatern1[0];
        quatern1[1] = -quatern1[1];
        quatern1[2] = -quatern1[2];
        quatern1[3] = -quatern1[3];
        if(stack[S_TmpDouble_1] < -1)
        {
            return (-PI / 2);
        }
    }

    if(stack[S_TmpDouble_1] > 1)
    {
        return (PI / 2);
    }

    return acos(stack[S_TmpDouble_1]);
}

void getEulerToRotationMatrix33(double* euler, double* rotation)
{
    stack[S_TmpDouble_1] = sin(euler[0]);  // sin(Z)
    stack[S_TmpDouble_2] = cos(euler[0]);  // cos(Z)
    stack[S_TmpDouble_3] = sin(euler[1]);  // sin(Y)
    stack[S_TmpDouble_4] = cos(euler[1]);  // cos(Y)
    stack[S_TmpDouble_5] = sin(euler[2]);  // sin(X)
    stack[S_TmpDouble_6] = cos(euler[2]);  // cos(X)

    rotation[0] = stack[S_TmpDouble_2]*stack[S_TmpDouble_4];
    rotation[1] = stack[S_TmpDouble_2]*stack[S_TmpDouble_3]*stack[S_TmpDouble_5]-stack[S_TmpDouble_6]*stack[S_TmpDouble_1];
    rotation[2] = stack[S_TmpDouble_2]*stack[S_TmpDouble_6]*stack[S_TmpDouble_3]+stack[S_TmpDouble_1]*stack[S_TmpDouble_5];
    rotation[3] = stack[S_TmpDouble_4]*stack[S_TmpDouble_1];
    rotation[4] = stack[S_TmpDouble_2]*stack[S_TmpDouble_6]+stack[S_TmpDouble_1]*stack[S_TmpDouble_3]*stack[S_TmpDouble_5];
    rotation[5] = stack[S_TmpDouble_6]*stack[S_TmpDouble_1]*stack[S_TmpDouble_3]-stack[S_TmpDouble_2]*stack[S_TmpDouble_5];
    rotation[6] = -stack[S_TmpDouble_3];
    rotation[7] = stack[S_TmpDouble_4]*stack[S_TmpDouble_5];
    rotation[8] = stack[S_TmpDouble_4]*stack[S_TmpDouble_6];   
}

void getRotationMatrix33ToEuler(double* rotation, double* euler)
{
    /*euler[1] = atan2(-rotation[6], sqrt(rotation[0]*rotation[0] + rotation[3]*rotation[3]));
    if(rotation[7]*rotation[7] + rotation[8]*rotation[8] > SQRT_DOUBLE_ACCURACY)
    {
        euler[0] = atan2(rotation[3], rotation[0]);
        euler[2] = atan2(rotation[7], rotation[8]);
    }
    else
    {
        euler[0] = atan2(-rotation[1], rotation[4]);
        euler[2] = 0;
    }*/
    euler[1] = atan2(-rotation[6], sqrt(rotation[7]*rotation[7] + rotation[8]*rotation[8]));
    if((rotation[7]*rotation[7] + rotation[8]*rotation[8]) > SQRT_DOUBLE_ACCURACY)
    {
        euler[0] = atan2(rotation[3], rotation[0]);
        euler[2] = atan2(rotation[7], rotation[8]);
    }
    else
    {
        euler[0] = atan2(-rotation[1], rotation[4]);
        euler[2] = 0;
    }    
}

void getQuaternToRotationMatrix33(double* quatern, double* rotation)
{
    // don't check if x^2+y^2+z^2+w^2 == 1, suppose the condition holds
    stack[S_TmpDouble_1] = 2 * quatern[0] * quatern[1]; // 2xy
    stack[S_TmpDouble_2] = 2 * quatern[1] * quatern[2]; // 2yz
    stack[S_TmpDouble_3] = 2 * quatern[2] * quatern[0]; // 2zx
    stack[S_TmpDouble_4] = 2 * quatern[3] * quatern[0]; // 2wx
    stack[S_TmpDouble_5] = 2 * quatern[3] * quatern[1]; // 2wy
    stack[S_TmpDouble_6] = 2 * quatern[3] * quatern[2]; // 2wz
    stack[S_TmpDouble_7] = quatern[3] * quatern[3]; // w^2

    rotation[0] = 2 * (stack[S_TmpDouble_7] + quatern[0] * quatern[0]) - 1; // r[0][0] = 2*(w^2+x^2)-1
    rotation[1] = stack[S_TmpDouble_1] - stack[S_TmpDouble_6];  // r[0][1] = 2*(xy-wz)
    rotation[2] = stack[S_TmpDouble_5] + stack[S_TmpDouble_3];  // r[0][2] = 2*(wy+zx)
    rotation[3] = stack[S_TmpDouble_1] + stack[S_TmpDouble_6];  // r[1][0] = 2*(xy+wz)-1
    rotation[4] = 2 * (stack[S_TmpDouble_7] + quatern[1] * quatern[1]) - 1; // r[1][1] = 2*(w^2+y^2)-1
    rotation[5] = stack[S_TmpDouble_2] - stack[S_TmpDouble_4];  // r[1][2] = 2*(yz-wx)
    rotation[6] = stack[S_TmpDouble_3] - stack[S_TmpDouble_5];  // r[2][0] = 2*(zx-wy)
    rotation[7] = stack[S_TmpDouble_4] + stack[S_TmpDouble_2];  // r[2][1] = 2*(wx+yz)
    rotation[8] = 2 * (stack[S_TmpDouble_7] + quatern[2] * quatern[2]) - 1; // r[2][2] = 2*(w^2+z^2)-1
}

void getRotationMatrix33ToQuatern(double* rotation, double* quatern)
{
    int max_id = 0;
    quatern[0] = sqrt(rotation[0] - rotation[4] - rotation[8] + 1) / 2; // x
    quatern[1] = sqrt(rotation[4] - rotation[0] - rotation[8] + 1) / 2; // y
    if(quatern[1] > quatern[0])
    {
        stack[S_TmpDouble_1] = quatern[1];  // store max value of {x,y,z,s}
        max_id = 1;
    }
    quatern[2] = sqrt(rotation[8] - rotation[0] - rotation[4] + 1) / 2; // z
    if(quatern[2] > stack[S_TmpDouble_1])
    {
        stack[S_TmpDouble_1] = quatern[2];
        max_id = 2;
    }
    quatern[3] = sqrt(rotation[0] + rotation[4] + rotation[8] + 1) / 2; // w
    if(quatern[3] > stack[S_TmpDouble_1])
    {
        max_id = 3;
    }

    switch(max_id)
    {
        case 0:
            if(rotation[3] + rotation[1] < 0) quatern[1] = -quatern[1];
            if(rotation[6] + rotation[2] < 0) quatern[2] = -quatern[2];
            if(rotation[7] - rotation[5] < 0) quatern[3] = -quatern[3];
            return;
        case 1:
            if(rotation[3] + rotation[1] < 0) quatern[0] = -quatern[0];
            if(rotation[7] + rotation[5] < 0) quatern[2] = -quatern[2];
            if(rotation[2] - rotation[6] < 0) quatern[3] = -quatern[3];
            return;
        case 2:
            if(rotation[6] + rotation[2] < 0) quatern[0] = -quatern[0];
            if(rotation[7] + rotation[5] < 0) quatern[1] = -quatern[1];
            if(rotation[3] - rotation[1] < 0) quatern[3] = -quatern[3];
            return;
        case 3:
            if(rotation[7] - rotation[5] < 0) quatern[0] = -quatern[0];
            if(rotation[2] - rotation[6] < 0) quatern[1] = -quatern[1];
            if(rotation[3] - rotation[1] < 0) quatern[2] = -quatern[2];
            return;
    }
}

void getEulerToQuatern(double* euler, double* quatern)
{
    getEulerToRotationMatrix33(euler, &stack[S_TmpMatrix33_1]);
    getRotationMatrix33ToQuatern(&stack[S_TmpMatrix33_1], quatern);
}

void getQuaternToEuler(double* quatern, double* euler)
{
    getQuaternToRotationMatrix33(quatern, &stack[S_TmpMatrix33_1]);
    getRotationMatrix33ToEuler(&stack[S_TmpMatrix33_1], euler);
}

void getQuaternVector4(double* start_quatern, double* end_quartern, double angle, double angle_distance_to_start, double* target_quatern)
{
    if(fabs(angle) > segment_alg_param.angle_valve)
    {
        // slerp interpolation
        stack[S_TmpDouble_1] = sin(angle);
        stack[S_TmpDouble_2] = sin((1 - angle_distance_to_start) * angle) / stack[S_TmpDouble_1];   // a(t)
        stack[S_TmpDouble_3] = sin(angle_distance_to_start * angle) / stack[S_TmpDouble_1]; // b(t)
        // target = a(t) * start + b(t) * end
        target_quatern[0] = stack[S_TmpDouble_2] * start_quatern[0] + stack[S_TmpDouble_3] * end_quartern[0];
        target_quatern[1] = stack[S_TmpDouble_2] * start_quatern[1] + stack[S_TmpDouble_3] * end_quartern[1];
        target_quatern[2] = stack[S_TmpDouble_2] * start_quatern[2] + stack[S_TmpDouble_3] * end_quartern[2];
        target_quatern[3] = stack[S_TmpDouble_2] * start_quatern[3] + stack[S_TmpDouble_3] * end_quartern[3];
    }
    else
    {
        stack[S_TmpDouble_1] = 1 - angle_distance_to_start;
        // target = (1-t) * start + t * end
        target_quatern[0] = stack[S_TmpDouble_1] * start_quatern[0] + angle_distance_to_start * end_quartern[0];
        target_quatern[1] = stack[S_TmpDouble_1] * start_quatern[1] + angle_distance_to_start * end_quartern[1];
        target_quatern[2] = stack[S_TmpDouble_1] * start_quatern[2] + angle_distance_to_start * end_quartern[2];
        target_quatern[3] = stack[S_TmpDouble_1] * start_quatern[3] + angle_distance_to_start * end_quartern[3];
    }
}


void getModelFk(ComplexAxisGroupModel* model_ptr, double* joints, double* pos_euler)
{
    getHomoTransMatrix44(0, joints[0], &stack[S_TmpMatrix44_1]);
    
    int offset = S_TmpMatrix44_1;
    for(int i = 1; i < model_ptr->link_num; ++i)
    {
        updateHomoTransMatrix44(i, joints[i]);
        getMatrix44MultiMatrix44(&stack[offset], &stack[S_HomoTransMatrix], &stack[offset + 16]);
        offset += 16;
    }
    
    // stack[offset] stroe the result
    pos_euler[0] = stack[offset + 3];   // x
    pos_euler[1] = stack[offset + 7];   // y
    pos_euler[2] = stack[offset + 11];  // z
    getRotationMatrix33FromHomoTransMatrix44(&stack[offset], &stack[S_TmpMatrix33_1]);
    getRotationMatrix33ToEuler(&stack[S_TmpMatrix33_1], &pos_euler[3]);
}


inline void doRowOperation(double* target_row, double* ref_row, double times, int row_size)
{
    for(int i = 0; i < row_size; ++i)
    {
        target_row[i] = target_row[i] - times * ref_row[i];
    }
}

inline void updateEquationSolution(double* matrix_a, double* matrix_b, int order)
{
    int l_index, diag_index, row_operation_size;
    int i, row, col, base_index;
    // AX=LUX=B, compute L & U
    for(col = 0; col < order; ++col)
    {
        diag_index = col*order+col;
        row_operation_size = order - col -1;
        for(row = col + 1; row < order; ++row)
        {
            l_index = row*order+col;
            matrix_a[l_index] = matrix_a[l_index] / matrix_a[diag_index];
            doRowOperation(&matrix_a[l_index+1], &matrix_a[diag_index+1], matrix_a[l_index], row_operation_size);
        }
    }

    // AX=LUX=B ==> make UX=C ==> LC=B, solve C
    for(row = 0; row < order; ++row)
    {
        stack[S_TmpDouble_1] = 0;
        for(i = 0; i < row; ++i)
        {
            stack[S_TmpDouble_1] += matrix_a[row*order + i] * stack[S_X + i];
        }
        stack[S_X + row] = matrix_b[row] - stack[S_TmpDouble_1];
    }

    // AX=LUX=B ==> make UX=C ==> LC=B, solve C ==> UX=C, solve X
    for(row = order - 1; row >= 0; --row)
    {
        stack[S_TmpDouble_1] = 0;
        base_index = row*order;
        for(i = row; i < order - 1; ++i)
        {
            stack[S_TmpDouble_1] += matrix_a[base_index+i+1] * stack[S_X+i+1];
        }
        stack[S_X + row] = (stack[S_X + row] - stack[S_TmpDouble_1]) / matrix_a[base_index+row];
    }
}

inline void updateMatrixA(double* interp_time, int order)
{
    int row, base_index0, base_index1, base_index2;

    stack[S_A] = 3 * interp_time[0] + 2 * interp_time[1] + interp_time[0] * interp_time[0] / interp_time[1];    // A[0][0]
    stack[S_A + 1] = interp_time[1];    // A[0][1]
    memset(&stack[S_A + 2], 0, sizeof(double) * (order - 2));   // A[0][2] ~ A[0][order-1]
    stack[S_A + order] = interp_time[1] - interp_time[0] * interp_time[0] / interp_time[1]; // A[1][0]
    stack[S_A + order + 1] = 2 * (interp_time[1] + interp_time[2]); // A[1][1]
    stack[S_A + order + 2] = interp_time[2];    // A[1][2]
    memset(&stack[S_A + order + 3], 0, sizeof(double) * (order - 3));   // A[1][3] ~ A[1][order-1]

    for(row = 2;  row < order - 2; ++row)   // A[2][0] ~ A[order - 3][order - 1]
    {
        base_index0 = S_A + row * order;
        memset(&stack[base_index0], 0, sizeof(double) * (row - 1));
        stack[base_index0 + row - 1] = interp_time[row];
        stack[base_index0 + row] = 2 * (interp_time[row] + interp_time[row + 1]);
        stack[base_index0 + row + 1] = interp_time[row + 1];
        memset(&stack[base_index0 + row + 2], 0, sizeof(double) * (order - row - 2));
    }

    base_index1 = S_A + order * (order - 2);
    memset(&stack[base_index1], 0, sizeof(double) * (order - 3));    // A[order - 2][0] ~ A[order - 2][order - 4]
    stack[base_index1 + order - 3] = interp_time[order - 2]; // A[order - 2][order - 3]
    stack[base_index1 + order - 2] = 2 * (interp_time[order - 2] + interp_time[order - 1]);  // A[order - 2][order - 2]
    stack[base_index1 + order - 1] = interp_time[order - 1] - interp_time[order] * interp_time[order] / interp_time[order - 1];  // A[order - 2][order - 1]
    memset(&stack[base_index1 + order], 0, sizeof(double) * (order - 2));    // A[order - 1][0] ~ A[order - 1][order - 3]
    base_index2 = base_index1 + 2 * order - 2;
    stack[base_index2] = interp_time[order - 1];    // A[order - 1][order - 2]
    stack[base_index2 + 1] = 3 * interp_time[order] + 2 * interp_time[order - 1] + interp_time[order] * interp_time[order] / interp_time[order - 1];    // A[order - 1][order - 1]
}

inline void updateMatrixB(double* path, double* interp_time, double* init_state, double* end_state, int order)
{
    stack[S_B] = 6 * (path[1] / interp_time[1] + init_state[0] / interp_time[0])
                - 6 * (1 / interp_time[0] + 1 / interp_time[1]) * (init_state[0] + init_state[1] * interp_time[0] + init_state[2] * interp_time[0] * interp_time[0] / 3)
                - init_state[2] * interp_time[0];
    stack[S_B + 1] = 6 * path[2] / interp_time[2]
                    - 6 * (1 / interp_time[1] + 1 / interp_time[2]) * path[1]
                    + 6 * (init_state[0] + init_state[1] * interp_time[0] + init_state[2] * interp_time[0] * interp_time[0] / 3) / interp_time[1];

    for(int row = 2; row < order - 2; ++row)
    {
        stack[S_B + row] = 6 * ((path[row + 1] - path[row]) / interp_time[row + 1] - (path[row] - path[row - 1]) / interp_time[row]);
    }

    stack[S_B + order - 2] = 6 * path[order - 3] / interp_time[order - 2]
                            - 6 * (1 / interp_time[order - 1] + 1 / interp_time[order - 2]) * path[order - 2]
                            + 6 * (end_state[0] - end_state[1] * interp_time[order] + end_state[2] * interp_time[order] * interp_time[order] / 3) / interp_time[order - 1];
    stack[S_B + order - 1] = 6 * end_state[0] / interp_time[order]
                            + 6 * path[order - 2] / interp_time[order - 1]
                            - 6 * (1 / interp_time[order] + 1 / interp_time[order - 1]) * (end_state[0] - end_state[1] * interp_time[order] + end_state[2] * interp_time[order] * interp_time[order] / 3)
                            - end_state[2] * interp_time[order];
}

void updateTrajPVA(int path_base, double* init_state, double* end_state, int order, int traj_base, int t_base)
{
    int i;
    updateMatrixA(&stack[t_base], order);   
    updateMatrixB(&stack[path_base], &stack[t_base], init_state, end_state, order); 
    updateEquationSolution(&stack[S_A], &stack[S_B], order);

    int traj_p = traj_base;
    int traj_v = traj_base + 50;
    int traj_a = traj_base + 100;
       
    // start point
    stack[traj_a] = init_state[2];
    stack[traj_v] = init_state[1];
    stack[traj_p] = init_state[0];

    // start add point
    stack[traj_a + 1] = stack[S_X];
    stack[traj_v + 1] = init_state[1] + (init_state[2] + stack[S_X]) * stack[t_base] / 2;
    stack[traj_p + 1] = init_state[0] 
                        + init_state[1] * stack[t_base]
                        + (2 * init_state[2] + stack[traj_a + 1]) * stack[t_base] * stack[t_base] / 6;

    // path[0]
    stack[traj_a + 2] = stack[S_X + 1];
    stack[traj_v + 2] = stack[traj_v + 1] + (stack[S_X] + stack[S_X + 1]) * stack[t_base + 1] / 2;
    stack[traj_p + 2] = stack[path_base + 1];

    // path[1] ~ path[N-3]
    for(i = 3; i < order; ++i)
    {
        stack[traj_a + i] = stack[S_X + i - 1];
        stack[traj_v + i] = stack[traj_v + i - 1] + (stack[traj_a + i - 1] + stack[traj_a + i]) * stack[t_base + i - 1] / 2;
        stack[traj_p + i] = stack[path_base + i - 1];
    }

    // path[N-2]
    stack[traj_a + order] = stack[S_X + order - 1];
    stack[traj_v + order] = end_state[1] - (end_state[2] + stack[traj_a + order]) * stack[t_base + order] / 2;
    stack[traj_p + order] = end_state[0]
                            - end_state[1] * stack[t_base + order]
                            + (2 * end_state[2] + stack[traj_a + order]) * stack[t_base + order] * stack[t_base + order] / 6;
    
    // path[N-1]
    stack[traj_a + order + 1] = end_state[2];
    stack[traj_v + order + 1] = end_state[1];
    stack[traj_p + order + 1] = end_state[0];   
}

void initComplexAxisGroupModel()
{
    model.robot_name = "rt-man";
    model.dh_type = DH_TYPE_STANDARD;
    model.cart_vm = 40000;
    model.cart_am = 400000;
    model.cart_jm = 4000000;
    model.link_num = 6;

    // link 0
    model.link[0].joint_type = JOINT_TYPE_ROTATE;
    model.link[0].link_kinematic.theta = 0;
    model.link[0].link_kinematic.d = 0.365;
    model.link[0].link_kinematic.alpha = M_PI/2;
    model.link[0].link_kinematic.a = 0.03;
    model.link[0].link_kinematic.theta_offset = 0;
    model.link[0].link_kinematic.d_offset = 0;
    model.link[0].link_kinematic.alpha_offset = 0;
    model.link[0].link_kinematic.a_offset = 0;
    model.link[0].link_kinematic.qlim[0] = -3.00197;
    model.link[0].link_kinematic.qlim[1] = 3.00197;
    model.link[0].link_kinematic.is_flip = false;
    model.link[0].motor_dynamic.vm = 4500;
    model.link[0].motor_dynamic.jm = 5 * 0.5 * 10000 / 1.3;
    model.link[0].motor_dynamic.gear = 81;

    // link 1
    model.link[1].joint_type = JOINT_TYPE_ROTATE;
    model.link[1].link_kinematic.theta = M_PI/2;
    model.link[1].link_kinematic.d = 0;
    model.link[1].link_kinematic.alpha = 0;
    model.link[1].link_kinematic.a = 0.3402;
    model.link[1].link_kinematic.theta_offset = 0;
    model.link[1].link_kinematic.d_offset = 0;
    model.link[1].link_kinematic.alpha_offset = 0;
    model.link[1].link_kinematic.a_offset = 0;
    model.link[1].link_kinematic.qlim[0] = -1.79769;
    model.link[1].link_kinematic.qlim[1] = 2.39110;
    model.link[1].link_kinematic.is_flip = false;
    model.link[1].motor_dynamic.vm = 4500;
    model.link[1].motor_dynamic.jm = 3.3 * 0.4 * 10000 / 0.44;
    model.link[1].motor_dynamic.gear = 100.908375;

    // link 2
    model.link[2].joint_type = JOINT_TYPE_ROTATE;
    model.link[2].link_kinematic.theta = 0;
    model.link[2].link_kinematic.d =  -0.0001952;
    model.link[2].link_kinematic.alpha = M_PI/2;
    model.link[2].link_kinematic.a = -0.03485;
    model.link[2].link_kinematic.theta_offset = 0;
    model.link[2].link_kinematic.d_offset = 0;
    model.link[2].link_kinematic.alpha_offset = 0;
    model.link[2].link_kinematic.a_offset = 0;
    model.link[2].link_kinematic.qlim[0] = -3.49066;
    model.link[2].link_kinematic.qlim[1] = 1.22173;
    model.link[2].link_kinematic.is_flip = false;
    model.link[2].motor_dynamic.vm = 4500;
    model.link[2].motor_dynamic.jm = 3.3 * 0.4 * 10000 / 0.44;
    model.link[2].motor_dynamic.gear = 81.053333;

    // link 3
    model.link[3].joint_type = JOINT_TYPE_ROTATE;
    model.link[3].link_kinematic.theta = 0;
    model.link[3].link_kinematic.d = 0.3503;
    model.link[3].link_kinematic.alpha = -M_PI/2;
    model.link[3].link_kinematic.a = 0;
    model.link[3].link_kinematic.theta_offset = 0;
    model.link[3].link_kinematic.d_offset = 0;
    model.link[3].link_kinematic.alpha_offset = 0;
    model.link[3].link_kinematic.a_offset = 0;
    model.link[3].link_kinematic.qlim[0] = -3.31613;
    model.link[3].link_kinematic.qlim[1] = 3.31613;
    model.link[3].link_kinematic.is_flip = false;
    model.link[3].motor_dynamic.vm = 4500;
    model.link[3].motor_dynamic.jm = 1.7 * 0.39 * 10000 / 0.18;
    model.link[3].motor_dynamic.gear = 59.987882;

    // link 4
    model.link[4].joint_type = JOINT_TYPE_ROTATE;
    model.link[4].link_kinematic.theta = 0;
    model.link[4].link_kinematic.d = 0;
    model.link[4].link_kinematic.alpha = M_PI/2;
    model.link[4].link_kinematic.a = 0;
    model.link[4].link_kinematic.theta_offset = 0;
    model.link[4].link_kinematic.d_offset = 0;
    model.link[4].link_kinematic.alpha_offset = 0;
    model.link[4].link_kinematic.a_offset = 0;
    model.link[4].link_kinematic.qlim[0] = -2.02458;
    model.link[4].link_kinematic.qlim[1] = 2.02458;
    model.link[4].link_kinematic.is_flip = false;
    model.link[4].motor_dynamic.vm = 4500;
    model.link[4].motor_dynamic.jm = 1.7 * 0.25 * 10000 / 0.17;
    model.link[4].motor_dynamic.gear = 66.75495;

    // link 5
    model.link[5].joint_type = JOINT_TYPE_ROTATE;
    model.link[5].link_kinematic.theta = 0;
    model.link[5].link_kinematic.d = 0.0965;
    model.link[5].link_kinematic.alpha = 0;
    model.link[5].link_kinematic.a = 0;
    model.link[5].link_kinematic.theta_offset = 0;
    model.link[5].link_kinematic.d_offset = 0;
    model.link[5].link_kinematic.alpha_offset = 0;
    model.link[5].link_kinematic.a_offset = 0;
    model.link[5].link_kinematic.qlim[0] = -6.28319;
    model.link[5].link_kinematic.qlim[1] = 6.28319;
    model.link[5].link_kinematic.is_flip = false;    
    model.link[5].motor_dynamic.vm = 4500;
    model.link[5].motor_dynamic.jm = 1.7 * 0.25 * 10000 / 0.17;
    model.link[5].motor_dynamic.gear = 44.671266;
#if 0
    // link 0
    model.link[0].joint_type = JOINT_TYPE_ROTATE;
    model.link[0].link_kinematic.theta = 0;
    model.link[0].link_kinematic.d = 0.365;
    model.link[0].link_kinematic.alpha = M_PI/2;
    model.link[0].link_kinematic.a = 0.03;
    model.link[0].link_kinematic.theta_offset = 0;
    model.link[0].link_kinematic.d_offset = 0;
    model.link[0].link_kinematic.alpha_offset = 0;
    model.link[0].link_kinematic.a_offset = 0;
    model.link[0].link_kinematic.qlim[0] = -3.00197;
    model.link[0].link_kinematic.qlim[1] = 3.00197;
    model.link[0].link_kinematic.is_flip = false;

    // link 1
    model.link[1].joint_type = JOINT_TYPE_ROTATE;
    model.link[1].link_kinematic.theta = M_PI/2;
    model.link[1].link_kinematic.d = 0;
    model.link[1].link_kinematic.alpha = 0;
    model.link[1].link_kinematic.a = 0.34;
    model.link[1].link_kinematic.theta_offset = 0;
    model.link[1].link_kinematic.d_offset = 0;
    model.link[1].link_kinematic.alpha_offset = 0;
    model.link[1].link_kinematic.a_offset = 0;
    model.link[1].link_kinematic.qlim[0] = -1.79769;
    model.link[1].link_kinematic.qlim[1] = 2.39110;
    model.link[1].link_kinematic.is_flip = false;

    // link 2
    model.link[2].joint_type = JOINT_TYPE_ROTATE;
    model.link[2].link_kinematic.theta = 0;
    model.link[2].link_kinematic.d = 0;
    model.link[2].link_kinematic.alpha = M_PI/2;
    model.link[2].link_kinematic.a = 0.035;
    model.link[2].link_kinematic.theta_offset = 0;
    model.link[2].link_kinematic.d_offset = 0;
    model.link[2].link_kinematic.alpha_offset = 0;
    model.link[2].link_kinematic.a_offset = 0;
    model.link[2].link_kinematic.qlim[0] = -3.49066;
    model.link[2].link_kinematic.qlim[1] = 1.22173;
    model.link[2].link_kinematic.is_flip = false;

    // link 3
    model.link[3].joint_type = JOINT_TYPE_ROTATE;
    model.link[3].link_kinematic.theta = 0;
    model.link[3].link_kinematic.d = 0.35;
    model.link[3].link_kinematic.alpha = -M_PI/2;
    model.link[3].link_kinematic.a = 0;
    model.link[3].link_kinematic.theta_offset = 0;
    model.link[3].link_kinematic.d_offset = 0;
    model.link[3].link_kinematic.alpha_offset = 0;
    model.link[3].link_kinematic.a_offset = 0;
    model.link[3].link_kinematic.qlim[0] = -3.31613;
    model.link[3].link_kinematic.qlim[1] = 3.31613;
    model.link[3].link_kinematic.is_flip = false;

    // link 4
    model.link[4].joint_type = JOINT_TYPE_ROTATE;
    model.link[4].link_kinematic.theta = 0;
    model.link[4].link_kinematic.d = 0;
    model.link[4].link_kinematic.alpha = M_PI/2;
    model.link[4].link_kinematic.a = 0;
    model.link[4].link_kinematic.theta_offset = 0;
    model.link[4].link_kinematic.d_offset = 0;
    model.link[4].link_kinematic.alpha_offset = 0;
    model.link[4].link_kinematic.a_offset = 0;
    model.link[4].link_kinematic.qlim[0] = -2.02458;
    model.link[4].link_kinematic.qlim[1] = 2.02458;
    model.link[4].link_kinematic.is_flip = false;

    // link 5
    model.link[5].joint_type = JOINT_TYPE_ROTATE;
    model.link[5].link_kinematic.theta = 0;
    model.link[5].link_kinematic.d = 0.0965;
    model.link[5].link_kinematic.alpha = 0;
    model.link[5].link_kinematic.a = 0;
    model.link[5].link_kinematic.theta_offset = 0;
    model.link[5].link_kinematic.d_offset = 0;
    model.link[5].link_kinematic.alpha_offset = 0;
    model.link[5].link_kinematic.a_offset = 0;
    model.link[5].link_kinematic.qlim[0] = -6.28319;
    model.link[5].link_kinematic.qlim[1] = 6.28319;
    model.link[5].link_kinematic.is_flip = false;
#endif
}

void initStack(ComplexAxisGroupModel* model_ptr)
{
    // robot model related
    for(int i=0; i<model_ptr->link_num; ++i)
    {
        stack[S_RealTheta+i] = model_ptr->link[i].link_kinematic.theta + model_ptr->link[i].link_kinematic.theta_offset;
        stack[S_RealD+i] = model_ptr->link[i].link_kinematic.d + model_ptr->link[i].link_kinematic.d_offset;
        stack[S_RealAlpha+i] = model_ptr->link[i].link_kinematic.alpha + model_ptr->link[i].link_kinematic.alpha_offset;
        stack[S_RealA+i] = model_ptr->link[i].link_kinematic.a + model_ptr->link[i].link_kinematic.a_offset;
        stack[S_ConstraintJointVelMax + i] = model_ptr->link[i].motor_dynamic.vm * PI * 2 / (60 * model_ptr->link[i].motor_dynamic.gear);
        stack[S_ConstraintJointJerkMax + i] = segment_alg_param.jerk_ratio * model_ptr->link[i].motor_dynamic.jm * model_ptr->link[i].motor_dynamic.gear;
    }

    // S_TransMatrix
    memset(&stack[S_TransMatrix], 0, sizeof(double) * 16);
    stack[S_TransMatrix + 15] = 1;

    // S_HomoTransMatrix
    memset(&stack[S_HomoTransMatrix], 0, sizeof(double) * 16);
    stack[S_HomoTransMatrix + 15] = 1;

    // S_NodeVector
    stack[S_BSplineNodeVector] = 0;
    stack[S_BSplineNodeVector + 1] = 0;
    stack[S_BSplineNodeVector + 2] = 0;
    stack[S_BSplineNodeVector + 3] = 1;
    stack[S_BSplineNodeVector + 4] = 1;
    stack[S_BSplineNodeVector + 5] = 1;

    // path count factor
    stack[S_PathCountFactorCartesian] = segment_alg_param.accuracy_cartesian_factor / 100.0;
    stack[S_PathCountFactorJoint] = segment_alg_param.accuracy_joint_factor / PI;
}

void initSegmentAlgParam(SegmentAlgParam* segment_alg_param_ptr)
{
    segment_alg_param.accuracy_cartesian_factor = segment_alg_param_ptr->accuracy_cartesian_factor;
    segment_alg_param.accuracy_joint_factor = segment_alg_param_ptr->accuracy_joint_factor;
    segment_alg_param.max_traj_points_num = segment_alg_param_ptr->max_traj_points_num;
    segment_alg_param.path_interval = segment_alg_param_ptr->path_interval;
    segment_alg_param.joint_interval = segment_alg_param_ptr->joint_interval;
    segment_alg_param.angle_interval = segment_alg_param_ptr->angle_interval;
    segment_alg_param.angle_valve = segment_alg_param_ptr->angle_valve;
    segment_alg_param.conservative_acc = segment_alg_param_ptr->conservative_acc;
    segment_alg_param.jerk_ratio = segment_alg_param_ptr->jerk_ratio;
    segment_alg_param.time_factor_1 = segment_alg_param_ptr->time_factor_1;
    segment_alg_param.time_factor_2 = segment_alg_param_ptr->time_factor_2;
    segment_alg_param.time_factor_3 = segment_alg_param_ptr->time_factor_3;
    segment_alg_param.time_factor_4 = segment_alg_param_ptr->time_factor_4;
    segment_alg_param.is_fake_dynamics = segment_alg_param_ptr->is_fake_dynamics;
    segment_alg_param.max_rescale_factor = segment_alg_param_ptr->max_rescale_factor;
    segment_alg_param.kinematics_ptr = segment_alg_param_ptr->kinematics_ptr;
    segment_alg_param.dynamics_ptr = segment_alg_param_ptr->dynamics_ptr;

    initStack(&model);
}

ErrorCode planPathJoint(const Joint &start, 
                            const MotionTarget &target, 
                            PathCache &path_cache)
{
    int i, j;
    // find max delta joint 
    double delta_joint;
    double max_delta_joint = 0;
    for(i = 0; i < model.link_num; ++i)
    {
        delta_joint = fabs(target.joint_target[i] - start[i]);
        if(delta_joint > max_delta_joint)
        {
            max_delta_joint = delta_joint;
        }
    }
    if(max_delta_joint < DOUBLE_ACCURACY)
    {
        return 6002;
    }
  
    // init unused data
    path_cache.smooth_in_index = -1;
    path_cache.smooth_out_index = -1;
    // compute interpolation points
    int path_count_minus_1 = ceil(max_delta_joint / segment_alg_param.joint_interval);
    path_cache.cache_length = path_count_minus_1 + 1;
  
    double joint_step, joint_to_start;
    
    for(i = 0; i < model.link_num; ++i)
    {    
        path_cache.cache[0].joint[i] = start[i];
        path_cache.cache[0].point_type = PATH_POINT;
        path_cache.cache[0].motion_type = MOTION_JOINT;    
        joint_step = (target.joint_target[i] - start[i]) / path_count_minus_1;
        joint_to_start = 0; 
        for(j = 1; j < path_count_minus_1; ++j)
        {
            joint_to_start += joint_step;
            path_cache.cache[j].joint[i] = start[i] + joint_to_start;
            path_cache.cache[j].point_type = PATH_POINT;
            path_cache.cache[j].motion_type = MOTION_JOINT;
        }
        path_cache.cache[path_count_minus_1].joint[i] = target.joint_target[i];
        path_cache.cache[path_count_minus_1].point_type = PATH_POINT;
        path_cache.cache[path_count_minus_1].motion_type = MOTION_JOINT;
    }

    return 0;
}

ErrorCode planPathLine(const PoseEuler &start, 
                            const MotionTarget &target, 
                            PathCache &path_cache)
{
    int i;
    
    // init unused data
    path_cache.smooth_in_index = -1;
    // compute MoveL length
    double path_length;
    double path_vector[3];
    getMoveLPathVector(start.position, target.pose_target.position, path_vector, path_length);   // MoveL length
    if(path_length < DOUBLE_ACCURACY)
    {
        return 6001;
    }
    
    // compute MoveL quatern angle
    double start_quatern[4], end_quatern[4];
    getMoveEulerToQuatern(start.orientation, start_quatern);
    getMoveEulerToQuatern(target.pose_target.orientation, end_quatern);
    double angle = getQuaternsIntersectionAngle(start_quatern, end_quatern);    // MoveL quatern angle

    int path_count_ideal = ceil(path_length / segment_alg_param.path_interval);
    int angle_count_ideal = ceil(angle / segment_alg_param.angle_interval);
    int max_count = ((path_count_ideal >= angle_count_ideal) ? path_count_ideal : angle_count_ideal);

    // find Pout distance to end point
    double point_distance_to_start = 0; // not scaled
    double angle_distance_to_start = 0; // scale to [0,1]
    if(target.cnt >= DOUBLE_ACCURACY)    // cnt is valid
    {
        // FIXME: small cnt will cause pulse in vel and acc of traj
        //double path_out_vel = target.vel * target.cnt;
        //double max_path_out_length = path_length / 2;
        //double path_length_after_out = path_out_vel * path_out_vel / (2 * segment_alg_param.conservative_acc);
        double path_out_vel = target.vel;
        double max_path_out_length = path_length / 2;
        double path_length_after_out = path_out_vel * path_out_vel * target.cnt/ (2 * segment_alg_param.conservative_acc);        
        if(path_length_after_out > max_path_out_length)
        {
            path_length_after_out = max_path_out_length;
        }     
        int path_count_after_out = ceil(path_length_after_out * max_count / path_length);
        double path_step_after_out = path_length_after_out / path_count_after_out;        
        
        double path_length_before_out = path_length - path_length_after_out;
        double angle_out_distance_to_start = path_length_before_out / path_length;
        int path_count_before_out = ceil(angle_out_distance_to_start * max_count);
        double path_step_before_out = path_length_before_out / path_count_before_out;

        double angle_step_before_out = angle_out_distance_to_start / path_count_before_out;
        double angle_step_after_out = (1.0 - angle_out_distance_to_start) / path_count_after_out;

        path_cache.cache[0].pose.position = start.position;
        path_cache.cache[0].pose.orientation.x = start_quatern[0];
        path_cache.cache[0].pose.orientation.y = start_quatern[1];
        path_cache.cache[0].pose.orientation.z = start_quatern[2];
        path_cache.cache[0].pose.orientation.w = start_quatern[3];
        path_cache.cache[0].point_type = PATH_POINT;
        path_cache.cache[0].motion_type = MOTION_LINE;
        for(i = 1; i <= path_count_before_out; ++i)
        {
            point_distance_to_start += path_step_before_out;
            angle_distance_to_start += angle_step_before_out;            
            getMoveLPathPoint(start.position, path_vector, point_distance_to_start, path_cache.cache[i].pose.position);
            getQuaternPoint(start_quatern, end_quatern, angle, angle_distance_to_start, path_cache.cache[i].pose.orientation);
            path_cache.cache[i].point_type = PATH_POINT;
            path_cache.cache[i].motion_type = MOTION_LINE;
        }
        path_cache.smooth_out_index = path_count_before_out;
        int path_count_total_minus_1 = path_count_before_out + path_count_after_out;
        path_cache.cache_length = path_count_total_minus_1 + 1;
        for(; i < path_count_total_minus_1; ++i)
        {
            point_distance_to_start += path_step_after_out;
            angle_distance_to_start += angle_step_after_out;
            getMoveLPathPoint(start.position, path_vector, point_distance_to_start, path_cache.cache[i].pose.position);
            getQuaternPoint(start_quatern, end_quatern, angle, angle_distance_to_start, path_cache.cache[i].pose.orientation);
            path_cache.cache[i].point_type = PATH_POINT;
            path_cache.cache[i].motion_type = MOTION_LINE;
        }
        path_cache.cache[path_count_total_minus_1].pose.position = target.pose_target.position; // keep accuracy
        path_cache.cache[path_count_total_minus_1].pose.orientation.x = end_quatern[0];
        path_cache.cache[path_count_total_minus_1].pose.orientation.y = end_quatern[1];
        path_cache.cache[path_count_total_minus_1].pose.orientation.z = end_quatern[2];
        path_cache.cache[path_count_total_minus_1].pose.orientation.w = end_quatern[3];
        path_cache.cache[i].point_type = PATH_POINT;
        path_cache.cache[i].motion_type = MOTION_LINE;
    }
    else    // cnt is invalid
    {    
        if(target.cnt >= -DOUBLE_ACCURACY)  // cnt = 0
        {
            path_cache.smooth_out_index = max_count;
        }
        else
        {
            path_cache.smooth_out_index = -1;
        }        
        path_cache.cache_length = max_count + 1;
        double path_step = path_length / max_count;
        double angle_step = 1.0 / max_count;
        path_cache.cache[0].pose.position = start.position;
        path_cache.cache[0].pose.orientation.x = start_quatern[0];
        path_cache.cache[0].pose.orientation.y = start_quatern[1];
        path_cache.cache[0].pose.orientation.z = start_quatern[2];
        path_cache.cache[0].pose.orientation.w = start_quatern[3];
        path_cache.cache[0].point_type = PATH_POINT;
        path_cache.cache[0].motion_type = MOTION_LINE;         
        for(i = 1; i < max_count; ++i)
        {
            point_distance_to_start += path_step;
            angle_distance_to_start += angle_step;
            getMoveLPathPoint(start.position, path_vector, point_distance_to_start, path_cache.cache[i].pose.position);
            getQuaternPoint(start_quatern, end_quatern, angle, angle_distance_to_start, path_cache.cache[i].pose.orientation);
            path_cache.cache[i].point_type = PATH_POINT;
            path_cache.cache[i].motion_type = MOTION_LINE;
        }
        path_cache.cache[max_count].pose.position = target.pose_target.position; // keep accuracy
        path_cache.cache[max_count].pose.orientation.x = end_quatern[0];
        path_cache.cache[max_count].pose.orientation.y = end_quatern[1];
        path_cache.cache[max_count].pose.orientation.z = end_quatern[2];
        path_cache.cache[max_count].pose.orientation.w = end_quatern[3];
        path_cache.cache[max_count].point_type = PATH_POINT;
        path_cache.cache[max_count].motion_type = MOTION_LINE;
    }
    
    return 0;
}

ErrorCode planPathCircle(const PoseEuler &start, 
                                const MotionTarget &target, 
                                PathCache &path_cache)
{
    return 0;
}

ErrorCode planPathSmoothJoint(const Joint &start, 
                                    const MotionTarget &via, 
                                    const MotionTarget &target, 
                                    PathCache &path_cache)
{
    return 0;
}

ErrorCode planPathSmoothLine(const PoseEuler &start, 
                                    const MotionTarget &via, 
                                    const MotionTarget &target, 
                                    PathCache &path_cache)
{
    int i;
    // compute path
    double path_length_start2via = getPointsDistance(start.position, via.pose_target.position);   
    double path_length_via2target;
    double path_vector_via2target[3];
    getMoveLPathVector(via.pose_target.position, target.pose_target.position, path_vector_via2target, path_length_via2target);
    double path_length_via2in = path_length_via2target / 2;
    
    if(path_length_start2via < path_length_via2in)
    {
        path_length_via2in = path_length_start2via;
    }   
    double path_length_in2target = path_length_via2target - path_length_via2in;
        
    int path_count_ideal_start2via = ceil(path_length_start2via / segment_alg_param.path_interval);
    int path_count_ideal_via2in = ceil(path_length_via2in / segment_alg_param.path_interval);
    Point point_in;
    getMoveLPathPoint(via.pose_target.position, path_vector_via2target, path_length_via2in, point_in);
    
    // compute quatern
    double quatern_start[4], quatern_via[4], quatern_in[4], quatern_target[4];
    getMoveEulerToQuatern(start.orientation, quatern_start);
    getMoveEulerToQuatern(via.pose_target.orientation, quatern_via);
    getMoveEulerToQuatern(target.pose_target.orientation, quatern_target);
    double angle_start2via = getQuaternsIntersectionAngle(quatern_start, quatern_via);   
    double angle_via2target = getQuaternsIntersectionAngle(quatern_via, quatern_target);
    double angle_count_ideal_start2via = ceil(angle_start2via / segment_alg_param.angle_interval);    
    double angle_count_ideal_via2target = ceil(angle_via2target / segment_alg_param.angle_interval);
    double angle_distance_via2in = path_length_via2in / path_length_via2target; // scale to [0,1]   
    double angle_in2target = (1 - angle_distance_via2in) * angle_via2target;
    int angle_count_ideal_via2in = ceil(angle_distance_via2in * angle_via2target / segment_alg_param.angle_interval);
    getQuaternVector4(quatern_via, quatern_target, angle_via2target, angle_distance_via2in, quatern_in);
    double angle_transition = getQuaternsIntersectionAngle(quatern_start, quatern_in);
    
    // determine path count
    int path_count_start2via = (path_count_ideal_start2via > angle_count_ideal_start2via ? path_count_ideal_start2via : angle_count_ideal_start2via);
    int path_count_via2in = (path_count_ideal_via2in > angle_count_ideal_via2in ? path_count_ideal_via2in : angle_count_ideal_via2in);
    int path_count_transition = path_count_start2via + path_count_via2in;
    path_cache.smooth_in_index = path_count_transition;

    // determine transition angle step
    double angle_step_transition = 1.0 / path_count_transition;

    // determine out point
    double point_distance_to_in = 0; // not scaled
    double angle_distance_to_start = 0; // scale to [0,1]
    double angle_distance_to_in = 0; // scale to [0,1]
    double angle_distance_to_out = 0; // scale to [0,1]
    double start_point[3], via_point[3], in_point[3];
    if(target.cnt >= DOUBLE_ACCURACY)
    {   // FIXME: small cnt will cause pulse in vel and acc of traj
        // compute path in2out and out2target
        //double path_out_vel = target.vel * target.cnt;
        //double max_path_length_out2target = path_length_via2target / 2;
        //double path_length_out2target = path_out_vel * path_out_vel / (2 * segment_alg_param.conservative_acc);  
        double path_out_vel = target.vel;
        double max_path_length_out2target = path_length_via2target / 2;
        double path_length_out2target = path_out_vel * path_out_vel * target.cnt / (2 * segment_alg_param.conservative_acc);
        if(path_length_out2target > max_path_length_out2target)
        {
            path_length_out2target = max_path_length_out2target;
        }
        double path_length_in2out = path_length_in2target - path_length_out2target;        
        Point point_out;
        getMoveLPathPoint(point_in, path_vector_via2target, path_length_in2out, point_out);
        int path_count_ideal_in2out = 0;
        if(path_length_in2out > DOUBLE_ACCURACY)
        {
            path_count_ideal_in2out = ceil(path_length_in2out / segment_alg_param.path_interval);
        }
        int path_count_ideal_out2target = ceil(path_length_out2target / segment_alg_param.path_interval);       
        // compute angle in2out and out2target
        double angle_distance_in2out = path_length_in2out / path_length_in2target;
        double quatern_out[4];
        getQuaternVector4(quatern_in, quatern_target, angle_in2target, angle_distance_in2out, quatern_out);
        double angle_in2out = getQuaternsIntersectionAngle(quatern_in, quatern_out);
        double angle_out2target = angle_in2target - angle_in2out;
        int angle_count_ideal_in2out = 0;
        if(angle_in2out > DOUBLE_ACCURACY)
        {
            angle_count_ideal_in2out = ceil(angle_in2out / segment_alg_param.angle_interval);
        }
        int angle_count_ideal_out2target = ceil(angle_out2target / segment_alg_param.angle_interval);

        // determine path count
        int path_count_in2out = (path_count_ideal_in2out > angle_count_ideal_in2out ? path_count_ideal_in2out : angle_count_ideal_in2out);
        int path_count_out2target = (path_count_ideal_out2target > angle_count_ideal_out2target ? path_count_ideal_out2target : angle_count_ideal_out2target);
        path_cache.smooth_out_index = path_cache.smooth_in_index + path_count_in2out;
        int path_cache_length_minus_1 = path_count_transition + path_count_in2out + path_count_out2target;
        path_cache.cache_length = path_cache_length_minus_1 + 1;

        // determine step
        double path_step_in2out = path_length_in2out / path_count_in2out;
        double path_step_out2target = path_length_out2target / path_count_out2target;
        double angle_step_in2out = 1.0 / path_count_in2out;
        double angle_step_out2target = 1.0 / path_count_out2target;

        // compute transition path
        getMovePointToVector3(start.position, start_point);
        getMovePointToVector3(via.pose_target.position, via_point);
        getMovePointToVector3(point_in, in_point);
        updateTransitionBSpLineResult(2, start_point, via_point, in_point, path_cache.smooth_in_index);

        path_cache.cache[0].pose.position = start.position;
        path_cache.cache[0].pose.orientation.x = quatern_start[0];
        path_cache.cache[0].pose.orientation.y = quatern_start[1];
        path_cache.cache[0].pose.orientation.z = quatern_start[2];
        path_cache.cache[0].pose.orientation.w = quatern_start[3];
        path_cache.cache[0].point_type = PATH_POINT;
        path_cache.cache[0].motion_type = MOTION_LINE;        
        for(i = 1; i < path_cache.smooth_in_index; ++i)
        {
            path_cache.cache[i].pose.position.x = stack[S_BSpLineResultXBase + i];
            path_cache.cache[i].pose.position.y = stack[S_BSpLineResultYBase + i];
            path_cache.cache[i].pose.position.z = stack[S_BSpLineResultZBase + i];
            angle_distance_to_start += angle_step_transition;
            getQuaternPoint(quatern_start, quatern_in, angle_transition, angle_distance_to_start, path_cache.cache[i].pose.orientation);
            path_cache.cache[i].point_type = TRANSITION_POINT;
            path_cache.cache[i].motion_type = MOTION_LINE;
        }
        path_cache.cache[path_cache.smooth_in_index].pose.position = point_in;
        path_cache.cache[path_cache.smooth_in_index].pose.orientation.x = quatern_in[0];
        path_cache.cache[path_cache.smooth_in_index].pose.orientation.y = quatern_in[1];
        path_cache.cache[path_cache.smooth_in_index].pose.orientation.z = quatern_in[2];
        path_cache.cache[path_cache.smooth_in_index].pose.orientation.w = quatern_in[3];
        path_cache.cache[path_cache.smooth_in_index].point_type = TRANSITION_POINT;
        path_cache.cache[path_cache.smooth_in_index].motion_type = MOTION_LINE;

        // compute in2out path
        for(i = path_cache.smooth_in_index + 1; i < path_cache.smooth_out_index; ++i)
        {
            point_distance_to_in += path_step_in2out;
            angle_distance_to_in += angle_step_in2out;
            getMoveLPathPoint(point_in, path_vector_via2target, point_distance_to_in, path_cache.cache[i].pose.position);
            getQuaternPoint(quatern_in, quatern_out, angle_in2out, angle_distance_to_in, path_cache.cache[i].pose.orientation);
            path_cache.cache[i].point_type = PATH_POINT;
            path_cache.cache[i].motion_type = MOTION_LINE;
        }
        path_cache.cache[path_cache.smooth_out_index].pose.position = point_out;
        path_cache.cache[path_cache.smooth_out_index].pose.orientation.x = quatern_out[0];
        path_cache.cache[path_cache.smooth_out_index].pose.orientation.y = quatern_out[1];
        path_cache.cache[path_cache.smooth_out_index].pose.orientation.z = quatern_out[2];
        path_cache.cache[path_cache.smooth_out_index].pose.orientation.w = quatern_out[3];
        path_cache.cache[path_cache.smooth_out_index].point_type = PATH_POINT;
        path_cache.cache[path_cache.smooth_out_index].motion_type = MOTION_LINE;        

        // compute out2target path
        for(i = path_cache.smooth_out_index + 1; i < path_cache_length_minus_1; ++i)
        {
            point_distance_to_in += path_step_out2target;
            angle_distance_to_out += angle_step_out2target;
            getMoveLPathPoint(point_in, path_vector_via2target, point_distance_to_in, path_cache.cache[i].pose.position);
            getQuaternPoint(quatern_out, quatern_target, angle_out2target, angle_distance_to_out, path_cache.cache[i].pose.orientation);
            path_cache.cache[i].point_type = PATH_POINT;
            path_cache.cache[i].motion_type = MOTION_LINE;
        }
        path_cache.cache[path_cache_length_minus_1].pose.position = target.pose_target.position;
        path_cache.cache[path_cache_length_minus_1].pose.orientation.x = quatern_target[0];
        path_cache.cache[path_cache_length_minus_1].pose.orientation.y = quatern_target[1];
        path_cache.cache[path_cache_length_minus_1].pose.orientation.z = quatern_target[2];
        path_cache.cache[path_cache_length_minus_1].pose.orientation.w = quatern_target[3];
        path_cache.cache[path_cache_length_minus_1].point_type = PATH_POINT;
        path_cache.cache[path_cache_length_minus_1].motion_type = MOTION_LINE;
    }
    else
    {       
        // compute in2target settings
        int path_count_ideal_in2target = ceil(path_length_in2target / segment_alg_param.path_interval);
        int angle_count_ideal_in2target = ceil(angle_in2target / segment_alg_param.angle_interval);
        int path_count_in2target = (path_count_ideal_in2target > angle_count_ideal_in2target ? path_count_ideal_in2target : angle_count_ideal_in2target);
        int path_cache_length_minus_1 = path_count_transition + path_count_in2target;
        path_cache.cache_length = path_cache_length_minus_1 + 1;       
        double path_step_in2target = path_length_in2target / path_count_in2target;
        double angle_step_in2target = 1.0 / path_count_in2target;
        if(target.cnt >= -DOUBLE_ACCURACY)  // cnt = 0
        {
            path_cache.smooth_out_index = path_cache_length_minus_1;
        }
        else
        {
            path_cache.smooth_out_index = -1;
        }  

        // compute transition path
        getMovePointToVector3(start.position, start_point);
        getMovePointToVector3(via.pose_target.position, via_point);
        getMovePointToVector3(point_in, in_point);
        updateTransitionBSpLineResult(2, start_point, via_point, in_point, path_cache.smooth_in_index);
        path_cache.cache[0].pose.position = start.position;
        path_cache.cache[0].pose.orientation.x = quatern_start[0];
        path_cache.cache[0].pose.orientation.y = quatern_start[1];
        path_cache.cache[0].pose.orientation.z = quatern_start[2];
        path_cache.cache[0].pose.orientation.w = quatern_start[3];
        path_cache.cache[0].point_type = PATH_POINT;
        path_cache.cache[0].motion_type = MOTION_LINE;         
        for(i = 1; i < path_cache.smooth_in_index; ++i)
        {
            path_cache.cache[i].pose.position.x = stack[S_BSpLineResultXBase + i];
            path_cache.cache[i].pose.position.y = stack[S_BSpLineResultYBase + i];
            path_cache.cache[i].pose.position.z = stack[S_BSpLineResultZBase + i];
            angle_distance_to_start += angle_step_transition;
            getQuaternPoint(quatern_start, quatern_in, angle_transition, angle_distance_to_start, path_cache.cache[i].pose.orientation);
            path_cache.cache[i].point_type = TRANSITION_POINT;
            path_cache.cache[i].motion_type = MOTION_LINE;
        }
        path_cache.cache[path_cache.smooth_in_index].pose.position = point_in;
        path_cache.cache[path_cache.smooth_in_index].pose.orientation.x = quatern_in[0];
        path_cache.cache[path_cache.smooth_in_index].pose.orientation.y = quatern_in[1];
        path_cache.cache[path_cache.smooth_in_index].pose.orientation.z = quatern_in[2];
        path_cache.cache[path_cache.smooth_in_index].pose.orientation.w = quatern_in[3];
        path_cache.cache[path_cache.smooth_in_index].point_type = TRANSITION_POINT;
        path_cache.cache[path_cache.smooth_in_index].motion_type = MOTION_LINE;

        // compute in2target path
        for(i = path_cache.smooth_in_index + 1; i < path_cache_length_minus_1; ++i)
        {
            point_distance_to_in += path_step_in2target;
            angle_distance_to_in += angle_step_in2target;
            getMoveLPathPoint(point_in, path_vector_via2target, point_distance_to_in, path_cache.cache[i].pose.position);
            getQuaternPoint(quatern_in, quatern_target, angle_in2target, angle_distance_to_in, path_cache.cache[i].pose.orientation);
            path_cache.cache[i].point_type = PATH_POINT;
            path_cache.cache[i].motion_type = MOTION_LINE;
        }
        path_cache.cache[path_cache_length_minus_1].pose.position = target.pose_target.position;
        path_cache.cache[path_cache_length_minus_1].pose.orientation.x = quatern_target[0];
        path_cache.cache[path_cache_length_minus_1].pose.orientation.y = quatern_target[1];
        path_cache.cache[path_cache_length_minus_1].pose.orientation.z = quatern_target[2];
        path_cache.cache[path_cache_length_minus_1].pose.orientation.w = quatern_target[3];
        path_cache.cache[path_cache_length_minus_1].point_type = PATH_POINT;
        path_cache.cache[path_cache_length_minus_1].motion_type = MOTION_LINE;
    }

    return 0;
}

ErrorCode planPathSmoothCircle(const PoseEuler &start, 
                                        const MotionTarget &via, 
                                        const MotionTarget &target, 
                                        PathCache &path_cache)
{
    return 0;
}

ErrorCode planTrajectory(const PathCache &path_cache, 
                                const JointState &start_state, 
                                double vel_ratio, 
                                double acc_ratio, 
                                TrajectoryCache &traj_cache)
{
    if(path_cache.cache_length == 0)
    {
        return 6003;
    }

    int path_index_array[25];
    int path_index_array_size, time_vector_size;
    double cmd_vel = path_cache.target.vel * vel_ratio;   // command velocity
    switch(path_cache.cache[0].motion_type)
    {
        case MOTION_LINE:
        {            
            int path_index_array_smooth_out_index;
            double path_length_start2out, path_length_out2end;
            // write S_PathIndexStep, S_PathIndexStepStart2Out, S_PathIndexStepOut2End, S_Path0~S_Path8
            generateMoveLPathPoint(path_cache, start_state, path_index_array, path_index_array_size, 
                                    path_index_array_smooth_out_index, path_length_start2out, path_length_out2end);   

            if(path_cache.smooth_out_index == -1)    // no cnt
            {
                traj_cache.smooth_out_index = -1;
            }
            else if(path_cache.smooth_out_index == (path_cache.cache_length - 1))    // cnt = 0
            {
                traj_cache.smooth_out_index = path_index_array_size;
            }
            else    // cnt > 0
            {
                traj_cache.smooth_out_index = path_index_array_smooth_out_index;
            }
           
            // write S_TrajT
            generateMoveLTimeVector(cmd_vel, path_index_array_size, path_index_array_smooth_out_index,
                                    path_length_start2out, path_length_out2end, time_vector_size);  
            break;
        }
        case MOTION_JOINT:
        {
            // write S_DeltaJointVector, S_PathIndexStep, S_Path0~S_Path8
            generateMoveJPathPoint(path_cache, path_index_array, path_index_array_size);
            // read S_DeltaJointVector, S_ConstraintJointVelMax
            // write S_TrajT
            generateMoveJTimeVector(vel_ratio, path_index_array_size, time_vector_size);  
            // MoveJ doesn't support smooth yet
            traj_cache.smooth_out_index = -1;
            break;
        }
        default:
        {
            return 6004;
        }        
    }    

    // init state {P,V,A}
    stack[S_TmpVector3_1 + 1] = 0;
    stack[S_TmpVector3_1 + 2] = 0;
    // end state {P,V,A}
    stack[S_TmpVector3_2 + 1] = 0;
    stack[S_TmpVector3_2 + 2] = 0;

    int i;
    int path_base = S_Path0;
    int traj_base = S_TrajP0;
    for(i = 0; i < model.link_num; ++i)
    {
        stack[S_TmpVector3_1] = stack[path_base];
        stack[S_TmpVector3_2] = stack[path_base + path_index_array_size - 1];
        // read: S_Path0 ~ S_Path8, S_TrajT
        // write: S_TrajP0 ~ S_TrajP8, S_TrajV0 ~ S_TrajV8, S_TrajA0 ~ S_TrajA8
        updateTrajPVA(path_base, &stack[S_TmpVector3_1], &stack[S_TmpVector3_2], path_index_array_size, traj_base, S_TrajT);
        path_base += 25;
        traj_base += 150;
    }
    
    // read: S_TrajV0 ~ S_TrajV8, S_TrajA0 ~ S_TrajA8
    // write: S_TrajPieceJ0 ~ S_TrajPieceJ5, S_TrajPieceA0 ~ S_TrajPieceA5, S_TrajPieceV0 ~ S_TrajPieceV5
    //        S_ConstraintJointPosA0 ~ S_ConstraintJointPosA5, S_ConstraintJointNegA0 ~ S_ConstraintJointNegA5
    generatePieceVectors(0, time_vector_size, S_TrajT, vel_ratio, acc_ratio);
    // read: S_TrajPieceV0 ~ S_TrajPieceV8, S_TrajPieceA0 ~ S_TrajPieceA8, S_TrajPieceJ0 ~ S_TrajPieceJ8      
    // write: S_TrajRescaleFactor
    generateRescaleFactorVector(time_vector_size, stack[S_TrajRescaleFactor]);

    adjustPVA2(time_vector_size, stack[S_TrajRescaleFactor]);
    // read: S_TrajRescaleFactor, S_TrajT
    // write: S_TrajRescaleT
    generateRescaleVector(time_vector_size);
    // read: S_TrajRescaleT
    // write: S_TrajAbsoluteT
    generateAbsoluteVector(time_vector_size);
    // read: S_TrajRescaleT, S_TrajRescaleFactor, S_TrajP0 ~ S_TrajP8, S_TrajA0 ~ S_TrajA8
    // write: S_TrajAbsoluteT, S_TrajCoeffJ0A3 ~ S_TrajCoeffJ8A3, S_TrajCoeffJ0A2 ~ S_TrajCoeffJ8A2, 
    //        S_TrajCoeffJ0A1 ~ S_TrajCoeffJ8A1, S_TrajCoeffJ0A0 ~ S_TrajCoeffJ8A0
    generateCoeff(time_vector_size);

    /*for(int i=0; i<time_vector_size; ++i)
        std::cout<<i<<" A3 = "<<stack[S_TrajCoeffJ1A3 + i]
                    <<" A2 = "<<stack[S_TrajCoeffJ1A2 + i]
                    <<" A1 = "<<stack[S_TrajCoeffJ1A1 + i]
                    <<" A0 = "<<stack[S_TrajCoeffJ1A0 + i]<<std::endl*/;

    // read: S_TrajCoeffJ0A3 ~ S_TrajCoeffJ8A3, S_TrajCoeffJ0A2 ~ S_TrajCoeffJ8A2, 
    //       S_TrajCoeffJ0A1 ~ S_TrajCoeffJ8A1, S_TrajCoeffJ0A0 ~ S_TrajCoeffJ8A0,
    //       S_TrajRescaleT
    generateTrajCache(traj_cache, time_vector_size, path_index_array, path_index_array_size);

    return 0;
}
                                
ErrorCode planTrajectorySmooth(const PathCache &path_cache, 
                                        const JointState &start_state, 
                                        const MotionTarget &via, 
                                        double vel_ratio, 
                                        double acc_ratio, 
                                        TrajectoryCache &traj_cache)
{
    if(path_cache.cache_length == 0)
    {
        return 6003;
    }

    if(path_cache.smooth_in_index == -1)
    {
        return 6005;
    }

    // compute smooth in index of traj points
    //double length_via2in;
    double path_vector_via2in[3];
    double length_via2in;
    getMoveLPathVector(via.pose_target.position, path_cache.cache[path_cache.smooth_in_index].pose.position, path_vector_via2in, length_via2in);
    double length_out2via = getPointsDistance(path_cache.cache[0].pose.position, via.pose_target.position);    
    int traj_smooth_in_index = computeTrajSmoothInIndex(length_out2via, length_via2in);
    int path_index_in, path_index_out, path_index_end;
    // write: S_PathIndexStepOut2End, S_PathIndexStepIn2Out, S_PathIndexStepIn2End
    computePathIndexStepVia2End(path_cache, via, length_via2in, path_index_in, path_index_out, path_index_end);   
    // write: S_Path0~S_Path8[0...path_index_in]
    generatePathPointVia2In(path_cache, via, length_via2in, path_vector_via2in, path_index_in);
    int path_index_array_in2end[25];
    // read: S_PathIndexStepOut2End, S_PathIndexStepIn2Out, S_PathIndexStepIn2End
    // write: S_Path0~S_Path8[0...path_index_out...path_index_end]
    generatePathPointIn2End(path_cache, path_index_in, path_index_out, path_index_end, path_index_array_in2end);

    int path_index_array_size_in2end = path_index_end - path_index_in + 1;
    //if(path_index_out == path_index_end)
    if(path_cache.target.cnt < -DOUBLE_ACCURACY)
    {
        traj_cache.smooth_out_index = -1;
    }
    else
    {
        traj_cache.smooth_out_index = traj_smooth_in_index + (path_index_out - path_index_in) - 1;
    }

    int time_vector_size_via2end;
    double cmd_vel = path_cache.target.vel * vel_ratio;   // command velocity
    // write: S_TrajT
    generatePathVia2EndTimeVector(cmd_vel, path_cache, length_via2in, path_index_in, path_index_out, path_index_end, time_vector_size_via2end);

    // init state {P,V,A}
    stack[S_TmpVector3_1 + 1] = 0;
    stack[S_TmpVector3_1 + 2] = 0;
    // end state {P,V,A}
    stack[S_TmpVector3_2 + 1] = 0;
    stack[S_TmpVector3_2 + 2] = 0;

    int i;
    int path_base = S_Path0;
    int traj_index_via;
    if(path_index_in == 0)
    {
        traj_index_via = traj_smooth_in_index;
    }
    else
    {
        traj_index_via = (traj_smooth_in_index - path_index_in - 1);
    } 
    int traj_index_end = traj_smooth_in_index + (path_index_end - path_index_in + 1);

    // make sure the in point of traj point locate at (S_TrajP0 + traj_smooth_in_index)
    int traj_base_via2end = S_TrajP0 + traj_index_via;  
    for(i = 0; i < model.link_num; ++i)
    {
        stack[S_TmpVector3_1] = stack[path_base];
        stack[S_TmpVector3_2] = stack[path_base + path_index_end];
        // read: S_Path0 ~ S_Path8, S_TrajT
        // write: S_TrajP0 ~ S_TrajP8, S_TrajV0 ~ S_TrajV8, S_TrajA0 ~ S_TrajA8
        updateTrajPVA(path_base, &stack[S_TmpVector3_1], &stack[S_TmpVector3_2], path_index_end + 1, traj_base_via2end, S_TrajT);
        path_base += 25;
        traj_base_via2end += 150;
    }

    int traj_piece_num_via2end = traj_index_end - traj_index_via;
    // read: S_TrajV0 ~ S_TrajV8, S_TrajA0 ~ S_TrajA8
    // write: S_TrajPieceJ0 ~ S_TrajPieceJ5, S_TrajPieceA0 ~ S_TrajPieceA5, S_TrajPieceV0 ~ S_TrajPieceV5
    //        S_ConstraintJointPosA0 ~ S_ConstraintJointPosA5, S_ConstraintJointNegA0 ~ S_ConstraintJointNegA5
    generatePieceVectors(traj_index_via, traj_piece_num_via2end, S_TrajT, vel_ratio, acc_ratio);
    double rescale_factor_via2end;
    // read: S_TrajPieceV0 ~ S_TrajPieceV8, S_TrajPieceA0 ~ S_TrajPieceA8, S_TrajPieceJ0 ~ S_TrajPieceJ8      
    // write: S_TrajRescaleFactor
    generateRescaleFactorVector(traj_piece_num_via2end, rescale_factor_via2end);  //rescale_factor_via2end = 1;
//std::cout<<"rescale_factor_via2end = "<<rescale_factor_via2end<<std::endl;
    // write: S_TrajT
    adjustTrajT(traj_piece_num_via2end, rescale_factor_via2end);
    adjustPVA(traj_index_via, traj_index_end, rescale_factor_via2end);

    // get out and in point state
    int out_point_base = S_OutPointState0;
    int in_point_base = S_InPointState0;
    int traj_base_in = S_TrajP0 + traj_smooth_in_index;
    double rescale_factor_via2end_square = rescale_factor_via2end * rescale_factor_via2end;
    for(i = 0; i < model.link_num; ++i)
    {
        stack[out_point_base] = start_state.angle[i];   // P
        stack[out_point_base + 1] = start_state.omega[i];// * rescale_factor_via2end;   // V
        stack[out_point_base + 2] = start_state.alpha[i];   // A
        stack[in_point_base] = stack[traj_base_in];            // P
        stack[in_point_base + 1] = stack[traj_base_in + 50];// * rescale_factor_via2end;   // V
        stack[in_point_base + 2] = stack[traj_base_in + 100];  // A
        in_point_base += 3;
        out_point_base += 3;
        traj_base_in += 150;
    }

//std::cout<<"out_state: "<<stack[S_OutPointState1]<<" "<<stack[S_OutPointState1 + 1]<<" "<<stack[S_OutPointState1 + 2]<<std::endl;
//std::cout<<"in_state: "<<stack[S_InPointState1]<<" "<<stack[S_InPointState1 + 1]<<" "<<stack[S_InPointState1 + 2]<<std::endl;    
    int path_array_size_out2in;
    int path_index_array_out2in[25];
    // write: S_Path0~S_Path8
    generatePathPointOut2In(path_cache, traj_smooth_in_index, path_array_size_out2in, path_index_array_out2in);

    int time_vector_size_out2in;
    double p1[9], pn_1[9];
    for(int i=0; i<model.link_num; ++i)
    {
        p1[i] = path_cache.cache[path_index_array_out2in[1]].joint[i];
        pn_1[i] = path_cache.cache[path_index_array_out2in[path_array_size_out2in - 2]].joint[i];
    }
    // write: S_TrajT_Smooth
    generatePathOut2InTimeVector(cmd_vel, length_out2via, length_via2in, path_array_size_out2in, time_vector_size_out2in, p1, pn_1);

    path_base = S_Path0;
    int traj_base_out2in = S_TrajP0; 
    int out_point_state_base = S_OutPointState0;
    int in_point_state_base = S_InPointState0;    
    
    for(i = 0; i < model.link_num; ++i)
    {
        stack[S_TmpVector3_1] = stack[path_base];
        // read: S_Path0 ~ S_Path8, S_TrajT
        // write: S_TrajP0 ~ S_TrajP8, S_TrajV0 ~ S_TrajV8, S_TrajA0 ~ S_TrajA8
        updateTrajPVA(path_base, &stack[out_point_state_base], &stack[in_point_state_base], path_array_size_out2in, traj_base_out2in, S_TrajT_Smooth);
        path_base += 25;
        traj_base_out2in += 150;
        in_point_state_base += 3;
        out_point_state_base += 3;
    }

    // read: S_TrajV0 ~ S_TrajV8, S_TrajA0 ~ S_TrajA8
    // write: S_TrajPieceJ0 ~ S_TrajPieceJ5, S_TrajPieceA0 ~ S_TrajPieceA5, S_TrajPieceV0 ~ S_TrajPieceV5
    //        S_ConstraintJointPosA0 ~ S_ConstraintJointPosA5, S_ConstraintJointNegA0 ~ S_ConstraintJointNegA5
    //generatePieceVectors(0, time_vector_size_out2in, S_TrajT_Smooth, vel_ratio, acc_ratio);

    double rescale_factor_out2in;
    // read: S_TrajPieceV0 ~ S_TrajPieceV8, S_TrajPieceA0 ~ S_TrajPieceA8, S_TrajPieceJ0 ~ S_TrajPieceJ8      
    // write: S_TrajRescaleFactor
    generateRescaleFactorVector(time_vector_size_out2in, rescale_factor_out2in);    rescale_factor_out2in = 1;   
    stack[S_TrajRescaleFactor] = rescale_factor_out2in;
    adjustTrajTSmooth(traj_smooth_in_index, rescale_factor_out2in);
    adjustPVA2(traj_smooth_in_index, rescale_factor_out2in); 

    // read: S_TrajT, S_TrajT_Smooth
    // write: S_TrajRescaleT
    generateRescaleVectorSmooth(time_vector_size_out2in, traj_piece_num_via2end, traj_smooth_in_index - traj_index_via);

/*for(int i=0; i<=traj_index_end; ++i)
    std::cout<<i<<" "<<stack[S_TrajP1 + i]<<" "<<stack[S_TrajV1 + i]<<" "<<stack[S_TrajA1 + i]<<std::endl;

for(int i=0; i<traj_index_end; ++i)
    std::cout<<i<<" "<<stack[S_TrajRescaleT + i]<<std::endl;*/

    // read: S_TrajRescaleT
    // write: S_TrajAbsoluteT    
    //generateAbsoluteVector(traj_index_end);
/*for(int i=0; i<=traj_index_end; ++i)
    std::cout<<i<<" "<<stack[S_TrajAbsoluteT + i]<<std::endl;*/
    // read: S_TrajRescaleFactor, S_TrajT
    // write: S_TrajRescaleT
    //generateRescaleVector(0, time_vector_size_out2in, traj_index_via, traj_piece_num_via2end);

    int time_vector_size_total = traj_index_end;
    // read: S_TrajRescaleT, S_TrajRescaleFactor, S_TrajP0 ~ S_TrajP8, S_TrajA0 ~ S_TrajA8
    // write: S_TrajAbsoluteT, S_TrajCoeffJ0A3 ~ S_TrajCoeffJ8A3, S_TrajCoeffJ0A2 ~ S_TrajCoeffJ8A2, 
    //        S_TrajCoeffJ0A1 ~ S_TrajCoeffJ8A1, S_TrajCoeffJ0A0 ~ S_TrajCoeffJ8A0
    generateCoeff(time_vector_size_total);
    //adjustCoeff(traj_smooth_in_index);

    generateTrajCacheSmooth(traj_cache, time_vector_size_total, 
                            path_index_array_out2in, path_array_size_out2in,
                            path_index_array_in2end, path_index_array_size_in2end);
    return 0;
}

ErrorCode planPauseTrajectory(const PathCache &path_cache, 
                                    const JointState &start_state, 
                                    double acc_ratio, 
                                    TrajectoryCache &traj_cache, 
                                    int &path_stop_index)
{
    return 0;
}

void getMoveLPathVector(const Point& start_point, const Point& end_point, double* path_vector, double& path_length)
{
    path_vector[0] = end_point.x - start_point.x; 
    path_vector[1] = end_point.y - start_point.y;
    path_vector[2] = end_point.z - start_point.z;
    path_length = getVector3Norm(path_vector);
    path_vector[0] /= path_length;
    path_vector[1] /= path_length;
    path_vector[2] /= path_length;
}

double getPointsDistance(const Point& point1, const Point& point2)
{
    stack[S_TmpVector3_1] = point1.x - point2.x; 
    stack[S_TmpVector3_1 + 1] = point1.y - point2.y; 
    stack[S_TmpVector3_1 + 2] = point1.z - point2.z; 
    return getVector3Norm(&stack[S_TmpVector3_1]);
}

void getMoveLPathPoint(const Point& start_point, double* path_vector, double distance, Point& target_point)
{
    target_point.x = start_point.x + path_vector[0] * distance;
    target_point.y = start_point.y + path_vector[1] * distance;
    target_point.z = start_point.z + path_vector[2] * distance;
}

void getMoveEulerToQuatern(const Euler& euler, double* quatern)
{
    stack[S_TmpVector3_1] = euler.a;
    stack[S_TmpVector3_1 + 1] = euler.b;
    stack[S_TmpVector3_1 + 2] = euler.c;
    getEulerToQuatern(&stack[S_TmpVector3_1], quatern);
}

void getMovePointToVector3(const Point& point, double* pos_vector)
{
    pos_vector[0] = point.x;
    pos_vector[1] = point.y;
    pos_vector[2] = point.z;
}

void getQuaternPoint(double* start_quatern, double* end_quartern, double angle, double angle_distance_to_start, Quaternion& target_quatern)
{
    if(fabs(angle) > segment_alg_param.angle_valve)
    {
        // slerp interpolation
        stack[S_TmpDouble_1] = sin(angle);
        stack[S_TmpDouble_2] = sin((1 - angle_distance_to_start) * angle) / stack[S_TmpDouble_1];   // a(t)
        stack[S_TmpDouble_3] = sin(angle_distance_to_start * angle) / stack[S_TmpDouble_1]; // b(t)
        // target = a(t) * start + b(t) * end
        target_quatern.x = stack[S_TmpDouble_2] * start_quatern[0] + stack[S_TmpDouble_3] * end_quartern[0];
        target_quatern.y = stack[S_TmpDouble_2] * start_quatern[1] + stack[S_TmpDouble_3] * end_quartern[1];
        target_quatern.z = stack[S_TmpDouble_2] * start_quatern[2] + stack[S_TmpDouble_3] * end_quartern[2];
        target_quatern.w = stack[S_TmpDouble_2] * start_quatern[3] + stack[S_TmpDouble_3] * end_quartern[3];
    }
    else
    {
        stack[S_TmpDouble_1] = 1 - angle_distance_to_start;
        // target = (1-t) * start + t * end
        target_quatern.x = stack[S_TmpDouble_1] * start_quatern[0] + angle_distance_to_start * end_quartern[0];
        target_quatern.y = stack[S_TmpDouble_1] * start_quatern[1] + angle_distance_to_start * end_quartern[1];
        target_quatern.z = stack[S_TmpDouble_1] * start_quatern[2] + angle_distance_to_start * end_quartern[2];
        target_quatern.w = stack[S_TmpDouble_1] * start_quatern[3] + angle_distance_to_start * end_quartern[3];
    }

    stack[S_TmpDouble_4] = sqrt(target_quatern.x * target_quatern.x
                                + target_quatern.y * target_quatern.y
                                + target_quatern.z * target_quatern.z
                                + target_quatern.w * target_quatern.w);
    target_quatern.x = target_quatern.x / stack[S_TmpDouble_4];
    target_quatern.y = target_quatern.y / stack[S_TmpDouble_4];
    target_quatern.z = target_quatern.z / stack[S_TmpDouble_4];
    target_quatern.w = target_quatern.w / stack[S_TmpDouble_4];
}

void generateMoveJPathPoint(const PathCache &path_cache, int* path_index_array, int& path_index_array_size)
{
    int i, j;
    // find max delta joint of all axes
    int max_delta_joint_index = S_DeltaJointVector;
    int cache_length_minus_1 = path_cache.cache_length - 1;
    for(i = 0; i < model.link_num; ++i)
    {
        stack[S_DeltaJointVector + i] = fabs(path_cache.cache[cache_length_minus_1].joint[i] - path_cache.cache[0].joint[i]);
        if(stack[S_DeltaJointVector + i] > stack[max_delta_joint_index])
        {
            max_delta_joint_index = S_DeltaJointVector + i;
        }
    }
    // compute path point step necessary for trajecotry computation
    double path_count_ideal = stack[max_delta_joint_index] * stack[S_PathCountFactorJoint];
    int path_index_array_size_minus_1;
    if(path_count_ideal > 20)
    {
        path_index_array_size_minus_1 = 20;
    }
    else if(path_count_ideal < 3)
    {
        path_index_array_size_minus_1 = 3;
    }
    else
    {
        path_index_array_size_minus_1 = ceil(path_count_ideal);
    }
    path_index_array_size = path_index_array_size_minus_1 + 1;  // include start point
    stack[S_PathIndexStep] = cache_length_minus_1 / (double)path_index_array_size_minus_1;

    // select point
    double path_index_ideal = 0;
    int path_address;
    // first point must be the start point on path
    path_index_array[0] = 0;
    path_address = S_Path0;
    for(i = 0; i < model.link_num; ++i)
    {
        stack[path_address] = path_cache.cache[0].joint[i];
        path_address += 25;
    }
    // middle points
    for(i = 1; i < path_index_array_size_minus_1; ++i)
    {
        path_index_ideal += stack[S_PathIndexStep];
        path_index_array[i] = round(path_index_ideal) - 1;
        path_address = S_Path0 + i;
        for(j = 0; j < model.link_num; ++j)
        {
            stack[path_address] = path_cache.cache[path_index_array[i]].joint[j];
            path_address += 25;
        }        
    }
    // last point must be the end point on path
    path_index_array[path_index_array_size_minus_1] = cache_length_minus_1;
    path_address = S_Path0 + path_index_array_size_minus_1;
    for(j = 0; j < model.link_num; ++j)
    {
        stack[path_address] = path_cache.cache[cache_length_minus_1].joint[j];
        path_address += 25;
    }
}

void generateMoveJTimeVector(double vel_ratio, int path_size, int& time_vector_size)
{
    // stack[S_JointDeltaBase] should have been filled in generateMoveJPathPoint()
    int i;
    // compute max time of the path for all joints
    double path_time;
    double max_path_time = 0;
    int path_index_of_max_path_time = 0;
    for(i = 0; i < model.link_num; ++i)
    {
        path_time = stack[S_DeltaJointVector + i] / (vel_ratio * stack[S_ConstraintJointVelMax + i]);
        if(path_time > max_path_time)
        {
            max_path_time = path_time;
            path_index_of_max_path_time = i;
        }
    }
    int path_piece_num = path_size - 1;
    double time_duration = max_path_time / path_piece_num;

    stack[S_TrajT] = time_duration * segment_alg_param.time_factor_1;
    stack[S_TrajT + 1] = time_duration * segment_alg_param.time_factor_2;
    for(i = 2; i < path_piece_num; ++i)
    {
        stack[S_TrajT + i] = time_duration;
    }
    stack[S_TrajT + path_piece_num] = time_duration * segment_alg_param.time_factor_3;
    stack[S_TrajT + path_size] = time_duration * segment_alg_param.time_factor_4;  
    time_vector_size = path_size + 1;
}

void generateMoveLPathPoint(const PathCache &path_cache, const JointState &start_state, 
                                    int* path_index_array, int& path_index_array_size,
                                    int& path_index_array_smooth_out_index, double& path_length_start2out, double& path_length_out2end)
{
    int i, j;
    int cache_length_minus_1 = path_cache.cache_length - 1;
    Joint ref_joint, result_joint;

    // compute total length of path
    double length_start2end = getPointsDistance(path_cache.cache[0].pose.position, path_cache.cache[path_cache.cache_length - 1].pose.position);
    if(path_cache.smooth_out_index == -1
        || path_cache.smooth_out_index == cache_length_minus_1)
    {
        // compute path point step necessary for trajecotry computation
        double path_count_ideal = length_start2end * stack[S_PathCountFactorCartesian];
        int path_index_array_size_minus_1;
        if(path_count_ideal > 20)
        {
            path_index_array_size_minus_1 = 20;
        }
        else if(path_count_ideal < 3)
        {
            path_index_array_size_minus_1 = 3;
        }
        else
        {
            path_index_array_size_minus_1 = ceil(path_count_ideal);
        }
        stack[S_PathIndexStep] = cache_length_minus_1 / (double)path_index_array_size_minus_1;
        path_index_array_size = path_index_array_size_minus_1 + 1;  // include start point 
        
        // select point
        double path_index_ideal = 0;
        // first point must be the start point on path
        path_index_array[0] = 0;
        // middle points
        for(i = 1; i < path_index_array_size_minus_1; ++i)
        {
            path_index_ideal += stack[S_PathIndexStep];
            path_index_array[i] = round(path_index_ideal) - 1;     
        }
        // last point must be the end point on path
        path_index_array[path_index_array_size_minus_1] = cache_length_minus_1;          
        path_index_array_smooth_out_index = -1;
        path_length_start2out = length_start2end;
        path_length_out2end = 0;
    }
    else
    {
        double length_start2out = getPointsDistance(path_cache.cache[0].pose.position, path_cache.cache[path_cache.smooth_out_index].pose.position);
        double length_out2end = length_start2end - length_start2out;       
        double path_count_ideal_start2end = length_start2end * stack[S_PathCountFactorCartesian];
        double path_count_ideal_start2out = length_start2out * stack[S_PathCountFactorCartesian];
        double path_count_ideal_out2end = length_out2end * stack[S_PathCountFactorCartesian];

        int path_index_array_start2out_minus_1;
        int path_index_array_out2end_minus_1;
        if(path_count_ideal_start2end > 19)
        {
            path_count_ideal_start2out = path_count_ideal_start2out * 19 / path_count_ideal_start2end;
            path_count_ideal_out2end = path_count_ideal_out2end * 19 / path_count_ideal_start2end;
        }

        if(path_count_ideal_start2out < 2)
        {
            path_index_array_start2out_minus_1 = 2;
        }
        else
        {
            path_index_array_start2out_minus_1 = ceil(path_count_ideal_start2out);
        }
        stack[S_PathIndexStepStart2Out] = path_cache.smooth_out_index / (double)path_index_array_start2out_minus_1;
        int path_index_array_start2out = path_index_array_start2out_minus_1 + 1;
                
        if(path_count_ideal_out2end < 2)
        {
            path_index_array_out2end_minus_1 = 2;
        }
        else
        {
            path_index_array_out2end_minus_1 = ceil(path_count_ideal_out2end);
        }                
        stack[S_PathIndexStepOut2End] = (cache_length_minus_1 - path_cache.smooth_out_index) / (double)path_index_array_out2end_minus_1;        
        int path_index_array_out2end = path_index_array_out2end_minus_1 + 1;
        path_index_array_size = path_index_array_start2out + path_index_array_out2end - 1;
        int path_index_array_size_minus_1 = path_index_array_size - 1;

        // select point
        double path_index_ideal = 0;
        // first point must be the start point on path
        path_index_array[0] = 0;
        // points from second point to smooth out point
        for(i = 1; i < path_index_array_start2out_minus_1; ++i)
        {
            path_index_ideal += stack[S_PathIndexStepStart2Out];
            path_index_array[i] = round(path_index_ideal) - 1;     
        }
        // this point must be the out point on path
        path_index_array[path_index_array_start2out_minus_1] = path_cache.smooth_out_index;
        path_index_array_smooth_out_index = path_index_array_start2out_minus_1;
        path_index_ideal = path_cache.smooth_out_index;
        // points from smooth out point to end point
        for(i = path_index_array_start2out; i < path_index_array_size_minus_1; ++i)
        {
            path_index_ideal += stack[S_PathIndexStepOut2End];
            path_index_array[i] = round(path_index_ideal) - 1;
        }
        // last point must be the end point on path
        path_index_array[path_index_array_size_minus_1] = cache_length_minus_1; 
        path_length_start2out = length_start2out;
        path_length_out2end = length_out2end;
    }
 
    // compute inverse kinematics
    // first point is the start point
    int path_address = S_Path0;
    for(i = 0; i < model.link_num; ++i)
    {
        stack[path_address] = start_state.angle[i];
        path_address += 25;
    }
    
    // the remaining points
    ref_joint = start_state.angle;
    for(i = 1; i < path_index_array_size; ++i)
    {                                   
        ref_joint = result_joint;
        path_address = S_Path0 + i;
        for(j = 0; j < model.link_num; ++j)
        {
            stack[path_address] = path_cache.cache[path_index_array[i]].joint[j];
            path_address += 25;
        }
    }   
}

void generateMoveLTimeVector(double vel, int path_size, int path_index_array_smooth_out_index, 
                                    double path_length_start2out, double path_length_out2end, int& time_vector_size)
{
    int i;    
    if(path_index_array_smooth_out_index == -1) // no smooth out point
    {
        double time_span_start2end = path_length_start2out / vel;   
        int path_piece_start2end = path_size - 1;
        double time_duration = time_span_start2end / path_piece_start2end;

        stack[S_TrajT] = time_duration * segment_alg_param.time_factor_1;
        stack[S_TrajT + 1] = time_duration * segment_alg_param.time_factor_2;
        for(i = 2; i < path_piece_start2end; ++i)
        {
            stack[S_TrajT + i] = time_duration;
        }
        stack[S_TrajT + path_piece_start2end] = time_duration * segment_alg_param.time_factor_3;
        stack[S_TrajT + path_size] = time_duration * segment_alg_param.time_factor_4;       
    }
    else
    {
        double time_span_start2out = path_length_start2out / vel;
        double time_span_out2end = path_length_out2end / vel;
        int path_piece_start2out = path_index_array_smooth_out_index;        
        int path_piece_out2end = path_size - path_index_array_smooth_out_index - 1;
        double time_duration_start2out = time_span_start2out / path_piece_start2out;
        double time_duration_out2end = time_span_out2end / path_piece_out2end;
        int path_size_minus_1 = path_size - 1;

        stack[S_TrajT] = time_duration_start2out * segment_alg_param.time_factor_1;
        stack[S_TrajT + 1] = time_duration_start2out * segment_alg_param.time_factor_2;
        for(i = 2; i <= path_piece_start2out; ++i)
        {
            stack[S_TrajT + i] = time_duration_start2out;
        }
        for(i = path_piece_start2out + 1; i < path_size_minus_1; ++i)
        {
            stack[S_TrajT + i] = time_duration_out2end;
        }
        stack[S_TrajT + path_size_minus_1] = time_duration_out2end * segment_alg_param.time_factor_3;
        stack[S_TrajT + path_size] = time_duration_out2end * segment_alg_param.time_factor_4;
    }
    time_vector_size = path_size + 1; 
}

int computeTrajSmoothInIndex(double length_out2via, double length_via2in)
{
    double path_count_ideal_transition = (length_out2via + length_via2in) * stack[S_PathCountFactorCartesian];
    if(path_count_ideal_transition > 20)
    {
        return 22;
    }
    else if(path_count_ideal_transition < 3)
    {
        return 5;
    }
    else
    {
        return (ceil(path_count_ideal_transition) + 2);
    }
}

void computePathIndexStepVia2End(const PathCache &path_cache, const MotionTarget &via, double length_via2in,
                                    int& path_index_in, int& path_index_out, int& path_index_end)
{
    int cache_length_minus_1 = path_cache.cache_length - 1;
    double length_in2end = getPointsDistance(path_cache.cache[path_cache.smooth_in_index].pose.position, path_cache.cache[cache_length_minus_1].pose.position);
    double path_count_ideal_via2in = length_via2in * stack[S_PathCountFactorCartesian];
    double path_count_ideal_in2end = length_in2end * stack[S_PathCountFactorCartesian];
    double path_count_ideal_via2end = path_count_ideal_via2in + path_count_ideal_in2end;
    if(path_count_ideal_via2end > 20)
    {
        path_count_ideal_via2in = path_count_ideal_via2in * 20 / path_count_ideal_via2end;
        path_count_ideal_in2end = path_count_ideal_in2end * 20 / path_count_ideal_via2end;
    }   
    if(path_count_ideal_via2in < DOUBLE_ACCURACY)
    {
        path_index_in = 0;
    }
    else
    {
        path_index_in = ceil(path_count_ideal_via2in);
    }

    if(path_cache.smooth_out_index == -1
        || path_cache.smooth_out_index == cache_length_minus_1)
    {       
        if(path_count_ideal_in2end < 3)
        {
            path_index_end = path_index_in + 3;
        }
        else
        {
            path_index_end = path_index_in + ceil(path_count_ideal_in2end);
        }
        path_index_out = path_index_end;

        if(path_index_in != 0)
        {
            stack[S_PathIndexStepIn2End] = (cache_length_minus_1 - path_cache.smooth_in_index) / (double)(path_index_end - path_index_in);
        }
        else
        {
            stack[S_PathIndexStepIn2End] = 0;
        }
    }
    else
    {
        double length_in2out = getPointsDistance(path_cache.cache[path_cache.smooth_in_index].pose.position, path_cache.cache[path_cache.smooth_out_index].pose.position);
        double length_out2end = length_in2end - length_in2out;
        double path_count_ideal_in2out = length_in2out * stack[S_PathCountFactorCartesian];
        double path_count_ideal_out2end = length_out2end * stack[S_PathCountFactorCartesian];
        if(path_count_ideal_via2end > 20)
        {
            path_count_ideal_in2out = path_count_ideal_in2out * 20 / path_count_ideal_via2end;
            path_count_ideal_out2end = path_count_ideal_out2end * 20 / path_count_ideal_via2end;
        }
        
        if(path_count_ideal_in2out < DOUBLE_ACCURACY)   // in and out is the same point
        {
            path_index_out = path_index_in;
        }
        else
        {
            path_index_out = path_index_in + ceil(path_count_ideal_in2out);
        }

        if(path_count_ideal_out2end < DOUBLE_ACCURACY)  // out and end is the same point
        {
            path_index_end = path_index_out;
        }
        else
        {
            path_index_end = path_index_out + ceil(path_count_ideal_out2end);
        }

        if(path_index_in != path_index_out)
        {
            stack[S_PathIndexStepIn2Out] = (path_cache.smooth_out_index - path_cache.smooth_in_index) / (double)(path_index_out - path_index_in);
        }
        else
        {
            stack[S_PathIndexStepIn2Out] = 0;
        }
        if(path_index_out != path_index_end)
        {
            stack[S_PathIndexStepOut2End] = (cache_length_minus_1 - path_cache.smooth_out_index) / (double)(path_index_end - path_index_out);
        }
        else
        {
            stack[S_PathIndexStepOut2End] = 0;
        }
        // FIXME: what if path_index_in = path_index_out = path_index_end ?
    }
}

void generatePathPointVia2In(const PathCache &path_cache, const MotionTarget &via, double length_via2in,
                                    double* path_vector_via2in, int path_index_in)
{
    int i, j;
    // get start joint
    Joint via_joint;
    segment_alg_param.kinematics_ptr->inverseKinematicsInUser(via.pose_target, path_cache.cache[0].joint, via_joint);
    int path_address = S_Path0;
    for(i = 0; i < model.link_num; ++i)
    {
        stack[path_address] = via_joint[i];
        path_address += 25;
    }

    // via and in points are the same point
    if(path_index_in == 0)
    {
        return;
    }

    // select traj point from via to in, not include the via point
    Pose point_array[10];   // point_array[0] is not used
    // compute length step    
    double length_step = length_via2in / path_index_in;
    double distance_to_via = 0;
    // compute angle step    
    double via_quatern[4], in_quatern[4];
    getMoveEulerToQuatern(via.pose_target.orientation, via_quatern);
    in_quatern[0] = path_cache.cache[path_cache.smooth_in_index].pose.orientation.x;
    in_quatern[1] = path_cache.cache[path_cache.smooth_in_index].pose.orientation.y;
    in_quatern[2] = path_cache.cache[path_cache.smooth_in_index].pose.orientation.z;
    in_quatern[3] = path_cache.cache[path_cache.smooth_in_index].pose.orientation.w;
    double angle_via2in = getQuaternsIntersectionAngle(via_quatern, in_quatern);
    double angle_step = angle_via2in / path_index_in;
    double angle_to_via = 0;
    // compute traj point from 1 to (path_index_in - 1)
    for(i = 1; i < path_index_in; ++i)
    {
        distance_to_via += length_step;
        angle_to_via += angle_step;
        getMoveLPathPoint(via.pose_target.position, path_vector_via2in, distance_to_via, point_array[i].position);
        getQuaternPoint(via_quatern, in_quatern, angle_via2in, angle_to_via, point_array[i].orientation);
    }
    // traj point should be on the 'in point' 
    point_array[path_index_in].position = path_cache.cache[path_cache.smooth_in_index].pose.position;
    point_array[path_index_in].orientation = path_cache.cache[path_cache.smooth_in_index].pose.orientation;    

    Joint result_joint;
    Joint ref_joint = via_joint;
    // IK point_array to S_Path0~S_Path8
    for(i = 1; i <= path_index_in; ++i)
    {
        segment_alg_param.kinematics_ptr->inverseKinematicsInUser(point_array[i], ref_joint, result_joint);
        path_address = S_Path0 + i;
        for(j = 0; j < model.link_num; ++j)
        {
            stack[path_address] = result_joint[j];
            path_address += 25;
        }
        ref_joint = result_joint;
    }
}

void generatePathPointIn2End(const PathCache &path_cache, int path_index_in, int path_index_out, int path_index_end, int* path_index_array_in2end)
{
    int i, j;
    double path_index_ideal = path_cache.smooth_in_index;
    int path_index_real;
    int path_address;
    int cache_length_minus_1 = path_cache.cache_length - 1;
    
    path_index_array_in2end[0] = path_cache.smooth_in_index;
    int path_index_array_in2end_index = 1;
    if(path_cache.smooth_out_index == -1
        || path_cache.smooth_out_index == cache_length_minus_1)   // without smooth
    {
        // path_index_out == path_index_end, stack[S_PathIndexStepIn2End] is valid
        // from (path_index_in + 1) to (path_index_end - 1)
        for(i = path_index_in + 1; i < path_index_end; ++i)
        {
            path_index_ideal += stack[S_PathIndexStepIn2End];
            path_index_real = round(path_index_ideal);
            path_index_array_in2end[path_index_array_in2end_index] = path_index_real;
            ++path_index_array_in2end_index;
            path_address = S_Path0 + i;
            for(j = 0; j < model.link_num; ++j)
            {
                stack[path_address] = path_cache.cache[path_index_real].joint[j];
                path_address += 25;
            }
        }
        // path_index_end should be the same as path end point
        path_index_array_in2end[path_index_array_in2end_index] = cache_length_minus_1;
        path_address = S_Path0 + path_index_end;
        for(i = 0; i < model.link_num; ++i)
        {
            stack[path_address] = path_cache.cache[cache_length_minus_1].joint[i];
            
            path_address += 25;
        }        
    }
    else    // with smooth
    {
        // path_index_out != path_index_end, stack[S_PathIndexStepIn2Out] & stack[S_PathIndexStepOut2End] are valid
        // from (path_index_in + 1) to (path_index_out - 1)
        for(i = path_index_in + 1; i < path_index_out; ++i)
        {
            path_index_ideal += stack[S_PathIndexStepIn2Out];
            path_index_real = round(path_index_ideal);
            path_index_array_in2end[path_index_array_in2end_index] = path_index_real;
            ++path_index_array_in2end_index;
            path_address = S_Path0 + i;
            for(j = 0; j < model.link_num; ++j)
            {
                stack[path_address] = path_cache.cache[path_index_real].joint[j];
                path_address += 25;
            }
        }        
        // path_index_out should be the same as path out point
        path_index_ideal = path_cache.smooth_out_index;
        path_index_array_in2end[path_index_array_in2end_index] = path_cache.smooth_out_index;
        ++path_index_array_in2end_index;
        path_address = S_Path0 + path_index_out;      
        for(i = 0; i < model.link_num; ++i)
        {
            stack[path_address] = path_cache.cache[path_cache.smooth_out_index].joint[i];
            path_address += 25;
        }
        // from (path_index_out + 1) to (path_index_end - 1)
        for(i = path_index_out + 1; i < path_index_end; ++i)
        {
            path_index_ideal += stack[S_PathIndexStepOut2End];
            path_index_real = round(path_index_ideal);
            path_index_array_in2end[path_index_array_in2end_index] = path_index_real;
            ++path_index_array_in2end_index;
            path_address = S_Path0 + i;
            for(j = 0; j < model.link_num; ++j)
            {
                stack[path_address] = path_cache.cache[path_index_real].joint[j];
                path_address += 25;
            }
        }
        // path_index_end should be the same as path end point        
        path_index_array_in2end[path_index_array_in2end_index] = cache_length_minus_1;
        path_address = S_Path0 + path_index_end;       
        for(i = 0; i < model.link_num; ++i)
        {
            stack[path_address] = path_cache.cache[cache_length_minus_1].joint[i];
            path_address += 25;
        }        
    }
}

void generatePathVia2EndTimeVector(double vel, const PathCache &path_cache, double length_via2in, 
                                            int path_index_in, int path_index_out, int path_index_end, int& time_vector_size_via2end)
{
    int i;
    int cache_length_minus_1 = path_cache.cache_length - 1;
    time_vector_size_via2end = path_index_end + 2;
    int time_vector_size_via2end_minus_1 = time_vector_size_via2end - 1;
    int time_vector_current_index = 0;
    if(path_index_in != 0)
    {
        double time_span_via2in = length_via2in / vel;    
        double time_duration_via2in = time_span_via2in / path_index_in;
        stack[S_TrajT] = time_duration_via2in * segment_alg_param.time_factor_1;
        stack[S_TrajT + 1] = time_duration_via2in * segment_alg_param.time_factor_2;
        for(i = 1; i < path_index_in; ++i)
        {
            stack[S_TrajT + i + 1] = time_duration_via2in;
        }
        time_vector_current_index = path_index_in + 1;
    }

    double length_in2end = getPointsDistance(path_cache.cache[path_cache.smooth_in_index].pose.position, path_cache.cache[cache_length_minus_1].pose.position);
    if(path_cache.smooth_out_index == -1
        || path_cache.smooth_out_index == cache_length_minus_1)
    {
        double time_span_in2end = length_in2end / vel;
        double time_duration_in2end = time_span_in2end / (path_index_end - path_index_in);
        for(i = time_vector_current_index; i < path_index_end; ++i)
        {
            stack[S_TrajT + i] = time_duration_in2end;
        }
        stack[S_TrajT + path_index_end] = time_duration_in2end * segment_alg_param.time_factor_3;
        stack[S_TrajT + time_vector_size_via2end_minus_1] = time_duration_in2end * segment_alg_param.time_factor_4;
    }
    else
    {
        double length_in2out = getPointsDistance(path_cache.cache[path_cache.smooth_in_index].pose.position, path_cache.cache[path_cache.smooth_out_index].pose.position);
        double length_out2end = length_in2end - length_in2out;
        double time_span_in2out = length_in2out / vel;
        double time_span_out2end = length_out2end / vel;
        double time_duration_in2out = time_span_in2out / (path_index_out - path_index_in);
        double time_duration_out2end = time_span_out2end / (path_index_end - path_index_out);
        for(i = time_vector_current_index; i <= path_index_out; ++i)
        {
            stack[S_TrajT + i] = time_duration_in2out;
        }
        for(i = path_index_out + 1; i < time_vector_size_via2end_minus_1; ++i)
        {
            stack[S_TrajT + i] = time_duration_out2end;
        }
        stack[S_TrajT + path_index_end] = time_duration_out2end * segment_alg_param.time_factor_3;
        stack[S_TrajT + time_vector_size_via2end_minus_1] = time_duration_out2end * segment_alg_param.time_factor_4; 
    }
}

void generatePathPointOut2In(const PathCache &path_cache, int traj_smooth_in_index, int& path_array_size, int* path_index_array_out2in)
{
    int i, j;
    path_array_size = traj_smooth_in_index - 1;
    int path_array_size_minus_1 = path_array_size - 1;
    stack[S_PathIndexStepOut2In] = path_cache.smooth_in_index / (double)path_array_size_minus_1;

    // fisrt traj point
    int path_address = S_Path0;
    for(i = 0; i < model.link_num; ++i)
    {
        stack[path_address] = path_cache.cache[0].joint[i];
        path_address += 25;
    }

    // middle traj points
    double path_index_ideal = 0;
    int path_index_real;
    path_index_array_out2in[0] = 0;
    int path_index_array_out2in_index = 1;
    for(i = 1; i < path_array_size_minus_1; ++i)
    {
        path_index_ideal += stack[S_PathIndexStepOut2In];
        path_index_real = round(path_index_ideal);
        
        path_index_array_out2in[path_index_array_out2in_index] = path_index_real;
        ++path_index_array_out2in_index;
        path_address = S_Path0 + i;
        for(j = 0; j < model.link_num; ++j)
        {
            stack[path_address] = path_cache.cache[path_index_real].joint[j];
            path_address += 25;
        }
    }

    // last traj points
    path_address = S_Path0 + path_array_size_minus_1;
    path_index_array_out2in[path_index_array_out2in_index] = path_cache.smooth_in_index;
    for(i = 0; i < model.link_num; ++i)
    {
        stack[path_address] = path_cache.cache[path_cache.smooth_in_index].joint[i];
        path_address += 25;
    }
}

void generatePathOut2InTimeVector(double cmd_vel, double length_out2via, double length_via2in, int path_array_size_out2in,int& time_vector_size_out2in, double p1[9], double pn_1[9])
{
    time_vector_size_out2in = path_array_size_out2in + 1;
    int path_array_size_out2in_minus_1 = path_array_size_out2in - 1;
//------------------------    
    double time_span_out2in = (length_out2via + length_via2in) / cmd_vel;    
    double time_duration_out2in = time_span_out2in / path_array_size_out2in_minus_1;
    stack[S_TrajT_Smooth] = time_duration_out2in * segment_alg_param.time_factor_1;
    stack[S_TrajT_Smooth + 1] = time_duration_out2in * segment_alg_param.time_factor_2;
    for(int i = 2; i < path_array_size_out2in_minus_1; ++i)
    {
        stack[S_TrajT_Smooth + i] = time_duration_out2in;
    }
    stack[S_TrajT_Smooth + path_array_size_out2in_minus_1] = time_duration_out2in * segment_alg_param.time_factor_3;
    stack[S_TrajT_Smooth + path_array_size_out2in] = time_duration_out2in * segment_alg_param.time_factor_4;
//std::cout<<"stack[S_TrajT_Smooth] org = "<<stack[S_TrajT_Smooth]<<std::endl;    
//-------------------------  
    double delta_t1, delta_tn_1;
    double v_avg1, v_avgn_1;
    int out_point_address = S_OutPointState0;
    int in_point_address = S_InPointState0;
    double delta_t1_max = 0, delta_tn_1_max = 0;
    for(int i = 0; i < model.link_num; ++i)
    {
        v_avg1 = (p1[i] - stack[out_point_address]) / time_duration_out2in;
        v_avgn_1 = (stack[in_point_address] - pn_1[i]) / time_duration_out2in;
        delta_t1 = (p1[i] - stack[out_point_address]) / (v_avg1 + stack[out_point_address + 1]);
        delta_tn_1 = (stack[in_point_address] - pn_1[i]) / (v_avgn_1 + stack[in_point_address + 1]);
//std::cout<<i<<"delta_t1 = "<<delta_t1<<" delta_tn_1 = "<<delta_tn_1<<std::endl;
        if(delta_t1 > delta_t1_max)
        {
            delta_t1_max = delta_t1;
        }
        if(delta_tn_1 > delta_tn_1_max)
        {
            delta_tn_1_max = delta_tn_1;
        }
        out_point_address += 3;
        in_point_address += 3;
    }

    stack[S_TrajT_Smooth] = delta_t1_max;
    stack[S_TrajT_Smooth + 1] = stack[S_TrajT_Smooth];
    stack[S_TrajT_Smooth + path_array_size_out2in_minus_1] = delta_tn_1_max;
    stack[S_TrajT_Smooth + path_array_size_out2in] = stack[S_TrajT_Smooth + path_array_size_out2in_minus_1];
    //std::cout<<"v_avg1 = "<<v_avg1<<" v_avgn_1 = "<<v_avgn_1<<std::endl;
    //std::cout<<"delta_t1 = "<<delta_t1<<" delta_tn_1 = "<<delta_tn_1<<std::endl;
//std::cout<<"stack[S_TrajT_Smooth] new = "<<stack[S_TrajT_Smooth]<<"stack[S_TrajT_Smooth + path_array_size_out2in_minus_1] = "<<stack[S_TrajT_Smooth + path_array_size_out2in_minus_1]<<std::endl;     
}

void generatePieceVectors(int traj_offset, int traj_piece_num, int t_base, double vel_ratio, double acc_ratio)
{
    int traj_piece_j_address = S_TrajPieceJ0;
    int traj_piece_v_address = S_TrajPieceV0;
    int traj_piece_a_address = S_TrajPieceA0;
    int traj_a_address = S_TrajA0 + traj_offset;
    int traj_v_address = S_TrajV0 + traj_offset;
    for(int i = 0; i <model.link_num; ++i)
    {
        for(int j = 0; j < traj_piece_num; ++j)
        {
            // compute jerk
            stack[traj_piece_j_address + j] = fabs((stack[traj_a_address + j + 1] - stack[traj_a_address + j]) / stack[t_base + j]) 
                                                / (stack[S_ConstraintJointJerkMax + i] * segment_alg_param.jerk_ratio);

            // compute vel
            double acc_product = stack[traj_a_address + j] * stack[traj_a_address + j + 1];
            if(acc_product >= 0)
            {
                if(fabs(stack[traj_a_address + j]) > fabs(stack[traj_a_address + j + 1]))
                {
                    stack[traj_piece_v_address + j] = fabs(stack[traj_v_address + j]) / (stack[S_ConstraintJointVelMax + i] * vel_ratio);
                }
                else
                {
                    stack[traj_piece_v_address + j] = fabs(stack[traj_v_address + j + 1]) / (stack[S_ConstraintJointVelMax + i] * vel_ratio);
                }
            }
            else
            {
                if(stack[traj_a_address + j] >= 0)
                {
                    stack[traj_piece_v_address + j] = (stack[traj_v_address + j] 
                                                    + stack[traj_a_address + j] * stack[traj_a_address + j] * stack[t_base + j] / (2 * (stack[traj_a_address + j] - stack[traj_a_address + j + 1])))
                                                    / (stack[S_ConstraintJointVelMax + i] * vel_ratio);
                }
                else
                {
                    stack[traj_piece_v_address + j] = (stack[traj_v_address + j] 
                                                    - stack[traj_a_address + j] * stack[traj_a_address + j] * stack[t_base + j] / (2 * (stack[traj_a_address + j + 1] - stack[traj_a_address + j])))
                                                    / (stack[S_ConstraintJointVelMax + i] * vel_ratio);
                }
            }
        }
        traj_piece_j_address += 25;
        traj_piece_v_address += 25;
        traj_piece_a_address += 25;
        traj_a_address += 150;
        traj_v_address += 150;
    }

    // compute acc
    float joint[6], omega[6], alpha[2][6];
    for(int i = 0; i < traj_piece_num; ++i)
    {    
        // quite mass
        if(segment_alg_param.is_fake_dynamics)
        {
            stack[S_ConstraintJointPosA0 + i] = 100;
            stack[S_ConstraintJointPosA1 + i] = 100;
            stack[S_ConstraintJointPosA2 + i] = 100;
            stack[S_ConstraintJointPosA3 + i] = 100;
            stack[S_ConstraintJointPosA4 + i] = 100;
            stack[S_ConstraintJointPosA5 + i] = 100;
            stack[S_ConstraintJointNegA0 + i] = -100;
            stack[S_ConstraintJointNegA1 + i] = -100;
            stack[S_ConstraintJointNegA2 + i] = -100;
            stack[S_ConstraintJointNegA3 + i] = -100;
            stack[S_ConstraintJointNegA4 + i] = -100;
            stack[S_ConstraintJointNegA5 + i] = -100;

        }
        else
        {
            joint[0] = (float)stack[S_TrajP0 + i + 1];
            joint[1] = (float)stack[S_TrajP1 + i + 1];
            joint[2] = (float)stack[S_TrajP2 + i + 1];
            joint[3] = (float)stack[S_TrajP3 + i + 1];
            joint[4] = (float)stack[S_TrajP4 + i + 1];
            joint[5] = (float)stack[S_TrajP5 + i + 1];
            omega[0] = (float)stack[S_TrajV0 + i + 1];
            omega[1] = (float)stack[S_TrajV1 + i + 1];
            omega[2] = (float)stack[S_TrajV2 + i + 1];
            omega[3] = (float)stack[S_TrajV3 + i + 1];
            omega[4] = (float)stack[S_TrajV4 + i + 1];
            omega[5] = (float)stack[S_TrajV5 + i + 1];
            segment_alg_param.dynamics_ptr->computeAccMax(joint, omega, alpha);
            stack[S_ConstraintJointPosA0 + i] = (double)alpha[0][0];
            stack[S_ConstraintJointPosA1 + i] = (double)alpha[0][1];
            stack[S_ConstraintJointPosA2 + i] = (double)alpha[0][2];
            stack[S_ConstraintJointPosA3 + i] = (double)alpha[0][3];
            stack[S_ConstraintJointPosA4 + i] = (double)alpha[0][4];
            stack[S_ConstraintJointPosA5 + i] = (double)alpha[0][5];
            stack[S_ConstraintJointNegA0 + i] = (double)alpha[1][0];
            stack[S_ConstraintJointNegA1 + i] = (double)alpha[1][1];
            stack[S_ConstraintJointNegA2 + i] = (double)alpha[1][2];
            stack[S_ConstraintJointNegA3 + i] = (double)alpha[1][3];
            stack[S_ConstraintJointNegA4 + i] = (double)alpha[1][4];
            stack[S_ConstraintJointNegA5 + i] = (double)alpha[1][5];
        }
        // it is ugly...ummm
        if(stack[S_TrajA0 + i + 1] >= 0)
        {
            if(stack[S_TrajA0 + i + 1] > stack[S_ConstraintJointPosA0 + i])
            {
                stack[S_TrajPieceA0 + i] = stack[S_TrajA0 + i + 1] / (stack[S_ConstraintJointPosA0 + i] * acc_ratio);
            }
            else
            {
                stack[S_TrajPieceA0 + i] = 0;
            }
        }
        else
        {
            if(stack[S_TrajA0 + i + 1] <= stack[S_ConstraintJointNegA0 + i])
            {
                stack[S_TrajPieceA0 + i] = stack[S_TrajA0 + i + 1] / (stack[S_ConstraintJointNegA0 + i] * acc_ratio);
            }
            else
            {
                stack[S_TrajPieceA0 + i] = 0;
            }
        }

        if(stack[S_TrajA1 + i + 1] >= 0)
        {
            if(stack[S_TrajA1 + i + 1] >= stack[S_ConstraintJointPosA1 + i])
            {
                stack[S_TrajPieceA1 + i] = stack[S_TrajA1 + i + 1] / (stack[S_ConstraintJointPosA1 + i] * acc_ratio);
            }
            else
            {
                stack[S_TrajPieceA1 + i] = 0;
            }
        }
        else
        {
            if(stack[S_TrajA1 + i + 1] <= stack[S_ConstraintJointNegA1 + i])
            {
                stack[S_TrajPieceA1 + i] = stack[S_TrajA1 + i + 1] / (stack[S_ConstraintJointNegA1 + i] * acc_ratio);
            }
            else
            {
                stack[S_TrajPieceA1 + i] = 0;
            }
        }   

        if(stack[S_TrajA2 + i + 1] >= 0)
        {
            if(stack[S_TrajA2 + i + 1] >= stack[S_ConstraintJointPosA2 + i])
            {
                stack[S_TrajPieceA2 + i] = stack[S_TrajA2 + i + 1] / (stack[S_ConstraintJointPosA2 + i] * acc_ratio);
            }
            else
            {
                stack[S_TrajPieceA2 + i] = 0;
            }
        }
        else
        {
            if(stack[S_TrajA2 + i + 1] <= stack[S_ConstraintJointNegA2 + i])
            {
                stack[S_TrajPieceA2 + i] = stack[S_TrajA2 + i + 1] / (stack[S_ConstraintJointNegA2 + i] * acc_ratio);
            }
            else
            {
                stack[S_TrajPieceA2 + i] = 0;
            }
        }     

        if(stack[S_TrajA3 + i + 1] >= 0)
        {
            if(stack[S_TrajA3 + i + 1] >= stack[S_ConstraintJointPosA3 + i])
            {
                stack[S_TrajPieceA3 + i] = stack[S_TrajA3 + i + 1] / (stack[S_ConstraintJointPosA3 + i] * acc_ratio);
            }
            else
            {
                stack[S_TrajPieceA3 + i] = 0;
            }
        }
        else
        {
            if(stack[S_TrajA3 + i + 1] <= stack[S_ConstraintJointNegA3 + i])
            {
                stack[S_TrajPieceA3 + i] = stack[S_TrajA3 + i + 1] / (stack[S_ConstraintJointNegA3 + i] * acc_ratio);
            }
            else
            {
                stack[S_TrajPieceA3 + i] = 0;
            }
        }   

        if(stack[S_TrajA4 + i + 1] >= 0)
        {
            if(stack[S_TrajA4 + i + 1] >= stack[S_ConstraintJointPosA4 + i])
            {
                stack[S_TrajPieceA4 + i] = stack[S_TrajA4 + i + 1] / (stack[S_ConstraintJointPosA4 + i] * acc_ratio);
            }
            else
            {
                stack[S_TrajPieceA4 + i] = 0;
            }
        }
        else
        {
            if(stack[S_TrajA4 + i + 1] <= stack[S_ConstraintJointNegA4 + i])
            {
                stack[S_TrajPieceA4 + i] = stack[S_TrajA4 + i + 1] / (stack[S_ConstraintJointNegA4 + i] * acc_ratio);
            }
            else
            {
                stack[S_TrajPieceA4 + i] = 0;
            }
        }    

        if(stack[S_TrajA5 + i + 1] >= 0)
        {
            if(stack[S_TrajA5 + i] >= stack[S_ConstraintJointPosA5 + i])
            {
                stack[S_TrajPieceA5 + i] = stack[S_TrajA5 + i + 1] / (stack[S_ConstraintJointPosA5 + i] * acc_ratio);
            }
            else
            {
                stack[S_TrajPieceA5 + i] = 0;
            }
        }
        else
        {
            if(stack[S_TrajA5 + i + 1] <= stack[S_ConstraintJointNegA5 + i])
            {
                stack[S_TrajPieceA5 + i] = stack[S_TrajA5 + i + 1] / (stack[S_ConstraintJointNegA5 + i] * acc_ratio);
            }
            else
            {
                stack[S_TrajPieceA5 + i] = 0;
            }
        }         
    }           
}

double getMaxTrajPiece(int base_address, int index)
{
    double max_value = 0;
    int piece_address = base_address + index;
    for(int i=0; i < model.link_num; ++i)
    {
        if(stack[piece_address] > max_value)
        {
            max_value = stack[piece_address];
        }
        piece_address += 25;
    }
    return max_value;
}

double getMax(double value0, double value1, double value2, double value3)
{
    double tmp1, tmp2;
    tmp1 = (value0 > value1 ? value0 : value1);
    tmp2 = (value2 > value3 ? value2 : value3);
    return (tmp1 > tmp2 ? tmp1 : tmp2);
}

void generateRescaleFactorVector(int time_vector_size, double& rescale_factor)
{
    double max_v_factor, max_a_factor, max_j_factor;
    int base_v_address = S_TrajPieceV0;
    int base_a_address = S_TrajPieceA0;
    int base_j_address = S_TrajPieceJ0;
    rescale_factor = 1;
    for(int i = 0; i < time_vector_size; ++i)
    {
        stack[S_TmpDouble_1] = getMaxTrajPiece(base_v_address, i);
        stack[S_TmpDouble_2] = getMaxTrajPiece(base_a_address, i);
        stack[S_TmpDouble_3] = getMaxTrajPiece(base_j_address, i);
        stack[S_TmpDouble_4] = getMax(1, stack[S_TmpDouble_1], sqrt(stack[S_TmpDouble_2]), pow(stack[S_TmpDouble_3], 0.333333));
        if(stack[S_TmpDouble_4] > rescale_factor)
        {
            rescale_factor = stack[S_TmpDouble_4];
        }
    }

    if(rescale_factor > segment_alg_param.max_rescale_factor)
    {
        rescale_factor = segment_alg_param.max_rescale_factor;
    }
}

void generateRescaleVector(int time_vector_size)
{
    for(int i = 0; i < time_vector_size; ++i)
    {
        stack[S_TrajRescaleT + i] = stack[S_TrajT + i] * stack[S_TrajRescaleFactor];
    }
}

void adjustTrajT(int time_vector_size, double rescale_factor)
{
    for(int i = 0; i < time_vector_size; ++i)
    {
        stack[S_TrajT + i ] = rescale_factor * stack[S_TrajT + i ];
    }
}

void adjustTrajTSmooth(int time_vector_size, double rescale_factor)
{
    for(int i = 0; i < time_vector_size; ++i)
    {
        stack[S_TrajT_Smooth + i ] = rescale_factor * stack[S_TrajT_Smooth + i ];
    }
}

void adjustPVA(int traj_index_via, int traj_index_end, double rescale_factor_via2end)
{
    int i, j;
    int v_base, a_base;
    double rescale_factor_via2end_square = rescale_factor_via2end * rescale_factor_via2end;   
    for(i = traj_index_via; i <= traj_index_end; ++i)
    {
        v_base = S_TrajV0;
        a_base = S_TrajA0;
        for(j = 0; j < model.link_num; ++j)
        {
            stack[v_base + i] = stack[v_base + i] / rescale_factor_via2end;
            stack[a_base + i] = stack[a_base + i] / rescale_factor_via2end_square;
            v_base += 150;
            a_base += 150;
        }
    }
}

void adjustPVA2(int traj_index_in, double rescale_factor)
{
    int i, j;
    int v_base, a_base;
    double rescale_factor_square = rescale_factor * rescale_factor;
    
    for(i = 1; i < traj_index_in; ++i)
    {
        v_base = S_TrajV0;
        a_base = S_TrajA0;
        for(j = 0; j < model.link_num; ++j)
        {
            stack[v_base + i] = stack[v_base + i] / rescale_factor;
            stack[a_base + i] = stack[a_base + i] / rescale_factor_square;
            v_base += 150;
            a_base += 150;
        }
    }
}

void generateRescaleVectorSmooth(int time_vector_size_out2in, int time_vector_size_in2end, int time_vector_size_via2in)
{
    int i;
    for(i = 0; i < time_vector_size_out2in; ++i)
    {
        stack[S_TrajRescaleT + i] = stack[S_TrajT_Smooth + i];
    }

    for(i = 0; i < time_vector_size_in2end; ++i)
    {
        stack[S_TrajRescaleT + time_vector_size_out2in + i] = stack[S_TrajT + time_vector_size_via2in + i];
    }    
}

void generateAbsoluteVector(int time_vector_size)
{
    stack[S_TrajAbsoluteT] = 0;
    for(int i = 0; i < time_vector_size; ++i)
    {
        stack[S_TrajAbsoluteT + i + 1] = stack[S_TrajAbsoluteT + i] + stack[S_TrajRescaleT + i];
    }
}

void generateCoeff(int time_vector_size)
{
    int i, j;
    int coeff_a5 = S_TrajCoeffJ0A5;
    int coeff_a4 = S_TrajCoeffJ0A4;
    int coeff_a3 = S_TrajCoeffJ0A3;
    int coeff_a2 = S_TrajCoeffJ0A2;
    int coeff_a1 = S_TrajCoeffJ0A1;
    int coeff_a0 = S_TrajCoeffJ0A0;
    int traj_p = S_TrajP0;
    int traj_v = S_TrajV0;
    int traj_a = S_TrajA0;
 
    // compute coeff
    for(i = 0; i < model.link_num; ++i)
    {
        for(j = 0; j < time_vector_size; ++j)
        {
            stack[S_TmpDouble_2] = stack[S_TrajRescaleT + j] * stack[S_TrajRescaleT + j];   // T^2
            stack[S_TmpDouble_3] = stack[S_TmpDouble_2] * stack[S_TrajRescaleT + j];    // T^3
            stack[S_TmpDouble_4] = stack[S_TmpDouble_3] * stack[S_TrajRescaleT + j];    // T^4
            stack[S_TmpDouble_5] = stack[S_TmpDouble_4] * stack[S_TrajRescaleT + j];    // T^5 
            stack[coeff_a5 + j] = (12 * (stack[traj_p + j + 1] - stack[traj_p + j])
                                    - 6 * (stack[traj_v + j] + stack[traj_v + j + 1]) * stack[S_TrajRescaleT + j]
                                    - (stack[traj_a + j] - stack[traj_a + j + 1]) * stack[S_TmpDouble_2])
                                    / (2 * stack[S_TmpDouble_5]);
            stack[coeff_a4 + j] = (30 * (stack[traj_p + j] - stack[traj_p + j + 1])
                                    + (16 * stack[traj_v + j] + 14 * stack[traj_v + j + 1]) * stack[S_TrajRescaleT + j]
                                    + (3 * stack[traj_a + j] - 2 * stack[traj_a + j +1]) * stack[S_TmpDouble_2])
                                    / (2 * stack[S_TmpDouble_4]);
            stack[coeff_a3 + j] = (20 * (stack[traj_p + j + 1] - stack[traj_p + j]) 
                                    - 4 * (3 * stack[traj_v + j] + 2 * stack[traj_v + j + 1]) * stack[S_TrajRescaleT + j]
                                    + (stack[traj_a + j + 1] - 3 * stack[traj_a + j]) * stack[S_TmpDouble_2])
                                    / (2 * stack[S_TmpDouble_3]);
            stack[coeff_a2 + j] = stack[traj_a + j] / 2;
            stack[coeff_a1 + j] = stack[traj_v + j];
            stack[coeff_a0 + j] = stack[traj_p + j];
        }
        traj_p += 150;
        traj_v += 150;
        traj_a += 150;
        coeff_a5 += 300;
        coeff_a4 += 300;
        coeff_a3 += 300;
        coeff_a2 += 300;
        coeff_a1 += 300;
        coeff_a0 += 300;
    }
}

void adjustCoeff(int traj_smooth_in_index)
{
    int coeff_a3 = S_TrajCoeffJ0A3;
    int coeff_a2 = S_TrajCoeffJ0A2;
    int coeff_a1 = S_TrajCoeffJ0A1;
    int coeff_a0 = S_TrajCoeffJ0A0;
    int traj_p = S_TrajP0;
    int traj_v= S_TrajV0;
    double p0, p1, v0, v1, t;
    for(int i = 0; i < model.link_num; ++i)
    {
        stack[coeff_a0] = stack[traj_p];
        stack[coeff_a1] = stack[traj_v];
        stack[coeff_a2] = 3 * (stack[traj_p + 1] - stack[traj_p]) / (stack[S_TrajRescaleT] * stack[S_TrajRescaleT])
                                - (2 * stack[traj_v] + stack[traj_v + 1]) / stack[S_TrajRescaleT];
        stack[coeff_a3] = 2 * (stack[traj_p] - stack[traj_p + 1]) / (stack[S_TrajRescaleT] * stack[S_TrajRescaleT] * stack[S_TrajRescaleT])
                                + (stack[traj_v] + stack[traj_v + 1]) / (stack[S_TrajRescaleT] * stack[S_TrajRescaleT]);

        traj_p += 150;
        traj_v += 150;
        coeff_a3 += 200;
        coeff_a2 += 200;
        coeff_a1 += 200;
        coeff_a0 += 200;
    }
}

void generateTrajCache(TrajectoryCache &traj_cache, int time_vector_size, int* path_index_array, int path_index_array_size)
{
    int i;
    traj_cache.cache_length = time_vector_size;
    int time_vector_size_minus_1 = time_vector_size - 1;
    for(i = 0; i < time_vector_size; ++i)
    {
        traj_cache.cache[i].axis[0].data[0] = stack[S_TrajCoeffJ0A0 + i];
        traj_cache.cache[i].axis[0].data[1] = stack[S_TrajCoeffJ0A1 + i];
        traj_cache.cache[i].axis[0].data[2] = stack[S_TrajCoeffJ0A2 + i];
        traj_cache.cache[i].axis[0].data[3] = stack[S_TrajCoeffJ0A3 + i];
        traj_cache.cache[i].axis[0].data[4] = stack[S_TrajCoeffJ0A4 + i];
        traj_cache.cache[i].axis[0].data[5] = stack[S_TrajCoeffJ0A5 + i];
        traj_cache.cache[i].axis[1].data[0] = stack[S_TrajCoeffJ1A0 + i];
        traj_cache.cache[i].axis[1].data[1] = stack[S_TrajCoeffJ1A1 + i];
        traj_cache.cache[i].axis[1].data[2] = stack[S_TrajCoeffJ1A2 + i];
        traj_cache.cache[i].axis[1].data[3] = stack[S_TrajCoeffJ1A3 + i];
        traj_cache.cache[i].axis[1].data[4] = stack[S_TrajCoeffJ1A4 + i];   
        traj_cache.cache[i].axis[1].data[5] = stack[S_TrajCoeffJ1A5 + i];   
        traj_cache.cache[i].axis[2].data[0] = stack[S_TrajCoeffJ2A0 + i];
        traj_cache.cache[i].axis[2].data[1] = stack[S_TrajCoeffJ2A1 + i];
        traj_cache.cache[i].axis[2].data[2] = stack[S_TrajCoeffJ2A2 + i];
        traj_cache.cache[i].axis[2].data[3] = stack[S_TrajCoeffJ2A3 + i];
        traj_cache.cache[i].axis[2].data[4] = stack[S_TrajCoeffJ2A4 + i];
        traj_cache.cache[i].axis[2].data[5] = stack[S_TrajCoeffJ2A5 + i];
        traj_cache.cache[i].axis[3].data[0] = stack[S_TrajCoeffJ3A0 + i];
        traj_cache.cache[i].axis[3].data[1] = stack[S_TrajCoeffJ3A1 + i];
        traj_cache.cache[i].axis[3].data[2] = stack[S_TrajCoeffJ3A2 + i];
        traj_cache.cache[i].axis[3].data[3] = stack[S_TrajCoeffJ3A3 + i];
        traj_cache.cache[i].axis[3].data[4] = stack[S_TrajCoeffJ3A4 + i];
        traj_cache.cache[i].axis[3].data[5] = stack[S_TrajCoeffJ3A5 + i];
        traj_cache.cache[i].axis[4].data[0] = stack[S_TrajCoeffJ4A0 + i];
        traj_cache.cache[i].axis[4].data[1] = stack[S_TrajCoeffJ4A1 + i];
        traj_cache.cache[i].axis[4].data[2] = stack[S_TrajCoeffJ4A2 + i];
        traj_cache.cache[i].axis[4].data[3] = stack[S_TrajCoeffJ4A3 + i];
        traj_cache.cache[i].axis[4].data[4] = stack[S_TrajCoeffJ4A4 + i];
        traj_cache.cache[i].axis[4].data[5] = stack[S_TrajCoeffJ4A5 + i];
        traj_cache.cache[i].axis[5].data[0] = stack[S_TrajCoeffJ5A0 + i];
        traj_cache.cache[i].axis[5].data[1] = stack[S_TrajCoeffJ5A1 + i];
        traj_cache.cache[i].axis[5].data[2] = stack[S_TrajCoeffJ5A2 + i];
        traj_cache.cache[i].axis[5].data[3] = stack[S_TrajCoeffJ5A3 + i];
        traj_cache.cache[i].axis[5].data[4] = stack[S_TrajCoeffJ5A4 + i];
        traj_cache.cache[i].axis[5].data[5] = stack[S_TrajCoeffJ5A5 + i];
        traj_cache.cache[i].duration = stack[S_TrajRescaleT + i];
    }
    
    traj_cache.cache[0].index_in_path_cache = -1;
    for(i = 1; i < (time_vector_size_minus_1 - 1); ++i)
    {
        traj_cache.cache[i].index_in_path_cache = path_index_array[i];
    }
    traj_cache.cache[time_vector_size_minus_1 - 1].index_in_path_cache = -1;
    traj_cache.cache[time_vector_size_minus_1].index_in_path_cache = path_index_array[path_index_array_size - 1];
}

void generateTrajCacheSmooth(TrajectoryCache &traj_cache, int time_vector_size, 
                                    int* path_index_array_out2in, int path_index_array_size_out2in,
                                    int* path_index_array_in2end, int path_index_array_size_in2end)
{
    int i;
    traj_cache.cache_length = time_vector_size;
    int traj_cache_length_minus_1 = traj_cache.cache_length - 1;
    int time_vector_size_out2in = path_index_array_size_out2in + 1; // include two flexible point
    int time_vector_size_out2in_minus_1 = path_index_array_size_out2in;
    
    for(i = 0; i < traj_cache.cache_length; ++i)
    {
        traj_cache.cache[i].axis[0].data[0] = stack[S_TrajCoeffJ0A0 + i];
        traj_cache.cache[i].axis[0].data[1] = stack[S_TrajCoeffJ0A1 + i];
        traj_cache.cache[i].axis[0].data[2] = stack[S_TrajCoeffJ0A2 + i];
        traj_cache.cache[i].axis[0].data[3] = stack[S_TrajCoeffJ0A3 + i];
        traj_cache.cache[i].axis[0].data[4] = stack[S_TrajCoeffJ0A4 + i];
        traj_cache.cache[i].axis[0].data[5] = stack[S_TrajCoeffJ0A5 + i];
        traj_cache.cache[i].axis[1].data[0] = stack[S_TrajCoeffJ1A0 + i];
        traj_cache.cache[i].axis[1].data[1] = stack[S_TrajCoeffJ1A1 + i];
        traj_cache.cache[i].axis[1].data[2] = stack[S_TrajCoeffJ1A2 + i];
        traj_cache.cache[i].axis[1].data[3] = stack[S_TrajCoeffJ1A3 + i];
        traj_cache.cache[i].axis[1].data[4] = stack[S_TrajCoeffJ1A4 + i];   
        traj_cache.cache[i].axis[1].data[5] = stack[S_TrajCoeffJ1A5 + i];   
        traj_cache.cache[i].axis[2].data[0] = stack[S_TrajCoeffJ2A0 + i];
        traj_cache.cache[i].axis[2].data[1] = stack[S_TrajCoeffJ2A1 + i];
        traj_cache.cache[i].axis[2].data[2] = stack[S_TrajCoeffJ2A2 + i];
        traj_cache.cache[i].axis[2].data[3] = stack[S_TrajCoeffJ2A3 + i];
        traj_cache.cache[i].axis[2].data[4] = stack[S_TrajCoeffJ2A4 + i];
        traj_cache.cache[i].axis[2].data[5] = stack[S_TrajCoeffJ2A5 + i];
        traj_cache.cache[i].axis[3].data[0] = stack[S_TrajCoeffJ3A0 + i];
        traj_cache.cache[i].axis[3].data[1] = stack[S_TrajCoeffJ3A1 + i];
        traj_cache.cache[i].axis[3].data[2] = stack[S_TrajCoeffJ3A2 + i];
        traj_cache.cache[i].axis[3].data[3] = stack[S_TrajCoeffJ3A3 + i];
        traj_cache.cache[i].axis[3].data[4] = stack[S_TrajCoeffJ3A4 + i];
        traj_cache.cache[i].axis[3].data[5] = stack[S_TrajCoeffJ3A5 + i];
        traj_cache.cache[i].axis[4].data[0] = stack[S_TrajCoeffJ4A0 + i];
        traj_cache.cache[i].axis[4].data[1] = stack[S_TrajCoeffJ4A1 + i];
        traj_cache.cache[i].axis[4].data[2] = stack[S_TrajCoeffJ4A2 + i];
        traj_cache.cache[i].axis[4].data[3] = stack[S_TrajCoeffJ4A3 + i];
        traj_cache.cache[i].axis[4].data[4] = stack[S_TrajCoeffJ4A4 + i];
        traj_cache.cache[i].axis[4].data[5] = stack[S_TrajCoeffJ4A5 + i];
        traj_cache.cache[i].axis[5].data[0] = stack[S_TrajCoeffJ5A0 + i];
        traj_cache.cache[i].axis[5].data[1] = stack[S_TrajCoeffJ5A1 + i];
        traj_cache.cache[i].axis[5].data[2] = stack[S_TrajCoeffJ5A2 + i];
        traj_cache.cache[i].axis[5].data[3] = stack[S_TrajCoeffJ5A3 + i];
        traj_cache.cache[i].axis[5].data[4] = stack[S_TrajCoeffJ5A4 + i];
        traj_cache.cache[i].axis[5].data[5] = stack[S_TrajCoeffJ5A5 + i];
        traj_cache.cache[i].duration = stack[S_TrajRescaleT + i];        
    }

    traj_cache.cache[0].index_in_path_cache = -1;
    for(i = 1; i < (time_vector_size_out2in_minus_1 - 1); ++i)
    {
        traj_cache.cache[i].index_in_path_cache = path_index_array_out2in[i];
    }
    traj_cache.cache[time_vector_size_out2in_minus_1 - 1].index_in_path_cache = -1;
    traj_cache.cache[time_vector_size_out2in_minus_1].index_in_path_cache = path_index_array_out2in[path_index_array_size_out2in - 1];
    for(i = time_vector_size_out2in; i < (traj_cache_length_minus_1 - 1); ++i)
    {
        traj_cache.cache[i].index_in_path_cache = path_index_array_in2end[i - time_vector_size_out2in + 1];
    }
    traj_cache.cache[traj_cache_length_minus_1 - 1].index_in_path_cache = -1;
    traj_cache.cache[traj_cache_length_minus_1].index_in_path_cache = path_index_array_in2end[path_index_array_size_in2end - 1];
}

void printTraj(TrajectoryCache &traj_cache, int index, double time_step)
{
    double absolute_time_vector[50];
    absolute_time_vector[0] = 0;
    for(int i = 1; i < traj_cache.cache_length + 1; ++i)
    {
        absolute_time_vector[i] = absolute_time_vector[i - 1] + traj_cache.cache[i - 1].duration;
    }

    int segment_index;
    double cur_time = 0;
    double p_value, v_value, a_value;
    int a3_base = S_TrajCoeffJ0A3 + 200*index;
    int a2_base = S_TrajCoeffJ0A2 + 200*index;
    int a1_base = S_TrajCoeffJ0A1 + 200*index;
    int a0_base = S_TrajCoeffJ0A0 + 200*index;    
    while(cur_time < absolute_time_vector[traj_cache.cache_length])
    {
        for(segment_index = 0; segment_index < traj_cache.cache_length; ++segment_index)
        {
            if(cur_time <= absolute_time_vector[segment_index])
            {
                break;
            }
        }
        if(segment_index == 0)
        {
            segment_index = 1;
        }

        p_value = stack[a3_base + segment_index - 1] * cur_time * cur_time * cur_time
                  + stack[a2_base + segment_index - 1] * cur_time * cur_time
                  + stack[a1_base + segment_index - 1] * cur_time
                  + stack[a0_base + segment_index - 1];
        v_value = 3 * stack[a3_base + segment_index - 1] * cur_time * cur_time
                  + 2 * stack[a2_base + segment_index - 1] * cur_time
                  + stack[a1_base + segment_index - 1];
        a_value = 6 * stack[a3_base + segment_index - 1] * cur_time
                  + 2 * stack[a2_base + segment_index - 1];

        std::cout<<p_value<<"  "<<v_value<<"  "<<a_value<<std::endl;
        cur_time += time_step;
    }
}

void printTraj2(TrajectoryCache &traj_cache, int index, double time_step, int end_segment)
{
    double absolute_time_vector[50];
    absolute_time_vector[0] = 0;

    for(int i = 1; i < traj_cache.cache_length + 1; ++i)
    {
        absolute_time_vector[i] = absolute_time_vector[i - 1] + traj_cache.cache[i - 1].duration;
    }
  
    int segment_index;
    double cur_time = 0;
    double delta_time = 0;
    double p_value, v_value, a_value;
    int a3_base = S_TrajCoeffJ0A3 + 200*index;
    int a2_base = S_TrajCoeffJ0A2 + 200*index;
    int a1_base = S_TrajCoeffJ0A1 + 200*index;
    int a0_base = S_TrajCoeffJ0A0 + 200*index;    
    while(cur_time < absolute_time_vector[end_segment])
    {
        for(segment_index = end_segment - 1; segment_index >= 0; --segment_index)
        {
            if(cur_time >= absolute_time_vector[segment_index])
            {
                break;
            }
        }

        delta_time = cur_time - absolute_time_vector[segment_index];
        p_value = traj_cache.cache[segment_index].axis[index].data[5] * delta_time * delta_time * delta_time * delta_time * delta_time
                  + traj_cache.cache[segment_index].axis[index].data[4] * delta_time * delta_time * delta_time * delta_time
                  + traj_cache.cache[segment_index].axis[index].data[3] * delta_time * delta_time * delta_time
                  + traj_cache.cache[segment_index].axis[index].data[2] * delta_time * delta_time
                  + traj_cache.cache[segment_index].axis[index].data[1] * delta_time
                  + traj_cache.cache[segment_index].axis[index].data[0];
        v_value = 5 * traj_cache.cache[segment_index].axis[index].data[5] * delta_time * delta_time * delta_time * delta_time
                  + 4 * traj_cache.cache[segment_index].axis[index].data[4] * delta_time * delta_time * delta_time
                  + 3 * traj_cache.cache[segment_index].axis[index].data[3] * delta_time * delta_time
                  + 2 * traj_cache.cache[segment_index].axis[index].data[2] * delta_time
                  + traj_cache.cache[segment_index].axis[index].data[1];
        a_value = 20 * traj_cache.cache[segment_index].axis[index].data[5] * delta_time * delta_time * delta_time
                  + 12 * traj_cache.cache[segment_index].axis[index].data[4] * delta_time * delta_time
                  + 6 * traj_cache.cache[segment_index].axis[index].data[3] * delta_time
                  + 2 * traj_cache.cache[segment_index].axis[index].data[2];

        std::cout<<segment_index<<" "<<cur_time<<" "<<p_value<<"  "<<v_value<<"  "<<a_value<<std::endl;
        cur_time += time_step;
    }


}

void printAllTraj(TrajectoryCache &traj_cache, double time_step)
{
    double absolute_time_vector[50];
    absolute_time_vector[0] = 0;
    for(int i = 1; i < traj_cache.cache_length + 1; ++i)
    {
        absolute_time_vector[i] = absolute_time_vector[i - 1] + traj_cache.cache[i - 1].duration;
    }

    int segment_index;
    double cur_time = 0;
    double p0_value, v0_value, a0_value;
    double p1_value, v1_value, a1_value;
    double p2_value, v2_value, a2_value;
    double p3_value, v3_value, a3_value;
    double p4_value, v4_value, a4_value;
    double p5_value, v5_value, a5_value;
  
    while(cur_time < absolute_time_vector[traj_cache.cache_length])
    {
        for(segment_index = 0; segment_index < traj_cache.cache_length; ++segment_index)
        {
            if(cur_time <= absolute_time_vector[segment_index])
            {
                break;
            }
        }
        if(segment_index == 0)
        {
            segment_index = 1;
        }

        p0_value = stack[S_TrajCoeffJ0A3 + segment_index - 1] * cur_time * cur_time * cur_time
                  + stack[S_TrajCoeffJ0A2 + segment_index - 1] * cur_time * cur_time
                  + stack[S_TrajCoeffJ0A1 + segment_index - 1] * cur_time
                  + stack[S_TrajCoeffJ0A0 + segment_index - 1];
        v0_value = 3 * stack[S_TrajCoeffJ0A3 + segment_index - 1] * cur_time * cur_time
                  + 2 * stack[S_TrajCoeffJ0A2 + segment_index - 1] * cur_time
                  + stack[S_TrajCoeffJ0A1 + segment_index - 1];
        a0_value = 6 * stack[S_TrajCoeffJ0A3 + segment_index - 1] * cur_time
                  + 2 * stack[S_TrajCoeffJ0A2 + segment_index - 1];

        p1_value = stack[S_TrajCoeffJ1A3 + segment_index - 1] * cur_time * cur_time * cur_time
                    + stack[S_TrajCoeffJ1A2 + segment_index - 1] * cur_time * cur_time
                    + stack[S_TrajCoeffJ1A1 + segment_index - 1] * cur_time
                    + stack[S_TrajCoeffJ1A0 + segment_index - 1];
        v1_value = 3 * stack[S_TrajCoeffJ1A3 + segment_index - 1] * cur_time * cur_time
                    + 2 * stack[S_TrajCoeffJ1A2 + segment_index - 1] * cur_time
                    + stack[S_TrajCoeffJ1A1 + segment_index - 1];
        a1_value = 6 * stack[S_TrajCoeffJ1A3 + segment_index - 1] * cur_time
                    + 2 * stack[S_TrajCoeffJ1A2 + segment_index - 1];

        p2_value = stack[S_TrajCoeffJ2A3 + segment_index - 1] * cur_time * cur_time * cur_time
                    + stack[S_TrajCoeffJ2A2 + segment_index - 1] * cur_time * cur_time
                    + stack[S_TrajCoeffJ2A1 + segment_index - 1] * cur_time
                    + stack[S_TrajCoeffJ2A0 + segment_index - 1];
        v2_value = 3 * stack[S_TrajCoeffJ2A3 + segment_index - 1] * cur_time * cur_time
                    + 2 * stack[S_TrajCoeffJ2A2 + segment_index - 1] * cur_time
                    + stack[S_TrajCoeffJ2A1 + segment_index - 1];
        a2_value = 6 * stack[S_TrajCoeffJ2A3 + segment_index - 1] * cur_time
                    + 2 * stack[S_TrajCoeffJ2A2 + segment_index - 1];

        p3_value = stack[S_TrajCoeffJ3A3 + segment_index - 1] * cur_time * cur_time * cur_time
                    + stack[S_TrajCoeffJ3A2 + segment_index - 1] * cur_time * cur_time
                    + stack[S_TrajCoeffJ3A1 + segment_index - 1] * cur_time
                    + stack[S_TrajCoeffJ3A0 + segment_index - 1];
        v3_value = 3 * stack[S_TrajCoeffJ3A3 + segment_index - 1] * cur_time * cur_time
                    + 2 * stack[S_TrajCoeffJ3A2 + segment_index - 1] * cur_time
                    + stack[S_TrajCoeffJ3A1 + segment_index - 1];
        a3_value = 6 * stack[S_TrajCoeffJ3A3 + segment_index - 1] * cur_time
                    + 2 * stack[S_TrajCoeffJ3A2 + segment_index - 1];

        p4_value = stack[S_TrajCoeffJ4A3 + segment_index - 1] * cur_time * cur_time * cur_time
                    + stack[S_TrajCoeffJ4A2 + segment_index - 1] * cur_time * cur_time
                    + stack[S_TrajCoeffJ4A1 + segment_index - 1] * cur_time
                    + stack[S_TrajCoeffJ4A0 + segment_index - 1];
        v4_value = 3 * stack[S_TrajCoeffJ4A3 + segment_index - 1] * cur_time * cur_time
                    + 2 * stack[S_TrajCoeffJ4A2 + segment_index - 1] * cur_time
                    + stack[S_TrajCoeffJ4A1 + segment_index - 1];
        a4_value = 6 * stack[S_TrajCoeffJ4A3 + segment_index - 1] * cur_time
                    + 2 * stack[S_TrajCoeffJ4A2 + segment_index - 1];

        p5_value = stack[S_TrajCoeffJ5A3 + segment_index - 1] * cur_time * cur_time * cur_time
                    + stack[S_TrajCoeffJ5A2 + segment_index - 1] * cur_time * cur_time
                    + stack[S_TrajCoeffJ5A1 + segment_index - 1] * cur_time
                    + stack[S_TrajCoeffJ5A0 + segment_index - 1];
        v5_value = 3 * stack[S_TrajCoeffJ5A3 + segment_index - 1] * cur_time * cur_time
                    + 2 * stack[S_TrajCoeffJ5A2 + segment_index - 1] * cur_time
                    + stack[S_TrajCoeffJ5A1 + segment_index - 1];
        a5_value = 6 * stack[S_TrajCoeffJ5A3 + segment_index - 1] * cur_time
                    + 2 * stack[S_TrajCoeffJ5A2 + segment_index - 1];
        //std::cout<<p0_value<<"  "<<p1_value<<"  "<<p2_value<<"  "<<p3_value<<"  "<<p4_value<<"  "<<p5_value<<std::endl;
        std::cout<<v0_value<<"  "<<v1_value<<"  "<<v2_value<<"  "<<v3_value<<"  "<<v4_value<<"  "<<v5_value<<std::endl;
        //std::cout<<a0_value<<"  "<<a1_value<<"  "<<a2_value<<"  "<<a3_value<<"  "<<a4_value<<"  "<<a5_value<<std::endl;
        cur_time += time_step;
    }

}

