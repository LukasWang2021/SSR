/*************************************************************************
	> File Name: motion_plan_basic_function.h
	> Author: 
	> Mail: 
	> Created Time: 2017年12月07日 星期四 16时41分13秒
 ************************************************************************/

#ifndef _MOTION_PLAN_BASIC_FUNCTION_H
#define _MOTION_PLAN_BASIC_FUNCTION_H

#include <math.h>
#include <fst_datatype.h>

#define MINIMUM_E6      0.000001
#define MINIMUM_E9      0.000000001
#define MINIMUM_E12     0.000000000001

namespace fst_algorithm
{

//------------------------------------------------------------------------------
// Function:    SIGN
// Summary: To judge the sign of a number.
// In:      x   -> number to judge
// Out:     None
// Return:  1   -> x > 0
//          0   -> x = 0
//          -1  -> x < 0
//------------------------------------------------------------------------------
static inline int SIGN(double x) {return x > MINIMUM_E9 ? 1 : (x < -MINIMUM_E9 ? -1 : 0);}

//------------------------------------------------------------------------------
// Function:    POLAR
// Summary: To judge the sign of a number.
// In:      x   -> number to judge
// Out:     None
// Return:  1   -> x >= 0
//          -1  -> x < 0
//------------------------------------------------------------------------------
static inline int POLAR(double x) {return x < 0 ? -1 : 1;}

//------------------------------------------------------------------------------
// Function:    MIN
// Summary: To get the smaller one of two numbers.
// In:      a   -> first number
//          b   -> second number
// Out:     None
// Return:  a   -> a < b
//          b   -> a > b
//------------------------------------------------------------------------------
static inline double MIN(double a, double b) {return (a < b) ? a : b;}

//------------------------------------------------------------------------------
// Function:    MAX
// Summary: To get the larger one of two numbers.
// In:      a   -> first number
//          b   -> second number
// Out:     None
// Return:  a   -> a > b
//          b   -> a < b
//------------------------------------------------------------------------------
static inline double MAX(double a, double b) {return (a > b) ? a : b;}

//------------------------------------------------------------------------------
// Function:    assignVec3
// Summary: Copy a vector to another one.
// In:      source      -> source vector
//          destination -> destination vector
// Out:     None
// Return:  None
//------------------------------------------------------------------------------
static inline void assignVec3(const double source[3], double destination[3])
{
    destination[0] = source[0];
    destination[1] = source[1];
    destination[2] = source[2];
}

//------------------------------------------------------------------------------
// Function:    assignVec4
// Summary: Copy a vector to another one.
// In:      source      -> source vector
//          destination -> destination vector
// Out:     None
// Return:  None
//------------------------------------------------------------------------------
static inline void assignVec4(const double source[4], double destination[4])
{
    destination[0] = source[0];
    destination[1] = source[1];
    destination[2] = source[2];
    destination[3] = source[3];
}
    
//------------------------------------------------------------------------------
// Function:    addVec2Vec
// Summary: Add two vectors, and put the result to another vector.
// In:      a1  -> first vector
//          a2  -> second vector
// Out:     res -> result vector
// Return:  None
//------------------------------------------------------------------------------
static inline void addVec2Vec(const double a1[3], const double a2[3], double res[3])
{
    res[0] = a1[0] + a2[0];
    res[1] = a1[1] + a2[1];
    res[2] = a1[2] + a2[2];
}
    
//------------------------------------------------------------------------------
// Function:    addVec2Vec
// Summary: Add two vectors, and put the result to the first vector.
// In:      a1  -> first vector
//          a2  -> second vector
// Out:     a1  -> result vector
// Return:  None
//------------------------------------------------------------------------------
static inline void addVec2Vec(double a1[3], const double a2[3])
{
    a1[0] += a2[0];
    a1[1] += a2[1];
    a1[2] += a2[2];
}
    
//------------------------------------------------------------------------------
// Function:    subVec2Vec
// Summary: First vector sub second one, and put the result to another vector.
// In:      s1  -> first vector
//          s2  -> second vector
// Out:     res -> result vector
// Return:  None
//------------------------------------------------------------------------------
static inline void subVec2Vec(const double s1[3], const double s2[3], double res[3])
{
    res[0] = s1[0] - s2[0];
    res[1] = s1[1] - s2[1];
    res[2] = s1[2] - s2[2];
}

//------------------------------------------------------------------------------
// Function:    subVec2Vec
// Summary: First vector sub second one, and put the result to the first vector.
// In:      s1  -> first vector
//          s2  -> second vector
// Out:     s1  -> result vector
// Return:  None
//------------------------------------------------------------------------------
static inline void subVec2Vec(double s1[3], const double s2[3])
{
    s1[0] -= s2[0];
    s1[1] -= s2[1];
    s1[2] -= s2[2];
}
    
//------------------------------------------------------------------------------
// Function:    mulScalar2Vec3
// Summary: Multiply a scalar to a vector.
// In:      s   -> scalar
//          v   -> vector
// Out:     v   -> result
// Return:  None
//------------------------------------------------------------------------------
static inline void mulScalar2Vec3(double s, double v[3])
{
    v[0] *= s;
    v[1] *= s;
    v[2] *= s;
}

//------------------------------------------------------------------------------
// Function:    mulScalar2Vec4
// Summary: Multiply a scalar to a vector.
// In:      s   -> scalar
//          v   -> vector
// Out:     v   -> result
// Return:  None
//------------------------------------------------------------------------------
static inline void mulScalar2Vec4(double s, double v[4])
{
    v[0] *= s;
    v[1] *= s;
    v[2] *= s;
    v[3] *= s;
}

//------------------------------------------------------------------------------
// Function:    mulMat2Vec
// Summary: Multiply a 4*4 matrix to a vector, and put the result to another vector.
// In:      m   -> 4*4 matrix
//          v   -> vector
// Out:     res -> result vector
// Return:  None
//------------------------------------------------------------------------------
static inline void mulMat2Vec(const double m[4][4], const double v[4], double res[4])
{
    res[0] = m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2] + m[0][3] * v[3];
    res[1] = m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2] + m[1][3] * v[3];
    res[2] = m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2] + m[2][3] * v[3];
    res[3] = m[3][0] * v[0] + m[3][1] * v[1] + m[3][2] * v[2] + m[3][3] * v[3];
}
    
//------------------------------------------------------------------------------
// Function:    rightMultiplyMat2Mat
// Summary: Multiply a 4*4 matrix to a 4*4 matrix, and put the result to the first matrix.
//          This is a right-multiply of matrix: m = m * n
// In:      m   -> 4*4 matrix
//          n   -> 4*4 matrix
// Out:     m   -> result
// Return:  None
//------------------------------------------------------------------------------
static inline void rightMultiplyMat2Mat(double m[4][4], const double n[4][4])
{
    double t0 = m[0][0] * n[0][0] + m[0][1] * n[1][0] + m[0][2] * n[2][0] + m[0][3] * n[3][0];
    double t1 = m[0][0] * n[0][1] + m[0][1] * n[1][1] + m[0][2] * n[2][1] + m[0][3] * n[3][1];
    double t2 = m[0][0] * n[0][2] + m[0][1] * n[1][2] + m[0][2] * n[2][2] + m[0][3] * n[3][2];
    double t3 = m[0][0] * n[0][3] + m[0][1] * n[1][3] + m[0][2] * n[2][3] + m[0][3] * n[3][3];

    m[0][0] = t0;
    m[0][1] = t1;
    m[0][2] = t2;
    m[0][3] = t3;
    t0 = m[1][0] * n[0][0] + m[1][1] * n[1][0] + m[1][2] * n[2][0] + m[1][3] * n[3][0];
    t1 = m[1][0] * n[0][1] + m[1][1] * n[1][1] + m[1][2] * n[2][1] + m[1][3] * n[3][1];
    t2 = m[1][0] * n[0][2] + m[1][1] * n[1][2] + m[1][2] * n[2][2] + m[1][3] * n[3][2];
    t3 = m[1][0] * n[0][3] + m[1][1] * n[1][3] + m[1][2] * n[2][3] + m[1][3] * n[3][3];
    m[1][0] = t0;
    m[1][1] = t1;
    m[1][2] = t2;
    m[1][3] = t3;
    t0 = m[2][0] * n[0][0] + m[2][1] * n[1][0] + m[2][2] * n[2][0] + m[2][3] * n[3][0];
    t1 = m[2][0] * n[0][1] + m[2][1] * n[1][1] + m[2][2] * n[2][1] + m[2][3] * n[3][1];
    t2 = m[2][0] * n[0][2] + m[2][1] * n[1][2] + m[2][2] * n[2][2] + m[2][3] * n[3][2];
    t3 = m[2][0] * n[0][3] + m[2][1] * n[1][3] + m[2][2] * n[2][3] + m[2][3] * n[3][3];
    m[2][0] = t0;
    m[2][1] = t1;
    m[2][2] = t2;
    m[2][3] = t3;
    t0 = m[3][0] * n[0][0] + m[3][1] * n[1][0] + m[3][2] * n[2][0] + m[3][3] * n[3][0];
    t1 = m[3][0] * n[0][1] + m[3][1] * n[1][1] + m[3][2] * n[2][1] + m[3][3] * n[3][1];
    t2 = m[3][0] * n[0][2] + m[3][1] * n[1][2] + m[3][2] * n[2][2] + m[3][3] * n[3][2];
    t3 = m[3][0] * n[0][3] + m[3][1] * n[1][3] + m[3][2] * n[2][3] + m[3][3] * n[3][3];
    m[3][0] = t0;
    m[3][1] = t1;
    m[3][2] = t2;
    m[3][3] = t3;
}

//------------------------------------------------------------------------------
// Function:    leftMultiplyMat2Mat
// Summary: Multiply a 4*4 matrix to a 4*4 matrix, and put the result to the second matrix.
//          This is a left-multiply of matrix: n = m * n
// In:      m   -> 4*4 matrix
//          n   -> 4*4 matrix
// Out:     n   -> result
// Return:  None
//------------------------------------------------------------------------------
static inline void leftMultiplyMat2Mat(const double m[4][4], double n[4][4])
{
    double t0 = m[0][0] * n[0][0] + m[0][1] * n[1][0] + m[0][2] * n[2][0] + m[0][3] * n[3][0];
    double t1 = m[1][0] * n[0][0] + m[1][1] * n[1][0] + m[1][2] * n[2][0] + m[1][3] * n[3][0];
    double t2 = m[2][0] * n[0][0] + m[2][1] * n[1][0] + m[2][2] * n[2][0] + m[2][3] * n[3][0];
    double t3 = m[3][0] * n[0][0] + m[3][1] * n[1][0] + m[3][2] * n[2][0] + m[3][3] * n[3][0];

    n[0][0] = t0;
    n[1][0] = t1;
    n[2][0] = t2;
    n[3][0] = t3;
    t0 = m[0][0] * n[0][1] + m[0][1] * n[1][1] + m[0][2] * n[2][1] + m[0][3] * n[3][1];
    t1 = m[1][0] * n[0][1] + m[1][1] * n[1][1] + m[1][2] * n[2][1] + m[1][3] * n[3][1];
    t2 = m[2][0] * n[0][1] + m[2][1] * n[1][1] + m[2][2] * n[2][1] + m[2][3] * n[3][1];
    t3 = m[3][0] * n[0][1] + m[3][1] * n[1][1] + m[3][2] * n[2][1] + m[3][3] * n[3][1];
    n[0][1] = t0;
    n[1][1] = t1;
    n[2][1] = t2;
    n[3][1] = t3;
    t0 = m[0][0] * n[0][2] + m[0][1] * n[1][2] + m[0][2] * n[2][2] + m[0][3] * n[3][2];
    t1 = m[1][0] * n[0][2] + m[1][1] * n[1][2] + m[1][2] * n[2][2] + m[1][3] * n[3][2];
    t2 = m[2][0] * n[0][2] + m[2][1] * n[1][2] + m[2][2] * n[2][2] + m[2][3] * n[3][2];
    t3 = m[3][0] * n[0][2] + m[3][1] * n[1][2] + m[3][2] * n[2][2] + m[3][3] * n[3][2];
    n[0][2] = t0;
    n[1][2] = t1;
    n[2][2] = t2;
    n[3][2] = t3;
    t0 = m[0][0] * n[0][3] + m[0][1] * n[1][3] + m[0][2] * n[2][3] + m[0][3] * n[3][3];
    t1 = m[1][0] * n[0][3] + m[1][1] * n[1][3] + m[1][2] * n[2][3] + m[1][3] * n[3][3];
    t2 = m[2][0] * n[0][3] + m[2][1] * n[1][3] + m[2][2] * n[2][3] + m[2][3] * n[3][3];
    t3 = m[3][0] * n[0][3] + m[3][1] * n[1][3] + m[3][2] * n[2][3] + m[3][3] * n[3][3];
    n[0][3] = t0;
    n[1][3] = t1;
    n[2][3] = t2;
    n[3][3] = t3;
}

//------------------------------------------------------------------------------
// Function:    mulMat2Mat
// Summary: Multiply a 4*4 matrix to a 4*4 matrix, and put the result to another matrix.
// In:      m   -> 4*4 matrix
//          n   -> 4*4 matrix
// Out:     res -> result matrix
// Return:  None
//------------------------------------------------------------------------------
static inline void mulMat2Mat(const double m[4][4], const double n[4][4], double res[4][4])
{
    res[0][0] = m[0][0] * n[0][0] + m[0][1] * n[1][0] + m[0][2] * n[2][0] + m[0][3] * n[3][0];
    res[0][1] = m[0][0] * n[0][1] + m[0][1] * n[1][1] + m[0][2] * n[2][1] + m[0][3] * n[3][1];
    res[0][2] = m[0][0] * n[0][2] + m[0][1] * n[1][2] + m[0][2] * n[2][2] + m[0][3] * n[3][2];
    res[0][3] = m[0][0] * n[0][3] + m[0][1] * n[1][3] + m[0][2] * n[2][3] + m[0][3] * n[3][3];
    res[1][0] = m[1][0] * n[0][0] + m[1][1] * n[1][0] + m[1][2] * n[2][0] + m[1][3] * n[3][0];
    res[1][1] = m[1][0] * n[0][1] + m[1][1] * n[1][1] + m[1][2] * n[2][1] + m[1][3] * n[3][1];
    res[1][2] = m[1][0] * n[0][2] + m[1][1] * n[1][2] + m[1][2] * n[2][2] + m[1][3] * n[3][2];
    res[1][3] = m[1][0] * n[0][3] + m[1][1] * n[1][3] + m[1][2] * n[2][3] + m[1][3] * n[3][3];
    res[2][0] = m[2][0] * n[0][0] + m[2][1] * n[1][0] + m[2][2] * n[2][0] + m[2][3] * n[3][0];
    res[2][1] = m[2][0] * n[0][1] + m[2][1] * n[1][1] + m[2][2] * n[2][1] + m[2][3] * n[3][1];
    res[2][2] = m[2][0] * n[0][2] + m[2][1] * n[1][2] + m[2][2] * n[2][2] + m[2][3] * n[3][2];
    res[2][3] = m[2][0] * n[0][3] + m[2][1] * n[1][3] + m[2][2] * n[2][3] + m[2][3] * n[3][3];
    res[3][0] = m[3][0] * n[0][0] + m[3][1] * n[1][0] + m[3][2] * n[2][0] + m[3][3] * n[3][0];
    res[3][1] = m[3][0] * n[0][1] + m[3][1] * n[1][1] + m[3][2] * n[2][1] + m[3][3] * n[3][1];
    res[3][2] = m[3][0] * n[0][2] + m[3][1] * n[1][2] + m[3][2] * n[2][2] + m[3][3] * n[3][2];
    res[3][3] = m[3][0] * n[0][3] + m[3][1] * n[1][3] + m[3][2] * n[2][3] + m[3][3] * n[3][3];
}

//------------------------------------------------------------------------------
// Function:    norm3
// Summary: Get the norm of a vector.
// In:      v   -> vector
// Out:     None
// Return:  norm of the input vector
//------------------------------------------------------------------------------
static inline double norm3(const double v[3]) {return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);}

//------------------------------------------------------------------------------
// Function:    norm4
// Summary: Get the norm of a vector.
// In:      v   -> vector
// Out:     None
// Return:  norm of the input vector
//------------------------------------------------------------------------------
static inline double norm4(const double v[4]) {return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);}

//------------------------------------------------------------------------------
// Function:    innerProduct3
// Summary: Get the inner product of two vectors.
// In:      v   -> first vector
//          u   -> second vector
// Out:     None
// Return:  inner product of two vectors
//------------------------------------------------------------------------------
static inline double innerProduct3(const double v[3], const double u[3])
{
    return v[0] * u[0] + v[1] * u[1] + v[2] * u[2];
}

//------------------------------------------------------------------------------
// Function:    innerProduct4
// Summary: Get the inner product of two vectors.
// In:      v   -> first vector
//          u   -> second vector
// Out:     None
// Return:  inner product of two vectors
//------------------------------------------------------------------------------
static inline double innerProduct4(const double v[4], const double u[4])
{
    return v[0] * u[0] + v[1] * u[1] + v[2] * u[2] + v[3] * u[3];
}

//------------------------------------------------------------------------------
// Function:    crossProduct
// Summary: Get the cross product of two vectors, and put the result to another vector.
// In:      v   -> first vector
//          u   -> second vector
// Out:     res -> result vector
// Return:  None
//------------------------------------------------------------------------------
static inline void crossProduct3(const double v[3], const double u[3], double res[3])
{
    res[0] = v[1] * u[2] - u[1] * v[2];
    res[1] = v[2] * u[0] - u[2] * v[0];
    res[2] = v[0] * u[1] - u[0] * v[1];
}

//------------------------------------------------------------------------------
// Function:    eye4
// Summary: Reset the matrix to 4*4 identity matrix.
// In:      None
// Out:     m   -> identity matrix
// Return:  None
//------------------------------------------------------------------------------
static inline void eye4(double m[4][4])
{
    m[0][0] = 1.0;
    m[0][1] = 0.0;
    m[0][2] = 0.0;
    m[0][3] = 0.0;
    m[1][0] = 0.0;
    m[1][1] = 1.0;
    m[1][2] = 0.0;
    m[1][3] = 0.0;
    m[2][0] = 0.0;
    m[2][1] = 0.0;
    m[2][2] = 1.0;
    m[2][3] = 0.0;
    m[3][0] = 0.0;
    m[3][1] = 0.0;
    m[3][2] = 0.0;
    m[3][3] = 1.0;
}

//------------------------------------------------------------------------------
// Function:    trans
// Summary: Set the matrix to   | 1 0 0 x |
//                              | 0 1 0 y |
//                              | 0 0 1 z |
//                              | 0 0 0 1 |
// In:      x, y, z
// Out:     m   -> output matrix
// Return:  None
//------------------------------------------------------------------------------
static inline void trans(double x, double y, double z, double m[4][4])
{
    m[0][0] = 1.0;
    m[0][1] = 0.0;
    m[0][2] = 0.0;
    m[0][3] = x;
    m[1][0] = 0.0;
    m[1][1] = 1.0;
    m[1][2] = 0.0;
    m[1][3] = y;
    m[2][0] = 0.0;
    m[2][1] = 0.0;
    m[2][2] = 1.0;
    m[2][3] = z;
    m[3][0] = 0.0;
    m[3][1] = 0.0;
    m[3][2] = 0.0;
    m[3][3] = 1.0;
}

//------------------------------------------------------------------------------
// Function:    rightRotateX
// Summary: Rotate the matrix with t rad, around X.
// In:      m   -> matrix
//          t   -> rotate angle
// Out:     m   -> output matrix
// Return:  None
//------------------------------------------------------------------------------
static inline void rightRotateX(double t, double m[4][4])
{
    double a = sin(t);
    double b = cos(t);

    double t1 = m[0][1] * b + m[0][2] * a;
    double t2 = m[0][2] * b - m[0][1] * a;
    m[0][1] = t1;
    m[0][2] = t2;

    t1 = m[1][1] * b + m[1][2] * a;
    t2 = m[1][2] * b - m[1][1] * a;
    m[1][1] = t1;
    m[1][2] = t2;
    
    t1 = m[2][1] * b + m[2][2] * a;
    t2 = m[2][2] * b - m[2][1] * a;
    m[2][1] = t1;
    m[2][2] = t2;

    t1 = m[3][1] * b + m[3][2] * a;
    t2 = m[3][2] * b - m[3][1] * a;
    m[3][1] = t1;
    m[3][2] = t2;
}

//------------------------------------------------------------------------------
// Function:    rightRotateY
// Summary: Rotate the matrix with t rad, around Y.
// In:      m   -> matrix
//          t   -> rotate angle
// Out:     m   -> output matrix
// Return:  None
//------------------------------------------------------------------------------
static inline void rightRotateY(double t, double m[4][4])
{
    double a = sin(t);
    double b = cos(t);

    double t1 = m[0][0] * b - m[0][2] * a;
    double t2 = m[0][0] * a + m[0][2] * b;
    m[0][0] = t1;
    m[0][2] = t2;

    t1 = m[1][0] * b - m[1][2] * a;
    t2 = m[1][0] * a + m[1][2] * b;
    m[1][0] = t1;
    m[1][2] = t2;

    t1 = m[2][0] * b - m[2][2] * a;
    t2 = m[2][0] * a + m[2][2] * b;
    m[2][0] = t1;
    m[2][2] = t2;

    t1 = m[3][0] * b - m[3][2] * a;
    t2 = m[3][0] * a + m[3][2] * b;
    m[3][0] = t1;
    m[3][2] = t2;
}

//------------------------------------------------------------------------------
// Function:    rightRotateZ
// Summary: Rotate the matrix with t rad, around Z.
// In:      m   -> matrix
//          t   -> rotate angle
// Out:     m   -> output matrix
// Return:  None
//------------------------------------------------------------------------------
static inline void rightRotateZ(double t, double m[4][4])
{
    double a = sin(t);
    double b = cos(t);

    double t1 = m[0][0] * b + m[0][1] * a;
    double t2 = m[0][1] * b - m[0][0] * a;
    m[0][0] = t1;
    m[0][1] = t2;

    t1 = m[1][0] * b + m[1][1] * a;
    t2 = m[1][1] * b - m[1][0] * a;
    m[1][0] = t1;
    m[1][1] = t2;

    t1 = m[2][0] * b + m[2][1] * a;
    t2 = m[2][1] * b - m[2][0] * a;
    m[2][0] = t1;
    m[2][1] = t2;

    t1 = m[3][0] * b + m[3][1] * a;
    t2 = m[3][1] * b - m[3][0] * a;
    m[3][0] = t1;
    m[3][1] = t2;
}
    
//------------------------------------------------------------------------------
// Function:    Pose2Vector
// Summary: Convert a pose to position and orientation vectors.
// In:      pose -> pose need to be converted
// Out:     p    -> position vector
//          o    -> orientation vector
// Return:  None
//------------------------------------------------------------------------------
static inline void Pose2Vector(const fst_controller::Pose &pose, double p[3], double o[4])
{
    p[0] = pose.position.x;
    p[1] = pose.position.y;
    p[2] = pose.position.z;
    o[0] = pose.orientation.w;
    o[1] = pose.orientation.x;
    o[2] = pose.orientation.y;
    o[3] = pose.orientation.z;
}

//------------------------------------------------------------------------------
// Function:    Vector2Pose
// Summary: Convert position and orientation vectors to pose.
// In:      p    -> position vector
//          o    -> orientation vector
// Out:     pose -> pose output
// Return:  None
//------------------------------------------------------------------------------
static inline void Vector2Pose(const double p[3], const double o[4], fst_controller::Pose &pose)
{
    pose.position.x = p[0];
    pose.position.y = p[1];
    pose.position.z = p[2];
    pose.orientation.w = o[0];
    pose.orientation.x = o[1];
    pose.orientation.y = o[2];
    pose.orientation.z = o[3];
}

//------------------------------------------------------------------------------
// Function:    Pose2Vector
// Summary: Convert a pose to a vector.
// In:      pose -> pose need to be converted
// Out:     p    -> vector
// Return:  None
//------------------------------------------------------------------------------
static inline void Pose2Vector(const fst_controller::Pose &pose, double p[7])
{
    p[0] = pose.position.x;
    p[1] = pose.position.y;
    p[2] = pose.position.z;
    p[3] = pose.orientation.w;
    p[4] = pose.orientation.x;
    p[5] = pose.orientation.y;
    p[6] = pose.orientation.z;
}

//------------------------------------------------------------------------------
// Function:    Vector2Pose
// Summary: Convert a vector to pose.
// In:      p    -> vector need to be converted
// Out:     pose -> pose output
// Return:  None
//------------------------------------------------------------------------------
static inline void Vector2Pose(const double p[7], fst_controller::Pose &pose)
{
    pose.position.x = p[0];
    pose.position.y = p[1];
    pose.position.z = p[2];
    pose.orientation.w = p[3];
    pose.orientation.x = p[4];
    pose.orientation.y = p[5];
    pose.orientation.z = p[6];

}
    
//------------------------------------------------------------------------------
// Function:    Vector2PoseEuler
// Summary: Convert a vector to pose-euler.
// In:      v   -> vector need to be converted
// Out:     pe  -> pose-euler
// Return:  None
//------------------------------------------------------------------------------
static inline void Vector2PoseEuler(const double v[6], fst_controller::PoseEuler &pose)
{
    pose.position.x = v[0];
    pose.position.y = v[1];
    pose.position.z = v[2];
    pose.orientation.a = v[3];
    pose.orientation.b = v[4];
    pose.orientation.c = v[5];
}

//------------------------------------------------------------------------------
// Function:    PoseEuler2Vector
// Summary: Convert a pose-euler to a vector.
// In:      pose -> pose-euler
// Out:     v    -> vector output
// Return:  None
//------------------------------------------------------------------------------
static inline void PoseEuler2Vector(const fst_controller::PoseEuler &pose, double v[6])
{
    v[0] = pose.position.x;
    v[1] = pose.position.y;
    v[2] = pose.position.z;
    v[3] = pose.orientation.a;
    v[4] = pose.orientation.b;
    v[5] = pose.orientation.c;
}


//------------------------------------------------------------------------------
// Function:    Vector2Euler
// Summary: Convert a vector to euler.
// In:      v   -> vector need to be converted
// Out:     e   -> euler
// Return:  None
//------------------------------------------------------------------------------
static inline void Vector2Euler(const double v[3], fst_controller::Euler &e)
{
    e.a = v[0];
    e.b = v[1];
    e.c = v[2];
}
    
//------------------------------------------------------------------------------
// Function:    Euler2Vector
// Summary: Convert a euler to a vector.
// In:      e   -> euler
// Out:     v   -> vector output
// Return:  None
//------------------------------------------------------------------------------
static inline void Euler2Vector(const fst_controller::Euler &e, double v[3])
{
    v[0] = e.a;
    v[1] = e.b;
    v[2] = e.c;
}
    
//------------------------------------------------------------------------------
// Function:    Vector2Quatern
// Summary: Convert a vector to quatern.
// In:      v   -> vector need to be converted
// Out:     q   -> quatern
// Return:  None
//------------------------------------------------------------------------------
static inline void Vector2Quatern(const double v[4], fst_controller::Quaternion &q)
{
    q.w = v[0];
    q.x = v[1];
    q.y = v[2];
    q.z = v[3];
}
    
//------------------------------------------------------------------------------
// Function:    Quatern2Vector
// Summary: Convert a quatern to a vector.
// In:      q   -> quatern
// Out:     v   -> vector output
// Return:  None
//------------------------------------------------------------------------------
static inline void Quatern2Vector(const fst_controller::Quaternion &q, double v[4])
{
    v[0] = q.w;
    v[1] = q.x;
    v[2] = q.y;
    v[3] = q.z;
}
    
//------------------------------------------------------------------------------
// Function:    Vector2Point
// Summary: Convert a vector to point.
// In:      v   -> vector need to be converted
// Out:     p   -> point(x, y, z)
// Return:  None
//------------------------------------------------------------------------------
static inline void Vector2Point(const double v[3], fst_controller::Point &p)
{
    p.x = v[0];
    p.y = v[1];
    p.z = v[2];
}
    
//------------------------------------------------------------------------------
// Function:    Point2Vector
// Summary: Convert a point to a vector.
// In:      p   -> point(x, y, z)
// Out:     v   -> vector output
// Return:  None
//------------------------------------------------------------------------------
static inline void Point2Vector(const fst_controller::Point &p, double v[3])
{
    v[0] = p.x;
    v[1] = p.y;
    v[2] = p.z;
}
    
//------------------------------------------------------------------------------
// Function:    Vector2Joint
// Summary: Convert a vector to joint.
// In:      v   -> vector need to be converted
// Out:     j   -> joint
// Return:  None
//------------------------------------------------------------------------------
static inline void Vector2Joint(const double v[6], fst_controller::Joint &j)
{
    j.j1 = v[0];
    j.j2 = v[1];
    j.j3 = v[2];
    j.j4 = v[3];
    j.j5 = v[4];
    j.j6 = v[5];
}
    
//------------------------------------------------------------------------------
// Function:    Joint2Vector
// Summary: Convert a joint to a vector.
// In:      j   -> joint
// Out:     v   -> vector output
// Return:  None
//------------------------------------------------------------------------------
static inline void Joint2Vector(const fst_controller::Joint &j, double v[6])
{
    v[0] = j.j1;
    v[1] = j.j2;
    v[2] = j.j3;
    v[3] = j.j4;
    v[4] = j.j5;
    v[5] = j.j6;
}
    
//------------------------------------------------------------------------------
// Function:    Vector2Omega
// Summary: Convert a vector to omega.
// In:      v   -> vector need to be converted
// Out:     w   -> omega
// Return:  None
//------------------------------------------------------------------------------
static inline void Vector2Omega(const double v[6], fst_controller::JointOmega &w)
{
    w.j1 = v[0];
    w.j2 = v[1];
    w.j3 = v[2];
    w.j4 = v[3];
    w.j5 = v[4];
    w.j6 = v[5];
}

//------------------------------------------------------------------------------
// Function:    Omega2Vector
// Summary: Convert a omega to a vector.
// In:      w   -> omega
// Out:     v   -> vector output
// Return:  None
//------------------------------------------------------------------------------
static inline void Omega2Vector(const fst_controller::JointOmega &w, double v[6])
{
    v[0] = w.j1;
    v[1] = w.j2;
    v[2] = w.j3;
    v[3] = w.j4;
    v[4] = w.j5;
    v[5] = w.j6;
}

//------------------------------------------------------------------------------
// Function:    PoseEuler2Matrix
// Summary: Convert a pose-euler to a 4*4 matrix.
// In:      pose -> pose euler need to be converted
// Out:     m    -> matrix output
// Return:  None
//------------------------------------------------------------------------------
static inline void PoseEuler2Matrix(const fst_controller::PoseEuler &pose, double m[4][4])
{
    trans(pose.position.x, pose.position.y, pose.position.z, m);
    rightRotateZ(pose.orientation.a, m);
    rightRotateY(pose.orientation.b, m);
    rightRotateX(pose.orientation.c, m);
}

//------------------------------------------------------------------------------
// Function:    Matrix2PoseEuler
// Summary: Convert a 4*4 matrix to pose-euler.
// In:      m    -> matrix need to be converted
// Out:     pose -> pose-euler output
// Return:  None
//------------------------------------------------------------------------------
static inline void Matrix2PoseEuler(const double m[4][4], fst_controller::PoseEuler &pose)
{
    pose.position.x = m[0][3];
    pose.position.y = m[1][3];
    pose.position.z = m[2][3];

    pose.orientation.b = atan2(-m[2][0], sqrt(m[0][0] * m[0][0] + m[1][0] * m[1][0]));

    if (m[2][1] * m[2][1] + m[2][2] * m[2][2] > MINIMUM_E12) {
        double cosb = cos(pose.orientation.b);
        pose.orientation.a = atan2(m[1][0] / cosb, m[0][0] / cosb);
        pose.orientation.c = atan2(m[2][1] / cosb, m[2][2] / cosb);
    }
    else {
        pose.orientation.a = atan2(-m[0][1], m[1][1]);
        pose.orientation.c = 0.0;
    }
}

//------------------------------------------------------------------------------
// Function:    Pose2Matrix
// Summary: Convert a pose to a 4*4 matrix.
// In:      pose -> pose need to be converted
// Out:     m    -> matrix output
// Return:  None
//------------------------------------------------------------------------------
static inline bool Pose2Matrix(const fst_controller::Pose &pose, double m[4][4])
{
    double ss = pose.orientation.w * pose.orientation.w;
    double aa = pose.orientation.x * pose.orientation.x;
    double bb = pose.orientation.y * pose.orientation.y;
    double cc = pose.orientation.z * pose.orientation.z;
    double sa = pose.orientation.w * pose.orientation.x;
    double sb = pose.orientation.w * pose.orientation.y;
    double sc = pose.orientation.w * pose.orientation.z;
    double ab = pose.orientation.x * pose.orientation.y;
    double ac = pose.orientation.x * pose.orientation.z;
    double bc = pose.orientation.y * pose.orientation.z;
    double sum = ss + aa + bb + cc;

    if (fabs(sum -1) < 0.0005) {
        ss /= sum; aa /= sum; bb /= sum; cc /= sum;
        sa /= sum; sb /= sum; sc /= sum;
        ab /= sum; ac /= sum; bc /= sum;
    }
    else {
        return false;
    }

    m[0][0] = 2 * (ss + aa) - 1;
    m[0][1] = 2 * (ab - sc);
    m[0][2] = 2 * (sb + ac);
    m[0][3] = pose.position.x;
    m[1][0] = 2 * (ab + sc);
    m[1][1] = 2 * (ss + bb) - 1;
    m[1][2] = 2 * (bc - sa);
    m[1][3] = pose.position.y;
    m[2][0] = 2 * (ac - sb);
    m[2][1] = 2 * (sa + bc);
    m[2][2] = 2 * (ss + cc) - 1;
    m[2][3] = pose.position.z;
    m[3][0] = 0.0;
    m[3][1] = 0.0;
    m[3][2] = 0.0;
    m[3][3] = 1.0;
    
    return true;
}

//------------------------------------------------------------------------------
// Function:    Matrix2Pose
// Summary: Convert a 4*4 matrix to pose.
// In:      m    -> matrix need to be converted
// Out:     pose -> pose output
// Return:  None
//------------------------------------------------------------------------------
static inline void Matrix2Pose(const double m[4][4], fst_controller::Pose &pose)
{
    pose.position.x = m[0][3];
    pose.position.y = m[1][3];
    pose.position.z = m[2][3];

    double ort[4] = {
                        sqrt(fabs(m[0][0]  + m[1][1] + m[2][2] + 1)) / 2,
                        sqrt(fabs(m[0][0]  - m[1][1] - m[2][2] + 1)) / 2,
                        sqrt(fabs(-m[0][0] + m[1][1] - m[2][2] + 1)) / 2,
                        sqrt(fabs(-m[0][0] - m[1][1] + m[2][2] + 1)) / 2
                    };

    double w = sqrt(fabs(m[0][0]  + m[1][1] + m[2][2] + 1)) / 2;
    double x = sqrt(fabs(m[0][0]  - m[1][1] - m[2][2] + 1)) / 2;
    double y = sqrt(fabs(-m[0][0] + m[1][1] - m[2][2] + 1)) / 2;
    double z = sqrt(fabs(-m[0][0] - m[1][1] + m[2][2] + 1)) / 2;

    size_t index = 0;
    double max_n = w;

    if (x > max_n) {index = 1; max_n = x;}
    if (y > max_n) {index = 2; max_n = y;}
    if (z > max_n) {index = 3; max_n = z;}

    // 数值最大的元素必然不为0，令其为正并以其为参考确定其他元素的符号
    switch (index)
    {
        case 0: x = (m[2][1] - m[1][2]) > 0 ? x : -x;
                y = (m[0][2] - m[2][0]) > 0 ? y : -y;
                z = (m[1][0] - m[0][1]) > 0 ? z : -z;
                break;
        case 1: w = (m[2][1] - m[1][2]) > 0 ? w : -w;
                y = (m[1][0] + m[0][1]) > 0 ? y : -y;
                z = (m[2][0] + m[0][2]) > 0 ? z : -z;
                break;
        case 2: w = (m[0][2] - m[2][0]) > 0 ? w : -w;
                x = (m[1][0] + m[0][1]) > 0 ? x : -x;
                z = (m[2][1] + m[1][2]) > 0 ? z : -z;
                break;
        case 3: w = (m[1][0] - m[0][1]) > 0 ? w : -w;
                x = (m[2][0] + m[0][2]) > 0 ? x : -x;
                y = (m[2][1] + m[1][2]) > 0 ? y : -y;
                break;
    }

    double n = sqrt(w * w + x * x + y * y + z * z);

    //四元数强制单位化
    pose.orientation.w = w / n;
    pose.orientation.x = x / n;
    pose.orientation.y = y / n;
    pose.orientation.z = z / n;
}

//------------------------------------------------------------------------------
// Function:    Pose2PoseEuler
// Summary: Convert a pose to pose-euler.
// In:      pose -> pose need to be converted
// Out:     pe   -> pose-euler output
// Return:  true -> success
//          false-> fail, input quaternion illegal
//------------------------------------------------------------------------------
static inline bool Pose2PoseEuler(const fst_controller::Pose &pose, fst_controller::PoseEuler &pe)
{
    double tmp[4][4];

    if (!Pose2Matrix(pose, tmp))
        return false;

    Matrix2PoseEuler(tmp, pe);
    return true;
}

//------------------------------------------------------------------------------
// Function:    PoseEuler2Pose
// Summary: Convert a pose-euler to pose.
// In:      pe   -> pose-euler needed to be converted
// Out:     pose -> pose output
// Return:  None
//------------------------------------------------------------------------------
static inline void PoseEuler2Pose(const fst_controller::PoseEuler &pe, fst_controller::Pose &pose)
{
    double tmp[4][4];

    PoseEuler2Matrix(pe, tmp);
    Matrix2Pose(tmp, pose);
}

//------------------------------------------------------------------------------
// Function:    PoseEuler2Pose
// Summary: Convert a pose-euler to pose.
// In:      pe   -> pose-euler needed to be converted
// Out:     None
// Return   pose
//------------------------------------------------------------------------------
static inline fst_controller::Pose  PoseEuler2Pose(const fst_controller::PoseEuler &pe)
{
    double tmp[4][4];
    fst_controller::Pose pose;

    PoseEuler2Matrix(pe, tmp);
    Matrix2Pose(tmp, pose);
    return pose;
}

//------------------------------------------------------------------------------
// Function:    det
// Summary: To return value of the determinant.
// In:      m   -> matrix needed to compute determinant
// Out:     None
// Return:  the value of determinant
//------------------------------------------------------------------------------
inline double det(const double m[4][4])
{
    return  m[0][0] * (m[1][1] * (m[2][2] * m[3][3] - m[3][2] * m[2][3]) - m[1][2] * (m[2][1] * m[3][3] - m[3][1] * m[2][3]) + m[1][3] * (m[2][1] * m[3][2] - m[3][1] * m[2][2]))
           -m[0][1] * (m[1][0] * (m[2][2] * m[3][3] - m[3][2] * m[2][3]) - m[1][2] * (m[2][0] * m[3][3] - m[3][0] * m[2][3]) + m[1][3] * (m[2][0] * m[3][2] - m[3][0] * m[2][2]))
           +m[0][2] * (m[1][0] * (m[2][1] * m[3][3] - m[3][1] * m[2][3]) - m[1][1] * (m[2][0] * m[3][3] - m[3][0] * m[2][3]) + m[1][3] * (m[2][0] * m[3][1] - m[3][0] * m[2][1]))
           -m[0][3] * (m[1][0] * (m[2][1] * m[3][2] - m[3][1] * m[2][2]) - m[1][1] * (m[2][0] * m[3][2] - m[3][0] * m[2][2]) + m[1][2] * (m[2][0] * m[3][1] - m[3][0] * m[2][1]));
}

//------------------------------------------------------------------------------
// Function:    inverseMatrix
// Summary: To get inverse matrix according to the input.
// In:      m     -> matrix input
// Out:     m     -> inverse matrix output
// Return:  true  -> inverse success
//          false -> inverse matrix doesn't exist
//------------------------------------------------------------------------------
static inline bool inverseMatrix(double m[4][4])
{
    double determ = det(m);
    if (determ < MINIMUM_E6) return false;

    double a00 = m[1][1] * (m[2][2] * m[3][3] - m[3][2] * m[2][3]) - m[1][2] * (m[2][1] * m[3][3] - m[3][1] * m[2][3]) + m[1][3] * (m[2][1] * m[3][2] - m[3][1] * m[2][2]);
    double a01 = m[1][0] * (m[2][2] * m[3][3] - m[3][2] * m[2][3]) - m[1][2] * (m[2][0] * m[3][3] - m[3][0] * m[2][3]) + m[1][3] * (m[2][0] * m[3][2] - m[3][0] * m[2][2]);
    double a02 = m[1][0] * (m[2][1] * m[3][3] - m[3][1] * m[2][3]) - m[1][1] * (m[2][0] * m[3][3] - m[3][0] * m[2][3]) + m[1][3] * (m[2][0] * m[3][1] - m[3][0] * m[2][1]);
    double a03 = m[1][0] * (m[2][1] * m[3][2] - m[3][1] * m[2][2]) - m[1][1] * (m[2][0] * m[3][2] - m[3][0] * m[2][2]) + m[1][2] * (m[2][0] * m[3][1] - m[3][0] * m[2][1]);
    double a10 = m[0][1] * (m[2][2] * m[3][3] - m[3][2] * m[2][3]) - m[0][2] * (m[2][1] * m[3][3] - m[3][1] * m[2][3]) + m[0][3] * (m[2][1] * m[3][2] - m[3][1] * m[2][2]);
    double a11 = m[0][0] * (m[2][2] * m[3][3] - m[3][2] * m[2][3]) - m[0][2] * (m[2][0] * m[3][3] - m[3][0] * m[2][3]) + m[0][3] * (m[2][0] * m[3][2] - m[3][0] * m[2][2]);
    double a12 = m[0][0] * (m[2][1] * m[3][3] - m[3][1] * m[2][3]) - m[0][1] * (m[2][0] * m[3][3] - m[3][0] * m[2][3]) + m[0][3] * (m[2][0] * m[3][1] - m[3][0] * m[2][1]);
    double a13 = m[0][0] * (m[2][1] * m[3][2] - m[3][1] * m[2][2]) - m[0][1] * (m[2][0] * m[3][2] - m[3][0] * m[2][2]) + m[0][2] * (m[2][0] * m[3][1] - m[3][0] * m[2][1]);
    double a20 = m[0][1] * (m[1][2] * m[3][3] - m[3][2] * m[1][3]) - m[0][2] * (m[1][1] * m[3][3] - m[3][1] * m[1][3]) + m[0][3] * (m[1][1] * m[3][2] - m[3][1] * m[1][2]);
    double a21 = m[0][0] * (m[1][2] * m[3][3] - m[3][2] * m[1][3]) - m[0][2] * (m[1][0] * m[3][3] - m[3][0] * m[1][3]) + m[0][3] * (m[1][0] * m[3][2] - m[3][0] * m[1][2]);
    double a22 = m[0][0] * (m[1][1] * m[3][3] - m[3][1] * m[1][3]) - m[0][1] * (m[1][0] * m[3][3] - m[3][0] * m[1][3]) + m[0][3] * (m[1][0] * m[3][1] - m[3][0] * m[1][1]);
    double a23 = m[0][0] * (m[1][1] * m[3][2] - m[3][1] * m[1][2]) - m[0][1] * (m[1][0] * m[3][2] - m[3][0] * m[1][2]) + m[0][2] * (m[1][0] * m[3][1] - m[3][0] * m[1][1]);
    double a30 = m[0][1] * (m[1][2] * m[2][3] - m[2][2] * m[1][3]) - m[0][2] * (m[1][1] * m[2][3] - m[2][1] * m[1][3]) + m[0][3] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]);
    double a31 = m[0][0] * (m[1][2] * m[2][3] - m[2][2] * m[1][3]) - m[0][2] * (m[1][0] * m[2][3] - m[2][0] * m[1][3]) + m[0][3] * (m[1][0] * m[2][2] - m[2][0] * m[1][2]);
    double a32 = m[0][0] * (m[1][1] * m[2][3] - m[2][1] * m[1][3]) - m[0][1] * (m[1][0] * m[2][3] - m[2][0] * m[1][3]) + m[0][3] * (m[1][0] * m[2][1] - m[2][0] * m[1][1]);
    double a33 = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) - m[0][1] * (m[1][0] * m[2][2] - m[2][0] * m[1][2]) + m[0][2] * (m[1][0] * m[2][1] - m[2][0] * m[1][1]);

    m[0][0] =  a00 / determ; m[0][1] = -a10 / determ; m[0][2] =  a20 / determ; m[0][3] = -a30 / determ;
    m[1][0] = -a01 / determ; m[1][1] =  a11 / determ; m[1][2] = -a21 / determ; m[1][3] =  a31 / determ;
    m[2][0] =  a02 / determ; m[2][1] = -a12 / determ; m[2][2] =  a22 / determ; m[2][3] = -a32 / determ;
    m[3][0] = -a03 / determ; m[3][1] =  a13 / determ; m[3][2] = -a23 / determ; m[3][3] =  a33 / determ;

    return true;
}

static inline void DHMatrix(const double l[4], double q, double m[4][4])
{
    double tmp[4][4];
    eye4(m);
    rightRotateX(l[0], m);
    trans(l[1], 0, 0, tmp);
    rightMultiplyMat2Mat(m, tmp);
    rightRotateZ(l[3] + q, m);
    trans(0, 0, l[2], tmp);
    rightMultiplyMat2Mat(m, tmp);
}

static inline double getDistance(const fst_controller::Pose &pose1, const fst_controller::Pose &pose2)
{
    double x = pose2.position.x - pose1.position.x;
    double y = pose2.position.y - pose1.position.y;
    double z = pose2.position.z - pose1.position.z;
    
    return sqrt(x * x + y * y + z * z);
}

static inline double innerProductQuatern(const fst_controller::Quaternion q1, const fst_controller::Quaternion q2)
{
    return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;
}

static inline double getOrientationAngle(const fst_controller::Pose &pose1, const fst_controller::Pose &pose2)
{
    double t = innerProductQuatern(pose1.orientation, pose2.orientation);
    
    if (t > 1.0)        t =  1.0;
    else if (t < -1.0)  t = -1.0;
    
    return acos(t);
}

static inline bool isJointInConstraint(const fst_controller::Joint jnt, const fst_controller::JointConstraint cons)
{
    return  jnt.j1 > cons.j1.lower - MINIMUM_E9 && jnt.j1 < cons.j1.upper + MINIMUM_E9 &&
            jnt.j2 > cons.j2.lower - MINIMUM_E9 && jnt.j2 < cons.j2.upper + MINIMUM_E9 &&
            jnt.j3 > cons.j3.lower - MINIMUM_E9 && jnt.j3 < cons.j3.upper + MINIMUM_E9 &&
            jnt.j4 > cons.j4.lower - MINIMUM_E9 && jnt.j4 < cons.j4.upper + MINIMUM_E9 &&
            jnt.j5 > cons.j5.lower - MINIMUM_E9 && jnt.j5 < cons.j5.upper + MINIMUM_E9 &&
            jnt.j6 > cons.j6.lower - MINIMUM_E9 && jnt.j6 < cons.j6.upper + MINIMUM_E9;
}

static inline int contrastJointWithConstraint(const fst_controller::Joint jnt, const fst_controller::JointConstraint cons)
{
    int res = 0;
    res = (res << 1) | (jnt.j6 > cons.j6.lower - MINIMUM_E9 && jnt.j6 < cons.j6.upper + MINIMUM_E9) ? 0 : 1;
    res = (res << 1) | (jnt.j5 > cons.j5.lower - MINIMUM_E9 && jnt.j5 < cons.j5.upper + MINIMUM_E9) ? 0 : 1;
    res = (res << 1) | (jnt.j4 > cons.j4.lower - MINIMUM_E9 && jnt.j4 < cons.j4.upper + MINIMUM_E9) ? 0 : 1;
    res = (res << 1) | (jnt.j3 > cons.j3.lower - MINIMUM_E9 && jnt.j3 < cons.j3.upper + MINIMUM_E9) ? 0 : 1;
    res = (res << 1) | (jnt.j2 > cons.j2.lower - MINIMUM_E9 && jnt.j2 < cons.j2.upper + MINIMUM_E9) ? 0 : 1;
    res = (res << 1) | (jnt.j1 > cons.j1.lower - MINIMUM_E9 && jnt.j1 < cons.j1.upper + MINIMUM_E9) ? 0 : 1;
    return res;
}

static inline bool isFirstConstraintCoveredBySecond(const fst_controller::JointConstraint &child,
                                                    const fst_controller::JointConstraint &parent)
{
    return !(child.j1.upper > parent.j1.upper || child.j1.lower < parent.j1.lower ||
             child.j2.upper > parent.j2.upper || child.j2.lower < parent.j2.lower ||
             child.j3.upper > parent.j3.upper || child.j3.lower < parent.j3.lower ||
             child.j4.upper > parent.j4.upper || child.j4.lower < parent.j4.lower ||
             child.j5.upper > parent.j5.upper || child.j5.lower < parent.j5.lower ||
             child.j6.upper > parent.j6.upper || child.j6.lower < parent.j6.lower ||
             child.j1.max_omega > parent.j1.max_omega || child.j1.max_alpha > parent.j1.max_alpha ||
             child.j2.max_omega > parent.j2.max_omega || child.j2.max_alpha > parent.j2.max_alpha ||
             child.j3.max_omega > parent.j3.max_omega || child.j3.max_alpha > parent.j3.max_alpha ||
             child.j4.max_omega > parent.j4.max_omega || child.j4.max_alpha > parent.j4.max_alpha ||
             child.j5.max_omega > parent.j5.max_omega || child.j5.max_alpha > parent.j5.max_alpha ||
             child.j6.max_omega > parent.j6.max_omega || child.j6.max_alpha > parent.j6.max_alpha);
}

}

#endif
