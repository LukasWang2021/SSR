/*************************************************************************
	> File Name: motion_plan_basic_function.h
	> Author:
	> Mail:
	> Created Time: 2017年12月07日 星期四 16时41分13秒
 ************************************************************************/

#ifndef BASIC_ALG_H
#define BASIC_ALG_H

#include <math.h>
#include <basic_alg_datatype.h>
#include <basic_constants.h>

namespace basic_alg
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
//static inline double MIN(double a, double b) {return (a < b) ? a : b;}

//------------------------------------------------------------------------------
// Function:    MAX
// Summary: To get the larger one of two numbers.
// In:      a   -> first number
//          b   -> second number
// Out:     None
// Return:  a   -> a > b
//          b   -> a < b
//------------------------------------------------------------------------------
//static inline double MAX(double a, double b) {return (a > b) ? a : b;}

//------------------------------------------------------------------------------
// Function:    assignVector
// Summary: Copy a vector to another one.
// In:      source      -> source vector
//          destination -> destination vector
// Out:     None
// Return:  None
//------------------------------------------------------------------------------
static inline void assignVector(const double (&source)[2], double (&destination)[2])
{
    destination[0] = source[0];
    destination[1] = source[1];
}
static inline void assignVector(const double (&source)[3], double (&destination)[3])
{
    destination[0] = source[0];
    destination[1] = source[1];
    destination[2] = source[2];
}
static inline void assignVector(const double (&source)[4], double (&destination)[4])
{
    destination[0] = source[0];
    destination[1] = source[1];
    destination[2] = source[2];
    destination[3] = source[3];
}
static inline void assignVector(const double (&source)[5], double (&destination)[5])
{
    destination[0] = source[0];
    destination[1] = source[1];
    destination[2] = source[2];
    destination[3] = source[3];
    destination[4] = source[4];
}
static inline void assignVector(const double (&source)[6], double (&destination)[6])
{
    destination[0] = source[0];
    destination[1] = source[1];
    destination[2] = source[2];
    destination[3] = source[3];
    destination[4] = source[4];
    destination[5] = source[5];
}
static inline void assignVector(const double (&source)[7], double (&destination)[7])
{
    destination[0] = source[0];
    destination[1] = source[1];
    destination[2] = source[2];
    destination[3] = source[3];
    destination[4] = source[4];
    destination[5] = source[5];
    destination[6] = source[6];
}
static inline void assignVector(const double (&source)[8], double (&destination)[8])
{
    destination[0] = source[0];
    destination[1] = source[1];
    destination[2] = source[2];
    destination[3] = source[3];
    destination[4] = source[4];
    destination[5] = source[5];
    destination[6] = source[6];
    destination[7] = source[7];
}
static inline void assignVector(const double (&source)[9], double (&destination)[9])
{
    destination[0] = source[0];
    destination[1] = source[1];
    destination[2] = source[2];
    destination[3] = source[3];
    destination[4] = source[4];
    destination[5] = source[5];
    destination[6] = source[6];
    destination[7] = source[7];
    destination[8] = source[8];
}

//------------------------------------------------------------------------------
// Function:    addVector
// Summary: Add two vectors, and put the result to another vector.
// In:      a1  -> first vector
//          a2  -> second vector
// Out:     res -> result vector
// Return:  None
//------------------------------------------------------------------------------
static inline void addVector(const double (&a1)[2], const double (&a2)[2], double (&res)[2])
{
    res[0] = a1[0] + a2[0];
    res[1] = a1[1] + a2[1];
}
static inline void addVector(const double (&a1)[3], const double (&a2)[3], double (&res)[3])
{
    res[0] = a1[0] + a2[0];
    res[1] = a1[1] + a2[1];
    res[2] = a1[2] + a2[2];
}
static inline void addVector(const double (&a1)[4], const double (&a2)[4], double (&res)[4])
{
    res[0] = a1[0] + a2[0];
    res[1] = a1[1] + a2[1];
    res[2] = a1[2] + a2[2];
    res[3] = a1[3] + a2[3];
}
static inline void addVector(const double (&a1)[5], const double (&a2)[5], double (&res)[5])
{
    res[0] = a1[0] + a2[0];
    res[1] = a1[1] + a2[1];
    res[2] = a1[2] + a2[2];
    res[3] = a1[3] + a2[3];
    res[4] = a1[4] + a2[4];
}
static inline void addVector(const double (&a1)[6], const double (&a2)[6], double (&res)[6])
{
    res[0] = a1[0] + a2[0];
    res[1] = a1[1] + a2[1];
    res[2] = a1[2] + a2[2];
    res[3] = a1[3] + a2[3];
    res[4] = a1[4] + a2[4];
    res[5] = a1[5] + a2[5];
}
static inline void addVector(const double (&a1)[7], const double (&a2)[7], double (&res)[7])
{
    res[0] = a1[0] + a2[0];
    res[1] = a1[1] + a2[1];
    res[2] = a1[2] + a2[2];
    res[3] = a1[3] + a2[3];
    res[4] = a1[4] + a2[4];
    res[5] = a1[5] + a2[5];
    res[6] = a1[6] + a2[6];
}
static inline void addVector(const double (&a1)[8], const double (&a2)[8], double (&res)[8])
{
    res[0] = a1[0] + a2[0];
    res[1] = a1[1] + a2[1];
    res[2] = a1[2] + a2[2];
    res[3] = a1[3] + a2[3];
    res[4] = a1[4] + a2[4];
    res[5] = a1[5] + a2[5];
    res[6] = a1[6] + a2[6];
    res[7] = a1[7] + a2[7];
}
static inline void addVector(const double (&a1)[9], const double (&a2)[9], double (&res)[9])
{
    res[0] = a1[0] + a2[0];
    res[1] = a1[1] + a2[1];
    res[2] = a1[2] + a2[2];
    res[3] = a1[3] + a2[3];
    res[4] = a1[4] + a2[4];
    res[5] = a1[5] + a2[5];
    res[6] = a1[6] + a2[6];
    res[7] = a1[7] + a2[7];
    res[8] = a1[8] + a2[8];
}

//------------------------------------------------------------------------------
// Function:    addVector
// Summary: Add two vectors, and put the result to the first vector.
// In:      a1  -> first vector
//          a2  -> second vector
// Out:     a1  -> result vector
// Return:  None
//------------------------------------------------------------------------------
static inline void addVector(double (&a1)[2], const double (&a2)[2])
{
    a1[0] += a2[0];
    a1[1] += a2[1];
}
static inline void addVector(double (&a1)[3], const double (&a2)[3])
{
    a1[0] += a2[0];
    a1[1] += a2[1];
    a1[2] += a2[2];
}
static inline void addVector(double (&a1)[4], const double (&a2)[4])
{
    a1[0] += a2[0];
    a1[1] += a2[1];
    a1[2] += a2[2];
    a1[3] += a2[3];
}
static inline void addVector(double (&a1)[5], const double (&a2)[5])
{
    a1[0] += a2[0];
    a1[1] += a2[1];
    a1[2] += a2[2];
    a1[3] += a2[3];
    a1[4] += a2[4];
}
static inline void addVector(double (&a1)[6], const double (&a2)[6])
{
    a1[0] += a2[0];
    a1[1] += a2[1];
    a1[2] += a2[2];
    a1[3] += a2[3];
    a1[4] += a2[4];
    a1[5] += a2[5];
}
static inline void addVector(double (&a1)[7], const double (&a2)[7])
{
    a1[0] += a2[0];
    a1[1] += a2[1];
    a1[2] += a2[2];
    a1[3] += a2[3];
    a1[4] += a2[4];
    a1[5] += a2[5];
    a1[6] += a2[6];
}
static inline void addVector(double (&a1)[8], const double (&a2)[8])
{
    a1[0] += a2[0];
    a1[1] += a2[1];
    a1[2] += a2[2];
    a1[3] += a2[3];
    a1[4] += a2[4];
    a1[5] += a2[5];
    a1[6] += a2[6];
    a1[7] += a2[7];
}
static inline void addVector(double (&a1)[9], const double (&a2)[9])
{
    a1[0] += a2[0];
    a1[1] += a2[1];
    a1[2] += a2[2];
    a1[3] += a2[3];
    a1[4] += a2[4];
    a1[5] += a2[5];
    a1[6] += a2[6];
    a1[7] += a2[7];
    a1[8] += a2[8];
}

//------------------------------------------------------------------------------
// Function:    subVector
// Summary: First vector sub second one, and put the result to another vector.
// In:      s1  -> first vector
//          s2  -> second vector
// Out:     res -> result vector
// Return:  None
//------------------------------------------------------------------------------
static inline void subVector(const double (&s1)[2], const double (&s2)[2], double (&res)[2])
{
    res[0] = s1[0] - s2[0];
    res[1] = s1[1] - s2[1];
}
static inline void subVector(const double (&s1)[3], const double (&s2)[3], double (&res)[3])
{
    res[0] = s1[0] - s2[0];
    res[1] = s1[1] - s2[1];
    res[2] = s1[2] - s2[2];
}
static inline void subVector(const double (&s1)[4], const double (&s2)[4], double (&res)[4])
{
    res[0] = s1[0] - s2[0];
    res[1] = s1[1] - s2[1];
    res[2] = s1[2] - s2[2];
    res[3] = s1[3] - s2[3];
}
static inline void subVector(const double (&s1)[5], const double (&s2)[5], double (&res)[5])
{
    res[0] = s1[0] - s2[0];
    res[1] = s1[1] - s2[1];
    res[2] = s1[2] - s2[2];
    res[3] = s1[3] - s2[3];
    res[4] = s1[4] - s2[4];
}
static inline void subVector(const double (&s1)[6], const double (&s2)[6], double (&res)[6])
{
    res[0] = s1[0] - s2[0];
    res[1] = s1[1] - s2[1];
    res[2] = s1[2] - s2[2];
    res[3] = s1[3] - s2[3];
    res[4] = s1[4] - s2[4];
    res[5] = s1[5] - s2[5];
}
static inline void subVector(const double (&s1)[7], const double (&s2)[7], double (&res)[7])
{
    res[0] = s1[0] - s2[0];
    res[1] = s1[1] - s2[1];
    res[2] = s1[2] - s2[2];
    res[3] = s1[3] - s2[3];
    res[4] = s1[4] - s2[4];
    res[5] = s1[5] - s2[5];
    res[6] = s1[6] - s2[6];
}
static inline void subVector(const double (&s1)[8], const double (&s2)[8], double (&res)[8])
{
    res[0] = s1[0] - s2[0];
    res[1] = s1[1] - s2[1];
    res[2] = s1[2] - s2[2];
    res[3] = s1[3] - s2[3];
    res[4] = s1[4] - s2[4];
    res[5] = s1[5] - s2[5];
    res[6] = s1[6] - s2[6];
    res[7] = s1[7] - s2[7];
}
static inline void subVector(const double (&s1)[9], const double (&s2)[9], double (&res)[9])
{
    res[0] = s1[0] - s2[0];
    res[1] = s1[1] - s2[1];
    res[2] = s1[2] - s2[2];
    res[3] = s1[3] - s2[3];
    res[4] = s1[4] - s2[4];
    res[5] = s1[5] - s2[5];
    res[6] = s1[6] - s2[6];
    res[7] = s1[7] - s2[7];
    res[8] = s1[8] - s2[8];
}

//------------------------------------------------------------------------------
// Function:    subVector
// Summary: First vector sub second one, and put the result to the first vector.
// In:      s1  -> first vector
//          s2  -> second vector
// Out:     s1  -> result vector
// Return:  None
//------------------------------------------------------------------------------
static inline void subVector(double (&s1)[2], const double (&s2)[2])
{
    s1[0] -= s2[0];
    s1[1] -= s2[1];
}
static inline void subVector(double (&s1)[3], const double (&s2)[3])
{
    s1[0] -= s2[0];
    s1[1] -= s2[1];
    s1[2] -= s2[2];
}
static inline void subVector(double (&s1)[4], const double (&s2)[4])
{
    s1[0] -= s2[0];
    s1[1] -= s2[1];
    s1[2] -= s2[2];
    s1[3] -= s2[3];
}
static inline void subVector(double (&s1)[5], const double (&s2)[5])
{
    s1[0] -= s2[0];
    s1[1] -= s2[1];
    s1[2] -= s2[2];
    s1[3] -= s2[3];
    s1[4] -= s2[4];
}
static inline void subVector(double (&s1)[6], const double (&s2)[6])
{
    s1[0] -= s2[0];
    s1[1] -= s2[1];
    s1[2] -= s2[2];
    s1[3] -= s2[3];
    s1[4] -= s2[4];
    s1[5] -= s2[5];
}
static inline void subVector(double (&s1)[7], const double (&s2)[7])
{
    s1[0] -= s2[0];
    s1[1] -= s2[1];
    s1[2] -= s2[2];
    s1[3] -= s2[3];
    s1[4] -= s2[4];
    s1[5] -= s2[5];
    s1[6] -= s2[6];
}
static inline void subVector(double (&s1)[8], const double (&s2)[8])
{
    s1[0] -= s2[0];
    s1[1] -= s2[1];
    s1[2] -= s2[2];
    s1[3] -= s2[3];
    s1[4] -= s2[4];
    s1[5] -= s2[5];
    s1[6] -= s2[6];
    s1[7] -= s2[7];
}
static inline void subVector(double (&s1)[9], const double (&s2)[9])
{
    s1[0] -= s2[0];
    s1[1] -= s2[1];
    s1[2] -= s2[2];
    s1[3] -= s2[3];
    s1[4] -= s2[4];
    s1[5] -= s2[5];
    s1[6] -= s2[6];
    s1[7] -= s2[7];
    s1[8] -= s2[8];
}

//------------------------------------------------------------------------------
// Function:    mulScalar2Vector
// Summary: Multiply a scalar to a vector.
// In:      s   -> scalar
//          v   -> vector
// Out:     v   -> result
// Return:  None
//------------------------------------------------------------------------------
static inline void mulScalar2Vector(double s, double (&v)[2])
{
    v[0] *= s;
    v[1] *= s;
}
static inline void mulScalar2Vector(double s, double (&v)[3])
{
    v[0] *= s;
    v[1] *= s;
    v[2] *= s;
}
static inline void mulScalar2Vector(double s, double (&v)[4])
{
    v[0] *= s;
    v[1] *= s;
    v[2] *= s;
    v[3] *= s;
}
static inline void mulScalar2Vector(double s, double (&v)[5])
{
    v[0] *= s;
    v[1] *= s;
    v[2] *= s;
    v[3] *= s;
    v[4] *= s;
}
static inline void mulScalar2Vector(double s, double (&v)[6])
{
    v[0] *= s;
    v[1] *= s;
    v[2] *= s;
    v[3] *= s;
    v[4] *= s;
    v[5] *= s;
}
static inline void mulScalar2Vector(double s, double (&v)[7])
{
    v[0] *= s;
    v[1] *= s;
    v[2] *= s;
    v[3] *= s;
    v[4] *= s;
    v[5] *= s;
    v[6] *= s;
}
static inline void mulScalar2Vector(double s, double (&v)[8])
{
    v[0] *= s;
    v[1] *= s;
    v[2] *= s;
    v[3] *= s;
    v[4] *= s;
    v[5] *= s;
    v[6] *= s;
    v[7] *= s;
}
static inline void mulScalar2Vector(double s, double (&v)[9])
{
    v[0] *= s;
    v[1] *= s;
    v[2] *= s;
    v[3] *= s;
    v[4] *= s;
    v[5] *= s;
    v[6] *= s;
    v[7] *= s;
    v[8] *= s;
}

//------------------------------------------------------------------------------
// Function:    mulMatrix2Vector
// Summary: Multiply a 4*4 matrix to a vector, and put the result to another vector.
// In:      m   -> 4*4 matrix
//          v   -> vector
// Out:     res -> result vector
// Return:  None
//------------------------------------------------------------------------------
static inline void mulMatrix2Vector(const double (&m)[4][4], const double (&v)[4], double (&res)[4])
{
    res[0] = m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2] + m[0][3] * v[3];
    res[1] = m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2] + m[1][3] * v[3];
    res[2] = m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2] + m[2][3] * v[3];
    res[3] = m[3][0] * v[0] + m[3][1] * v[1] + m[3][2] * v[2] + m[3][3] * v[3];
}

//------------------------------------------------------------------------------
// Function:    mulMatrix2Vector
// Summary: Multiply a 4*4 matrix to a vector, and put the result to the vector.
// In:      m   -> 4*4 matrix
//          v   -> vector
// Out:     v   -> result
// Return:  None
//------------------------------------------------------------------------------
static inline void mulMatrix2Vector(const double (&m)[4][4], double (&v)[4])
{
    double v0 = m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2] + m[0][3] * v[3];
    double v1 = m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2] + m[1][3] * v[3];
    double v2 = m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2] + m[2][3] * v[3];
    v[3] = m[3][0] * v[0] + m[3][1] * v[1] + m[3][2] * v[2] + m[3][3] * v[3];
    v[2] = v2;
    v[1] = v1;
    v[0] = v0;
}

//------------------------------------------------------------------------------
// Function:    rightMulMatrix2Matrix
// Summary: Multiply a 4*4 matrix to a 4*4 matrix, and put the result to the first matrix.
//          This is a right-multiply of matrix: m = m * n
// In:      m   -> 4*4 matrix
//          n   -> 4*4 matrix
// Out:     m   -> result
// Return:  None
//------------------------------------------------------------------------------
static inline void rightMulMatrix2Matrix(double (&m)[4][4], const double (&n)[4][4])
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
// Function:    leftMulMatrix2Matrix
// Summary: Multiply a 4*4 matrix to a 4*4 matrix, and put the result to the second matrix.
//          This is a left-multiply of matrix: n = m * n
// In:      m   -> 4*4 matrix
//          n   -> 4*4 matrix
// Out:     n   -> result
// Return:  None
//------------------------------------------------------------------------------
static inline void leftMulMatrix2Matrix(const double (&m)[4][4], double (&n)[4][4])
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
// Function:    mulMatrix2Matrix
// Summary: Multiply a 4*4 matrix to a 4*4 matrix, and put the result to another matrix.
// In:      m   -> 4*4 matrix
//          n   -> 4*4 matrix
// Out:     res -> result matrix
// Return:  None
//------------------------------------------------------------------------------
static inline void mulMatrix2Matrix(const double (&m)[4][4], const double (&n)[4][4], double (&res)[4][4])
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
// Function:    norm
// Summary: Get the norm of a vector.
// In:      v   -> vector
// Out:     None
// Return:  norm of the input vector
//------------------------------------------------------------------------------
static inline double norm(const double (&v)[2])
{
    return sqrt(v[0] * v[0] + v[1] * v[1]);
}
static inline double norm(const double (&v)[3])
{
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}
static inline double norm(const double (&v)[4])
{
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3]);
}
static inline double norm(const double (&v)[5])
{
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3] + v[4] * v[4]);
}
static inline double norm(const double (&v)[6])
{
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3] + v[4] * v[4] + v[5] * v[5]);
}
static inline double norm(const double (&v)[7])
{
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3] + v[4] * v[4] + v[5] * v[5] + v[6] * v[6]);
}
static inline double norm(const double (&v)[8])
{
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3] + v[4] * v[4] + v[5] * v[5] + v[6] * v[6] + v[7] * v[7]);
}
static inline double norm(const double (&v)[9])
{
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3] + v[4] * v[4] + v[5] * v[5] + v[6] * v[6] + v[7] * v[7] + v[8] * v[8]);
}

//------------------------------------------------------------------------------
// Function:    innerProduct
// Summary: Get the inner product of two vectors.
// In:      v   -> first vector
//          u   -> second vector
// Out:     None
// Return:  inner product of two vectors
//------------------------------------------------------------------------------
static inline double innerProduct(const double (&v)[2], const double (&u)[2])
{
    return v[0] * u[0] + v[1] * u[1];
}
static inline double innerProduct(const double (&v)[3], const double (&u)[3])
{
    return v[0] * u[0] + v[1] * u[1] + v[2] * u[2];
}
static inline double innerProduct(const double (&v)[4], const double (&u)[4])
{
    return v[0] * u[0] + v[1] * u[1] + v[2] * u[2] + v[3] * u[3];
}
static inline double innerProduct(const double (&v)[5], const double (&u)[5])
{
    return v[0] * u[0] + v[1] * u[1] + v[2] * u[2] + v[3] * u[3] + v[4] * u[4];
}
static inline double innerProduct(const double (&v)[6], const double (&u)[6])
{
    return v[0] * u[0] + v[1] * u[1] + v[2] * u[2] + v[3] * u[3] + v[4] * u[4] + v[5] * u[5];
}
static inline double innerProduct(const double (&v)[7], const double (&u)[7])
{
    return v[0] * u[0] + v[1] * u[1] + v[2] * u[2] + v[3] * u[3] + v[4] * u[4] + v[5] * u[5] + v[6] * u[6];
}
static inline double innerProduct(const double (&v)[8], const double (&u)[8])
{
    return v[0] * u[0] + v[1] * u[1] + v[2] * u[2] + v[3] * u[3] + v[4] * u[4] + v[5] * u[5] + v[6] * u[6] + v[7] * u[7];
}
static inline double innerProduct(const double (&v)[9], const double (&u)[9])
{
    return v[0] * u[0] + v[1] * u[1] + v[2] * u[2] + v[3] * u[3] + v[4] * u[4] + v[5] * u[5] + v[6] * u[6] + v[7] * u[7] + v[8] * u[8];
}

//------------------------------------------------------------------------------
// Function:    crossProduct
// Summary: Get the cross product of two vectors, and put the result to another vector.
// In:      v   -> first vector
//          u   -> second vector
// Out:     res -> result vector
// Return:  None
//------------------------------------------------------------------------------
static inline void crossProduct(const double (&v)[3], const double (&u)[3], double (&res)[3])
{
    res[0] = v[1] * u[2] - u[1] * v[2];
    res[1] = v[2] * u[0] - u[2] * v[0];
    res[2] = v[0] * u[1] - u[0] * v[1];
}

static inline double getAngleBetweenTwoVectors(const double(&v)[3], const double(&u)[3])
{
	double axb[3];
	crossProduct(v, u, axb);
	return atan2(norm(axb), innerProduct(v, u));
}

//------------------------------------------------------------------------------
// Function:    eye
// Summary: Reset the matrix to identity matrix.
// In:      None
// Out:     m   -> identity matrix
// Return:  None
//------------------------------------------------------------------------------
static inline void eye(double (&m)[3][3])
{
    m[0][0] = 1.0;  m[0][1] = 0.0;  m[0][2] = 0.0;
    m[1][0] = 0.0;  m[1][1] = 1.0;  m[1][2] = 0.0;
    m[2][0] = 0.0;  m[2][1] = 0.0;  m[2][2] = 1.0;
}
static inline void eye(double (&m)[4][4])
{
    m[0][0] = 1.0;  m[0][1] = 0.0;  m[0][2] = 0.0;  m[0][3] = 0.0;
    m[1][0] = 0.0;  m[1][1] = 1.0;  m[1][2] = 0.0;  m[1][3] = 0.0;
    m[2][0] = 0.0;  m[2][1] = 0.0;  m[2][2] = 1.0;  m[2][3] = 0.0;
    m[3][0] = 0.0;  m[3][1] = 0.0;  m[3][2] = 0.0;  m[3][3] = 1.0;
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
static inline void trans(double x, double y, double z, double (&m)[4][4])
{
    m[0][0] = 1.0;  m[0][1] = 0.0;  m[0][2] = 0.0;  m[0][3] = x;
    m[1][0] = 0.0;  m[1][1] = 1.0;  m[1][2] = 0.0;  m[1][3] = y;
    m[2][0] = 0.0;  m[2][1] = 0.0;  m[2][2] = 1.0;  m[2][3] = z;
    m[3][0] = 0.0;  m[3][1] = 0.0;  m[3][2] = 0.0;  m[3][3] = 1.0;
}

//------------------------------------------------------------------------------
// Function:    rotateX
// Summary: Rotate the matrix with t rad, around X.
// In:      m   -> matrix
//          t   -> rotate angle
// Out:     m   -> output matrix
// Return:  None
//------------------------------------------------------------------------------
static inline void rotateX(double t, double (&m)[4][4])
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
// Function:    rotateY
// Summary: Rotate the matrix with t rad, around Y.
// In:      m   -> matrix
//          t   -> rotate angle
// Out:     m   -> output matrix
// Return:  None
//------------------------------------------------------------------------------
static inline void rotateY(double t, double (&m)[4][4])
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
// Function:    rotateZ
// Summary: Rotate the matrix with t rad, around Z.
// In:      m   -> matrix
//          t   -> rotate angle
// Out:     m   -> output matrix
// Return:  None
//------------------------------------------------------------------------------
static inline void rotateZ(double t, double (&m)[4][4])
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

static inline void normalizeQuaternion(Quaternion &q)
{
    double norm = sqrt(q.w_ * q.w_ + q.x_ * q.x_ + q.y_ * q.y_ + q.z_ * q.z_);

    q.w_ /= norm;
    q.x_ /= norm;
    q.y_ /= norm;
    q.z_ /= norm;
}

//------------------------------------------------------------------------------
// Function:    PoseEuler2Matrix
// Summary: Convert a pose-euler to a 4*4 matrix.
// In:      pose -> pose euler need to be converted
// Out:     m    -> matrix output
// Return:  None
//------------------------------------------------------------------------------
static inline void PoseEuler2Matrix(const PoseEuler &pose, double (&m)[4][4])
{
    trans(pose.point_.x_, pose.point_.y_, pose.point_.z_, m);
    rotateZ(pose.euler_.a_, m);
    rotateY(pose.euler_.b_, m);
    rotateX(pose.euler_.c_, m);
}

//------------------------------------------------------------------------------
// Function:    Matrix2PoseEuler
// Summary: Convert a 4*4 matrix to pose-euler.
// In:      m    -> matrix need to be converted
// Out:     pose -> pose-euler output
// Return:  None
//------------------------------------------------------------------------------
static inline void Matrix2PoseEuler(const double (&m)[4][4], PoseEuler &pose)
{
    pose.point_.x_ = m[0][3];
    pose.point_.y_ = m[1][3];
    pose.point_.z_ = m[2][3];

    pose.euler_.b_ = atan2(-m[2][0], sqrt(m[0][0] * m[0][0] + m[1][0] * m[1][0]));

    if (m[2][1] * m[2][1] + m[2][2] * m[2][2] > MINIMUM_E12)
    {
        double cosb = cos(pose.euler_.b_);
        pose.euler_.a_ = atan2(m[1][0] / cosb, m[0][0] / cosb);
        pose.euler_.c_ = atan2(m[2][1] / cosb, m[2][2] / cosb);
    }
    else
    {
        pose.euler_.a_ = atan2(-m[0][1], m[1][1]);
        pose.euler_.c_ = 0.0;
    }
}

//------------------------------------------------------------------------------
// Function:    Pose2Matrix
// Summary: Convert a pose to a 4*4 matrix.
// In:      pose -> pose need to be converted
// Out:     m    -> matrix output
// Return:  None
//------------------------------------------------------------------------------
static inline void Pose2Matrix(const PoseQuaternion &pose, double (&m)[4][4])
{
    Quaternion q(pose.quaternion_);

    if (fabs(sqrt(q.w_ * q.w_ + q.x_ * q.x_ + q.y_ * q.y_ + q.z_ * q.z_) - 1) > 0.0005)
    {
        normalizeQuaternion(q);
    }

    double ss = q.w_ * q.w_;
    double aa = q.x_ * q.x_;
    double bb = q.y_ * q.y_;
    double cc = q.z_ * q.z_;
    double sa = q.w_ * q.x_;
    double sb = q.w_ * q.y_;
    double sc = q.w_ * q.z_;
    double ab = q.x_ * q.y_;
    double ac = q.x_ * q.z_;
    double bc = q.y_ * q.z_;
    double sum = ss + aa + bb + cc;

    ss /= sum; aa /= sum; bb /= sum; cc /= sum;
    sa /= sum; sb /= sum; sc /= sum;
    ab /= sum; ac /= sum; bc /= sum;
    m[0][0] = 2 * (ss + aa) - 1;
    m[0][1] = 2 * (ab - sc);
    m[0][2] = 2 * (sb + ac);
    m[0][3] = pose.point_.x_;
    m[1][0] = 2 * (ab + sc);
    m[1][1] = 2 * (ss + bb) - 1;
    m[1][2] = 2 * (bc - sa);
    m[1][3] = pose.point_.y_;
    m[2][0] = 2 * (ac - sb);
    m[2][1] = 2 * (sa + bc);
    m[2][2] = 2 * (ss + cc) - 1;
    m[2][3] = pose.point_.z_;
    m[3][0] = 0.0;
    m[3][1] = 0.0;
    m[3][2] = 0.0;
    m[3][3] = 1.0;

}

//------------------------------------------------------------------------------
// Function:    Matrix2Pose
// Summary: Convert a 4*4 matrix to pose.
// In:      m    -> matrix need to be converted
// Out:     pose -> pose output
// Return:  None
//------------------------------------------------------------------------------
static inline void Matrix2Pose(const double (&m)[4][4], PoseQuaternion &pose)
{
    pose.point_.x_ = m[0][3];
    pose.point_.y_ = m[1][3];
    pose.point_.z_ = m[2][3];

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
        case 0:
            x = (m[2][1] - m[1][2]) > 0 ? x : -x;
            y = (m[0][2] - m[2][0]) > 0 ? y : -y;
            z = (m[1][0] - m[0][1]) > 0 ? z : -z;
            break;
        case 1:
            w = (m[2][1] - m[1][2]) > 0 ? w : -w;
            y = (m[1][0] + m[0][1]) > 0 ? y : -y;
            z = (m[2][0] + m[0][2]) > 0 ? z : -z;
            break;
        case 2:
            w = (m[0][2] - m[2][0]) > 0 ? w : -w;
            x = (m[1][0] + m[0][1]) > 0 ? x : -x;
            z = (m[2][1] + m[1][2]) > 0 ? z : -z;
            break;
        case 3:
            w = (m[1][0] - m[0][1]) > 0 ? w : -w;
            x = (m[2][0] + m[0][2]) > 0 ? x : -x;
            y = (m[2][1] + m[1][2]) > 0 ? y : -y;
            break;
    }

    double norm = sqrt(w * w + x * x + y * y + z * z);

    //四元数强制单位化
    pose.quaternion_.w_ = w / norm;
    pose.quaternion_.x_ = x / norm;
    pose.quaternion_.y_ = y / norm;
    pose.quaternion_.z_ = z / norm;
}

//------------------------------------------------------------------------------
// Function:    Pose2PoseEuler
// Summary: Convert a pose to pose-euler.
// In:      pose -> pose need to be converted
// Out:     pe   -> pose-euler output
// Return:  true -> success
//          false-> fail, input quaternion illegal
//------------------------------------------------------------------------------
static inline void Pose2PoseEuler(const PoseQuaternion &pose, PoseEuler &pe)
{
    Quaternion q(pose.quaternion_);

    if (fabs(sqrt(q.w_ * q.w_ + q.x_ * q.x_ + q.y_ * q.y_ + q.z_ * q.z_) - 1) > 0.0005)
    {
        normalizeQuaternion(q);
    }

    pe.point_ = pose.point_;
    pe.euler_.c_ = atan2((q.w_ * q.x_ + q.y_ * q.z_) * 2, 1 - (q.x_ * q.x_ + q.y_ * q.y_) * 2);
    pe.euler_.b_ = asin((q.w_ * q.y_ - q.z_ * q.x_) * 2);
    pe.euler_.a_ = atan2((q.w_ * q.z_ + q.x_ * q.y_) * 2, 1 - (q.y_ * q.y_ + q.z_ * q.z_) * 2);
}

static inline PoseEuler Pose2PoseEuler(const PoseQuaternion &pose)
{
    PoseEuler pe;
    Quaternion q(pose.quaternion_);

    if (fabs(sqrt(q.w_ * q.w_ + q.x_ * q.x_ + q.y_ * q.y_ + q.z_ * q.z_) - 1) > 0.0005)
    {
        normalizeQuaternion(q);
    }

    pe.point_ = pose.point_;
    pe.euler_.c_ = atan2((q.w_ * q.x_ + q.y_ * q.z_) * 2, 1 - (q.x_ * q.x_ + q.y_ * q.y_) * 2);
    pe.euler_.b_ = asin((q.w_ * q.y_ - q.z_ * q.x_) * 2);
    pe.euler_.a_ = atan2((q.w_ * q.z_ + q.x_ * q.y_) * 2, 1 - (q.y_ * q.y_ + q.z_ * q.z_) * 2);

    return pe;
}

//------------------------------------------------------------------------------
// Function:    PoseEuler2Pose
// Summary: Convert a pose-euler to pose.
// In:      pe   -> pose-euler needed to be converted
// Out:     pose -> pose output
// Return:  None
//------------------------------------------------------------------------------
static inline void PoseEuler2Pose(const PoseEuler &pe, PoseQuaternion &pose)
{
    pose.point_ = pe.point_;

    double ca = cos(pe.euler_.a_ / 2);
    double cb = cos(pe.euler_.b_ / 2);
    double cc = cos(pe.euler_.c_ / 2);
    double sa = sin(pe.euler_.a_ / 2);
    double sb = sin(pe.euler_.b_ / 2);
    double sc = sin(pe.euler_.c_ / 2);

    pose.quaternion_.w_ = ca * cb * cc + sa * sb * sc;
    pose.quaternion_.z_ = sa * cb * cc - ca * sb * sc;
    pose.quaternion_.y_ = ca * sb * cc + sa * cb * sc;
    pose.quaternion_.x_ = ca * cb * sc - sa * sb * cc;

    int index = 0;
    double max = fabs(pose.quaternion_.w_);

    if (fabs(pose.quaternion_.z_) > max) { index = 1; max = fabs(pose.quaternion_.z_); }
    if (fabs(pose.quaternion_.y_) > max) { index = 2; max = fabs(pose.quaternion_.y_); }
    if (fabs(pose.quaternion_.x_) > max) { index = 3; max = fabs(pose.quaternion_.x_); }

    if ((index == 0 && pose.quaternion_.w_ < 0) || (index == 1 && pose.quaternion_.z_ < 0) || (index == 2 && pose.quaternion_.y_ < 0) || (index == 3 && pose.quaternion_.x_ < 0))
    {
        pose.quaternion_.w_ = -pose.quaternion_.w_;
        pose.quaternion_.z_ = -pose.quaternion_.z_;
        pose.quaternion_.y_ = -pose.quaternion_.y_;
        pose.quaternion_.x_ = -pose.quaternion_.x_;
    }
}

//------------------------------------------------------------------------------
// Function:    PoseEuler2Pose
// Summary: Convert a pose-euler to pose.
// In:      pe   -> pose-euler needed to be converted
// Out:     None
// Return   pose
//------------------------------------------------------------------------------
static inline PoseQuaternion  PoseEuler2Pose(const PoseEuler &pe)
{
    PoseQuaternion pose;
    pose.point_ = pe.point_;

    double ca = cos(pe.euler_.a_ / 2);
    double cb = cos(pe.euler_.b_ / 2);
    double cc = cos(pe.euler_.c_ / 2);
    double sa = sin(pe.euler_.a_ / 2);
    double sb = sin(pe.euler_.b_ / 2);
    double sc = sin(pe.euler_.c_ / 2);

    pose.quaternion_.w_ = ca * cb * cc + sa * sb * sc;
    pose.quaternion_.z_ = sa * cb * cc - ca * sb * sc;
    pose.quaternion_.y_ = ca * sb * cc + sa * cb * sc;
    pose.quaternion_.x_ = ca * cb * sc - sa * sb * cc;

    int index = 0;
    double max = fabs(pose.quaternion_.w_);

    if (fabs(pose.quaternion_.z_) > max) { index = 1; max = fabs(pose.quaternion_.z_); }
    if (fabs(pose.quaternion_.y_) > max) { index = 2; max = fabs(pose.quaternion_.y_); }
    if (fabs(pose.quaternion_.x_) > max) { index = 3; max = fabs(pose.quaternion_.x_); }

    if ((index == 0 && pose.quaternion_.w_ < 0) || (index == 1 && pose.quaternion_.z_ < 0) || (index == 2 && pose.quaternion_.y_ < 0) || (index == 3 && pose.quaternion_.x_ < 0))
    {
        pose.quaternion_.w_ = -pose.quaternion_.w_;
        pose.quaternion_.z_ = -pose.quaternion_.z_;
        pose.quaternion_.y_ = -pose.quaternion_.y_;
        pose.quaternion_.x_ = -pose.quaternion_.x_;
    }

    return pose;
}

static inline Quaternion Euler2Quaternion(const Euler &euler)
{
    Quaternion quaternion;
    double ca = cos(euler.a_ / 2);
    double cb = cos(euler.b_ / 2);
    double cc = cos(euler.c_ / 2);
    double sa = sin(euler.a_ / 2);
    double sb = sin(euler.b_ / 2);
    double sc = sin(euler.c_ / 2);

    quaternion.w_ = ca * cb * cc + sa * sb * sc;
    quaternion.z_ = sa * cb * cc - ca * sb * sc;
    quaternion.y_ = ca * sb * cc + sa * cb * sc;
    quaternion.x_ = ca * cb * sc - sa * sb * cc;

    int index = 0;
    double max = fabs(quaternion.w_);

    if (fabs(quaternion.z_) > max) { index = 1; max = fabs(quaternion.z_); }
    if (fabs(quaternion.y_) > max) { index = 2; max = fabs(quaternion.y_); }
    if (fabs(quaternion.x_) > max) { index = 3; max = fabs(quaternion.x_); }

    if ((index == 0 && quaternion.w_ < 0) || (index == 1 && quaternion.z_ < 0) || (index == 2 && quaternion.y_ < 0) || (index == 3 && quaternion.x_ < 0))
    {
        quaternion.w_ = -quaternion.w_;
        quaternion.z_ = -quaternion.z_;
        quaternion.y_ = -quaternion.y_;
        quaternion.x_ = -quaternion.x_;
    }

    return quaternion;
}

static inline Euler Quaternion2Euler(const Quaternion &quaternion)
{
    Euler euler;
    Quaternion q(quaternion);

    if (fabs(sqrt(q.w_ * q.w_ + q.x_ * q.x_ + q.y_ * q.y_ + q.z_ * q.z_) - 1) > 0.0005)
    {
        normalizeQuaternion(q);
    }
    
    euler.c_ = atan2((q.w_ * q.x_ + q.y_ * q.z_) * 2, 1 - (q.x_ * q.x_ + q.y_ * q.y_) * 2);
    euler.b_ = asin((q.w_ * q.y_ - q.z_ * q.x_) * 2);
    euler.a_ = atan2((q.w_ * q.z_ + q.x_ * q.y_) * 2, 1 - (q.y_ * q.y_ + q.z_ * q.z_) * 2);
    return euler;
}

//------------------------------------------------------------------------------
// Function:    det
// Summary: To return value of the determinant.
// In:      m   -> matrix needed to compute determinant
// Out:     None
// Return:  the value of determinant
//------------------------------------------------------------------------------
inline double det(const double (&m)[4][4])
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
static inline bool inverseMatrix(double (&m)[4][4])
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


static inline void DHMatrix(const double (&l)[4], double q, double (&m)[4][4])
{
    double tmp[4][4];
    eye(m);
    rotateX(l[0], m);
    trans(l[1], 0, 0, tmp);
    rightMulMatrix2Matrix(m, tmp);
    rotateZ(l[3] + q, m);
    trans(0, 0, l[2], tmp);
    rightMulMatrix2Matrix(m, tmp);
}

static inline double getDistance(const Point &p1, const Point &p2)
{
    double x = p2.x_ - p1.x_;
    double y = p2.y_ - p1.y_;
    double z = p2.z_ - p1.z_;

    return sqrt(x * x + y * y + z * z);
}

static inline double getDistance(const PoseQuaternion &pose1, const PoseQuaternion &pose2)
{
    double x = pose2.point_.x_ - pose1.point_.x_;
    double y = pose2.point_.y_ - pose1.point_.y_;
    double z = pose2.point_.z_ - pose1.point_.z_;

    return sqrt(x * x + y * y + z * z);
}

static inline double innerProductQuatern(const Quaternion q1, const Quaternion q2)
{
    return q1.w_ * q2.w_ + q1.x_ * q2.x_ + q1.y_ * q2.y_ + q1.z_ * q2.z_;
}

static inline double getOrientationAngle(const Quaternion &q1, const Quaternion &q2)
{
    double t = innerProductQuatern(q1, q2);

    if (t > 1.0)        t =  1.0;
    else if (t < -1.0)  t = -1.0;

    return acos(t);
}

static inline double getOrientationAngle(const Euler &e1, const Euler &e2)
{
    double t = innerProductQuatern(Euler2Quaternion(e1), Euler2Quaternion(e2));

    if (t > 1.0)        t =  1.0;
    else if (t < -1.0)  t = -1.0;

    return acos(t);
}

static inline double getOrientationAngle(const PoseQuaternion &pose1, const PoseQuaternion &pose2)
{
    return getOrientationAngle(pose1.quaternion_, pose2.quaternion_);
}

static inline double maxInArray(double *array, int length)
{
    double max = array[0];

    for (int i = 1; i < length; i++)
    {
        if (array[i] > max)
        {
            max = array[i];
        }
    }

    return max;
}

static inline double minInArray(double *array, int length)
{
    double min = array[0];

    for (int i = 1; i < length; i++)
    {
        if (array[i] < min)
        {
            min = array[i];
        }
    }

    return min;
}

static inline void point2Double(const Point &p, double (&array)[3])
{
    array[0] = p.x_;
    array[1] = p.y_;
    array[2] = p.z_;
}


static inline void quaternion2Double(const Quaternion &q, double (&array)[4])
{
    array[0] = q.x_;
    array[1] = q.y_;
    array[2] = q.z_;
    array[3] = q.w_;
}

static inline void getSepticSpline(double p0, double v0, double a0, double j0, double p1, double v1, double a1, double j1, double duration, double (&coeff)[8])
{
    double h = p1 - p0;
    double t1 = duration;
    double t2 = t1 * duration;
    double t3 = t2 * duration;
    double t4 = t3 * duration;
    double t5 = t4 * duration;
    double t6 = t5 * duration;
    double t7 = t6 * duration;

    coeff[0] = p0;
    coeff[1] = v0;
    coeff[2] = a0 / 2;
    coeff[3] = j0 / 6;
    coeff[4] = (h * 210 - t1 * (t1 * (a0 * 30 - a1 * 15) + t2 * (j0 * 4 + j1) + v0 * 120 + v1 * 90)) / (t4 * 6);
    coeff[5] = (t1 * (t1 * (a0 * 20 - a1 * 14) + t2 * (j0 * 2 + j1) + v0 * 90 + v1 * 78) - h * 168) / (t5 * 2);
    coeff[6] = (h * 420 - t1 * (t1 * (a0 * 45 - a1 * 39) + t2 * (j0 * 4 + j1 * 3) + v0 * 216 + v1 * 204)) / (t6 * 6);
    coeff[7] = (t1 * (t1 * (a0 * 12 - a1 * 12) + t2 * (j0 + j1) + v0 * 60 + v1 * 60) - h * 120) / (t7 * 6);
}

static inline void sampleSepticSpline(double t, const double (&coeff)[8], double &p, double &v, double &a, double &j)
{
    double t1 = t;
    double t2 = t1 * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;
    double t6 = t5 * t;
    double t7 = t6 * t;

    p = coeff[0] + 
        coeff[1] * t1 + 
        coeff[2] * t2 + 
        coeff[3] * t3 + 
        coeff[4] * t4 + 
        coeff[5] * t5 + 
        coeff[6] * t6 + 
        coeff[7] * t7;
    v = coeff[1] + 
        coeff[2] * t1 * 2 + 
        coeff[3] * t2 * 3 + 
        coeff[4] * t3 * 4 + 
        coeff[5] * t4 * 5 + 
        coeff[6] * t5 * 6 + 
        coeff[7] * t6 * 7;
    a = coeff[2] * 2 + 
        coeff[3] * t1 * 6 + 
        coeff[4] * t2 * 12 + 
        coeff[5] * t3 * 20 + 
        coeff[6] * t4 * 30 + 
        coeff[7] * t5 * 42;
    j = coeff[3] * 6 + 
        coeff[4] * t1 * 24 + 
        coeff[5] * t2 * 60 + 
        coeff[6] * t3 * 120 + 
        coeff[7] * t4 * 210;
}

static inline  void sampleSlerpInterpolationQuaternion(const Quaternion &start, const Quaternion end, double ratio, double orientation_angle, Quaternion &result)
{
    Quaternion alternative_end;
    alternative_end = end;

    double dot_product = start.x_*end.x_ + start.y_*end.y_ + start.z_*end.z_ + start.w_*end.w_;
    if (dot_product < 0)
    {
        alternative_end.x_ = -end.x_;
        alternative_end.y_ = -end.y_;
        alternative_end.z_ = -end.z_;
        alternative_end.w_ = -end.w_;
    }

    if (orientation_angle < 0.001)
    {
        // ��̬�н�С��0.1rad,��̬���Բ�ֵ
        result.w_ = (1 - ratio) * start.w_ + ratio * alternative_end.w_;
        result.x_ = (1 - ratio) * start.x_ + ratio * alternative_end.x_;
        result.y_ = (1 - ratio) * start.y_ + ratio * alternative_end.y_;
        result.z_ = (1 - ratio) * start.z_ + ratio * alternative_end.z_;
    }
    else
    {
        // ��̬�нǴ���0.1rad,��̬������ֵ
        result.w_ = (sin((1 - ratio) * orientation_angle) * start.w_ + sin(ratio * orientation_angle) * alternative_end.w_) / sin(orientation_angle);
        result.x_ = (sin((1 - ratio) * orientation_angle) * start.x_ + sin(ratio * orientation_angle) * alternative_end.x_) / sin(orientation_angle);
        result.y_ = (sin((1 - ratio) * orientation_angle) * start.y_ + sin(ratio * orientation_angle) * alternative_end.y_) / sin(orientation_angle);
        result.z_ = (sin((1 - ratio) * orientation_angle) * start.z_ + sin(ratio * orientation_angle) * alternative_end.z_) / sin(orientation_angle);
    }
}

inline bool binarySearch(double* datas, int length, double target, int& data_index)
{
	if (length <= 0) return false;
    if (length == 1 && target < datas[0])
    {		
        data_index = 0;		
        return true;	
    }
	int left = 0;	
    int right = length - 1;	
    int mid = 0;	
    while (left <= right)	
    {		
        mid = (right + left) / 2;		

        if (datas[mid - 1] < target && target < datas[mid])		
        {			
            data_index = mid;			
            return true;		
        }
		else if (datas[mid] < target)		
        {			
            left = mid + 1;		
        }
		else if (target < datas[mid - 1])		
        {			
            right = mid - 1;	
    	}	
    }	
	return false;
}

}
#endif


