#ifndef ROTATION_MATRIX_H
#define ROTATION_MATRIX_H

#include <stddef.h>
#include "euler.h"
#include "quaternion.h"
#include "point.h"
#include "matrix33.h"

namespace basic_alg
{
class Euler;
class Quaternion;

class RotationMatrix: public Matrix33
{
public:    
    RotationMatrix();
    ~RotationMatrix();

    void initByStandardDh(double alpha, double theta);
    void convertToQuaternion(basic_alg::Quaternion& quaternion) const;
    void convertToEuler(basic_alg::Euler& euler) const; 

    void multiplyByTransVector(Point& trans_vector, Point& result_vector) const;
    void getVectorN(Point& n_vector) const;
    void getVectorS(Point& s_vector) const;
    void getVectorA(Point& a_vector) const;
    
};

}


#endif

