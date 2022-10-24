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
/**
 * @brief RotationMatrix is the object to represent orientation. It is a 3 by 3 matrix.
 * @details RotationMatrix inherits basic_alg::Matrix33. It is a special 3 by 3 matrix marks as M.\n
 *          It can be expressed in form of {N, S, A}, where N, S, A represent the tree colums of the matrix respectively.\n
 *          The rotation matrix has the following characteristics:\n
 *          - the norms of N,S,A are 1.\n
 *          - M^-1 = M^T.\n
 *          - M*M^T = I, where I is a 3 by 3 identity matrix.
 * @see basic_alg::Matrix33
 */
class RotationMatrix: public Matrix33
{
public:
     /**
     * @brief initialize the rotation matrix object by DH parameters.
     * @details The rotation matrix is initialized as follows:
     * @code
     * cos(theta)    -sin(theta)*cos(alpha)     sin(theta)*sin(alpha)
     * sin(theta)     cos(theta)*cos(alpha)    -cos(theta)*sin(alpha)
     *     0                sin(alpha)                cos(alpha)
     * @endcode
     * @param [in] alpha alpha of DH parameters 
     * @param [in] theta theta of DH parameters
     * @return void
     */
    void initByStandardDh(double alpha, double theta);
    /**
     * @brief Convert the rotation matrix object to quaternion object.
     * @details Convert the orientation expressed by rotation matrix to the one expressed by quaternion.\n
     *          A quaternion expression has the form of {x_, y_, z_, w_}.
     * @param [out] quaternion The quaternion object.
     * @return void
     * @see basic_alg::Quaternion
     */ 
    void convertToQuaternion(basic_alg::Quaternion& quaternion) const;
    /**
     * @brief Convert the rotation matrix object to euler object.
     * @details Convert the orientation expressed by rotation matrix to the one expressed by euler angle.\n
     *          An euler expression has the form of {a_, b_, c_}.
     * @param [out] euler The euler object.
     * @return void
     * @see basic_alg::Euler
     */
    void convertToEuler(basic_alg::Euler& euler) const; 
    /**
     * @brief Get the result of RotationMatrix*TransVector
     * @details Let the rotation matrix multiply by a transition vector specified by its argument. 
     * @param [in] trans_vector Transition vector, 3 by 1 in size.
     * @param [out] result_vector A vector, result of the multiplication, 3 by 1 in size.
     * @return void
     */
    void multiplyByTransVector(const Point& trans_vector, Point& result_vector) const;
    /**
     * @brief Get vector N from the rotation matrix.
     * @details Get the first colum of the rotation matrix, 3 by 1 in size.\n
     * @param [out] n_vector The first colum of the rotation matrix.
     * @return void
     */
    void getVectorN(Point& n_vector) const;
    /**
     * @brief Get vector S from the rotation matrix.
     * @details Get the second colum of the rotation matrix, 3 by 1 in size.\n
     * @param [out] s_vector The second colum of the rotation matrix.
     * @return void
     */
    void getVectorS(Point& s_vector) const;
    /**
     * @brief Get vector A from the rotation matrix.
     * @details Get the third colum of the rotation matrix, 3 by 1 in size.\n
     * @param [out] a_vector The third colum of the rotation matrix.
     * @return void
     */ 
    void getVectorA(Point& a_vector) const;

	virtual bool inverse(double valve = 0.001);
    void inverse(RotationMatrix& result_matrix) const;

    //RotationMatrix& operator=(const RotationMatrix& matrix);

};

}


#endif

