#ifndef QUATERNION_H
#define QUATERNION_H

#include <stddef.h>
#include "euler.h"
#include "rotation_matrix.h"
#include <string>

namespace basic_alg
{
class Euler;
class RotationMatrix;

/**
 * @brief Quaternion is the object to represent orientation. It includes four member variables {x_, y_, z_, w_}.
 * @details A quaternion can be expressed in form of q = w_ + x_*i + y_*j + z_*k, where i,j,k are three unit rotation axes vector orthogonal with each other.\n
 *          In consequence, it can be written to {vector_u, w_}, where vector_u = {x_, y_, z_}. vector_u is the imaginary part and w_ is the real part.\n
 *          It makes nonsense to look into each element of {x_, y_, z_, w_}.\n
 * @note The class Quaternion is defined as unit quaternion which implies the equation "x^2 + y^2 + z^2 + w_^2 = 1" holds. 
 */
class Quaternion
{
public:
    double x_;
    double y_;
    double z_;
    double w_;

    /**
     * @brief Check if the quaternion object is a unit quaternion.
     * @details Compute the norm-square of the quaternion by formula (x_^2 + y_^2 + z_^2 + w_^2) and check if the norm-square is close enough to 1.\n
     *          The maximum permissible deviation of the norm-square is specified by its argument.\n
     * @param [in] valve The maximum permissible deviations of norm-square to 1.
     * @retval true The quaternion object is a unit quarternion.
     * @retval false  The quaternion object is not a unit quarternion.
     */
    bool isValid(double valve = 0.001) const;
    /**
     * @brief Compare if two orientations expressed in quaternion are equal.
     * @details Compares {x_, y_, z_, w_} of the quaternion object and comparing quaternion object specified by its arguments.\n
     *          The maximum permissible deviations of {x_, y_, z_, w_} between the quaternion object and the comparing one are specified by its arguments.\n
     * @param [in] quaternion Another quaternion object, used as the comparing object.
     * @param [in] valve The maximum permissible deviations of {x_, y_, z_, w_}
     * @retval true All deviations of {x_, y_, z_, w_} between the quaternion object and the comparing one are under the valve value.
     * @retval false  At least one deviation of {x_, y_, z_, w_} between the quaternion object and the comparing one is over the valve value.
     */
    bool isEqual(const Quaternion& quaternion, double valve = 0.001) const;
    bool isEquivalent(const Euler& euler, double valve = 0.001) const;
    bool isEquivalent(const Quaternion& quaternion, double valve = 0.001) const;
    double norm();//获取四元数的模长
    void zero();
    /**
    * @brief Reverse the quaternion expression.
    * @details Reverse all member variables of the quaternion object. Let x_=-x_, y_=-y_, z_=-z_, w_=-w_.\n
    * @return void
    */
    void reverse();
    /**
     * @brief Convert the quaternion object to euler object.
     * @details Convert the orientation expressed by quaternion to the one expressed by euler angle.\n
     *          An euler expression has the form of {a_, b_, c_}.
     * @param [out] euler The euler object.
     * @return void
     * @see basic_alg::Euler
     */ 
    void convertToEuler(basic_alg::Euler& euler) const;
    /**
     * @brief Convert the quaternion object to rotation matrix object.
     * @details Convert the orientation expressed by quaternion angle to the one expressed by rotation matrix.\n
     *          A rotation matrix is a 3 by 3 matrix in size.\n
     * @param [out] matrix The rotation martrix object.
     * @return void
     * @see basic_alg::RotationMatrix
     */
    void convertToRotationMatrix(basic_alg::RotationMatrix& matrix) const;
    /**
     * @brief Compute the included angle of two quaternions.
     * @details Compute the included angle of the quaternion and the one specified by its argument.\n
     *          The formular to compute the included angle is given by "theta = acos(quaternion1 * quaternion2)".\n
     *          The result always choose the smaller included angle in range of 0~pi/2.\n
     * @param [in] quaternion The second quaternion object.
     * @return The included angle in radian in range 0~pi/2.
     */
    double getIncludedAngle(const Euler& euler) const;
    double getIncludedAngle(const Quaternion& quaternion) const;
    
    /**
     * @brief Operator[] overload.
     * @details Get a reference of the member variable x_ or y_ or z_ or w_ by its argument.\n
     * @param [in] index The index of the specified member variable. 
     * - Value 0 returns x_. 
     * - value 1 returns y_.
     * - value 2 returns z_.
     * - Value 3 returns w_.
     * - All other values will cause assertion failed.
     * @return A reference of the specified member variable.
     */
    double& operator[](size_t index);
    const double& operator[](size_t index) const;
    const Quaternion operator+(const Quaternion& q);
    const Quaternion operator-(const Quaternion& q);
    const Quaternion operator*(double value);
    const Quaternion operator/(double value);
    void print(std::string comment = "") const;
};

}



#endif
