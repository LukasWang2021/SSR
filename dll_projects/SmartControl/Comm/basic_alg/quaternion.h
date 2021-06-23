#ifndef QUATERNION_H
#define QUATERNION_H

/**
 * @file quaternion.h
 * @brief The file is the header file of class "Quaternion".
 * @author zhengyu.shen
 */

#include <stddef.h>
#include "euler.h"
#include "rotation_matrix.h"
#include <string>
/**
 * @brief basic_alg includes all foundational mathmatics computation.
 */
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
    double x_;  /**< imaginary part in i direction*/
    double y_;  /**< imaginary part in j direction*/
    double z_;  /**< imaginary part in k direction*/
    double w_;  /**< real part*/
     /**
     * @brief Make the quaternion only have its real part.
     * @details Let x_=0, y_=0, z_=0, w_=1.\n
     * @return void
     */
    void zero();
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
    bool isEqual(Quaternion& quaternion, double valve = 0.001) const;
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
     * @param [in] quaternion The second quaternion object.
     * @return The included angle in radian in range 0~pi.
     */    
    double getIncludedAngle(Quaternion& quaternion) const;
    /**
     * @brief Compute the quaternion between this and the given one.
     * @details Compute the quaternion between this and the given one. 
                The expected quaternion is computed by slerp method.\n
     * @param [in] end_quaternion Another quaternion object.
     * @param [in] angle_to_start The angle of the included angle of the expected one and this\n
                   The value must be in range of [0,included_angle].\n
                   If it is out of the range, the function will return a zero quaternion.\n
     * @return The expected quaternion.
     */ 
    Quaternion getQuaternionBetween(Quaternion& end_quaternion, double angle_to_start);
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
    /**
    * @brief Print all member variables.
    * @details Print member variables {x_, y_, z_, w_} in format.\n
    * @param [in] comment First line to print.
    * @return void
    * @par code example:
    * @code 
    *    Quaternion quaternion;
    *    quaternion.x_ = 0.1;
    *    quaternion.y_ = 0.2;
    *    quaternion.z_ = 0.3;
    *    quaternion.w_ = 0.4;
    *    quaternion.print("this is a test:");
    * @endcode
    * @par code result:
    * @code
    *    root>this is a test:
    *    root> x = 0.1
    *    root> y = 0.2
    *    root> z = 0.3
    *    root> w = 0.4
    *    root>
    * @endcode
    */
    void print(std::string comment = "") const;
};

}



#endif
