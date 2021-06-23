#ifndef EULER_H
#define EULER_H

/**
 * @file euler.h
 * @brief The file is the header file of class "Euler".
 * @author zhengyu.shen
 */

#include <stddef.h>
#include "rotation_matrix.h"
#include "quaternion.h"
#include <string>
/**
 * @brief basic_alg includes all foundational mathmatics computation.
 */
namespace basic_alg
{
class Quaternion;
class RotationMatrix;

/**
 * @brief Euler is the object to represent orientation. It includes three member variables {a_, b_, c_}.
 * @details 'a_' is the angle rotate by Z axis; 'b_' is the angle rotate by Y axis; 'c_' is the angle rotate by X axis.\n
 *          The unit of the three angles is radian.\n
 */
class Euler
{
public:
    double a_;  /**< Angle rotate by Z axis, unit in radian*/
    double b_;  /**< Angle rotate by Y axis, unit in radian*/
    double c_;  /**< Angle rotate by X axis, unit in radian*/

     /**
     * @brief Make all elements to be zero.
     * @details Let a_=0, b_=0, c_=0.\n
     * @return void
     */
    void zero();
    /**
     * @brief Compare if two orientations expressed in euler angle are equal.
     * @details Compares {a_, b_, c_} of the euler object and comparing euler object specified by its arguments.\n
     *          The maximum permissible deviations of {a_, b_, c_} between the euler object and the comparing one are specified by its arguments.\n
     * @param [in] euler Another euler object, used as the comparing object.
     * @param [in] valve The maximum permissible deviations of {a_, b_, c_}
     * @retval true All deviations of {a_, b_, c_} between the euler object and the comparing one are under the valve value.
     * @retval false  At least one deviation of {a_, b_, c_} between the euler object and the comparing one is over the valve value.
     */
    bool isEqual(Euler& euler, double valve = 0.001) const;
    /**
     * @brief Convert the euler object to quaternion object.
     * @details Convert the orientation expressed by euler angle to the one expressed by quaternion.\n
     *          A quaternion expression has the form of {x_, y_, z_, w_}.
     * @param [out] quaternion The quaternion object.
     * @return void
     * @see basic_alg::Quaternion
     */   
    void convertToQuaternion(basic_alg::Quaternion& quaternion) const;
    /**
     * @brief Convert the euler object to rotation matrix object.
     * @details Convert the orientation expressed by euler angle to the one expressed by rotation matrix.\n
     *          A rotation matrix is a 3 by 3 matrix in size.\n
     * @param [out] matrix The rotation martrix object.
     * @return void
     * @see basic_alg::RotationMatrix
     */ 
    void convertToRotationMatrix(basic_alg::RotationMatrix& matrix) const;
    /**
     * @brief Operator[] overload.
     * @details Get a reference of the member variable a_ or b_ or c_ by its argument.\n
     * @param [in] index The index of the specified member variable. 
     * - Value 0 returns a_. 
     * - value 1 returns b_.
     * - value 2 returns c_.
     * - All other values will cause assertion failed.
     * @return A reference of the specified member variable.
     */    
    double& operator[](size_t index);
    const double& operator[](size_t index) const;    
    /**
    * @brief Print all member variables.
    * @details Print member variables j1_~j9_ in format.\n
    * @param [in] comment First line to print.
    * @return void
    * @par code example:
    * @code 
    *    Joint joint;
    *    euler.a_ = 0.1;
    *    euler.b_ = 0.2;
    *    euler.c_ = 0.3;
    *    euler.print("this is a test:");
    * @endcode
    * @par code result:
    * @code
    *    root>this is a test:
    *    root> a = 0.1
    *    root> b = 0.2
    *    root> c = 0.3
    *    root>
    * @endcode
    */
    void print(std::string comment = "") const;
};


}

#endif

