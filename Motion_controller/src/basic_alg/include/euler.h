#ifndef EULER_H
#define EULER_H


#include <stddef.h>
#include "rotation_matrix.h"
#include "quaternion.h"
#include <string>

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
    double a_;
    double b_;
    double c_;
    /**
     * @brief Compare if two orientations expressed in euler angle are equal.
     * @details Compares {a_, b_, c_} of the euler object and comparing euler object specified by its arguments.\n
     *          The maximum permissible deviations of {a_, b_, c_} between the euler object and the comparing one are specified by its arguments.\n
     * @param [in] euler Another euler object, used as the comparing object.
     * @param [in] valve The maximum permissible deviations of {a_, b_, c_}
     * @retval true All deviations of {a_, b_, c_} between the euler object and the comparing one are under the valve value.
     * @retval false  At least one deviation of {a_, b_, c_} between the euler object and the comparing one is over the valve value.
     */
    bool isEqual(const Euler& euler, double valve = 0.001) const;
    bool isEquivalent(const Euler& euler, double valve = 0.001) const;
    bool isEquivalent(const Quaternion& quaternion, double valve = 0.001) const;
    double getIncludedAngle(const Euler& euler) const;
    double getIncludedAngle(const Quaternion& quaternion) const;
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
    
    double& operator[](size_t index);
    const double& operator[](size_t index) const;

    void print(std::string comment = "") const;
};


}

#endif

