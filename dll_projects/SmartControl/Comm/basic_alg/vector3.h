#ifndef VECTOR3_H
#define VECTOR3_H

/**
 * @file vector3.h
 * @brief The file is the header file of class "Vector3".
 * @author zhengyu.shen
 */

#include <stddef.h>
#include <string>
/**
 * @brief basic_alg includes all foundational mathmatics computation.
 */
namespace basic_alg
{
/**
 * @brief Vector3 is the object to represent a 3 by 1 matrix.
 * @details A Vector3 object can be thought of as a point or a vector in 3D space.\n
 */
class Vector3
{
public:
    double x_;  /**< value in X direction of 3D space.*/
    double y_;  /**< value in Y direction of 3D space.*/
    double z_;  /**< value in Z direction of 3D space.*/
    /**
     * @brief Make all elements to be zero.
     * @details Let x_=0, y_=0, z_=0.\n
     * @return void
     */
    void zero();
    /**
     * @brief Compare if two vectors are equal.
     * @details Compares {x_, y_, z_} of the vector object and comparing point object specified by its arguments.\n
     *          The maximum permissible deviations of {x_, y_, z_} between the point object and the comparing one are specified by its arguments.\n
     * @param [in] vector Another point object, used as the comparing object.
     * @param [in] valve The maximum permissible deviations of {x_, y_, z_}
     * @retval true All deviations of {x_, y_, z_} between the vector object and the comparing one are under the valve value.
     * @retval false  At least one deviation of {x_, y_, z_} between the vector object and the comparing one is over the valve value.
     */
    bool isEqual(Vector3& vector, double valve = 0.001) const;
    /**
     * @brief Check if two vectors are parallel.
     * @details It is difficult to make a judgment of parallization when the norm of the two vectors are tiny small.\n
     *           In consequence, valve value is used to deal with the problem under such conditions.\n
     * @param [in] vector Another vector object, used as the comparing object.
     * @param [in] valve Valve value of making judgement.
     * @retval true Self is parallel with the given vector object.
     * @retval false Self is not parallel with the given vector object.
     */
    bool isParallel(Vector3& vector, double valve = 0.001) const;
    /**
     * @brief Check if two vectors are vertical.
     * @details The dot product of the two vectors should be zero if they are vertical.\n
     * @param [in] vector Another vector object, used as the comparing object.
     * @param [in] valve Valve value of making judgement.
     * @retval true Self is vertical to the given vector object.
     * @retval false Self is not vertical to the given vector object.
     */
    bool isVertical(Vector3& vector, double valve = 0.001) const;
    /**
    * @brief Reverse the value of {x_, y_, z_}.
    * @details Reverse all member variables of the vector object. Let x_=-x_, y_=-y_, z_=-z_.\n
    * @return void
    */
    void reverse();
    /**
     * @brief Compute the distance between two points.
     * @details Compute the distance between self and the point specialized by its argument.\n
     * @param [in] point Another point object.
     * @return The distance of the two points.
     */    
    double distanceToPoint(Vector3& point) const;
    /**
     * @brief Compute the point on the line across this point and given point.
     * @details Compute the the point on the line across this point and given point.\n
     * @param [in] end_point Another point object.
     * @param [in] distance_to_start The distance between the expected point and this point,\n
                   positive value means the direction torwards to the end point, \n
                   negative values means the direction backwards to the end point.\n
     * @return The expected point.
     */ 
    Vector3 getPointBetween(Vector3& end_point, double distance_to_start);
    /**
     * @brief Compute the norm of the vector.
     * @details The norm of the vector is given by sqrt(x_ * x_ + y_ * y_ + z_ * z_).\n
     * @return The norm of self.
     */    
    double norm() const;
    /**
     * @brief Normalize self to a unit vector.
     * @details A unit vector implies "x_ * x_ + y_ * y_ + z_ * z_ = 1" holds.\n
                 If the norm of a vector is zero, it is not possible to be normalized.
     * @param valve Valve value used to judge if the norm of a vector is zero.
     * @retval true Normalization is success.
     * @retval false Normalization is failed.
     */    
    bool normalize(double valve = 0.001);
    /**
     * @brief Dot product between two vectors. 
     * @details Compute the dot product value of self*point.\n
     * @param [in] vector Another vector object.
     * @return The result of dot product.
     */    
    double dotProduct(const Vector3& vector) const;
    /**
     * @brief Cross product between two vectors. 
     * @details Do cross product operation: result = self.*by.\n
     * @param [in] by Another vector object.
     * @param [out] result Result of cross product, 3 by 1 vector.
     * @return void
     */     
    void crossProduct(const Vector3& by, Vector3& result);
    /**
     * @brief Cross product between two vectors. 
     * @details Do cross product operation: self = self.*by.\n
     * @param [in] by Another vector object.
     * @return void
     */
    void crossProduct(const Vector3& by);
    /**
     * @brief Operator[] overload.
     * @details Get a reference of the member variable x_ or y_ or z_ by its argument.\n
     * @param [in] index The index of the specified member variable. 
     * - Value 0 returns x_. 
     * - value 1 returns y_.
     * - value 2 returns z_.
     * - All other values will cause assertion failed.
     * @return A reference of the specified member variable.
     */       
    double& operator[](size_t index);
    const double& operator[](size_t index) const;
     /**
     * @brief Operator+ overload.
     * @details Do math operation: this + point.\n
     *          x_new = x_ + point.x_;\n
     *          y_new = y_ + point.y_;\n
     *          z_new = z_ + point.z_;\n
     * @param [in] vector Vector to be added.
     * @return The result of the add operation.
     */
    const Vector3 operator+(const Vector3& vector);
    /**
     * @brief Operator- overload.
     * @details Do math operation: this - point.\n
     *          x_new = x_ - point.x_;\n
     *          y_new = y_ - point.y_;\n
     *          z_new = z_ - point.z_;\n     
     * @param [in] vector Vector to be subtracted.
     * @return The result of the subtraction operation.
     */      
    const Vector3 operator-(const Vector3& vector);    
    /**
     * @brief Operator+= overload.
     * @details Do math operation: this = this + point.\n
     *          this.x_ = this.x_ + point.x_;\n
     *          this.y_ = this.y_ + point.y_;\n
     *          this.z_ = this.z_ + point.z_;\n     
     * @param [in] vector Vector to be added.
     * @return A reference of self.
     */     
    Vector3& operator+=(const Vector3& vector);
    /**
     * @brief Operator-= overload.
     * @details Do math operation: this = this - point.\n
     *          this.x_ = this.x_ - point.x_;\n
     *          this.y_ = this.y_ - point.y_;\n
     *          this.z_ = this.z_ - point.z_;\n     
     * @param [in] vector Vector to be subtracted.
     * @return A reference of self.
     */     
    Vector3& operator-=(const Vector3& vector);
    /**
     * @brief Operator* overload.
     * @details Do math operation: value*this.\n
     *          x_new = value * this.x_;\n
     *          y_new = value * this.y_;\n
     *          z_new = value * this.z_;\n
     * @param [in] value Times to be multiplied.
     * @return The result vector object contains {x_new, y_new, z_new}.
     */     
    const Vector3 operator*(double value);
    /**
    * @brief Print all member variables.
    * @details Print member variables x_,y_,z_ in format.\n
    * @param [in] comment First line to print.
    * @return void
    * @par code example:
    * @code 
    *    Vector3 vector;
    *    vector.x_ = 1;
    *    vector.y_ = 2;
    *    vector.z_ = 3.5;
    *    vector.print("this is a test:");
    * @endcode
    * @par code result:
    * @code
    *    root>this is a test:
    *    root> x = 1
    *    root> y = 2
    *    root> z = 3.5
    *    root>
    * @endcode
    */    
    void print(std::string comment = "") const;
};


}



#endif

