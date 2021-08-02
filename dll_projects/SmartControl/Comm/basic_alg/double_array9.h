#ifndef DOUBLE_ARRAY9_H
#define DOUBLE_ARRAY9_H

/**
 * @file double_array9.h
 * @brief The file is the header file of class "DoubleArray9".
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
 * @brief DoubleArray9 can represent any object that has up to 9 double elements.
 * @details DoubleArray9 includes 9 double elements in it.
 */
class DoubleArray9
{
public:
    double   a0_;   /**< the first element.*/
    double   a1_;   /**< the second element.*/
    double   a2_;   /**< the third element.*/
    double   a3_;   /**< the fourth element.*/
    double   a4_;   /**< the fifth element.*/
    double   a5_;   /**< the sixth element.*/
    double   a6_;   /**< the seventh element.*/
    double   a7_;   /**< the eighth element.*/
    double   a8_;   /**< the ninth element.*/

    /**
     * @brief Make all elements to be zero.
     * @details Let a0_~a8_=0.\n
     * @return void
     */
    void zero();
    /**
     * @brief Compare if two array objects are equal.
     * @details Compares the first N elements of the array object and comparing one specified by its arguments.\n
     *          The maximum permissible deviations of each element between the matrix object and the comparing one are specified by its arguments.\n
     * @param [in] array Another array object, used as the comparing object.
     * @param [in] element_number The number of elements to be compared, start from a0_.
     * @param [in] valve The maximum permissible deviations of each element.
     * @retval true All deviations of the expected elements between the array object and the comparing one are under the valve value.
     * @retval false  At least one deviation of the expected elements between the array object and the comparing one is over the valve value.
     */
    bool isEqual(DoubleArray9& array, size_t element_number, double valve = 0.001) const;

    /**
     * @brief Operator[] overload.
     * @details Get a reference of the member variable a0_~a8_ by its argument.\n
     * @param [in] index The index of the specified member variable. 
     * - Value 0 returns a0_. 
     * - value 1 returns a1_.
     * - value 2 returns a2_.
     * - value 3 returns a3_.
     * - Value 4 returns a4_. 
     * - value 5 returns a5_.
     * - value 6 returns a6_.
     * - value 7 returns a7_.
     * - value 8 returns a8_.
     * - All other values will cause assertion failed.
     * @return A reference of the specified member variable.
     */
    double& operator[](size_t index);
    const double& operator[](size_t index) const;
    /**
     * @brief Operator+ overload.
     * @details Do math operation: this + array.\n
     *          a0_new = a0_ + array.a0_;\n
     *          a1_new = a1_ + array.a1_;\n
     *          a2_new = a2_ + array.a2_;\n   
     *          a3_new = a3_ + array.a3_;\n  
     *          a4_new = a4_ + array.a4_;\n
     *          a5_new = a5_ + array.a5_;\n
     *          a6_new = a6_ + array.a6_;\n   
     *          a7_new = a7_ + array.a7_;\n 
     *          a8_new = a8_ + array.a8_;\n 
     * @param [in] array Array to be added.
     * @return The result of the add operation.
     */    
    const DoubleArray9 operator+(const DoubleArray9& array);
    /**
     * @brief Operator- overload.
     * @details Do math operation: this - array.\n
     *          a0_new = a0_ - array.a0_;\n
     *          a1_new = a1_ - array.a1_;\n
     *          a2_new = a2_ - array.a2_;\n   
     *          a3_new = a3_ - array.a3_;\n  
     *          a4_new = a4_ - array.a4_;\n
     *          a5_new = a5_ - array.a5_;\n
     *          a6_new = a6_ - array.a6_;\n   
     *          a7_new = a7_ - array.a7_;\n    
     *          a8_new = a8_ - array.a8_;\n   
     * @param [in] array Array to be subtracted.
     * @return The result of the subtraction operation.
     */    
    const DoubleArray9 operator-(const DoubleArray9& array);
    /**
     * @brief Operator+= overload.
     * @details Do math operation: this = this + array.\n
     *          this.a0_ = this.a0_ + joint.a0_;\n
     *          this.a1_ = this.a1_ + joint.a1_;\n
     *          this.a2_ = this.a2_ + joint.a2_;\n
     *          this.a3_ = this.a3_ + joint.a3_;\n
     *          this.a4_ = this.a4_ + joint.a4_;\n
     *          this.a5_ = this.a5_ + joint.a5_;\n
     *          this.a6_ = this.a6_ + joint.a6_;\n
     *          this.a7_ = this.a7_ + joint.a7_;\n 
     *          this.a8_ = this.a8_ + joint.a8_;\n
     * @param [in] array Array to be added.
     * @return A reference of self.
     */     
    DoubleArray9& operator+=(const DoubleArray9& array);
    /**
     * @brief Operator- overload.
     * @details Do math operation: this = this - array.\n
     *          this.a0_ = this.a0_ - joint.a0_;\n
     *          this.a1_ = this.a1_ - joint.a1_;\n
     *          this.a2_ = this.a2_ - joint.a2_;\n
     *          this.a3_ = this.a3_ - joint.a3_;\n 
     *          this.a4_ = this.a4_ - joint.a4_;\n
     *          this.a5_ = this.a5_ - joint.a5_;\n
     *          this.a6_ = this.a6_ - joint.a6_;\n
     *          this.a7_ = this.a7_ - joint.a7_;\n 
     *          this.a8_ = this.a8_ - joint.a8_;\n 
     * @param [in] array Array to be subtracted.
     * @return A reference of self.
     */     
    DoubleArray9& operator-=(const DoubleArray9& array);
    /**
    * @brief Print all member variables.
    * @details Print member variables a0_~a8_ in format.\n
    * @param [in] comment First line to print.
    * @return void
    * @par code example:
    * @code 
    *    DoubleArray9 array;
    *    array.a0_ = 0.1;
    *    array.a1_ = 0.2;
    *    array.a2_ = 0.3;
    *    array.a3_ = 0.4;
    *    array.a4_ = 0.5;
    *    array.a5_ = 0.6;
    *    array.a6_ = 0.7;
    *    array.a7_ = 0.8;
    *    array.a8_ = 0.9;
    *    array.print("this is a test:");
    * @endcode
    * @par code result:
    * @code
    *    root>this is a test:
    *    root> a0 = 0.1
    *    root> a1 = 0.2
    *    root> a2 = 0.3
    *    root> a3 = 0.4
    *    root> a4 = 0.5
    *    root> a5 = 0.6
    *    root> a6 = 0.7
    *    root> a7 = 0.8  
    *    root> a8 = 0.9  
    *    root>
    * @endcode
    */
    void print(std::string comment = "") const;

};

}


#endif

