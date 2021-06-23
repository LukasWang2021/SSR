#ifndef MATRIX33_H
#define MATRIX33_H

/**
 * @file matrix33.h
 * @brief The file is the header file of class "Matrix33".
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
 * @brief Matrix33 is the object to represent a general 3 by 3 matrix.
 * @details Matrix33 has a two-dimensional array to store all 9 element as follows:\n
 * @code
 *        matrix_[0][0]    matrix_[0][1]    matrix_[0][2]
 *        matrix_[1][0]    matrix_[1][1]    matrix_[1][2]
 *        matrix_[2][0]    matrix_[2][1]    matrix_[2][2]
 * @endcode
 */
class Matrix33
{
public:
    double matrix_[3][3]; /**< Store all elements of the matrix.*/

    /**
     * @brief Make the matrix to be an identity matrix.
     * @details Make the matrix to be an identity matrix.\n
     * @return void
     */
    void identity();
    /**
     * @brief Compare if two matrices are equal.
     * @details Compares all 9 elements of the matrix object and comparing one specified by its arguments.\n
     *          The maximum permissible deviations of each element between the matrix object and the comparing one are specified by its arguments.\n
     * @param [in] matrix Another matrix object, used as the comparing object.
     * @param [in] valve The maximum permissible deviations of each element.
     * @retval true All deviations of the 9 elements between the matrix object and the comparing one are under the valve value.
     * @retval false  At least one deviation of the 9 elements between the matrix object and the comparing one is over the valve value.
     */
    bool isEqual(Matrix33& matrix, double valve = 0.001) const;
    /**
     * @brief Do math operation left_matrix * this.
     * @details Do math operation: this = left_matrix * this.\n
     * @param [in] left_matrix Multiplication item on left side.
     * @return The result of the multiplication.
     */
    Matrix33& leftMultiply(Matrix33& left_matrix);
    /**
     * @brief Do math operation left_matrix * this.
     * @details Do math operation: result_matrix = left_matrix * this.\n
     * @param [in] left_matrix Multiplication item on left side.
     * @param [out] result_matrix The result of the multiplication.
     * @return void
     */    
    void leftMultiply(Matrix33& left_matrix, Matrix33& result_matrix);
    /**
     * @brief Do math operation this * right_matrix.
     * @details Do math operation: this = this * right_matrix.\n
     * @param [in] right_matrix Multiplication item on right side.
     * @return The result of the multiplication.
     */    
    Matrix33& rightMultiply(Matrix33& right_matrix);
    /**
     * @brief Do math operation this * right_matrix.
     * @details Do math operation: result_matrix = this * right_matrix.\n
     * @param [in] right_matrix Multiplication item on right side.
     * @param [out] result_matrix The result of the multiplication.
     * @return void
     */ 
    void rightMultiply(Matrix33& right_matrix, Matrix33& result_matrix);
    /**
     * @brief Operator= overload.
     * @details Envalue a Matrix33 object to self.\n
     * @param [in] matrix The source object.
     * @return A reference to self.
     */
    Matrix33& operator=(const Matrix33& matrix);
    /**
    * @brief Print all member variables.
    * @details Print member variables matrix_ in format.\n
    * @param [in] comment First line to print.
    * @return void
    * @par code example:
    * @code 
    *     Matrix33 matrix;
    *     matrix.matrix_[0][0] = 1; matrix.matrix_[0][1] = 0; matrix.matrix_[0][2] = 0;
    *     matrix.matrix_[1][0] = 0; matrix.matrix_[1][1] = 1; matrix.matrix_[1][2] = 0;
    *     matrix.matrix_[2][0] = 0; matrix.matrix_[2][1] = 0; matrix.matrix_[2][2] = 1;     
    *     matrix.print("this is a test:");
    * @endcode
    * @par code result:
    * @code
    *     root>this is a test:
    *     root>1 0 0
    *     root>0 1 0
    *     root>0 0 1
    *     root>
    * @endcode
    */
    void print(std::string comment = "") const;
private:
    /**
     * @brief Do math operation left_matrix * right_matrix.
     * @details Do math operation: result_matrix = left_matrix * right_matrix.\n
     * @param [in] left_matrix Multiplication item on left side.
     * @param [in] right_matrix Multiplication item on right side.
     * @param [out] result_matrix The result of the multiplication.
     * @return void
     */     
    void multiply(Matrix33& left_matrix, Matrix33& right_matrix, Matrix33& result_matrix);
};

}


#endif

