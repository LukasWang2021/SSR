#ifndef MATRIX44_H
#define MATRIX44_H

#include <stddef.h>
#include <string>

namespace basic_alg
{
/**
 * @brief Matrix44 is the object to represent a general 4 by 4 matrix.
 * @details Matrix44 has a two-dimensional array to store all 16 element as follows:\n
 * @code
 *        matrix_[0][0]    matrix_[0][1]    matrix_[0][2]    matrix_[0][3]
 *        matrix_[1][0]    matrix_[1][1]    matrix_[1][2]    matrix_[1][3]
 *        matrix_[2][0]    matrix_[2][1]    matrix_[2][2]    matrix_[2][3]
 *        matrix_[3][0]    matrix_[3][1]    matrix_[3][2]    matrix_[3][3]
 * @endcode
 */
class Matrix44
{
public:
    double matrix_[4][4];
    /**
     * @brief Compare if two matrices are equal.
     * @details Compares all 16 elements of the matrix object and comparing one specified by its arguments.\n
     *          The maximum permissible deviations of each element between the matrix object and the comparing one are specified by its arguments.\n
     * @param [in] matrix Another matrix object, used as the comparing object.
     * @param [in] valve The maximum permissible deviations of each element.
     * @retval true All deviations of the 16 elements between the matrix object and the comparing one are under the valve value.
     * @retval false  At least one deviation of the 16 elements between the matrix object and the comparing one is over the valve value.
     */
    bool isEqual(const Matrix44& matrix, double valve = 0.001) const;
    /**
     * @brief Do math operation left_matrix * this.
     * @details Do math operation: this = left_matrix * this.\n
     * @param [in] left_matrix Multiplication item on left side.
     * @return The result of the multiplication.
     */ 
    Matrix44& leftMultiply(const Matrix44& left_matrix);
    /**
     * @brief Do math operation left_matrix * this.
     * @details Do math operation: result_matrix = left_matrix * this.\n
     * @param [in] left_matrix Multiplication item on left side.
     * @param [out] result_matrix The result of the multiplication.
     * @return void
     */
    void leftMultiply(const Matrix44& left_matrix, Matrix44& result_matrix) const;
    /**
     * @brief Do math operation this * right_matrix.
     * @details Do math operation: this = this * right_matrix.\n
     * @param [in] right_matrix Multiplication item on right side.
     * @return The result of the multiplication.
     */
    Matrix44& rightMultiply(const Matrix44& right_matrix);
    /**
     * @brief Do math operation this * right_matrix.
     * @details Do math operation: result_matrix = this * right_matrix.\n
     * @param [in] right_matrix Multiplication item on right side.
     * @param [out] result_matrix The result of the multiplication.
     * @return void
     */
    void rightMultiply(const Matrix44& right_matrix, Matrix44& result_matrix) const;
    /**
     * @brief Operator= overload.
     * @details Envalue a Matrix44 object to self.\n
     * @param [in] matrix The source object.
     * @return A reference to self.
     */
    Matrix44& operator=(const Matrix44& matrix);
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
    void multiply(const Matrix44& left_matrix, const Matrix44& right_matrix, Matrix44& result_matrix) const;
};

}


#endif

