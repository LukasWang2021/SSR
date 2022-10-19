#ifndef MATRIX66_H
#define MATRIX66_H

#include <stddef.h>
#include <string>
#include "joint.h"
namespace basic_alg
{
/**
 * @brief Matrix66 is the object to represent a general 6 by 6 matrix.
 * @details Matrix66 has a two-dimensional array to store all 36 element 
 */

class Matrix66
{
public:
    double matrix_[6][6];
	
	Matrix66& eye(double gain=1.0);
    /**
     * @brief Do math operation left_matrix * this.
     * @details Do math operation: this = left_matrix * this.\n
     * @param [in] left_matrix Multiplication item on left side.
     * @return The result of the multiplication.
     */ 
    Matrix66& leftMultiply(const Matrix66& left_matrix);
    /**
     * @brief Do math operation left_matrix * this.
     * @details Do math operation: result_matrix = left_matrix * this.\n
     * @param [in] left_matrix Multiplication item on left side.
     * @param [out] result_matrix The result of the multiplication.
     * @return void
     */
    void leftMultiply(const Matrix66& left_matrix, Matrix66& result_matrix) const;
    /**
     * @brief Do math operation this * right_matrix.
     * @details Do math operation: this = this * right_matrix.\n
     * @param [in] right_matrix Multiplication item on right side.
     * @return The result of the multiplication.
     */
    Matrix66& rightMultiply(const Matrix66& right_matrix);
    /**
     * @brief Do math operation this * right_matrix.
     * @details Do math operation: result_matrix = this * right_matrix.\n
     * @param [in] right_matrix Multiplication item on right side.
     * @param [out] result_matrix The result of the multiplication.
     * @return void
     */
    void rightMultiply(const Matrix66& right_matrix, Matrix66& result_matrix) const;
    /**
     * @brief Operator= overload.
     * @details Envalue a Matrix66 object to self.\n
     * @param [in] matrix The source object.
     * @return A reference to self.
     */
    Matrix66& operator=(const Matrix66& matrix);
	Matrix66& sum(const Matrix66& matrix, int eye=0);
	
    void multiplyVect(double(&src_vect)[6], double(&res_vect)[6]);
	void multiplyVect(const Joint src_vect, Joint& res_vect);
    void print(std::string comment = "") const;

	void transpose(void);
	void transpose(Matrix66& res_mtx);
private:  
     /**
     * @brief Do math operation left_matrix * right_matrix.
     * @details Do math operation: result_matrix = left_matrix * right_matrix.\n
     * @param [in] left_matrix Multiplication item on left side.
     * @param [in] right_matrix Multiplication item on right side.
     * @param [out] result_matrix The result of the multiplication.
     * @return void
     */   
    void multiply(const Matrix66& left_matrix, const Matrix66& right_matrix, Matrix66& result_matrix) const;
};

}


#endif


