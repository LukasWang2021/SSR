#ifndef TRANS_MATRIX_H
#define TRANS_MATRIX_H

#include "rotation_matrix.h"
#include "point.h"
#include "pose_euler.h"
#include "pose_quaternion.h"
#include <string>

namespace basic_alg
{
class PoseEuler;
class PoseQuaternion;

/**
 * @brief TransMatrix is the object to represent a pose in 3D space. It is a 4 by 4 transformation matrix.
 * @details A pose consists of a transition part and an orientation part.\n
            The transition part is described by member variable point_.\n
            The orientation part is described by member variable rotation_matrix_.\n
            The definition of the transformation matrix is as follows:\n
 * @code
 *          R[0][0]    R[0][1]    R[0][2]    P.x_
 *          R[1][0]    R[1][1]    R[1][2]    P.y_
 *          R[2][0]    R[2][1]    R[2][2]    P.z_
 *             0          0          0        1
 * @endcode
 *          where R represents for rotation_matrix_.matrix_ and P represents for trans_vector_.
 * @note The last row of the transformation matrix is always {0, 0, 0, 1}. It is not necessary to save the last row of the matrix.
 */
class TransMatrix
{
public:
    RotationMatrix rotation_matrix_;
    Point trans_vector_;

    TransMatrix();
    /**
     * @brief Constructor.
     * @details Construct the object by DH parameters.
     * @param [in] d d of DH parameters.
     * @param [in] a a of DH parameters.
     * @param [in] alpha alpha of DH parameters.
     * @param [in] theta theta of DH parameters.
     */
    TransMatrix(double d, double a, double alpha, double theta);
    ~TransMatrix();
    /**
     * @brief Compare if two poses are equal.
     * @details Compare both trans_vector_ and rotation_matrix_ with given valve value.\n
     * @param [in] matrix Another pose object, used as the comparing object.
     * @param [in] valve The maximum permissible deviations for comparing trans_vector_ and rotation_matrix_.
     * @retval true Self pose is equal to the comparing pose.
     * @retval false Self pose is not equal to the comparing pose.
     */
    bool isEqual(const TransMatrix& matrix, double valve = 0.001) const;
    /**
     * @brief Convert the TransMatrix object to PoseEuler object.
     * @details Keep transition part and do convertion for orientation part.
     * @param [out] pose_euler The PoseEuler object.
     * @return void
     * @see basic_alg::PoseEuler
     */
    void convertToPoseEuler(basic_alg::PoseEuler& pose_euler) const;
    /**
     * @brief Convert the TransMatrix object to PoseQuaternion object.
     * @details Keep transition part and do convertion for orientation part.
     * @param [out] pose_quaternion The PoseQuaternion object.
     * @return void
     * @see basic_alg::PoseQuaternion
     */
    void convertToPoseQuaternion(basic_alg::PoseQuaternion& pose_quaternion) const;   
    /**
     * @brief Do math operation left_matrix * this.
     * @details Do math operation: this = left_matrix * this.\n
     * @param [in] left_matrix Multiplication item on left side.
     * @return The result of the multiplication.
     */ 
    TransMatrix& leftMultiply(const TransMatrix& left_matrix);
    /**
      * @brief Do math operation left_matrix * this.
      * @details Do math operation: result_matrix = left_matrix * this.\n
      * @param [in] left_matrix Multiplication item on left side.
      * @param [out] result_matrix The result of the multiplication.
      * @return void
      */
    void leftMultiply(const TransMatrix& left_matrix, TransMatrix& result_matrix) const;
    /**
      * @brief Do math operation this * right_matrix.
      * @details Do math operation: this = this * right_matrix.\n
      * @param [in] right_matrix Multiplication item on right side.
      * @return The result of the multiplication.
      */
    TransMatrix& rightMultiply(const TransMatrix& right_matrix);
    /**
     * @brief Do math operation this * right_matrix.
     * @details Do math operation: result_matrix = this * right_matrix.\n
     * @param [in] right_matrix Multiplication item on right side.
     * @param [out] result_matrix The result of the multiplication.
     * @return void
     */
    void rightMultiply(const TransMatrix& right_matrix, TransMatrix& result_matrix) const;
    /**
     * @brief Compute the inverse matrix.
     * @details Do math operation: this = inv(this).\n
     * @param [in] valve Valve value for internal use.
     * @retval true Inverse matrix of the transformation matrix is exist and computed.
     * @retval false Inverse matrix of the transformation matrix is not exist.
     */
    bool inverse(double valve = 0.001);
    /**
     * @brief Compute the inverse matrix.
     * @details Do math operation: result_matrix = inv(this).\n
     * @param [out] result_matrix Inverse of the transformation matrix.
     * @param [in] valve Valve value for internal use.
     * @retval true Inverse matrix of the transformation matrix is exist and computed.
     * @retval false Inverse matrix of the transformation matrix is not exist.
     */
    bool inverse(TransMatrix& result_matrix, double valve = 0.001);
    void print(std::string comment = "") const;
    void print_(std::string comment = "") const;
	
	void inverse_simplify(void);	
	void inverse_simplify(TransMatrix& result_matrix);
    
private:
    /**
     * @brief Do math operation left_matrix * right_matrix.
     * @details Do math operation: result_matrix = left_matrix * right_matrix.\n
     * @param [in] left_matrix Multiplication item on left side.
     * @param [in] right_matrix Multiplication item on right side.
     * @param [out] result_matrix The result of the multiplication.
     * @return void
     */
    void multiply(const TransMatrix& left_matrix, const TransMatrix& right_matrix, TransMatrix& result_matrix) const;
};

}

#endif
