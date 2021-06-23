#ifndef TRANS_MATRIX_H
#define TRANS_MATRIX_H

/**
 * @file trans_matrix.h
 * @brief The file is the header file of class "TransMatrix".
 * @author zhengyu.shen
 */

#include "rotation_matrix.h"
#include "vector3.h"
#include "pose_euler.h"
#include "pose_quaternion.h"
#include <string>
/**
 * @brief basic_alg includes all foundational mathmatics computation.
 */
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
    Vector3 trans_vector_;                /**< Transition part of the pose.\n*/
    RotationMatrix rotation_matrix_;    /**< Orientation part of the pose.\n*/

    /**
     * @brief Default constructor.
     */
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
    /**
     * @brief Default destructor.
     */    
    ~TransMatrix();
    /**
     * @brief Initialize the matrix.
     * @details Initialize the matrix by dh parameters.
     * @param [in] d d of DH parameters.
     * @param [in] a a of DH parameters.
     * @param [in] alpha alpha of DH parameters.
     * @param [in] theta theta of DH parameters.
     */
    void initByDh(double d, double a, double alpha, double theta);
    /**
     * @brief Make the matrix to be an identity matrix.
     * @details Make the matrix to be an identity matrix.\n
     * @return void
     */
    void identity();
    /**
     * @brief Compare if two poses are equal.
     * @details Compare both trans_vector_ and rotation_matrix_ with given valve value.\n
     * @param [in] matrix Another pose object, used as the comparing object.
     * @param [in] valve The maximum permissible deviations for comparing trans_vector_ and rotation_matrix_.
     * @retval true Self pose is equal to the comparing pose.
     * @retval false Self pose is not equal to the comparing pose.
     */
    bool isEqual(TransMatrix& matrix, double valve = 0.001) const;
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
    TransMatrix& leftMultiply(TransMatrix& left_matrix);
     /**
      * @brief Do math operation left_matrix * this.
      * @details Do math operation: result_matrix = left_matrix * this.\n
      * @param [in] left_matrix Multiplication item on left side.
      * @param [out] result_matrix The result of the multiplication.
      * @return void
      */
    void leftMultiply(TransMatrix& left_matrix, TransMatrix& result_matrix);
     /**
      * @brief Do math operation this * right_matrix.
      * @details Do math operation: this = this * right_matrix.\n
      * @param [in] right_matrix Multiplication item on right side.
      * @return The result of the multiplication.
      */
    TransMatrix& rightMultiply(TransMatrix& right_matrix);
     /**
     * @brief Do math operation this * right_matrix.
     * @details Do math operation: result_matrix = this * right_matrix.\n
     * @param [in] right_matrix Multiplication item on right side.
     * @param [out] result_matrix The result of the multiplication.
     * @return void
     */     
    void rightMultiply(TransMatrix& right_matrix, TransMatrix& result_matrix);
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
     /**
     * @brief Print all member variables.
     * @details Print member variables in format.\n
     * @param [in] comment First line to print.
     * @return void
     * @par code example:
     * @code 
     *    TransMatrix pose;
     *    pose.trans_vector_.x_ = 1;
     *    pose.trans_vector_.y_ = 2;
     *    pose.trans_vector_.z_ = 3;
     *    pose.rotation_matrix_.matrix_[0][0]_ = 1; pose.rotation_matrix_.matrix_[0][1]_ = 0; pose.rotation_matrix_.matrix_[0][2]_ = 0;
     *    pose.rotation_matrix_.matrix_[1][0]_ = 0; pose.rotation_matrix_.matrix_[1][1]_ = 1; pose.rotation_matrix_.matrix_[1][2]_ = 0;
     *    pose.rotation_matrix_.matrix_[2][0]_ = 0; pose.rotation_matrix_.matrix_[2][1]_ = 0; pose.rotation_matrix_.matrix_[2][2]_ = 1;
     *    pose.print("this is a test:");
     * @endcode
     * @par code result:
     * @code
     *    root>this is a test:
     *    root>trans vector:
     *    root> x = 1
     *    root> y = 2
     *    root> z = 3
     *    root>rotation matrix:
     *    root>1 0 0
     *    root>0 1 0
     *    root>0 0 1
     *    root>
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
    void multiply(TransMatrix& left_matrix, TransMatrix& right_matrix, TransMatrix& result_matrix);
};

}

#endif
