#ifndef POSE_QUATERNION_H
#define POSE_QUATERNION_H

/**
 * @file pose_quaternion.h
 * @brief The file is the header file of class "PoseQuaternion".
 * @author zhengyu.shen
 */

#include "vector3.h"
#include "quaternion.h"
#include "pose_euler.h"
#include "trans_matrix.h"
#include <string>
/**
 * @brief basic_alg includes all foundational mathmatics computation.
 */
namespace basic_alg
{
class PoseEuler;
class TransMatrix;

/**
 * @brief PoseQuaternion is the object to represent a pose in 3D space. It includes two member variables {trans_vector_, quaternion_}.
 * @details A pose consists of a transition part and an orientation part.\n
            The transition part is described by member variable point_.\n
            The orientation part is described by member variable quaternion_.\n
 */
class PoseQuaternion
{
public:
    Vector3 trans_vector_;    /**< Transition part of the pose.\n*/
    Quaternion quaternion_; /**< Orientation part of the pose.\n*/
     /**
     * @brief Express the original point of some coordinate system with pose-quaternion expression.
     * @details Transition and rotation are all equals to zero according to the coordinate system.\n
     * @return void
     */
    void zero();
    /**
     * @brief Compare if two poses are equal.
     * @details Compare both trans_vector_ and quaternion_ with given valve value.\n
     * @param [in] pose_quaternion Another pose object, used as the comparing object.
     * @param [in] valve The maximum permissible deviations for comparing trans_vector_ and quaternion_.
     * @retval true Self pose is equal to the comparing pose.
     * @retval false Self pose is not equal to the comparing pose.
     */
    bool isEqual(PoseQuaternion& pose_quaternion, double valve = 0.001) const;
    /**
     * @brief Convert the PoseQuaternion object to PoseEuler object.
     * @details Keep transition part and do convertion for orientation part.
     * @param [out] pose_euler The PoseEuler object.
     * @return void
     * @see basic_alg::PoseEuler
     */     
    void convertToPoseEuler(basic_alg::PoseEuler& pose_euler) const;
     /**
     * @brief Convert the PoseQuaternion object to TransMatrix object.
     * @details Combine transition part and orientation part together to form a 4 by 4 transformation matrix.\n
     * @param [out] matrix The TransMatrix object.
     * @return void
     * @see basic_alg::TransMatrix
     */ 
    void convertToTransMatrix(basic_alg::TransMatrix& matrix) const;  
     /**
     * @brief Print all member variables.
     * @details Print member variables in format.\n
     * @param [in] comment First line to print.
     * @return void
     * @par code example:
     * @code 
     *    PoseQuaternion pose;
     *    pose.trans_vector_.x_ = 1;
     *    pose.trans_vector_.y_ = 2;
     *    pose.trans_vector_.z_ = 3;
     *    pose.quaternion_.x_ = 1;
     *    pose.quaternion_.y_ = 0;
     *    pose.quaternion_.z_ = 0;
     *    pose.quaternion_.w_ = 0;
     *    pose.print("this is a test:");
     * @endcode
     * @par code result:
     * @code
     *    root>this is a test:
     *    root>trans vector:
     *    root> x = 1
     *    root> y = 2
     *    root> z = 3
     *    root>quaternion:
     *    root> x = 1
     *    root> y = 0
     *    root> z = 0
     *    root> w = 0
     *    root>
     * @endcode
     */
    void print(std::string comment = "") const;
};

}


#endif

