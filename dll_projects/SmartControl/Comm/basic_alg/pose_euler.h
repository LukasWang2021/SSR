#ifndef POSE_EULER_H
#define POSE_EULER_H

/**
 * @file pose_euler.h
 * @brief The file is the header file of class "PoseEuler".
 * @author zhengyu.shen
 */

#include "vector3.h"
#include "euler.h"
#include "pose_quaternion.h"
#include "trans_matrix.h"
/**
 * @brief basic_alg includes all foundational mathmatics computation.
 */
namespace basic_alg
{
class PoseQuaternion;
class TransMatrix;

/**
 * @brief PoseEuler is the object to represent a pose in 3D space. It includes two member variables {trans_vector_, euler_}.
 * @details A pose consists of a transition part and an orientation part.\n
            The transition part is described by member variable trans_vector_.\n
            The orientation part is described by member variable euler_.\n
 */
class PoseEuler
{
public:
    Vector3 trans_vector_;    /**< Transition part of the pose.\n*/
    Euler euler_;           /**< Orientation part of the pose.\n*/
     /**
     * @brief Express the original point of some coordinate system with pose-euler expression.
     * @details Transition and rotation are all equals to zero according to the coordinate system.\n
     * @return void
     */
    void zero();
    /**
     * @brief Compare if two poses are equal.
     * @details Compare both trans_vector_ and euler_ with given valve value.\n
     * @param [in] pose_euler Another pose object, used as the comparing object.
     * @param [in] valve The maximum permissible deviations for comparing trans_vector_ and euler_.
     * @retval true Self pose is equal to the comparing pose.
     * @retval false Self pose is not equal to the comparing pose.
     */
    bool isEqual(PoseEuler& pose_euler, double valve = 0.001) const;
    /**
     * @brief Convert the PoseEuler object to PoseQuaternion object.
     * @details Keep transition part and do convertion for orientation part.
     * @param [out] pose_quaternion The PoseQuaternion object.
     * @return void
     * @see basic_alg::PoseQuaternion
     */    
    void convertToPoseQuaternion(basic_alg::PoseQuaternion& pose_quaternion) const;
    /**
     * @brief Convert the PoseEuler object to TransMatrix object.
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
    *    PoseEuler pose;
    *    pose.trans_vector_.x_ = 1;
    *    pose.trans_vector_.y_ = 2;
    *    pose.trans_vector_.z_ = 3;
    *    pose.euler_.a_ = 0.1;
    *    pose.euler_.b_ = 0.2;
    *    pose.euler_.c_ = 0.3;
    *    pose.print("this is a test:");
    * @endcode
    * @par code result:
    * @code
    *    root>this is a test:
    *    root>trans vector:
    *    root> x = 1
    *    root> y = 2
    *    root> z = 3
    *    root>euler:
    *    root> a = 1
    *    root> b = 2
    *    root> c = 3
    *    root>
    * @endcode
    */    
    void print(std::string comment = "") const;
};

}


#endif

