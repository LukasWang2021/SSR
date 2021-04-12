#ifndef POSE_EULER_H
#define POSE_EULER_H

#include "point.h"
#include "euler.h"
#include "pose_quaternion.h"
#include "trans_matrix.h"

namespace basic_alg
{
class PoseQuaternion;
class TransMatrix;

/**
 * @brief PoseEuler is the object to represent a pose in 3D space. It includes two member variables {point_, euler_}.
 * @details A pose consists of a transition part and an orientation part.\n
            The transition part is described by member variable point_.\n
            The orientation part is described by member variable euler_.\n
 */
class PoseEuler
{
public:
    Point point_;
    Euler euler_;
    /**
     * @brief Compare if two poses are equal.
     * @details Compare both trans_vector_ and euler_ with given valve value.\n
     * @param [in] pose_euler Another pose object, used as the comparing object.
     * @param [in] valve The maximum permissible deviations for comparing trans_vector_ and euler_.
     * @retval true Self pose is equal to the comparing pose.
     * @retval false Self pose is not equal to the comparing pose.
     */
    bool isEqual(const PoseEuler& pose_euler, double valve = 0.001) const;
    bool isEquivalent(const PoseEuler& pose_euler, double valve = 0.001) const;
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

    void print(std::string comment = "") const;
};

}


#endif

