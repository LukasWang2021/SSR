/*************************************************************************
	> File Name: arm_kinematics.h
	> Author: 
	> Mail: 
	> Created Time: 2018年08月28日 星期二 13时29分58秒
 ************************************************************************/

#ifndef _ARM_KINEMATICS_H
#define _ARM_KINEMATICS_H

#include <base_kinematics.h>

#ifndef JOINT_OF_ARM
#define JOINT_OF_ARM    6
#endif

namespace fst_mc
{

class ArmKinematics : public BaseKinematics
{
  public:
    void initKinematics(double (&dh_matrix)[NUM_OF_JOINT][4]);

    //ErrorCode   chainIK(const Pose &pose, Joint &ref, Joint &res);
    //ErrorCode   chainIK(const PoseEuler &pose, Joint &ref, Joint &res);


  private:
    void forwardKinematics(const Joint &joint, basic_alg::Matrix &matrix);
    ErrorCode inverseKinematics(const basic_alg::Matrix &matrix, Joint (&solutions)[8], size_t length);
    ErrorCode inverseKinematics(const basic_alg::Matrix &matrix, const Joint &ref, Joint &res);
};


}

#endif
