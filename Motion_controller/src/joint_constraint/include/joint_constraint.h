/*************************************************************************
	> File Name: motion_control_constraint.h
	> Author:
	> Mail:
	> Created Time: 2018年08月07日 星期二 10时58分29秒
 ************************************************************************/
#ifndef _JOINT_CONSTRAINT_H
#define _JOINT_CONSTRAINT_H

#include <basic_alg.h>


namespace group_space
{


struct JointConstraint  // 关节限位
{
    basic_alg::Joint upper;    // 上限
    basic_alg::Joint lower;    // 下限
};

enum ConstraintMask
{
    CONSTRAINT_UNMASK = 0,
    CONSTRAINT_MASKED = 1,
};

class Constraint
{
  public:
    Constraint(void);
    ~Constraint(void);

    bool initConstraint(const basic_alg::Joint &lower, const basic_alg::Joint &upper, uint32_t joint_num);
    bool initConstraint(const JointConstraint &constraint, uint32_t joint_num);
    bool isMask(uint32_t index);
    void setMask(uint32_t index);
    void resetMask(uint32_t index);
    void getConstraint(basic_alg::Joint &lower, basic_alg::Joint &upper) const;
    void setConstraint(const basic_alg::Joint &lower, const basic_alg::Joint &upper);
    void getConstraint(JointConstraint &constraint) const;
    void setConstraint(const JointConstraint &constraint);

    basic_alg::Joint& upper(void);
    basic_alg::Joint& lower(void);
    const basic_alg::Joint& upper(void) const;
    const basic_alg::Joint& lower(void) const;

    bool isJointInConstraint(const basic_alg::Joint &joint, double value = MINIMUM_E6) const;
    bool isCoverConstaint(const Constraint &constraint, double value = MINIMUM_E6) const;
    bool isCoverConstaint(const JointConstraint &constraint, double value = MINIMUM_E6) const;
    bool isCoveredByConstaint(const Constraint &constraint, double value = MINIMUM_E6) const;
    bool isCoveredByConstaint(const JointConstraint &constraint, double value = MINIMUM_E6) const;

    uint32_t getNumberOfJoint(void) const;

  private:
    uint32_t joint_num_;
    ConstraintMask  mask_[NUM_OF_JOINT] = {CONSTRAINT_UNMASK};
    JointConstraint constraint_;
};

}


#endif //_MOTION_CONTROL_CONSTRAINT_H
