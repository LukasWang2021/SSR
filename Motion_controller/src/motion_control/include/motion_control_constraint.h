/*************************************************************************
	> File Name: motion_control_constraint.h
	> Author:
	> Mail:
	> Created Time: 2018年08月07日 星期二 10时58分29秒
 ************************************************************************/



#ifndef _MOTION_CONTROL_CONSTRAINT_H
#define _MOTION_CONTROL_CONSTRAINT_H

#include <motion_control_datatype.h>
#include "motion_control_datatype.h"

namespace fst_mc
{

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

    bool initConstraint(const Joint &lower, const Joint &upper, size_t joint_num);
    bool initConstraint(const JointConstraint &constraint, size_t joint_num);

    size_t getNumberOfJoint(void) const;

    bool setMask(size_t *index, size_t length);
    bool resetMask(size_t *index, size_t length);
    bool isJointMasked(size_t index);

    void getConstraint(Joint &lower, Joint &upper) const;
    void setConstraint(const Joint &lower, const Joint &upper);

    void getConstraint(JointConstraint &constraint) const;
    void setConstraint(const JointConstraint &constraint);

    Joint& upper(void);
    Joint& lower(void);
    const Joint& upper(void) const;
    const Joint& lower(void) const;

    bool isJointInConstraint(const Joint &joint) const;
    bool isCoverConstaint(const Constraint &constraint) const;
    bool isCoverConstaint(const JointConstraint &constraint) const;
    bool isCoveredByConstaint(const Constraint &constraint) const;
    bool isCoveredByConstaint(const JointConstraint &constraint) const;

  private:
    size_t joint_num_;
    ConstraintMask  mask_[NUM_OF_JOINT] = {CONSTRAINT_UNMASK};
    JointConstraint constraint_;
};

}


#endif //_MOTION_CONTROL_CONSTRAINT_H