/*************************************************************************
	> File Name: motion_control_base_group.cpp
	> Author:
	> Mail:
	> Created Time: 2018年08月07日 星期二 11时25分56秒
 ************************************************************************/
#include <string.h>
#include <joint_constraint.h>

using namespace basic_alg;

namespace fst_mc
{

Constraint::Constraint()
{
    joint_num_ = 0;
    memset(mask_, 0, sizeof(mask_));
    memset(&constraint_, 0, sizeof(JointConstraint));
}

Constraint::~Constraint(void)
{}

bool Constraint::initConstraint(const Joint &lower, const Joint &upper, uint32_t joint_num)
{
    if (joint_num <= NUM_OF_JOINT)
    {
        joint_num_ = joint_num;

        for (uint32_t i = 0; i < joint_num_; i++)
        {
            constraint_.lower[i] = lower[i];
            constraint_.upper[i] = upper[i];
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool Constraint::initConstraint(const JointConstraint &constraint, uint32_t joint_num)
{
    return initConstraint(constraint.lower, constraint.upper, joint_num);
}

uint32_t Constraint::getNumberOfJoint(void) const
{
    return joint_num_;
}

void Constraint::setMask(uint32_t index)
{
    if (index < joint_num_)
    {
        mask_[index] = CONSTRAINT_MASKED;
    }
}

void Constraint::resetMask(uint32_t index)
{
    if (index < joint_num_)
    {
        mask_[index] = CONSTRAINT_UNMASK;
    }
}

bool Constraint::isMask(uint32_t index)
{
    return index < joint_num_ ? (mask_[index] == CONSTRAINT_MASKED) : (false);
}

void Constraint::setConstraint(const Joint &lower, const Joint &upper)
{
    for (uint32_t i = 0; i < joint_num_; i++)
    {
        constraint_.lower[i] = lower[i];
        constraint_.upper[i] = upper[i];
    }
}

void Constraint::getConstraint(Joint &lower, Joint &upper) const
{
    for (uint32_t i = 0; i < joint_num_; i++)
    {
        lower[i] = constraint_.lower[i];
        upper[i] = constraint_.upper[i];
    }

    for (uint32_t i = joint_num_; i < NUM_OF_JOINT; i++)
    {
        lower[i] = 0;
        upper[i] = 0;
    }
}

void Constraint::setConstraint(const JointConstraint &constraint)
{
    setConstraint(constraint.lower, constraint.upper);
}

void Constraint::getConstraint(JointConstraint &constraint) const
{
    getConstraint(constraint.lower, constraint.upper);
}

Joint& Constraint::upper(void)
{
    return constraint_.upper;
}

Joint& Constraint::lower(void)
{
    return constraint_.lower;
}

const Joint& Constraint::upper(void) const
{
    return constraint_.upper;
}

const Joint& Constraint::lower(void) const
{
    return constraint_.lower;
}

bool Constraint::isJointInConstraint(const Joint &joint, double value) const
{
    for (uint32_t i = 0; i < joint_num_; i++)
    {
        if (mask_[i] == CONSTRAINT_UNMASK)
        {
            if (!((joint[i] > constraint_.lower[i] - value) && (joint[i] < constraint_.upper[i] + value)))
            {
                return false;
            }
        }
    }

    return true;
}

bool Constraint::isCoverConstaint(const Constraint &constraint, double value) const
{
    if (constraint.getNumberOfJoint() == joint_num_)
    {
        for (uint32_t i = 0; i < joint_num_; i++)
        {
            if (!((constraint.lower()[i] > constraint_.lower[i] - value) && (constraint.upper()[i] < constraint_.upper[i] + value)))
            {
                return false;
            }
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool Constraint::isCoverConstaint(const JointConstraint &constraint, double value) const
{
    for (uint32_t i = 0; i < joint_num_; i++)
    {
        if (!((constraint.lower[i] > constraint_.lower[i] - value) && (constraint.upper[i] < constraint_.upper[i] + value)))
        {
            return false;
        }
    }

    return true;
}


bool Constraint::isCoveredByConstaint(const Constraint &constraint, double value) const
{
    if (constraint.getNumberOfJoint() == joint_num_)
    {
        for (uint32_t i = 0; i < joint_num_; i++)
        {
            if (!((constraint_.lower[i] > constraint.lower()[i] - value) && (constraint_.upper[i] < constraint.upper()[i] + value)))
            {
                return false;
            }
        }

        return true;
    }
    else
    {
        return false;
    }
}

bool Constraint::isCoveredByConstaint(const JointConstraint &constraint, double value) const
{
    for (uint32_t i = 0; i < joint_num_; i++)
    {
        if (!((constraint_.lower[i] > constraint.lower[i] - value) && (constraint_.upper[i] < constraint.upper[i] + value)))
        {
            return false;
        }
    }

    return true;
}



}

