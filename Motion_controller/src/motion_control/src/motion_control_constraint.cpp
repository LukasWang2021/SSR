/*************************************************************************
	> File Name: motion_control_base_group.cpp
	> Author:
	> Mail:
	> Created Time: 2018年08月07日 星期二 11时25分56秒
 ************************************************************************/

#include <assert.h>
#include <string.h>
#include <motion_control_constraint.h>

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

bool Constraint::initConstraint(const Joint &lower, const Joint &upper, size_t joint_num)
{
    if (joint_num <= NUM_OF_JOINT)
    {
        joint_num_ = joint_num;

        for (size_t i = 0; i < joint_num_; i++)
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

bool Constraint::initConstraint(const JointConstraint &constraint, size_t joint_num)
{
    return initConstraint(constraint.lower, constraint.upper, joint_num);
}

size_t Constraint::getNumberOfJoint(void) const
{
    return joint_num_;
}

bool Constraint::setMask(size_t *index, size_t length)
{
    for (size_t i = 0; i < length; i++)
    {
        if (index[i] < joint_num_)
        {
            mask_[index[i]] = CONSTRAINT_MASKED;
        }
        else
        {
            return false;
        }
    }

    return true;
}

bool Constraint::resetMask(size_t *index, size_t length)
{
    for (size_t i = 0; i < length; i++)
    {
        if (index[i] < joint_num_)
        {
            mask_[index[i]] = CONSTRAINT_UNMASK;
        }
        else
        {
            return false;
        }
    }

    return true;
}

bool Constraint::isJointMasked(size_t index)
{
    return index < joint_num_ ? (mask_[index] == CONSTRAINT_MASKED) : (false);
}

void Constraint::setConstraint(const Joint &lower, const Joint &upper)
{
    for (size_t i = 0; i < joint_num_; i++)
    {
        constraint_.lower[i] = lower[i];
        constraint_.upper[i] = upper[i];
    }
}

void Constraint::getConstraint(Joint &lower, Joint &upper) const
{
    for (size_t i = 0; i < NUM_OF_JOINT; i++)
    {
        lower[i] = constraint_.lower[i];
        upper[i] = constraint_.upper[i];
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

bool Constraint::isJointInConstraint(const Joint &joint) const
{
    for (size_t i = 0; i < joint_num_; i++)
    {
        if (mask_[i] == CONSTRAINT_UNMASK && (joint[i] < constraint_.lower[i] || joint[i] > constraint_.upper[i]))
        {
            return false;
        }
    }

    return true;
}

bool Constraint::isCoverConstaint(const Constraint &constraint) const
{
    if (constraint.getNumberOfJoint() == joint_num_)
    {
        for (size_t i = 0; i < joint_num_; i++)
        {
            if (constraint.lower()[i] < constraint_.lower[i]  || constraint.upper()[i] > constraint_.upper[i])
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

bool Constraint::isCoverConstaint(const JointConstraint &constraint) const
{
    for (size_t i = 0; i < joint_num_; i++)
    {
        if (constraint.lower[i] < constraint_.lower[i]  || constraint.upper[i] > constraint_.upper[i])
        {
            return false;
        }
    }

    return true;
}


bool Constraint::isCoveredByConstaint(const Constraint &constraint) const
{
    if (constraint.getNumberOfJoint() == joint_num_)
    {
        for (size_t i = 0; i < joint_num_; i++)
        {
            if (constraint_.lower[i] < constraint.lower()[i] || constraint_.upper[i] > constraint.upper()[i])
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

bool Constraint::isCoveredByConstaint(const JointConstraint &constraint) const
{
    for (size_t i = 0; i < joint_num_; i++)
    {
        if (constraint_.lower[i] < constraint.lower[i] || constraint_.upper[i] > constraint.upper[i])
        {
            return false;
        }
    }

    return true;
}



}

