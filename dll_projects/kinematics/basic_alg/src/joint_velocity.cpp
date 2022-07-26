#include "joint_velocity.h"
#include <assert.h>
#include <math.h>
#include <iostream>

using namespace std;
using namespace basic_alg;


bool JointVelocity::isEqual(JointVelocity& joint_vel, double valve) const
{
    if(fabs(v1_ - joint_vel.v1_) < valve
        && fabs(v2_ - joint_vel.v2_) < valve
        && fabs(v3_ - joint_vel.v3_) < valve
        && fabs(v4_ - joint_vel.v4_) < valve
        && fabs(v5_ - joint_vel.v5_) < valve
        && fabs(v6_ - joint_vel.v6_) < valve
        && fabs(v7_ - joint_vel.v7_) < valve
        && fabs(v8_ - joint_vel.v8_) < valve
        && fabs(v9_ - joint_vel.v9_) < valve)
    {
        return true;
    }
    else
    {
        return false;
    }
}

double& JointVelocity::operator[](size_t index) 
{
    assert(index < 9); 
    return *(&v1_ + index);
}

const double& JointVelocity::operator[](size_t index) const 
{
    assert(index < 9); 
    return *(&v1_ + index);
}

const JointVelocity JointVelocity::operator+(const JointVelocity& joint_vel)
{
    JointVelocity result;
    result.v1_ = v1_ + joint_vel.v1_;
    result.v2_ = v2_ + joint_vel.v2_;
    result.v3_ = v3_ + joint_vel.v3_;
    result.v4_ = v4_ + joint_vel.v4_;
    result.v5_ = v5_ + joint_vel.v5_;
    result.v6_ = v6_ + joint_vel.v6_;
    result.v7_ = v7_ + joint_vel.v7_;
    result.v8_ = v8_ + joint_vel.v8_;
    result.v9_ = v9_ + joint_vel.v9_;
    return result;
}

const JointVelocity JointVelocity::operator-(const JointVelocity& joint_vel)
{
    JointVelocity result;
    result.v1_ = v1_ - joint_vel.v1_;
    result.v2_ = v2_ - joint_vel.v2_;
    result.v3_ = v3_ - joint_vel.v3_;
    result.v4_ = v4_ - joint_vel.v4_;
    result.v5_ = v5_ - joint_vel.v5_;
    result.v6_ = v6_ - joint_vel.v6_;
    result.v7_ = v7_ - joint_vel.v7_;
    result.v8_ = v8_ - joint_vel.v8_;
    result.v9_ = v9_ - joint_vel.v9_;
    return result;    
}

JointVelocity& JointVelocity::operator+=(const JointVelocity& joint_vel)
{
    v1_ += joint_vel.v1_;
    v2_ += joint_vel.v2_;
    v3_ += joint_vel.v3_;
    v4_ += joint_vel.v4_;
    v5_ += joint_vel.v5_;
    v6_ += joint_vel.v6_;
    v7_ += joint_vel.v7_;
    v8_ += joint_vel.v8_;
    v9_ += joint_vel.v9_;
    return *this;
}

JointVelocity& JointVelocity::operator-=(const JointVelocity& joint_vel)
{
    v1_ -= joint_vel.v1_;
    v2_ -= joint_vel.v2_;
    v3_ -= joint_vel.v3_;
    v4_ -= joint_vel.v4_;
    v5_ -= joint_vel.v5_;
    v6_ -= joint_vel.v6_;
    v7_ -= joint_vel.v7_;
    v8_ -= joint_vel.v8_;
    v9_ -= joint_vel.v9_;
    return *this;
}

void JointVelocity::print(std::string comment) const
{
    std::cout<<comment<<std::endl;
    std::cout<<" v1 = "<<v1_
             <<" v2 = "<<v2_
             <<" v3 = "<<v3_
             <<" v4 = "<<v4_
             <<" v5 = "<<v5_
             <<" v6 = "<<v6_
             <<" v7 = "<<v7_
             <<" v8 = "<<v8_
             <<" v9 = "<<v9_<<std::endl;
}


