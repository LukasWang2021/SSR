#include "joint_acceleration.h"
#include <assert.h>
#include <math.h>
#include <iostream>

using namespace std;
using namespace basic_alg;


bool JointAcceleration::isEqual(JointAcceleration& joint_acc, double valve) const
{
    if(fabs(a1_ - joint_acc.a1_) < valve
        && fabs(a2_ - joint_acc.a2_) < valve
        && fabs(a3_ - joint_acc.a3_) < valve
        && fabs(a4_ - joint_acc.a4_) < valve
        && fabs(a5_ - joint_acc.a5_) < valve
        && fabs(a6_ - joint_acc.a6_) < valve
        && fabs(a7_ - joint_acc.a7_) < valve
        && fabs(a8_ - joint_acc.a8_) < valve
        && fabs(a9_ - joint_acc.a9_) < valve)
    {
        return true;
    }
    else
    {
        return false;
    }
}

double& JointAcceleration::operator[](size_t index) 
{
    assert(index < 9); 
    return *(&a1_ + index);
}

const double& JointAcceleration::operator[](size_t index) const 
{
    assert(index < 9); 
    return *(&a1_ + index);
}

const JointAcceleration JointAcceleration::operator+(const JointAcceleration& joint_acc)
{
    JointAcceleration result;
    result.a1_ = a1_ + joint_acc.a1_;
    result.a2_ = a2_ + joint_acc.a2_;
    result.a3_ = a3_ + joint_acc.a3_;
    result.a4_ = a4_ + joint_acc.a4_;
    result.a5_ = a5_ + joint_acc.a5_;
    result.a6_ = a6_ + joint_acc.a6_;
    result.a7_ = a7_ + joint_acc.a7_;
    result.a8_ = a8_ + joint_acc.a8_;
    result.a9_ = a9_ + joint_acc.a9_;
    return result;
}

const JointAcceleration JointAcceleration::operator-(const JointAcceleration& joint_acc)
{
    JointAcceleration result;
    result.a1_ = a1_ - joint_acc.a1_;
    result.a2_ = a2_ - joint_acc.a2_;
    result.a3_ = a3_ - joint_acc.a3_;
    result.a4_ = a4_ - joint_acc.a4_;
    result.a5_ = a5_ - joint_acc.a5_;
    result.a6_ = a6_ - joint_acc.a6_;
    result.a7_ = a7_ - joint_acc.a7_;
    result.a8_ = a8_ - joint_acc.a8_;
    result.a9_ = a9_ - joint_acc.a9_;
    return result;    
}

JointAcceleration& JointAcceleration::operator+=(const JointAcceleration& joint_acc)
{
    a1_ += joint_acc.a1_;
    a2_ += joint_acc.a2_;
    a3_ += joint_acc.a3_;
    a4_ += joint_acc.a4_;
    a5_ += joint_acc.a5_;
    a6_ += joint_acc.a6_;
    a7_ += joint_acc.a7_;
    a8_ += joint_acc.a8_;
    a9_ += joint_acc.a9_;
    return *this;
}

JointAcceleration& JointAcceleration::operator-=(const JointAcceleration& joint_acc)
{
    a1_ -= joint_acc.a1_;
    a2_ -= joint_acc.a2_;
    a3_ -= joint_acc.a3_;
    a4_ -= joint_acc.a4_;
    a5_ -= joint_acc.a5_;
    a6_ -= joint_acc.a6_;
    a7_ -= joint_acc.a7_;
    a8_ -= joint_acc.a8_;
    a9_ -= joint_acc.a9_;
    return *this;
}

void JointAcceleration::print(std::string comment) const
{
    std::cout<<comment<<std::endl;
    std::cout<<" a1 = "<<a1_
             <<" a2 = "<<a2_
             <<" a3 = "<<a3_
             <<" a4 = "<<a4_
             <<" a5 = "<<a5_
             <<" a6 = "<<a6_
             <<" a7 = "<<a7_
             <<" a8 = "<<a8_
             <<" a9 = "<<a9_<<std::endl;
}


