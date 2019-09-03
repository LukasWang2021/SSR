#include "joint_torque.h"
#include <assert.h>
#include <math.h>
#include <iostream>

using namespace std;
using namespace basic_alg;


bool JointTorque::isEqual(JointTorque& joint_torque, double valve) const
{
    if(fabs(t1_ - joint_torque.t1_) < valve
        && fabs(t2_ - joint_torque.t2_) < valve
        && fabs(t3_ - joint_torque.t3_) < valve
        && fabs(t4_ - joint_torque.t4_) < valve
        && fabs(t5_ - joint_torque.t5_) < valve
        && fabs(t6_ - joint_torque.t6_) < valve
        && fabs(t7_ - joint_torque.t7_) < valve
        && fabs(t8_ - joint_torque.t8_) < valve
        && fabs(t9_ - joint_torque.t9_) < valve)
    {
        return true;
    }
    else
    {
        return false;
    }
}

double& JointTorque::operator[](size_t index) 
{
    assert(index < 9); 
    return *(&t1_ + index);
}

const double& JointTorque::operator[](size_t index) const 
{
    assert(index < 9); 
    return *(&t1_ + index);
}

const JointTorque JointTorque::operator+(const JointTorque& joint_torque)
{
    JointTorque result;
    result.t1_ = t1_ + joint_torque.t1_;
    result.t2_ = t2_ + joint_torque.t2_;
    result.t3_ = t3_ + joint_torque.t3_;
    result.t4_ = t4_ + joint_torque.t4_;
    result.t5_ = t5_ + joint_torque.t5_;
    result.t6_ = t6_ + joint_torque.t6_;
    result.t7_ = t7_ + joint_torque.t7_;
    result.t8_ = t8_ + joint_torque.t8_;
    result.t9_ = t9_ + joint_torque.t9_;
    return result;
}

const JointTorque JointTorque::operator-(const JointTorque& joint_torque)
{
    JointTorque result;
    result.t1_ = t1_ - joint_torque.t1_;
    result.t2_ = t2_ - joint_torque.t2_;
    result.t3_ = t3_ - joint_torque.t3_;
    result.t4_ = t4_ - joint_torque.t4_;
    result.t5_ = t5_ - joint_torque.t5_;
    result.t6_ = t6_ - joint_torque.t6_;
    result.t7_ = t7_ - joint_torque.t7_;
    result.t8_ = t8_ - joint_torque.t8_;
    result.t9_ = t9_ - joint_torque.t9_;
    return result;    
}

JointTorque& JointTorque::operator+=(const JointTorque& joint_torque)
{
    t1_ += joint_torque.t1_;
    t2_ += joint_torque.t2_;
    t3_ += joint_torque.t3_;
    t4_ += joint_torque.t4_;
    t5_ += joint_torque.t5_;
    t6_ += joint_torque.t6_;
    t7_ += joint_torque.t7_;
    t8_ += joint_torque.t8_;
    t9_ += joint_torque.t9_;
    return *this;
}

JointTorque& JointTorque::operator-=(const JointTorque& joint_torque)
{
    t1_ -= joint_torque.t1_;
    t2_ -= joint_torque.t2_;
    t3_ -= joint_torque.t3_;
    t4_ -= joint_torque.t4_;
    t5_ -= joint_torque.t5_;
    t6_ -= joint_torque.t6_;
    t7_ -= joint_torque.t7_;
    t8_ -= joint_torque.t8_;
    t9_ -= joint_torque.t9_;
    return *this;
}

void JointTorque::print(std::string comment) const
{
    std::cout<<comment<<std::endl;
    std::cout<<" t1 = "<<t1_
             <<" t2 = "<<t2_
             <<" t3 = "<<t3_
             <<" t4 = "<<t4_
             <<" t5 = "<<t5_
             <<" t6 = "<<t6_
             <<" t7 = "<<t7_
             <<" t8 = "<<t8_
             <<" t9 = "<<t9_<<std::endl;
}


