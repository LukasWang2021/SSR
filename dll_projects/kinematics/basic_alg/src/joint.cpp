#include "joint.h"
#include <assert.h>
#include <math.h>
#include <iostream>

using namespace std;
using namespace basic_alg;

void Joint::zero()
{
    j1_ = 0.0;
    j2_ = 0.0;
    j3_ = 0.0;
    j4_ = 0.0;
    j5_ = 0.0;
    j6_ = 0.0;
    j7_ = 0.0;
    j8_ = 0.0;
    j9_ = 0.0;
}

bool Joint::isEqual(Joint& joint, double valve) const
{
    if(fabs(j1_ - joint.j1_) < valve
        && fabs(j2_ - joint.j2_) < valve
        && fabs(j3_ - joint.j3_) < valve
        && fabs(j4_ - joint.j4_) < valve
        && fabs(j5_ - joint.j5_) < valve
        && fabs(j6_ - joint.j6_) < valve
        && fabs(j7_ - joint.j7_) < valve
        && fabs(j8_ - joint.j8_) < valve
        && fabs(j9_ - joint.j9_) < valve)
    {
        return true;
    }
    else
    {
        return false;
    }
}

double& Joint::operator[](size_t index) 
{
    assert(index < 9); 
    return *(&j1_ + index);
}

const double& Joint::operator[](size_t index) const 
{
    assert(index < 9); 
    return *(&j1_ + index);
}

const Joint Joint::operator+(const Joint& joint)
{
    Joint result;
    result.j1_ = j1_ + joint.j1_;
    result.j2_ = j2_ + joint.j2_;
    result.j3_ = j3_ + joint.j3_;
    result.j4_ = j4_ + joint.j4_;
    result.j5_ = j5_ + joint.j5_;
    result.j6_ = j6_ + joint.j6_;
    result.j7_ = j7_ + joint.j7_;
    result.j8_ = j8_ + joint.j8_;
    result.j9_ = j9_ + joint.j9_;
    return result;
}

const Joint Joint::operator-(const Joint& joint)
{
    Joint result;
    result.j1_ = j1_ - joint.j1_;
    result.j2_ = j2_ - joint.j2_;
    result.j3_ = j3_ - joint.j3_;
    result.j4_ = j4_ - joint.j4_;
    result.j5_ = j5_ - joint.j5_;
    result.j6_ = j6_ - joint.j6_;
    result.j7_ = j7_ - joint.j7_;
    result.j8_ = j8_ - joint.j8_;
    result.j9_ = j9_ - joint.j9_;
    return result;    
}

Joint& Joint::operator+=(const Joint& joint)
{
    j1_ += joint.j1_;
    j2_ += joint.j2_;
    j3_ += joint.j3_;
    j4_ += joint.j4_;
    j5_ += joint.j5_;
    j6_ += joint.j6_;
    j7_ += joint.j7_;
    j8_ += joint.j8_;
    j9_ += joint.j9_;
    return *this;
}

Joint& Joint::operator-=(const Joint& joint)
{
    j1_ -= joint.j1_;
    j2_ -= joint.j2_;
    j3_ -= joint.j3_;
    j4_ -= joint.j4_;
    j5_ -= joint.j5_;
    j6_ -= joint.j6_;
    j7_ -= joint.j7_;
    j8_ -= joint.j8_;
    j9_ -= joint.j9_;
    return *this;
}

void Joint::print(std::string comment) const
{
    std::cout<<comment<<std::endl;
    std::cout<<" j1 = "<<j1_
             <<" j2 = "<<j2_
             <<" j3 = "<<j3_
             <<" j4 = "<<j4_
             <<" j5 = "<<j5_
             <<" j6 = "<<j6_
             <<" j7 = "<<j7_
             <<" j8 = "<<j8_
             <<" j9 = "<<j9_<<std::endl;
}


