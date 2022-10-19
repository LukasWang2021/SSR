#include "double_array9.h"
#include <assert.h>
#include <math.h>
#include <iostream>

using namespace std;
using namespace basic_alg;

void DoubleArray9::zero()
{
    a0_ = 0;
    a1_ = 0;
    a2_ = 0;
    a3_ = 0;
    a4_ = 0;
    a5_ = 0;
    a6_ = 0;
    a7_ = 0;
    a8_ = 0;    
}

bool DoubleArray9::isEqual(DoubleArray9& array, size_t element_number, double valve) const
{
    assert(element_number > 0);
    assert(element_number < 10);
    for(size_t i = 0; i < element_number; ++i)
    {
        if(fabs((*this)[i] - array[i]) > valve)
        {
            return false;
        }
    }
    return true;
}

double& DoubleArray9::operator[](size_t index) 
{
    assert(index < 9); 
    return *(&a0_ + index);
}

const double& DoubleArray9::operator[](size_t index) const 
{
    assert(index < 9); 
    return *(&a0_ + index);
}

const DoubleArray9 DoubleArray9::operator+(const DoubleArray9& array)
{
    DoubleArray9 result;
    result.a0_ = a0_ + array.a0_;
    result.a1_ = a1_ + array.a1_;
    result.a2_ = a2_ + array.a2_;
    result.a3_ = a3_ + array.a3_;
    result.a4_ = a4_ + array.a4_;
    result.a5_ = a5_ + array.a5_;
    result.a6_ = a6_ + array.a6_;
    result.a7_ = a7_ + array.a7_; 
    result.a8_ = a8_ + array.a8_; 
    return result;
}

const DoubleArray9 DoubleArray9::operator-(const DoubleArray9& array)
{
    DoubleArray9 result;
    result.a0_ = a0_ - array.a0_;
    result.a1_ = a1_ - array.a1_;
    result.a2_ = a2_ - array.a2_;
    result.a3_ = a3_ - array.a3_;
    result.a4_ = a4_ - array.a4_;
    result.a5_ = a5_ - array.a5_;
    result.a6_ = a6_ - array.a6_;
    result.a7_ = a7_ - array.a7_; 
    result.a8_ = a8_ - array.a8_; 
    return result;    
}

DoubleArray9& DoubleArray9::operator+=(const DoubleArray9& array)
{
    a0_ += array.a0_;
    a1_ += array.a1_;
    a2_ += array.a2_;
    a3_ += array.a3_;
    a4_ += array.a4_;
    a5_ += array.a5_;
    a6_ += array.a6_;
    a7_ += array.a7_;
    a8_ += array.a8_;
    return *this;
}

DoubleArray9& DoubleArray9::operator-=(const DoubleArray9& array)
{
    a0_ -= array.a0_;
    a1_ -= array.a1_;
    a2_ -= array.a2_;
    a3_ -= array.a3_;
    a4_ -= array.a4_;
    a5_ -= array.a5_;
    a6_ -= array.a6_;
    a7_ -= array.a7_;  
    a8_ -= array.a8_;
    return *this;
}

void DoubleArray9::print(std::string comment) const
{
    std::cout<<comment<<std::endl;
    std::cout<<" a0 = "<<a0_
             <<" a1 = "<<a1_
             <<" a2 = "<<a2_
             <<" a3 = "<<a3_
             <<" a4 = "<<a4_
             <<" a5 = "<<a5_
             <<" a6 = "<<a6_
             <<" a7 = "<<a7_
             <<" a8 = "<<a8_<<std::endl;
}


