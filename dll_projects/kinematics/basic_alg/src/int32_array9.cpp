#include "int32_array9.h"
#include <assert.h>
#include <math.h>
#include <iostream>

using namespace std;
using namespace basic_alg;

void Int32Array9::zero()
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

bool Int32Array9::isEqual(Int32Array9& array, size_t element_number) const
{
    assert(element_number > 0);
    assert(element_number < 10);
    for(size_t i = 0; i < element_number; ++i)
    {
        if((*this)[i] != array[i])
        {
            return false;
        }
    }
    return true;
}

int32_t& Int32Array9::operator[](size_t index) 
{
    assert(index < 9); 
    return *(&a0_ + index);
}

const int32_t& Int32Array9::operator[](size_t index) const 
{
    assert(index < 9); 
    return *(&a0_ + index);
}

const Int32Array9 Int32Array9::operator+(const Int32Array9& array)
{
    Int32Array9 result;
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

const Int32Array9 Int32Array9::operator-(const Int32Array9& array)
{
    Int32Array9 result;
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

Int32Array9& Int32Array9::operator+=(const Int32Array9& array)
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

Int32Array9& Int32Array9::operator-=(const Int32Array9& array)
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

void Int32Array9::print(std::string comment) const
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


