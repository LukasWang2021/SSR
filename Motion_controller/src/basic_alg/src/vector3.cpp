#include "vector3.h"
#include <assert.h>
#include <math.h>
#include <iostream>

using namespace std;
using namespace basic_alg;

void Vector3::zero()
{
    x_ = 0;
    y_ = 0;
    z_ = 0;
}

bool Vector3::isEqual(Vector3& vector, double valve) const
{
    if(fabs(x_ - vector.x_) < valve
        && fabs(y_ - vector.y_) < valve
        && fabs(z_ - vector.z_) <valve)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Vector3::isParallel(Vector3& vector, double valve) const
{
    double max;
    int32_t max_index;
    max = vector.x_;
    max_index = 1;
    if(fabs(max) < fabs(vector.y_))
    {
        max = vector.y_;
        max_index = 2;
    }
    if(fabs(max) < fabs(vector.z_))
    {
        max = vector.z_;
        max_index = 3;
    }

    double scale_factor;
    switch(max_index)
    {
        case 1:
            if(fabs(x_) < valve)
            {
                return false;
            }
            else
            {
                scale_factor = vector.x_ / x_;
                if(fabs(scale_factor * y_ - vector.y_) < valve
                    && fabs(scale_factor * z_ - vector.z_) < valve)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
        case 2:
            if(fabs(y_) < valve)
            {
                return false;
            }
            else
            {
                scale_factor = vector.y_ / y_;
                if(fabs(scale_factor * x_ - vector.x_) < valve
                    && fabs(scale_factor * z_ - vector.z_) < valve)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
        case 3:
            if(fabs(z_) < valve)
            {
                return false;
            }
            else
            {
                scale_factor = vector.z_ / z_;
                if(fabs(scale_factor * x_ - vector.x_) < valve
                    && fabs(scale_factor * y_ - vector.y_) < valve)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
        default:
            return false;
    }
}

bool Vector3::isVertical(Vector3& vector, double valve) const
{
    if(fabs(x_ * vector.x_ + y_ * vector.y_ + z_ * vector.z_) > valve)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void Vector3::reverse()
{
    x_ = -x_;
    y_ = -y_;
    z_ = -z_;
}

double Vector3::distanceToPoint(Vector3& vector) const
{
    double delta_x = x_ - vector.x_;
    double delta_y = y_ - vector.y_;
    double delta_z = z_ - vector.z_;
    return sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
}

Vector3 Vector3::getPointBetween(Vector3& end_point, double distance_to_start)
{
    Vector3 result;
    Vector3 direction_vector = end_point - *this;
    if(!direction_vector.normalize())
    {
        result = *this;
    }
    else
    {
        result = *this + direction_vector * distance_to_start;
    }
    return result;
}

double Vector3::norm() const
{
    return sqrt(x_ * x_ + y_ * y_ + z_ * z_);
}

bool Vector3::normalize(double valve)
{
    double norm = this->norm();
    if(norm >= valve)
    {
        x_ = x_ / norm;
        y_ = y_ / norm;
        z_ = z_ / norm;
        return true;
    }
    else
    {
        return false;
    }
}

double Vector3::dotProduct(const Vector3& vector) const
{
    return (x_ * vector.x_ + y_ * vector.y_ + z_ * vector.z_);
}

void Vector3::crossProduct(const Vector3& by, Vector3& result)
{
    if(&result != this)
    {
        result.x_ = y_ * by.z_ - z_ * by.y_;
        result.y_ = z_ * by.x_ - x_ * by.z_;
        result.z_ = x_ * by.y_ - y_ * by.x_;
    }
    else
    {
        crossProduct(by);
    }
}

void Vector3::crossProduct(const Vector3& by)
{
    Vector3 tmp;
    tmp.x_ = y_ * by.z_ - z_ * by.y_;
    tmp.y_ = z_ * by.x_ - x_ * by.z_;
    tmp.z_ = x_ * by.y_ - y_ * by.x_;
    *this = tmp;
}

double& Vector3::operator[](size_t index) 
{
    assert(index < 3); 
    return *(&x_ + index);
}

const double& Vector3::operator[](size_t index) const 
{
    assert(index < 3); 
    return *(&x_ + index);
}

const Vector3 Vector3::operator+(const Vector3& point)
{
    Vector3 result;
    result.x_ = x_ + point.x_;
    result.y_ = y_ + point.y_;
    result.z_ = z_ + point.z_;
    return result;
}

const Vector3 Vector3::operator-(const Vector3& vector)
{
    Vector3 result;
    result.x_ = x_ - vector.x_;
    result.y_ = y_ - vector.y_;
    result.z_ = z_ - vector.z_;
    return result;
}

Vector3& Vector3::operator+=(const Vector3& vector)
{
    x_ += vector.x_;
    y_ += vector.y_;
    z_ += vector.z_;
    return *this;
}

Vector3& Vector3::operator-=(const Vector3& vector)
{
    x_ -= vector.x_;
    y_ -= vector.y_;
    z_ -= vector.z_;
    return *this;
}

const Vector3 Vector3::operator*(double value)
{
    Vector3 result;    
    result.x_ = value * x_;
    result.y_ = value * y_;
    result.z_ = value * z_;
    return result;
}

void Vector3::print(std::string comment) const
{
    std::cout<<comment<<std::endl;
    std::cout<<" x = "<<x_
             <<" y = "<<y_
             <<" z = "<<z_<<std::endl;
}

