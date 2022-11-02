#include "point.h"
#include <assert.h>
#include <math.h>
#include <iostream>

using namespace std;
using namespace basic_alg;


bool Point::isEqual(const Point& point, double valve) const
{
    if(fabs(x_ - point.x_) < valve
        && fabs(y_ - point.y_) < valve
        && fabs(z_ - point.z_) <valve)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Point::isParallel(const Point& point, double valve) const
{
    double max1;
    int max_index;
    max1 = point.x_;
    max_index = 1;
    if(fabs(max1) < fabs(point.y_))
    {
        max1 = point.y_;
        max_index = 2;
    }
    if(fabs(max1) < fabs(point.z_))
    {
        max1 = point.z_;
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
                scale_factor = point.x_ / x_;
                if(fabs(scale_factor * y_ - point.y_) < valve
                    && fabs(scale_factor * z_ - point.z_) < valve)
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
                scale_factor = point.y_ / y_;
                if(fabs(scale_factor * x_ - point.x_) < valve
                    && fabs(scale_factor * z_ - point.z_) < valve)
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
                scale_factor = point.z_ / z_;
                if(fabs(scale_factor * x_ - point.x_) < valve
                    && fabs(scale_factor * y_ - point.y_) < valve)
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

bool Point::isVertical(const Point& point, double valve) const
{
    if(fabs(x_ * point.x_ + y_ * point.y_ + z_ * point.z_) > valve)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void Point::reverse()
{
    x_ = -x_;
    y_ = -y_;
    z_ = -z_;
}

void Point::zero()
{
    x_ = 0.0;
    y_ = 0.0;
    z_ = 0.0;
}

double Point::distanceToPoint(const Point& point) const
{
    double delta_x = x_ - point.x_;
    double delta_y = y_ - point.y_;
    double delta_z = z_ - point.z_;
    return sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
}

double Point::norm() const
{
    return sqrt(x_ * x_ + y_ * y_ + z_ * z_);
}

bool Point::normalize(double valve)
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

double Point::dotProduct(const Point& point) const
{
    return (x_ * point.x_ + y_ * point.y_ + z_ * point.z_);
}

void Point::crossProduct(const Point& by, Point& result) const
{
    result.x_ = y_ * by.z_ - z_ * by.y_;
    result.y_ = z_ * by.x_ - x_ * by.z_;
    result.z_ = x_ * by.y_ - y_ * by.x_;
}

void Point::crossProduct(const Point& by)
{
    Point tmp;
    tmp.x_ = y_ * by.z_ - z_ * by.y_;
    tmp.y_ = z_ * by.x_ - x_ * by.z_;
    tmp.z_ = x_ * by.y_ - y_ * by.x_;
    *this = tmp;
}

double& Point::operator[](size_t index) 
{
    assert(index < 3); 
    return *(&x_ + index);
}

const double& Point::operator[](size_t index) const 
{
    assert(index < 3); 
    return *(&x_ + index);
}

const Point Point::operator+(const Point& point) const
{
    Point result;
    result.x_ = x_ + point.x_;
    result.y_ = y_ + point.y_;
    result.z_ = z_ + point.z_;
    return result;
}

const Point Point::operator-(const Point& point) const
{
    Point result;
    result.x_ = x_ - point.x_;
    result.y_ = y_ - point.y_;
    result.z_ = z_ - point.z_;
    return result;
}


Point& Point::operator+=(const Point& point)
{
    x_ += point.x_;
    y_ += point.y_;
    z_ += point.z_;
    return *this;
}

Point& Point::operator-=(const Point& point)
{
    x_ -= point.x_;
    y_ -= point.y_;
    z_ -= point.z_;
    return *this;
}

const Point Point::operator*(double value) const
{
    Point result;    
    result.x_ = value * x_;
    result.y_ = value * y_;
    result.z_ = value * z_;
    return result;
}


const Point Point::operator/(double value) const
{
    Point result;
    if (value != 0)
    {
        result.x_ = x_ / value;
        result.y_ = y_ / value;
        result.z_ = z_ / value;
    }
    else
    {
        result.x_ = 0;
        result.y_ = 0;
        result.z_ = 0;
    }
    return result;
}

void Point::print(std::string comment) const
{
    std::cout<<comment<<std::endl;
    std::cout<<" x = "<<x_
             <<" y = "<<y_
             <<" z = "<<z_<<std::endl;
}

