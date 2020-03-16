#ifndef POINT_H
#define POINT_H


#include <stddef.h>
#include <string>


namespace basic_alg
{

class Point
{
public:
    double x_;
    double y_;
    double z_;

    bool isEqual(const Point& point, double valve = 0.001) const;
    bool isParallel(const Point& point, double valve = 0.001) const;
    bool isVertical(const Point& point, double valve = 0.001) const;
    void reverse();
    double distanceToPoint(const Point& point) const;
    double norm() const;
    bool normalize(double valve = 0.001);
    double dotProduct(const Point& point) const;
    void crossProduct(const Point& by, Point& result) const;
    void crossProduct(const Point& by);

    
    double& operator[](size_t index);
    const double& operator[](size_t index) const;
    const Point operator+(const Point& point) const;
    const Point operator-(const Point& point) const;
    Point& operator+=(const Point& point);
    Point& operator-=(const Point& point);
    const Point operator*(double value) const;
    
    void print(std::string comment = "") const;
};


}



#endif

