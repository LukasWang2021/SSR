#ifndef JOINT_H
#define JOINT_H

#include <stddef.h>
#include <string>

namespace basic_alg
{

class Joint
{
public:
    double   j1_;
    double   j2_;
    double   j3_;
    double   j4_;
    double   j5_;
    double   j6_;
    double   j7_;
    double   j8_;
    double   j9_;

    Joint();
    ~Joint();

    bool isEqual(Joint& joint, double valve = 0.001) const;

    double& operator[](size_t index);
    const double& operator[](size_t index) const;
    const Joint operator+(const Joint& joint);
    const Joint operator-(const Joint& joint);
    Joint& operator+=(const Joint& joint);
    Joint& operator-=(const Joint& joint);

    void print(std::string comment = "") const;

};

}


#endif

