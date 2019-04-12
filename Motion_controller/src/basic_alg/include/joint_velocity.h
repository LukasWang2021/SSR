#ifndef JOINT_VELOCITY_H
#define JOINT_VELOCITY_H

#include <stddef.h>
#include <string>


namespace basic_alg
{

class JointVelocity
{
public:
    double   v1_;
    double   v2_;
    double   v3_;
    double   v4_;
    double   v5_;
    double   v6_;
    double   v7_;
    double   v8_;
    double   v9_;

    bool isEqual(JointVelocity& joint_vel, double valve = 0.001) const;

    double& operator[](size_t index);
    const double& operator[](size_t index) const;
    const JointVelocity operator+(const JointVelocity& joint_vel);
    const JointVelocity operator-(const JointVelocity& joint_vel);
    JointVelocity& operator+=(const JointVelocity& joint_vel);
    JointVelocity& operator-=(const JointVelocity& joint_vel);

    void print(std::string comment = "") const;

};

}


#endif

