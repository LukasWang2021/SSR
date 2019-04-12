#ifndef JOINT_ACCELERATION_H
#define JOINT_ACCELERATION_H

#include <stddef.h>
#include <string>


namespace basic_alg
{

class JointAcceleration
{
public:
    double   a1_;
    double   a2_;
    double   a3_;
    double   a4_;
    double   a5_;
    double   a6_;
    double   a7_;
    double   a8_;
    double   a9_;

    bool isEqual(JointAcceleration& joint_acc, double valve = 0.001) const;

    double& operator[](size_t index);
    const double& operator[](size_t index) const;
    const JointAcceleration operator+(const JointAcceleration& joint_acc);
    const JointAcceleration operator-(const JointAcceleration& joint_acc);
    JointAcceleration& operator+=(const JointAcceleration& joint_acc);
    JointAcceleration& operator-=(const JointAcceleration& joint_acc);

    void print(std::string comment = "") const;

};

}


#endif

