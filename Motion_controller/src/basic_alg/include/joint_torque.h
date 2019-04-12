#ifndef JOINT_TORQUE_H
#define JOINT_TORQUE_H

#include <stddef.h>
#include <string>


namespace basic_alg
{

class JointTorque
{
public:
    double   t1_;
    double   t2_;
    double   t3_;
    double   t4_;
    double   t5_;
    double   t6_;
    double   t7_;
    double   t8_;
    double   t9_;

    bool isEqual(JointTorque& joint_torque, double valve = 0.001) const;

    double& operator[](size_t index);
    const double& operator[](size_t index) const;
    const JointTorque operator+(const JointTorque& joint_torque);
    const JointTorque operator-(const JointTorque& joint_torque);
    JointTorque& operator+=(const JointTorque& joint_torque);
    JointTorque& operator-=(const JointTorque& joint_torque);

    void print(std::string comment = "") const;

};

}


#endif

