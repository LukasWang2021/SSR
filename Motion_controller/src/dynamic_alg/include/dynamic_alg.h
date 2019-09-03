#ifndef DYNAMICS_H
#define DYNAMICS_H

#include "error_code.h"
#include "basic_alg_datatype.h"
#include "dynamic_alg_payload.h"

namespace basic_alg
{

typedef struct
{
    double XXR;
    double XY;
    double XYR;
    double XZ;
    double XZR;
    double YZ;
    //double YZR;//not use now.
    double ZZ;
    double ZZR;
    double MX;
    double MXR;
    double MY;
    double MYR;
    double Im;
    double FS;
    double FV;
}DynamicAlgParam;

typedef struct
{
    double m_load;
    double lcx_load;
    double lcy_load;
    double lcz_load;
    double Ixx_load;
    double Iyy_load;
    double Izz_load;
    double Ixy_load;
    double Ixz_load;
    double Iyz_load;
}DynamicAlgLoadParam;
/*
typedef struct
{
    double d;
    double a;
    double alpha;
    double offset;
}DH;
*/
class DynamicAlg
{
public:
    virtual ~DynamicAlg(){}

    virtual bool initDynamicAlg(std::string file_path) = 0;
    
    virtual bool isValid() = 0;

    virtual bool getTorqueInverseDynamics(const Joint& joint, const JointVelocity& vel, const JointAcceleration& acc, 
                                               JointTorque &torque) = 0;

    virtual bool getAccMax(const Joint& joint, const JointVelocity& vel, JointAcceleration &acc_pos, JointAcceleration &acc_neg) = 0;
    virtual bool getAccDirectDynamics(const Joint& joint, const JointVelocity& vel, const JointTorque& torque,
                                            JointAcceleration &acc) = 0;

    virtual ErrorCode setPayload(int id) = 0;
    virtual void getPayload(int &id) = 0;

    virtual ErrorCode addPayload(const PayloadInfo& info) = 0;
    virtual ErrorCode deletePayload(int id) = 0;
    virtual ErrorCode updatePayload(const PayloadInfo& info) = 0;
    virtual ErrorCode movePayload(int expect_id, int original_id) = 0;
    virtual ErrorCode getPayloadInfoById(int id, PayloadInfo& info) = 0;
    virtual std::vector<PayloadSummaryInfo> getAllValidPayloadSummaryInfo(void) = 0;
    virtual void getAllValidPayloadSummaryInfo(std::vector<PayloadSummaryInfo>& info_list) = 0;  

};

}

#endif



