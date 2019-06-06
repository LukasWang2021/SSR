#ifndef DYNAMIC_ALG_RTM_H
#define DYNAMIC_ALG_RTM_H

#include "axis_group_model.h"
#include "dynamic_alg.h"
#include "dynamic_alg_rtm_param.h"
#include "common_log.h"
#include "parameter_manager/parameter_manager_param_group.h"
#include "kinematics_rtm.h"


namespace basic_alg
{

class DynamicAlgRTM : public DynamicAlg
{
public:
    DynamicAlgRTM();
    ~DynamicAlgRTM();

    virtual bool initDynamicAlg(std::string file_path);

    virtual bool updateLoadParam(void);
    virtual bool updateLoadParam(DynamicAlgLoadParam load_param);

    virtual bool isValid();

    virtual bool getTorqueInverseDynamics(const Joint& joint, const JointVelocity& vel, const JointAcceleration& acc, 
                                               JointTorque &torque);

    virtual bool getAccMax(const Joint& joint, const JointVelocity& vel, JointAcceleration &acc_pos, JointAcceleration &acc_neg);
    virtual bool getAccDirectDynamics(const Joint& joint, const JointVelocity& vel, const JointTorque& torque, 
                                           JointAcceleration &acc);

    static const double DYN_DOUBLE_ACCURACY = 1e-6;
    static const double G = 9.81;
    static const int LINKS = 6;
    static const int PARAM_SET = 52;

private:

    void computePaiElementInverseDynamics(DynamicAlgParam* dynamics_alg_param_ptr, size_t link_num);
    void computePaiElementInverseDynamics(void);
    void computeMExpression(const Joint& joint, double m[LINKS][LINKS]);
    void computeCExpression(const Joint& joint, const JointVelocity& vel, double c[LINKS][LINKS]);
    void computeGExpression(const Joint& joint, double g[LINKS]);
    void computeTauHe(const Joint& joint, double tau_he[LINKS]);
    void computeMatrixElementInverseDynamics(const Joint& joint, const JointVelocity& vel, const JointAcceleration& acc);
    
    int sign(double value);
    //matrix inverse alg
    bool getMatrixInverse(const double src[LINKS][LINKS], int n, double dest[LINKS][LINKS]);
    double getMatrixRank(const double src[LINKS][LINKS], int n);
    void getAdjointMatrix(const double src[LINKS][LINKS], int n, double dest[LINKS][LINKS]);

    //matrix LU alg
    bool getMatrixLUInverse(const double src[LINKS][LINKS], int n, double dest[LINKS][LINKS]);
    bool matrixLUPDecomposition(double A[LINKS*LINKS], double L[LINKS*LINKS], double U[LINKS*LINKS], int P[LINKS]);
    bool matrixLUPSolve(const double L[LINKS*LINKS], const double U[LINKS*LINKS], const int P[LINKS], const double b[LINKS], double inv_A_column[LINKS]);
    bool matrixTranspose(double matrix[LINKS*LINKS], int m, int n);

    DH base_dh_;
    DH arm_dh_[6];
    DynamicAlgRTMParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    fst_parameter::ParamGroup param_;
    std::string file_path_;
    bool is_valid_;

    //input values.
    double q1, q2, q3, q4, q5, q6, q7, q8, q9;
    double dq1, dq2, dq3, dq4, dq5, dq6, dq7, dq8, dq9;
    double ddq1, ddq2, ddq3, ddq4, ddq5, ddq6, ddq7, ddq8, ddq9;
    double ZZR1, FS1, FV1, XXR2, XY2, XZR2, YZ2, ZZR2, MXR2, MY2, FS2, FV2, XXR3, XYR3, XZ3, YZ3, ZZR3, MXR3, MYR3, Im3, FS3, FV3,
           XXR4, XY4, XZ4, YZ4, ZZR4, MX4, MYR4, Im4, FS4, FV4, XXR5, XY5, XZ5, YZ5, ZZR5, MX5, MYR5, Im5, FS5, FV5,
           XXR6, XY6, XZ6, YZ6, ZZ6, MX6, MY6, Im6, FS6, FV6;//puma, 52 parameters.
    double m_load, lcx_load, lcy_load, lcz_load, Ixx_load, Iyy_load, Izz_load, Ixy_load, Ixz_load, Iyz_load;//load param set.
    
    //for A matrix of inverse dynamics
    double a2, a3, a4, d4, d6;


    //middle variable by /test/matlab_convert.cpp
    double C1,S1,C2,S2,C3,S3,C4,S4,C5,S5;
    double C6,S6,DV61,W12,W22,WP12,WP22,DV22,DV32,DV42;
    double DV52,DV62,U112,U212,U312,U322,VSP12,VSP22,VP12,VP22;
    double W13,W23,W33,WP13,WP23,WP33,DV13,DV23,DV33,DV43;
    double DV53,DV63,U113,U213,U313,U123,U223,U323,VSP13,VSP23;
    double VSP33,VP13,VP23,W14,W24,W34,WP14,WP24,WP34,DV14;
    double DV24,DV34,DV44,DV54,DV64,U114,U214,U314,U124,U224;
    double U324,VSP14,VSP24,VSP34,VP14,VP24,W15,W25,W35,WP15;
    double WP25,WP35,DV15,DV25,DV35,DV45,DV55,DV65,U115,U215;
    double U315,U125,U225,U325,VP15,VP25,W16,W26,W36,WP16;
    double WP26,WP36,DV16,DV26,DV36,DV46,DV56,DV66,U116,U216;
    double U316,U126,U226,U326,VSP16,VSP26,VSP36,VP16,VP26,DG1ZZ1;
    double DG1FS1,DG1FV1,N13XX2,DG1XX2,DG2XX2,No21XY2,No22XY2,No23XY2,N13XY2,DG1XY2;
    double DG2XY2,No21XZ2,No22XZ2,No23XZ2,N13XZ2,DG1XZ2,DG2XZ2,No21YZ2,No22YZ2,No23YZ2;
    double N13YZ2,DG1YZ2,DG2YZ2,N13ZZ2,DG1ZZ2,DG2ZZ2,N13MX2,DG1MX2,DG2MX2,N13MY2;
    double DG1MY2,DG2MY2,DG2FS2,DG2FV2,N21XX3,N22XX3,N13XX3,DG1XX3,DG2XX3,DG3XX3;
    double No31XY3,No32XY3,No33XY3,N21XY3,N22XY3,N13XY3,DG1XY3,DG2XY3,DG3XY3,No31XZ3;
    double No32XZ3,No33XZ3,N21XZ3,N22XZ3,N13XZ3,DG1XZ3,DG2XZ3,DG3XZ3,No31YZ3,No32YZ3;
    double No33YZ3,N21YZ3,N22YZ3,N13YZ3,DG1YZ3,DG2YZ3,DG3YZ3,N21ZZ3,N22ZZ3,N13ZZ3;
    double DG1ZZ3,DG2ZZ3,DG3ZZ3,FDI32MX3,N21MX3,N22MX3,N23MX3,N13MX3,DG1MX3,DG2MX3;
    double DG3MX3,FDI32MY3,N21MY3,N22MY3,N23MY3,N13MY3,DG1MY3,DG2MY3,DG3MY3,DG3Im3;
    double DG3FS3,DG3FV3,N31XX4,N33XX4,N21XX4,N22XX4,N13XX4,DG1XX4,DG2XX4,DG3XX4;
    double DG4XX4,No41XY4,No42XY4,No43XY4,N31XY4,N33XY4,N21XY4,N22XY4,N13XY4,DG1XY4;
    double DG2XY4,DG3XY4,DG4XY4,No41XZ4,No42XZ4,No43XZ4,N31XZ4,N33XZ4,N21XZ4,N22XZ4;
    double N13XZ4,DG1XZ4,DG2XZ4,DG3XZ4,DG4XZ4,No41YZ4,No42YZ4,No43YZ4,N31YZ4,N33YZ4;
    double N21YZ4,N22YZ4,N13YZ4,DG1YZ4,DG2YZ4,DG3YZ4,DG4YZ4,N31ZZ4,N33ZZ4,N21ZZ4;
    double N22ZZ4,N13ZZ4,DG1ZZ4,DG2ZZ4,DG3ZZ4,DG4ZZ4,FDI41MX4,FDI43MX4,N31MX4,N32MX4;
    double N33MX4,FDI32MX4,N21MX4,N22MX4,N23MX4,N13MX4,DG1MX4,DG2MX4,DG3MX4,DG4MX4;
    double FDI41MY4,FDI43MY4,N31MY4,N32MY4,N33MY4,FDI32MY4,N21MY4,N22MY4,N23MY4,N13MY4;
    double DG1MY4,DG2MY4,DG3MY4,DG4MY4,DG4Im4,DG4FS4,DG4FV4,N41XX5,N43XX5,N31XX5;
    double N33XX5,N21XX5,N22XX5,N13XX5,DG1XX5,DG2XX5,DG3XX5,DG4XX5,DG5XX5,No51XY5;
    double No52XY5,No53XY5,N41XY5,N43XY5,N31XY5,N33XY5,N21XY5,N22XY5,N13XY5,DG1XY5;
    double DG2XY5,DG3XY5,DG4XY5,DG5XY5,No51XZ5,No52XZ5,No53XZ5,N41XZ5,N43XZ5,N31XZ5;
    double N33XZ5,N21XZ5,N22XZ5,N13XZ5,DG1XZ5,DG2XZ5,DG3XZ5,DG4XZ5,DG5XZ5,No51YZ5;
    double No52YZ5,No53YZ5,N41YZ5,N43YZ5,N31YZ5,N33YZ5,N21YZ5,N22YZ5,N13YZ5,DG1YZ5;
    double DG2YZ5,DG3YZ5,DG4YZ5,DG5YZ5,N41ZZ5,N43ZZ5,N31ZZ5,N33ZZ5,N21ZZ5,N22ZZ5;
    double N13ZZ5,DG1ZZ5,DG2ZZ5,DG3ZZ5,DG4ZZ5,DG5ZZ5,FDI51MX5,FDI53MX5,N41MX5,N43MX5;
    double FDI41MX5,FDI43MX5,N31MX5,N32MX5,N33MX5,FDI32MX5,N21MX5,N22MX5,N23MX5,N13MX5;
    double DG1MX5,DG2MX5,DG3MX5,DG4MX5,DG5MX5,FDI51MY5,FDI53MY5,FDI41MY5,FDI43MY5,N31MY5;
    double N32MY5,N33MY5,FDI32MY5,N21MY5,N22MY5,N23MY5,N13MY5,DG1MY5,DG2MY5,DG3MY5;
    double DG4MY5,DG5MY5,DG5Im5,DG5FS5,DG5FV5,N51XX6,N53XX6,N41XX6,N43XX6,N31XX6;
    double N33XX6,N21XX6,N22XX6,N13XX6,DG1XX6,DG2XX6,DG3XX6,DG4XX6,DG5XX6,DG6XX6;
    double No61XY6,No62XY6,No63XY6,N51XY6,N53XY6,N41XY6,N43XY6,N31XY6,N33XY6,N21XY6;
    double N22XY6,N13XY6,DG1XY6,DG2XY6,DG3XY6,DG4XY6,DG5XY6,DG6XY6,No61XZ6,No62XZ6;
    double No63XZ6,N51XZ6,N53XZ6,N41XZ6,N43XZ6,N31XZ6,N33XZ6,N21XZ6,N22XZ6,N13XZ6;
    double DG1XZ6,DG2XZ6,DG3XZ6,DG4XZ6,DG5XZ6,DG6XZ6,No61YZ6,No62YZ6,No63YZ6,N51YZ6;
    double N53YZ6,N41YZ6,N43YZ6,N31YZ6,N33YZ6,N21YZ6,N22YZ6,N13YZ6,DG1YZ6,DG2YZ6;
    double DG3YZ6,DG4YZ6,DG5YZ6,DG6YZ6,N51ZZ6,N53ZZ6,N41ZZ6,N43ZZ6,N31ZZ6,N33ZZ6;
    double N21ZZ6,N22ZZ6,N13ZZ6,DG1ZZ6,DG2ZZ6,DG3ZZ6,DG4ZZ6,DG5ZZ6,DG6ZZ6,FDI61MX6;
    double FDI63MX6,N51MX6,N53MX6,FDI51MX6,FDI53MX6,N41MX6,N43MX6,FDI41MX6,FDI43MX6,N31MX6;
    double N32MX6,N33MX6,FDI32MX6,N21MX6,N22MX6,N23MX6,N13MX6,DG1MX6,DG2MX6,DG3MX6;
    double DG4MX6,DG5MX6,DG6MX6,FDI61MY6,FDI63MY6,N51MY6,N53MY6,FDI51MY6,FDI53MY6,N41MY6;
    double N43MY6,FDI41MY6,FDI43MY6,N31MY6,N32MY6,N33MY6,FDI32MY6,N21MY6,N22MY6,N23MY6;
    double N13MY6,DG1MY6,DG2MY6,DG3MY6,DG4MY6,DG5MY6,DG6MY6,DG6Im6,DG6FS6,DG6FV6;

};
}


#endif


