#ifndef _ONLINE_TRJ_PLANNER_H_
#define _ONLINE_TRJ_PLANNER_H_
#include "basic_alg.h"
#include "quaternion.h"
#include "matrix33.h"
#include "vector3.h"
#include <algorithm>
#include <string>
#include "yaml_help.h"

/**
 * @brief structure of online trajectory points
 * @details 
 * @param status status of points
 * @param x_ x value in cartesian coordinates
 * @param y_ y value in cartesian coordinates
 * @param z_ z value in cartesian coordinates
 * @param a_ a value in quaternian
 * @param b_ b value in quaternian
 * @param c_ c value in quaternian
 * @return 
 */
typedef struct {
    int  status;
    double x_;
    double y_;
    double z_;
    double a_;
    double b_;
    double c_;
}TrjPoint;

/**
 * @brief structure of onlineTrjAlgParam
 * @details 
 * @param sample_time touch sampling time interval
 * @param generate_traj_interval output trajectory interval
 * @param N_step_P sampling interval of getting VP points - suggesting range <= 5
 * @param N_step_Q sampling interval of getting VQ points
 * @param N_interp_P ((NstepP*Tsample)/GEN_TN)
 * @param N_interp_Q ((NstepQ*Tsample)/GEN_TN)
 * @param trj_ratio_xyz xyz mapping ratio
 * @param trj_ratio_abc abc mapping ratio
 * @param online_receive_Tmatrix_buff_len cache volume of receving matrixes from touch, configure in yaml
 * @return 
 */
typedef struct{
    double sample_time;
    double generate_traj_interval;
    int N_step_P;
    int N_step_Q;
    double N_interp_P;
    double N_interp_Q;
    double trj_ratio_xyz;
    double trj_ratio_abc;
    int online_receive_Tmatrix_buff_len;
}onlineTrjAlgParam;

using namespace std;
using namespace basic_alg;

/**
 * @brief main structure of OnlineTrajectoryPlanner
 */
class OnlineTrajectoryPlanner
{
public:
    // storage of necessary parameters
    onlineTrjAlgParam online_alg_params_;
    // temporary cache buffter for Bspline algorithm output
    TrjPoint trj_point_buf[160];
    // data of receving matrixes from 3Dtouch device
    int receive_TmatrixCnt = 0;
    // count of the number of receving matrixes from 3Dtouch device
    int read_TmatrixCnt = 0; 

    // constructor
    OnlineTrajectoryPlanner();

    // destructor
    ~OnlineTrajectoryPlanner();

    /**
    * @brief get double sign
    * @param x the input double number
    * @return integer, -1 for x < 0, else return 0
    */
    int sign(double x);

    
    /**
    * @brief turn euler angles (Radians) to rotating matrix
    * @param [in] a euler angle
    * @param [in] b euler angle
    * @param [in] c euler angle
    * @return rotating matrix
    */
    Matrix33 rpy2r(double a, double b, double c);


    /**
    * @brief turn rotating matrix (Matrix33&) to euler angles
    * @param [in] R the input Matrix33&
    * @return with euler angles data
    */
    Vector3 rtm_rpy(Matrix33& R);


    /**
    * @brief turn input matrix inverse, and write into the second parameter
    * @param [in] m44 the input Matrix44
    * @param [out] resT the output Matrix44
    * @return void
    */
    void get_matrix44f_inverse(Matrix44 m44,Matrix44 &resT);


    /**
    * @brief turn rotating matrix to quaternion
    * @param [in] R the input Matrix33&
    * @return the transfered quaternion
    */
    Quaternion rtm_r2quat(Matrix33& R);


    /**
    * @brief turn quaternion to rotating matrix
    * @param [in] q the input Quaternion&
    * @return the transfered Matrix33
    */
    Matrix33 rtm_quat2r(Quaternion& q);
    

    /**
    * @brief get the inverse of quaternion
    * @param [in] q the input Quaternion&
    * @return the inversed quaternion
    */
    Quaternion quatinv(Quaternion& q);


    /**
    * @brief get the logarithm of quaternion
    * @param [in] q the input Quaternion&
    * @return the logarithm quaternion
    */
    Quaternion quatlog(Quaternion& q);


    /**
    * @brief get the exponential quaternion of input quaternion
    * @param [in] q the input Quaternion&
    * @return the exponential quaternion
    */
    Quaternion quatexp(Quaternion& q);
    

    /**
    * @brief quaternion cross product 
    * @param [in] q1 the input Quaternion&
    * @param [in] q2 the input Quaternion&
    * @return result in quaternion
    */
    Quaternion quatmultiply(Quaternion& q1, Quaternion& q2);


    /**
    * @brief normalize Q and turn it into euler angle 
    * @param [in] Q the input Quaternion&
    * @return result in Vector3
    */
    Vector3 rtm_quat2abc(Quaternion& Q);


    /**
    * @brief turn euler angle to quaternion
    * @param [in] abc the input euler angle in Vector3
    * @return result in Quaternion
    */
    Quaternion rtm_abc2quat(Vector3& abc);


    Quaternion rtm_Slerpt(Quaternion& q0, Quaternion& q1, double t);


    Quaternion rtm_Squad(Quaternion& q0, Quaternion& q1, Quaternion& q2,Quaternion& q3,double t);
    
    
    int traj_on_FIR_Bspline(Vector3 xyz, Vector3 abc,int status, int online_TrjpointBufIndex);
    
    void Fir_Bspline_algorithm_test(void);
    void Fir_Bspline_algorithm_test2(void);


    Matrix44 rtm_reorthog(Matrix44 &T);
    Matrix33 rtm_reorthog(Matrix33 &T);


    bool DynamicBaseCoordTransformation(Matrix44 T_r0_R, Matrix44 Touch_h0_v,  Matrix44 Touch_ht_v, double k_xyz,double k_abc, Matrix44& resM);
    bool get_increment_matrix(Matrix44 T_ck,Matrix44 T_k1, Matrix44 T_k, Matrix44 &resT);
    void rtm_r2xyzabc(Matrix44& u,Vector3& res_xyz, Vector3& res_abc);
   
    
    void online_trajectory_algorithm_params_init();

    int setOnlineTrjRatio_xyz(double data_ratio);
    int setOnlineTrjRatio_abc(double data_ratio);

    double get_online_trj_ratio_xyz();
    double get_online_trj_ratio_abc();

    bool load_OnlineMove_params_Config();


    // ------------------------------- unused or undefined functions ------------------------------------------

    /**
    * @brief set double precision
    * @brief function for old algorithms, not use any more, but DO NOT DELETE
    * @param x the input double number
    * @param precision the precision number
    * @return double
    */
    //double roundn(double x, int precision);

    /**
    * @brief turn vector3 to diagnol matrix
    * @brief function for old algorithms, not use any more, but DO NOT DELETE
    * |x|         | 0   -z   y | 
    * |y|         | z   0   -x | 
    * |z|         |-y   x    0 | 
    * @param [in] vec3 the input Vector3&
    * @return the transfered Matrix33
    */
    //Matrix33 skew(Vector3& vec3);

    /**
    * @brief quaternion dot product 
    * @brief function for old algorithms, not use any more, but DO NOT DELETE
    * @param [in] q1 the input Quaternion&
    * @param [in] q2 the input Quaternion&
    * @return result in double
    */
    //double quat_dot_multiply(Quaternion& q1, Quaternion& q2);

    /**
    * @brief function for old algorithms, not use any more, but DO NOT DELETE
    */
    //Vector3 update_wgoal(Matrix33& M, Quaternion& qtemp, Vector3& alpha, double tTmin, double ts);

    /**
    * @brief function for old algorithms, not use any more, but DO NOT DELETE
    */
    //Vector3 rtm_abcDiff(Vector3& abc,Vector3 abc_rate);

    /**
    * @brief function for old algorithms, not use any more, but DO NOT DELETE
    */
    //void traj_on_FB(int m, int lambda, int Nstep, Vector3 VPp_point[], int NVP, Vector3 Pout[]);
    //Vector3 * traj_on_FB(int m, int lambda, int Nstep, Vector3 * VPp);

    /**
    * @brief function for old algorithms, not use any more, but DO NOT DELETE
    */
    //void traj_on_Squad(int Nstep, Vector3* VPp, Quaternion* Qnew, Vector3* abc, Quaternion* Qold);
    //void traj_on_Squad(int Nstep, Vector3 VPp_abc[], int NVP, Quaternion Qnew[], Vector3 abc[], Quaternion Qold[]);
    
    //int traj_on_FIR_Bspline(Vector3 xyz, Vector3 abc,int status);

    //void TrjPointPlanning();

    //void pointer_fuzhi_test(Quaternion q_in[], int num, Quaternion q_out[]);

    //void function_test();

    //bool FixedBaseCoordTransformation(Matrix44& T_touchm,Matrix44& resM);

    
    // ------------------------------------------- end ----------------------------------------------------------

private:
    // yaml help
    base_space::YamlHelp yaml_help_;

    // parameter file path of online trajectory planing
    std::string config_OnlineMove_params_file_path_;
};

#endif