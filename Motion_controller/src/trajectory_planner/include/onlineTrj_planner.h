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
    //int sign(double x);

    
    /**
    * @brief turn euler angles (Radians) to rotating matrix (ZYX)
    * @param [in] x euler angle
    * @param [in] y euler angle
    * @param [in] z euler angle
    * @return A Matrix33, which is a rotation matrix
    */
    Matrix33 online_turnEulerMatrix(double x, double y, double z);


    /**
    * @brief turn rotating matrix (Matrix33&) to euler angles
    * @param [in] rotation_matrix the input Matrix33&
    * @return euler angles data (XYZ)
    */
    Vector3 online_turnMatrixEuler(Matrix33& rotation_matrix);
    Euler online_turnMatrixEuler_(Matrix33& rotation_matrix);

    /**
    * @brief turn input matrix inverse, and write into the second parameter
    * @param [in] matrix_in the input Matrix44
    * @param [out] matrix_out the address of output Matrix44 (&matrix_out)
    * @return void
    */
    void online_getMatrixInv(Matrix44 matrix_in,Matrix44 &matrix_out);
    void online_getMatrixInv(TransMatrix m44,TransMatrix &resT);


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

    /**
    * @brief get interpolation from q0 to q1
    * @param [in] q0 begin quaternion
    * @param [in] q1 end quaternion
    * @param [in] t time
    * @return result in Quaternion
    */
    Quaternion rtm_Slerpt(Quaternion& q0, Quaternion& q1, double t);

    /**
    * @brief get interpolation from q1 to q2, q0 and q3 are vp points for algorithm
    * @param [in] q0 vp point at front
    * @param [in] q1 begin quaternion
    * @param [in] q2 end quaternion
    * @param [in] q3 vp point follow behind
    * @param [in] t time
    * @return result in Quaternion
    */
    Quaternion rtm_Squad(Quaternion& q0, Quaternion& q1, Quaternion& q2,Quaternion& q3,double t);
    
    /**
    * @brief smooth algorithm using Bspline
    * @param [in] xyz cartesian data
    * @param [in] abc quaternion data
    * @param [in] status 0 means beginning, 1 means moving(holding touch button), 2 means stop(release touch button)
    * @param [in] online_TrijpointBufIndex get index of output from buffer list 'trj_point_buf[]'
    * @return number of total points
    */
    int traj_on_FIR_Bspline(Point xyz, Euler abc,int status, int online_TrjpointBufIndex);
    //int traj_on_FIR_Bspline(Vector3 xyz, Vector3 abc,int status, int online_TrjpointBufIndex);

    
    /**
    * @brief orthogonalize the input matrix
    * @param [in] T input Matrix33 / Matrix44 / RotationMatrix
    * @details this function WILL NOT change the input matrix variable
    * @return the orthogonalized matrix
    */
    Matrix44 rtm_reorthog(Matrix44 &T);
    Matrix33 rtm_reorthog(Matrix33 &T);
    RotationMatrix rtm_reorthog(RotationMatrix &T);


    /**
    * @brief transfer Matrix44 to TransMatrix
    * @param [in] ma the input Matrix44 type
    * @param [out] mb the output TransMatrix type, should be empty and initialized for input
    * @return void
    */
    void turnM2T(const Matrix44 ma, TransMatrix mb);

    /**
    * @brief transfer TransMatrix to Matrix44
    * @param [in] ma the input TransMatrix type
    * @param [out] mb the output Matrix44 type, should be empty and initialized for input
    * @details this function will clean mb before data transfer
    * @return void
    */
    void turnT2M(const TransMatrix ma, Matrix44 mb);


    /**
    * @brief transfer touch device's position to robots' position, for more detail ,see tech docs
    * @details T_r0_R: initial robot end-matrix when press the button
    *          Touch_h0_v: initial touch end-matrix in touch coordinates when press the button
    *          Touch_ht_v: touch end-matrix when moving (holding button and not release after initial press)
    *          k_xyz / k_abc: ratio of mapping touch device movement to robot
    *          resM: output matrix (of robot)
    * @return always return true
    */
    bool DynamicBaseCoordTransformation(Matrix44 T_r0_R, Matrix44 Touch_h0_v,  Matrix44 Touch_ht_v, double k_xyz,double k_abc, Matrix44& resM);
    bool DynamicBaseCoordTransformation(TransMatrix T_r0_R, TransMatrix Touch_h0_v,  TransMatrix Touch_ht_v, double k_xyz,double k_abc, TransMatrix& resM);

    /**
    * @brief get increment process
    * @return boolean, whether it is success or not
    */
    bool get_increment_matrix(Matrix44 T_ck,Matrix44 T_k1, Matrix44 T_k, Matrix44 &resT);
    bool get_increment_matrix(TransMatrix T_ck, TransMatrix T_k1, TransMatrix T_k, TransMatrix &resT);

    /**
    * @brief turn matrix44 to xyzabc
    * @param [in] u type &Matrix44
    * @param [out] res_xyz the output cartesian coordinate xyz
    * @param [out] res_abc the output quaternion 
    * @return void
    */
    void rtm_r2xyzabc(Matrix44& u,Vector3& res_xyz, Vector3& res_abc);
    void rtm_r2xyzabc(TransMatrix& u, Point& res_xyz, Euler& res_abc);

    /**
    * @brief get params from yaml file for online trajectory planning
    * @return void
    */
    void online_trajectory_algorithm_params_init();
    bool load_OnlineMove_params_Config();

    /**
    * @brief functions of setting cartesian ratio (from touch to robot), this will change yaml file
    * @return 1 for fail, 0 for success
    */
    int setOnlineTrjRatio_xyz(double data_ratio);

    /**
    * @brief functions of setting quaternion ratio (from touch to robot), this will change yaml file
    * @return 1 for fail, 0 for success
    */
    int setOnlineTrjRatio_abc(double data_ratio);

    /**
    * @brief functions of asking for cartesian ratio
    * @return current ratio
    */
    double get_online_trj_ratio_xyz();

    /**
    * @brief functions of asking for quaternion ratio
    * @return current ratio
    */
    double get_online_trj_ratio_abc();



    // ------------------------------- unused or undefined functions ------------------------------------------

    /**
    * @brief test functions for Bspline
    */
    void Fir_Bspline_algorithm_test(void);
    void Fir_Bspline_algorithm_test2(void);

    

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