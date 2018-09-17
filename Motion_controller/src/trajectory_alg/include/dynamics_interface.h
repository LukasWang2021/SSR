/*************************************************************************
	> File Name: dynamics_interface.h
	> Author: Zhengyu Shen
	> Mail:   zhengyu.shen@foresight-robotics.com
	> Created Time: 2018/04/18 
 ************************************************************************/

#ifndef DYNAMICS_INTERFACE_H
#define DYNAMICS_INTERFACE_H

#define MAXAXES 6

#include <string>
#include <vector>

using namespace std;
namespace fst_algorithm
{

typedef struct
{
    double rated_torque; //the rated torque of the motor
    double centre_of_mass[3]; // the centre of the mass of the motor
    double inertia_sensor[3][3]; //inertia sensor of the motor
    double mass_of_motor; //mass of the motor
    double jm;  //inertia of the motor 
    double gr;  //gear ratio
    double ap[MAXAXES]; //alpha
    double offset[MAXAXES];//offset
    double r[MAXAXES][3]; //centre of link
    double i[MAXAXES][3][3]; //tensor of inertia
    double m[MAXAXES]; //mass of link
    bool mdh; //true:std-dh false:mod-dh
    double b[MAXAXES]; //joint friction  coefficient
    double tc[MAXAXES][2]; //Coulomb friction coefficient
}ServelModel;

typedef struct
{
    std::string model_name; //name of the robot model
    double a[MAXAXES]; //a
    double d[MAXAXES]; //d
    double ap[MAXAXES]; //alpha
    double offset[MAXAXES];//offset
    double qlim[MAXAXES][2]; //joint limit
    double r[MAXAXES][3]; //centre of link
    double i[MAXAXES][3][3]; //tensor of inertia
    double m[MAXAXES]; //mass of link
    bool mdh; //true:std-dh false:mod-dh
    double b[MAXAXES]; //joint friction  coefficient
    double tc[MAXAXES][2]; //Coulomb friction coefficient
}RobotModel;


class DynamicsInterface
{
public:
    DynamicsInterface();
    ~DynamicsInterface();

    enum
    {
        MAX_AXES = 6,    
    };
    
   /*
    Description: set servo parameters of all axes
    input: servo_model[]
    output: none
    return: true -> all parameters all valid
            false-> one or more parameters are invalid
    */
    bool initServoModel(const ServelModel sv_model[MAX_AXES]);

   /*
    Description: set robot parameters
    input: robot_model
    output: none
    return: true -> all parameters all valid
            false-> one or more parameters are invalid
    */
    bool initRobotModel(const RobotModel& rob_model);

    /*
    Description: if the dynamics interface can work correctly.
                 judgement condition is (is_robot_model_ready && is_servo_model_ready)
    input: none
    output: none
    return: true -> the dynamics interface is ready to use
            false-> robot model or servo model are not be initialized
    */
    bool isDynamicsReady();
    /* function:getOptimalSolution 
    Description: get the properly solution about inverse-kinematics results.
    input : j_sol[6][8] -- all of the inverse-kinematics solutions.
            j_ref[6] -- reference value
            opttype -- optimal option:1 -- energy optimal 2--user define
            assigneddir -- 5 3 1 direction ( user define)
    output :ikres[6] -- the ik result.
    */
    bool getOptimalSolution(const double j_sol[8][MAX_AXES],const double j_ref[MAX_AXES],int opttype,int assigneddir,double ikres[MAX_AXES]);
    /*
    Description: get the max possible acceraltion of all axes 
                 at current joint/omega/alpha status
    input: joint: current position in joint space
           omega: current velocity in joint space
    output: alpha_max: the maximum acceraltion in joint space
    return: true -> alpha_max is avaiable
            false-> alpha_max is not avaiable, error happened in computation 
    */
    bool computeAccMax(const double joint[MAX_AXES], const double omega[MAX_AXES], 
                           double alpha_max[2][MAX_AXES]);
    /*
    Description: get the current inertia of all axes in view of motor shaft.
                 'current' means the status when we calling function computeAccMax();
    input: none
    output: inertia: the current inertia
    return: true -> inertia is avaiable
            false-> inertia is not avaiable, error happened in computation 
    */    
    bool getInertia(const double q[MAXAXES],const double ddq[MAXAXES],double inertia[MAXAXES]);
    /*
    Description: get the current counter torque of all axes in view of motor shaft.
                 'current' means the status when we calling function computeAccMax();
    input: alpha: current accerlation
    output: counter_torque: the current counter torque
    return: true -> counter_torque is avaiable
            false-> counter_torque is not avaiable, error happened in computation 
    */  
    bool getCounterTorque(const double alpha[MAX_AXES], double counter_torque[MAX_AXES]);
    
    bool getkineJacobian(const double q[MAX_AXES],double jacob[MAX_AXES][MAX_AXES]);
public:
    /* vector cross */
    bool Cross(double a[3],double b[3],double c[3]);
    /* 3*3 matrix multiply 3*1 vector*/
    bool Multiply3331(double a[3][3],double b[3],double c[3]);
    /* 3*3 matrix multiply 3*3 matrix*/
    bool Multiply3333(double a[3][3],double b[3][3],double c[3][3]);
    /* 4*4 matrix multiply 4*4 4*4 matrix*/
    bool Multiply4444(double a[4][4],double b[4][4],double c[4][4]);
    bool Multiply6666(double a[6][6],double b[6][6],double c[6][6]);
    /*get the Jacobian*/
    bool getJacobian(const double q[MAX_AXES],double jacob[MAX_AXES][6][MAX_AXES]);
    /*get forward kinematics solve*/
    bool getforwardkinematics(const double q[MAX_AXES],double t[4][4]);
    /*algebraic inverse kinematics solve*/
    bool getinversekinematics(double t[4][4],double j_ref[6],double solve[6]);
    void ReviseJoint(double J[6],double Joint_ref[6]);
    int JudgeAxis(double theta[6]);        //判断关节是否超出限位；
    /*this method to compute the link homogeneous transformation matrix*/
    bool getA(int ni,const double q[MAX_AXES],double a[4][4]);
    /*get the link rotation matrix*/
    bool getR(const double a[4][4],double r[3][3]);
    /*partial differential of the link homogeneous transformation matrix*/
    bool getU3(int idx_i,int idx_j,int idx_k,const double q[MAX_AXES],double U[4][4]);
    /*get the link Pseudo inertia matrix*/
    bool getpseudoI(int idx_i,double Pse_I[4][4]);
    /*get the dynamic equation M*/
    bool getM(int idx_i,int idx_j,const double q[MAX_AXES],double &M);
    /*get the dynamic equation C*/
    bool getC(int idx_i,int idx_j,int idx_k,const double q[MAX_AXES],double &C);
	/* newton-euler recurive algorithm */
    bool rne_tau(const double q[MAX_AXES],const double dq[MAX_AXES],const double ddq[MAX_AXES],double T[MAX_AXES]);
    bool rne(const double q[MAX_AXES],const double dq[MAX_AXES],const double ddq[MAX_AXES],double grav[3],double rv[MAX_AXES]);
    bool rne_M(const double mq[MAX_AXES][MAX_AXES],const double mdq[MAX_AXES][MAX_AXES],const double mddq[MAX_AXES][MAX_AXES],double M[MAX_AXES][MAX_AXES]);
    bool rne_C(const double cq[MAX_AXES],const double dq[MAX_AXES],double C[MAX_AXES][MAX_AXES]);
    bool rne_G(const double gq[MAX_AXES],double grv[3],double G[MAX_AXES]);
    //bool getSymbolicG(const double q[MAX_AXES]);
	//bool getSymbolicM(const double q[MAX_AXES]);
    //bool getSymbolicC(const double q[MAX_AXES]);
    /*get the dynamic equation G*/
    bool getG(int idx_i,const double q[MAX_AXES],double& G);
    /* get mcg matrix and refresh class variable*/
    bool computeMCG(const double q[MAX_AXES]);
    /*compute the torque of each joint*/
    bool getT(const double q[MAX_AXES],const double dq[MAX_AXES],const double ddq[MAX_AXES],double T[MAX_AXES]);
    /*compute the secondary torque*/
    bool getTm(const double q[MAX_AXES],const double dq[MAX_AXES],double Tm[MAX_AXES]);
    /*get the inverse matrix of M*/
    bool getInverseM(const double M[MAX_AXES][MAX_AXES],double IM[MAX_AXES][MAX_AXES]);
    /* 6*6 Matrix Multiple 6*1 vector*/
    bool getMtxMulVec(const double IM[MAX_AXES][MAX_AXES],const double V[MAX_AXES],double Res[MAX_AXES]);
    /* transpose matrix*/
    bool getTranspose(const double M[4][4],double TM[4][4]);
    bool getTranspose33(const double M[3][3],double TM[3][3]);
    /* trace */
    double TraceMatrix(const double M[4][4]);
    double MatDet(double *p, int n);                    //求矩阵的行列式  
    double Creat_M(double *p, int m, int n, int k);    //求矩阵元素A(m, n)的代数余之式  
    //void print(double *p, int n);                    //输出矩阵n*n  
    bool Gauss(const double A[MAX_AXES][MAX_AXES], double B[MAX_AXES][MAX_AXES]);    //采用部分主元的高斯消去法求方阵A的逆矩
    //get Upi trans-Upi Ip
    bool getMiddleArray(const double q[MAX_AXES]);
private:
    bool is_robot_model_ready_;
    bool is_servo_model_ready_;

    ServelModel servo_model_[MAX_AXES];
    RobotModel robot_model_;

    double inertia_[MAX_AXES];
    double counter_torque_[MAX_AXES];

    double m_[MAX_AXES][MAX_AXES];
    double c_[MAX_AXES][MAX_AXES][MAX_AXES];
    double g_[MAX_AXES];

    double q_[MAX_AXES];
    double dq_[MAX_AXES];
    double ddq_[MAX_AXES];

    //my middle-variable
    double Upi_[6][6][4][4],Ip_[6][4][4];
    double trans_Upi_[6][6][4][4];
    double TransUpiIp_[6][6][4][4];
    double q1,q2,q3,q4,q5,q6;
    double a1,a2,a3,d1,d4,d6;
    double m1,m2,m3,m4,m5,m6;
    double I11,I12,I13,I14,I15,I16,I21,I22,I23,I24,I25,I26,I31,I32,I33,I34,I35,I36;
    double I41,I42,I43,I44,I45,I46,I51,I52,I53,I54,I55,I56,I61,I62,I63,I64,I65,I66;
    double x1,x2,x3,x4,x5,x6,y1,y2,y3,y4,y5,y6,z1,z2,z3,z4,z5,z6;
    double c1,s1,c2,s2,c3,s3,c4,s4,c5,s5,c6,s6;
    double g0;

};

}

#endif 

