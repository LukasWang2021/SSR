#include "matrix44.h"
#include "basic_alg.h"
#include <unistd.h>
#include "trans_matrix.h"
#include <iostream>
#include <math.h>
#include "matrix33.h"
#include <stdio.h>
#include "onlineTrj_planner.h"
#include <iomanip>

using namespace std;

void turnM44toD(Matrix44 m1, double *m2);
void turnTMtoD(TransMatrix m1, double *m2);
bool compareD(double a, double b);


int main()
{
    OnlineTrajectoryPlanner ol_ptr;
    int32_t inv_error = true;

    // example 1
    //1 -2 -1 -2
    //0  0  0  0
    //0  9  6  9
    //0  1  2  3
    double p_matrix[16] = {1, -2, -1, -2,
                           2, -1, 4, 5,
                           4, 1, 2, 1,
                           0, 0, 0, 1};

    // exmaple 2
    // //0.5  0.866  0  -3.23
    // //0  0  1  -5
    // //0.866  -0.5  0  -1.598
    // //0  0  0  1
    // double p_matrix[16] = {0.5, 0, 0.866, 3,
    //                        0.866, 0, -0.5, 2,
    //                        0, 1, 0, 5,
    //                        0, 0, 0, 1};

    // example 3
    // double p_matrix[16] = {1, 2, 3, 4,
    //                        2, 3, 1, 2,
    //                        1, 1, 1, -1,
    //                        0, 0, 0, 1};
    double p_inv[16];



    cout<<"Given matrix is: "<<endl;

    for(int i=0;i<16;++i)
    {
        cout<<std::setprecision(2)<<p_matrix[i]<<"\t";
        if((i+1)%4 == 0 && i != 0)
        {
            cout<<endl;
        }
    }
    cout<<endl;
    
    // do inverse
    inv_error = basic_alg::inverse(p_matrix, 4, p_inv);
    if(inv_error != 0)
    {
        cout<<"Expect inv failed, return"<<endl;
        return 0;
    }

    cout<<"Expect p_matrix is: "<<endl;
    for(int i=0;i<16;++i)
    {
        cout<<std::setprecision(2)<<p_inv[i]<<"\t";
        if((i+1)%4 == 0 && i != 0)
        {
            cout<<endl;
        }
    }
    cout<<endl;


    basic_alg::TransMatrix T1;
    basic_alg::TransMatrix T1_rec;
    basic_alg::Matrix44 T2;
    basic_alg::Matrix44 T2_rec;

    // t2 data copy
    // for(int i=0;i<16;++i)
    // {
    //     T2.matrix_[i/4][i%4] = p_matrix[i];
    // }
    T2.matrix_[0][0] = p_matrix[0];
    T2.matrix_[0][1] = p_matrix[1];
    T2.matrix_[0][2] = p_matrix[2];
    T2.matrix_[0][3] = p_matrix[3];

    T2.matrix_[1][0] = p_matrix[4];
    T2.matrix_[1][1] = p_matrix[5];
    T2.matrix_[1][2] = p_matrix[6];
    T2.matrix_[1][3] = p_matrix[7];

    T2.matrix_[2][0] = p_matrix[8];
    T2.matrix_[2][1] = p_matrix[9];
    T2.matrix_[2][2] = p_matrix[10];
    T2.matrix_[2][3] = p_matrix[11];

    T2.matrix_[3][0] = p_matrix[12];
    T2.matrix_[3][1] = p_matrix[13];
    T2.matrix_[3][2] = p_matrix[14];
    T2.matrix_[3][3] = p_matrix[15];

    // t1 data copy
    T1.rotation_matrix_.matrix_[0][0] = p_matrix[0];
    T1.rotation_matrix_.matrix_[0][1] = p_matrix[1];
    T1.rotation_matrix_.matrix_[0][2] = p_matrix[2];
    T1.trans_vector_.x_ = p_matrix[3];
    T1.rotation_matrix_.matrix_[1][0] = p_matrix[4];
    T1.rotation_matrix_.matrix_[1][1] = p_matrix[5];
    T1.rotation_matrix_.matrix_[1][2] = p_matrix[6];
    T1.trans_vector_.y_ = p_matrix[7];
    T1.rotation_matrix_.matrix_[2][0] = p_matrix[8];
    T1.rotation_matrix_.matrix_[2][1] = p_matrix[9];
    T1.rotation_matrix_.matrix_[2][2] = p_matrix[10];
    T1.trans_vector_.z_ = p_matrix[11];


    // print before using inv function - SAME
    // cout<<"before proceess inv, two matrixs are: "<<endl;
    // cout<<"T1 TransMatrix is: "<<endl;
    // T1.print();
    // cout<<"T2 Matrix44 is: "<<endl;
    // T2.print();


    
    ol_ptr.online_getMatrixInv(T1, T1_rec);
    ol_ptr.online_getMatrixInv(T2, T2_rec);

    cout<<"Transmatrix INV (transmatrix form): ";
    T1_rec.print();
    cout<<endl;
    cout<<"T2 inv is: "<<endl;
    T2_rec.print();

    double pc_matrix[16], pc_matrix_[16];
    turnM44toD(T2_rec, pc_matrix_);
    turnTMtoD(T1_rec, pc_matrix);

    for(int i=0; i<12; ++i)
    {
        if(!compareD(pc_matrix[i], pc_matrix_[i]))
        {
            cout<<"the "<<i<<" th element in output result is wrong, check it"<<endl;
        }
    }


}

void turnM44toD(Matrix44 m1, double *m2)
{
    for(int i=0;i<16;++i)
    {
        m2[i] = m1.matrix_[i/4][i%4];
    }
}

void turnTMtoD(TransMatrix m1, double *m2)
{
    m2[0] = m1.rotation_matrix_.matrix_[0][0];
    m2[1] = m1.rotation_matrix_.matrix_[0][1];
    m2[2] = m1.rotation_matrix_.matrix_[0][2];
    m2[3] = m1.trans_vector_.x_;
    m2[4] = m1.rotation_matrix_.matrix_[1][0];
    m2[5] = m1.rotation_matrix_.matrix_[1][1];
    m2[6] = m1.rotation_matrix_.matrix_[1][2];
    m2[7] = m1.trans_vector_.y_;
    m2[8] = m1.rotation_matrix_.matrix_[2][0];
    m2[9] = m1.rotation_matrix_.matrix_[2][1];
    m2[10] = m1.rotation_matrix_.matrix_[2][2];
    m2[11] = m1.trans_vector_.z_;
    m2[12] = 0;
    m2[13] = 0;
    m2[14] = 0;
    m2[15] = 1;
}

bool compareD(double a, double b)
{
    if(fabs(a-b) > 0.000001)
    {
        return false;
    }
    return true;
}