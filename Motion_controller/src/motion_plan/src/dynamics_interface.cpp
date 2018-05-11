/*************************************************************************
	> File Name: dynamics_interface.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018???4???9???星期???09???5???8??? ************************************************************************/

#include <iostream>
#include <math.h>
#include <motion_plan_matrix.h>
#include <motion_plan_variable.h>
#include <motion_plan_basic_function.h>
#include <dynamics_interface.h>
#include <string.h>
#include <motion_plan_reuse.h>
#include <log_manager/log_manager_logger.h>
#include <time.h>

using namespace fst_log;
using namespace std;
using namespace fst_controller;

namespace fst_algorithm
{
    bool DynamicsInterface::Cross(double a[3],double b[3],double c[3])
    {
        c[0]=a[1]*b[2] - a[2]*b[1];
        c[1]=a[2]*b[0] - a[0]*b[2];
        c[2]=a[0]*b[1] - a[1]*b[0];

        return true;
    }
    /* 3*3 matrix multiply 3*1 vector*/
    bool DynamicsInterface::Multiply3331(double a[3][3],double b[3],double c[3])
    {
         c[0]=a[0][0]*b[0] + a[0][1]*b[1] + a[0][2]*b[2];
         c[1]=a[1][0]*b[0] + a[1][1]*b[1] + a[1][2]*b[2];
         c[2]=a[2][0]*b[0] + a[2][1]*b[1] + a[2][2]*b[2];

         return true;

    }
    /*get the Jacobian*/
    bool DynamicsInterface::getJacobian(const double q[MAXAXES],double jacob[MAXAXES][6][MAXAXES])
    {
        double JOT[3][MAXAXES];
        double JPT[3][MAXAXES];
        double v[3];
        double AQ[4][4];
        double RL[3][3];
        double pl[3],tmpl[3];
        double A[4][4],AL[4][4];
        double plast[3];
        double zlast[3];
        double df[3];
        double czd[3];

        double z0[3]={0,0,1};
        double p0[3]={0,0,0};

        for(int i=0;i<MAXAXES;i++)
        {
            double I[4][4]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
            for(int j=0;j<MAXAXES;j++)
            {
                v[0]=robot_model_.r[j][0];
                v[1]=robot_model_.r[j][1];
                v[2]=robot_model_.r[j][2];

                if(j==0)
                {
                    if(!getA(1,q,AQ))
                        return false;
                    if(!getR(AQ,RL))
                        return false;
                    if(!Multiply3331(RL,v,pl))
                        return false;
                }
                else
                {
                    if(!getA(j+1,q,AQ))
                        return false;
                    Multiply4444(I,AQ,I);
                    if(!getR(I,RL))
                        return false;
                    if(!Multiply3331(RL,v,tmpl))
                        return false;
                    pl[0]=tmpl[0]+I[0][3];
                    pl[1]=tmpl[1]+I[1][3];
                    pl[2]=tmpl[2]+I[2][3];
                }
                double KI[4][4]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
                for(int k=0;k<MAXAXES;k++)
                {
//                    if(!getA(k+1,q,A))
//                       return false;
//                    Multiply4444(KI,A,KI);
                    if(k==0)
                    {
                        plast[0]=p0[0];
                        plast[1]=p0[1];
                        plast[2]=p0[2];

                        zlast[0]=z0[0];
                        zlast[1]=z0[1];
                        zlast[2]=z0[2];
                    }      
                    else
                    {
                        if(!getA(k,q,AL))
                            return false;
                        Multiply4444(KI,AL,KI);
                        plast[0]=KI[0][3];
                        plast[1]=KI[1][3];
                        plast[2]=KI[2][3];
                        zlast[0]=KI[0][2];
                        zlast[1]=KI[1][2];
                        zlast[2]=KI[2][2];
                    }            
                    df[0]=pl[0]-plast[0];
                    df[1]=pl[1]-plast[1];
                    df[2]=pl[2]-plast[2];
                    if(!Cross(zlast,df,czd))
                        return false;

                    jacob[i][0][j]=czd[0];
                    jacob[i][1][j]=czd[1];
                    jacob[i][2][j]=czd[2];
                    jacob[i][3][j]=zlast[0];
                    jacob[i][4][j]=zlast[1];
                    jacob[i][5][j]=zlast[2];
                }
            }
        }
        return true;

    }
    /*this method to compute the link homogeneous transformation matrix*/
    bool DynamicsInterface::getA(int ni,const double q[MAXAXES],double a[4][4])
    {
        double rq[MAXAXES];

        for(int i=0;i<MAXAXES;i++)
        {
            rq[i]=q[i]+robot_model_.offset[i];
        }

        if( ni<=0 || ni>MAXAXES )
        {
            return false;
        }
        if(ni==0)
        {
            memset(a, 0, sizeof(a));  //使用memset方法  
            a[0][0]=a[1][1]=a[2][2]=a[3][3]=1;
            return true;          
        }
        else
        {          
            if(robot_model_.mdh==true)
            {
                // standard dh 
//                T =    [    ct  -st*ca  st*sa   L.a*ct
//                            st  ct*ca   -ct*sa  L.a*st
//                            0   sa      ca      d
//                            0   0       0       1];
                double tmp[4][4];

                int i=ni;

                    //FST_INFO("ct%d=%f st%d=%f ca%d=%f sa%d=%f a%d=%f d%d=%f\n",\
                        i,cos(q[i-1]),i,sin(q[i-1]),i,cos(robot_model_.ap[i-1]),i,sin(robot_model_.ap[i-1]),robot_model_.a[i-1],robot_model_.d[i-1]);
                tmp[0][0]=cos(rq[i-1]);tmp[0][1]=-sin(rq[i-1])*cos(robot_model_.ap[i-1]); tmp[0][2]=sin(rq[i-1])*sin(robot_model_.ap[i-1]);tmp[0][3]=robot_model_.a[i-1]*cos(rq[i-1]);
                tmp[1][0]=sin(rq[i-1]);tmp[1][1]=cos(rq[i-1])*cos(robot_model_.ap[i-1]);  tmp[1][2]=-cos(rq[i-1])*sin(robot_model_.ap[i-1]);tmp[1][3]=robot_model_.a[i-1]*sin(rq[i-1]);
                tmp[2][0]=0;          tmp[2][1]=sin(robot_model_.ap[i-1]);              tmp[2][2]=cos(robot_model_.ap[i-1]);tmp[2][3]=robot_model_.d[i-1];    
                tmp[3][0]=0;          tmp[3][1]=0;                                      tmp[3][2]=0;tmp[3][3]=1;    
                 
                memcpy(a,tmp,sizeof(tmp));                
            }
            else
            {
                // modified dh
//                T =    [    ct      -st     0   L.a
        //                    st*ca   ct*ca   -sa -sa*d
        //                    st*sa   ct*sa   ca  ca*d
        //                    0       0       0   1];
                double tmp[4][4];

                int i=ni;

                tmp[0][0]=cos(rq[i-1]);tmp[0][1]=-sin(rq[i-1]);tmp[0][2]=0;tmp[0][3]=robot_model_.a[i-1];//*cos(robot_model_.ap[i]);tmp[0][2]=sin(q[i])*sin(robot_model_.ap[i]);tmp[0][3]=robot_model_.a[i]*cos(q[i]);
                tmp[1][0]=sin(rq[i-1])*cos(robot_model_.ap[i-1]);tmp[1][1]=cos(rq[i-1])*cos(robot_model_.ap[i-1]);tmp[1][2]=-sin(robot_model_.ap[i-1]);tmp[1][3]=-sin(robot_model_.ap[i-1])*robot_model_.d[i-1];
                tmp[2][0]=sin(rq[i-1])*sin(robot_model_.ap[i-1]);tmp[2][1]=cos(rq[i-1])*sin(robot_model_.ap[i-1]);tmp[2][2]=cos(robot_model_.ap[i-1]);tmp[2][3]=cos(robot_model_.ap[i-1])*robot_model_.d[i-1];    
                tmp[3][0]=0;tmp[3][1]=0;tmp[3][2]=0;tmp[3][3]=1;    

 
                memcpy(a,tmp,sizeof(tmp));             
            } 
                   
        }
        return true;

    }
    /*get the link rotation matrix*/
    bool DynamicsInterface::getR(const double a[4][4],double r[3][3])
    {

        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                r[i][j]=a[i][j];
            }
        }
        return true;
    }
    /*partial differential of the link homogeneous transformation matrix*/
    bool DynamicsInterface::getU3(int idx_i,int idx_j,int idx_k,const double q[MAXAXES],double U[4][4])
    {
        memset(U,0,sizeof(U));
        double A0[4][4];
        double Q[4][4];

        A0[0][0]=1;A0[0][1]=0;A0[0][2]=0;A0[0][3]=0;
        A0[1][0]=0;A0[1][1]=1;A0[1][2]=0;A0[1][3]=0;
        A0[2][0]=0;A0[2][1]=0;A0[2][2]=1;A0[2][3]=0;
        A0[3][0]=0;A0[3][1]=0;A0[3][2]=0;A0[3][3]=1;

        Q[0][0]=0;Q[0][1]=-1;Q[0][2]=0;Q[0][3]=0;
        Q[1][0]=1;Q[1][1]=0;Q[1][2]=0;Q[1][3]=0;       
        Q[2][0]=0;Q[2][1]=0;Q[2][2]=0;Q[2][3]=0; 
        Q[3][0]=0;Q[3][1]=0;Q[3][2]=0;Q[3][3]=0; 

        if((idx_j>idx_i) || (idx_k>idx_i))
        {
            //FST_INFO("---111");
            return false;
        }
        double tmp[4][4];
        double QQ[4][4];
        double Ai[4][4];
        double QQAi[4][4];
        double A0QQAi[4][4];
        double A0QAi[4][4];
        double QAi[4][4];

        for(int i=0;i<idx_i;i++)
        {
            if((idx_j==i+1) && (idx_k==i+1))
            {
                if(getA(i+1,q,Ai))
                {
                    if(!Multiply4444(Q, Q, QQ))
                        return false;
                    
                    if(!Multiply4444(QQ, Ai, QQAi))
                        return false;
                    
                    if(!Multiply4444(A0, QQAi, A0QQAi))
                        return false;
                    
                    memcpy(A0,A0QQAi,sizeof(double)*16);   
                 
                }
                else
                {//FST_INFO("---112");
                    return false;
                }
            }
            else if((idx_j==i+1) || (idx_k==i+1))
            {
                if(getA(i+1,q,Ai))
                {
                    Multiply4444(Q, Ai, QAi);

                    Multiply4444(A0,QAi,A0QAi);
                    memcpy(A0,A0QAi,sizeof(double)*16);
                 
                }
                else
                {//FST_INFO("---113");
                    return false;
                }                
            }
            else
            {
                if(getA(i+1,q,Ai))
                {
                    Multiply4444(A0, Ai, tmp);
                    memcpy(A0,tmp,sizeof(double)*16);

                }
                else
                {//FST_INFO("---114");
                    return false;
                }
            }
        }
        memcpy(U,A0,sizeof(A0));
        return true;
    }
    /*get the link Pseudo inertia matrix*/
    bool DynamicsInterface::getpseudoI(int idx_i,double Pse_I[4][4])
    {
        if((idx_i<=0) || (idx_i>MAXAXES))
        {
            return false;
        }

        Pse_I[0][0]=(-robot_model_.i[idx_i-1][0][0]+robot_model_.i[idx_i-1][1][1]+robot_model_.i[idx_i-1][2][2])/2;
        Pse_I[1][1]=(robot_model_.i[idx_i-1][0][0]-robot_model_.i[idx_i-1][1][1]+robot_model_.i[idx_i-1][2][2])/2;
        Pse_I[2][2]=(robot_model_.i[idx_i-1][0][0]+robot_model_.i[idx_i-1][1][1]-robot_model_.i[idx_i-1][2][2])/2;
        Pse_I[0][3]=robot_model_.m[idx_i-1]*robot_model_.r[idx_i-1][0];
        Pse_I[1][3]=robot_model_.m[idx_i-1]*robot_model_.r[idx_i-1][1];
        Pse_I[2][3]=robot_model_.m[idx_i-1]*robot_model_.r[idx_i-1][2];
        Pse_I[3][3]=robot_model_.m[idx_i-1];
        Pse_I[0][1]=robot_model_.i[idx_i-1][0][1];
        Pse_I[1][2]=robot_model_.i[idx_i-1][1][2];
        Pse_I[0][2]=robot_model_.i[idx_i-1][0][2];
        Pse_I[1][0]=Pse_I[0][1];
        Pse_I[2][1]=Pse_I[1][2];
        Pse_I[2][0]=Pse_I[0][2];
        Pse_I[3][0]=Pse_I[0][3];
        Pse_I[3][1]=Pse_I[1][3];
        Pse_I[3][2]=Pse_I[2][3];

        return true;

    }
    /* transpose matrix*/
    bool DynamicsInterface::getTranspose(const double M[4][4],double TM[4][4])
    {
        //[ M00, M10, M20, M30]
        //[ M01, M11, M21, M31]
        //[ M02, M12, M22, M32]
        //[ M03, M13, M23, M33]
        
        TM[0][0]=M[0][0];TM[0][1]=M[1][0];TM[0][2]=M[2][0];TM[0][3]=M[3][0];
        TM[1][0]=M[0][1];TM[1][1]=M[1][1];TM[1][2]=M[2][1];TM[1][3]=M[3][1];
        TM[2][0]=M[0][2];TM[2][1]=M[1][2];TM[2][2]=M[2][2];TM[2][3]=M[3][2];
        TM[3][0]=M[0][3];TM[3][1]=M[1][3];TM[3][2]=M[2][3];TM[3][3]=M[3][3];
        return true;
    }
    /* trace */
    double DynamicsInterface::TraceMatrix(const double M[4][4])
    {
        double tcm =0;

        tcm = M[0][0]+M[1][1]+M[2][2]+M[3][3];

        return tcm;
    }
    //get Upi trans-Upi Ip
    bool DynamicsInterface::getMiddleArray(const double q[MAX_AXES])
    {
        q1=q[0];q2=q[1];q3=q[2];q4=q[3];q5=q[4];q6=q[5];
        c1=cos(q1);s1=sin(q1);c2=cos(q2);s2=sin(q2);c3=cos(q3);s3=sin(q3);c4=cos(q4);s4=sin(q4);c5=cos(q5);s5=sin
(q5);c6=cos(q6);s6=sin(q6);
        a1=robot_model_.a[0];a2=robot_model_.a[1];a3=robot_model_.a[2];
        d1=robot_model_.d[0];d4=robot_model_.d[3];d6=robot_model_.d[5];
        m1=robot_model_.m[0];m2=robot_model_.m[1];m3=robot_model_.m[2];m4=robot_model_.m[3];m5=robot_model_.m
[4];m6=robot_model_.m[5];
        I11=robot_model_.i[0][0][0];I12=robot_model_.i[0][1][1];I13=robot_model_.i[0][2][2];I14=robot_model_.i[0]
[0][1];I15=robot_model_.i[0][1][2];I16=robot_model_.i[0][0][2];
        I21=robot_model_.i[1][0][0];I22=robot_model_.i[1][1][1];I23=robot_model_.i[1][2][2];I24=robot_model_.i[1]
[0][1];I25=robot_model_.i[1][1][2];I26=robot_model_.i[1][0][2];
        I31=robot_model_.i[2][0][0];I32=robot_model_.i[2][1][1];I33=robot_model_.i[2][2][2];I34=robot_model_.i[2]
[0][1];I35=robot_model_.i[2][1][2];I36=robot_model_.i[2][0][2];
        I41=robot_model_.i[3][0][0];I42=robot_model_.i[3][1][1];I43=robot_model_.i[3][2][2];I44=robot_model_.i[3]
[0][1];I45=robot_model_.i[3][1][2];I46=robot_model_.i[3][0][2];
        I51=robot_model_.i[4][0][0];I52=robot_model_.i[4][1][1];I53=robot_model_.i[4][2][2];I54=robot_model_.i[4]
[0][1];I55=robot_model_.i[4][1][2];I56=robot_model_.i[4][0][2];
        I61=robot_model_.i[5][0][0];I62=robot_model_.i[5][1][1];I63=robot_model_.i[5][2][2];I64=robot_model_.i[5]
[0][1];I65=robot_model_.i[5][1][2];I66=robot_model_.i[5][0][2];
        x1=robot_model_.r[0][0];y1=robot_model_.r[0][1];z1=robot_model_.r[0][2];
        x2=robot_model_.r[1][0];y2=robot_model_.r[1][1];z1=robot_model_.r[1][2];
        x3=robot_model_.r[2][0];y3=robot_model_.r[2][1];z1=robot_model_.r[2][2];
        x4=robot_model_.r[3][0];y4=robot_model_.r[3][1];z1=robot_model_.r[3][2];
        x5=robot_model_.r[4][0];y5=robot_model_.r[4][1];z1=robot_model_.r[4][2];
        x6=robot_model_.r[5][0];y6=robot_model_.r[5][1];z1=robot_model_.r[5][2];        
        /*for(int i=0;i<MAXAXES;i++)
        {
            getpseudoI(i,Ip_[i]);
            for(int j=0;j<MAXAXES;j++)
            {
                
                if(!getU3(i+1,j+1,0,q,Upi_[i][j]))
                    return false;            

                getTranspose(Upi_[i][j],trans_Upi_[i][j]);

                Multiply4444(Ip_[i], trans_Upi_[i][j], TransUpiIp_[i][j]);  
            }
            
        }*/
        //int k= idx_i>idx_j?idx_i:idx_j;
        /*for(int i=0;i<MAXAXES;i++)
        {
            getpseudoI(i+1,Ip_[i]);
            for(int j=0;j<=i;j++)
            {
                if(!getU3(i+1,j+1,0,q,Upi_[i][j]))
                    return false;    
                getTranspose(Upi_[i][j],trans_Upi_[i][j]);
                Multiply4444(Ip_[i],trans_Upi_[i][j],TransUpiIp_[i][j]);
            }
        }*/
        
    }    
    /*get the dynamic equation M*/
    bool DynamicsInterface::getM(int idx_i,int idx_j,const double q[MAXAXES],double &M)
    {
        int k= idx_i>idx_j?idx_i:idx_j;
        
        double Dij=0;
        //double Upj[4][4],UpjIp[4][4],UpjIp_[4][4];
        double tmp[4][4],tmp2[4][4];
        //double Upi[4][4];
        //double trans_Upi[4][4],TransUpiIp[4][4];
        //double Ip[4][4];

        for(int i=k;i<=MAXAXES;i++)
        {
            //if(!getU3(i,idx_j,0,q,Upj))
            //{
            //    return false;
            //} 
            //if(!getU3(i,idx_i,0,q,Upi))
            //{
            //    return false;
            //} 


            //for(int m=0;m<4;m++)
           // {
            //    for(int n=0;n<4;n++)
            //    {
            //        FST_INFO("Upj[%d][%d]=%f\n",m,n,Upj[m][n]);
                    //FST_INFO("Upi[%d][%d]=%f\n",m,n,Upi[m][n]);
            //        FST_INFO("Upj_[%d][%d]=%f\n",m,n,Upi_[i-1][idx_j-1][m][n]);
            //    }
            //}
            //getTranspose(Upi,trans_Upi);
            //for(int m=0;m<4;m++)
            //{
             //   for(int n=0;n<4;n++)
            //    {
            //        FST_INFO("trans_Upi[%d][%d]=%f\n",m,n,trans_Upi[m][n]);
                    //FST_INFO("Upi[%d][%d]=%f\n",m,n,Upi[m][n]);
            //        FST_INFO("trans_Upi_[%d][%d]=%f\n",m,n,trans_Upi_[i-1][idx_i-1][m][n]);
            //    }
            //}            
            //if(!getpseudoI(i,Ip))
            //{
            //    return false;
            //}    

            //Multiply4444(Upj, Ip, UpjIp);
           
            //Multiply4444(UpjIp, trans_Upi, tmp);

            //Multiply4444(Upi_[i-1][idx_j-1],Ip_[i-1],UpjIp_);

            //Multiply4444(UpjIp_,trans_Upi_[i-1][idx_i-1],tmp2);


            Multiply4444(Upi_[i-1][idx_j-1],TransUpiIp_[i-1][idx_i-1],tmp);
            
            Dij=Dij+TraceMatrix(tmp);

        }
        M=Dij;
        return true;
    }
    bool DynamicsInterface::getSymbolicG(const double q[MAX_AXES])
    {
        double G1,G2,G3,G4,G5,G6;

        G1=  0.0;
        G2=  g0*m3*(a2*c2+a3*c2*c3-a3*s2*s3)+g0*m4*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)+g0*m5*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)+g0*m6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)+g0*m3*x3*(c2*c3-s2*s3)-g0*m4*y4*(c2*s3+c3*s2)+g0*m3*z3*(c2*s3+c3*s2)+a2*g0*m2*c2-g0*m5*x5*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+g0*m5*z5*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+g0*m6*z6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+g0*m2*x2*c2-g0*m2*y2*s2-g0*m6*x6*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+g0*m6*y6*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))+g0*m4*x4*c4*(c2*c3-s2*s3)-g0*m5*y5*s4*(c2*c3-s2*s3)-g0*m4*z4*s4*(c2*c3-s2*s3);
        G3=  g0*m6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3)+g0*m3*(a3*c2*c3-a3*s2*s3)+g0*m4*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)+g0*m5*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)+g0*m3*x3*(c2*c3-s2*s3)-g0*m4*y4*(c2*s3+c3*s2)+g0*m3*z3*(c2*s3+c3*s2)-g0*m5*x5*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+g0*m5*z5*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+g0*m6*z6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-g0*m6*x6*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+g0*m6*y6*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))+g0*m4*x4*c4*(c2*c3-s2*s3)-g0*m5*y5*s4*(c2*c3-s2*s3)-g0*m4*z4*s4*(c2*c3-s2*s3);
        G4=  -g0*m6*x6*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))-g0*m6*y6*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))-g0*m5*y5*c4*(c2*s3+c3*s2)-g0*m4*z4*c4*(c2*s3+c3*s2)-g0*m4*x4*s4*(c2*s3+c3*s2)-g0*m5*z5*s4*s5*(c2*s3+c3*s2)-g0*m6*z6*s4*s5*(c2*s3+c3*s2)-d6*g0*m6*s4*s5*(c2*s3+c3*s2)-g0*m5*x5*c5*s4*(c2*s3+c3*s2);
        G5=  d6*g0*m6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+g0*m5*x5*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+g0*m5*z5*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+g0*m6*z6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+g0*m6*x6*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-g0*m6*y6*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2));
        G6=  -g0*m6*x6*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-g0*m6*y6*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2));

        g_[0]=G1;g_[1]=G2;g_[2]=G3;g_[3]=G4;g_[4]=G5;g_[5]=G6;

    }
    bool DynamicsInterface::getSymbolicM(const double q[MAX_AXES])
    {
        double M11,M12,M13,M14,M15,M16,M21,M22,M23,M24,M25,M26,M31,M32,M33,M34,M35,M36;
        double M41,M42,M43,M44,M45,M46,M51,M52,M53,M54,M55,M56,M61,M62,M63,M64,M65,M66;
        
        M11=  (c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I54*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I56*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I54*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I56*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I55*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I56*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I55*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I46*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I44*(c1*c2*s3+c1*c3*s2)+m4*x4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I46*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-I45*(c1*c2*s3+c1*c3*s2)+m4*z4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(c2*s1*s3+c3*s1*s2)*((c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))+I44*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+I45*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m4*y4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(s1*s2*s3-c2*c3*s1)*(I36*(c2*s1*s3+c3*s1*s2)-(s1*s2*s3-c2*c3*s1)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))-I34*c1+m3*x3*(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c2*s1*s3+c3*s1*s2)*(I36*(s1*s2*s3-c2*c3*s1)-(c2*s1*s3+c3*s1*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+I35*c1-m3*z3*(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+c1*(c1*(I11*(-1.0/2.0)+I12*(1.0/2.0)+I13*(1.0/2.0))+I16*s1+a1*m1*x1*c1)+s1*(I16*c1+s1*(I11*(1.0/2.0)+I12*(1.0/2.0)-I13*(1.0/2.0))+a1*m1*z1*c1)-c1*(-c1*(I11*(1.0/2.0)+I12*(1.0/2.0)-I13*(1.0/2.0))+I16*s1+a1*m1*z1*s1)+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*x6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*y6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+s1*(s1*(I31*(1.0/2.0)-I32*(1.0/2.0)+I33*(1.0/2.0))+I34*(c1*c2*c3-c1*s2*s3)+I35*(c1*c2*s3+c1*c3*s2)+m3*y3*(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(m5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m5*y5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+m5*x5*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+m5*z5*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+s1*(-I16*c1+s1*(I11*(-1.0/2.0)+I12*(1.0/2.0)+I13*(1.0/2.0))+a1*m1*x1*s1)-(-m6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+m6*x6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+m6*y6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+m6*z6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)*(m3*(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-m3*y3*c1-m3*x3*(s1*s2*s3-c2*c3*s1)+m3*z3*(c2*s1*s3+c3*s1*s2))+(a1*c1+a2*c1*c2)*(m2*(a1*c1+a2*c1*c2)+m2*z2*s1+m2*x2*c1*c2-m2*y2*c1*s2)-(c1*c2*s3+c1*c3*s2)*(I44*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+I45*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))+m4*y4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(-m4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+m4*x4*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+m4*z4*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m4*y4*(c2*s1*s3+c3*s1*s2))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(m4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m4*x4*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+m4*z4*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m4*y4*(c1*c2*s3+c1*c3*s2))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(-m5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+m5*y5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m5*x5*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+m5*z5*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+I66*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(m6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m6*x6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*y6*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*z6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(c1*c2*c3-c1*s2*s3)*(I34*s1+I36*(c1*c2*s3+c1*c3*s2)+(c1*c2*c3-c1*s2*s3)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(c1*c2*s3+c1*c3*s2)*(I35*s1+I36*(c1*c2*c3-c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+s1*(s1*(I21*(1.0/2.0)+I22*(1.0/2.0)-I23*(1.0/2.0))+I26*c1*c2-I25*c1*s2+m2*z2*(a1*c1+a2*c1*c2))+c1*(I34*(s1*s2*s3-c2*c3*s1)-I35*(c2*s1*s3+c3*s1*s2)+c1*(I31*(1.0/2.0)-I32*(1.0/2.0)+I33*(1.0/2.0))-m3*y3*(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(m3*(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m3*y3*s1+m3*x3*(c1*c2*c3-c1*s2*s3)+m3*z3*(c1*c2*s3+c1*c3*s2))*(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(a1*s1+a2*c2*s1)*(m2*(a1*s1+a2*c2*s1)-m2*z2*c1-m2*y2*s1*s2+m2*x2*c2*s1)+c1*(c1*(I21*(1.0/2.0)+I22*(1.0/2.0)-I23*(1.0/2.0))-I26*c2*s1+I25*s1*s2-m2*z2*(a1*s1+a2*c2*s1))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I44*(c2*s1*s3+c3*s1*s2)+I46*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-m4*x4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I45*(c2*s1*s3+c3*s1*s2)+I46*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-m4*z4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*((c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+I55*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m5*y5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+I66*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*((c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+I55*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m5*y5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+a1*c1*(a1*m1*c1+m1*x1*c1+m1*z1*s1)+c1*c2*(I26*s1+c1*c2*(I21*(-1.0/2.0)+I22*(1.0/2.0)+I23*(1.0/2.0))-I24*c1*s2+m2*x2*(a1*c1+a2*c1*c2))+a1*s1*(a1*m1*s1-m1*z1*c1+m1*x1*s1)-c1*s2*(I25*s1+I24*c1*c2-c1*s2*(I21*(1.0/2.0)-I22*(1.0/2.0)+I23*(1.0/2.0))+m2*y2*(a1*c1+a2*c1*c2))-c2*s1*(I26*c1-c2*s1*(I21*(-1.0/2.0)+I22*(1.0/2.0)+I23*(1.0/2.0))+I24*s1*s2-m2*x2*(a1*s1+a2*c2*s1))+s1*s2*(I25*c1-I24*c2*s1+s1*s2*(I21*(1.0/2.0)-I22*(1.0/2.0)+I23*(1.0/2.0))-m2*y2*(a1*s1+a2*c2*s1));
        M12=  -(m5*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-m5*x5*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+m5*z5*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*s4*(c2*s1*s3+c3*s1*s2))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+I54*s4*(c1*c2*s3+c1*c3*s2))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-I55*s4*(c1*c2*s3+c1*c3*s2))+(a1*s1+a2*c2*s1)*(a2*m2*c1*s2+m2*y2*c1*c2+m2*x2*c1*s2)-c1*(I34*(c1*c2*s3+c1*c3*s2)-I35*(c1*c2*c3-c1*s2*s3)+m3*y3*(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-(a1*c1+a2*c1*c2)*(m2*x2*s1*s2+a2*m2*s1*s2+m2*y2*c2*s1)-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*((c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-I66*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*z6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+(m3*(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m3*x3*(c1*c2*s3+c1*c3*s2)-m3*z3*(c1*c2*c3-c1*s2*s3))*(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(c2*s1*s3+c3*s1*s2)*((c1*c2*c3-c1*s2*s3)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))+I44*c4*(c1*c2*s3+c1*c3*s2)+m4*y4*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-I45*s4*(c1*c2*s3+c1*c3*s2))-(m6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+m6*z6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m6*x6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*y6*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(m6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m6*x6*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))-m6*y6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-m6*z6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(m3*(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+m3*x3*(c2*s1*s3+c3*s1*s2)+m3*z3*(s1*s2*s3-c2*c3*s1))*(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(m5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m5*x5*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-m5*z5*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-m5*y5*s4*(c1*c2*s3+c1*c3*s2))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I44*(s1*s2*s3-c2*c3*s1)-m4*x4*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-c4*(c2*s1*s3+c3*s1*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))+I46*s4*(c2*s1*s3+c3*s1*s2))+(m4*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m4*y4*(c1*c2*c3-c1*s2*s3)+m4*x4*c4*(c1*c2*s3+c1*c3*s2)-m4*z4*s4*(c1*c2*s3+c1*c3*s2))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I45*(s1*s2*s3-c2*c3*s1)-m4*z4*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-I46*c4*(c2*s1*s3+c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))-s1*(I34*(c2*s1*s3+c3*s1*s2)+I35*(s1*s2*s3-c2*c3*s1)+m3*y3*(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))-(s1*s2*s3-c2*c3*s1)*(-I36*(c1*c2*c3-c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))+(c2*s1*s3+c3*s1*s2)*(I36*(c1*c2*s3+c1*c3*s2)-(c1*c2*c3-c1*s2*s3)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-c1*(I25*c1*c2+I26*c1*s2+a2*m2*z2*c1*s2)+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-m6*x6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))-I65*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I54*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-I55*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-I54*s4*(c2*s1*s3+c3*s1*s2))-(c1*c2*s3+c1*c3*s2)*((s1*s2*s3-c2*c3*s1)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-m4*y4*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-I44*c4*(c2*s1*s3+c3*s1*s2)+I45*s4*(c2*s1*s3+c3*s1*s2))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+I55*s4*(c2*s1*s3+c3*s1*s2))-(m4*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-m4*y4*(s1*s2*s3-c2*c3*s1)+m4*x4*c4*(c2*s1*s3+c3*s1*s2)-m4*z4*s4*(c2*s1*s3+c3*s1*s2))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-I66*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-m6*z6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I54*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-I55*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+m5*y5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I44*(c1*c2*c3-c1*s2*s3)+m4*x4*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+c4*(c1*c2*s3+c1*c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I46*s4*(c1*c2*s3+c1*c3*s2))-s1*(I25*c2*s1+I26*s1*s2+a2*m2*z2*s1*s2)-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I45*(c1*c2*c3-c1*s2*s3)+I46*c4*(c1*c2*s3+c1*c3*s2)+m4*z4*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))-(c1*c2*c3-c1*s2*s3)*(I36*(s1*s2*s3-c2*c3*s1)+(c2*s1*s3+c3*s1*s2)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))-(c1*c2*s3+c1*c3*s2)*(I36*(c2*s1*s3+c3*s1*s2)+(s1*s2*s3-c2*c3*s1)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))-(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I65*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+c2*s1*(I24*c1*c2+c1*s2*(I21*(-1.0/2.0)+I22*(1.0/2.0)+I23*(1.0/2.0))+a2*m2*x2*c1*s2)-s1*s2*(c1*c2*(I21*(1.0/2.0)-I22*(1.0/2.0)+I23*(1.0/2.0))+I24*c1*s2+a2*m2*y2*c1*s2)-c1*c2*(I24*c2*s1+s1*s2*(I21*(-1.0/2.0)+I22*(1.0/2.0)+I23*(1.0/2.0))+a2*m2*x2*s1*s2)+c1*s2*(c2*s1*(I21*(1.0/2.0)-I22*(1.0/2.0)+I23*(1.0/2.0))+I24*s1*s2+a2*m2*y2*s1*s2);
        M13=  -s1*(I34*(c2*s1*s3+c3*s1*s2)+I35*(s1*s2*s3-c2*c3*s1)+m3*y3*(a3*c2*s1*s3+a3*c3*s1*s2))+(m4*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+m4*y4*(c1*c2*c3-c1*s2*s3)+m4*x4*c4*(c1*c2*s3+c1*c3*s2)-m4*z4*s4*(c1*c2*s3+c1*c3*s2))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(s1*s2*s3-c2*c3*s1)*(-I36*(c1*c2*c3-c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a3*c1*c2*s3+a3*c1*c3*s2))+(c2*s1*s3+c3*s1*s2)*(I36*(c1*c2*s3+c1*c3*s2)-(c1*c2*c3-c1*s2*s3)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a3*c1*c2*s3+a3*c1*c3*s2))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I54*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-I55*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(-I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+I65*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-I54*s4*(c2*s1*s3+c3*s1*s2))-(c1*c2*s3+c1*c3*s2)*((s1*s2*s3-c2*c3*s1)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-m4*y4*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-I44*c4*(c2*s1*s3+c3*s1*s2)+I45*s4*(c2*s1*s3+c3*s1*s2))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)+I55*s4*(c2*s1*s3+c3*s1*s2))-(m4*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-m4*y4*(s1*s2*s3-c2*c3*s1)+m4*x4*c4*(c2*s1*s3+c3*s1*s2)-m4*z4*s4*(c2*s1*s3+c3*s1*s2))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-I66*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I54*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-I55*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+m5*y5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-(c1*c2*c3-c1*s2*s3)*(I36*(s1*s2*s3-c2*c3*s1)+(c2*s1*s3+c3*s1*s2)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a3*c2*s1*s3+a3*c3*s1*s2))-(c1*c2*s3+c1*c3*s2)*(I36*(c2*s1*s3+c3*s1*s2)+(s1*s2*s3-c2*c3*s1)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a3*c2*s1*s3+a3*c3*s1*s2))-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I44*(c1*c2*c3-c1*s2*s3)+m4*x4*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+c4*(c1*c2*s3+c1*c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I46*s4*(c1*c2*s3+c1*c3*s2))-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I45*(c1*c2*c3-c1*s2*s3)+m4*z4*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+I46*c4*(c1*c2*s3+c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))-(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I65*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))-(m5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-m5*x5*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+m5*z5*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*s4*(c2*s1*s3+c3*s1*s2))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+I54*s4*(c1*c2*s3+c1*c3*s2))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-I55*s4*(c1*c2*s3+c1*c3*s2))-c1*(I34*(c1*c2*s3+c1*c3*s2)-I35*(c1*c2*c3-c1*s2*s3)+m3*y3*(a3*c1*c2*s3+a3*c1*c3*s2))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*((c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-I66*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*z6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))-(m6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)+m6*z6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m6*x6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*y6*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(m3*(a3*c1*c2*s3+a3*c1*c3*s2)+m3*x3*(c1*c2*s3+c1*c3*s2)-m3*z3*(c1*c2*c3-c1*s2*s3))*(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(c2*s1*s3+c3*s1*s2)*((c1*c2*c3-c1*s2*s3)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))+m4*y4*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+I44*c4*(c1*c2*s3+c1*c3*s2)-I45*s4*(c1*c2*s3+c1*c3*s2))-(m6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)-m6*x6*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+m6*y6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+m6*z6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(m3*(a3*c2*s1*s3+a3*c3*s1*s2)+m3*x3*(c2*s1*s3+c3*s1*s2)+m3*z3*(s1*s2*s3-c2*c3*s1))*(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I44*(s1*s2*s3-c2*c3*s1)-m4*x4*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-c4*(c2*s1*s3+c3*s1*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))+I46*s4*(c2*s1*s3+c3*s1*s2))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I45*(s1*s2*s3-c2*c3*s1)-m4*z4*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-I46*c4*(c2*s1*s3+c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+(m5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+m5*x5*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-m5*z5*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-m5*y5*s4*(c1*c2*s3+c1*c3*s2))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3);
        M14=   -(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I46*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I46*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+(I46*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-(I46*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-(m4*x4*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m4*z4*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(-m5*y5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+m5*x5*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m5*z5*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I44*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-I45*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(c2*s1*s3+c3*s1*s2)*(I44*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-I45*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*x6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*y6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I55*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(-m5*y5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+m5*x5*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+m5*z5*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-d6*m6*x6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-d6*m6*y6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+I66*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-d6*m6*z6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I55*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(m4*x4*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m4*z4*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(-I54*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(-I55*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+I56*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(-I54*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(-I55*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+I56*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+I66*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-d6*m6*z6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(m6*x6*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+m6*y6*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-d6*m6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m6*z6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(m6*x6*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+m6*y6*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-d6*m6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m6*z6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3);
        M15=   -(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*((c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+I65*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(d6*m6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+m6*z6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-m6*x6*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+m6*y6*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I54*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))-I55*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I66*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I65*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-I64*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(I54*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-I55*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*((c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+I65*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))+(m5*x5*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m5*z5*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(m5*x5*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))-m5*z5*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I66*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I65*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-I64*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(d6*m6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+m6*z6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-m6*x6*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*y6*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3);
        M16=   (s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(m6*x6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-m6*y6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-I66*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))-(m6*x6*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-m6*y6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-I66*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))));
        M21=   (c1*c2*s3+c1*c3*s2)*(I36*(c2*s1*s3+c3*s1*s2)-(s1*s2*s3-c2*c3*s1)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))-I34*c1+m3*x3*(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c1*c2*c3-c1*s2*s3)*(I36*(s1*s2*s3-c2*c3*s1)-(c2*s1*s3+c3*s1*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+I35*c1-m3*z3*(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)*(m5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m5*y5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+m5*x5*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+m5*z5*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))-(m4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m4*x4*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+m4*z4*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m4*y4*(c1*c2*s3+c1*c3*s2))*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(-m6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+m6*x6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+m6*y6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+m6*z6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)*(m3*(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m3*y3*s1+m3*x3*(c1*c2*c3-c1*s2*s3)+m3*z3*(c1*c2*s3+c1*c3*s2))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I54*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I56*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I55*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(s1*s2*s3-c2*c3*s1)*(I44*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+I45*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))+m4*y4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)*(-m4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+m4*x4*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+m4*z4*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m4*y4*(c2*s1*s3+c3*s1*s2))-(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)*(-m5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+m5*y5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m5*x5*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+m5*z5*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))+(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)*(m3*(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-m3*y3*c1-m3*x3*(s1*s2*s3-c2*c3*s1)+m3*z3*(c2*s1*s3+c3*s1*s2))-(m6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m6*x6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*y6*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*z6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(c2*s1*s3+c3*s1*s2)*(I34*s1+I36*(c1*c2*s3+c1*c3*s2)+(c1*c2*c3-c1*s2*s3)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(s1*s2*s3-c2*c3*s1)*(I35*s1+I36*(c1*c2*c3-c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+I66*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+I66*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*x6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*y6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I54*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I56*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I55*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I56*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c1*c2*c3-c1*s2*s3)*((c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))+I44*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+I45*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m4*y4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+s4*(c2*s1*s3+c3*s1*s2)*((c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+I55*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m5*y5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-c4*(c2*s1*s3+c3*s1*s2)*(I46*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I44*(c1*c2*s3+c1*c3*s2)+m4*x4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+s4*(c2*s1*s3+c3*s1*s2)*(I46*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-I45*(c1*c2*s3+c1*c3*s2)+m4*z4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-c2*s1*(I25*s1+I24*c1*c2-c1*s2*(I21*(1.0/2.0)-I22*(1.0/2.0)+I23*(1.0/2.0))+m2*y2*(a1*c1+a2*c1*c2))-s1*s2*(I26*s1+c1*c2*(I21*(-1.0/2.0)+I22*(1.0/2.0)+I23*(1.0/2.0))-I24*c1*s2+m2*x2*(a1*c1+a2*c1*c2))-c1*c2*(I25*c1-I24*c2*s1+s1*s2*(I21*(1.0/2.0)-I22*(1.0/2.0)+I23*(1.0/2.0))-m2*y2*(a1*s1+a2*c2*s1))-c1*s2*(I26*c1-c2*s1*(I21*(-1.0/2.0)+I22*(1.0/2.0)+I23*(1.0/2.0))+I24*s1*s2-m2*x2*(a1*s1+a2*c2*s1))-c4*(c1*c2*s3+c1*c3*s2)*(I44*(c2*s1*s3+c3*s1*s2)+I46*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-m4*x4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+s4*(c1*c2*s3+c1*c3*s2)*(I45*(c2*s1*s3+c3*s1*s2)+I46*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-m4*z4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+s4*(c1*c2*s3+c1*c3*s2)*((c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+I55*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m5*y5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-a2*s1*s2*(m2*(a1*c1+a2*c1*c2)+m2*z2*s1+m2*x2*c1*c2-m2*y2*c1*s2)+a2*c1*s2*(m2*(a1*s1+a2*c2*s1)-m2*z2*c1-m2*y2*s1*s2+m2*x2*c2*s1);
        M22=   (m3*(a2*c2+a3*c2*c3-a3*s2*s3)+m3*x3*(c2*c3-s2*s3)+m3*z3*(c2*s3+c3*s2))*(a2*c2+a3*c2*c3-a3*s2*s3)+(m3*(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+m3*x3*(c2*s1*s3+c3*s1*s2)+m3*z3*(s1*s2*s3-c2*c3*s1))*(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(c2*s1*s3+c3*s1*s2)*(I36*(s1*s2*s3-c2*c3*s1)+(c2*s1*s3+c3*s1*s2)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+(s1*s2*s3-c2*c3*s1)*(I36*(c2*s1*s3+c3*s1*s2)+(s1*s2*s3-c2*c3*s1)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+(c1*c2*c3-c1*s2*s3)*((c1*c2*c3-c1*s2*s3)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))+I44*c4*(c1*c2*s3+c1*c3*s2)+m4*y4*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-I45*s4*(c1*c2*s3+c1*c3*s2))-(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I56*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-I54*s4*(c2*c3-s2*s3)+m5*x5*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I56*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I55*s4*(c2*c3-s2*s3)-m5*z5*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-I54*s4*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+I55*s4*(c2*s1*s3+c3*s1*s2))+c2*(c2*(I21*(-1.0/2.0)+I22*(1.0/2.0)+I23*(1.0/2.0))-I24*s2+a2*m2*x2*c2)-s2*(I24*c2-s2*(I21*(1.0/2.0)-I22*(1.0/2.0)+I23*(1.0/2.0))+a2*m2*y2*c2)+(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)*(m4*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)-m4*y4*(c2*s3+c3*s2)+m4*x4*c4*(c2*c3-s2*s3)-m4*z4*s4*(c2*c3-s2*s3))+(m6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m6*x6*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))-m6*y6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-m6*z6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2)))*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+(c2*s3+c3*s2)*((c2*s3+c3*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-I44*c4*(c2*c3-s2*s3)+I45*s4*(c2*c3-s2*s3)-m4*y4*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3))+(m6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+m6*z6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m6*x6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*y6*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)*(m3*(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m3*x3*(c1*c2*s3+c1*c3*s2)-m3*z3*(c1*c2*c3-c1*s2*s3))+(c1*c2*s3+c1*c3*s2)*(-I36*(c1*c2*c3-c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-(c1*c2*c3-c1*s2*s3)*(I36*(c1*c2*s3+c1*c3*s2)-(c1*c2*c3-c1*s2*s3)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*((c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-I66*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*z6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)*(m5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m5*x5*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-m5*z5*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-m5*y5*s4*(c1*c2*s3+c1*c3*s2))+(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*((c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))-I66*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+m6*z6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+(m4*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-m4*y4*(s1*s2*s3-c2*c3*s1)+m4*x4*c4*(c2*s1*s3+c3*s1*s2)-m4*z4*s4*(c2*s1*s3+c3*s1*s2))*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)*(m4*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m4*y4*(c1*c2*c3-c1*s2*s3)+m4*x4*c4*(c1*c2*s3+c1*c3*s2)-m4*z4*s4*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I65*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-m6*x6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))-I65*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-I66*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-m6*z6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))+(m5*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-m5*x5*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+m5*z5*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*s4*(c2*s1*s3+c3*s1*s2))*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)*(m5*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)-m5*x5*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+m5*z5*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-m5*y5*s4*(c2*c3-s2*s3))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I66*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+I64*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I65*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-I64*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3))+(m6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)+m6*z6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-m6*x6*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+m6*y6*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3)))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)+(s1*s2*s3-c2*c3*s1)*((s1*s2*s3-c2*c3*s1)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-m4*y4*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-I44*c4*(c2*s1*s3+c3*s1*s2)+I45*s4*(c2*s1*s3+c3*s1*s2))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+I54*s4*(c1*c2*s3+c1*c3*s2))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-I55*s4*(c1*c2*s3+c1*c3*s2))+(c2*c3-s2*s3)*(I36*(c2*s3+c3*s2)+(c2*c3-s2*s3)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a2*c2+a3*c2*c3-a3*s2*s3))+(c2*s3+c3*s2)*(I36*(c2*c3-s2*s3)+(c2*s3+c3*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a2*c2+a3*c2*c3-a3*s2*s3))+s4*(c2*s1*s3+c3*s1*s2)*(I54*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-I55*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-c4*(c2*c3-s2*s3)*(I44*(c2*s3+c3*s2)-c4*(c2*c3-s2*s3)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))+I46*s4*(c2*c3-s2*s3)-m4*x4*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3))+s4*(c2*c3-s2*s3)*(I45*(c2*s3+c3*s2)-I46*c4*(c2*c3-s2*s3)+s4*(c2*c3-s2*s3)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-m4*z4*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3))+a2*c2*(a2*m2*c2+m2*x2*c2-m2*y2*s2)+c1*c2*(c1*c2*(I21*(1.0/2.0)-I22*(1.0/2.0)+I23*(1.0/2.0))+I24*c1*s2+a2*m2*y2*c1*s2)+c1*s2*(I24*c1*c2+c1*s2*(I21*(-1.0/2.0)+I22*(1.0/2.0)+I23*(1.0/2.0))+a2*m2*x2*c1*s2)-c4*(c2*s1*s3+c3*s1*s2)*(I44*(s1*s2*s3-c2*c3*s1)-m4*x4*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-c4*(c2*s1*s3+c3*s1*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))+I46*s4*(c2*s1*s3+c3*s1*s2))+s4*(c2*c3-s2*s3)*(I54*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-I55*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+s4*(c2*c3-s2*s3)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))-m5*y5*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3))+c2*s1*(c2*s1*(I21*(1.0/2.0)-I22*(1.0/2.0)+I23*(1.0/2.0))+I24*s1*s2+a2*m2*y2*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I45*(s1*s2*s3-c2*c3*s1)-m4*z4*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-I46*c4*(c2*s1*s3+c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+s1*s2*(I24*c2*s1+s1*s2*(I21*(-1.0/2.0)+I22*(1.0/2.0)+I23*(1.0/2.0))+a2*m2*x2*s1*s2)+c4*(c1*c2*s3+c1*c3*s2)*(I44*(c1*c2*c3-c1*s2*s3)+m4*x4*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+c4*(c1*c2*s3+c1*c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I46*s4*(c1*c2*s3+c1*c3*s2))-s4*(c1*c2*s3+c1*c3*s2)*(I54*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-I55*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+m5*y5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-s4*(c1*c2*s3+c1*c3*s2)*(I45*(c1*c2*c3-c1*s2*s3)+I46*c4*(c1*c2*s3+c1*c3*s2)+m4*z4*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+a2*c1*s2*(a2*m2*c1*s2+m2*y2*c1*c2+m2*x2*c1*s2)+a2*s1*s2*(m2*x2*s1*s2+a2*m2*s1*s2+m2*y2*c2*s1);
        M23=  (c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*((c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-I66*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*z6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))+(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*((c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))-I66*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+m6*z6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3))+(c1*c2*s3+c1*c3*s2)*(-I36*(c1*c2*c3-c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a3*c1*c2*s3+a3*c1*c3*s2))-(c1*c2*c3-c1*s2*s3)*(I36*(c1*c2*s3+c1*c3*s2)-(c1*c2*c3-c1*s2*s3)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a3*c1*c2*s3+a3*c1*c3*s2))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))+(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)*(m4*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-m4*y4*(s1*s2*s3-c2*c3*s1)+m4*x4*c4*(c2*s1*s3+c3*s1*s2)-m4*z4*s4*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I65*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))+(m5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+m5*x5*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-m5*z5*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-m5*y5*s4*(c1*c2*s3+c1*c3*s2))*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+(m4*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+m4*y4*(c1*c2*c3-c1*s2*s3)+m4*x4*c4*(c1*c2*s3+c1*c3*s2)-m4*z4*s4*(c1*c2*s3+c1*c3*s2))*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(-I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+I65*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-I66*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))+(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)*(m5*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)-m5*x5*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+m5*z5*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-m5*y5*s4*(c2*c3-s2*s3))+(m5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-m5*x5*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+m5*z5*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*s4*(c2*s1*s3+c3*s1*s2))*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I66*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+I64*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I65*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-I64*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3))+(m6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3)+m6*z6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-m6*x6*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+m6*y6*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3)))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)+(s1*s2*s3-c2*c3*s1)*((s1*s2*s3-c2*c3*s1)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-m4*y4*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-I44*c4*(c2*s1*s3+c3*s1*s2)+I45*s4*(c2*s1*s3+c3*s1*s2))+(c2*c3-s2*s3)*(I36*(c2*s3+c3*s2)+(c2*c3-s2*s3)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a3*c2*c3-a3*s2*s3))+(c2*s3+c3*s2)*(I36*(c2*c3-s2*s3)+(c2*s3+c3*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a3*c2*c3-a3*s2*s3))+(m3*(a3*c2*c3-a3*s2*s3)+m3*x3*(c2*c3-s2*s3)+m3*z3*(c2*s3+c3*s2))*(a2*c2+a3*c2*c3-a3*s2*s3)-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+I54*s4*(c1*c2*s3+c1*c3*s2))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-I55*s4*(c1*c2*s3+c1*c3*s2))+(m3*(a3*c2*s1*s3+a3*c3*s1*s2)+m3*x3*(c2*s1*s3+c3*s1*s2)+m3*z3*(s1*s2*s3-c2*c3*s1))*(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I56*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-I54*s4*(c2*c3-s2*s3)+m5*x5*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3))+(c2*s1*s3+c3*s1*s2)*(I36*(s1*s2*s3-c2*c3*s1)+(c2*s1*s3+c3*s1*s2)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a3*c2*s1*s3+a3*c3*s1*s2))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I56*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I55*s4*(c2*c3-s2*s3)-m5*z5*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3))+(s1*s2*s3-c2*c3*s1)*(I36*(c2*s1*s3+c3*s1*s2)+(s1*s2*s3-c2*c3*s1)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a3*c2*s1*s3+a3*c3*s1*s2))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-I54*s4*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)+I55*s4*(c2*s1*s3+c3*s1*s2))+(c1*c2*c3-c1*s2*s3)*((c1*c2*c3-c1*s2*s3)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))+m4*y4*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+I44*c4*(c1*c2*s3+c1*c3*s2)-I45*s4*(c1*c2*s3+c1*c3*s2))+(m4*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)-m4*y4*(c2*s3+c3*s2)+m4*x4*c4*(c2*c3-s2*s3)-m4*z4*s4*(c2*c3-s2*s3))*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)+(c2*s3+c3*s2)*((c2*s3+c3*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-I44*c4*(c2*c3-s2*s3)+I45*s4*(c2*c3-s2*s3)-m4*y4*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3))-(m6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)-m6*x6*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+m6*y6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+m6*z6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2)))*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+(m6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)+m6*z6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m6*x6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*y6*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(m3*(a3*c1*c2*s3+a3*c1*c3*s2)+m3*x3*(c1*c2*s3+c1*c3*s2)-m3*z3*(c1*c2*c3-c1*s2*s3))*(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+s4*(c2*c3-s2*s3)*(I54*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-I55*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+s4*(c2*c3-s2*s3)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))-m5*y5*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3))-c4*(c2*s1*s3+c3*s1*s2)*(I44*(s1*s2*s3-c2*c3*s1)-m4*x4*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-c4*(c2*s1*s3+c3*s1*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))+I46*s4*(c2*s1*s3+c3*s1*s2))+s4*(c2*s1*s3+c3*s1*s2)*(I45*(s1*s2*s3-c2*c3*s1)-m4*z4*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-I46*c4*(c2*s1*s3+c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+c4*(c1*c2*s3+c1*c3*s2)*(I44*(c1*c2*c3-c1*s2*s3)+m4*x4*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+c4*(c1*c2*s3+c1*c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I46*s4*(c1*c2*s3+c1*c3*s2))+s4*(c2*s1*s3+c3*s1*s2)*(I54*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-I55*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-s4*(c1*c2*s3+c1*c3*s2)*(I54*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-I55*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+m5*y5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-c4*(c2*c3-s2*s3)*(I44*(c2*s3+c3*s2)-c4*(c2*c3-s2*s3)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))+I46*s4*(c2*c3-s2*s3)-m4*x4*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3))-s4*(c1*c2*s3+c1*c3*s2)*(I45*(c1*c2*c3-c1*s2*s3)+m4*z4*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+I46*c4*(c1*c2*s3+c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+s4*(c2*c3-s2*s3)*(I45*(c2*s3+c3*s2)-I46*c4*(c2*c3-s2*s3)+s4*(c2*c3-s2*s3)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-m4*z4*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3));
        M24=   -(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+I66*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-d6*m6*z6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(m4*x4*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m4*z4*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(-m5*y5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+m5*x5*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m5*z5*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-d6*m6*x6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-d6*m6*y6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c1*c2*c3-c1*s2*s3)*(I44*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-I45*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I64*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))+(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*(c2*s3+c3*s2)+d6*m6*x6*s4*s5*(c2*s3+c3*s2))-(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I64*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))+(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c2*s3+c3*s2)+d6*m6*y6*s4*s5*(c2*s3+c3*s2))-(m6*x6*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))+m6*y6*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))+d6*m6*s4*s5*(c2*s3+c3*s2)+m6*z6*s4*s5*(c2*s3+c3*s2))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*x6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(m4*z4*c4*(c2*s3+c3*s2)+m4*x4*s4*(c2*s3+c3*s2))*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*y6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+I66*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-d6*m6*z6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(s1*s2*s3-c2*c3*s1)*(I44*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-I45*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(I45*c4*(c2*s3+c3*s2)+I44*s4*(c2*s3+c3*s2))*(c2*s3+c3*s2)-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(-I54*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(-I55*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+I56*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))-(-m5*y5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+m5*x5*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+m5*z5*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I55*c4*(c2*s3+c3*s2)+s4*s5*(c2*s3+c3*s2)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I56*c5*s4*(c2*s3+c3*s2))+(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I54*c4*(c2*s3+c3*s2)+c5*s4*(c2*s3+c3*s2)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*(c2*s3+c3*s2))-(m4*x4*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m4*z4*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(m5*y5*c4*(c2*s3+c3*s2)+m5*x5*c5*s4*(c2*s3+c3*s2)+m5*z5*s4*s5*(c2*s3+c3*s2))*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)-(m6*x6*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+m6*y6*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-d6*m6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m6*z6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(-I55*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+I56*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(-I54*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(m6*x6*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+m6*y6*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-d6*m6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m6*z6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I65*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))+I66*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))+s4*s5*(c2*s3+c3*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s4*s5*(c2*s3+c3*s2))+c4*(c1*c2*s3+c1*c3*s2)*(I46*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))+s4*(c1*c2*s3+c1*c3*s2)*(I46*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+s4*(c2*c3-s2*s3)*(c4*(c2*s3+c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I55*s4*s5*(c2*s3+c3*s2)+I54*c5*s4*(c2*s3+c3*s2))-c4*(c2*s1*s3+c3*s1*s2)*(I46*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))-c4*(c2*c3-s2*s3)*(I46*c4*(c2*s3+c3*s2)+s4*(c2*s3+c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))-s4*(c2*s1*s3+c3*s1*s2)*(I46*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+s4*(c2*c3-s2*s3)*(c4*(c2*s3+c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+I46*s4*(c2*s3+c3*s2))+s4*(c1*c2*s3+c1*c3*s2)*(-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I55*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-s4*(c2*s1*s3+c3*s1*s2)*(-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I55*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)));
        M25=  (m5*x5*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+m5*z5*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)))*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)+(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*((s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-I65*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+d6*m6*z6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)))+(d6*m6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+m6*z6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-m6*x6*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+m6*y6*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I66*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I65*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-I64*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*((c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+I65*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*((c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+I65*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I66*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I65*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-I64*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I66*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+d6*m6*x6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I65*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+I64*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)))+(m6*z6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+d6*m6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+m6*x6*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-m6*y6*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)-(m5*x5*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m5*z5*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(I56*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-(I56*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+(m5*x5*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))-m5*z5*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(d6*m6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+m6*z6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-m6*x6*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*y6*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(I56*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-(I56*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-s4*(I54*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+I55*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)))*(c2*c3-s2*s3)-s4*(c1*c2*s3+c1*c3*s2)*(I54*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))-I55*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+s4*(I54*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-I55*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(c2*s1*s3+c3*s1*s2);
        M26=  -(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I65*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+I66*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2)))-(m6*x6*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-m6*y6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(m6*x6*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))+m6*y6*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2)))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)+(m6*x6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-m6*y6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I64*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I64*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))+(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I65*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-I66*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-I66*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)));
        M31=  (c1*c2*s3+c1*c3*s2)*(I36*(c2*s1*s3+c3*s1*s2)-(s1*s2*s3-c2*c3*s1)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))-I34*c1+m3*x3*(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c1*c2*c3-c1*s2*s3)*(I36*(s1*s2*s3-c2*c3*s1)-(c2*s1*s3+c3*s1*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+I35*c1-m3*z3*(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)*(-m4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+m4*x4*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+m4*z4*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m4*y4*(c2*s1*s3+c3*s1*s2))-(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)*(m6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m6*x6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*y6*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*z6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))-(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)*(-m5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+m5*y5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m5*x5*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+m5*z5*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))+(a3*c1*c2*s3+a3*c1*c3*s2)*(m3*(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-m3*y3*c1-m3*x3*(s1*s2*s3-c2*c3*s1)+m3*z3*(c2*s1*s3+c3*s1*s2))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I54*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I56*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I55*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(s1*s2*s3-c2*c3*s1)*(I44*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+I45*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))+m4*y4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)*(m5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m5*y5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+m5*x5*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+m5*z5*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))-(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)*(m4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m4*x4*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+m4*z4*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m4*y4*(c1*c2*s3+c1*c3*s2))-(c2*s1*s3+c3*s1*s2)*(I34*s1+I36*(c1*c2*s3+c1*c3*s2)+(c1*c2*c3-c1*s2*s3)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(s1*s2*s3-c2*c3*s1)*(I35*s1+I36*(c1*c2*c3-c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(a3*c2*s1*s3+a3*c3*s1*s2)*(m3*(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m3*y3*s1+m3*x3*(c1*c2*c3-c1*s2*s3)+m3*z3*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+I66*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+I66*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(-m6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+m6*x6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+m6*y6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+m6*z6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*x6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*y6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I54*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I56*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I55*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I56*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c1*c2*c3-c1*s2*s3)*((c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))+I44*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+I45*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m4*y4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+s4*(c2*s1*s3+c3*s1*s2)*((c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+I55*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m5*y5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-c4*(c2*s1*s3+c3*s1*s2)*(I46*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I44*(c1*c2*s3+c1*c3*s2)+m4*x4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+s4*(c2*s1*s3+c3*s1*s2)*(I46*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-I45*(c1*c2*s3+c1*c3*s2)+m4*z4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-c4*(c1*c2*s3+c1*c3*s2)*(I44*(c2*s1*s3+c3*s1*s2)+I46*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-m4*x4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+s4*(c1*c2*s3+c1*c3*s2)*(I45*(c2*s1*s3+c3*s1*s2)+I46*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-m4*z4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+s4*(c1*c2*s3+c1*c3*s2)*((c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+I55*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m5*y5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3));
        M32=   (c2*s1*s3+c3*s1*s2)*(I36*(s1*s2*s3-c2*c3*s1)+(c2*s1*s3+c3*s1*s2)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+(s1*s2*s3-c2*c3*s1)*(I36*(c2*s1*s3+c3*s1*s2)+(s1*s2*s3-c2*c3*s1)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)*(m4*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-m4*y4*(s1*s2*s3-c2*c3*s1)+m4*x4*c4*(c2*s1*s3+c3*s1*s2)-m4*z4*s4*(c2*s1*s3+c3*s1*s2))+(c1*c2*c3-c1*s2*s3)*((c1*c2*c3-c1*s2*s3)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))+I44*c4*(c1*c2*s3+c1*c3*s2)+m4*y4*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-I45*s4*(c1*c2*s3+c1*c3*s2))+(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)*(m5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m5*x5*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-m5*z5*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-m5*y5*s4*(c1*c2*s3+c1*c3*s2))-(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I56*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-I54*s4*(c2*c3-s2*s3)+m5*x5*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I56*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I55*s4*(c2*c3-s2*s3)-m5*z5*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-I54*s4*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+I55*s4*(c2*s1*s3+c3*s1*s2))+(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)*(m4*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m4*y4*(c1*c2*c3-c1*s2*s3)+m4*x4*c4*(c1*c2*s3+c1*c3*s2)-m4*z4*s4*(c1*c2*s3+c1*c3*s2))+(c2*s3+c3*s2)*((c2*s3+c3*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-I44*c4*(c2*c3-s2*s3)+I45*s4*(c2*c3-s2*s3)-m4*y4*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3))+(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)*(m5*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)-m5*x5*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+m5*z5*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-m5*y5*s4*(c2*c3-s2*s3))+(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)*(m5*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-m5*x5*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+m5*z5*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*s4*(c2*s1*s3+c3*s1*s2))+(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3)*(m6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)+m6*z6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-m6*x6*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+m6*y6*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3)))+(a3*c2*c3-a3*s2*s3)*(m3*(a2*c2+a3*c2*c3-a3*s2*s3)+m3*x3*(c2*c3-s2*s3)+m3*z3*(c2*s3+c3*s2))+(c1*c2*s3+c1*c3*s2)*(-I36*(c1*c2*c3-c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-(c1*c2*c3-c1*s2*s3)*(I36*(c1*c2*s3+c1*c3*s2)-(c1*c2*c3-c1*s2*s3)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))+(a3*c2*s1*s3+a3*c3*s1*s2)*(m3*(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+m3*x3*(c2*s1*s3+c3*s1*s2)+m3*z3*(s1*s2*s3-c2*c3*s1))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*((c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-I66*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*z6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*((c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))-I66*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+m6*z6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I65*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)*(m4*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)-m4*y4*(c2*s3+c3*s2)+m4*x4*c4*(c2*c3-s2*s3)-m4*z4*s4*(c2*c3-s2*s3))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-m6*x6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))-I65*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-I66*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-m6*z6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-(m6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m6*x6*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))-m6*y6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-m6*z6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2)))*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)+(m6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+m6*z6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m6*x6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*y6*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I66*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+I64*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I65*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-I64*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3))+(s1*s2*s3-c2*c3*s1)*((s1*s2*s3-c2*c3*s1)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-m4*y4*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-I44*c4*(c2*s1*s3+c3*s1*s2)+I45*s4*(c2*s1*s3+c3*s1*s2))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+I54*s4*(c1*c2*s3+c1*c3*s2))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-I55*s4*(c1*c2*s3+c1*c3*s2))+(a3*c1*c2*s3+a3*c1*c3*s2)*(m3*(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m3*x3*(c1*c2*s3+c1*c3*s2)-m3*z3*(c1*c2*c3-c1*s2*s3))+(c2*c3-s2*s3)*(I36*(c2*s3+c3*s2)+(c2*c3-s2*s3)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a2*c2+a3*c2*c3-a3*s2*s3))+(c2*s3+c3*s2)*(I36*(c2*c3-s2*s3)+(c2*s3+c3*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a2*c2+a3*c2*c3-a3*s2*s3))+s4*(c2*s1*s3+c3*s1*s2)*(I54*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-I55*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-c4*(c2*c3-s2*s3)*(I44*(c2*s3+c3*s2)-c4*(c2*c3-s2*s3)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))+I46*s4*(c2*c3-s2*s3)-m4*x4*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3))+s4*(c2*c3-s2*s3)*(I45*(c2*s3+c3*s2)-I46*c4*(c2*c3-s2*s3)+s4*(c2*c3-s2*s3)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-m4*z4*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3))-c4*(c2*s1*s3+c3*s1*s2)*(I44*(s1*s2*s3-c2*c3*s1)-m4*x4*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-c4*(c2*s1*s3+c3*s1*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))+I46*s4*(c2*s1*s3+c3*s1*s2))+s4*(c2*c3-s2*s3)*(I54*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-I55*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+s4*(c2*c3-s2*s3)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))-m5*y5*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3))+s4*(c2*s1*s3+c3*s1*s2)*(I45*(s1*s2*s3-c2*c3*s1)-m4*z4*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-I46*c4*(c2*s1*s3+c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+c4*(c1*c2*s3+c1*c3*s2)*(I44*(c1*c2*c3-c1*s2*s3)+m4*x4*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+c4*(c1*c2*s3+c1*c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I46*s4*(c1*c2*s3+c1*c3*s2))-s4*(c1*c2*s3+c1*c3*s2)*(I54*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-I55*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+m5*y5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-s4*(c1*c2*s3+c1*c3*s2)*(I45*(c1*c2*c3-c1*s2*s3)+I46*c4*(c1*c2*s3+c1*c3*s2)+m4*z4*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)));
        M33=   (c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*((c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-I66*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*z6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))+(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*((c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))-I66*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+m6*z6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3))+(c1*c2*s3+c1*c3*s2)*(-I36*(c1*c2*c3-c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a3*c1*c2*s3+a3*c1*c3*s2))-(c1*c2*c3-c1*s2*s3)*(I36*(c1*c2*s3+c1*c3*s2)-(c1*c2*c3-c1*s2*s3)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a3*c1*c2*s3+a3*c1*c3*s2))+(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)*(m4*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)-m4*y4*(c2*s3+c3*s2)+m4*x4*c4*(c2*c3-s2*s3)-m4*z4*s4*(c2*c3-s2*s3))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I65*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(-I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+I65*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-I66*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))+(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)*(m6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)+m6*z6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m6*x6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*y6*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))+(m6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)-m6*x6*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+m6*y6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+m6*z6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2)))*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I66*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+I64*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I65*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-I64*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3))+(s1*s2*s3-c2*c3*s1)*((s1*s2*s3-c2*c3*s1)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-m4*y4*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-I44*c4*(c2*s1*s3+c3*s1*s2)+I45*s4*(c2*s1*s3+c3*s1*s2))+(c2*c3-s2*s3)*(I36*(c2*s3+c3*s2)+(c2*c3-s2*s3)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a3*c2*c3-a3*s2*s3))+(c2*s3+c3*s2)*(I36*(c2*c3-s2*s3)+(c2*s3+c3*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a3*c2*c3-a3*s2*s3))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+I54*s4*(c1*c2*s3+c1*c3*s2))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-I55*s4*(c1*c2*s3+c1*c3*s2))+(a3*c1*c2*s3+a3*c1*c3*s2)*(m3*(a3*c1*c2*s3+a3*c1*c3*s2)+m3*x3*(c1*c2*s3+c1*c3*s2)-m3*z3*(c1*c2*c3-c1*s2*s3))-(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I56*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-I54*s4*(c2*c3-s2*s3)+m5*x5*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3))+(c2*s1*s3+c3*s1*s2)*(I36*(s1*s2*s3-c2*c3*s1)+(c2*s1*s3+c3*s1*s2)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a3*c2*s1*s3+a3*c3*s1*s2))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I56*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I55*s4*(c2*c3-s2*s3)-m5*z5*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3))+(s1*s2*s3-c2*c3*s1)*(I36*(c2*s1*s3+c3*s1*s2)+(s1*s2*s3-c2*c3*s1)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a3*c2*s1*s3+a3*c3*s1*s2))+(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)*(m4*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-m4*y4*(s1*s2*s3-c2*c3*s1)+m4*x4*c4*(c2*s1*s3+c3*s1*s2)-m4*z4*s4*(c2*s1*s3+c3*s1*s2))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-I54*s4*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)+I55*s4*(c2*s1*s3+c3*s1*s2))+(c1*c2*c3-c1*s2*s3)*((c1*c2*c3-c1*s2*s3)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))+m4*y4*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+I44*c4*(c1*c2*s3+c1*c3*s2)-I45*s4*(c1*c2*s3+c1*c3*s2))+(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)*(m5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+m5*x5*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-m5*z5*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-m5*y5*s4*(c1*c2*s3+c1*c3*s2))+(c2*s3+c3*s2)*((c2*s3+c3*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-I44*c4*(c2*c3-s2*s3)+I45*s4*(c2*c3-s2*s3)-m4*y4*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3))+(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)*(m4*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+m4*y4*(c1*c2*c3-c1*s2*s3)+m4*x4*c4*(c1*c2*s3+c1*c3*s2)-m4*z4*s4*(c1*c2*s3+c1*c3*s2))+(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)*(m5*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)-m5*x5*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+m5*z5*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-m5*y5*s4*(c2*c3-s2*s3))+(m6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3)+m6*z6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-m6*x6*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+m6*y6*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3)))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3)+(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)*(m5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-m5*x5*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+m5*z5*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*s4*(c2*s1*s3+c3*s1*s2))+(a3*c2*c3-a3*s2*s3)*(m3*(a3*c2*c3-a3*s2*s3)+m3*x3*(c2*c3-s2*s3)+m3*z3*(c2*s3+c3*s2))+(a3*c2*s1*s3+a3*c3*s1*s2)*(m3*(a3*c2*s1*s3+a3*c3*s1*s2)+m3*x3*(c2*s1*s3+c3*s1*s2)+m3*z3*(s1*s2*s3-c2*c3*s1))+s4*(c2*c3-s2*s3)*(I54*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-I55*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+s4*(c2*c3-s2*s3)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))-m5*y5*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3))-c4*(c2*s1*s3+c3*s1*s2)*(I44*(s1*s2*s3-c2*c3*s1)-m4*x4*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-c4*(c2*s1*s3+c3*s1*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))+I46*s4*(c2*s1*s3+c3*s1*s2))+s4*(c2*s1*s3+c3*s1*s2)*(I45*(s1*s2*s3-c2*c3*s1)-m4*z4*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-I46*c4*(c2*s1*s3+c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+c4*(c1*c2*s3+c1*c3*s2)*(I44*(c1*c2*c3-c1*s2*s3)+m4*x4*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+c4*(c1*c2*s3+c1*c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I46*s4*(c1*c2*s3+c1*c3*s2))+s4*(c2*s1*s3+c3*s1*s2)*(I54*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-I55*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-s4*(c1*c2*s3+c1*c3*s2)*(I54*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-I55*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+m5*y5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-c4*(c2*c3-s2*s3)*(I44*(c2*s3+c3*s2)-c4*(c2*c3-s2*s3)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))+I46*s4*(c2*c3-s2*s3)-m4*x4*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3))-s4*(c1*c2*s3+c1*c3*s2)*(I45*(c1*c2*c3-c1*s2*s3)+m4*z4*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+I46*c4*(c1*c2*s3+c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+s4*(c2*c3-s2*s3)*(I45*(c2*s3+c3*s2)-I46*c4*(c2*c3-s2*s3)+s4*(c2*c3-s2*s3)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-m4*z4*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3));
        M34=   -(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+I66*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-d6*m6*z6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-d6*m6*x6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(-m5*y5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+m5*x5*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+m5*z5*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-d6*m6*y6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(m4*x4*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m4*z4*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-(c1*c2*c3-c1*s2*s3)*(I44*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-I45*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))-(m5*y5*c4*(c2*s3+c3*s2)+m5*x5*c5*s4*(c2*s3+c3*s2)+m5*z5*s4*s5*(c2*s3+c3*s2))*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)+(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I64*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))+(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*(c2*s3+c3*s2)+d6*m6*x6*s4*s5*(c2*s3+c3*s2))-(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I64*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))+(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c2*s3+c3*s2)+d6*m6*y6*s4*s5*(c2*s3+c3*s2))-(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)*(m6*x6*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+m6*y6*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-d6*m6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m6*z6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*x6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*y6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+I66*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-d6*m6*z6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(s1*s2*s3-c2*c3*s1)*(I44*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-I45*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))-(m6*x6*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+m6*y6*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-d6*m6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m6*z6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)+(I45*c4*(c2*s3+c3*s2)+I44*s4*(c2*s3+c3*s2))*(c2*s3+c3*s2)-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(-I54*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(-I55*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+I56*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(m4*x4*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m4*z4*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)+(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)*(-m5*y5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+m5*x5*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m5*z5*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I55*c4*(c2*s3+c3*s2)+s4*s5*(c2*s3+c3*s2)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I56*c5*s4*(c2*s3+c3*s2))+(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I54*c4*(c2*s3+c3*s2)+c5*s4*(c2*s3+c3*s2)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*(c2*s3+c3*s2))-(m6*x6*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))+m6*y6*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))+d6*m6*s4*s5*(c2*s3+c3*s2)+m6*z6*s4*s5*(c2*s3+c3*s2))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3)-(m4*z4*c4*(c2*s3+c3*s2)+m4*x4*s4*(c2*s3+c3*s2))*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(-I55*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+I56*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(-I54*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I65*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))+I66*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))+s4*s5*(c2*s3+c3*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s4*s5*(c2*s3+c3*s2))+c4*(c1*c2*s3+c1*c3*s2)*(I46*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))+s4*(c1*c2*s3+c1*c3*s2)*(I46*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+s4*(c2*c3-s2*s3)*(c4*(c2*s3+c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I55*s4*s5*(c2*s3+c3*s2)+I54*c5*s4*(c2*s3+c3*s2))-c4*(c2*s1*s3+c3*s1*s2)*(I46*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))-c4*(c2*c3-s2*s3)*(I46*c4*(c2*s3+c3*s2)+s4*(c2*s3+c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))-s4*(c2*s1*s3+c3*s1*s2)*(I46*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+s4*(c2*c3-s2*s3)*(c4*(c2*s3+c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+I46*s4*(c2*s3+c3*s2))+s4*(c1*c2*s3+c1*c3*s2)*(-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I55*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-s4*(c2*s1*s3+c3*s1*s2)*(-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I55*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)));
        M35=   (c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*((s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-I65*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+d6*m6*z6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(m5*x5*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))-m5*z5*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I66*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I65*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-I64*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*((c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+I65*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*((c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+I65*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(d6*m6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+m6*z6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-m6*x6*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*y6*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I66*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I65*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-I64*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I66*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+d6*m6*x6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I65*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+I64*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)))+(I56*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-(I56*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+(m5*x5*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+m5*z5*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)))*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)+(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)*(d6*m6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+m6*z6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-m6*x6*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+m6*y6*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))+(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3)*(m6*z6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+d6*m6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+m6*x6*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-m6*y6*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))-(I56*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-(I56*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-(m5*x5*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m5*z5*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-s4*(I54*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+I55*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)))*(c2*c3-s2*s3)-s4*(c1*c2*s3+c1*c3*s2)*(I54*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))-I55*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+s4*(I54*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-I55*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(c2*s1*s3+c3*s1*s2);
        M36=   -(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I65*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+I66*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2)))+(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I64*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I64*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))+(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I65*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-I66*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))-(m6*x6*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))+m6*y6*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2)))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3)+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))+(m6*x6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-m6*y6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)+(m6*x6*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-m6*y6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-I66*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)));
        M41=   (c1*s4+c4*(s1*s2*s3-c2*c3*s1))*((c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+I55*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m5*y5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*x6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*y6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I46*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I44*(c1*c2*s3+c1*c3*s2)+m4*x4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I46*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-I45*(c1*c2*s3+c1*c3*s2)+m4*z4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I44*(c2*s1*s3+c3*s1*s2)+I46*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-m4*x4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I45*(c2*s1*s3+c3*s1*s2)+I46*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-m4*z4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*((c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+I55*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m5*y5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I65*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+I66*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I65*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+I66*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I54*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I56*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I55*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I56*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I54*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I56*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I55*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(m6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m6*x6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*y6*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*z6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))+d6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(-m6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+m6*x6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+m6*y6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+m6*z6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)));
        M42=   -(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-m6*x6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))+(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))-I65*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I66*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+I64*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3))-(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*(I65*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-I64*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3))-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I44*(s1*s2*s3-c2*c3*s1)-m4*x4*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-c4*(c2*s1*s3+c3*s1*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))+I46*s4*(c2*s1*s3+c3*s1*s2))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I45*(s1*s2*s3-c2*c3*s1)-m4*z4*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-I46*c4*(c2*s1*s3+c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I54*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-I55*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I54*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-I55*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+m5*y5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I44*(c1*c2*c3-c1*s2*s3)+m4*x4*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+c4*(c1*c2*s3+c1*c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I46*s4*(c1*c2*s3+c1*c3*s2))+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I45*(c1*c2*c3-c1*s2*s3)+I46*c4*(c1*c2*s3+c1*c3*s2)+m4*z4*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))-(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))-(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I65*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+c4*(c2*s3+c3*s2)*(I45*(c2*s3+c3*s2)-I46*c4*(c2*c3-s2*s3)+s4*(c2*c3-s2*s3)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-m4*z4*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*((c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-I66*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*z6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+s4*(c2*s3+c3*s2)*(I44*(c2*s3+c3*s2)-c4*(c2*c3-s2*s3)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))+I46*s4*(c2*c3-s2*s3)-m4*x4*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3))+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I65*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-I66*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-m6*z6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))+c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I56*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-I54*s4*(c2*s1*s3+c3*s1*s2))+c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+I54*s4*(c1*c2*s3+c1*c3*s2))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+I55*s4*(c2*s1*s3+c3*s1*s2))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-I55*s4*(c1*c2*s3+c1*c3*s2))+c4*(c2*s3+c3*s2)*(I54*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-I55*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+s4*(c2*c3-s2*s3)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))-m5*y5*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3))-s4*s5*(c2*s3+c3*s2)*((c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))-I66*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+m6*z6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3))-d6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(m6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m6*x6*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))-m6*y6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-m6*z6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2)))-c5*s4*(c2*s3+c3*s2)*(I56*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-I54*s4*(c2*c3-s2*s3)+m5*x5*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3))+s4*s5*(c2*s3+c3*s2)*(I56*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I55*s4*(c2*c3-s2*s3)-m5*z5*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3))+d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(m6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+m6*z6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m6*x6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*y6*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))-d6*s4*s5*(c2*s3+c3*s2)*(m6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)+m6*z6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-m6*x6*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+m6*y6*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3)));
        M43=   -(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I44*(s1*s2*s3-c2*c3*s1)-m4*x4*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-c4*(c2*s1*s3+c3*s1*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))+I46*s4*(c2*s1*s3+c3*s1*s2))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I45*(s1*s2*s3-c2*c3*s1)-m4*z4*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-I46*c4*(c2*s1*s3+c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I54*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-I55*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I54*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-I55*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+m5*y5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))-(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I65*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))-(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I44*(c1*c2*c3-c1*s2*s3)+m4*x4*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+c4*(c1*c2*s3+c1*c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I46*s4*(c1*c2*s3+c1*c3*s2))+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I45*(c1*c2*c3-c1*s2*s3)+m4*z4*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+I46*c4*(c1*c2*s3+c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))-(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))-(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(-I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+I65*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))-(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I66*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+I64*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3))-(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*(I65*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-I64*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3))+c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I56*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-I54*s4*(c2*s1*s3+c3*s1*s2))+c4*(c2*s3+c3*s2)*(I54*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-I55*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+s4*(c2*c3-s2*s3)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))-m5*y5*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3))+c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+I54*s4*(c1*c2*s3+c1*c3*s2))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)+I55*s4*(c2*s1*s3+c3*s1*s2))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-I55*s4*(c1*c2*s3+c1*c3*s2))+c4*(c2*s3+c3*s2)*(I45*(c2*s3+c3*s2)-I46*c4*(c2*c3-s2*s3)+s4*(c2*c3-s2*s3)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-m4*z4*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3))+s4*(c2*s3+c3*s2)*(I44*(c2*s3+c3*s2)-c4*(c2*c3-s2*s3)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))+I46*s4*(c2*c3-s2*s3)-m4*x4*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*((c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-I66*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*z6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I65*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-I66*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))-c5*s4*(c2*s3+c3*s2)*(I56*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-I54*s4*(c2*c3-s2*s3)+m5*x5*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3))+s4*s5*(c2*s3+c3*s2)*(I56*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I55*s4*(c2*c3-s2*s3)-m5*z5*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3))+d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(m6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)+m6*z6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m6*x6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*y6*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))-s4*s5*(c2*s3+c3*s2)*((c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))-I66*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+m6*z6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3))+d6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(m6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)-m6*x6*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+m6*y6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+m6*z6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2)))-d6*s4*s5*(c2*s3+c3*s2)*(m6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3)+m6*z6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-m6*x6*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+m6*y6*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3)));
        M44=   -(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I46*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I46*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I55*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I64*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-d6*m6*x6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I64*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-d6*m6*y6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I55*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I64*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))+(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*(c2*s3+c3*s2)+d6*m6*x6*s4*s5*(c2*s3+c3*s2))+(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*(I64*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))+(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c2*s3+c3*s2)+d6*m6*y6*s4*s5*(c2*s3+c3*s2))+(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I64*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*x6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I64*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*y6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(I46*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-(I46*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I65*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+I66*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-d6*m6*z6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(-I54*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+c4*(c2*s3+c3*s2)*(c4*(c2*s3+c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I55*s4*s5*(c2*s3+c3*s2)+I54*c5*s4*(c2*s3+c3*s2))+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(-I55*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+I56*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+c4*(c2*s3+c3*s2)*(c4*(c2*s3+c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+I46*s4*(c2*s3+c3*s2))+c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(-I54*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+s4*(c2*s3+c3*s2)*(I46*c4*(c2*s3+c3*s2)+s4*(c2*s3+c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(-I55*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+I56*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I65*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+I66*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-d6*m6*z6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+c5*s4*(c2*s3+c3*s2)*(I54*c4*(c2*s3+c3*s2)+c5*s4*(c2*s3+c3*s2)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*(c2*s3+c3*s2))+s4*s5*(c2*s3+c3*s2)*(I55*c4*(c2*s3+c3*s2)+s4*s5*(c2*s3+c3*s2)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I56*c5*s4*(c2*s3+c3*s2))-d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(m6*x6*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+m6*y6*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-d6*m6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m6*z6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+s4*s5*(c2*s3+c3*s2)*(I65*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))+I66*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))+s4*s5*(c2*s3+c3*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s4*s5*(c2*s3+c3*s2))-d6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(m6*x6*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+m6*y6*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-d6*m6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m6*z6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+d6*s4*s5*(c2*s3+c3*s2)*(m6*x6*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))+m6*y6*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))+d6*m6*s4*s5*(c2*s3+c3*s2)+m6*z6*s4*s5*(c2*s3+c3*s2));
        M45=   (I54*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))-I55*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I54*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-I55*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I66*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I65*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-I64*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I66*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I65*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-I64*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I66*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+d6*m6*x6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)))-(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*(I65*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+I64*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)))-s5*(I56*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-c4*(I54*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+I55*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)))*(c2*s3+c3*s2)+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*((c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+I65*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I56*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I56*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*((c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+I65*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+c5*(I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s4*s5*(I56*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))*(c2*s3+c3*s2)+d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(d6*m6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+m6*z6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-m6*x6*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+m6*y6*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))-s4*s5*(c2*s3+c3*s2)*((s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-I65*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+d6*m6*z6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)))+d6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(d6*m6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+m6*z6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-m6*x6*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*y6*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))-c5*s4*(I56*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*(c2*s3+c3*s2)-d6*s4*s5*(c2*s3+c3*s2)*(m6*z6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+d6*m6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+m6*x6*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-m6*y6*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)));
        M46=   -(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I64*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*(I64*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))+(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))+(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I65*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-I66*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))-s5*(I65*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-I66*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+d6*s5*(m6*x6*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-m6*y6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+s4*s5*(c2*s3+c3*s2)*(I65*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+I66*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2)))+d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(m6*x6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-m6*y6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))+d6*s4*s5*(m6*x6*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))+m6*y6*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2)))*(c2*s3+c3*s2);
        M51=   (s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I54*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I56*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I55*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+I66*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+I66*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I54*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I56*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I55*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I56*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-d6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(m6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m6*x6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*y6*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*z6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))+d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(-m6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+m6*x6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+m6*y6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+m6*z6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))+c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*x6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*y6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3));
        M52=   (c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I56*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-I54*s4*(c2*c3-s2*s3)+m5*x5*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3))-(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I56*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I55*s4*(c2*c3-s2*s3)-m5*z5*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*((c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-I66*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*z6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*((c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))-I66*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+m6*z6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-I66*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-m6*z6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+I54*s4*(c1*c2*s3+c1*c3*s2))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-I55*s4*(c1*c2*s3+c1*c3*s2))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-I54*s4*(c2*s1*s3+c3*s1*s2))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+I55*s4*(c2*s1*s3+c3*s1*s2))+d6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(m6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+m6*z6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m6*x6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*y6*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))-c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+d6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(m6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)+m6*z6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-m6*x6*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+m6*y6*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3)))-d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(m6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m6*x6*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))-m6*y6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-m6*z6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2)))+c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I66*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+I64*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3))-c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-m6*x6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I65*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-I64*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3))-s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))-I65*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2));
        M53=   (s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*((c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))-I66*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+m6*z6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-I66*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+I54*s4*(c1*c2*s3+c1*c3*s2))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-I55*s4*(c1*c2*s3+c1*c3*s2))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-I54*s4*(c2*s1*s3+c3*s1*s2))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)+I55*s4*(c2*s1*s3+c3*s1*s2))+(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I56*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-I54*s4*(c2*c3-s2*s3)+m5*x5*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3))-(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I56*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I55*s4*(c2*c3-s2*s3)-m5*z5*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*((c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-I66*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*z6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))+s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(-I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+I65*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))+d6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(m6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)+m6*z6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m6*x6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*y6*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))-c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))+d6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(m6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3)+m6*z6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-m6*x6*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+m6*y6*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3)))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))+c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I66*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+I64*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3))+d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(m6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)-m6*x6*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+m6*y6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+m6*z6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2)))-s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I65*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-I64*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3))-c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2));
        M54=   -(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+I66*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-d6*m6*z6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(-I54*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(-I55*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+I56*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))-(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I55*c4*(c2*s3+c3*s2)+s4*s5*(c2*s3+c3*s2)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I56*c5*s4*(c2*s3+c3*s2))-(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I54*c4*(c2*s3+c3*s2)+c5*s4*(c2*s3+c3*s2)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*(c2*s3+c3*s2))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(-I55*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+I56*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(-I54*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+I66*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-d6*m6*z6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I65*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))+I66*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))+s4*s5*(c2*s3+c3*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s4*s5*(c2*s3+c3*s2))-d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(m6*x6*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+m6*y6*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-d6*m6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m6*z6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*x6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*y6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-d6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(m6*x6*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+m6*y6*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-d6*m6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m6*z6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-d6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(m6*x6*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))+m6*y6*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))+d6*m6*s4*s5*(c2*s3+c3*s2)+m6*z6*s4*s5*(c2*s3+c3*s2))+c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-d6*m6*x6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-d6*m6*y6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I64*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))+(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*(c2*s3+c3*s2)+d6*m6*x6*s4*s5*(c2*s3+c3*s2))+s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I64*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))+(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c2*s3+c3*s2)+d6*m6*y6*s4*s5*(c2*s3+c3*s2));
        M55=   (s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*((s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-I65*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+d6*m6*z6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*((c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+I65*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*((c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+I65*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(I56*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+(I56*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-(I56*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-(I56*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I66*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I65*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+I64*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)))-c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I66*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-I64*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I65*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-I64*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+d6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(m6*z6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+d6*m6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+m6*x6*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-m6*y6*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))+d6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(d6*m6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+m6*z6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-m6*x6*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+m6*y6*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))+d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(d6*m6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+m6*z6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-m6*x6*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*y6*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))+c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I66*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+d6*m6*x6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)));
        M56=   -(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I65*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+I66*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2)))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I65*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-I66*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-I66*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))+d6*(m6*x6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-m6*y6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(m6*x6*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-m6*y6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))+c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-d6*(m6*x6*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))+m6*y6*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2)))*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I64*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I64*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))+(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)));
        M61=   (c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*x6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*y6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3));
        M62=   (c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I65*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-m6*x6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))-I65*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I66*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+I64*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3))-(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I65*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-I64*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3));
        M63=   -(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I66*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+I64*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3))-(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I65*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-I64*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I65*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(-I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+I65*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2));
        M64=   -(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*x6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*y6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I64*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))+(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*(c2*s3+c3*s2)+d6*m6*x6*s4*s5*(c2*s3+c3*s2))+(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I64*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))+(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c2*s3+c3*s2)+d6*m6*y6*s4*s5*(c2*s3+c3*s2))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-d6*m6*x6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-d6*m6*y6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)));
        M65=   (c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I66*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I66*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I65*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-I64*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I65*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-I64*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I66*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+d6*m6*x6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)))-(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I65*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+I64*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)));
        M66=   -(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I64*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I64*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))+(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)));

        m_[0][0]=M11;m_[0][1]=M12;m_[0][2]=M13;m_[0][3]=M14;m_[0][4]=M15;m_[0][5]=M16;
        m_[1][0]=M21;m_[1][1]=M22;m_[1][2]=M23;m_[1][3]=M24;m_[1][4]=M25;m_[1][5]=M26;
        m_[2][0]=M31;m_[2][1]=M32;m_[2][2]=M33;m_[2][3]=M34;m_[2][4]=M35;m_[2][5]=M36;
        m_[3][0]=M41;m_[3][1]=M42;m_[3][2]=M43;m_[3][3]=M44;m_[3][4]=M45;m_[3][5]=M46;
        m_[4][0]=M51;m_[4][1]=M52;m_[4][2]=M53;m_[4][3]=M54;m_[4][4]=M55;m_[4][5]=M56;
        m_[5][0]=M61;m_[5][1]=M62;m_[5][2]=M63;m_[5][3]=M64;m_[5][4]=M65;m_[5][5]=M66;        
    }
    bool DynamicsInterface::getSymbolicC(const double q[MAX_AXES])
    {
        double 
C111,C122,C133,C144,C155,C166,C112,C113,C114,C115,C116,C123,C124,C125,C126,C134,C135,C136,C145,C146,C156;
        double 
C211,C222,C233,C244,C255,C266,C212,C213,C214,C215,C216,C223,C224,C225,C226,C234,C235,C236,C245,C246,C256;
        double 
C311,C322,C333,C344,C355,C366,C312,C313,C314,C315,C316,C323,C324,C325,C326,C334,C335,C336,C345,C346,C356;
        double 
C411,C422,C433,C444,C455,C466,C412,C413,C414,C415,C416,C423,C424,C425,C426,C434,C435,C436,C445,C446,C456;
        double 
C511,C522,C533,C544,C555,C566,C512,C513,C514,C515,C516,C523,C524,C525,C526,C534,C535,C536,C545,C546,C556;
        double 
C611,C622,C633,C644,C655,C666,C612,C613,C614,C615,C616,C623,C624,C625,C626,C634,C635,C636,C645,C646,C656;

        C111= 0;
        C122=   -(a1*c1+a2*c1*c2)*(-m2*y2*s1*s2+a2*m2*c2*s1+m2*x2*c2*s1)-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))*((c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+I65*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-I66*(c6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(m3*(a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3)+m3*x3*(c1*c2*c3-c1*s2*s3)+m3*z3*(c1*c2*s3+c1*c3*s2))*(a1*s1+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3)+(c2*s1*s3+c3*s1*s2)*((c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-m4*y4*(d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-I44*c4*(c1*c2*c3-c1*s2*s3)+I45*s4*(c1*c2*c3-c1*s2*s3))-(m6*
(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3)+m6*z6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-m6*x6*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+m6*y6*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-
c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1)))*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(m6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-
c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-m6*x6*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*
(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+m6*y6*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-
c6*s4*(c1*c2*c3-c1*s2*s3))+m6*z6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1
-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(m3*
(a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-m3*x3*(s1*s2*s3-c2*c3*s1)+m3*z3*(c2*s1*s3+c3*s1*s2))*
(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(m5*(d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-m5*x5*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+m5*z5*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-m5*y5*s4*
(c1*c2*c3-c1*s2*s3))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))*(I44*(c2*s1*s3+c3*s1*s2)+c4*(s1*s2*s3-c2*c3*s1)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I46*s4*
(s1*s2*s3-c2*c3*s1)-m4*x4*(d4*(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(m4*(d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-m4*y4*(c1*c2*s3+c1*c3*s2)+m4*x4*c4*(c1*c2*c3-c1*s2*s3)-
m4*z4*s4*(c1*c2*c3-c1*s2*s3))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))*(I45*(c2*s1*s3+c3*s1*s2)+I46*c4*(s1*s2*s3-c2*c3*s1)-s4*(s1*s2*s3-c2*c3*s1)*(I41*(1.0/2.0)+I42*
(1.0/2.0)-I43*(1.0/2.0))-m4*z4*(d4*(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-s1*(-I34*(s1*s2*s3-
c2*c3*s1)+I35*(c2*s1*s3+c3*s1*s2)+m3*y3*(a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(s1*s2*s3-c2*c3*s1)*(I36*
(c1*c2*s3+c1*c3*s2)+(c1*c2*c3-c1*s2*s3)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3))+(c2*s1*s3+c3*s1*s2)*(I36*(c1*c2*c3-c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*
(1.0/2.0))+m3*z3*(a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-c1*(I26*c1*c2-I25*c1*s2+a2*m2*z2*c1*c2)-(s6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(s6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+I66*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*
(c1*c2*c3-c1*s2*s3))-(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(-I64*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*
(c1*c2*c3-c1*s2*s3))+I65*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*
(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(c4*s1-
s4*(c1*c2*c3-c1*s2*s3))*(-I54*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+I55*(c5*(c2*s1*s3+c3*s1*s2)-
c4*s5*(s1*s2*s3-c2*c3*s1))+s4*(s1*s2*s3-c2*c3*s1)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+m5*y5*(d4*
(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*
(I56*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I54*s4*(s1*s2*s3-c2*c3*s1)+m5*x5*(d4*
(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c1*c2*s3+c1*c3*s2)*((c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)-
I42*(1.0/2.0)+I43*(1.0/2.0))+I44*c4*(s1*s2*s3-c2*c3*s1)-I45*s4*(s1*s2*s3-c2*c3*s1)-m4*y4*(d4*
(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*
(-I56*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I55*s4*(s1*s2*s3-c2*c3*s1)+m5*z5*(d4*
(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(m4*(d4*(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3)-m4*y4*(c2*s1*s3+c3*s1*s2)-m4*x4*c4*(s1*s2*s3-c2*c3*s1)+m4*z4*s4*(s1*s2*s3-c2*c3*s1))*(a1*c1+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*
(I65*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))-I66*(c6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-
c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-
c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I54*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-I55*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-m5*y5*(d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+s4*(c1*c2*c3-c1*s2*s3)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*
(1.0/2.0)))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I44*(c1*c2*s3+c1*c3*s2)-m4*x4*(d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-c4*(c1*c2*c3-c1*s2*s3)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*
(1.0/2.0))+I46*s4*(c1*c2*c3-c1*s2*s3))-s1*(I26*c2*s1-I25*s1*s2+a2*m2*z2*c2*s1)+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*
(I45*(c1*c2*s3+c1*c3*s2)-m4*z4*(d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-I46*c4*(c1*c2*c3-
c1*s2*s3)+s4*(c1*c2*c3-c1*s2*s3)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))-(c1*c2*c3-c1*s2*s3)*(I36*
(c2*s1*s3+c3*s1*s2)-(s1*s2*s3-c2*c3*s1)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3))-(c1*c2*s3+c1*c3*s2)*(-I36*(s1*s2*s3-c2*c3*s1)+(c2*s1*s3+c3*s1*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*
(1.0/2.0))+m3*z3*(a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3
-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I66*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+I64*(s6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*
(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c6*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I65*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-I64*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*
(s1*s2*s3-c2*c3*s1))+(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-
c2*c3*s1))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(m5*(d4*(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-m5*x5*
(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+m5*z5*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-
c2*c3*s1))+m5*y5*s4*(s1*s2*s3-c2*c3*s1))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-
(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-I54*s4*(c1*c2*c3-c1*s2*s3))+(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+I55*s4*(c1*c2*c3-c1*s2*s3))+(a1*s1+a2*c2*s1)*
(a2*m2*c1*c2+m2*x2*c1*c2-m2*y2*c1*s2)-c1*(I34*(c1*c2*c3-c1*s2*s3)+I35*(c1*c2*s3+c1*c3*s2)+m3*y3*
(a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+c2*s1*(c1*c2*(I21*(-1.0/2.0)+I22*(1.0/2.0)+I23*(1.0/2.0))-
I24*c1*s2+a2*m2*x2*c1*c2)-s1*s2*(I24*c1*c2-c1*s2*(I21*(1.0/2.0)-I22*(1.0/2.0)+I23*(1.0/2.0))+a2*m2*y2*c1*c2)-
c1*c2*(c2*s1*(I21*(-1.0/2.0)+I22*(1.0/2.0)+I23*(1.0/2.0))-I24*s1*s2+a2*m2*x2*c2*s1)+c1*s2*(I24*c2*s1-s1*s2*(I21*
(1.0/2.0)-I22*(1.0/2.0)+I23*(1.0/2.0))+a2*m2*y2*c2*s1);;
        C133=   -s1*(-I34*(s1*s2*s3-c2*c3*s1)+I35*(c2*s1*s3+c3*s1*s2)+m3*y3*(a3*c2*c3*s1-a3*s1*s2*s3))+(m4*(d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-m4*y4*(c1*c2*s3+c1*c3*s2)+m4*x4*c4*(c1*c2*c3-c1*s2*s3)-m4*z4*s4*
(c1*c2*c3-c1*s2*s3))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(s1*s2*s3-c2*c3*s1)*(I36*
(c1*c2*s3+c1*c3*s2)+(c1*c2*c3-c1*s2*s3)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a3*c1*c2*c3-
a3*c1*s2*s3))+(c2*s1*s3+c3*s1*s2)*(I36*(c1*c2*c3-c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*
(1.0/2.0))+m3*z3*(a3*c1*c2*c3-a3*c1*s2*s3))-(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(-I54*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))+I55*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+m5*y5*(d4*
(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+s4*(s1*s2*s3-c2*c3*s1)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-
(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(s6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+I66*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*
(c1*c2*c3-c1*s2*s3))-(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(-I64*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-
c1*s2*s3))+I65*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-
c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))-(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*
(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I54*s4*(s1*s2*s3-c2*c3*s1))-(c1*c2*s3+c1*c3*s2)*((c2*s1*s3+c3*s1*s2)*
(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-m4*y4*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I44*c4*
(s1*s2*s3-c2*c3*s1)-I45*s4*(s1*s2*s3-c2*c3*s1))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(-I56*
(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I55*s4*(s1*s2*s3-
c2*c3*s1))-(m4*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)-m4*y4*(c2*s1*s3+c3*s1*s2)-m4*x4*c4*(s1*s2*s3-
c2*c3*s1)+m4*z4*s4*(s1*s2*s3-c2*c3*s1))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-
c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))-I66*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-
c1*s2*s3))+(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*
(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))+(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))*(I54*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-I55*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*
(c1*c2*c3-c1*s2*s3))-m5*y5*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)+s4*(c1*c2*c3-c1*s2*s3)*(I51*(1.0/2.0)-
I52*(1.0/2.0)+I53*(1.0/2.0)))-(c1*c2*c3-c1*s2*s3)*(I36*(c2*s1*s3+c3*s1*s2)-(s1*s2*s3-c2*c3*s1)*(I31*(-
1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a3*c2*c3*s1-a3*s1*s2*s3))-(c1*c2*s3+c1*c3*s2)*(-I36*(s1*s2*s3-
c2*c3*s1)+(c2*s1*s3+c3*s1*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a3*c2*c3*s1-
a3*s1*s2*s3))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I44*(c1*c2*s3+c1*c3*s2)-m4*x4*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-
a3*c1*s2*s3)-c4*(c1*c2*c3-c1*s2*s3)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))+I46*s4*(c1*c2*c3-
c1*s2*s3))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I45*(c1*c2*s3+c1*c3*s2)-m4*z4*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-
a3*c1*s2*s3)-I46*c4*(c1*c2*c3-c1*s2*s3)+s4*(c1*c2*c3-c1*s2*s3)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))-(s6*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I66*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+I64*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*
(s1*s2*s3-c2*c3*s1))-(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-
c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3))-(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I65*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-I64*(c6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*
(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3))-(m5*(d4*
(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)-m5*x5*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+m5*z5*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+m5*y5*s4*(s1*s2*s3-c2*c3*s1))*(a1*c1+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*
(I56*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-I54*s4*(c1*c2*c3-
c1*s2*s3))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3
-c1*s2*s3))-(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*
(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)+I55*s4*(c1*c2*c3-c1*s2*s3))-c1*(I34*(c1*c2*c3-c1*s2*s3)+I35*
(c1*c2*s3+c1*c3*s2)+m3*y3*(a3*c1*c2*c3-a3*c1*s2*s3))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*
((c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-I66*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3))-(m6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3)+m6*z6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-m6*x6*(c6*
(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+m6*y6*(s6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1)))*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(m3*(a3*c1*c2*c3-
a3*c1*s2*s3)+m3*x3*(c1*c2*c3-c1*s2*s3)+m3*z3*(c1*c2*s3+c1*c3*s2))*(a1*s1+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3)+(c2*s1*s3+c3*s1*s2)*((c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-m4*y4*(d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-I44*c4*(c1*c2*c3-c1*s2*s3)+I45*s4*(c1*c2*c3-c1*s2*s3))+(m6*(d6*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-m6*x6*(c6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+m6*y6*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*
(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+m6*z6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3)))*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3)-(m3*(a3*c2*c3*s1-a3*s1*s2*s3)-m3*x3*(s1*s2*s3-c2*c3*s1)+m3*z3*(c2*s1*s3+c3*s1*s2))*
(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I44*(c2*s1*s3+c3*s1*s2)-m4*x4*(d4*
(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+c4*(s1*s2*s3-c2*c3*s1)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-
I46*s4*(s1*s2*s3-c2*c3*s1))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I45*(c2*s1*s3+c3*s1*s2)-m4*z4*(d4*
(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I46*c4*(s1*s2*s3-c2*c3*s1)-s4*(s1*s2*s3-c2*c3*s1)*(I41*(1.0/2.0)+I42*
(1.0/2.0)-I43*(1.0/2.0)))+(m5*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-m5*x5*(s5*(c1*c2*s3+c1*c3*s2)-
c4*c5*(c1*c2*c3-c1*s2*s3))+m5*z5*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-m5*y5*s4*(c1*c2*c3-c1*s2*s3))*
(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3);;
        C144=   -(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I46*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))
*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I46*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+(I46*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+(I46*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*
(1.0/2.0)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+(m4*x4*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+m4*z4*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1)))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(m5*y5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+m5*x5*c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+m5*z5*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))*(a1*c1+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(c1*c2*s3+c1*c3*s2)*(I44*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+I45*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c2*s1*s3+c3*s1*s2)*(I44*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+I45*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*
(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-c5*s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+(s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+c5*c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+d6*m6*x6*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c5*c6*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-c5*s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0))+I65*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+d6*m6*y6*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))-(c1*c4-
s4*(s1*s2*s3-c2*c3*s1))*((c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+I55*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+(m5*y5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+m5*x5*c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+m5*z5*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-c5*s6*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c5*c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))+I66*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+d6*m6*x6*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(c6*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(s6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))+c5*c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-c5*s6*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+d6*m6*y6*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*
(I65*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-c5*s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+I66*(s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+c5*c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-
I63*(1.0/2.0))+d6*m6*z6*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*((c1*c4-s4*(s1*s2*s3-
c2*c3*s1))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+I55*s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1)))+(m4*x4*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+m4*z4*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))*(I54*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))*(I55*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I56*c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))*(I54*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))*(I55*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I56*c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))*(I65*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-c5*s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+I66*(s6*(c1*c4-
s4*(s1*s2*s3-c2*c3*s1))+c5*c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I61*
(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(m6*x6*(s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+c5*c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+m6*y6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-c5*s6*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1)))+d6*m6*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+m6*z6*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))*(a1*c1+d6*
(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3)+(m6*x6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c5*c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+m6*y6*(c6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))-c5*s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+d6*m6*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+m6*z6*s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3);;
        C155=   (s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*((s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))-I65*s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))+(d6*m6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+m6*z6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+m6*x6*c6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-m6*y6*s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*
(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3)-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I54*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+I55*
(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I66*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+c6*
(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*
(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2)))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I65*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+I64*c6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))-(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0)))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0)))+(I54*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+I55*(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))*((s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+I66*c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-I65*s6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))+(s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0)))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0)))+(m5*x5*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+m5*z5*(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(m5*x5*(c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+m5*z5*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I66*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*(c2*s1*s3+c3*s1*s2))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I65*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+I64*c6*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2)))+(d6*m6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*z6*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*x6*c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-
m6*y6*s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3);;
        C166=   (s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*
(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*(c1*c4-
s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))+(m6*x6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*
(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+m6*y6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+I66*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))+(m6*x6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*y6*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-
s5*(c1*c2*s3+c1*c3*s2)))+I66*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))));;
        C112=   (s1*s2*s3-c2*c3*s1)*(I36*(s1*s2*s3-c2*c3*s1)+(c2*s1*s3+c3*s1*s2)*(I31*(-1.0/2.0)+I32*
(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))-(c2*s1*s3+c3*s1*s2)*(I36*
(c2*s1*s3+c3*s1*s2)+(s1*s2*s3-c2*c3*s1)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*
(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))-(m5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m5*x5*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-m5*z5*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-m5*y5*s4*
(c1*c2*s3+c1*c3*s2))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*
((c1*c2*c3-c1*s2*s3)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))+I44*c4*(c1*c2*s3+c1*c3*s2)+m4*y4*(-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-I45*s4*(c1*c2*s3+c1*c3*s2))-s1*(I34*(c1*c2*s3+c1*c3*s2)-I35*(c1*c2*c3-
c1*s2*s3)+m3*y3*(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-(m4*(-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m4*y4*(c1*c2*c3-c1*s2*s3)+m4*x4*c4*(c1*c2*s3+c1*c3*s2)-m4*z4*s4*
(c1*c2*s3+c1*c3*s2))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(m4*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-m4*y4*(s1*s2*s3-c2*c3*s1)+m4*x4*c4*(c2*s1*s3+c3*s1*s2)-m4*z4*s4*
(c2*s1*s3+c3*s1*s2))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*
(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))-(a1*s1+a2*c2*s1)*
(m2*x2*s1*s2+a2*m2*s1*s2+m2*y2*c2*s1)+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I65*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-
c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*
(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*((c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*
(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-
I66*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*z6*(d4*(s1*s2*s3-
c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))-(m5*(d4*
(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-m5*x5*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+m5*z5*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*s4*(c2*s1*s3+c3*s1*s2))*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*
(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-m6*x6*(-d6*(c5*
(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-(c6*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(c6*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))-I65*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))+c1*(I34*(c2*s1*s3+c3*s1*s2)+I35*(s1*s2*s3-c2*c3*s1)+m3*y3*
(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))-(c1*c2*c3-c1*s2*s3)*(-I36*(c1*c2*c3-c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I31*(-
1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-(c1*c2*s3+c1*c3*s2)*(I36*
(c1*c2*s3+c1*c3*s2)-(c1*c2*c3-c1*s2*s3)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*
(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I44*(s1*s2*s3-c2*c3*s1)-m4*x4*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-c4*(c2*s1*s3+c3*s1*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*
(1.0/2.0))+I46*s4*(c2*s1*s3+c3*s1*s2))-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I45*(s1*s2*s3-c2*c3*s1)-m4*z4*(d4*(s1*s2*s3
-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-I46*c4*(c2*s1*s3+c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)+I42*
(1.0/2.0)-I43*(1.0/2.0)))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-I66*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*
(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-m6*z6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-(m3*(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+m3*x3*
(c2*s1*s3+c3*s1*s2)+m3*z3*(s1*s2*s3-c2*c3*s1))*(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(m6*(-d6*(c5*(c1*c2*c3-
c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m6*x6*(c6*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))-m6*y6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-m6*z6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2)))*
(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3)-s1*(I25*c1*c2+I26*c1*s2+a2*m2*z2*c1*s2)-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I54*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))-I55*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-
(m6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+m6*z6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-
m6*x6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*y6*(s6*(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*
(I54*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-I55*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+m5*y5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I51*
(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I44*(c1*c2*c3-c1*s2*s3)+m4*x4*(-d4*
(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+c4*(c1*c2*s3+c1*c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*
(1.0/2.0))-I46*s4*(c1*c2*s3+c1*c3*s2))+c1*(I25*c2*s1+I26*s1*s2+a2*m2*z2*s1*s2)-(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*
(I45*(c1*c2*c3-c1*s2*s3)+I46*c4*(c1*c2*s3+c1*c3*s2)+m4*z4*(-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*
(1.0/2.0)))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0))-m5*x5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+I54*s4*(c1*c2*s3+c1*c3*s2))-(s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-
(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(-d4*
(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-I55*s4*(c1*c2*s3+c1*c3*s2))-(c2*s1*s3+c3*s1*s2)*((s1*s2*s3-
c2*c3*s1)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-m4*y4*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-I44*c4*(c2*s1*s3+c3*s1*s2)+I45*s4*(c2*s1*s3+c3*s1*s2))-
(a1*c1+a2*c1*c2)*(a2*m2*c1*s2+m2*y2*c1*c2+m2*x2*c1*s2)-(m3*(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m3*x3*
(c1*c2*s3+c1*c3*s2)-m3*z3*(c1*c2*c3-c1*s2*s3))*(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-I54*s4*(c2*s1*s3+c3*s1*s2))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+I55*s4*(c2*s1*s3+c3*s1*s2))-c1*c2*(I24*c1*c2+c1*s2*(I21*(-
1.0/2.0)+I22*(1.0/2.0)+I23*(1.0/2.0))+a2*m2*x2*c1*s2)+c1*s2*(c1*c2*(I21*(1.0/2.0)-I22*(1.0/2.0)+I23*
(1.0/2.0))+I24*c1*s2+a2*m2*y2*c1*s2)-c2*s1*(I24*c2*s1+s1*s2*(I21*(-1.0/2.0)+I22*(1.0/2.0)+I23*
(1.0/2.0))+a2*m2*x2*s1*s2)+s1*s2*(c2*s1*(I21*(1.0/2.0)-I22*(1.0/2.0)+I23*(1.0/2.0))+I24*s1*s2+a2*m2*y2*s1*s2);;
        C113=   -(c1*c2*c3-c1*s2*s3)*(-I36*(c1*c2*c3-c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I31*(-1.0/2.0)+I32*
(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a3*c1*c2*s3+a3*c1*c3*s2))-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I44*(s1*s2*s3-c2*c3*s1)-
m4*x4*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-c4*(c2*s1*s3+c3*s1*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*
(1.0/2.0))+I46*s4*(c2*s1*s3+c3*s1*s2))-(c1*c2*s3+c1*c3*s2)*(I36*(c1*c2*s3+c1*c3*s2)-(c1*c2*c3-c1*s2*s3)*(I31*
(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a3*c1*c2*s3+a3*c1*c3*s2))-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I45*
(s1*s2*s3-c2*c3*s1)-m4*z4*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-I46*c4*(c2*s1*s3+c3*s1*s2)+s4*
(c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))-(m3*(a3*c2*s1*s3+a3*c3*s1*s2)+m3*x3*
(c2*s1*s3+c3*s1*s2)+m3*z3*(s1*s2*s3-c2*c3*s1))*(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*
(c1*c2*s3+c1*c3*s2))-I66*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(c5*
(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(d6*(c5*(c1*c2*c3
-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))+(m6*(d6*(c5*(c1*c2*c3-
c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)-m6*x6*(c6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+m6*y6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+m6*z6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2)))*
(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3)-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I54*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-I55*(c5*
(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)+s4*
(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-(m6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)+m6*z6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))-m6*x6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))+m6*y6*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3)-(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I54*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-I55*(c5*
(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+m5*y5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-s4*
(c1*c2*s3+c1*c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I44*(c1*c2*c3-
c1*s2*s3)+m4*x4*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+c4*(c1*c2*s3+c1*c3*s2)*(I41*(-1.0/2.0)+I42*
(1.0/2.0)+I43*(1.0/2.0))-I46*s4*(c1*c2*s3+c1*c3*s2))-(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I45*(c1*c2*c3-
c1*s2*s3)+m4*z4*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+I46*c4*(c1*c2*s3+c1*c3*s2)-s4*
(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(-d4*(c1*c2*c3-
c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+I54*s4*(c1*c2*s3+c1*c3*s2))-(c2*s1*s3+c3*s1*s2)*((s1*s2*s3-c2*c3*s1)*(I41*
(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-m4*y4*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-I44*c4*
(c2*s1*s3+c3*s1*s2)+I45*s4*(c2*s1*s3+c3*s1*s2))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I56*
(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-I55*s4*
(c1*c2*s3+c1*c3*s2))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-I54*s4*(c2*s1*s3+c3*s1*s2))-(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-
(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(s1*s2*s3
-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)+I55*s4*(c2*s1*s3+c3*s1*s2))-(m3*(a3*c1*c2*s3+a3*c1*c3*s2)+m3*x3*
(c1*c2*s3+c1*c3*s2)-m3*z3*(c1*c2*c3-c1*s2*s3))*(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(s1*s2*s3-c2*c3*s1)*(I36*
(s1*s2*s3-c2*c3*s1)+(c2*s1*s3+c3*s1*s2)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*
(a3*c2*s1*s3+a3*c3*s1*s2))-(c2*s1*s3+c3*s1*s2)*(I36*(c2*s1*s3+c3*s1*s2)+(s1*s2*s3-c2*c3*s1)*(I31*(1.0/2.0)+I32*
(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a3*c2*s1*s3+a3*c3*s1*s2))-(m5*(-d4*(c1*c2*c3-
c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+m5*x5*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-m5*z5*(c5*(c1*c2*c3-
c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-m5*y5*s4*(c1*c2*s3+c1*c3*s2))*(a1*c1+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*((c1*c2*c3-c1*s2*s3)*(I41*(1.0/2.0)-I42*
(1.0/2.0)+I43*(1.0/2.0))+m4*y4*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+I44*c4*(c1*c2*s3+c1*c3*s2)-
I45*s4*(c1*c2*s3+c1*c3*s2))-s1*(I34*(c1*c2*s3+c1*c3*s2)-I35*(c1*c2*c3-c1*s2*s3)+m3*y3*(a3*c1*c2*s3+a3*c1*c3*s2))-
(m4*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-m4*y4*(s1*s2*s3-c2*c3*s1)+m4*x4*c4*(c2*s1*s3+c3*s1*s2)-
m4*z4*s4*(c2*s1*s3+c3*s1*s2))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(m4*(-d4*(c1*c2*c3-
c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+m4*y4*(c1*c2*c3-c1*s2*s3)+m4*x4*c4*(c1*c2*s3+c1*c3*s2)-m4*z4*s4*
(c1*c2*s3+c1*c3*s2))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*
(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I65*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I64*(c6*(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(s1*s2*s3-
c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))+(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*((c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*
(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-
I66*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*z6*(d4*(s1*s2*s3-
c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))-(m5*(d4*(s1*s2*s3-
c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-m5*x5*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+m5*z5*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*s4*(c2*s1*s3+c3*s1*s2))*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*
(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*
(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))+(c6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(-I64*(c6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+I65*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-
c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))+c1*(I34*(c2*s1*s3+c3*s1*s2)+I35*(s1*s2*s3-c2*c3*s1)+m3*y3*
(a3*c2*s1*s3+a3*c3*s1*s2));;
        C114=   (-m5*y5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+m5*x5*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+m5*z5*s5*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(c6*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*x6*s5*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3)))-(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I64*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c6*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*s5*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*y6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(m4*x4*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-
m4*z4*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))*(I46*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I41*(-1.0/2.0)+I42*
(1.0/2.0)+I43*(1.0/2.0)))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I46*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))-(c1*c2*s3+c1*c3*s2)*(I44*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-
I45*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+(m6*x6*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1)))+m6*y6*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-d6*m6*s5*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))-m6*z6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))+I66*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-s5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-d6*m6*z6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))*(-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3))+I55*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(m6*x6*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+m6*y6*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-
d6*m6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m6*z6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3
-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(-I55*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+I56*c5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(-I54*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c2*s1*s3+c3*s1*s2)*(I44*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))-I45*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(-(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I55*s5*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1)))-(m4*x4*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m4*z4*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(-m5*y5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+m5*x5*c5*(c1*c4
-s4*(s1*s2*s3-c2*c3*s1))+m5*z5*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1)))+(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-d6*m6*x6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c6*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(s6*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-
d6*m6*y6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(-I54*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0))+I56*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(-
I55*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+I56*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+I66*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-
d6*m6*z6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(I46*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*
(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+(I46*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3));;
        C115=   (m5*x5*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m5*z5*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(s6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I66*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*
(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I66*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-c6*
(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*
(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I65*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-I64*c6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(c6*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I65*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-I64*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+s6*
(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*
(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(I54*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))-I55*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0)))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0)))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*((c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+I65*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*((c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+I65*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(d6*m6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+m6*z6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-
m6*x6*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*y6*s6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(m5*x5*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))-m5*z5*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(a1*c1+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I54*(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-I55*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(d6*m6*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+m6*z6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))-m6*x6*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+m6*y6*s6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(I56*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0)))*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-(I56*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0)))*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2));;
        C116=   -(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))
*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(c6*(c1*c4
-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))*(I65*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))-I66*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))))-(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))-(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))*(c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(m6*x6*(c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-m6*y6*(s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-I66*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-
s5*(c1*c2*s3+c1*c3*s2))))-(m6*x6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))-m6*y6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3);;
        C123=   -s1*(-I34*(s1*s2*s3-c2*c3*s1)+I35*(c2*s1*s3+c3*s1*s2)+m3*y3*(a3*c2*c3*s1-a3*s1*s2*s3))+(m4*(d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-m4*y4*(c1*c2*s3+c1*c3*s2)+m4*x4*c4*(c1*c2*c3-c1*s2*s3)-m4*z4*s4*
(c1*c2*c3-c1*s2*s3))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(s1*s2*s3-c2*c3*s1)*(I36*
(c1*c2*s3+c1*c3*s2)+(c1*c2*c3-c1*s2*s3)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a3*c1*c2*c3-
a3*c1*s2*s3))+(c2*s1*s3+c3*s1*s2)*(I36*(c1*c2*c3-c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*
(1.0/2.0))+m3*z3*(a3*c1*c2*c3-a3*c1*s2*s3))-(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(-I54*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))+I55*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+m5*y5*(d4*
(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+s4*(s1*s2*s3-c2*c3*s1)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-
(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(s6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+I66*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*
(c1*c2*c3-c1*s2*s3))-(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(-I64*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-
c1*s2*s3))+I65*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-
c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))-(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*
(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I54*s4*(s1*s2*s3-c2*c3*s1))-(c1*c2*s3+c1*c3*s2)*((c2*s1*s3+c3*s1*s2)*
(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-m4*y4*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I44*c4*
(s1*s2*s3-c2*c3*s1)-I45*s4*(s1*s2*s3-c2*c3*s1))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(-I56*
(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I55*s4*(s1*s2*s3-
c2*c3*s1))-(m4*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)-m4*y4*(c2*s1*s3+c3*s1*s2)-m4*x4*c4*(s1*s2*s3-
c2*c3*s1)+m4*z4*s4*(s1*s2*s3-c2*c3*s1))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-
c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))-I66*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-
c1*s2*s3))+(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*
(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))+(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))*(I54*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-I55*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*
(c1*c2*c3-c1*s2*s3))-m5*y5*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)+s4*(c1*c2*c3-c1*s2*s3)*(I51*(1.0/2.0)-
I52*(1.0/2.0)+I53*(1.0/2.0)))-(c1*c2*c3-c1*s2*s3)*(I36*(c2*s1*s3+c3*s1*s2)-(s1*s2*s3-c2*c3*s1)*(I31*(-
1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a3*c2*c3*s1-a3*s1*s2*s3))-(c1*c2*s3+c1*c3*s2)*(-I36*(s1*s2*s3-
c2*c3*s1)+(c2*s1*s3+c3*s1*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a3*c2*c3*s1-
a3*s1*s2*s3))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I44*(c1*c2*s3+c1*c3*s2)-m4*x4*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-
a3*c1*s2*s3)-c4*(c1*c2*c3-c1*s2*s3)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))+I46*s4*(c1*c2*c3-
c1*s2*s3))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I45*(c1*c2*s3+c1*c3*s2)-m4*z4*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-
a3*c1*s2*s3)-I46*c4*(c1*c2*c3-c1*s2*s3)+s4*(c1*c2*c3-c1*s2*s3)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))-(s6*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I66*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+I64*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*
(s1*s2*s3-c2*c3*s1))-(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-
c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3))-(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I65*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-I64*(c6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*
(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3))-(m5*(d4*
(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)-m5*x5*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+m5*z5*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+m5*y5*s4*(s1*s2*s3-c2*c3*s1))*(a1*c1+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*
(I56*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-I54*s4*(c1*c2*c3-
c1*s2*s3))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3
-c1*s2*s3))-(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*
(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)+I55*s4*(c1*c2*c3-c1*s2*s3))-c1*(I34*(c1*c2*c3-c1*s2*s3)+I35*
(c1*c2*s3+c1*c3*s2)+m3*y3*(a3*c1*c2*c3-a3*c1*s2*s3))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*
((c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-I66*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3))-(m6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3)+m6*z6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-m6*x6*(c6*
(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+m6*y6*(s6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1)))*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(m3*(a3*c1*c2*c3-
a3*c1*s2*s3)+m3*x3*(c1*c2*c3-c1*s2*s3)+m3*z3*(c1*c2*s3+c1*c3*s2))*(a1*s1+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3)+(c2*s1*s3+c3*s1*s2)*((c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-m4*y4*(d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-I44*c4*(c1*c2*c3-c1*s2*s3)+I45*s4*(c1*c2*c3-c1*s2*s3))+(m6*(d6*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-m6*x6*(c6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+m6*y6*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*
(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+m6*z6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3)))*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3)-(m3*(a3*c2*c3*s1-a3*s1*s2*s3)-m3*x3*(s1*s2*s3-c2*c3*s1)+m3*z3*(c2*s1*s3+c3*s1*s2))*
(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I44*(c2*s1*s3+c3*s1*s2)-m4*x4*(d4*
(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+c4*(s1*s2*s3-c2*c3*s1)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-
I46*s4*(s1*s2*s3-c2*c3*s1))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I45*(c2*s1*s3+c3*s1*s2)-m4*z4*(d4*
(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I46*c4*(s1*s2*s3-c2*c3*s1)-s4*(s1*s2*s3-c2*c3*s1)*(I41*(1.0/2.0)+I42*
(1.0/2.0)-I43*(1.0/2.0)))+(m5*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-m5*x5*(s5*(c1*c2*s3+c1*c3*s2)-
c4*c5*(c1*c2*c3-c1*s2*s3))+m5*z5*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-m5*y5*s4*(c1*c2*c3-c1*s2*s3))*
(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3);;
        C124=   (c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(c4*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*
(1.0/2.0))+I54*c5*s4*(c2*s1*s3+c3*s1*s2)+I55*s4*s5*(c2*s1*s3+c3*s1*s2))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))*(I55*c4*(c2*s1*s3+c3*s1*s2)+I56*c5*s4*(c2*s1*s3+c3*s1*s2)+s4*s5*(c2*s1*s3+c3*s1*s2)*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I54*c4*
(c2*s1*s3+c3*s1*s2)+c5*s4*(c2*s1*s3+c3*s1*s2)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*
(c2*s1*s3+c3*s1*s2))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I64*(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))+(c4*s6*
(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*
(c1*c2*s3+c1*c3*s2)+d6*m6*x6*s4*s5*(c1*c2*s3+c1*c3*s2))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*
(c1*c2*s3+c1*c3*s2))+(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+I65*s4*s5*(c1*c2*s3+c1*c3*s2)+d6*m6*y6*s4*s5*(c1*c2*s3+c1*c3*s2))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*
(I46*c4*(c1*c2*s3+c1*c3*s2)+s4*(c1*c2*s3+c1*c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))+(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))*(c4*(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+I46*s4*
(c1*c2*s3+c1*c3*s2))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(c4*c6*(c1*c2*s3+c1*c3*s2)-
c5*s4*s6*(c1*c2*s3+c1*c3*s2))+I66*(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))+s4*s5*
(c1*c2*s3+c1*c3*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s4*s5*(c1*c2*s3+c1*c3*s2))+(I46*c4*
(c2*s1*s3+c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+(c4*(c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+I46*s4*(c2*s1*s3+c3*s1*s2))*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3))+(c2*s1*s3+c3*s1*s2)*(I45*c4*(c1*c2*s3+c1*c3*s2)+I44*s4*(c1*c2*s3+c1*c3*s2))+(m4*z4*c4*
(c2*s1*s3+c3*s1*s2)+m4*x4*s4*(c2*s1*s3+c3*s1*s2))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3)+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*
(I64*(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))+(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*
(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*(c2*s1*s3+c3*s1*s2)+d6*m6*x6*s4*s5*
(c2*s1*s3+c3*s1*s2))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I64*(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+(c4*c6*(c2*s1*s3+c3*s1*s2)-
c5*s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*
(c2*s1*s3+c3*s1*s2)+d6*m6*y6*s4*s5*(c2*s1*s3+c3*s1*s2))-(I45*c4*(c2*s1*s3+c3*s1*s2)+I44*s4*(c2*s1*s3+c3*s1*s2))*
(c1*c2*s3+c1*c3*s2)-(m6*x6*(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))+m6*y6*(c4*c6*
(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))+d6*m6*s4*s5*(c1*c2*s3+c1*c3*s2)+m6*z6*s4*s5*(c1*c2*s3+c1*c3*s2))
*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3)-(m4*z4*c4*(c1*c2*s3+c1*c3*s2)+m4*x4*s4*(c1*c2*s3+c1*c3*s2))*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(c4*(c1*c2*s3+c1*c3*s2)
*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*s4*(c1*c2*s3+c1*c3*s2)+I55*s4*s5*(c1*c2*s3+c1*c3*s2))-
(m5*y5*c4*(c1*c2*s3+c1*c3*s2)+m5*x5*c5*s4*(c1*c2*s3+c1*c3*s2)+m5*z5*s4*s5*(c1*c2*s3+c1*c3*s2))*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(m5*y5*c4*(c2*s1*s3+c3*s1*s2)+m5*x5*c5*s4*
(c2*s1*s3+c3*s1*s2)+m5*z5*s4*s5*(c2*s1*s3+c3*s1*s2))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3)+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I65*(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*
(c2*s1*s3+c3*s1*s2))+I66*(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+s4*s5*(c2*s1*s3+c3*s1*s2)*(I61*
(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s4*s5*(c2*s1*s3+c3*s1*s2))+(m6*x6*(c4*s6*
(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+m6*y6*(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*
(c2*s1*s3+c3*s1*s2))+d6*m6*s4*s5*(c2*s1*s3+c3*s1*s2)+m6*z6*s4*s5*(c2*s1*s3+c3*s1*s2))*(a1*c1+d6*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I54*c4*(c1*c2*s3+c1*c3*s2)+c5*s4*(c1*c2*s3+c1*c3*s2)*(I51*
(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*(c1*c2*s3+c1*c3*s2))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))*(I55*c4*(c1*c2*s3+c1*c3*s2)+I56*c5*s4*(c1*c2*s3+c1*c3*s2)+s4*s5*(c1*c2*s3+c1*c3*s2)*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)));;
        C125=   (d6*m6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+m6*z6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+m6*x6*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-m6*y6*s6*(c5*(c1*c2*c3-c1*s2*s3)-
c4*s5*(c1*c2*s3+c1*c3*s2)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I54*(c5*(c1*c2*c3-
c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+I55*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))+(s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I66*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))+c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I65*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+I64*c6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))-(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-
c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))-(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*((s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*
(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I65*s6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))+(d6*m6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+m6*z6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+m6*x6*c6*
(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m6*y6*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2)))*
(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3)-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*
(I66*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*
(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*
(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I65*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+I64*c6*(c5*(c1*c2*c3-
c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))+(I54*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I55*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*((s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))-I65*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))+(m5*x5*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+m5*z5*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3)+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0)))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0)))+(m5*x5*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+m5*z5*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2)))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3);;
        C126=   (s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*
(I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+(c6*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(s6*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))+(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*
(c1*c2*s3+c1*c3*s2))+I66*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2)))-
(m6*x6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+m6*y6*(c6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-
c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*
(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(m6*x6*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))+m6*y6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2)))*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))+I66*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)));;
        C134=   (c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(c4*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*
(1.0/2.0))+I54*c5*s4*(c2*s1*s3+c3*s1*s2)+I55*s4*s5*(c2*s1*s3+c3*s1*s2))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))*(I55*c4*(c2*s1*s3+c3*s1*s2)+I56*c5*s4*(c2*s1*s3+c3*s1*s2)+s4*s5*(c2*s1*s3+c3*s1*s2)*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I54*c4*
(c2*s1*s3+c3*s1*s2)+c5*s4*(c2*s1*s3+c3*s1*s2)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*
(c2*s1*s3+c3*s1*s2))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I64*(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))+(c4*s6*
(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*
(c1*c2*s3+c1*c3*s2)+d6*m6*x6*s4*s5*(c1*c2*s3+c1*c3*s2))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*
(c1*c2*s3+c1*c3*s2))+(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+I65*s4*s5*(c1*c2*s3+c1*c3*s2)+d6*m6*y6*s4*s5*(c1*c2*s3+c1*c3*s2))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*
(I46*c4*(c1*c2*s3+c1*c3*s2)+s4*(c1*c2*s3+c1*c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))+(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))*(c4*(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+I46*s4*
(c1*c2*s3+c1*c3*s2))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(c4*c6*(c1*c2*s3+c1*c3*s2)-
c5*s4*s6*(c1*c2*s3+c1*c3*s2))+I66*(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))+s4*s5*
(c1*c2*s3+c1*c3*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s4*s5*(c1*c2*s3+c1*c3*s2))+(I46*c4*
(c2*s1*s3+c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+(c4*(c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+I46*s4*(c2*s1*s3+c3*s1*s2))*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3))+(c2*s1*s3+c3*s1*s2)*(I45*c4*(c1*c2*s3+c1*c3*s2)+I44*s4*(c1*c2*s3+c1*c3*s2))+(m4*z4*c4*
(c2*s1*s3+c3*s1*s2)+m4*x4*s4*(c2*s1*s3+c3*s1*s2))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3)+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*
(I64*(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))+(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*
(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*(c2*s1*s3+c3*s1*s2)+d6*m6*x6*s4*s5*
(c2*s1*s3+c3*s1*s2))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I64*(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+(c4*c6*(c2*s1*s3+c3*s1*s2)-
c5*s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*
(c2*s1*s3+c3*s1*s2)+d6*m6*y6*s4*s5*(c2*s1*s3+c3*s1*s2))-(I45*c4*(c2*s1*s3+c3*s1*s2)+I44*s4*(c2*s1*s3+c3*s1*s2))*
(c1*c2*s3+c1*c3*s2)-(m6*x6*(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))+m6*y6*(c4*c6*
(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))+d6*m6*s4*s5*(c1*c2*s3+c1*c3*s2)+m6*z6*s4*s5*(c1*c2*s3+c1*c3*s2))
*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3)-(m4*z4*c4*(c1*c2*s3+c1*c3*s2)+m4*x4*s4*(c1*c2*s3+c1*c3*s2))*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(c4*(c1*c2*s3+c1*c3*s2)
*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*s4*(c1*c2*s3+c1*c3*s2)+I55*s4*s5*(c1*c2*s3+c1*c3*s2))-
(m5*y5*c4*(c1*c2*s3+c1*c3*s2)+m5*x5*c5*s4*(c1*c2*s3+c1*c3*s2)+m5*z5*s4*s5*(c1*c2*s3+c1*c3*s2))*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(m5*y5*c4*(c2*s1*s3+c3*s1*s2)+m5*x5*c5*s4*
(c2*s1*s3+c3*s1*s2)+m5*z5*s4*s5*(c2*s1*s3+c3*s1*s2))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3)+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I65*(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*
(c2*s1*s3+c3*s1*s2))+I66*(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+s4*s5*(c2*s1*s3+c3*s1*s2)*(I61*
(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s4*s5*(c2*s1*s3+c3*s1*s2))+(m6*x6*(c4*s6*
(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+m6*y6*(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*
(c2*s1*s3+c3*s1*s2))+d6*m6*s4*s5*(c2*s1*s3+c3*s1*s2)+m6*z6*s4*s5*(c2*s1*s3+c3*s1*s2))*(a1*c1+d6*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I54*c4*(c1*c2*s3+c1*c3*s2)+c5*s4*(c1*c2*s3+c1*c3*s2)*(I51*
(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*(c1*c2*s3+c1*c3*s2))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))*(I55*c4*(c1*c2*s3+c1*c3*s2)+I56*c5*s4*(c1*c2*s3+c1*c3*s2)+s4*s5*(c1*c2*s3+c1*c3*s2)*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)));;
        C135=   (d6*m6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+m6*z6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+m6*x6*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-m6*y6*s6*(c5*(c1*c2*c3-c1*s2*s3)-
c4*s5*(c1*c2*s3+c1*c3*s2)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I54*(c5*(c1*c2*c3-
c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+I55*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))+(s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I66*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))+c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I65*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+I64*c6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))-(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-
c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))-(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*((s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*
(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I65*s6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))+(d6*m6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+m6*z6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+m6*x6*c6*
(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m6*y6*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2)))*
(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3)-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*
(I66*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*
(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*
(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I65*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+I64*c6*(c5*(c1*c2*c3-
c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))+(I54*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I55*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*((s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))-I65*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))+(m5*x5*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+m5*z5*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3)+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0)))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0)))+(m5*x5*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+m5*z5*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2)))*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3);;
        C136=   (s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*
(I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+(c6*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(s6*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))+(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*
(c1*c2*s3+c1*c3*s2))+I66*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2)))-
(m6*x6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+m6*y6*(c6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-(I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-
c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*
(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(m6*x6*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))+m6*y6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2)))*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))+I66*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)));;
        C145=   -(m5*z5*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m5*x5*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(a1*c1+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(m6*z6*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+d6*m6*c5*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+m6*y6*s5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m6*x6*c6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3)-(m5*z5*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m5*x5*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I66*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-c6*s5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+d6*m6*x6*c5*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I66*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-c6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+d6*m6*x6*c5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*
(I65*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-I64*c6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+s5*s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(c6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I65*c5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))-I64*c6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+s5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))*(c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*s5*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I65*s5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+d6*m6*z6*c5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I61*
(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I65*s5*s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+d6*m6*z6*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I55*c5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-I54*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(d6*m6*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m6*z6*c5*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))-m6*x6*c6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m6*y6*s5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*
(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3)-(I55*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-I54*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-I56*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-
s5*(c1*c2*s3+c1*c3*s2))*(I56*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I56*c5*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))+(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*
(1.0/2.0)-I53*(1.0/2.0))-I56*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)));;
        C146=   (s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*
(I64*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(c6*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(c6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(c6*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(I64*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1)))-(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-
s5*(c1*c2*s3+c1*c3*s2)))+(I64*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(s6*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))*
(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))-I66*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))))+(m6*x6*(c6*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-m6*y6*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-
s4*(s1*s2*s3-c2*c3*s1))))*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*
(I65*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-I66*(c6*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))))+(m6*x6*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))-m6*y6*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))))*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3);;
        C156=   (s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*c6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+I66*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))-(s6*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*c6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-
s5*(c1*c2*s3+c1*c3*s2)))*(c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))-(m6*y6*c6*(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+m6*x6*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2)))*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*
(I65*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+I66*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2)))-(m6*y6*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*x6*s6*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)));;
        C211=   -(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*
(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+I66*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))
*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(m6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m6*x6*(s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*y6*(c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*z6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I54*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+I56*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(a1*c1+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(s1*s2*s3-c2*c3*s1)*((c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)-
I42*(1.0/2.0)+I43*(1.0/2.0))+I44*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+I45*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m4*y4*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*
(I55*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(a1*c1+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)*(m3*
(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m3*y3*s1+m3*x3*(c1*c2*c3-c1*s2*s3)+m3*z3*(c1*c2*s3+c1*c3*s2))+(c6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-
s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3
-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))-m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c2*s1*s3+c3*s1*s2)*(I36*(c2*s1*s3+c3*s1*s2)-(s1*s2*s3-
c2*c3*s1)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))-I34*c1+m3*x3*(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-
(s1*s2*s3-c2*c3*s1)*(I36*(s1*s2*s3-c2*c3*s1)-(c2*s1*s3+c3*s1*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*
(1.0/2.0))+I35*c1-m3*z3*(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))*(I54*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I56*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0))-m5*x5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(I55*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I56*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0))-m5*z5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c1*c2*c3-c1*s2*s3)*(I44*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+I45*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)-I42*
(1.0/2.0)+I43*(1.0/2.0))+m4*y4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(c6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*x6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3
-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+m6*y6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*
(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+I66*(s6*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(a1*c1+d6*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3))+(c1*c2*s3+c1*c3*s2)*(I34*s1+I36*(c1*c2*s3+c1*c3*s2)+(c1*c2*c3-c1*s2*s3)*(I31*(-1.0/2.0)+I32*
(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(c1*c2*c3-c1*s2*s3)*(I35*s1+I36*(c1*c2*c3
-c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a1*c1+a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3))+(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)*(m5*(a1*c1+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m5*y5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+m5*x5*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+m5*z5*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2)))+(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)*(m3*(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-m3*y3*c1-
m3*x3*(s1*s2*s3-c2*c3*s1)+m3*z3*(c2*s1*s3+c3*s1*s2))-(-m4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3)+m4*x4*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+m4*z4*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m4*y4*(c2*s1*s3+c3*s1*s2))*
(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(-m6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+m6*x6*(s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+m6*y6*(c6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+m6*z6*(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(-m5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3)+m5*y5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m5*x5*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))+m5*z5*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(m4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3)+m4*x4*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+m4*z4*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m4*y4*(c1*c2*s3+c1*c3*s2))*
(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I45*(c2*s1*s3+c3*s1*s2)+I46*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-m4*z4*
(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+s4*(c2*s1*s3+c3*s1*s2)*((c1*c4-s4*(s1*s2*s3-
c2*c3*s1))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))+I55*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m5*y5*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-s4*(c1*c2*s3+c1*c3*s2)*((c4*s1-s4*(c1*c2*c3-
c1*s2*s3))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))+I55*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m5*y5*(a1*c1+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+c4*(c1*c2*s3+c1*c3*s2)*(I46*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I44*
(c1*c2*s3+c1*c3*s2)+m4*x4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-s4*(c1*c2*s3+c1*c3*s2)*
(I46*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-
I45*(c1*c2*s3+c1*c3*s2)+m4*z4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+c1*c2*
(I25*s1+I24*c1*c2-c1*s2*(I21*(1.0/2.0)-I22*(1.0/2.0)+I23*(1.0/2.0))+m2*y2*(a1*c1+a2*c1*c2))+c1*s2*(I26*s1+c1*c2*
(I21*(-1.0/2.0)+I22*(1.0/2.0)+I23*(1.0/2.0))-I24*c1*s2+m2*x2*(a1*c1+a2*c1*c2))-c2*s1*(I25*c1-I24*c2*s1+s1*s2*(I21*
(1.0/2.0)-I22*(1.0/2.0)+I23*(1.0/2.0))-m2*y2*(a1*s1+a2*c2*s1))-s1*s2*(I26*c1-c2*s1*(I21*(-1.0/2.0)+I22*
(1.0/2.0)+I23*(1.0/2.0))+I24*s1*s2-m2*x2*(a1*s1+a2*c2*s1))-c4*(c2*s1*s3+c3*s1*s2)*(I44*(c2*s1*s3+c3*s1*s2)+I46*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-m4*x4*
(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+a2*c1*s2*(m2*
(a1*c1+a2*c1*c2)+m2*z2*s1+m2*x2*c1*c2-m2*y2*c1*s2)+a2*s1*s2*(m2*(a1*s1+a2*c2*s1)-m2*z2*c1-
m2*y2*s1*s2+m2*x2*c2*s1);;
        C222= 0;
        C233=   (c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*((c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-
c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-
c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-I66*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-
c2*c3*s1))+m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-
a3*s1*s2*s3))+(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*((c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(1.0/2.0)+I62*
(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-I66*(c6*(s5*(c2*c3-
s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+m6*z6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2))+(c1*c2*s3+c1*c3*s2)*(I36*(c1*c2*s3+c1*c3*s2)+(c1*c2*c3-c1*s2*s3)*(I31*(-
1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a3*c1*c2*c3-a3*c1*s2*s3))-(c1*c2*c3-c1*s2*s3)*(I36*(c1*c2*c3-
c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a3*c1*c2*c3-a3*c1*s2*s3))-(c6*
(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I66*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))+I64*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-(c6*
(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-
a3*s1*s2*s3))+(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)*(m4*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-
a3*s1*s2*s3)-m4*y4*(c2*s1*s3+c3*s1*s2)-m4*x4*c4*(s1*s2*s3-c2*c3*s1)+m4*z4*s4*(s1*s2*s3-c2*c3*s1))+(s6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I65*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))-I64*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+(s6*
(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-
a3*s1*s2*s3))+(m5*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-m5*x5*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-
c1*s2*s3))+m5*z5*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-m5*y5*s4*(c1*c2*c3-c1*s2*s3))*(-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+(m4*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-m4*y4*
(c1*c2*s3+c1*c3*s2)+m4*x4*c4*(c1*c2*c3-c1*s2*s3)-m4*z4*s4*(c1*c2*c3-c1*s2*s3))*(-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*
(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+I66*
(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*
(c1*c2*c3-c1*s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*
(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(-I64*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-
c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+I65*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+(s6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*y6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-
a3*c1*s2*s3))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-
c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))-I66*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-
c1*s2*s3))+(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*
(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))-(d4*
(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)*(m5*(-d4*(c2*c3-s2*s3)+a3*c2*s3+a3*c3*s2)+m5*x5*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))-m5*z5*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-m5*y5*s4*(c2*s3+c3*s2))+(m5*(d4*
(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)-m5*x5*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+m5*z5*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+m5*y5*s4*(s1*s2*s3-c2*c3*s1))*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I66*
(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+I64*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-(c6*
(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*
(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3
-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I65*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-I64*(c6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-
a3*c3*s2))+(m6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2)+m6*z6*(c5*(c2*c3-
s2*s3)-c4*s5*(c2*s3+c3*s2))-m6*x6*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+m6*y6*(s6*(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2)))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-
s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)+(s1*s2*s3-c2*c3*s1)*((c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*
(1.0/2.0))-m4*y4*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I44*c4*(s1*s2*s3-c2*c3*s1)-I45*s4*(s1*s2*s3-
c2*c3*s1))-(c2*c3-s2*s3)*(-I36*(c2*c3-s2*s3)+(c2*s3+c3*s2)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*
(a3*c2*s3+a3*c3*s2))-(c2*s3+c3*s2)*(I36*(c2*s3+c3*s2)-(c2*c3-s2*s3)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*
(1.0/2.0))+m3*z3*(a3*c2*s3+a3*c3*s2))-(m3*(a3*c2*s3+a3*c3*s2)+m3*x3*(c2*s3+c3*s2)-m3*z3*(c2*c3-s2*s3))*
(a2*c2+a3*c2*c3-a3*s2*s3)+(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*
(c1*c2*c3-c1*s2*s3))-(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0))+m5*x5*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-I54*s4*(c1*c2*c3-c1*s2*s3))+(c5*(c1*c2*c3-
c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)+I55*s4*(c1*c2*c3-c1*s2*s3))+(m3*(a3*c2*c3*s1-a3*s1*s2*s3)-m3*x3*
(s1*s2*s3-c2*c3*s1)+m3*z3*(c2*s1*s3+c3*s1*s2))*(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-
s2*s3))*(I56*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0))+I54*s4*(c2*s3+c3*s2)-m5*x5*(-d4*(c2*c3-s2*s3)+a3*c2*s3+a3*c3*s2))+(c2*s1*s3+c3*s1*s2)*
(I36*(c2*s1*s3+c3*s1*s2)-(s1*s2*s3-c2*c3*s1)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a3*c2*c3*s1-
a3*s1*s2*s3))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I56*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-(c5*(c2*c3-
s2*s3)-c4*s5*(c2*s3+c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-I55*s4*(c2*s3+c3*s2)+m5*z5*(-d4*(c2*c3-
s2*s3)+a3*c2*s3+a3*c3*s2))+(s1*s2*s3-c2*c3*s1)*(-I36*(s1*s2*s3-c2*c3*s1)+(c2*s1*s3+c3*s1*s2)*(I31*(1.0/2.0)+I32*
(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a3*c2*c3*s1-a3*s1*s2*s3))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I56*
(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I54*s4*(s1*s2*s3-
c2*c3*s1))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(-I56*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-
c2*c3*s1))+(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*
(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I55*s4*(s1*s2*s3-c2*c3*s1))-(c1*c2*c3-c1*s2*s3)*
((c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-m4*y4*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-
a3*c1*s2*s3)-I44*c4*(c1*c2*c3-c1*s2*s3)+I45*s4*(c1*c2*c3-c1*s2*s3))-(m4*(-d4*(c2*c3-
s2*s3)+a3*c2*s3+a3*c3*s2)+m4*y4*(c2*c3-s2*s3)+m4*x4*c4*(c2*s3+c3*s2)-m4*z4*s4*(c2*s3+c3*s2))*(d4*
(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)+(c2*s3+c3*s2)*((c2*c3-s2*s3)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*
(1.0/2.0))+I44*c4*(c2*s3+c3*s2)-I45*s4*(c2*s3+c3*s2)+m4*y4*(-d4*(c2*c3-s2*s3)+a3*c2*s3+a3*c3*s2))+(m6*(d6*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-m6*x6*(c6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+m6*y6*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*
(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+m6*z6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3)))*(-d6*
(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+(m6*
(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3)+m6*z6*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-m6*x6*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*
(s1*s2*s3-c2*c3*s1))+m6*y6*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1)))*(d4*
(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(m3*
(a3*c1*c2*c3-a3*c1*s2*s3)+m3*x3*(c1*c2*c3-c1*s2*s3)+m3*z3*(c1*c2*s3+c1*c3*s2))*
(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+s4*(c2*c3-s2*s3)*(I54*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-I55*(c5*(c2*c3-
s2*s3)-c4*s5*(c2*s3+c3*s2))-s4*(c2*s3+c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+m5*y5*(-d4*(c2*c3-
s2*s3)+a3*c2*s3+a3*c3*s2))-c4*(c2*s1*s3+c3*s1*s2)*(I44*(c2*s1*s3+c3*s1*s2)-m4*x4*(d4*
(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+c4*(s1*s2*s3-c2*c3*s1)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-
I46*s4*(s1*s2*s3-c2*c3*s1))+s4*(c2*s1*s3+c3*s1*s2)*(I45*(c2*s1*s3+c3*s1*s2)-m4*z4*(d4*
(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I46*c4*(s1*s2*s3-c2*c3*s1)-s4*(s1*s2*s3-c2*c3*s1)*(I41*(1.0/2.0)+I42*
(1.0/2.0)-I43*(1.0/2.0)))-c4*(c1*c2*s3+c1*c3*s2)*(I44*(c1*c2*s3+c1*c3*s2)-m4*x4*(d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-c4*(c1*c2*c3-c1*s2*s3)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*
(1.0/2.0))+I46*s4*(c1*c2*c3-c1*s2*s3))-s4*(c2*s1*s3+c3*s1*s2)*(-I54*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-
c2*c3*s1))+I55*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+m5*y5*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-
a3*s1*s2*s3)+s4*(s1*s2*s3-c2*c3*s1)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))+s4*(c1*c2*s3+c1*c3*s2)*(I54*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-I55*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-m5*y5*(d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)+s4*(c1*c2*c3-c1*s2*s3)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-
c4*(c2*c3-s2*s3)*(I44*(c2*c3-s2*s3)+c4*(c2*s3+c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I46*s4*
(c2*s3+c3*s2)+m4*x4*(-d4*(c2*c3-s2*s3)+a3*c2*s3+a3*c3*s2))+s4*(c1*c2*s3+c1*c3*s2)*(I45*(c1*c2*s3+c1*c3*s2)-m4*z4*
(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-I46*c4*(c1*c2*c3-c1*s2*s3)+s4*(c1*c2*c3-c1*s2*s3)*(I41*
(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+s4*(c2*c3-s2*s3)*(I45*(c2*c3-s2*s3)+I46*c4*(c2*s3+c3*s2)-s4*(c2*s3+c3*s2)*
(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+m4*z4*(-d4*(c2*c3-s2*s3)+a3*c2*s3+a3*c3*s2));;
        C244=   -(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-c5*s6*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+I66*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c5*c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1)))-(m4*x4*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+m4*z4*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(m5*y5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m5*x5*c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+m5*z5*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(c6*
(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))-c5*s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c5*c6*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+d6*m6*x6*s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))-(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*
(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c5*c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-
c5*s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+d6*m6*y6*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-
s2*s3))*(I64*(c6*s4*(c2*s3+c3*s2)+c4*c5*s6*(c2*s3+c3*s2))+(s4*s6*(c2*s3+c3*s2)-c4*c5*c6*(c2*s3+c3*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*c4*s5*(c2*s3+c3*s2)-d6*m6*x6*c4*s5*(c2*s3+c3*s2))+(s6*(s5*(c2*s3+c3*s2)-
c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I64*(s4*s6*(c2*s3+c3*s2)-c4*c5*c6*(c2*s3+c3*s2))+(c6*s4*
(c2*s3+c3*s2)+c4*c5*s6*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*c4*s5*(c2*s3+c3*s2)-
d6*m6*y6*c4*s5*(c2*s3+c3*s2))+(m6*x6*(s4*s6*(c2*s3+c3*s2)-c4*c5*c6*(c2*s3+c3*s2))+m6*y6*(c6*s4*
(c2*s3+c3*s2)+c4*c5*s6*(c2*s3+c3*s2))-d6*m6*c4*s5*(c2*s3+c3*s2)-m6*z6*c4*s5*(c2*s3+c3*s2))*(d4*(c2*s3+c3*s2)+d6*
(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)+(c1*c2*c3-c1*s2*s3)*(I44*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+I45*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*
(c1*c2*s3+c1*c3*s2))*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-c5*s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+(s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+c5*c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+d6*m6*x6*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))-(m4*x4*c4*(c2*s3+c3*s2)-m4*z4*s4*
(c2*s3+c3*s2))*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c5*c6*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-c5*s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0))+I65*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+d6*m6*y6*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))-(c5*
(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-c5*s6*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3)))+I66*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c5*c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+(s1*s2*s3-
c2*c3*s1)*(I44*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+I45*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(I44*c4*(c2*s3+c3*s2)-I45*s4*
(c2*s3+c3*s2))*(c2*s3+c3*s2)+(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I54*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I55*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+I56*c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-
I53*(1.0/2.0)))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(-I55*s4*(c2*s3+c3*s2)+c4*s5*(c2*s3+c3*s2)*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I56*c4*c5*(c2*s3+c3*s2))+(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(-I54*s4*
(c2*s3+c3*s2)+c4*c5*(c2*s3+c3*s2)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*c4*s5*(c2*s3+c3*s2))+(m5*y5*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+m5*x5*c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+m5*z5*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(-
d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+(m4*x4*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+m4*z4*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(-m5*y5*s4*
(c2*s3+c3*s2)+m5*x5*c4*c5*(c2*s3+c3*s2)+m5*z5*c4*s5*(c2*s3+c3*s2))*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)+(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I65*(c6*s4*(c2*s3+c3*s2)+c4*c5*s6*(c2*s3+c3*s2))+I66*(s4*s6*(c2*s3+c3*s2)-
c4*c5*c6*(c2*s3+c3*s2))-c4*s5*(c2*s3+c3*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-d6*m6*z6*c4*s5*
(c2*s3+c3*s2))-(m6*x6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c5*c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+m6*y6*(c6*(c1*c4-
s4*(s1*s2*s3-c2*c3*s1))-c5*s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+d6*m6*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+m6*z6*s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I55*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I56*c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I54*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))+c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(m6*x6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c5*c6*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3)))+m6*y6*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-c5*s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+d6*m6*s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+m6*z6*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+c4*(c1*c2*s3+c1*c3*s2)*(I46*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3))+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))-s4*
(c1*c2*s3+c1*c3*s2)*(I46*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I41*(1.0/2.0)+I42*
(1.0/2.0)-I43*(1.0/2.0)))+s4*(c2*c3-s2*s3)*(-s4*(c2*s3+c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*
(1.0/2.0))+I54*c4*c5*(c2*s3+c3*s2)+I55*c4*s5*(c2*s3+c3*s2))-c4*(c2*s1*s3+c3*s1*s2)*(I46*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))-c4*(c2*c3-s2*s3)*(c4*
(c2*s3+c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I46*s4*(c2*s3+c3*s2))+s4*(c2*s1*s3+c3*s1*s2)*(I46*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+s4*
(c2*c3-s2*s3)*(I46*c4*(c2*s3+c3*s2)-s4*(c2*s3+c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))-s4*
(c1*c2*s3+c1*c3*s2)*((c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+I55*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+s4*(c2*s1*s3+c3*s1*s2)*((c1*c4-s4*(s1*s2*s3-c2*c3*s1))
*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+I55*s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1)));;
        C255=   -(m5*x5*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-m5*z5*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))*(d4*
(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)+(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*((c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+I65*s6*
(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+d6*m6*z6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))-(d6*m6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+m6*z6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+m6*x6*c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-m6*y6*s6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I56*
(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))*(I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I66*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*
(c1*c2*s3+c1*c3*s2))*(I65*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+I64*c6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))-(c5*
(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*((s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*
(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-I65*s6*
(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*((s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))-I65*s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))*(I66*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+c6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))-(s6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I65*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*(c2*s1*s3+c3*s1*s2))+I64*c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-s6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*
(I66*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+d6*m6*x6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2)))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I65*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))-I64*c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))+(m6*z6*(c5*(c2*c3-s2*s3)-
c4*s5*(c2*s3+c3*s2))+d6*m6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-m6*x6*c6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))+m6*y6*s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*
(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)-(m5*x5*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+m5*z5*
(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(I56*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-(I56*(c5*(c2*c3
-s2*s3)-c4*s5*(c2*s3+c3*s2))-(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*
(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+(m5*x5*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+m5*z5*
(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))*(-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+(d6*m6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+m6*z6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*x6*c6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-m6*y6*s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*
(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-
(I56*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+(I56*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*(I54*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-I55*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))*
(c2*c3-s2*s3)-s4*(c1*c2*s3+c1*c3*s2)*(I54*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+I55*(s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))+s4*(I54*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))+I55*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))*(c2*s1*s3+c3*s1*s2);;
        C266=   (c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I65*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*
(c2*s3+c3*s2))-I66*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2)))+(m6*x6*(s6*(c4*s1-s4*(c1*c2*c3
-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*y6*(c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(m6*x6*(c6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))-m6*y6*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2)))*(d4*
(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)-(m6*x6*(s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+m6*y6*(c6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*
(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(c6*(s5*(c2*s3+c3*s2)-c4*c5*
(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I64*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-(c6*(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(s6*(s5*
(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I64*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))-(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0)))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+I66*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3
-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))*(I65*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))+I66*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(s6*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0)))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*
(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0)));;
        C212=0;
        C213= 0;
        C214=   (m6*x6*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+m6*y6*(c6*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-d6*m6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-
m6*z6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(c1*c2*c3-c1*s2*s3)*(I44*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-I45*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1)))+(s1*s2*s3-c2*c3*s1)*(I44*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-I45*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3)))+(m6*x6*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+m6*y6*(c6*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-d6*m6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m6*z6*s5*(c4*s1
-s4*(c1*c2*c3-c1*s2*s3)))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I65*
(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+I66*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-
c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))-d6*m6*z6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(-I54*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0))+I56*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(-I55*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+I56*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(-I55*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+I56*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*
(1.0/2.0)-I53*(1.0/2.0)))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(-I54*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(c6*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))-d6*m6*x6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))+(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*x6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(s6*
(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-d6*m6*y6*s5*(c1*c4-
s4*(s1*s2*s3-c2*c3*s1)))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*
(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-
d6*m6*y6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(m4*x4*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m4*z4*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1)))*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(-m5*y5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+m5*x5*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m5*z5*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+I66*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1)))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-d6*m6*z6*s5*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(-m5*y5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+m5*x5*c5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+m5*z5*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-
(m4*x4*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m4*z4*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+c4*(c1*c2*s3+c1*c3*s2)*(I46*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))+c4*(c2*s1*s3+c3*s1*s2)*(I46*(s1*s4+c4*(c1*c2*c3
-c1*s2*s3))-(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))+s4*(c1*c2*s3+c1*c3*s2)*
(I46*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*
(1.0/2.0)))+s4*(c2*s1*s3+c3*s1*s2)*(I46*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I41*
(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+s4*(c2*s1*s3+c3*s1*s2)*(-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)-
I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I55*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+s4*
(c1*c2*s3+c1*c3*s2)*(-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))+I55*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)));;
        C215=   -(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I66*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I65*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))-I64*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+s6*(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(m5*x5*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))-m5*z5*(c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*((c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+I65*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(m5*x5*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m5*z5*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(-
d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(d6*m6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))+m6*z6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-m6*x6*c6*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*y6*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))*
(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I56*
(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))*(I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*((c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))-I66*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+I65*s6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(d6*m6*
(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+m6*z6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))-m6*x6*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+m6*y6*s6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3
-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*
(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*
(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))*(I66*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-c6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(s6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I65*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-
s5*(c1*c2*s3+c1*c3*s2))-I64*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+s6*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-s4*(c2*s1*s3+c3*s1*s2)*(I54*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))-I55*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-s4*(I54*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-I55*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*
(c1*c2*s3+c1*c3*s2);;
        C216=   (I65*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))-I66*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))))*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-(I64*(s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*
(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(c6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))-(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-
s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))*(s6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))*(I65*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))-I66*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(c6*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(s6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0)))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*
(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(c6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0)))-(m6*x6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))-m6*y6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))))*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(m6*x6*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-m6*y6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2);;
        C223=   (c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*((c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-
c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-
c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-I66*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-
c2*c3*s1))+m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-
a3*s1*s2*s3))+(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*((c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(1.0/2.0)+I62*
(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-I66*(c6*(s5*(c2*c3-
s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+m6*z6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2))+(c1*c2*s3+c1*c3*s2)*(I36*(c1*c2*s3+c1*c3*s2)+(c1*c2*c3-c1*s2*s3)*(I31*(-
1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a3*c1*c2*c3-a3*c1*s2*s3))-(c1*c2*c3-c1*s2*s3)*(I36*(c1*c2*c3-
c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a3*c1*c2*c3-a3*c1*s2*s3))-(c6*
(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I66*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))+I64*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-(c6*
(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-
a3*s1*s2*s3))+(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)*(m4*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-
a3*s1*s2*s3)-m4*y4*(c2*s1*s3+c3*s1*s2)-m4*x4*c4*(s1*s2*s3-c2*c3*s1)+m4*z4*s4*(s1*s2*s3-c2*c3*s1))+(s6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I65*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))-I64*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+(s6*
(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-
a3*s1*s2*s3))+(m5*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-m5*x5*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-
c1*s2*s3))+m5*z5*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-m5*y5*s4*(c1*c2*c3-c1*s2*s3))*(-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+(m4*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-m4*y4*
(c1*c2*s3+c1*c3*s2)+m4*x4*c4*(c1*c2*c3-c1*s2*s3)-m4*z4*s4*(c1*c2*c3-c1*s2*s3))*(-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*
(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+I66*
(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*
(c1*c2*c3-c1*s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*
(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(-I64*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-
c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+I65*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+(s6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*y6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-
a3*c1*s2*s3))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-
c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))-I66*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-
c1*s2*s3))+(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*
(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))-(d4*
(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)*(m5*(-d4*(c2*c3-s2*s3)+a3*c2*s3+a3*c3*s2)+m5*x5*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))-m5*z5*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-m5*y5*s4*(c2*s3+c3*s2))+(m5*(d4*
(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)-m5*x5*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+m5*z5*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+m5*y5*s4*(s1*s2*s3-c2*c3*s1))*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I66*
(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+I64*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-(c6*
(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*
(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3
-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I65*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-I64*(c6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-
a3*c3*s2))+(m6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2)+m6*z6*(c5*(c2*c3-
s2*s3)-c4*s5*(c2*s3+c3*s2))-m6*x6*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+m6*y6*(s6*(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2)))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-
s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)+(s1*s2*s3-c2*c3*s1)*((c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*
(1.0/2.0))-m4*y4*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I44*c4*(s1*s2*s3-c2*c3*s1)-I45*s4*(s1*s2*s3-
c2*c3*s1))-(c2*c3-s2*s3)*(-I36*(c2*c3-s2*s3)+(c2*s3+c3*s2)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*
(a3*c2*s3+a3*c3*s2))-(c2*s3+c3*s2)*(I36*(c2*s3+c3*s2)-(c2*c3-s2*s3)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*
(1.0/2.0))+m3*z3*(a3*c2*s3+a3*c3*s2))-(m3*(a3*c2*s3+a3*c3*s2)+m3*x3*(c2*s3+c3*s2)-m3*z3*(c2*c3-s2*s3))*
(a2*c2+a3*c2*c3-a3*s2*s3)+(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*
(c1*c2*c3-c1*s2*s3))-(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0))+m5*x5*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-I54*s4*(c1*c2*c3-c1*s2*s3))+(c5*(c1*c2*c3-
c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)+I55*s4*(c1*c2*c3-c1*s2*s3))+(m3*(a3*c2*c3*s1-a3*s1*s2*s3)-m3*x3*
(s1*s2*s3-c2*c3*s1)+m3*z3*(c2*s1*s3+c3*s1*s2))*(a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-
s2*s3))*(I56*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0))+I54*s4*(c2*s3+c3*s2)-m5*x5*(-d4*(c2*c3-s2*s3)+a3*c2*s3+a3*c3*s2))+(c2*s1*s3+c3*s1*s2)*
(I36*(c2*s1*s3+c3*s1*s2)-(s1*s2*s3-c2*c3*s1)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a3*c2*c3*s1-
a3*s1*s2*s3))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I56*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-(c5*(c2*c3-
s2*s3)-c4*s5*(c2*s3+c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-I55*s4*(c2*s3+c3*s2)+m5*z5*(-d4*(c2*c3-
s2*s3)+a3*c2*s3+a3*c3*s2))+(s1*s2*s3-c2*c3*s1)*(-I36*(s1*s2*s3-c2*c3*s1)+(c2*s1*s3+c3*s1*s2)*(I31*(1.0/2.0)+I32*
(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a3*c2*c3*s1-a3*s1*s2*s3))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I56*
(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I54*s4*(s1*s2*s3-
c2*c3*s1))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(-I56*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-
c2*c3*s1))+(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*
(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I55*s4*(s1*s2*s3-c2*c3*s1))-(c1*c2*c3-c1*s2*s3)*
((c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-m4*y4*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-
a3*c1*s2*s3)-I44*c4*(c1*c2*c3-c1*s2*s3)+I45*s4*(c1*c2*c3-c1*s2*s3))-(m4*(-d4*(c2*c3-
s2*s3)+a3*c2*s3+a3*c3*s2)+m4*y4*(c2*c3-s2*s3)+m4*x4*c4*(c2*s3+c3*s2)-m4*z4*s4*(c2*s3+c3*s2))*(d4*
(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)+(c2*s3+c3*s2)*((c2*c3-s2*s3)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*
(1.0/2.0))+I44*c4*(c2*s3+c3*s2)-I45*s4*(c2*s3+c3*s2)+m4*y4*(-d4*(c2*c3-s2*s3)+a3*c2*s3+a3*c3*s2))+(m6*(d6*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-m6*x6*(c6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+m6*y6*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*
(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+m6*z6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3)))*(-d6*
(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+(m6*
(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3)+m6*z6*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-m6*x6*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*
(s1*s2*s3-c2*c3*s1))+m6*y6*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1)))*(d4*
(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(m3*
(a3*c1*c2*c3-a3*c1*s2*s3)+m3*x3*(c1*c2*c3-c1*s2*s3)+m3*z3*(c1*c2*s3+c1*c3*s2))*
(a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+s4*(c2*c3-s2*s3)*(I54*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-I55*(c5*(c2*c3-
s2*s3)-c4*s5*(c2*s3+c3*s2))-s4*(c2*s3+c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+m5*y5*(-d4*(c2*c3-
s2*s3)+a3*c2*s3+a3*c3*s2))-c4*(c2*s1*s3+c3*s1*s2)*(I44*(c2*s1*s3+c3*s1*s2)-m4*x4*(d4*
(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+c4*(s1*s2*s3-c2*c3*s1)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-
I46*s4*(s1*s2*s3-c2*c3*s1))+s4*(c2*s1*s3+c3*s1*s2)*(I45*(c2*s1*s3+c3*s1*s2)-m4*z4*(d4*
(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I46*c4*(s1*s2*s3-c2*c3*s1)-s4*(s1*s2*s3-c2*c3*s1)*(I41*(1.0/2.0)+I42*
(1.0/2.0)-I43*(1.0/2.0)))-c4*(c1*c2*s3+c1*c3*s2)*(I44*(c1*c2*s3+c1*c3*s2)-m4*x4*(d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-c4*(c1*c2*c3-c1*s2*s3)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*
(1.0/2.0))+I46*s4*(c1*c2*c3-c1*s2*s3))-s4*(c2*s1*s3+c3*s1*s2)*(-I54*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-
c2*c3*s1))+I55*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+m5*y5*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-
a3*s1*s2*s3)+s4*(s1*s2*s3-c2*c3*s1)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))+s4*(c1*c2*s3+c1*c3*s2)*(I54*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-I55*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-m5*y5*(d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)+s4*(c1*c2*c3-c1*s2*s3)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-
c4*(c2*c3-s2*s3)*(I44*(c2*c3-s2*s3)+c4*(c2*s3+c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I46*s4*
(c2*s3+c3*s2)+m4*x4*(-d4*(c2*c3-s2*s3)+a3*c2*s3+a3*c3*s2))+s4*(c1*c2*s3+c1*c3*s2)*(I45*(c1*c2*s3+c1*c3*s2)-m4*z4*
(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-I46*c4*(c1*c2*c3-c1*s2*s3)+s4*(c1*c2*c3-c1*s2*s3)*(I41*
(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+s4*(c2*c3-s2*s3)*(I45*(c2*c3-s2*s3)+I46*c4*(c2*s3+c3*s2)-s4*(c2*s3+c3*s2)*
(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+m4*z4*(-d4*(c2*c3-s2*s3)+a3*c2*s3+a3*c3*s2));;
        C224=   -(m5*y5*c4*(c1*c2*s3+c1*c3*s2)+m5*x5*c5*s4*(c1*c2*s3+c1*c3*s2)+m5*z5*s4*s5*(c1*c2*s3+c1*c3*s2))*(-
d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(m6*x6*(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*
(c2*s1*s3+c3*s1*s2))+m6*y6*(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))+d6*m6*s4*s5*
(c2*s1*s3+c3*s1*s2)+m6*z6*s4*s5*(c2*s1*s3+c3*s1*s2))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-
s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))+(c4*s6*
(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*
(c1*c2*s3+c1*c3*s2)+d6*m6*x6*s4*s5*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*
(c1*c2*s3+c1*c3*s2))+(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+I65*s4*s5*(c1*c2*s3+c1*c3*s2)+d6*m6*y6*s4*s5*(c1*c2*s3+c1*c3*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(I65*(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))+I66*(c4*s6*
(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+s4*s5*(c2*s1*s3+c3*s1*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+d6*m6*z6*s4*s5*(c2*s1*s3+c3*s1*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(c4*c6*
(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))+I66*(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*
(c1*c2*s3+c1*c3*s2))+s4*s5*(c1*c2*s3+c1*c3*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s4*s5*
(c1*c2*s3+c1*c3*s2))+(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I64*(c4*c6*(c2*c3-s2*s3)-
c5*s4*s6*(c2*c3-s2*s3))+(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+I66*s4*s5*(c2*c3-s2*s3)+d6*m6*x6*s4*s5*(c2*c3-s2*s3))-(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*
(c2*c3-s2*s3))*(I64*(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))+(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-s2*s3))*
(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c2*c3-s2*s3)+d6*m6*y6*s4*s5*(c2*c3-s2*s3))-(m6*x6*(c4*s6*
(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))+m6*y6*(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-s2*s3))+d6*m6*s4*s5*(c2*c3-
s2*s3)+m6*z6*s4*s5*(c2*c3-s2*s3))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-
a3*s2*s3)+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*(c4*c6*
(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))+(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*(c2*s1*s3+c3*s1*s2)+d6*m6*x6*s4*s5*(c2*s1*s3+c3*s1*s2))-(s6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*(c4*s6*
(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c2*s1*s3+c3*s1*s2)+d6*m6*y6*s4*s5*(c2*s1*s3+c3*s1*s2))-
(m4*z4*c4*(c2*c3-s2*s3)+m4*x4*s4*(c2*c3-s2*s3))*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)-(c1*c2*c3-c1*s2*s3)*
(I45*c4*(c1*c2*s3+c1*c3*s2)+I44*s4*(c1*c2*s3+c1*c3*s2))-(m4*z4*c4*(c2*s1*s3+c3*s1*s2)+m4*x4*s4*
(c2*s1*s3+c3*s1*s2))*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(I45*c4*(c2*c3-s2*s3)+I44*s4*(c2*c3
-s2*s3))*(c2*s3+c3*s2)-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I55*c4*(c2*s1*s3+c3*s1*s2)+I56*c5*s4*
(c2*s1*s3+c3*s1*s2)+s4*s5*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I54*c4*(c2*s1*s3+c3*s1*s2)+c5*s4*(c2*s1*s3+c3*s1*s2)*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*(c2*s1*s3+c3*s1*s2))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I55*c4*(c2*c3-
s2*s3)+s4*s5*(c2*c3-s2*s3)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I56*c5*s4*(c2*c3-s2*s3))+(s5*(c2*s3+c3*s2)-
c4*c5*(c2*c3-s2*s3))*(I54*c4*(c2*c3-s2*s3)+c5*s4*(c2*c3-s2*s3)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0))+I56*s4*s5*(c2*c3-s2*s3))-(m5*y5*c4*(c2*s1*s3+c3*s1*s2)+m5*x5*c5*s4*(c2*s1*s3+c3*s1*s2)+m5*z5*s4*s5*
(c2*s1*s3+c3*s1*s2))*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(m6*x6*(c4*s6*
(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))+m6*y6*(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*
(c1*c2*s3+c1*c3*s2))+d6*m6*s4*s5*(c1*c2*s3+c1*c3*s2)+m6*z6*s4*s5*(c1*c2*s3+c1*c3*s2))*(-d6*(c5*(c1*c2*c3-
c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(m5*y5*c4*(c2*c3-
s2*s3)+m5*x5*c5*s4*(c2*c3-s2*s3)+m5*z5*s4*s5*(c2*c3-s2*s3))*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)+(I45*c4*
(c2*s1*s3+c3*s1*s2)+I44*s4*(c2*s1*s3+c3*s1*s2))*(s1*s2*s3-c2*c3*s1)-(m4*z4*c4*(c1*c2*s3+c1*c3*s2)+m4*x4*s4*
(c1*c2*s3+c1*c3*s2))*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))*(I54*c4*(c1*c2*s3+c1*c3*s2)+c5*s4*(c1*c2*s3+c1*c3*s2)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0))+I56*s4*s5*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I55*c4*
(c1*c2*s3+c1*c3*s2)+I56*c5*s4*(c1*c2*s3+c1*c3*s2)+s4*s5*(c1*c2*s3+c1*c3*s2)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0)))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I65*(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-s2*s3))+I66*(c4*s6*
(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))+s4*s5*(c2*c3-s2*s3)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+d6*m6*z6*s4*s5*(c2*c3-s2*s3))+s4*(c2*c3-s2*s3)*(c4*(c2*c3-s2*s3)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*
(1.0/2.0))+I55*s4*s5*(c2*c3-s2*s3)+I54*c5*s4*(c2*c3-s2*s3))-c4*(c1*c2*s3+c1*c3*s2)*(I46*c4*(c1*c2*s3+c1*c3*s2)+s4*
(c1*c2*s3+c1*c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))-c4*(c2*c3-s2*s3)*(I46*c4*(c2*c3-s2*s3)+s4*(c2*c3
-s2*s3)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))+s4*(c2*s1*s3+c3*s1*s2)*(c4*(c2*s1*s3+c3*s1*s2)*(I51*
(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*s4*(c2*s1*s3+c3*s1*s2)+I55*s4*s5*(c2*s1*s3+c3*s1*s2))+s4*
(c1*c2*s3+c1*c3*s2)*(c4*(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+I46*s4*
(c1*c2*s3+c1*c3*s2))+s4*(c2*c3-s2*s3)*(c4*(c2*c3-s2*s3)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+I46*s4*(c2*c3-
s2*s3))-c4*(I46*c4*(c2*s1*s3+c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))*
(c2*s1*s3+c3*s1*s2)+s4*(c4*(c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+I46*s4*
(c2*s1*s3+c3*s1*s2))*(c2*s1*s3+c3*s1*s2)+s4*(c1*c2*s3+c1*c3*s2)*(c4*(c1*c2*s3+c1*c3*s2)*(I51*(1.0/2.0)-I52*
(1.0/2.0)+I53*(1.0/2.0))+I54*c5*s4*(c1*c2*s3+c1*c3*s2)+I55*s4*s5*(c1*c2*s3+c1*c3*s2));;
        C225=   (c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I66*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))-(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*
(c2*s1*s3+c3*s1*s2))*(I65*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+I64*c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))-s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+d6*m6*y6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))-(m5*x5*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-
s2*s3))+m5*z5*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)-(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*((s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+I66*c6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-I65*s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+d6*m6*z6*
(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*
(1.0/2.0)-I53*(1.0/2.0)))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0)))+(d6*m6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+m6*z6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+m6*x6*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-m6*y6*s6*(c5*(c1*c2*c3-c1*s2*s3)-
c4*s5*(c1*c2*s3+c1*c3*s2)))*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I66*
(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+c6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+d6*m6*x6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-
s2*s3)))-(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I65*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-
s2*s3))+I64*c6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(1.0/2.0)-
I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))-(m6*z6*(s5*(c2*s3+c3*s2)-c4*c5*
(c2*c3-s2*s3))+d6*m6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+m6*x6*c6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-
m6*y6*s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3)))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-
s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)+(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0)))-(I56*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I51*(1.0/2.0)+I52*
(1.0/2.0)-I53*(1.0/2.0)))*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+(I56*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-
s2*s3))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I66*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*
(c1*c2*s3+c1*c3*s2))*(I65*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+I64*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))-s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+d6*m6*y6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))*((s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+I66*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-I65*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))-(m5*x5*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+m5*z5*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(d6*m6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+m6*z6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+m6*x6*c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-
m6*y6*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(m5*x5*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+m5*z5*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))*(-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*((s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I65*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))+s4*(c2*s1*s3+c3*s1*s2)*(I54*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+I55*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))+s4*(I54*(c5*(c2*s3+c3*s2)+c4*s5*
(c2*c3-s2*s3))+I55*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))*(c2*c3-s2*s3)-s4*(c1*c2*s3+c1*c3*s2)*(I54*(c5*(c1*c2*c3
-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+I55*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)));;
        C226=   (c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I65*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*
(c2*c3-s2*s3))+I66*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3)))+(m6*x6*(s6*(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))+m6*y6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(m6*x6*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*
(c2*c3-s2*s3))+m6*y6*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3)))*(d4*(c2*s3+c3*s2)+d6*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*
(I65*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+I66*(s6*(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))*(I65*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+I66*
(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2)))+(s6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*
(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*
(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I64*(c6*
(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*
(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*
(c2*c3-s2*s3))*(I64*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))+(c6*(s5*(c2*s3+c3*s2)-c4*c5*
(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*
(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*
(c2*s1*s3+c3*s1*s2))+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(m6*x6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*
(c1*c2*s3+c1*c3*s2))+m6*y6*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2)))*(-
d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2);;
        C234=   -(m5*y5*c4*(c1*c2*s3+c1*c3*s2)+m5*x5*c5*s4*(c1*c2*s3+c1*c3*s2)+m5*z5*s4*s5*(c1*c2*s3+c1*c3*s2))*(-
d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(m6*x6*(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*
(c2*s1*s3+c3*s1*s2))+m6*y6*(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))+d6*m6*s4*s5*
(c2*s1*s3+c3*s1*s2)+m6*z6*s4*s5*(c2*s1*s3+c3*s1*s2))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-
s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))+(c4*s6*
(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*
(c1*c2*s3+c1*c3*s2)+d6*m6*x6*s4*s5*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*
(c1*c2*s3+c1*c3*s2))+(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+I65*s4*s5*(c1*c2*s3+c1*c3*s2)+d6*m6*y6*s4*s5*(c1*c2*s3+c1*c3*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(I65*(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))+I66*(c4*s6*
(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+s4*s5*(c2*s1*s3+c3*s1*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+d6*m6*z6*s4*s5*(c2*s1*s3+c3*s1*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(c4*c6*
(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))+I66*(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*
(c1*c2*s3+c1*c3*s2))+s4*s5*(c1*c2*s3+c1*c3*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s4*s5*
(c1*c2*s3+c1*c3*s2))+(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I64*(c4*c6*(c2*c3-s2*s3)-
c5*s4*s6*(c2*c3-s2*s3))+(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+I66*s4*s5*(c2*c3-s2*s3)+d6*m6*x6*s4*s5*(c2*c3-s2*s3))-(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*
(c2*c3-s2*s3))*(I64*(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))+(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-s2*s3))*
(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c2*c3-s2*s3)+d6*m6*y6*s4*s5*(c2*c3-s2*s3))-(m6*x6*(c4*s6*
(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))+m6*y6*(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-s2*s3))+d6*m6*s4*s5*(c2*c3-
s2*s3)+m6*z6*s4*s5*(c2*c3-s2*s3))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-
a3*s2*s3)+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*(c4*c6*
(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))+(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*(c2*s1*s3+c3*s1*s2)+d6*m6*x6*s4*s5*(c2*s1*s3+c3*s1*s2))-(s6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*(c4*s6*
(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c2*s1*s3+c3*s1*s2)+d6*m6*y6*s4*s5*(c2*s1*s3+c3*s1*s2))-
(m4*z4*c4*(c2*c3-s2*s3)+m4*x4*s4*(c2*c3-s2*s3))*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)-(c1*c2*c3-c1*s2*s3)*
(I45*c4*(c1*c2*s3+c1*c3*s2)+I44*s4*(c1*c2*s3+c1*c3*s2))-(m4*z4*c4*(c2*s1*s3+c3*s1*s2)+m4*x4*s4*
(c2*s1*s3+c3*s1*s2))*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(I45*c4*(c2*c3-s2*s3)+I44*s4*(c2*c3
-s2*s3))*(c2*s3+c3*s2)-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I55*c4*(c2*s1*s3+c3*s1*s2)+I56*c5*s4*
(c2*s1*s3+c3*s1*s2)+s4*s5*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I54*c4*(c2*s1*s3+c3*s1*s2)+c5*s4*(c2*s1*s3+c3*s1*s2)*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*(c2*s1*s3+c3*s1*s2))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I55*c4*(c2*c3-
s2*s3)+s4*s5*(c2*c3-s2*s3)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I56*c5*s4*(c2*c3-s2*s3))+(s5*(c2*s3+c3*s2)-
c4*c5*(c2*c3-s2*s3))*(I54*c4*(c2*c3-s2*s3)+c5*s4*(c2*c3-s2*s3)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0))+I56*s4*s5*(c2*c3-s2*s3))-(m5*y5*c4*(c2*s1*s3+c3*s1*s2)+m5*x5*c5*s4*(c2*s1*s3+c3*s1*s2)+m5*z5*s4*s5*
(c2*s1*s3+c3*s1*s2))*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(m6*x6*(c4*s6*
(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))+m6*y6*(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*
(c1*c2*s3+c1*c3*s2))+d6*m6*s4*s5*(c1*c2*s3+c1*c3*s2)+m6*z6*s4*s5*(c1*c2*s3+c1*c3*s2))*(-d6*(c5*(c1*c2*c3-
c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(m5*y5*c4*(c2*c3-
s2*s3)+m5*x5*c5*s4*(c2*c3-s2*s3)+m5*z5*s4*s5*(c2*c3-s2*s3))*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)+(I45*c4*
(c2*s1*s3+c3*s1*s2)+I44*s4*(c2*s1*s3+c3*s1*s2))*(s1*s2*s3-c2*c3*s1)-(m4*z4*c4*(c1*c2*s3+c1*c3*s2)+m4*x4*s4*
(c1*c2*s3+c1*c3*s2))*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))*(I54*c4*(c1*c2*s3+c1*c3*s2)+c5*s4*(c1*c2*s3+c1*c3*s2)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0))+I56*s4*s5*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I55*c4*
(c1*c2*s3+c1*c3*s2)+I56*c5*s4*(c1*c2*s3+c1*c3*s2)+s4*s5*(c1*c2*s3+c1*c3*s2)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0)))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I65*(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-s2*s3))+I66*(c4*s6*
(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))+s4*s5*(c2*c3-s2*s3)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+d6*m6*z6*s4*s5*(c2*c3-s2*s3))+s4*(c2*c3-s2*s3)*(c4*(c2*c3-s2*s3)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*
(1.0/2.0))+I55*s4*s5*(c2*c3-s2*s3)+I54*c5*s4*(c2*c3-s2*s3))-c4*(c1*c2*s3+c1*c3*s2)*(I46*c4*(c1*c2*s3+c1*c3*s2)+s4*
(c1*c2*s3+c1*c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))-c4*(c2*c3-s2*s3)*(I46*c4*(c2*c3-s2*s3)+s4*(c2*c3
-s2*s3)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))+s4*(c2*s1*s3+c3*s1*s2)*(c4*(c2*s1*s3+c3*s1*s2)*(I51*
(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*s4*(c2*s1*s3+c3*s1*s2)+I55*s4*s5*(c2*s1*s3+c3*s1*s2))+s4*
(c1*c2*s3+c1*c3*s2)*(c4*(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+I46*s4*
(c1*c2*s3+c1*c3*s2))+s4*(c2*c3-s2*s3)*(c4*(c2*c3-s2*s3)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+I46*s4*(c2*c3-
s2*s3))-c4*(I46*c4*(c2*s1*s3+c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))*
(c2*s1*s3+c3*s1*s2)+s4*(c4*(c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+I46*s4*
(c2*s1*s3+c3*s1*s2))*(c2*s1*s3+c3*s1*s2)+s4*(c1*c2*s3+c1*c3*s2)*(c4*(c1*c2*s3+c1*c3*s2)*(I51*(1.0/2.0)-I52*
(1.0/2.0)+I53*(1.0/2.0))+I54*c5*s4*(c1*c2*s3+c1*c3*s2)+I55*s4*s5*(c1*c2*s3+c1*c3*s2));;
        C235=   (c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I66*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))-(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*
(c2*s1*s3+c3*s1*s2))*(I65*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+I64*c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))-s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+d6*m6*y6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))-(m5*x5*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-
s2*s3))+m5*z5*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))*(d4*(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)-(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*((s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+I66*c6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-I65*s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+d6*m6*z6*
(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*
(1.0/2.0)-I53*(1.0/2.0)))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0)))+(d6*m6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+m6*z6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+m6*x6*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-m6*y6*s6*(c5*(c1*c2*c3-c1*s2*s3)-
c4*s5*(c1*c2*s3+c1*c3*s2)))*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I66*
(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+c6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+d6*m6*x6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-
s2*s3)))-(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I65*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-
s2*s3))+I64*c6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(1.0/2.0)-
I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))-(m6*z6*(s5*(c2*s3+c3*s2)-c4*c5*
(c2*c3-s2*s3))+d6*m6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+m6*x6*c6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-
m6*y6*s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3)))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-
s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)+(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0)))-(I56*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I51*(1.0/2.0)+I52*
(1.0/2.0)-I53*(1.0/2.0)))*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+(I56*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-
s2*s3))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I66*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*
(c1*c2*s3+c1*c3*s2))*(I65*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+I64*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))-s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+d6*m6*y6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))*((s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+I66*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-I65*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))-(m5*x5*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+m5*z5*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(d6*m6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+m6*z6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+m6*x6*c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-
m6*y6*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(m5*x5*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+m5*z5*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))*(-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*((s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I65*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))+s4*(c2*s1*s3+c3*s1*s2)*(I54*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+I55*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))+s4*(I54*(c5*(c2*s3+c3*s2)+c4*s5*
(c2*c3-s2*s3))+I55*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))*(c2*c3-s2*s3)-s4*(c1*c2*s3+c1*c3*s2)*(I54*(c5*(c1*c2*c3
-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+I55*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)));;
        C236=   (c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I65*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*
(c2*c3-s2*s3))+I66*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3)))+(m6*x6*(s6*(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))+m6*y6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(m6*x6*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*
(c2*c3-s2*s3))+m6*y6*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3)))*(d4*(c2*s3+c3*s2)+d6*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*
(I65*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+I66*(s6*(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))*(I65*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+I66*
(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2)))+(s6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*
(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*
(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I64*(c6*
(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*
(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*
(c2*c3-s2*s3))*(I64*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))+(c6*(s5*(c2*s3+c3*s2)-c4*c5*
(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*
(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*
(c2*s1*s3+c3*s1*s2))+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(m6*x6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*
(c1*c2*s3+c1*c3*s2))+m6*y6*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2)))*(-
d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2);;
        C245=   (c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I61*
(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I65*s5*s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+d6*m6*z6*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(m6*z6*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+d6*m6*c5*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3))+m6*y6*s5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m6*x6*c6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(-
d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-
(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I66*c5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))-c6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s5*s6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))+d6*m6*x6*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I65*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-I64*c6*s5*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))+s5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*c5*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(m5*z5*c5*s4*(c2*s3+c3*s2)-m5*x5*s4*s5*(c2*s3+c3*s2))*(d4*
(c2*s3+c3*s2)+a2*c2+a3*c2*c3-a3*s2*s3)-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(c5*s4*(c2*s3+c3*s2)*(I61*
(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*s4*s5*(c2*s3+c3*s2)+I65*s4*s5*s6*(c2*s3+c3*s2)+d6*m6*z6*c5*s4*
(c2*s3+c3*s2))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-I56*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))*(I56*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0)))-(m5*z5*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m5*x5*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(-d4*
(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-
s4*s6*(c1*c2*s3+c1*c3*s2))*(I66*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-c6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+d6*m6*x6*c5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I65*c5*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))-I64*c6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+s5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)-
I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-
s2*s3))+s4*s6*(c2*c3-s2*s3))*(I66*c5*s4*(c2*s3+c3*s2)-c6*s4*s5*(c2*s3+c3*s2)*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+I64*s4*s5*s6*(c2*s3+c3*s2)+d6*m6*x6*c5*s4*(c2*s3+c3*s2))-(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-
c6*s4*(c2*c3-s2*s3))*(I65*c5*s4*(c2*s3+c3*s2)-I64*c6*s4*s5*(c2*s3+c3*s2)+s4*s5*s6*(c2*s3+c3*s2)*(I61*(1.0/2.0)-
I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*c5*s4*(c2*s3+c3*s2))-(d6*m6*c5*s4*(c2*s3+c3*s2)+m6*z6*c5*s4*(c2*s3+c3*s2)-
m6*x6*c6*s4*s5*(c2*s3+c3*s2)+m6*y6*s4*s5*s6*(c2*s3+c3*s2))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-
s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(c5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I65*s5*s6*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3))+d6*m6*z6*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(c5*s4*(c2*s3+c3*s2)*(I51*(1.0/2.0)+I52*
(1.0/2.0)-I53*(1.0/2.0))-I56*s4*s5*(c2*s3+c3*s2))*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-(s4*s5*(c2*s3+c3*s2)*
(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-I56*c5*s4*(c2*s3+c3*s2))*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-
s2*s3))+(m5*z5*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m5*x5*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(d6*m6*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m6*z6*c5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))-m6*x6*c6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m6*y6*s5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(d4*(s1*s2*s3-
c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I56*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*
(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(c5*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-I56*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-s4*
(c2*s1*s3+c3*s1*s2)*(I55*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-I54*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-s4*(I54*s4*s5*
(c2*s3+c3*s2)-I55*c5*s4*(c2*s3+c3*s2))*(c2*c3-s2*s3)+s4*(c1*c2*s3+c1*c3*s2)*(I55*c5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-I54*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)));;
        C246=   -(m6*x6*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))-m6*y6*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*
(c2*s3+c3*s2)))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-a3*s2*s3)-(c6*(s5*
(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I64*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))-(c4*c6*
(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(s6*(s5*(c2*s3+c3*s2)-c4*c5*
(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I64*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))-(c4*s6*
(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))*(I65*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-I66*(c6*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))-(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0)))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*
(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(m6*x6*(c6*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-m6*y6*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+(m6*x6*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))-m6*y6*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))))*(-d6*(c5*
(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+(I65*(s6*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-I66*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1))))*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+(I65*(c4*s6*
(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))-I66*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2)))*(c5*(c2*s3+c3*s2)+c4*s5*
(c2*c3-s2*s3))-(I64*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c6*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(c6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))-(I64*(c6*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-
c6*s4*(c2*s1*s3+c3*s1*s2));;
        C256=   -(I65*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+I66*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))*
(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+(m6*y6*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+m6*x6*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-
c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-(m6*y6*c6*(s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*x6*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2)))*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-(m6*y6*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+m6*x6*s6*(c5*(c2*c3-
s2*s3)-c4*s5*(c2*s3+c3*s2)))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a2*c2+a3*c2*c3-
a3*s2*s3)+(I64*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))-(c6*(c5*
(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2)))*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(I65*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+I66*s6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))*(I64*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+s6*(s5*(c1*s4+c4*(s1*s2*s3
-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2)))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*c6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+I66*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))+(s6*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(c6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*
(c1*c2*s3+c1*c3*s2))*(I64*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+s6*(s5*(s1*s4+c4*(c1*c2*c3
-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)));;
        C311=   -(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*
(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+I66*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))
*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))*(I54*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I56*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0))+m5*x5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(s1*s2*s3-c2*c3*s1)*
((c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))+I44*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+I45*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))-m4*y4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c5*(c1*c2*c3-
c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I55*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-
s5*(c1*c2*s3+c1*c3*s2))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-
I53*(1.0/2.0))+m5*z5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(c6*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-
c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))-m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c2*s1*s3+c3*s1*s2)*(I36*(c2*s1*s3+c3*s1*s2)-(s1*s2*s3-
c2*c3*s1)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))-I34*c1+m3*x3*(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-
(s1*s2*s3-c2*c3*s1)*(I36*(s1*s2*s3-c2*c3*s1)-(c2*s1*s3+c3*s1*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*
(1.0/2.0))+I35*c1-m3*z3*(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))*(I54*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I56*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0))-m5*x5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(I55*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I56*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0))-m5*z5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(a3*c2*s1*s3+a3*c3*s1*s2)*(m3*
(a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-m3*y3*c1-m3*x3*(s1*s2*s3-c2*c3*s1)+m3*z3*(c2*s1*s3+c3*s1*s2))+(-d4*
(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)*(m5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3)+m5*y5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+m5*x5*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))+m5*z5*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))-(d4*(s1*s2*s3-
c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)*(-m4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+m4*x4*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+m4*z4*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m4*y4*(c2*s1*s3+c3*s1*s2))-(d4*(s1*s2*s3-
c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)*(-m6*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3)+m6*x6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))+m6*y6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))+m6*z6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))-(d4*(s1*s2*s3-
c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)*(-m5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+m5*y5*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m5*x5*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+m5*z5*(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))+(c1*c2*c3-c1*s2*s3)*(I44*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+I45*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*
(1.0/2.0))+m4*y4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(-d4*(c1*c2*c3-
c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)*(m4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m4*x4*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+m4*z4*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m4*y4*(c1*c2*s3+c1*c3*s2))+(c6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*x6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3
-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+m6*y6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*
(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+I66*(s6*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(a1*c1+d6*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3))+(c1*c2*s3+c1*c3*s2)*(I34*s1+I36*(c1*c2*s3+c1*c3*s2)+(c1*c2*c3-c1*s2*s3)*(I31*(-1.0/2.0)+I32*
(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(m6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m6*x6*(s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*y6*(c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*z6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-
a3*c1*c2*s3-a3*c1*c3*s2)-(c1*c2*c3-c1*s2*s3)*(I35*s1+I36*(c1*c2*c3-c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I31*
(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(a3*c1*c2*s3+a3*c1*c3*s2)*
(m3*(a1*c1+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m3*y3*s1+m3*x3*(c1*c2*c3-c1*s2*s3)+m3*z3*(c1*c2*s3+c1*c3*s2))+s4*
(c2*s1*s3+c3*s1*s2)*(I45*(c2*s1*s3+c3*s1*s2)+I46*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*
(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-m4*z4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3))+s4*(c2*s1*s3+c3*s1*s2)*((c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*
(1.0/2.0))+I54*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+I55*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*(c2*s1*s3+c3*s1*s2))-m5*y5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-s4*
(c1*c2*s3+c1*c3*s2)*((c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+I55*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m5*y5*
(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+c4*(c1*c2*s3+c1*c3*s2)*(I46*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I44*
(c1*c2*s3+c1*c3*s2)+m4*x4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-s4*(c1*c2*s3+c1*c3*s2)*
(I46*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-
I45*(c1*c2*s3+c1*c3*s2)+m4*z4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-c4*
(c2*s1*s3+c3*s1*s2)*(I44*(c2*s1*s3+c3*s1*s2)+I46*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*
(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-m4*x4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3));;
        C322=   (c2*s1*s3+c3*s1*s2)*(I36*(c2*s1*s3+c3*s1*s2)-(s1*s2*s3-c2*c3*s1)*(I31*(-1.0/2.0)+I32*
(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(s1*s2*s3-c2*c3*s1)*(-I36*(s1*s2*s3-
c2*c3*s1)+(c2*s1*s3+c3*s1*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3))+(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)*(m4*(d4*(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3)-m4*y4*(c2*s1*s3+c3*s1*s2)-m4*x4*c4*(s1*s2*s3-c2*c3*s1)+m4*z4*s4*(s1*s2*s3-c2*c3*s1))-(c1*c2*c3-
c1*s2*s3)*((c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))-m4*y4*(d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-I44*c4*(c1*c2*c3-c1*s2*s3)+I45*s4*(c1*c2*c3-c1*s2*s3))-
(c2*c3-s2*s3)*(-I36*(c2*c3-s2*s3)+(c2*s3+c3*s2)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*
(a2*s2+a3*c2*s3+a3*c3*s2))+(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)*(m5*(d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-m5*x5*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-
c1*s2*s3))+m5*z5*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-m5*y5*s4*(c1*c2*c3-c1*s2*s3))-(c2*s3+c3*s2)*
(I36*(c2*s3+c3*s2)-(c2*c3-s2*s3)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*(1.0/2.0))+m3*z3*(a2*s2+a3*c2*s3+a3*c3*s2))-(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I54*s4*(s1*s2*s3-
c2*c3*s1)+m5*x5*(d4*(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(-I56*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I55*s4*(s1*s2*s3-c2*c3*s1)+m5*z5*(d4*
(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)*(m4*(d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-m4*y4*(c1*c2*s3+c1*c3*s2)+m4*x4*c4*(c1*c2*c3-c1*s2*s3)-
m4*z4*s4*(c1*c2*c3-c1*s2*s3))-(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I56*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-
(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I54*s4*(c2*s3+c3*s2)-m5*x5*(-
d4*(c2*c3-s2*s3)+a2*s2+a3*c2*s3+a3*c3*s2))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I56*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))-(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-I55*s4*
(c2*s3+c3*s2)+m5*z5*(-d4*(c2*c3-s2*s3)+a2*s2+a3*c2*s3+a3*c3*s2))+(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)*
(m5*(d4*(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)-m5*x5*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-
c2*c3*s1))+m5*z5*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+m5*y5*s4*(s1*s2*s3-c2*c3*s1))+(c2*s3+c3*s2)*
((c2*c3-s2*s3)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))+I44*c4*(c2*s3+c3*s2)-I45*s4*(c2*s3+c3*s2)+m4*y4*(-d4*
(c2*c3-s2*s3)+a2*s2+a3*c2*s3+a3*c3*s2))-(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)*(m5*(-d4*(c2*c3-
s2*s3)+a2*s2+a3*c2*s3+a3*c3*s2)+m5*x5*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-m5*z5*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))-m5*y5*s4*(c2*s3+c3*s2))-(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-
a3*s2*s3)*(m6*(-d4*(c2*c3-s2*s3)-d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+a2*s2+a3*c2*s3+a3*c3*s2)-m6*z6*(c5*
(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+m6*x6*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))-m6*y6*
(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2)))+(c1*c2*s3+c1*c3*s2)*(I36*
(c1*c2*s3+c1*c3*s2)+(c1*c2*c3-c1*s2*s3)*(I31*(-1.0/2.0)+I32*(1.0/2.0)+I33*(1.0/2.0))+m3*x3*(a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3))-(c1*c2*c3-c1*s2*s3)*(I36*(c1*c2*c3-c1*s2*s3)+(c1*c2*s3+c1*c3*s2)*(I31*(1.0/2.0)+I32*(1.0/2.0)-I33*
(1.0/2.0))+m3*z3*(a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(a3*c2*s1*s3+a3*c3*s1*s2)*(m3*(a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3)-m3*x3*(s1*s2*s3-c2*c3*s1)+m3*z3*(c2*s1*s3+c3*s1*s2))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*((c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+I65*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-I66*(c6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(a3*c2*c3-a3*s2*s3)*(m3*
(a2*s2+a3*c2*s3+a3*c3*s2)+m3*x3*(c2*s3+c3*s2)-m3*z3*(c2*c3-s2*s3))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I66*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+I64*(s6*
(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*
(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(s6*
(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I65*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))-I64*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+(s6*
(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-
c2*c3*s1))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*
(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+I66*
(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*
(c1*c2*c3-c1*s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*
(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(-I64*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-
c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+I65*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+(s6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*y6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*((c5*(c2*c3-s2*s3)-
c4*s5*(c2*s3+c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-I66*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))-m6*z6*(-
d4*(c2*c3-s2*s3)-d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+a2*s2+a3*c2*s3+a3*c3*s2))-(c5*(c1*c2*c3-c1*s2*s3)-
c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))-
I66*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(d6*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(d4*
(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)*(m4*(-d4*(c2*c3-s2*s3)+a2*s2+a3*c2*s3+a3*c3*s2)+m4*y4*(c2*c3-s2*s3)+m4*x4*c4*
(c2*s3+c3*s2)-m4*z4*s4*(c2*s3+c3*s2))-(m6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-m6*x6*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-
c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+m6*y6*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3
-c1*s2*s3))+m6*z6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3)))*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)+(m6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+m6*z6*(c5*(c2*s1*s3+c3*s1*s2)-
c4*s5*(s1*s2*s3-c2*c3*s1))-m6*x6*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-
c2*c3*s1))+m6*y6*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1)))*(d4*(s1*s2*s3-
c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)+(s1*s2*s3-c2*c3*s1)*
((c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)-I42*(1.0/2.0)+I43*(1.0/2.0))+I44*c4*(s1*s2*s3-c2*c3*s1)-I45*s4*(s1*s2*s3-
c2*c3*s1)-m4*y4*(d4*(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*
(c1*c2*c3-c1*s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-I54*s4*(c1*c2*c3-c1*s2*s3))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*
(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+I55*s4*(c1*c2*c3-c1*s2*s3))+(a3*c1*c2*s3+a3*c1*c3*s2)*(m3*
(a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m3*x3*(c1*c2*c3-c1*s2*s3)+m3*z3*(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c2*s3+c3*s2)-
c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I66*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+I64*(s6*(s5*(c2*c3-
s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*
(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-m6*x6*(-d4*(c2*c3-s2*s3)-d6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))+a2*s2+a3*c2*s3+a3*c3*s2))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I65*(c5*
(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-I64*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+(s6*(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-m6*y6*(-d4*
(c2*c3-s2*s3)-d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+a2*s2+a3*c2*s3+a3*c3*s2))-c4*(c2*c3-s2*s3)*(I44*(c2*c3-
s2*s3)+c4*(c2*s3+c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I46*s4*(c2*s3+c3*s2)+m4*x4*(-d4*(c2*c3-
s2*s3)+a2*s2+a3*c2*s3+a3*c3*s2))+s4*(c2*c3-s2*s3)*(I45*(c2*c3-s2*s3)+I46*c4*(c2*s3+c3*s2)-s4*(c2*s3+c3*s2)*(I41*
(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+m4*z4*(-d4*(c2*c3-s2*s3)+a2*s2+a3*c2*s3+a3*c3*s2))-c4*(c2*s1*s3+c3*s1*s2)*
(I44*(c2*s1*s3+c3*s1*s2)+c4*(s1*s2*s3-c2*c3*s1)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I46*s4*(s1*s2*s3-
c2*c3*s1)-m4*x4*(d4*(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+s4*(c2*s1*s3+c3*s1*s2)*(I45*
(c2*s1*s3+c3*s1*s2)+I46*c4*(s1*s2*s3-c2*c3*s1)-s4*(s1*s2*s3-c2*c3*s1)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-
m4*z4*(d4*(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+s4*(c2*c3-s2*s3)*(I54*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))-I55*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-s4*(c2*s3+c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*
(1.0/2.0))+m5*y5*(-d4*(c2*c3-s2*s3)+a2*s2+a3*c2*s3+a3*c3*s2))-c4*(c1*c2*s3+c1*c3*s2)*(I44*(c1*c2*s3+c1*c3*s2)-
m4*x4*(d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-c4*(c1*c2*c3-c1*s2*s3)*(I41*(-1.0/2.0)+I42*
(1.0/2.0)+I43*(1.0/2.0))+I46*s4*(c1*c2*c3-c1*s2*s3))+s4*(c1*c2*s3+c1*c3*s2)*(I54*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*
(c1*c2*c3-c1*s2*s3))-I55*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-m5*y5*(d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+s4*(c1*c2*c3-c1*s2*s3)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*
(1.0/2.0)))+s4*(c1*c2*s3+c1*c3*s2)*(I45*(c1*c2*s3+c1*c3*s2)-m4*z4*(d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3)-I46*c4*(c1*c2*c3-c1*s2*s3)+s4*(c1*c2*c3-c1*s2*s3)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))-s4*
(c2*s1*s3+c3*s1*s2)*(-I54*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+I55*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))+s4*(s1*s2*s3-c2*c3*s1)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+m5*y5*(d4*
(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3));;
        C333= 0;
        C344=   -(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-c5*s6*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+I66*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c5*c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1)))+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))-c5*s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c5*c6*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+d6*m6*x6*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(m5*y5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+m5*x5*c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+m5*z5*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-
(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+c5*c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-c5*s6*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+d6*m6*y6*s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I64*(c6*s4*
(c2*s3+c3*s2)+c4*c5*s6*(c2*s3+c3*s2))+(s4*s6*(c2*s3+c3*s2)-c4*c5*c6*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))-I66*c4*s5*(c2*s3+c3*s2)-d6*m6*x6*c4*s5*(c2*s3+c3*s2))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-
s2*s3))-c6*s4*(c2*c3-s2*s3))*(I64*(s4*s6*(c2*s3+c3*s2)-c4*c5*c6*(c2*s3+c3*s2))+(c6*s4*(c2*s3+c3*s2)+c4*c5*s6*
(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*c4*s5*(c2*s3+c3*s2)-d6*m6*y6*c4*s5*
(c2*s3+c3*s2))+(m4*x4*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+m4*z4*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(-d4*(c1*c2*c3-
c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-(-m5*y5*s4*(c2*s3+c3*s2)+m5*x5*c4*c5*(c2*s3+c3*s2)+m5*z5*c4*s5*(c2*s3+c3*s2))*
(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)+(c1*c2*c3-c1*s2*s3)*(I44*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+I45*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))-(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)*(m6*x6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c5*c6*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1)))+m6*y6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-c5*s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+d6*m6*s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+m6*z6*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-c5*s6*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c5*c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))+I66*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+d6*m6*x6*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))-(s6*
(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+c5*c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-c5*s6*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+d6*m6*y6*s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-c5*s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+I66*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c5*c6*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3)))+s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3)))+(s1*s2*s3-c2*c3*s1)*(I44*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+I45*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-
(m6*x6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c5*c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+m6*y6*(c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-c5*s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+d6*m6*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+m6*z6*s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3)))*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-
a3*c1*c3*s2)+(I44*c4*(c2*s3+c3*s2)-I45*s4*(c2*s3+c3*s2))*(c2*s3+c3*s2)+(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))*(I54*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))
*(I55*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I56*c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(-I55*s4*(c2*s3+c3*s2)+c4*s5*
(c2*s3+c3*s2)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I56*c4*c5*(c2*s3+c3*s2))+(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-
s2*s3))*(-I54*s4*(c2*s3+c3*s2)+c4*c5*(c2*s3+c3*s2)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*c4*s5*
(c2*s3+c3*s2))-(m4*x4*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+m4*z4*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(d4*(s1*s2*s3-
c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)*(m5*y5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+m5*x5*c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+m5*z5*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(d4*(c2*s3+c3*s2)+d6*
(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3)*(m6*x6*(s4*s6*(c2*s3+c3*s2)-c4*c5*c6*
(c2*s3+c3*s2))+m6*y6*(c6*s4*(c2*s3+c3*s2)+c4*c5*s6*(c2*s3+c3*s2))-d6*m6*c4*s5*(c2*s3+c3*s2)-m6*z6*c4*s5*
(c2*s3+c3*s2))-(m4*x4*c4*(c2*s3+c3*s2)-m4*z4*s4*(c2*s3+c3*s2))*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)+(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I65*(c6*s4*(c2*s3+c3*s2)+c4*c5*s6*(c2*s3+c3*s2))+I66*(s4*s6*(c2*s3+c3*s2)-
c4*c5*c6*(c2*s3+c3*s2))-c4*s5*(c2*s3+c3*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-d6*m6*z6*c4*s5*
(c2*s3+c3*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I55*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I56*c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I54*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+c4*
(c1*c2*s3+c1*c3*s2)*(I46*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I41*(-1.0/2.0)+I42*
(1.0/2.0)+I43*(1.0/2.0)))-s4*(c1*c2*s3+c1*c3*s2)*(I46*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+s4*(c2*c3-s2*s3)*(-s4*(c2*s3+c3*s2)*(I51*(1.0/2.0)-I52*
(1.0/2.0)+I53*(1.0/2.0))+I54*c4*c5*(c2*s3+c3*s2)+I55*c4*s5*(c2*s3+c3*s2))-c4*(c2*s1*s3+c3*s1*s2)*(I46*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))-c4*(c2*c3-s2*s3)
*(c4*(c2*s3+c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I46*s4*(c2*s3+c3*s2))+s4*(c2*s1*s3+c3*s1*s2)*(I46*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+s4*
(c2*c3-s2*s3)*(I46*c4*(c2*s3+c3*s2)-s4*(c2*s3+c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))-s4*
(c1*c2*s3+c1*c3*s2)*((c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+I55*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+s4*(c2*s1*s3+c3*s1*s2)*((c1*c4-s4*(s1*s2*s3-c2*c3*s1))
*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+I55*s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1)));;
        C355=   (c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*((c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(1.0/2.0)+I62*
(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+I65*s6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))+d6*m6*z6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))+(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))
*(I56*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))*(I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(m5*x5*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+m5*z5*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))*(-d4*
(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*
(c1*c2*s3+c1*c3*s2))*(I66*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+c6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))-(s6*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I65*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+I64*c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-s6*(c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*((s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-I65*s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))-(c5*(c1*c2*c3-c1*s2*s3)-
c4*s5*(c1*c2*s3+c1*c3*s2))*((s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*
(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-I65*s6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))-
(d6*m6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*z6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+m6*x6*c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-m6*y6*s6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-
c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))*(I66*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+c6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))-(s6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I65*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*(c2*s1*s3+c3*s1*s2))+I64*c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-s6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*
(I66*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+d6*m6*x6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2)))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I65*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))-I64*c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))-(I56*(s5*(c2*c3-
s2*s3)+c4*c5*(c2*s3+c3*s2))-(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))*
(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-(I56*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-(m5*x5*(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-m5*z5*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))*(d4*(c2*s3+c3*s2)+a3*c2*c3-
a3*s2*s3)-(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)*
(d6*m6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+m6*z6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+m6*x6*c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-m6*y6*s6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3
-a3*s2*s3)*(m6*z6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+d6*m6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-m6*x6*c6*
(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+m6*y6*s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)))-(I56*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+(I56*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-(m5*x5*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+m5*z5*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))*
(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)+s4*(I54*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-I55*(c5*(c2*c3-
s2*s3)-c4*s5*(c2*s3+c3*s2)))*(c2*c3-s2*s3)-s4*(c1*c2*s3+c1*c3*s2)*(I54*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))+I55*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))+s4*(I54*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+I55*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))*
(c2*s1*s3+c3*s1*s2);;
        C366=   (c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I65*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*
(c2*s3+c3*s2))-I66*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2)))-(c6*(s5*(c2*s3+c3*s2)-c4*c5*
(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I64*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-(c6*(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(s6*(s5*
(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I64*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))-(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0)))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+I66*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))-(m6*x6*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))-m6*y6*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2)))*(d4*(c2*s3+c3*s2)+d6*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3)+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3
-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3
-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(m6*x6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*
(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+m6*y6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)-(m6*x6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3
-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*y6*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-
c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(c4*s1-s4*(c1*c2*c3
-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+I66*(s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3
-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3
-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)));;
        C312= 0;
        C313= 0;
        C314=   -(m4*x4*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m4*z4*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))*(-d4*(c1*c2*c3-
c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-(-m5*y5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+m5*x5*c5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+m5*z5*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-(c1*c2*c3-
c1*s2*s3)*(I44*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-I45*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(s1*s2*s3-c2*c3*s1)*(I44*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3))-I45*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))-(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)*(-
m5*y5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+m5*x5*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+m5*z5*s5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))-(m4*x4*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m4*z4*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(d4*(s1*s2*s3-
c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(s1*s4+c4*(c1*c2*c3
-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+I66*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3
-c1*s2*s3)))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-d6*m6*z6*s5*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(-I54*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1)))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(-I55*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+I56*c5*(c1*c4-
s4*(s1*s2*s3-c2*c3*s1))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))-(c5*
(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(-I55*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+I56*c5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))*(-I54*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1)))+(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-d6*m6*x6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c6*
(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*x6*s5*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3)))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*
(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-
d6*m6*y6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*
(c2*s1*s3+c3*s1*s2))*(I64*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c6*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*s5*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*y6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))*(I65*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+I66*(s6*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I61*
(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-d6*m6*z6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(m6*x6*(s6*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+m6*y6*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1)))-d6*m6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m6*z6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(d6*(c5*
(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)+(m6*x6*(s6*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+m6*y6*(c6*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-d6*m6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m6*z6*s5*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)+c4*(c1*c2*s3+c1*c3*s2)*(I46*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-(c1*c4-
s4*(s1*s2*s3-c2*c3*s1))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))+c4*(c2*s1*s3+c3*s1*s2)*(I46*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))+s4*
(c1*c2*s3+c1*c3*s2)*(I46*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I41*(1.0/2.0)+I42*
(1.0/2.0)-I43*(1.0/2.0)))+s4*(c2*s1*s3+c3*s1*s2)*(I46*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+s4*(c2*s1*s3+c3*s1*s2)*(-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*
(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I55*s5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))+s4*(c1*c2*s3+c1*c3*s2)*(-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*
(1.0/2.0))+I54*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I55*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)));;
        C315=   -(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I66*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I65*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))-I64*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+s6*(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*((c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+I65*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*
(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)*(d6*m6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))+m6*z6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-m6*x6*c6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+m6*y6*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2)))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*
(1.0/2.0)-I53*(1.0/2.0)))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0)))+(m5*x5*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))-m5*z5*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*((c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I61*
(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+I65*s6*
(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*
(1.0/2.0)-I53*(1.0/2.0)))+(m5*x5*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m5*z5*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+(c6*(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I66*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))-c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*
(c2*s1*s3+c3*s1*s2))*(I65*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-I64*c6*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-
(d6*m6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+m6*z6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))-m6*x6*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*y6*s6*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)-s4*(c2*s1*s3+c3*s1*s2)*(I54*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))-I55*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-s4*(I54*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-I55*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*
(c1*c2*s3+c1*c3*s2);;
        C316=   (I65*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))-I66*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))))*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-(I64*(s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*
(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(c6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))-(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-
s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))*(s6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(m6*x6*(c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-m6*y6*(s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*
(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)+(m6*x6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-
s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-m6*y6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))*(I65*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))-I66*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(c6*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(s6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0)))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*
(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(c6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0)));;
        C323= 0;
        C324=   -(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)*(m5*y5*c4*(c2*s1*s3+c3*s1*s2)+m5*x5*c5*s4*
(c2*s1*s3+c3*s1*s2)+m5*z5*s4*s5*(c2*s1*s3+c3*s1*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*
(c1*c2*s3+c1*c3*s2))*(I64*(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))+(c4*s6*
(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*
(c1*c2*s3+c1*c3*s2)+d6*m6*x6*s4*s5*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*
(c1*c2*s3+c1*c3*s2))+(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+I65*s4*s5*(c1*c2*s3+c1*c3*s2)+d6*m6*y6*s4*s5*(c1*c2*s3+c1*c3*s2))-(m5*y5*c4*(c2*c3-s2*s3)+m5*x5*c5*s4*
(c2*c3-s2*s3)+m5*z5*s4*s5*(c2*c3-s2*s3))*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(I65*(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))+I66*(c4*s6*
(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+s4*s5*(c2*s1*s3+c3*s1*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+d6*m6*z6*s4*s5*(c2*s1*s3+c3*s1*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(c4*c6*
(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))+I66*(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*
(c1*c2*s3+c1*c3*s2))+s4*s5*(c1*c2*s3+c1*c3*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s4*s5*
(c1*c2*s3+c1*c3*s2))+(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I64*(c4*c6*(c2*c3-s2*s3)-
c5*s4*s6*(c2*c3-s2*s3))+(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+I66*s4*s5*(c2*c3-s2*s3)+d6*m6*x6*s4*s5*(c2*c3-s2*s3))-(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*
(c2*c3-s2*s3))*(I64*(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))+(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-s2*s3))*
(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c2*c3-s2*s3)+d6*m6*y6*s4*s5*(c2*c3-s2*s3))+(m6*x6*(c4*s6*
(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))+m6*y6*(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*
(c1*c2*s3+c1*c3*s2))+d6*m6*s4*s5*(c1*c2*s3+c1*c3*s2)+m6*z6*s4*s5*(c1*c2*s3+c1*c3*s2))*(d6*(c5*(c1*c2*c3-c1*s2*s3)-
c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*
(c2*s1*s3+c3*s1*s2))+(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+I66*s4*s5*(c2*s1*s3+c3*s1*s2)+d6*m6*x6*s4*s5*(c2*s1*s3+c3*s1*s2))-(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*
(c2*s1*s3+c3*s1*s2))+(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+I65*s4*s5*(c2*s1*s3+c3*s1*s2)+d6*m6*y6*s4*s5*(c2*s1*s3+c3*s1*s2))-(m4*z4*c4*
(c1*c2*s3+c1*c3*s2)+m4*x4*s4*(c1*c2*s3+c1*c3*s2))*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-(c1*c2*c3-
c1*s2*s3)*(I45*c4*(c1*c2*s3+c1*c3*s2)+I44*s4*(c1*c2*s3+c1*c3*s2))+(I45*c4*(c2*c3-s2*s3)+I44*s4*(c2*c3-s2*s3))*
(c2*s3+c3*s2)-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I55*c4*(c2*s1*s3+c3*s1*s2)+I56*c5*s4*
(c2*s1*s3+c3*s1*s2)+s4*s5*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I54*c4*(c2*s1*s3+c3*s1*s2)+c5*s4*(c2*s1*s3+c3*s1*s2)*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*(c2*s1*s3+c3*s1*s2))-(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)*(m6*x6*(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*
(c2*s1*s3+c3*s1*s2))+m6*y6*(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))+d6*m6*s4*s5*
(c2*s1*s3+c3*s1*s2)+m6*z6*s4*s5*(c2*s1*s3+c3*s1*s2))-(m5*y5*c4*(c1*c2*s3+c1*c3*s2)+m5*x5*c5*s4*
(c1*c2*s3+c1*c3*s2)+m5*z5*s4*s5*(c1*c2*s3+c1*c3*s2))*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I55*c4*(c2*c3-s2*s3)+s4*s5*(c2*c3-s2*s3)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0))+I56*c5*s4*(c2*c3-s2*s3))+(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I54*c4*(c2*c3-s2*s3)+c5*s4*(c2*c3-
s2*s3)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*(c2*c3-s2*s3))-(m6*x6*(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*
(c2*c3-s2*s3))+m6*y6*(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-s2*s3))+d6*m6*s4*s5*(c2*c3-s2*s3)+m6*z6*s4*s5*(c2*c3-
s2*s3))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3)-(m4*z4*c4*(c2*c3-
s2*s3)+m4*x4*s4*(c2*c3-s2*s3))*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)+(I45*c4*(c2*s1*s3+c3*s1*s2)+I44*s4*
(c2*s1*s3+c3*s1*s2))*(s1*s2*s3-c2*c3*s1)-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I54*c4*
(c1*c2*s3+c1*c3*s2)+c5*s4*(c1*c2*s3+c1*c3*s2)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*
(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I55*c4*(c1*c2*s3+c1*c3*s2)+I56*c5*s4*
(c1*c2*s3+c1*c3*s2)+s4*s5*(c1*c2*s3+c1*c3*s2)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))-(m4*z4*c4*
(c2*s1*s3+c3*s1*s2)+m4*x4*s4*(c2*s1*s3+c3*s1*s2))*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I65*(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-s2*s3))+I66*(c4*s6*(c2*c3-
s2*s3)+c5*c6*s4*(c2*c3-s2*s3))+s4*s5*(c2*c3-s2*s3)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s4*s5*
(c2*c3-s2*s3))+s4*(c2*c3-s2*s3)*(c4*(c2*c3-s2*s3)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I55*s4*s5*(c2*c3-
s2*s3)+I54*c5*s4*(c2*c3-s2*s3))-c4*(c1*c2*s3+c1*c3*s2)*(I46*c4*(c1*c2*s3+c1*c3*s2)+s4*(c1*c2*s3+c1*c3*s2)*(I41*(-
1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))-c4*(c2*c3-s2*s3)*(I46*c4*(c2*c3-s2*s3)+s4*(c2*c3-s2*s3)*(I41*(-
1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))+s4*(c2*s1*s3+c3*s1*s2)*(c4*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)-I52*
(1.0/2.0)+I53*(1.0/2.0))+I54*c5*s4*(c2*s1*s3+c3*s1*s2)+I55*s4*s5*(c2*s1*s3+c3*s1*s2))+s4*(c1*c2*s3+c1*c3*s2)*(c4*
(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+I46*s4*(c1*c2*s3+c1*c3*s2))+s4*(c2*c3-s2*s3)*(c4*
(c2*c3-s2*s3)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+I46*s4*(c2*c3-s2*s3))-c4*(I46*c4*(c2*s1*s3+c3*s1*s2)+s4*
(c2*s1*s3+c3*s1*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))*(c2*s1*s3+c3*s1*s2)+s4*(c4*(c2*s1*s3+c3*s1*s2)*
(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+I46*s4*(c2*s1*s3+c3*s1*s2))*(c2*s1*s3+c3*s1*s2)+s4*(c1*c2*s3+c1*c3*s2)
*(c4*(c1*c2*s3+c1*c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*s4*(c1*c2*s3+c1*c3*s2)+I55*s4*s5*
(c1*c2*s3+c1*c3*s2));;
        C325=   (c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I66*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))-(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*
(c2*s1*s3+c3*s1*s2))*(I65*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+I64*c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))-s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+d6*m6*y6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*
((s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-I65*s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+d6*m6*z6*(s5*(c2*s3+c3*s2)-c4*c5*
(c2*c3-s2*s3)))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0)))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0)))-(m5*x5*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+m5*z5*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)*(d6*m6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+m6*z6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+m6*x6*c6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m6*y6*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2)))+(c6*(s5*
(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I66*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+c6*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c2*s3+c3*s2)+c4*s5*
(c2*c3-s2*s3))+d6*m6*x6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))-(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*
(c2*c3-s2*s3))*(I65*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+I64*c6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-s6*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(c2*s3+c3*s2)-c4*c5*
(c2*c3-s2*s3)))+(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0)))+(m5*x5*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+m5*z5*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2)))*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-(I56*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-
s2*s3))+(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+(I56*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-
s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-(m5*x5*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+m5*z5*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))*(d4*(c2*s3+c3*s2)+a3*c2*c3-
a3*s2*s3)+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I66*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*
(c1*c2*s3+c1*c3*s2))*(I65*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+I64*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))-s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+d6*m6*y6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))*((s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+I66*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-I65*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))-(d6*m6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+m6*z6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+m6*x6*c6*(c5*
(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-m6*y6*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2)))*(d6*
(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)-(d4*
(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3)*(m6*z6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-
s2*s3))+d6*m6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+m6*x6*c6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-m6*y6*s6*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3)))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*((s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))-I65*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2)))+s4*(c2*s1*s3+c3*s1*s2)*(I54*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I55*
(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))+s4*(I54*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+I55*(s5*
(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))*(c2*c3-s2*s3)-s4*(c1*c2*s3+c1*c3*s2)*(I54*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+I55*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)));;
        C326=   (c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I65*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*
(c2*c3-s2*s3))+I66*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3)))+(m6*x6*(s6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+m6*y6*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2)))*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*
(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+I66*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+I66*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2)))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*
(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+(c6*
(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0)))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(c6*(s5*
(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I64*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*
(c2*c3-s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0)))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I64*(s6*(s5*(c2*s3+c3*s2)-c4*c5*
(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))+(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))*(I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*
(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0)))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))+(m6*x6*(s6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))+m6*y6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)+(m6*x6*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-
s2*s3))+m6*y6*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3)))*(d4*(c2*s3+c3*s2)+d6*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3);;
        C334=   -(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)*(m5*y5*c4*(c2*s1*s3+c3*s1*s2)+m5*x5*c5*s4*
(c2*s1*s3+c3*s1*s2)+m5*z5*s4*s5*(c2*s1*s3+c3*s1*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*
(c1*c2*s3+c1*c3*s2))*(I64*(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))+(c4*s6*
(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*
(c1*c2*s3+c1*c3*s2)+d6*m6*x6*s4*s5*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*
(c1*c2*s3+c1*c3*s2))+(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+I65*s4*s5*(c1*c2*s3+c1*c3*s2)+d6*m6*y6*s4*s5*(c1*c2*s3+c1*c3*s2))-(m5*y5*c4*(c2*c3-s2*s3)+m5*x5*c5*s4*
(c2*c3-s2*s3)+m5*z5*s4*s5*(c2*c3-s2*s3))*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(I65*(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))+I66*(c4*s6*
(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+s4*s5*(c2*s1*s3+c3*s1*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+d6*m6*z6*s4*s5*(c2*s1*s3+c3*s1*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(c4*c6*
(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))+I66*(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*
(c1*c2*s3+c1*c3*s2))+s4*s5*(c1*c2*s3+c1*c3*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s4*s5*
(c1*c2*s3+c1*c3*s2))+(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I64*(c4*c6*(c2*c3-s2*s3)-
c5*s4*s6*(c2*c3-s2*s3))+(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+I66*s4*s5*(c2*c3-s2*s3)+d6*m6*x6*s4*s5*(c2*c3-s2*s3))-(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*
(c2*c3-s2*s3))*(I64*(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))+(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-s2*s3))*
(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c2*c3-s2*s3)+d6*m6*y6*s4*s5*(c2*c3-s2*s3))+(m6*x6*(c4*s6*
(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))+m6*y6*(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*
(c1*c2*s3+c1*c3*s2))+d6*m6*s4*s5*(c1*c2*s3+c1*c3*s2)+m6*z6*s4*s5*(c1*c2*s3+c1*c3*s2))*(d6*(c5*(c1*c2*c3-c1*s2*s3)-
c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*
(c2*s1*s3+c3*s1*s2))+(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+I66*s4*s5*(c2*s1*s3+c3*s1*s2)+d6*m6*x6*s4*s5*(c2*s1*s3+c3*s1*s2))-(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*
(c2*s1*s3+c3*s1*s2))+(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+I65*s4*s5*(c2*s1*s3+c3*s1*s2)+d6*m6*y6*s4*s5*(c2*s1*s3+c3*s1*s2))-(m4*z4*c4*
(c1*c2*s3+c1*c3*s2)+m4*x4*s4*(c1*c2*s3+c1*c3*s2))*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-(c1*c2*c3-
c1*s2*s3)*(I45*c4*(c1*c2*s3+c1*c3*s2)+I44*s4*(c1*c2*s3+c1*c3*s2))+(I45*c4*(c2*c3-s2*s3)+I44*s4*(c2*c3-s2*s3))*
(c2*s3+c3*s2)-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I55*c4*(c2*s1*s3+c3*s1*s2)+I56*c5*s4*
(c2*s1*s3+c3*s1*s2)+s4*s5*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I54*c4*(c2*s1*s3+c3*s1*s2)+c5*s4*(c2*s1*s3+c3*s1*s2)*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*(c2*s1*s3+c3*s1*s2))-(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)*(m6*x6*(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*
(c2*s1*s3+c3*s1*s2))+m6*y6*(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))+d6*m6*s4*s5*
(c2*s1*s3+c3*s1*s2)+m6*z6*s4*s5*(c2*s1*s3+c3*s1*s2))-(m5*y5*c4*(c1*c2*s3+c1*c3*s2)+m5*x5*c5*s4*
(c1*c2*s3+c1*c3*s2)+m5*z5*s4*s5*(c1*c2*s3+c1*c3*s2))*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I55*c4*(c2*c3-s2*s3)+s4*s5*(c2*c3-s2*s3)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0))+I56*c5*s4*(c2*c3-s2*s3))+(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I54*c4*(c2*c3-s2*s3)+c5*s4*(c2*c3-
s2*s3)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*(c2*c3-s2*s3))-(m6*x6*(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*
(c2*c3-s2*s3))+m6*y6*(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-s2*s3))+d6*m6*s4*s5*(c2*c3-s2*s3)+m6*z6*s4*s5*(c2*c3-
s2*s3))*(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3)-(m4*z4*c4*(c2*c3-
s2*s3)+m4*x4*s4*(c2*c3-s2*s3))*(d4*(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)+(I45*c4*(c2*s1*s3+c3*s1*s2)+I44*s4*
(c2*s1*s3+c3*s1*s2))*(s1*s2*s3-c2*c3*s1)-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I54*c4*
(c1*c2*s3+c1*c3*s2)+c5*s4*(c1*c2*s3+c1*c3*s2)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*
(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I55*c4*(c1*c2*s3+c1*c3*s2)+I56*c5*s4*
(c1*c2*s3+c1*c3*s2)+s4*s5*(c1*c2*s3+c1*c3*s2)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))-(m4*z4*c4*
(c2*s1*s3+c3*s1*s2)+m4*x4*s4*(c2*s1*s3+c3*s1*s2))*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I65*(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-s2*s3))+I66*(c4*s6*(c2*c3-
s2*s3)+c5*c6*s4*(c2*c3-s2*s3))+s4*s5*(c2*c3-s2*s3)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s4*s5*
(c2*c3-s2*s3))+s4*(c2*c3-s2*s3)*(c4*(c2*c3-s2*s3)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I55*s4*s5*(c2*c3-
s2*s3)+I54*c5*s4*(c2*c3-s2*s3))-c4*(c1*c2*s3+c1*c3*s2)*(I46*c4*(c1*c2*s3+c1*c3*s2)+s4*(c1*c2*s3+c1*c3*s2)*(I41*(-
1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))-c4*(c2*c3-s2*s3)*(I46*c4*(c2*c3-s2*s3)+s4*(c2*c3-s2*s3)*(I41*(-
1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))+s4*(c2*s1*s3+c3*s1*s2)*(c4*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)-I52*
(1.0/2.0)+I53*(1.0/2.0))+I54*c5*s4*(c2*s1*s3+c3*s1*s2)+I55*s4*s5*(c2*s1*s3+c3*s1*s2))+s4*(c1*c2*s3+c1*c3*s2)*(c4*
(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+I46*s4*(c1*c2*s3+c1*c3*s2))+s4*(c2*c3-s2*s3)*(c4*
(c2*c3-s2*s3)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+I46*s4*(c2*c3-s2*s3))-c4*(I46*c4*(c2*s1*s3+c3*s1*s2)+s4*
(c2*s1*s3+c3*s1*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0)))*(c2*s1*s3+c3*s1*s2)+s4*(c4*(c2*s1*s3+c3*s1*s2)*
(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+I46*s4*(c2*s1*s3+c3*s1*s2))*(c2*s1*s3+c3*s1*s2)+s4*(c1*c2*s3+c1*c3*s2)
*(c4*(c1*c2*s3+c1*c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*c5*s4*(c1*c2*s3+c1*c3*s2)+I55*s4*s5*
(c1*c2*s3+c1*c3*s2));;
        C335=   (c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I66*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))-(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*
(c2*s1*s3+c3*s1*s2))*(I65*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+I64*c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))-s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+d6*m6*y6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))-(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*
((s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-I65*s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+d6*m6*z6*(s5*(c2*s3+c3*s2)-c4*c5*
(c2*c3-s2*s3)))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0)))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0)))-(m5*x5*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+m5*z5*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)*(d6*m6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+m6*z6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+m6*x6*c6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m6*y6*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2)))+(c6*(s5*
(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I66*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+c6*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c2*s3+c3*s2)+c4*s5*
(c2*c3-s2*s3))+d6*m6*x6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))-(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*
(c2*c3-s2*s3))*(I65*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+I64*c6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-s6*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(c2*s3+c3*s2)-c4*c5*
(c2*c3-s2*s3)))+(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0)))+(m5*x5*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+m5*z5*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2)))*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-(I56*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-
s2*s3))+(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+(I56*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-
s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-(m5*x5*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+m5*z5*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))*(d4*(c2*s3+c3*s2)+a3*c2*c3-
a3*s2*s3)+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I66*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*
(c1*c2*s3+c1*c3*s2))*(I65*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+I64*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))-s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+d6*m6*y6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))*((s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+I66*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-I65*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))-(d6*m6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+m6*z6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+m6*x6*c6*(c5*
(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-m6*y6*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2)))*(d6*
(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)-(d4*
(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3)*(m6*z6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-
s2*s3))+d6*m6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+m6*x6*c6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-m6*y6*s6*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3)))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*((s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))-I65*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2)))+s4*(c2*s1*s3+c3*s1*s2)*(I54*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I55*
(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))+s4*(I54*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+I55*(s5*
(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))*(c2*c3-s2*s3)-s4*(c1*c2*s3+c1*c3*s2)*(I54*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+I55*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)));;
        C336=   (c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I65*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*
(c2*c3-s2*s3))+I66*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3)))+(m6*x6*(s6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+m6*y6*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2)))*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*
(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+I66*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+I66*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2)))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*
(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+(c6*
(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0)))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(c6*(s5*
(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I64*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*
(c2*c3-s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0)))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*(I64*(s6*(s5*(c2*s3+c3*s2)-c4*c5*
(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))+(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))*(I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*
(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0)))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))+(m6*x6*(s6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))+m6*y6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)+(m6*x6*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-
s2*s3))+m6*y6*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3)))*(d4*(c2*s3+c3*s2)+d6*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3);;
        C345=   (c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I61*
(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I65*s5*s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+d6*m6*z6*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I66*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-c6*s5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+d6*m6*x6*c5*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*
(I65*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-I64*c6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+s5*s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(c5*s4*(c2*s3+c3*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*s4*s5*
(c2*s3+c3*s2)+I65*s4*s5*s6*(c2*s3+c3*s2)+d6*m6*z6*c5*s4*(c2*s3+c3*s2))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-I56*s5*(c1*c4-
s4*(s1*s2*s3-c2*c3*s1)))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I56*c5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))+(m5*z5*c5*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))-m5*x5*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(d4*(s1*s2*s3-
c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)+(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)*(d6*m6*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m6*z6*c5*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))-m6*x6*c6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m6*y6*s5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c6*
(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I66*c5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-c6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s5*s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+d6*m6*x6*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I65*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-I64*c6*s5*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+s5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*c5*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))*(I66*c5*s4*
(c2*s3+c3*s2)-c6*s4*s5*(c2*s3+c3*s2)*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s4*s5*s6*
(c2*s3+c3*s2)+d6*m6*x6*c5*s4*(c2*s3+c3*s2))-(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*
(I65*c5*s4*(c2*s3+c3*s2)-I64*c6*s4*s5*(c2*s3+c3*s2)+s4*s5*s6*(c2*s3+c3*s2)*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+d6*m6*y6*c5*s4*(c2*s3+c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(c5*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*s5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+I65*s5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+d6*m6*z6*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(c5*s4*
(c2*s3+c3*s2)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-I56*s4*s5*(c2*s3+c3*s2))*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-
s2*s3))-(s4*s5*(c2*s3+c3*s2)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-I56*c5*s4*(c2*s3+c3*s2))*(s5*
(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-(m5*z5*c5*s4*(c2*s3+c3*s2)-m5*x5*s4*s5*(c2*s3+c3*s2))*(d4*
(c2*s3+c3*s2)+a3*c2*c3-a3*s2*s3)+(m6*z6*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+d6*m6*c5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+m6*y6*s5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m6*x6*c6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(d6*(c5*
(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)-(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I56*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*
(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(c5*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-I56*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(m5*z5*c5*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m5*x5*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(-d4*(c1*c2*c3-
c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-(d4*(c2*s3+c3*s2)+d6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3)*
(d6*m6*c5*s4*(c2*s3+c3*s2)+m6*z6*c5*s4*(c2*s3+c3*s2)-m6*x6*c6*s4*s5*(c2*s3+c3*s2)+m6*y6*s4*s5*s6*(c2*s3+c3*s2))-
s4*(c2*s1*s3+c3*s1*s2)*(I55*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-I54*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-s4*
(I54*s4*s5*(c2*s3+c3*s2)-I55*c5*s4*(c2*s3+c3*s2))*(c2*c3-s2*s3)+s4*(c1*c2*s3+c1*c3*s2)*(I55*c5*(c4*s1-s4*(c1*c2*c3
-c1*s2*s3))-I54*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)));;
        C346=   -(m6*x6*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-m6*y6*(s6*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))))*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)-(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*
(c2*c3-s2*s3))*(I64*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))-(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*
(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))*
(I64*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))-(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0)))-(m6*x6*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-m6*y6*
(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))))*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))*(I65*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-I66*(c6*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))-(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0)))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*
(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(m6*x6*(c4*c6*(c2*s3+c3*s2)-
c5*s4*s6*(c2*s3+c3*s2))-m6*y6*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2)))*(d4*(c2*s3+c3*s2)+d6*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3)+(I65*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1)))-I66*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))))*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+(I65*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))-I66*(c4*c6*(c2*s3+c3*s2)-
c5*s4*s6*(c2*s3+c3*s2)))*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-(I64*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))-(I64*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(s6*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))*(s6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2));;
        C356=   -(I65*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+I66*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))*
(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+(I64*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+s6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*
(c2*c3-s2*s3))-(c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(c5*
(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))+(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I65*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+I66*s6*
(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I64*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0)))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(c6*
(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))+(m6*y6*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+m6*x6*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))*(d4*(s1*s2*s3-
c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)-(m6*y6*c6*(c5*(c2*c3-
s2*s3)-c4*s5*(c2*s3+c3*s2))+m6*x6*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))*(d4*(c2*s3+c3*s2)+d6*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+a3*c2*c3-a3*s2*s3)+(m6*y6*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+m6*x6*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))*(d6*(c5*(c1*c2*c3-
c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)+(c5*(c1*c2*c3-c1*s2*s3)-
c4*s5*(c1*c2*s3+c1*c3*s2))*(I65*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+I66*s6*(s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*
(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))-
(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I64*c6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)));;
        C411=   (s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I64*(c6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+I66*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*x6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(c6*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*y6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*
(I46*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-
I44*(c1*c2*s3+c1*c3*s2)+m4*x4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(s1*s4+c4*(c1*c2*c3
-c1*s2*s3))*(I46*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*
(1.0/2.0))-I45*(c1*c2*s3+c1*c3*s2)+m4*z4*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(s6*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*
(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))-m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I44*
(c2*s1*s3+c3*s1*s2)+I46*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I41*(-1.0/2.0)+I42*
(1.0/2.0)+I43*(1.0/2.0))-m4*x4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))*(I45*(c2*s1*s3+c3*s1*s2)+I46*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*
(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-m4*z4*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*((c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*
(1.0/2.0))+I54*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+I55*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*(c2*s1*s3+c3*s1*s2))-m5*y5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(s1*s4+c4*(c1*c2*c3
-c1*s2*s3))*((c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+I54*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+I55*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m5*y5*
(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I65*(c6*(c4*s1
-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+I66*(s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(a1*c1+d6*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-s5*(c1*c4-
s4*(s1*s2*s3-c2*c3*s1))*(I65*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))+I66*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))-m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I54*(c1*c4-s4*(s1*s2*s3
-c2*c3*s1))+I56*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I55*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))+I56*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I54*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+I56*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(a1*c1+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I55*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(a1*c1+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(-m6*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3)+m6*x6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))+m6*y6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))+m6*z6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))-d6*s5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))*(m6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m6*x6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*y6*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*z6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)));;
        C422=   (s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I64*(s6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+I66*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*
(c1*c2*c3-c1*s2*s3))-(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))*(-I64*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+I65*
(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*
(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3
-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))
*(I66*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+I64*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-
(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-
m6*x6*(-d4*(c2*c3-s2*s3)-d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+a2*s2+a3*c2*s3+a3*c3*s2))-(c4*c6*(c2*s3+c3*s2)-
c5*s4*s6*(c2*s3+c3*s2))*(I65*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-I64*(c6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-m6*y6*(-d4*(c2*c3-s2*s3)-d6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))+a2*s2+a3*c2*s3+a3*c3*s2))-(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I44*(c2*s1*s3+c3*s1*s2)+c4*(s1*s2*s3-
c2*c3*s1)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I46*s4*(s1*s2*s3-c2*c3*s1)-m4*x4*(d4*
(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I45*
(c2*s1*s3+c3*s1*s2)+I46*c4*(s1*s2*s3-c2*c3*s1)-s4*(s1*s2*s3-c2*c3*s1)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))-
m4*z4*(d4*(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(-I54*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+I55*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+s4*(s1*s2*s3
-c2*c3*s1)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+m5*y5*(d4*(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3))-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I54*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-I55*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-m5*y5*(d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+s4*
(c1*c2*c3-c1*s2*s3)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I44*
(c1*c2*s3+c1*c3*s2)-m4*x4*(d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-c4*(c1*c2*c3-c1*s2*s3)*(I41*(-
1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))+I46*s4*(c1*c2*c3-c1*s2*s3))-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I45*
(c1*c2*s3+c1*c3*s2)-m4*z4*(d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-I46*c4*(c1*c2*c3-c1*s2*s3)+s4*
(c1*c2*c3-c1*s2*s3)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))-(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-
s4*(s1*s2*s3-c2*c3*s1)))*(I66*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+I64*(s6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*
(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c6*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I65*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-
c2*c3*s1))-I64*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+(s6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-
c2*c3*s1))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*((c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-
c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-I66*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-
c2*c3*s1))+m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-
c2*c3*s1))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+c4*(c2*s3+c3*s2)*(I45*(c2*c3-s2*s3)+I46*c4*(c2*s3+c3*s2)-s4*
(c2*s3+c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+m4*z4*(-d4*(c2*c3-s2*s3)+a2*s2+a3*c2*s3+a3*c3*s2))+s4*
(c2*s3+c3*s2)*(I44*(c2*c3-s2*s3)+c4*(c2*s3+c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I46*s4*
(c2*s3+c3*s2)+m4*x4*(-d4*(c2*c3-s2*s3)+a2*s2+a3*c2*s3+a3*c3*s2))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I65*(s6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))-I66*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*
(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I61*
(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I56*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I54*s4*(s1*s2*s3-c2*c3*s1)+m5*x5*(d4*
(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I56*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-I54*s4*
(c1*c2*c3-c1*s2*s3))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(-I56*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-
c2*c3*s1))+(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I55*s4*
(s1*s2*s3-c2*c3*s1)+m5*z5*(d4*(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+s5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))*(I56*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-
c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3)+I55*s4*(c1*c2*c3-c1*s2*s3))+c4*(c2*s3+c3*s2)*(I54*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-I55*(c5*
(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-s4*(c2*s3+c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+m5*y5*(-d4*(c2*c3-
s2*s3)+a2*s2+a3*c2*s3+a3*c3*s2))-d6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(m6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*
(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-m6*x6*(c6*(s5*(c1*c2*s3+c1*c3*s2)-
c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+m6*y6*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-
c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+m6*z6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3)))-s4*s5*
(c2*s3+c3*s2)*((c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-I66*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))-m6*z6*(-d4*(c2*c3-s2*s3)-d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+a2*s2+a3*c2*s3+a3*c3*s2))-c5*s4*
(c2*s3+c3*s2)*(I56*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I54*s4*(c2*s3+c3*s2)-m5*x5*(-d4*(c2*c3-
s2*s3)+a2*s2+a3*c2*s3+a3*c3*s2))+d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(m6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+m6*z6*(c5*(c2*s1*s3+c3*s1*s2)-
c4*s5*(s1*s2*s3-c2*c3*s1))-m6*x6*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-
c2*c3*s1))+m6*y6*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1)))+s4*s5*
(c2*s3+c3*s2)*(I56*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-I55*s4*(c2*s3+c3*s2)+m5*z5*(-d4*(c2*c3-
s2*s3)+a2*s2+a3*c2*s3+a3*c3*s2))+d6*s4*s5*(c2*s3+c3*s2)*(m6*(-d4*(c2*c3-s2*s3)-d6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))+a2*s2+a3*c2*s3+a3*c3*s2)-m6*z6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+m6*x6*(c6*(s5*(c2*c3-
s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))-m6*y6*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*
(c2*s3+c3*s2)));;
        C433=   -(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I44*(c2*s1*s3+c3*s1*s2)-m4*x4*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1
-a3*s1*s2*s3)+c4*(s1*s2*s3-c2*c3*s1)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I46*s4*(s1*s2*s3-
c2*c3*s1))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I45*(c2*s1*s3+c3*s1*s2)-m4*z4*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-
a3*s1*s2*s3)+I46*c4*(s1*s2*s3-c2*c3*s1)-s4*(s1*s2*s3-c2*c3*s1)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))-
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(-I54*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+I55*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+m5*y5*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+s4*(s1*s2*s3
-c2*c3*s1)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I54*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-I55*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-m5*y5*(d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)+s4*(c1*c2*c3-c1*s2*s3)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-
(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I66*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))+I64*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-(c6*
(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-
a3*s1*s2*s3))-(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I65*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-I64*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*
(s1*s2*s3-c2*c3*s1))+(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-
c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I44*(c1*c2*s3+c1*c3*s2)-m4*x4*(d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-c4*(c1*c2*c3-c1*s2*s3)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*
(1.0/2.0))+I46*s4*(c1*c2*c3-c1*s2*s3))-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I45*(c1*c2*s3+c1*c3*s2)-m4*z4*(d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-I46*c4*(c1*c2*c3-c1*s2*s3)+s4*(c1*c2*c3-c1*s2*s3)*(I41*(1.0/2.0)+I42*
(1.0/2.0)-I43*(1.0/2.0)))+(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I64*(s6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+I66*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*
(c1*c2*c3-c1*s2*s3))-(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))+(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))*(-I64*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+I65*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*
(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3
-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))-(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I66*
(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+I64*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-(c6*
(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*
(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2))-(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*
(c2*s3+c3*s2))*(I65*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-I64*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*y6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2))+c5*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))*(I56*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-
a3*s1*s2*s3)+I54*s4*(s1*s2*s3-c2*c3*s1))+c4*(c2*s3+c3*s2)*(I54*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-I55*(c5*
(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-s4*(c2*s3+c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+m5*y5*(-d4*(c2*c3-
s2*s3)+a3*c2*s3+a3*c3*s2))-c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I56*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-
c1*s2*s3))-(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*
(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-I54*s4*(c1*c2*c3-c1*s2*s3))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(-
I56*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I55*s4*(s1*s2*s3-
c2*c3*s1))+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I56*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)+I55*s4*(c1*c2*c3-c1*s2*s3))+c4*(c2*s3+c3*s2)*(I45*(c2*c3-
s2*s3)+I46*c4*(c2*s3+c3*s2)-s4*(c2*s3+c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+m4*z4*(-d4*(c2*c3-
s2*s3)+a3*c2*s3+a3*c3*s2))+s4*(c2*s3+c3*s2)*(I44*(c2*c3-s2*s3)+c4*(c2*s3+c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*
(1.0/2.0))-I46*s4*(c2*s3+c3*s2)+m4*x4*(-d4*(c2*c3-s2*s3)+a3*c2*s3+a3*c3*s2))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*
((c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-I66*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I65*(s6*(s5*(c1*c2*s3+c1*c3*s2)-
c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))-I66*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-
c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*
(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))-c5*s4*(c2*s3+c3*s2)*(I56*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I54*s4*(c2*s3+c3*s2)-m5*x5*(-d4*
(c2*c3-s2*s3)+a3*c2*s3+a3*c3*s2))+s4*s5*(c2*s3+c3*s2)*(I56*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-(c5*(c2*c3-
s2*s3)-c4*s5*(c2*s3+c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-I55*s4*(c2*s3+c3*s2)+m5*z5*(-d4*(c2*c3-
s2*s3)+a3*c2*s3+a3*c3*s2))+d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(m6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3)+m6*z6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))-m6*x6*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-
c2*c3*s1))+m6*y6*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1)))-s4*s5*
(c2*s3+c3*s2)*((c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-I66*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))+m6*z6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2))-d6*s5*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3))*(m6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-m6*x6*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*
(c1*c2*c3-c1*s2*s3))+m6*y6*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-
c1*s2*s3))+m6*z6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3)))-d6*s4*s5*(c2*s3+c3*s2)*(m6*(d4*(c2*c3-
s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2)+m6*z6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-
m6*x6*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+m6*y6*(s6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2)));;
        C444= 0;
        C455=   (I54*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+I55*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I54*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+I55*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2)))+(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I66*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*
(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))+(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1)))*(I65*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+I64*c6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))+(s6*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I66*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))+(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))*(I65*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+I64*c6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-
I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))-(c4*s6*
(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I66*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-c6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))+d6*m6*x6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))-(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*(I65*
(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-I64*c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+s6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))-s5*
(I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c4*(I54*(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-I55*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))*(c2*s3+c3*s2)-s5*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))*((s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+I66*c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-I65*s6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))-c5*(c1*c4-
s4*(s1*s2*s3-c2*c3*s1))*(I56*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))
*(I56*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*((s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-I65*s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))-c5*(I56*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+s4*s5*(I56*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))-(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))*(c2*s3+c3*s2)-
d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(d6*m6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+m6*z6*(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+m6*x6*c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))-m6*y6*s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-s4*s5*(c2*s3+c3*s2)*
((c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*(c2*c3-
s2*s3)+c4*c5*(c2*s3+c3*s2))+I65*s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+d6*m6*z6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2)))-d6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(d6*m6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+m6*z6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*x6*c6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-m6*y6*s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-
c5*s4*(I56*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0)))*(c2*s3+c3*s2)-d6*s4*s5*(c2*s3+c3*s2)*(m6*z6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))+d6*m6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-m6*x6*c6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))+m6*y6*s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2)));;
        C466=   (I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1
-s4*(c1*c2*c3-c1*s2*s3)))+(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I64*(s6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*(I64*(c6*(s5*(c2*c3-
s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))-(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*
(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))+(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1)))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))+(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4
-s4*(s1*s2*s3-c2*c3*s1)))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I65*(c6*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+I66*(s6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))-s5*(I65*(c6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+I66*(s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*s5*
(m6*x6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*y6*
(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))-s4*s5*(c2*s3+c3*s2)*(I65*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-I66*
(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2)))-d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(m6*x6*(s6*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+m6*y6*(c6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))+d6*s4*s5*(m6*x6*(c6*(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))-m6*y6*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*
(c2*s3+c3*s2)))*(c2*s3+c3*s2);;
        C412=   (s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I64*(s6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-m6*x6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1)))*(I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))-I65*(c5*
(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*
(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*
(I44*(s1*s2*s3-c2*c3*s1)-m4*x4*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-c4*(c2*s1*s3+c3*s1*s2)*
(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))+I46*s4*(c2*s1*s3+c3*s1*s2))+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I45*
(s1*s2*s3-c2*c3*s1)-m4*z4*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-I46*c4*(c2*s1*s3+c3*s1*s2)+s4*
(c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I54*(s5*(s1*s2*s3
-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-I55*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-
(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*
(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))-(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))*(I65*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-
c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*
(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*
(I54*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-I55*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+m5*y5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I51*
(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I44*(c1*c2*c3-c1*s2*s3)+m4*x4*(-d4*
(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+c4*(c1*c2*s3+c1*c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*
(1.0/2.0))-I46*s4*(c1*c2*s3+c1*c3*s2))-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I45*(c1*c2*c3-c1*s2*s3)+I46*c4*
(c1*c2*s3+c1*c3*s2)+m4*z4*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I41*
(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*((c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-I66*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))+m6*z6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I65*(s6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-I66*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*
(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-m6*z6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))+c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I56*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0))+m5*x5*(d4*(s1*s2*s3-c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-I54*s4*(c2*s1*s3+c3*s1*s2))-s5*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+I55*s4*(c2*s1*s3+c3*s1*s2))-c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I56*
(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+I54*s4*
(c1*c2*s3+c1*c3*s2))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-
(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(-d4*
(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-I55*s4*(c1*c2*s3+c1*c3*s2))+d6*s5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))*(m6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m6*x6*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*
(c1*c2*s3+c1*c3*s2))-m6*y6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-
m6*z6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2)))+d6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(m6*(d4*(s1*s2*s3-
c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+m6*z6*(c5*
(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m6*x6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))+m6*y6*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)));;
        C413=   (s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I54*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-I55*(c5*
(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m5*y5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)+s4*
(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3)))*(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-
c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))-(c6*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I65*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I64*(c6*
(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(s1*s2*s3-
c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))-(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))*(I54*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-I55*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+m5*y5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I51*
(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))+(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I44*(c1*c2*c3-c1*s2*s3)+m4*x4*(-d4*
(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+c4*(c1*c2*s3+c1*c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-
I46*s4*(c1*c2*s3+c1*c3*s2))-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I45*(c1*c2*c3-c1*s2*s3)+m4*z4*(-d4*(c1*c2*c3-
c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+I46*c4*(c1*c2*s3+c1*c3*s2)-s4*(c1*c2*s3+c1*c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-
I43*(1.0/2.0)))+(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I64*(s6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-
c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))+(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(-
I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+I65*(c5*(c1*c2*c3-c1*s2*s3)-
c4*s5*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-
c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))-(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I44*(s1*s2*s3-c2*c3*s1)-m4*x4*(d4*(s1*s2*s3-
c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-c4*(c2*s1*s3+c3*s1*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))+I46*s4*
(c2*s1*s3+c3*s1*s2))+(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I45*(s1*s2*s3-c2*c3*s1)-m4*z4*(d4*(s1*s2*s3-
c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-I46*c4*(c2*s1*s3+c3*s1*s2)+s4*(c2*s1*s3+c3*s1*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-
I43*(1.0/2.0)))+c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I56*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(s1*s2*s3-
c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-I54*s4*(c2*s1*s3+c3*s1*s2))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I65*(s6*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-I66*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*
(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-
c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0))-m5*z5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)+I55*s4*(c2*s1*s3+c3*s1*s2))-c5*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(-d4*(c1*c2*c3-
c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+I54*s4*(c1*c2*s3+c1*c3*s2))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I56*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(-d4*(c1*c2*c3-c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-I55*s4*
(c1*c2*s3+c1*c3*s2))+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*((c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*
(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*
(c2*s1*s3+c3*s1*s2))-I66*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*z6*
(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))+d6*s5*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(m6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)+m6*z6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m6*x6*(c6*
(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*y6*(s6*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))-d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(m6*(d6*(c5*(c1*c2*c3-
c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2)-m6*x6*(c6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+m6*y6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+m6*z6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2)));;
        C414=0;
        C415=   -(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I54*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))-
I55*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(I54*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))-I55*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I66*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*
(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))*(I65*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-I64*c6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I66*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))-c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1)))*(I65*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-I64*c6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-
I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-c5*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))*(I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))
*(I56*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*((c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+I65*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+c5*(I56*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s5*(I56*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*(c2*s1*s3+c3*s1*s2))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-
I53*(1.0/2.0)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*((c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+I65*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+d6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(d6*m6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))+m6*z6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-
m6*x6*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+m6*y6*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*(c2*s1*s3+c3*s1*s2)))-d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(d6*m6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))+m6*z6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-m6*x6*c6*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*y6*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)));;
        C416=   (I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4
-s4*(s1*s2*s3-c2*c3*s1)))-(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))-(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I64*(c6*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(s6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0)))+(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I64*(s6*(c1*c4
-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(c6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0)))-s5*(I65*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))-I66*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I65*(s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-I66*(c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))-d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*
(m6*x6*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-m6*y6*
(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))+d6*s5*(m6*x6*
(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-m6*y6*(s6*(c1*c4
-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3));;
        C423=   -(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I44*(c2*s1*s3+c3*s1*s2)-m4*x4*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1
-a3*s1*s2*s3)+c4*(s1*s2*s3-c2*c3*s1)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*(1.0/2.0))-I46*s4*(s1*s2*s3-
c2*c3*s1))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I45*(c2*s1*s3+c3*s1*s2)-m4*z4*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-
a3*s1*s2*s3)+I46*c4*(s1*s2*s3-c2*c3*s1)-s4*(s1*s2*s3-c2*c3*s1)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0)))-
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(-I54*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+I55*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+m5*y5*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+s4*(s1*s2*s3
-c2*c3*s1)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I54*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-I55*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-m5*y5*(d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)+s4*(c1*c2*c3-c1*s2*s3)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0)))-
(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I66*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))+I64*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-(c6*
(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-
a3*s1*s2*s3))-(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I65*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-I64*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*
(s1*s2*s3-c2*c3*s1))+(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-
c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3))+(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I44*(c1*c2*s3+c1*c3*s2)-m4*x4*(d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-c4*(c1*c2*c3-c1*s2*s3)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*
(1.0/2.0))+I46*s4*(c1*c2*c3-c1*s2*s3))-(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I45*(c1*c2*s3+c1*c3*s2)-m4*z4*(d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-I46*c4*(c1*c2*c3-c1*s2*s3)+s4*(c1*c2*c3-c1*s2*s3)*(I41*(1.0/2.0)+I42*
(1.0/2.0)-I43*(1.0/2.0)))+(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I64*(s6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+I66*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*
(c1*c2*c3-c1*s2*s3))-(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))+(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))*(-I64*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+I65*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*
(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3
-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))-(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I66*
(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+I64*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-(c6*
(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*
(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2))-(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*
(c2*s3+c3*s2))*(I65*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-I64*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*y6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2))+c5*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))*(I56*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-
a3*s1*s2*s3)+I54*s4*(s1*s2*s3-c2*c3*s1))+c4*(c2*s3+c3*s2)*(I54*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-I55*(c5*
(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-s4*(c2*s3+c3*s2)*(I51*(1.0/2.0)-I52*(1.0/2.0)+I53*(1.0/2.0))+m5*y5*(-d4*(c2*c3-
s2*s3)+a3*c2*s3+a3*c3*s2))-c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I56*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-
c1*s2*s3))-(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*
(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-I54*s4*(c1*c2*c3-c1*s2*s3))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(-
I56*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I55*s4*(s1*s2*s3-
c2*c3*s1))+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I56*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)+I55*s4*(c1*c2*c3-c1*s2*s3))+c4*(c2*s3+c3*s2)*(I45*(c2*c3-
s2*s3)+I46*c4*(c2*s3+c3*s2)-s4*(c2*s3+c3*s2)*(I41*(1.0/2.0)+I42*(1.0/2.0)-I43*(1.0/2.0))+m4*z4*(-d4*(c2*c3-
s2*s3)+a3*c2*s3+a3*c3*s2))+s4*(c2*s3+c3*s2)*(I44*(c2*c3-s2*s3)+c4*(c2*s3+c3*s2)*(I41*(-1.0/2.0)+I42*(1.0/2.0)+I43*
(1.0/2.0))-I46*s4*(c2*s3+c3*s2)+m4*x4*(-d4*(c2*c3-s2*s3)+a3*c2*s3+a3*c3*s2))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*
((c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-I66*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I65*(s6*(s5*(c1*c2*s3+c1*c3*s2)-
c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))-I66*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-
c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*
(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))-c5*s4*(c2*s3+c3*s2)*(I56*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I54*s4*(c2*s3+c3*s2)-m5*x5*(-d4*
(c2*c3-s2*s3)+a3*c2*s3+a3*c3*s2))+s4*s5*(c2*s3+c3*s2)*(I56*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-(c5*(c2*c3-
s2*s3)-c4*s5*(c2*s3+c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-I55*s4*(c2*s3+c3*s2)+m5*z5*(-d4*(c2*c3-
s2*s3)+a3*c2*s3+a3*c3*s2))+d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(m6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3)+m6*z6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))-m6*x6*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-
c2*c3*s1))+m6*y6*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1)))-s4*s5*
(c2*s3+c3*s2)*((c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-I66*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))+m6*z6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2))-d6*s5*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3))*(m6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-m6*x6*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*
(c1*c2*c3-c1*s2*s3))+m6*y6*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-
c1*s2*s3))+m6*z6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3)))-d6*s4*s5*(c2*s3+c3*s2)*(m6*(d4*(c2*c3-
s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2)+m6*z6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-
m6*x6*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+m6*y6*(s6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2)));;
        C424=0;
        C425=   (s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I66*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))+(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*
(I65*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+I64*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-
s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))+(I54*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+I55*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I66*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+c6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))+(c6*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I65*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+I64*c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2)))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I54*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I55*
(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))+(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I66*(s5*
(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+c6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))-I64*s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+d6*m6*x6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))+(c4*c6*
(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*(I65*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+I64*c6*(c5*(c2*s3+c3*s2)+c4*s5*
(c2*c3-s2*s3))-s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*
(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))+c4*(I54*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+I55*(s5*(c2*s3+c3*s2)-c4*c5*
(c2*c3-s2*s3)))*(c2*s3+c3*s2)-c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0)))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))-s5*(c4*s1-s4*(c1*c2*c3
-c1*s2*s3))*((s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+I66*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-I65*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))-s5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))*((s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*
(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I65*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))-c5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I56*(c5*
(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+s4*s5*(I56*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+(s5*(c2*s3+c3*s2)-c4*c5*
(c2*c3-s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))*(c2*s3+c3*s2)-d6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*
(d6*m6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+m6*z6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+m6*x6*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-m6*y6*s6*(c5*(c1*c2*c3-c1*s2*s3)-
c4*s5*(c1*c2*s3+c1*c3*s2)))-d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(d6*m6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+m6*z6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+m6*x6*c6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m6*y6*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2)))+s4*s5*
(c2*s3+c3*s2)*((s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-I65*s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+d6*m6*z6*(s5*(c2*s3+c3*s2)-c4*c5*
(c2*c3-s2*s3)))+c5*s4*(I56*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*(c2*s3+c3*s2)+d6*s4*s5*(c2*s3+c3*s2)*(m6*z6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-
s2*s3))+d6*m6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+m6*x6*c6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-m6*y6*s6*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3)));;
        C426=   -(I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))+(c6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0)))*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(I64*(c6*(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(s6*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I64*(c6*
(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*
(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*(I64*
(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))+(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*
(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+(c6*
(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0)))-(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I64*(c6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+s5*(I65*(c6*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+I66*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*
(I65*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+I66*(s6*(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))+d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(m6*x6*(s6*
(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))+m6*y6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2)))-s4*s5*(c2*s3+c3*s2)*(I65*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3
-s2*s3))+s4*s6*(c2*c3-s2*s3))+I66*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3)))+d6*s5*(m6*x6*
(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+m6*y6*(c6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*s4*s5*(m6*x6*
(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))+m6*y6*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-
s2*s3))+s4*s6*(c2*c3-s2*s3)))*(c2*s3+c3*s2);;
        C434= 0;
        C435=   (s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I66*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))+(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*
(I65*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+I64*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-
s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))+(I54*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+I55*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I66*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+c6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))+(c6*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I65*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+I64*c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2)))+(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I54*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I55*
(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))+(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I66*(s5*
(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+c6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))-I64*s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+d6*m6*x6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))+(c4*c6*
(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*(I65*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+I64*c6*(c5*(c2*s3+c3*s2)+c4*s5*
(c2*c3-s2*s3))-s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*
(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))+c4*(I54*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+I55*(s5*(c2*s3+c3*s2)-c4*c5*
(c2*c3-s2*s3)))*(c2*s3+c3*s2)-c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0)))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))-s5*(c4*s1-s4*(c1*c2*c3
-c1*s2*s3))*((s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+I66*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-I65*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+d6*m6*z6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))-s5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))*((s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*
(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I65*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+d6*m6*z6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))-c5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I56*(c5*
(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+s4*s5*(I56*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+(s5*(c2*s3+c3*s2)-c4*c5*
(c2*c3-s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))*(c2*s3+c3*s2)-d6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*
(d6*m6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+m6*z6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+m6*x6*c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-m6*y6*s6*(c5*(c1*c2*c3-c1*s2*s3)-
c4*s5*(c1*c2*s3+c1*c3*s2)))-d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(d6*m6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+m6*z6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+m6*x6*c6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m6*y6*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2)))+s4*s5*
(c2*s3+c3*s2)*((s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I66*c6*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-I65*s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+d6*m6*z6*(s5*(c2*s3+c3*s2)-c4*c5*
(c2*c3-s2*s3)))+c5*s4*(I56*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0)))*(c2*s3+c3*s2)+d6*s4*s5*(c2*s3+c3*s2)*(m6*z6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-
s2*s3))+d6*m6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+m6*x6*c6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))-m6*y6*s6*(c5*
(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3)));;
        C436=   -(I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))+(c6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0)))*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(I64*(c6*(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(s6*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I64*(c6*
(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*
(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*(I64*
(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))+(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*
(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+(c6*
(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0)))-(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I64*(c6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+s5*(I65*(c6*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+I66*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*
(I65*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+I66*(s6*(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))+d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(m6*x6*(s6*
(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))+m6*y6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2)))-s4*s5*(c2*s3+c3*s2)*(I65*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3
-s2*s3))+s4*s6*(c2*c3-s2*s3))+I66*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3)))+d6*s5*(m6*x6*
(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+m6*y6*(c6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*s4*s5*(m6*x6*
(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))+m6*y6*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-
s2*s3))+s4*s6*(c2*c3-s2*s3)))*(c2*s3+c3*s2);;
        C445=   -(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I66*c5*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))-c6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s5*s6*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+d6*m6*x6*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(c6*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I65*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-I64*c6*s5*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+s5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*c5*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I66*c5*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-c6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+I64*s5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+d6*m6*x6*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c6*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I65*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-I64*c6*s5*(c1*c4
-s4*(s1*s2*s3-c2*c3*s1))+s5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+d6*m6*y6*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(I55*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-I54*s5*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I55*c5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))-I54*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I66*c5*s4*
(c2*s3+c3*s2)-c6*s4*s5*(c2*s3+c3*s2)*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s4*s5*s6*
(c2*s3+c3*s2)+d6*m6*x6*c5*s4*(c2*s3+c3*s2))+(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*(I65*c5*s4*(c2*s3+c3*s2)-
I64*c6*s4*s5*(c2*s3+c3*s2)+s4*s5*s6*(c2*s3+c3*s2)*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*c5*s4*
(c2*s3+c3*s2))-c4*(I54*s4*s5*(c2*s3+c3*s2)-I55*c5*s4*(c2*s3+c3*s2))*(c2*s3+c3*s2)+s5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))*(c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*s5*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+I65*s5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+d6*m6*z6*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+c5*
(I56*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*
(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I65*s5*s6*(c1*c4-s4*(s1*s2*s3
-c2*c3*s1))+d6*m6*z6*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+s5*(c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*
(1.0/2.0)-I53*(1.0/2.0))-I56*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))*(I56*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0)))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)+I52*
(1.0/2.0)-I53*(1.0/2.0))-I56*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-c5*s4*(s4*s5*(c2*s3+c3*s2)*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0))-I56*c5*s4*(c2*s3+c3*s2))*(c2*s3+c3*s2)+s4*s5*(c5*s4*(c2*s3+c3*s2)*(I51*(1.0/2.0)+I52*
(1.0/2.0)-I53*(1.0/2.0))-I56*s4*s5*(c2*s3+c3*s2))*(c2*s3+c3*s2)+d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(d6*m6*c5*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+m6*z6*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m6*x6*c6*s5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+m6*y6*s5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+s4*s5*(c2*s3+c3*s2)*(c5*s4*(c2*s3+c3*s2)*(I61*
(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-I66*c6*s4*s5*(c2*s3+c3*s2)+I65*s4*s5*s6*(c2*s3+c3*s2)+d6*m6*z6*c5*s4*
(c2*s3+c3*s2))+d6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(m6*z6*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+d6*m6*c5*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+m6*y6*s5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m6*x6*c6*s5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))+d6*s4*s5*(c2*s3+c3*s2)*(d6*m6*c5*s4*(c2*s3+c3*s2)+m6*z6*c5*s4*(c2*s3+c3*s2)-m6*x6*c6*s4*s5*
(c2*s3+c3*s2)+m6*y6*s4*s5*s6*(c2*s3+c3*s2));;
        C446=   (I64*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(s6*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))*(c6*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(I64*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*
(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1)))-(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I64*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))-(c4*c6*
(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*
(c2*s3+c3*s2))*(I64*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))-(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*
(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))+(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))*(I64*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(s6*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-(s6*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I64*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3)))-(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+s5*(I65*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))-I66*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))))*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I65*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1)))-I66*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))))-s4*s5*(c2*s3+c3*s2)*
(I65*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))-I66*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2)))-d6*s5*(m6*x6*
(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-m6*y6*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-
c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(m6*x6*
(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-m6*y6*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))))+d6*s4*s5*(c2*s3+c3*s2)*(m6*x6*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))-
m6*y6*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2)));;
        C456=   -(c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+I64*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))*(c6*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(I64*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0)))*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(I64*c6*(c5*
(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0)))*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))+(c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))*(c4*c6*(c2*s3+c3*s2)-
c5*s4*s6*(c2*s3+c3*s2))-(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(c6*(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))-(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1)))*(I64*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+s6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+s5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))*(I65*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+I66*s6*(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))+s5*(I65*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+I66*s6*
(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+d6*s5*(m6*y6*c6*(s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*x6*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2)))*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+d6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(m6*y6*c6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+m6*x6*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2)))+s4*s5*(I65*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+I66*s6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2)))*(c2*s3+c3*s2)+d6*s4*s5*(m6*y6*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+m6*x6*s6*(c5*(c2*c3-
s2*s3)-c4*s5*(c2*s3+c3*s2)))*(c2*s3+c3*s2);;
        C511=   (s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I54*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+I56*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))*(I54*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I56*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0))+m5*x5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I55*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I56*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0))-m5*z5*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I55*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+I56*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0))+m5*z5*(a1*c1+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-
s5*(c1*c2*s3+c1*c3*s2)))+I66*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+m6*z6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*
(I65*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+I66*(s6*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-m6*z6*(d4*
(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-
a3*s1*s2*s3))-d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(m6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+m6*x6*(s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*y6*(c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*z6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))+c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+I66*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*x6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-s6*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+m6*y6*(a1*c1+d6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-d6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))*(-m6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+m6*x6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+m6*y6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+m6*z6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))+c6*(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))-m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3));;
        C522=   (c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I56*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-(s5*(c2*c3-
s2*s3)+c4*c5*(c2*s3+c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I54*s4*(c2*s3+c3*s2)-m5*x5*(-d4*(c2*c3-
s2*s3)+a2*s2+a3*c2*s3+a3*c3*s2))-(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I56*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))-(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-I55*s4*
(c2*s3+c3*s2)+m5*z5*(-d4*(c2*c3-s2*s3)+a2*s2+a3*c2*s3+a3*c3*s2))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))*((c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+I65*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-I66*(c6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-
c1*s2*s3))-I66*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(d6*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*((c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+I65*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-I66*(c6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))-m6*z6*(-d4*(c2*c3-s2*s3)-d6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))+a2*s2+a3*c2*s3+a3*c3*s2))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-I54*s4*
(c1*c2*c3-c1*s2*s3))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*c2*s3+c1*c3*s2)-
c4*c5*(c1*c2*c3-c1*s2*s3))-(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0))-m5*z5*(d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)+I55*s4*(c1*c2*c3-c1*s2*s3))-(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-
(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I54*s4*(s1*s2*s3-
c2*c3*s1)+m5*x5*(d4*(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))*(-I56*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I55*s4*(s1*s2*s3-c2*c3*s1)+m5*z5*(d4*
(c2*s1*s3+c3*s1*s2)+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+d6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))*(m6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-
c2*c3*s1))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3)+m6*z6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-m6*x6*(c6*
(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+m6*y6*(s6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1)))-c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*(c2*s1*s3+c3*s1*s2))*(I66*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+I64*(s6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*
(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+s6*
(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-
c2*c3*s1))-I64*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+(s6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-
c2*c3*s1))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(m6*
(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3)-
m6*x6*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+m6*y6*(s6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+m6*z6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*
(c1*c2*c3-c1*s2*s3)))-d6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(m6*(-d4*(c2*c3-s2*s3)-d6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))+a2*s2+a3*c2*s3+a3*c3*s2)-m6*z6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+m6*x6*(c6*(s5*(c2*c3-
s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))-m6*y6*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*
(c2*s3+c3*s2)))+c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s5*(c1*c2*s3+c1*c3*s2)-
c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+I66*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-(c6*
(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))*(-I64*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+I65*
(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*
(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3
-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*
(I66*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+I64*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-
(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-
m6*x6*(-d4*(c2*c3-s2*s3)-d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+a2*s2+a3*c2*s3+a3*c3*s2))-s6*(c5*(c2*c3-s2*s3)-
c4*s5*(c2*s3+c3*s2))*(I65*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-I64*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-
s4*s6*(c2*s3+c3*s2))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0))-m6*y6*(-d4*(c2*c3-s2*s3)-d6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))+a2*s2+a3*c2*s3+a3*c3*s2));;
        C533=   (s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*((c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(1.0/2.0)+I62*
(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-I66*(c6*(s5*(c2*c3-
s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+m6*z6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))-I66*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*
(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I61*
(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-I54*s4*(c1*c2*c3-
c1*s2*s3))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3
-c1*s2*s3))-(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*
(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)+I55*s4*(c1*c2*c3-c1*s2*s3))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-
a3*s1*s2*s3)+I54*s4*(s1*s2*s3-c2*c3*s1))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(-I56*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I55*s4*(s1*s2*s3-
c2*c3*s1))+(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I56*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-(s5*(c2*c3-
s2*s3)+c4*c5*(c2*s3+c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I54*s4*(c2*s3+c3*s2)-m5*x5*(-d4*(c2*c3-
s2*s3)+a3*c2*s3+a3*c3*s2))-(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I56*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-(c5*
(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-I55*s4*(c2*s3+c3*s2)+m5*z5*(-d4*
(c2*c3-s2*s3)+a3*c2*s3+a3*c3*s2))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*((c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-I66*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3))-s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(-
I64*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+I65*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*
(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3
-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))+d6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))*(m6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1
-a3*s1*s2*s3)+m6*z6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-m6*x6*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+m6*y6*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-
c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1)))-c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I66*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+I64*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*
(s1*s2*s3-c2*c3*s1))-(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-
c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3))+d6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(m6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3
-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2)+m6*z6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-m6*x6*(c6*(s5*(c2*c3-
s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+m6*y6*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*
(c2*s3+c3*s2)))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))-I64*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+(s6*
(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-
a3*s1*s2*s3))+c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I66*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+I64*(s6*(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2))-d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(m6*(d6*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-m6*x6*(c6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+m6*y6*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*
(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+m6*z6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3)))-s6*(c5*
(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I65*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-I64*(c6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-
a3*c3*s2))+c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*
(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+I66*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-(c6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-
a3*c1*s2*s3));;
        C544=   -(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-c5*s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+I66*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c5*c6*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3)))+s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3)))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I54*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3)))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I55*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+I56*c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-
I53*(1.0/2.0)))-(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(-I55*s4*(c2*s3+c3*s2)+c4*s5*(c2*s3+c3*s2)*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+I56*c4*c5*(c2*s3+c3*s2))-(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(-I54*s4*
(c2*s3+c3*s2)+c4*c5*(c2*s3+c3*s2)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*c4*s5*(c2*s3+c3*s2))-(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I55*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+I56*c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0)))+(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I54*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))
*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))*(I65*(c6*s4*(c2*s3+c3*s2)+c4*c5*s6*(c2*s3+c3*s2))+I66*(s4*s6*(c2*s3+c3*s2)-c4*c5*c6*(c2*s3+c3*s2))-
c4*s5*(c2*s3+c3*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-d6*m6*z6*c4*s5*(c2*s3+c3*s2))-(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-c5*s6*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1)))+I66*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c5*c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))-d6*(c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(m6*x6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c5*c6*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3)))+m6*y6*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-c5*s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+d6*m6*s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+m6*z6*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-c5*s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+(s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+c5*c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+d6*m6*x6*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))-s6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c5*c6*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-c5*s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0))+I65*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+d6*m6*y6*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+d6*(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(m6*x6*(s4*s6*(c2*s3+c3*s2)-c4*c5*c6*(c2*s3+c3*s2))+m6*y6*(c6*s4*
(c2*s3+c3*s2)+c4*c5*s6*(c2*s3+c3*s2))-d6*m6*c4*s5*(c2*s3+c3*s2)-m6*z6*c4*s5*(c2*s3+c3*s2))-d6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(m6*x6*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c5*c6*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1)))+m6*y6*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-c5*s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+d6*m6*s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+m6*z6*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I64*
(c6*s4*(c2*s3+c3*s2)+c4*c5*s6*(c2*s3+c3*s2))+(s4*s6*(c2*s3+c3*s2)-c4*c5*c6*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))-I66*c4*s5*(c2*s3+c3*s2)-d6*m6*x6*c4*s5*(c2*s3+c3*s2))+c6*(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-c5*s6*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c5*c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))+I66*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+d6*m6*x6*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))-s6*(c5*
(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I64*(s4*s6*(c2*s3+c3*s2)-c4*c5*c6*(c2*s3+c3*s2))+(c6*s4*
(c2*s3+c3*s2)+c4*c5*s6*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*c4*s5*(c2*s3+c3*s2)-
d6*m6*y6*c4*s5*(c2*s3+c3*s2))-s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))+c5*c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-c5*s6*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+d6*m6*y6*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)));;
        C555= 0;
        C566=   (s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I65*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*
(c2*s3+c3*s2))-I66*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2)))-(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+I66*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+I66*(s6*(c4*s1-s4*(c1*c2*c3
-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))-d6*(m6*x6*(s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+m6*y6*(c6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))+c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*
(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-d6*(c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(m6*x6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+m6*y6*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))-s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(c1*c4
-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(c6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0)))+c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-
s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-d6*(m6*x6*
(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))-m6*y6*(s6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2)))*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-
s5*(c1*c2*s3+c1*c3*s2)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))+c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I64*
(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I64*(c6*
(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))-(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*
(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)));;
        C512=   (s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0))-m5*x5*(-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+I54*s4*(c1*c2*s3+c1*c3*s2))+(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-
(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(-d4*
(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)-I55*s4*(c1*c2*s3+c1*c3*s2))+(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*((c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*
(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-
I66*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*z6*(d4*(s1*s2*s3-
c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))-(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-(s5*(s1*s2*s3
-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)-I54*s4*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(s1*s2*s3-
c2*c3*s1)+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+I55*s4*(c2*s1*s3+c3*s1*s2))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))*(I65*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-I66*
(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-m6*z6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-c6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-
c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+s6*(s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I65*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-
I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*
(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))+d6*
(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(m6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2)+m6*x6*(c6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))-m6*y6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-m6*z6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2)))+c6*(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(c6*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))-m6*x6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*
(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))-I65*(c5*(c1*c2*c3-c1*s2*s3)-
c4*s5*(c1*c2*s3+c1*c3*s2))-(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))+d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(m6*
(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2)+m6*z6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-
m6*x6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*y6*(s6*(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)));;
        C513=   -(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))-(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*
(1.0/2.0))+m5*x5*(d4*(s1*s2*s3-c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)-I54*s4*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-(c5*(s1*s2*s3
-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*(d4*(s1*s2*s3-
c2*c3*s1)+a3*c2*s1*s3+a3*c3*s1*s2)+I55*s4*(c2*s1*s3+c3*s1*s2))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))*(I65*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))-I66*
(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))*(I56*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))-m5*x5*(-d4*(c1*c2*c3-
c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)+I54*s4*(c1*c2*s3+c1*c3*s2))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))*(I56*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(-d4*(c1*c2*c3-
c1*s2*s3)+a3*c1*c2*s3+a3*c1*c3*s2)-I55*s4*(c1*c2*s3+c1*c3*s2))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))*((c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+I65*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-I66*(c6*(s5*
(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+m6*z6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*
(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))-d6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))*(m6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3
-a3*c1*c3*s2)-m6*x6*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+m6*y6*(s6*
(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+m6*z6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2)))+c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-
c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))-s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(-I64*(c6*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+I65*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-
c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))+d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(m6*(d4*
(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2)+m6*z6*(c5*
(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-m6*x6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))+m6*y6*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))-c6*
(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*
(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))+s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I65*
(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*
(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2));;
        C514=   (c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+I66*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))-s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-d6*m6*z6*s5*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))-(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(-I54*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1)))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(-I55*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+I56*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-
I53*(1.0/2.0)))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(-I54*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s5*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(-I55*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+I56*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I51*(1.0/2.0)+I52*(1.0/2.0)-
I53*(1.0/2.0)))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+I66*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1)))-s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))-d6*m6*z6*s5*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1)))+d6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(m6*x6*(s6*(s1*s4+c4*(c1*c2*c3
-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+m6*y6*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))-d6*m6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-m6*z6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-c6*(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))+(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*x6*s5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))-d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(m6*x6*(s6*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+m6*y6*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1)))-d6*m6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-m6*z6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+s6*(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))+(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*y6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+c6*(s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1)))+(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-d6*m6*x6*s5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1)))-s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*
(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-d6*m6*y6*s5*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1)));;
        C515= 0;
        C516=   -(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-I66*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-
s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))*(I65*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))-I66*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))))+c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*
(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+s6*(s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-d6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(m6*x6*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-m6*y6*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))))-c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(c4*s1
-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))-(c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0)))+d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(m6*x6*(c6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))-m6*y6*(s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))))-s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)));;
        C523=   (s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*((c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(1.0/2.0)+I62*
(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-I66*(c6*(s5*(c2*c3-
s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+m6*z6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2))-(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))-I66*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*
(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I61*
(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+m6*z6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))+(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I56*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))*(I51*(-
1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-I54*s4*(c1*c2*c3-
c1*s2*s3))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I56*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3
-c1*s2*s3))-(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-m5*z5*
(d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)+I55*s4*(c1*c2*c3-c1*s2*s3))-(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*(c2*s1*s3+c3*s1*s2))*(I56*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+m5*x5*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-
a3*s1*s2*s3)+I54*s4*(s1*s2*s3-c2*c3*s1))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(-I56*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))*(I51*
(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))+m5*z5*(d4*(c2*s1*s3+c3*s1*s2)+a3*c2*c3*s1-a3*s1*s2*s3)+I55*s4*(s1*s2*s3-
c2*c3*s1))+(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I56*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-(s5*(c2*c3-
s2*s3)+c4*c5*(c2*s3+c3*s2))*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I54*s4*(c2*s3+c3*s2)-m5*x5*(-d4*(c2*c3-
s2*s3)+a3*c2*s3+a3*c3*s2))-(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I56*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-(c5*
(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*(1.0/2.0))-I55*s4*(c2*s3+c3*s2)+m5*z5*(-d4*
(c2*c3-s2*s3)+a3*c2*s3+a3*c3*s2))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*((c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+I65*(s6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-I66*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+m6*z6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3))-s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(-
I64*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+I65*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*
(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3
-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))+d6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))*(m6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1
-a3*s1*s2*s3)+m6*z6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-m6*x6*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+m6*y6*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-
c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1)))-c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I66*(c5*
(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+I64*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*
(s1*s2*s3-c2*c3*s1))-(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-
c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3))+d6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(m6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3
-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2)+m6*z6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-m6*x6*(c6*(s5*(c2*c3-
s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+m6*y6*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*
(c2*s3+c3*s2)))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I65*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*
(s1*s2*s3-c2*c3*s1))-I64*(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+(s6*
(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-
a3*s1*s2*s3))+c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I66*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+I64*(s6*(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2))-d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(m6*(d6*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3)-m6*x6*(c6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))+m6*y6*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*
(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+m6*z6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3)))-s6*(c5*
(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I65*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-I64*(c6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-
a3*c3*s2))+c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*
(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+I66*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))-(c6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-
a3*c1*s2*s3));;
        C524=   -(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I65*(c4*c6*(c2*s1*s3+c3*s1*s2)-
c5*s4*s6*(c2*s1*s3+c3*s1*s2))+I66*(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+s4*s5*
(c2*s1*s3+c3*s1*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s4*s5*(c2*s1*s3+c3*s1*s2))+(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I65*(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))+I66*
(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))+s4*s5*(c1*c2*s3+c1*c3*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-
I63*(1.0/2.0))+d6*m6*z6*s4*s5*(c1*c2*s3+c1*c3*s2))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*
(I55*c4*(c2*s1*s3+c3*s1*s2)+I56*c5*s4*(c2*s1*s3+c3*s1*s2)+s4*s5*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)+I52*(1.0/2.0)-
I53*(1.0/2.0)))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I54*c4*(c2*s1*s3+c3*s1*s2)+c5*s4*
(c2*s1*s3+c3*s1*s2)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*(c2*s1*s3+c3*s1*s2))-(s5*(c2*c3-
s2*s3)+c4*c5*(c2*s3+c3*s2))*(I55*c4*(c2*c3-s2*s3)+s4*s5*(c2*c3-s2*s3)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0))+I56*c5*s4*(c2*c3-s2*s3))-(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I54*c4*(c2*c3-s2*s3)+c5*s4*(c2*c3-
s2*s3)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*(c2*c3-s2*s3))-(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I54*c4*(c1*c2*s3+c1*c3*s2)+c5*s4*(c1*c2*s3+c1*c3*s2)*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*(c1*c2*s3+c1*c3*s2))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))
*(I55*c4*(c1*c2*s3+c1*c3*s2)+I56*c5*s4*(c1*c2*s3+c1*c3*s2)+s4*s5*(c1*c2*s3+c1*c3*s2)*(I51*(1.0/2.0)+I52*(1.0/2.0)-
I53*(1.0/2.0)))-(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I65*(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-s2*s3))+I66*
(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))+s4*s5*(c2*c3-s2*s3)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+d6*m6*z6*s4*s5*(c2*c3-s2*s3))+s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*
(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))+(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))*
(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c1*c2*s3+c1*c3*s2)+d6*m6*y6*s4*s5*(c1*c2*s3+c1*c3*s2))-d6*
(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(m6*x6*(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))+m6*y6*(c4*c6*(c2*c3-
s2*s3)-c5*s4*s6*(c2*c3-s2*s3))+d6*m6*s4*s5*(c2*c3-s2*s3)+m6*z6*s4*s5*(c2*c3-s2*s3))-d6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(m6*x6*(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+m6*y6*(c4*c6*
(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))+d6*m6*s4*s5*(c2*s1*s3+c3*s1*s2)+m6*z6*s4*s5*
(c2*s1*s3+c3*s1*s2))-c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I64*(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-
s2*s3))+(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*(c2*c3
-s2*s3)+d6*m6*x6*s4*s5*(c2*c3-s2*s3))+d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(m6*x6*(c4*s6*
(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))+m6*y6*(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*
(c1*c2*s3+c1*c3*s2))+d6*m6*s4*s5*(c1*c2*s3+c1*c3*s2)+m6*z6*s4*s5*(c1*c2*s3+c1*c3*s2))+s6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))*(I64*(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))+(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-s2*s3))*
(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c2*c3-s2*s3)+d6*m6*y6*s4*s5*(c2*c3-s2*s3))+c6*(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*
(c2*s1*s3+c3*s1*s2))+(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+I66*s4*s5*(c2*s1*s3+c3*s1*s2)+d6*m6*x6*s4*s5*(c2*s1*s3+c3*s1*s2))-c6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))+(c4*s6*
(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*
(c1*c2*s3+c1*c3*s2)+d6*m6*x6*s4*s5*(c1*c2*s3+c1*c3*s2))-s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))*(I64*(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+(c4*c6*(c2*s1*s3+c3*s1*s2)-
c5*s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*
(c2*s1*s3+c3*s1*s2)+d6*m6*y6*s4*s5*(c2*s1*s3+c3*s1*s2));;
        C525=0;
        C526=   (s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I65*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*
(c2*c3-s2*s3))+I66*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3)))+(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))+I66*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))+(c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+I66*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*
(c1*c2*s3+c1*c3*s2)))-c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+s6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*
(c2*s1*s3+c3*s1*s2))+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(c6*
(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+s6*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*
(c1*c2*s3+c1*c3*s2))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))+d6*(m6*x6*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-
s2*s3))+m6*y6*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3)))*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))+d6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(m6*x6*(s6*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))+m6*y6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2)))+d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*
(m6*x6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+m6*y6*(c6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2)))+c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I64*
(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*
(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I64*(s6*
(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))+(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*
(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)));;
        C534=   -(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I65*(c4*c6*(c2*s1*s3+c3*s1*s2)-
c5*s4*s6*(c2*s1*s3+c3*s1*s2))+I66*(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+s4*s5*
(c2*s1*s3+c3*s1*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*(1.0/2.0))+d6*m6*z6*s4*s5*(c2*s1*s3+c3*s1*s2))+(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I65*(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))+I66*
(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))+s4*s5*(c1*c2*s3+c1*c3*s2)*(I61*(1.0/2.0)+I62*(1.0/2.0)-
I63*(1.0/2.0))+d6*m6*z6*s4*s5*(c1*c2*s3+c1*c3*s2))-(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*
(I55*c4*(c2*s1*s3+c3*s1*s2)+I56*c5*s4*(c2*s1*s3+c3*s1*s2)+s4*s5*(c2*s1*s3+c3*s1*s2)*(I51*(1.0/2.0)+I52*(1.0/2.0)-
I53*(1.0/2.0)))+(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I54*c4*(c2*s1*s3+c3*s1*s2)+c5*s4*
(c2*s1*s3+c3*s1*s2)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*(c2*s1*s3+c3*s1*s2))-(s5*(c2*c3-
s2*s3)+c4*c5*(c2*s3+c3*s2))*(I55*c4*(c2*c3-s2*s3)+s4*s5*(c2*c3-s2*s3)*(I51*(1.0/2.0)+I52*(1.0/2.0)-I53*
(1.0/2.0))+I56*c5*s4*(c2*c3-s2*s3))-(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I54*c4*(c2*c3-s2*s3)+c5*s4*(c2*c3-
s2*s3)*(I51*(-1.0/2.0)+I52*(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*(c2*c3-s2*s3))-(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I54*c4*(c1*c2*s3+c1*c3*s2)+c5*s4*(c1*c2*s3+c1*c3*s2)*(I51*(-1.0/2.0)+I52*
(1.0/2.0)+I53*(1.0/2.0))+I56*s4*s5*(c1*c2*s3+c1*c3*s2))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))
*(I55*c4*(c1*c2*s3+c1*c3*s2)+I56*c5*s4*(c1*c2*s3+c1*c3*s2)+s4*s5*(c1*c2*s3+c1*c3*s2)*(I51*(1.0/2.0)+I52*(1.0/2.0)-
I53*(1.0/2.0)))-(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I65*(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-s2*s3))+I66*
(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))+s4*s5*(c2*c3-s2*s3)*(I61*(1.0/2.0)+I62*(1.0/2.0)-I63*
(1.0/2.0))+d6*m6*z6*s4*s5*(c2*c3-s2*s3))+s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*
(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))+(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))*
(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c1*c2*s3+c1*c3*s2)+d6*m6*y6*s4*s5*(c1*c2*s3+c1*c3*s2))-d6*
(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(m6*x6*(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))+m6*y6*(c4*c6*(c2*c3-
s2*s3)-c5*s4*s6*(c2*c3-s2*s3))+d6*m6*s4*s5*(c2*c3-s2*s3)+m6*z6*s4*s5*(c2*c3-s2*s3))-d6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(m6*x6*(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+m6*y6*(c4*c6*
(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))+d6*m6*s4*s5*(c2*s1*s3+c3*s1*s2)+m6*z6*s4*s5*
(c2*s1*s3+c3*s1*s2))-c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I64*(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-
s2*s3))+(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*(c2*c3
-s2*s3)+d6*m6*x6*s4*s5*(c2*c3-s2*s3))+d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(m6*x6*(c4*s6*
(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))+m6*y6*(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*
(c1*c2*s3+c1*c3*s2))+d6*m6*s4*s5*(c1*c2*s3+c1*c3*s2)+m6*z6*s4*s5*(c1*c2*s3+c1*c3*s2))+s6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))*(I64*(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))+(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-s2*s3))*
(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c2*c3-s2*s3)+d6*m6*y6*s4*s5*(c2*c3-s2*s3))+c6*(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*
(c2*s1*s3+c3*s1*s2))+(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+I66*s4*s5*(c2*s1*s3+c3*s1*s2)+d6*m6*x6*s4*s5*(c2*s1*s3+c3*s1*s2))-c6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))+(c4*s6*
(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*
(c1*c2*s3+c1*c3*s2)+d6*m6*x6*s4*s5*(c1*c2*s3+c1*c3*s2))-s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))*(I64*(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+(c4*c6*(c2*s1*s3+c3*s1*s2)-
c5*s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*
(c2*s1*s3+c3*s1*s2)+d6*m6*y6*s4*s5*(c2*s1*s3+c3*s1*s2));;
        C535= 0;
        C536=   (s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I65*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*
(c2*c3-s2*s3))+I66*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3)))+(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I65*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))+I66*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2)))+(c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I65*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+I66*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*
(c1*c2*s3+c1*c3*s2)))-c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(c6*(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+s6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*
(c2*s1*s3+c3*s1*s2))+(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))-c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(c6*
(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))+s6*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*
(c1*c2*s3+c1*c3*s2))+(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))+d6*(m6*x6*(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-
s2*s3))+m6*y6*(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3)))*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))+d6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(m6*x6*(s6*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))+m6*y6*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2)))+d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*
(m6*x6*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+m6*y6*(c6*(s5*(c1*c2*c3-
c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2)))+c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I64*
(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*(c2*c3-s2*s3))+(s6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*
(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I64*(s6*
(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))-c6*s4*(c2*c3-s2*s3))+(c6*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+s4*s6*
(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)));;
        C545= 0;
        C546=   (c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I65*(s6*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-I66*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))))+(I65*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-I66*(c6*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))))*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))+(I65*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))-I66*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*
(c2*s3+c3*s2)))*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))*(I64*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(c6*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-s6*(s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3)))-(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0)))-d6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(m6*x6*(c6*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-m6*y6*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-
s4*(s1*s2*s3-c2*c3*s1))))-d6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(m6*x6*(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*
(c2*s3+c3*s2))-m6*y6*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2)))-c6*(I64*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*
(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-s6*(I64*
(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*
(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0)))*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*(c2*s1*s3+c3*s1*s2))+c6*(I64*(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))-(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*
(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+s6*(I64*
(c4*c6*(c2*s3+c3*s2)-c5*s4*s6*(c2*s3+c3*s2))-(c4*s6*(c2*s3+c3*s2)+c5*c6*s4*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0)))*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2))*(m6*x6*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-m6*y6*(s6*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))));;
        C556=   -(I65*c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+I66*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))*
(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I65*c6*(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+I66*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2)))+(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I65*c6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+I66*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))+d6*
(m6*y6*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+m6*x6*s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-
c5*(c2*s1*s3+c3*s1*s2)))*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-c6*(I64*c6*(c5*(c2*c3-s2*s3)-
c4*s5*(c2*s3+c3*s2))+s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))*(c5*
(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+s6*(c6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+I64*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-c6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I64*c6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0)))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(c6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))+d6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*
(m6*y6*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*x6*s6*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))-c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I64*c6*(s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0)))-d6*(m6*y6*c6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))+m6*x6*s6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+s6*(s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2)));;
        C611=   -(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))
*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s6*(c4*s1
-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))+I66*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*x6*(a1*c1+d6*(s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-
a3*c1*s2*s3))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*
(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c6*(c4*s1-
s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+m6*y6*(a1*c1+d6*(s5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-
(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(c6*(c1*c4-
s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+I66*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*
(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))+(s6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*
(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))-m6*y6*(d4*(c2*s1*s3+c3*s1*s2)+a1*s1-d6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3));;
        C622=   (c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*
(I66*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+I64*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-
c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-
c2*c3*s1))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-
c4*s5*(s1*s2*s3-c2*c3*s1))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I65*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-I64*(c6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*
(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a2*c2*s1+a3*c2*c3*s1-a3*s1*s2*s3))-(c6*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(s6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+I66*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*
(c1*c2*c3-c1*s2*s3))-(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*
(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(-I64*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*
(c1*c2*c3-c1*s2*s3))+I65*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*
(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a2*c1*c2+a3*c1*c2*c3-a3*c1*s2*s3))-(s6*(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I66*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+I64*(s6*(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-m6*x6*(-d4*(c2*c3-s2*s3)-d6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))+a2*s2+a3*c2*s3+a3*c3*s2))-(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I65*(c5*
(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-I64*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))+(s6*(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-m6*y6*(-d4*
(c2*c3-s2*s3)-d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))+a2*s2+a3*c2*s3+a3*c3*s2));;
        C633=   -(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I66*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))+I64*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-(c6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*c3-s2*s3)+d6*(c5*
(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2))-(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))*(I65*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-I64*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*y6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2))+(c6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I66*(c5*(c2*s1*s3+c3*s1*s2)-
c4*s5*(s1*s2*s3-c2*c3*s1))+I64*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-
(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-
c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3))-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I65*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-I64*(c6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*
(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3))-(c6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(s6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+I66*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*
(c1*c2*c3-c1*s2*s3))-(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(-I64*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-
c1*s2*s3))+I65*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-
c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3));;
        C644=   -(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))
*(I64*(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-c5*s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+(s6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))+c5*c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+d6*m6*x6*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c5*c6*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-c5*s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0))+I65*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+d6*m6*y6*s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3)))-(s6*
(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I64*(c6*s4*(c2*s3+c3*s2)+c4*c5*s6*
(c2*s3+c3*s2))+(s4*s6*(c2*s3+c3*s2)-c4*c5*c6*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-
I66*c4*s5*(c2*s3+c3*s2)-d6*m6*x6*c4*s5*(c2*s3+c3*s2))-(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))*(I64*(s4*s6*(c2*s3+c3*s2)-c4*c5*c6*(c2*s3+c3*s2))+(c6*s4*(c2*s3+c3*s2)+c4*c5*s6*(c2*s3+c3*s2))*
(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))-I65*c4*s5*(c2*s3+c3*s2)-d6*m6*y6*c4*s5*(c2*s3+c3*s2))-(c6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(c6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))-c5*s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c5*c6*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+d6*m6*x6*s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I64*(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c5*c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))+(c6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))-c5*s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+d6*m6*y6*s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1)));;
        C655=   -(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))
*(I66*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))-(c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I66*(s5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*
(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I65*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*
(c2*s1*s3+c3*s1*s2))+I64*c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-s6*(c5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(c1*s4+c4*(s1*s2*s3-
c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I65*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+I64*c6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2)))-(s6*
(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I66*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-c6*(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))+d6*m6*x6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2)))-(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))*(I65*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-I64*c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+s6*(s5*
(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2)));;
        C666= 0;
        C612=   -(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))
*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-
c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*
(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-m6*x6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))-(s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))-I65*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(s6*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*y6*(-d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-d4*(c1*c2*c3-
c1*s2*s3)+a2*c1*s2+a3*c1*c2*s3+a3*c1*c3*s2))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-
c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-
c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2))-(s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I65*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+a2*s1*s2+a3*c2*s1*s3+a3*c3*s1*s2));;
        C613=   -(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))
*(I64*(s6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))+I66*(c5*(c1*c2*c3-
c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))-(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))-s4*s6*
(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*
(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-a3*c1*c3*s2))+(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(-I64*(c6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*
(c1*c2*s3+c1*c3*s2))-s4*s6*(c1*c2*s3+c1*c3*s2))+I65*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+(s6*(s5*
(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*y6*(d6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d4*(c1*c2*c3-c1*s2*s3)-a3*c1*c2*s3-
a3*c1*c3*s2))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*
(I66*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+I64*(s6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))-(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*
(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-
c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*
(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I65*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-
I64*(c6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+s4*s6*(c2*s1*s3+c3*s1*s2))+(s6*(s5*(s1*s2*s3-c2*c3*s1)-
c4*c5*(c2*s1*s3+c3*s1*s2))-c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*
(s1*s2*s3-c2*c3*s1)+d6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))+a3*c2*s1*s3+a3*c3*s1*s2));;
        C614=   (c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*
(I64*(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-
c5*c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-d6*m6*x6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(s6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-c5*c6*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3)))+(c6*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-d6*m6*y6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(c6*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(c6*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1)))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I66*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-
d6*m6*x6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(s6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*c6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1)))+(c6*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+c5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0))-I65*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-d6*m6*y6*s5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)));;
        C615=   -(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))
*(I66*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2))-c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*
(c1*c2*s3+c1*c3*s2))+d6*m6*x6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I65*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-
s5*(c1*c2*s3+c1*c3*s2))-I64*c6*(s5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))+s6*(s5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))+c5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(s1*s4+c4*
(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))+(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I66*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-c6*(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s6*(s5*
(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))-(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I65*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2))-I64*c6*(s5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))+s6*(s5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))-c5*(c2*s1*s3+c3*s1*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)));;
        C616= 0;
        C623=   -(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I66*(c5*(c2*c3-s2*s3)-c4*s5*
(c2*s3+c3*s2))+I64*(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))-(c6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*c3-s2*s3)+d6*(c5*
(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2))-(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))*(I65*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-I64*(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+m6*y6*(d4*(c2*c3-s2*s3)+d6*(c5*(c2*c3-s2*s3)-c4*s5*(c2*s3+c3*s2))-a3*c2*s3-a3*c3*s2))+(c6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I66*(c5*(c2*s1*s3+c3*s1*s2)-
c4*s5*(s1*s2*s3-c2*c3*s1))+I64*(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))-
(c6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d4*(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-
c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3))-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I65*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))-I64*(c6*(s5*
(c2*s1*s3+c3*s1*s2)+c4*c5*(s1*s2*s3-c2*c3*s1))-s4*s6*(s1*s2*s3-c2*c3*s1))+(s6*(s5*(c2*s1*s3+c3*s1*s2)+c4*c5*
(s1*s2*s3-c2*c3*s1))+c6*s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d4*
(c2*s1*s3+c3*s1*s2)+d6*(c5*(c2*s1*s3+c3*s1*s2)-c4*s5*(s1*s2*s3-c2*c3*s1))+a3*c2*c3*s1-a3*s1*s2*s3))-(c6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(s6*(s5*
(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))+I66*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*
(c1*c2*c3-c1*s2*s3))-(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-c1*s2*s3))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+m6*x6*(d6*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*
(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-
c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(-I64*(c6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-c1*s2*s3))+s4*s6*(c1*c2*c3-
c1*s2*s3))+I65*(c5*(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+(s6*(s5*(c1*c2*s3+c1*c3*s2)-c4*c5*(c1*c2*c3-
c1*s2*s3))-c6*s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+m6*y6*(d6*(c5*
(c1*c2*s3+c1*c3*s2)+c4*s5*(c1*c2*c3-c1*s2*s3))+d4*(c1*c2*s3+c1*c3*s2)+a3*c1*c2*c3-a3*c1*s2*s3));;
        C624=   (s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I64*(c4*c6*(c2*c3-s2*s3)-
c5*s4*s6*(c2*c3-s2*s3))+(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+I66*s4*s5*(c2*c3-s2*s3)+d6*m6*x6*s4*s5*(c2*c3-s2*s3))+(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))*(I64*(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))+(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-s2*s3))*
(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c2*c3-s2*s3)+d6*m6*y6*s4*s5*(c2*c3-s2*s3))-(c6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(c4*c6*
(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))+(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*(c2*s1*s3+c3*s1*s2)+d6*m6*x6*s4*s5*(c2*s1*s3+c3*s1*s2))+(c6*(c4*s1
-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(c4*c6*
(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))+(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*(c1*c2*s3+c1*c3*s2)+d6*m6*x6*s4*s5*(c1*c2*s3+c1*c3*s2))+(s6*(c1*c4
-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(c4*s6*
(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c2*s1*s3+c3*s1*s2)+d6*m6*y6*s4*s5*(c2*s1*s3+c3*s1*s2))-(s6*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(c4*s6*
(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))+(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c1*c2*s3+c1*c3*s2)+d6*m6*y6*s4*s5*(c1*c2*s3+c1*c3*s2));;
        C625=   -(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))
*(I66*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*
(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*
(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*
(c2*s3+c3*s2))*(I66*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+c6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+d6*m6*x6*(s5*(c2*s3+c3*s2)-
c4*c5*(c2*c3-s2*s3)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I65*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+I64*c6*(c5*(c1*c2*c3-c1*s2*s3)-
c4*s5*(c1*c2*s3+c1*c3*s2))-s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+d6*m6*y6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))+(c6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I65*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+I64*c6*(c5*(c2*s3+c3*s2)+c4*s5*
(c2*c3-s2*s3))-s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*
(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I66*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I65*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+I64*c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2)));;
        C626=0;
        C634=   (s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I64*(c4*c6*(c2*c3-s2*s3)-
c5*s4*s6*(c2*c3-s2*s3))+(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+I66*s4*s5*(c2*c3-s2*s3)+d6*m6*x6*s4*s5*(c2*c3-s2*s3))+(c6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))-s4*s6*
(c2*s3+c3*s2))*(I64*(c4*s6*(c2*c3-s2*s3)+c5*c6*s4*(c2*c3-s2*s3))+(c4*c6*(c2*c3-s2*s3)-c5*s4*s6*(c2*c3-s2*s3))*
(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c2*c3-s2*s3)+d6*m6*y6*s4*s5*(c2*c3-s2*s3))-(c6*(c1*c4-s4*
(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(c4*c6*
(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))+(c4*s6*(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*(c2*s1*s3+c3*s1*s2)+d6*m6*x6*s4*s5*(c2*s1*s3+c3*s1*s2))+(c6*(c4*s1
-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(c4*c6*
(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))+(c4*s6*(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I66*s4*s5*(c1*c2*s3+c1*c3*s2)+d6*m6*x6*s4*s5*(c1*c2*s3+c1*c3*s2))+(s6*(c1*c4
-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I64*(c4*s6*
(c2*s1*s3+c3*s1*s2)+c5*c6*s4*(c2*s1*s3+c3*s1*s2))+(c4*c6*(c2*s1*s3+c3*s1*s2)-c5*s4*s6*(c2*s1*s3+c3*s1*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c2*s1*s3+c3*s1*s2)+d6*m6*y6*s4*s5*(c2*s1*s3+c3*s1*s2))-(s6*
(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I64*(c4*s6*
(c1*c2*s3+c1*c3*s2)+c5*c6*s4*(c1*c2*s3+c1*c3*s2))+(c4*c6*(c1*c2*s3+c1*c3*s2)-c5*s4*s6*(c1*c2*s3+c1*c3*s2))*(I61*
(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+I65*s4*s5*(c1*c2*s3+c1*c3*s2)+d6*m6*y6*s4*s5*(c1*c2*s3+c1*c3*s2));;
        C635=   -(c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))
*(I66*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+c6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*
(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))+d6*m6*x6*
(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*(c2*s3+c3*s2))+c6*s4*
(c2*s3+c3*s2))*(I66*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+c6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(-
1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))+d6*m6*x6*(s5*(c2*s3+c3*s2)-
c4*c5*(c2*c3-s2*s3)))+(s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*
(c1*c2*s3+c1*c3*s2)))*(I65*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2))+I64*c6*(c5*(c1*c2*c3-c1*s2*s3)-
c4*s5*(c1*c2*s3+c1*c3*s2))-s6*(c5*(c1*c2*c3-c1*s2*s3)-c4*s5*(c1*c2*s3+c1*c3*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*
(1.0/2.0))+d6*m6*y6*(s5*(c1*c2*c3-c1*s2*s3)+c4*c5*(c1*c2*s3+c1*c3*s2)))+(c6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I65*(s5*(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3))+I64*c6*(c5*(c2*s3+c3*s2)+c4*s5*
(c2*c3-s2*s3))-s6*(c5*(c2*s3+c3*s2)+c4*s5*(c2*c3-s2*s3))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*
(c2*s3+c3*s2)-c4*c5*(c2*c3-s2*s3)))-(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*
(c2*s1*s3+c3*s1*s2)))*(I66*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2))+c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))-I64*s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))+d6*m6*x6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*(c2*s1*s3+c3*s1*s2)))+(s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+c6*(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I65*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2))+I64*c6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*(c2*s1*s3+c3*s1*s2))-s6*(c5*(s1*s2*s3-c2*c3*s1)+c4*s5*
(c2*s1*s3+c3*s1*s2))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*(s5*(s1*s2*s3-c2*c3*s1)-c4*c5*
(c2*s1*s3+c3*s1*s2)));;
        C636=0;
        C645=   (c6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-s6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*
(I66*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))-c6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*
(1.0/2.0))+I64*s5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+d6*m6*x6*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))-(s6*(c4*s1-s4*
(c1*c2*c3-c1*s2*s3))+c6*(c5*(s1*s4+c4*(c1*c2*c3-c1*s2*s3))-s5*(c1*c2*s3+c1*c3*s2)))*(I65*c5*(c4*s1-s4*(c1*c2*c3-
c1*s2*s3))-I64*c6*s5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))+s5*s6*(c4*s1-s4*(c1*c2*c3-c1*s2*s3))*(I61*(1.0/2.0)-I62*
(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*c5*(c4*s1-s4*(c1*c2*c3-c1*s2*s3)))+(s6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))+c6*s4*(c2*s3+c3*s2))*(I66*c5*s4*(c2*s3+c3*s2)-c6*s4*s5*(c2*s3+c3*s2)*(I61*(-1.0/2.0)+I62*
(1.0/2.0)+I63*(1.0/2.0))+I64*s4*s5*s6*(c2*s3+c3*s2)+d6*m6*x6*c5*s4*(c2*s3+c3*s2))+(c6*(s5*(c2*c3-s2*s3)+c4*c5*
(c2*s3+c3*s2))-s4*s6*(c2*s3+c3*s2))*(I65*c5*s4*(c2*s3+c3*s2)-I64*c6*s4*s5*(c2*s3+c3*s2)+s4*s5*s6*(c2*s3+c3*s2)*
(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*c5*s4*(c2*s3+c3*s2))+(c6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-s6*
(c5*(c1*s4+c4*(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I66*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-c6*s5*(c1*c4-
s4*(s1*s2*s3-c2*c3*s1))*(I61*(-1.0/2.0)+I62*(1.0/2.0)+I63*(1.0/2.0))+I64*s5*s6*(c1*c4-s4*(s1*s2*s3-
c2*c3*s1))+d6*m6*x6*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1)))-(s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))+c6*(c5*(c1*s4+c4*
(s1*s2*s3-c2*c3*s1))+s5*(c2*s1*s3+c3*s1*s2)))*(I65*c5*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))-I64*c6*s5*(c1*c4-s4*(s1*s2*s3
-c2*c3*s1))+s5*s6*(c1*c4-s4*(s1*s2*s3-c2*c3*s1))*(I61*(1.0/2.0)-I62*(1.0/2.0)+I63*(1.0/2.0))+d6*m6*y6*c5*(c1*c4-
s4*(s1*s2*s3-c2*c3*s1)));;
        C646= 0;
        C656=0;

        c_[0][0][0]=C111;c_[0][1][1]=C122;c_[0][2][2]=C133;c_[0][3][3]=C144;c_[0][4][4]=C155;c_[0][5][5]=C166;
        c_[0][0][1]=C112;c_[0][0][2]=C113;c_[0][0][3]=C114;c_[0][0][4]=C115;c_[0][0][5]=C116;
        c_[0][1][2]=C123;c_[0][1][3]=C124;c_[0][1][4]=C125;c_[0][1][5]=C126;
        c_[0][2][3]=C134;c_[0][2][4]=C135;c_[0][2][5]=C136;
        c_[0][3][4]=C145;c_[0][3][5]=C146;
        c_[0][4][5]=C156;
        c_[1][0][0]=C211;c_[1][1][1]=C222;c_[1][2][2]=C233;c_[1][3][3]=C244;c_[1][4][4]=C255;c_[1][5][5]=C266;
        c_[1][0][1]=C212;c_[1][0][2]=C213;c_[1][0][3]=C214;c_[1][0][4]=C215;c_[1][0][5]=C216;
        c_[1][1][2]=C223;c_[1][1][3]=C224;c_[1][1][4]=C225;c_[1][1][5]=C226;
        c_[1][2][3]=C234;c_[1][2][4]=C235;c_[1][2][5]=C236;
        c_[1][3][4]=C245;c_[1][3][5]=C246;
        c_[1][4][5]=C256;
        c_[2][0][0]=C311;c_[2][1][1]=C322;c_[2][2][2]=C333;c_[2][3][3]=C344;c_[2][4][4]=C355;c_[2][5][5]=C366;
        c_[2][0][1]=C312;c_[2][0][2]=C313;c_[2][0][3]=C314;c_[2][0][4]=C315;c_[2][0][5]=C316;
        c_[2][1][2]=C323;c_[2][1][3]=C324;c_[2][1][4]=C325;c_[2][1][5]=C326;
        c_[2][2][3]=C334;c_[2][2][4]=C335;c_[2][2][5]=C336;
        c_[2][3][4]=C345;c_[2][3][5]=C346;
        c_[2][4][5]=C356;
        c_[3][0][0]=C411;c_[3][1][1]=C422;c_[3][2][2]=C433;c_[3][3][3]=C444;c_[3][4][4]=C455;c_[3][5][5]=C466;
        c_[3][0][1]=C412;c_[3][0][2]=C413;c_[3][0][3]=C414;c_[3][0][4]=C415;c_[3][0][5]=C416;
        c_[3][1][2]=C423;c_[3][1][3]=C424;c_[3][1][4]=C425;c_[3][1][5]=C426;
        c_[3][2][3]=C434;c_[3][2][4]=C435;c_[3][2][5]=C436;
        c_[3][3][4]=C445;c_[3][3][5]=C446;
        c_[3][4][5]=C456;
        c_[4][0][0]=C511;c_[4][1][1]=C522;c_[4][2][2]=C533;c_[4][3][3]=C544;c_[4][4][4]=C555;c_[4][5][5]=C566;
        c_[4][0][1]=C512;c_[4][0][2]=C513;c_[4][0][3]=C514;c_[4][0][4]=C515;c_[4][0][5]=C516;
        c_[4][1][2]=C523;c_[4][1][3]=C524;c_[4][1][4]=C525;c_[4][1][5]=C526;
        c_[4][2][3]=C534;c_[4][2][4]=C535;c_[4][2][5]=C536;
        c_[4][3][4]=C545;c_[4][3][5]=C546;
        c_[4][4][5]=C556;
        c_[5][0][0]=C611;c_[5][1][1]=C622;c_[5][2][2]=C633;c_[5][3][3]=C644;c_[5][4][4]=C655;c_[5][5][5]=C666;
        c_[5][0][1]=C612;c_[5][0][2]=C613;c_[5][0][3]=C614;c_[5][0][4]=C615;c_[5][0][5]=C616;
        c_[5][1][2]=C623;c_[5][1][3]=C624;c_[5][1][4]=C625;c_[5][1][5]=C626;
        c_[5][2][3]=C634;c_[5][2][4]=C635;c_[5][2][5]=C636;
        c_[5][3][4]=C645;c_[5][3][5]=C646;
        c_[5][4][5]=C656;

        return true;

    }
    /*get the dynamic equation C*/
    bool DynamicsInterface::getC(int idx_i,int idx_j,int idx_k,const double q[MAXAXES],double &C)
    {
        int p=idx_i>idx_j?idx_i:idx_j;

        p=p>idx_k?p:idx_k;

        double Dijk=0;
        double Upjk[4][4];
        double tmp[4][4];
        double trans_Upi[4][4],TransUpiIp[4][4];

        for(int i=p;i<=MAXAXES;i++)
        {
            if(!getU3(i,idx_j,idx_k,q,Upjk))
            {
                return false;
            } 
            //if(!getU3(i,idx_i,0,q,Upi))
            //{
            //    return false;
            //}

            //getTranspose(Upi,trans_Upi);

            //if(!getpseudoI(i,Ip))
            //{
            //    return false;
            //}    
            //Multiply4444(Upjk, Ip, UpjkIp);
            Multiply4444(Upjk, TransUpiIp_[i-1][idx_i-1], tmp);
            //Multiply4444(Ip_[i-1],trans_Upi_[i-1][idx_i-1],TransUpiIp_[i-1][idx_i-1]);
            //Multiply4444(Upi_[i-1][idx_j-1],TransUpiIp_[i-1][idx_i-1],tmp);
            Dijk+=TraceMatrix(tmp);
        }
        C=Dijk;
        return true;
    }
    /*get the dynamic equation G*/
    bool DynamicsInterface::getG(int idx_i,const double q[MAXAXES],double& G)
    {
        double Di=0;
        double g[4]={0,0,-9.81,0};
        double r[4];//={,,,1};
        double tmp=0;

        if((idx_i<=0) || (idx_i>MAXAXES))
        {
            
            return false;
        }
        for(int i=idx_i;i<=MAXAXES;i++)
        {
            r[0]=robot_model_.r[i-1][0];
            r[1]=robot_model_.r[i-1][1];
            r[2]=robot_model_.r[i-1][2];
            r[3]=1;
            //if(!getU3(i,idx_i,0,q,Upi))
            //{
            //    return false;
            //}
            
            tmp=- robot_model_.m[i-1]*9.81*Upi_[i-1][idx_i-1][2][3] - robot_model_.m[i-1]*9.81*Upi_[i-1][idx_i-1][2][0]*r[0] - robot_model_.m[i-1]*9.81*Upi_[i-1][idx_i-1][2][1]*r[1] - robot_model_.m[i-1]*9.81*Upi_[i-1][idx_i-1][2][2]*r[2];
            Di+=tmp;
        } 

        G=-Di;
        return true;  
    }

    bool DynamicsInterface::computeMCG(const double q[MAXAXES])
    {
        clock_t start, finish;
        double Total_time;  
        
        //start = clock();   
        getMiddleArray(q);           
        /*for (int i=0;i<MAXAXES;i++)
        {   
            if(!getG(i+1,q,g_[i]))
            {
                //FST_INFO("---11");
                return false;
            }
            for(int j=0;j<MAXAXES;j++)
            {
                if(!getM(i+1,j+1,q,m_[i][j]))
                {
                    //FST_INFO("---12");
                    return false;
                }
            }
        }*/
        getSymbolicM(q);
        getSymbolicG(q);
        //finish = clock();
        //Total_time = (double)(finish-start) / CLOCKS_PER_SEC;
        //FST_INFO( "compute G & M:%f seconds/n", Total_time);

        //start = clock(); 
        getSymbolicC(q);
        /*
        for(int i=0;i<MAXAXES;i++)
        {
            if(i==0)
            {
                c_[0][0][0]=0;
                getC(i+1,2,2,q,c_[i][1][1]);
                getC(i+1,3,3,q,c_[i][2][2]);  
                getC(i+1,4,4,q,c_[i][3][3]);
                getC(i+1,5,5,q,c_[i][4][4]);
                getC(i+1,6,6,q,c_[i][5][5]);      
                getC(i+1,1,2,q,c_[i][0][1]);
                getC(i+1,1,3,q,c_[i][0][2]);  
                getC(i+1,1,4,q,c_[i][0][3]);
                getC(i+1,1,5,q,c_[i][0][4]);
                getC(i+1,1,6,q,c_[i][0][5]); 
                getC(i+1,2,3,q,c_[i][1][2]);  
                getC(i+1,2,4,q,c_[i][1][3]);
                getC(i+1,2,5,q,c_[i][1][4]);
                getC(i+1,2,6,q,c_[i][1][5]); 
                getC(i+1,3,4,q,c_[i][2][3]);
                getC(i+1,3,5,q,c_[i][2][4]);
                getC(i+1,3,6,q,c_[i][2][5]); 
                getC(i+1,4,5,q,c_[i][3][4]);
                getC(i+1,4,6,q,c_[i][3][5]); 
                getC(i+1,5,6,q,c_[i][4][5]); 
            }
            else if(i==1)
            {
                c_[1][1][1]=c_[1][0][1]=c_[1][0][2]=0;
                getC(i+1,1,1,q,c_[i][0][0]);
                getC(i+1,3,3,q,c_[i][2][2]);  
                getC(i+1,4,4,q,c_[i][3][3]);
                getC(i+1,5,5,q,c_[i][4][4]);     
                getC(i+1,1,4,q,c_[i][0][3]);
                getC(i+1,1,5,q,c_[i][0][4]);
                getC(i+1,1,6,q,c_[i][0][5]); 
                getC(i+1,2,3,q,c_[i][1][2]);  
                getC(i+1,2,4,q,c_[i][1][3]);
                getC(i+1,2,5,q,c_[i][1][4]);
                getC(i+1,2,6,q,c_[i][1][5]); 
                getC(i+1,3,4,q,c_[i][2][3]);
                getC(i+1,3,5,q,c_[i][2][4]);
                getC(i+1,3,6,q,c_[i][2][5]); 
                getC(i+1,4,5,q,c_[i][3][4]);
                getC(i+1,4,6,q,c_[i][3][5]); 
                getC(i+1,5,6,q,c_[i][4][5]);  

            }
            else if(i==2)
            {
                c_[2][2][2]=c_[2][0][1]=c_[2][0][2]=c_[2][1][2]=0;
                getC(i+1,1,1,q,c_[i][0][0]);
                getC(i+1,2,2,q,c_[i][1][1]);
                getC(i+1,4,4,q,c_[i][3][3]);
                getC(i+1,5,5,q,c_[i][4][4]);
                getC(i+1,6,6,q,c_[i][5][5]);      
                getC(i+1,1,4,q,c_[i][0][3]);
                getC(i+1,1,5,q,c_[i][0][4]);
                getC(i+1,1,6,q,c_[i][0][5]); 
                getC(i+1,2,4,q,c_[i][1][3]);
                getC(i+1,2,5,q,c_[i][1][4]);
                getC(i+1,2,6,q,c_[i][1][5]); 
                getC(i+1,3,4,q,c_[i][2][3]);
                getC(i+1,3,5,q,c_[i][2][4]);
                getC(i+1,3,6,q,c_[i][2][5]); 
                getC(i+1,4,5,q,c_[i][3][4]);
                getC(i+1,4,6,q,c_[i][3][5]); 
                getC(i+1,5,6,q,c_[i][4][5]);                 
            }
            else if(i==3)
            {
                c_[3][3][3]=c_[3][0][3]=c_[3][1][3]=c_[3][2][3]=0;
                getC(i+1,1,1,q,c_[i][0][0]);
                getC(i+1,2,2,q,c_[i][1][1]);
                getC(i+1,3,3,q,c_[i][2][2]);  
                getC(i+1,5,5,q,c_[i][4][4]);
                getC(i+1,6,6,q,c_[i][5][5]);      
                getC(i+1,1,2,q,c_[i][0][1]);
                getC(i+1,1,3,q,c_[i][0][2]);  
                getC(i+1,1,5,q,c_[i][0][4]);
                getC(i+1,1,6,q,c_[i][0][5]); 
                getC(i+1,2,3,q,c_[i][1][2]);  
                getC(i+1,2,5,q,c_[i][1][4]);
                getC(i+1,2,6,q,c_[i][1][5]); 
                getC(i+1,3,5,q,c_[i][2][4]);
                getC(i+1,3,6,q,c_[i][2][5]); 
                getC(i+1,4,5,q,c_[i][3][4]);
                getC(i+1,4,6,q,c_[i][3][5]); 
                getC(i+1,5,6,q,c_[i][4][5]);                 
            }   
            else if(i==4)
            {
                c_[4][4][4]=c_[4][0][4]=c_[4][1][4]=c_[4][2][4]=c_[4][3][4]=0;
                getC(i+1,1,1,q,c_[i][0][0]);
                getC(i+1,2,2,q,c_[i][1][1]);
                getC(i+1,3,3,q,c_[i][2][2]);  
                getC(i+1,4,4,q,c_[i][3][3]);
                getC(i+1,6,6,q,c_[i][5][5]);      
                getC(i+1,1,2,q,c_[i][0][1]);
                getC(i+1,1,3,q,c_[i][0][2]);  
                getC(i+1,1,4,q,c_[i][0][3]);
                getC(i+1,1,6,q,c_[i][0][5]); 
                getC(i+1,2,3,q,c_[i][1][2]);  
                getC(i+1,2,4,q,c_[i][1][3]);
                getC(i+1,2,6,q,c_[i][1][5]); 
                getC(i+1,3,4,q,c_[i][2][3]);
                getC(i+1,3,6,q,c_[i][2][5]); 
                getC(i+1,4,6,q,c_[i][3][5]); 
                getC(i+1,5,6,q,c_[i][4][5]);                 
            }         
            else if(i==5)
            {
                c_[5][5][5]=c_[5][0][5]=c_[5][1][5]=c_[5][2][5]=c_[5][3][5]=c_[5][4][5]=0;
                getC(i+1,1,1,q,c_[i][0][0]);
                getC(i+1,2,2,q,c_[i][1][1]);
                getC(i+1,3,3,q,c_[i][2][2]);  
                getC(i+1,4,4,q,c_[i][3][3]);
                getC(i+1,5,5,q,c_[i][4][4]);   
                getC(i+1,1,2,q,c_[i][0][1]);
                getC(i+1,1,3,q,c_[i][0][2]);  
                getC(i+1,1,4,q,c_[i][0][3]);
                getC(i+1,1,5,q,c_[i][0][4]);
                getC(i+1,2,3,q,c_[i][1][2]);  
                getC(i+1,2,4,q,c_[i][1][3]);
                getC(i+1,2,5,q,c_[i][1][4]);
                getC(i+1,3,4,q,c_[i][2][3]);
                getC(i+1,3,5,q,c_[i][2][4]);
                getC(i+1,4,5,q,c_[i][3][4]);              
            }

        }*/

        //finish = clock();
        //Total_time = (double)(finish-start) / CLOCKS_PER_SEC;
        //FST_INFO( "compute C %f seconds/n", Total_time);
        return true;
    }
    /*compute the torque of each joint*/
    bool DynamicsInterface::getT(const double q[MAXAXES],const double dq[MAXAXES],const double ddq[MAXAXES],double T[MAXAXES])
    {
        if(!computeMCG(q))
            return false;

        for(int i=0;i<MAXAXES;i++)
        {
            T[i]=m_[i][0]*ddq[0]+m_[i][1]*ddq[1]+m_[i][2]*ddq[2]+m_[i][3]*ddq[3]+m_[i][4]*ddq[4]+m_[i][5]*ddq[5]+\
              c_[i][0][0]*dq[0]*dq[0]+c_[i][1][1]*dq[1]*dq[1]+c_[i][2][2]*dq[2]*dq[2]+c_[i][3][3]*dq[3]*dq[3]+c_[i][4][4]*dq[4]*dq[4]+c_[i][5][5]*dq[5]*dq[5]+\
            2*c_[i][0][1]*dq[0]*dq[1]+2*c_[i][0][2]*dq[0]*dq[2]+2*c_[i][0][3]*dq[0]*dq[3]+2*c_[i][0][4]*dq[0]*dq[4]+2*c_[i][0][5]*dq[0]*dq[5]+\
            2*c_[i][1][2]*dq[1]*dq[2]+2*c_[i][1][3]*dq[1]*dq[3]+2*c_[i][1][4]*dq[1]*dq[4]+2*c_[i][1][5]*dq[1]*dq[5]+\
            2*c_[i][2][3]*dq[2]*dq[3]+2*c_[i][2][4]*dq[2]*dq[4]+2*c_[i][2][5]*dq[2]*dq[5]+\
            2*c_[i][3][4]*dq[3]*dq[4]+2*c_[i][3][5]*dq[3]*dq[5]+\
            2*c_[i][4][5]*dq[4]*dq[5]+g_[i];
            //FST_INFO("T[%d]=%f\n",i,T[i]);
        }
        return true;

    }
    /*compute the secondary torque*/
    bool DynamicsInterface::getTm(const double q[MAXAXES],const double dq[MAXAXES],double Tm[MAXAXES])
    {
        clock_t start, finish;
        double Total_time;

        //start = clock();        
        if(!computeMCG(q))
            return false;        
        //finish = clock();
        //Total_time = (double)(finish-start) / CLOCKS_PER_SEC;
        //FST_INFO( "compute MCG %f seconds/n", Total_time);
        for(int i=0;i<MAXAXES;i++)
        {
            Tm[i]=c_[i][0][0]*dq[0]*dq[0]+c_[i][1][1]*dq[1]*dq[1]+c_[i][2][2]*dq[2]*dq[2]+c_[i][3][3]*dq[3]*dq[3]+c_[i][4][4]*dq[4]*dq[4]+c_[i][5][5]*dq[5]*dq[5]+\
            2*c_[i][0][1]*dq[0]*dq[1]+2*c_[i][0][2]*dq[0]*dq[2]+2*c_[i][0][3]*dq[0]*dq[3]+2*c_[i][0][4]*dq[0]*dq[4]+2*c_[i][0][5]*dq[0]*dq[5]+\
            2*c_[i][1][2]*dq[1]*dq[2]+2*c_[i][1][3]*dq[1]*dq[3]+2*c_[i][1][4]*dq[1]*dq[4]+2*c_[i][1][5]*dq[1]*dq[5]+\
            2*c_[i][2][3]*dq[2]*dq[3]+2*c_[i][2][4]*dq[2]*dq[4]+2*c_[i][2][5]*dq[2]*dq[5]+\
            2*c_[i][3][4]*dq[3]*dq[4]+2*c_[i][3][5]*dq[3]*dq[5]+\
            2*c_[i][4][5]*dq[4]*dq[5]+g_[i];

        }
        return true;        
    }
    //-----------------------------------------------  
    //功能: 求矩???n*n)的行列式  
    //入口参数: 矩阵的首地址，矩阵的行数  
    //返回??? 矩阵的行列式??? 
    //----------------------------------------------  
    float DynamicsInterface::MatDet(float *p, int n)  
    {  
        int r, c, m;  
        int lop = 0;  
        float result = 0;  
        float mid = 1;  
      
        if (n != 1)  
        {  
            lop = (n == 2) ? 1 : n;            //控制求和循环次数,若为2阶，则循???次，否则为n??? 
            for (m = 0; m < lop; m++)  
            {  
                mid = 1;            //顺序求和, 主对角线元素相乘之和  
                for (r = 0, c = m; r < n; r++, c++)  
                {  
                    mid = mid * (*(p + r*n + c%n));  
                }  
                result += mid;  
            }  
            for (m = 0; m < lop; m++)  
            {  
                mid = 1;            //逆序相减, 减去次对角线元素乘积  
                for (r = 0, c = n - 1 - m + n; r < n; r++, c--)  
                {  
                    mid = mid * (*(p + r*n + c%n));  
                }  
                result -= mid;  
            }  
        }  
        else  
            result = *p;  
        return result;  
    } 
    //----------------------------------------------------------------------------  
    //功能: 求k*k矩阵中元素A(m, n)的代数余之式  
    //入口参数: k*k矩阵的首地址，矩阵元素A的下标m,n,矩阵行数k  
    //返回??? k*k矩阵中元素A(m, n)的代数余之式  
    //----------------------------------------------------------------------------  
    float DynamicsInterface::Creat_M(float *p, int m, int n, int k)  
    {  
        int len;  
        int i, j;  
        float mid_result = 0;  
        int sign = 1;  
        float *p_creat, *p_mid;  
      
        len = (k - 1)*(k - 1);            //k阶矩阵的代数余之式为k-1阶矩??? 
        p_creat = (float*)calloc(len, sizeof(float)); //分配内存单元  
        p_mid = p_creat;  
        for (i = 0; i < k; i++)  
        {  
            for (j = 0; j < k; j++)  
            {  
                if (i != m && j != n) //将除第i行和第j列外的所有元素存储到以p_mid为首地址的内存单??? 
                {  
                    *p_mid++ = *(p + i*k + j);  
                }  
            }  
        }  
        sign = (m + n) % 2 == 0 ? 1 : -1;    //代数余之式前面的正、负??? 
        mid_result = (float)sign*MatDet(p_creat, k - 1);  
        free(p_creat);  
        return mid_result;  
    }  
    //------------------------------------------------------------------  
    //功能: 采用部分主元的高斯消去法求方阵A的逆矩阵B  
    //入口参数: 输入方阵，输出方阵，方阵阶数  
    //返回??? true or false  
    //-------------------------------------------------------------------  
    bool DynamicsInterface::Gauss(const double A[MAX_AXES][MAX_AXES], double B[MAX_AXES][MAX_AXES])  
    {  
        int i, j, k;  
        double max, temp;  
        double t[MAX_AXES][MAX_AXES];                //临时矩阵  
        //将A矩阵存放在临时矩阵t[n][n]??? 
        for (i = 0; i < MAX_AXES; i++)  
        {  
            for (j = 0; j < MAX_AXES; j++)  
            {  
                t[i][j] = A[i][j];  
            }  
        }  
        //初始化B矩阵为单位阵  
        for (i = 0; i < MAX_AXES; i++)  
        {  
            for (j = 0; j < MAX_AXES; j++)  
            {  
                B[i][j] = (i == j) ? (float)1 : 0;  
            }  
        }  
        for (i = 0; i < MAX_AXES; i++)  
        {  
            //寻找主元  
            max = t[i][i];  
            k = i;  
            for (j = i + 1; j < MAX_AXES; j++)  
            {  
                if (fabs(t[j][i]) > fabs(max))  
                {  
                    max = t[j][i];  
                    k = j;  
                }  
            }  
            //如果主元所在行不是第i行，进行行交??? 
            if (k != i)  
            {  
                for (j = 0; j < MAX_AXES; j++)  
                {  
                    temp = t[i][j];  
                    t[i][j] = t[k][j];  
                    t[k][j] = temp;  
                    //B伴随交换  
                    temp = B[i][j];  
                    B[i][j] = B[k][j];  
                    B[k][j] = temp;  
                }  
            }  
            //判断主元是否???, 若是, 则矩阵A不是满秩矩阵,不存在逆矩??? 
            if (t[i][i] == 0)  
            {  
                cout << "There is no inverse matrix!";  
                return false;  
            }  
            //消去A的第i列除去i行以外的各行元素  
            temp = t[i][i];  
            for (j = 0; j < MAX_AXES; j++)  
            {  
                t[i][j] = t[i][j] / temp;        //主对角线上的元素变为1  
                B[i][j] = B[i][j] / temp;        //伴随计算  
            }  
            for (j = 0; j < MAX_AXES; j++)        //??????>第n??? 
            {  
                if (j != i)                //不是第i??? 
                {  
                    temp = t[j][i];  
                    for (k = 0; k < MAX_AXES; k++)        //第j行元???- i行元???j列i行元??? 
                    {  
                        t[j][k] = t[j][k] - t[i][k] * temp;  
                        B[j][k] = B[j][k] - B[i][k] * temp;  
                    }  
                }  
            }  
        }  
        return true;  
    }  
    /*get the inverse matrix of M*/
    bool DynamicsInterface::getInverseM(const double M[MAXAXES][MAXAXES],double IM[MAXAXES][MAXAXES])
    {

        //运用高斯消去法求该矩阵的逆矩阵并输出  
        if (!Gauss(M, IM))  
        {  
            return false;  
        }   

        return true;
    }
    /* 6*6 Matrix Multiple 6*1 vector*/
    bool DynamicsInterface::getMtxMulVec(const double IM[MAXAXES][MAXAXES],const double V[MAXAXES],double Res[MAXAXES])
    {
         Res[0]=IM[0][0]*V[0] + IM[0][1]*V[1] + IM[0][2]*V[2] + IM[0][3]*V[3] + IM[0][4]*V[4] + IM[0][5]*V[5];
         Res[1]=IM[1][0]*V[0] + IM[1][1]*V[1] + IM[1][2]*V[2] + IM[1][3]*V[3] + IM[1][4]*V[4] + IM[1][5]*V[5];
         Res[2]=IM[2][0]*V[0] + IM[2][1]*V[1] + IM[2][2]*V[2] + IM[2][3]*V[3] + IM[2][4]*V[4] + IM[2][5]*V[5];
         Res[3]=IM[3][0]*V[0] + IM[3][1]*V[1] + IM[3][2]*V[2] + IM[3][3]*V[3] + IM[3][4]*V[4] + IM[3][5]*V[5];
         Res[4]=IM[4][0]*V[0] + IM[4][1]*V[1] + IM[4][2]*V[2] + IM[4][3]*V[3] + IM[4][4]*V[4] + IM[4][5]*V[5];
         Res[5]=IM[5][0]*V[0] + IM[5][1]*V[1] + IM[5][2]*V[2] + IM[5][3]*V[3] + IM[5][4]*V[4] + IM[5][5]*V[5];

         return true;
    }

    /*
    Description: set servo parameters of all axes
    input: servo_model[]
    output: none
    return: true -> all parameters all valid
            false-> one or more parameters are invalid
    */
    bool DynamicsInterface::initServoModel(const ServelModel sv_model[MAXAXES])
    {
        //double centre_of_mass[3]; // the centre of the mass of the motor
        //double inertia_sensor[3][3]; //inertia sensor of the motor
        //double mass_of_motor; //mass of the motor
        //double jm;  //inertia of the motor 
        //double gr;  //gear ratio
        for(int i=0;i<MAXAXES;i++)
        {
            servo_model_[i].rated_torque=sv_model[i].rated_torque;
            servo_model_[i].mass_of_motor=sv_model[i].mass_of_motor;
            servo_model_[i].jm=sv_model[i].jm;
            servo_model_[i].gr=sv_model[i].gr;
            for(int j=0;j<3;j++)
            {
                servo_model_[i].centre_of_mass[j]=sv_model[i].centre_of_mass[j];
                for(int k=0;k<3;k++)
                {
                    servo_model_[i].inertia_sensor[j][k]=sv_model[i].inertia_sensor[j][k];
                }
            }

        }
        is_servo_model_ready_=true;

        return is_servo_model_ready_;
    }
   /*
    Description: set robot parameters
    input: robot_model
    output: none
    return: true -> all parameters all valid
            false-> one or more parameters are invalid
    */
    bool DynamicsInterface::initRobotModel(const RobotModel& rob_model)
    {
        //std::string model_name; //name of the robot model
        //double a[MAXAXES]; //a
        //double d[MAXAXES]; //d
        //double ap[MAXAXES]; //alpha
        //double offset[MAXAXES];//offset
        //double r[MAXAXES][3]; //centre of link
        //double i[MAXAXES][6]; //tensor of inertia
        //double m[MAXAXES]; //mass of link
        //bool mdh; //true:std-dh false:mod-dh
        //double b[MAXAXES]; //joint friction  coefficient
        //double tc[MAXAXES][2]; //Coulomb friction coefficient

        robot_model_.mdh=rob_model.mdh;

        for(int i=0;i<MAXAXES;i++)
        {
            robot_model_.a[i]=rob_model.a[i];
            robot_model_.d[i]=rob_model.d[i];
            robot_model_.ap[i]=rob_model.ap[i];
            robot_model_.offset[i]=rob_model.offset[i];
            robot_model_.m[i]=rob_model.m[i];
            robot_model_.b[i]=rob_model.b[i];

            for(int j=0;j<3;j++)
            {
                robot_model_.r[i][j]=rob_model.r[i][j];
            }
            for(int j=0;j<2;j++)
            {
                robot_model_.tc[i][j]=rob_model.tc[i][j];
            }
            for(int j=0;j<6;j++)
            {
                for(int k=0;k<6;k++)
                {
                    robot_model_.i[i][j][k]=rob_model.i[i][j][k];
                }
                
            }
        }
        is_robot_model_ready_=true;

        return is_robot_model_ready_;
    }

    void DynamicsInterface::setRatedTorque(double torque[6])
    {
        for(int i=0; i<6; ++i)
        {
            servo_model_[i].rated_torque = torque[i];
        }
    }
    /*
    Description: if the dynamics interface can work correctly.
                 judgement condition is (is_robot_model_ready && is_servo_model_ready)
    input: none
    output: none
    return: true -> the dynamics interface is ready to use
            false-> robot model or servo model are not be initialized
    */
    bool DynamicsInterface::isDynamicsReady()
    {
        return is_servo_model_ready_ && is_robot_model_ready_;
    }
    /*
    Description: get the max possible acceraltion of all axes 
                 at current joint/omega/alpha status
    input: joint: current position in joint space
           omega: current velocity in joint space
    output: alpha_max: the maximum acceraltion in joint space
    return: true -> alpha_max is avaiable
            false-> alpha_max is not avaiable, error happened in computation 
    */
    bool DynamicsInterface::computeAccMax(const double joint[MAXAXES], const double omega[MAXAXES], double alpha_max[2][MAXAXES])
    {
        clock_t start, finish;
        double Total_time;

        //FST_INFO("---1");
        double Tm[MAXAXES];
        double diffT[2][MAXAXES];
        double IB[MAXAXES][MAXAXES];

        for(int i=0;i<MAXAXES;i++)
        {
            q_[i]=joint[i];
            dq_[i]=omega[i];
        }
        //FST_INFO("---2");

        //start = clock();
        if(!getTm(q_,dq_,Tm))
            return false;
        //FST_INFO("---3");
        //finish = clock();
        //Total_time = (double)(finish-start) / CLOCKS_PER_SEC;
        //FST_INFO( "getTm:%f seconds/n", Total_time);

        for(int i=0;i<MAXAXES;i++)
        {
            diffT[0][i]=servo_model_[i].rated_torque-Tm[i];
            diffT[1][i]=-servo_model_[i].rated_torque-Tm[i];
        }
        //FST_INFO("---4");
        //start = clock();
        if(!getInverseM(m_,IB))
            return false;
        //finish = clock();
        //Total_time = (double)(finish-start) / CLOCKS_PER_SEC;
        //FST_INFO( "getInverseM:%f seconds/n", Total_time);        
        //FST_INFO("---5");

        //start=clock();
        if(!getMtxMulVec(IB,diffT[0],alpha_max[0]))
            return false;
        if(!getMtxMulVec(IB,diffT[1],alpha_max[1]))
            return false;
        //FST_INFO("---6");
        //finish = clock();
        //Total_time = (double)(finish-start) / CLOCKS_PER_SEC;
        //FST_INFO( "getMtxMulVec:%f seconds/n", Total_time);
        //for(int j=0;j<2;j++)
        //{
        //    for(int i=0;i<MAX_AXES;i++)
        //    {
        //       FST_INFO("acc[%d][%d]=%f\n",j+1,i+1,alpha_max[j][i]);
        //    }            
        //}


        return true;
    }
        /* 4*4 matrix multiply 4*4 4*4 matrix*/
    bool DynamicsInterface::Multiply4444(double a[4][4],double b[4][4],double c[4][4])
    {
        //[ a00*b00 + a01*b10 + a02*b20 + a03*b30, a00*b01 + a01*b11 + a02*b21 + a03*b31, a00*b02 + a01*b12 + a02*b22 + a03*b32, a00*b03 + a01*b13 + a02*b23 + a03*b33]
        //[ a10*b00 + a11*b10 + a12*b20 + a13*b30, a10*b01 + a11*b11 + a12*b21 + a13*b31, a10*b02 + a11*b12 + a12*b22 + a13*b32, a10*b03 + a11*b13 + a12*b23 + a13*b33]
        //[ a20*b00 + a21*b10 + a22*b20 + a23*b30, a20*b01 + a21*b11 + a22*b21 + a23*b31, a20*b02 + a21*b12 + a22*b22 + a23*b32, a20*b03 + a21*b13 + a22*b23 + a23*b33]
        //[ a30*b00 + a31*b10 + a32*b20 + a33*b30, a30*b01 + a31*b11 + a32*b21 + a33*b31, a30*b02 + a31*b12 + a32*b22 + a33*b32, a30*b03 + a31*b13 + a32*b23 + a33*b33]

        c[0][0]=a[0][0]*b[0][0] + a[0][1]*b[1][0] + a[0][2]*b[2][0]+a[0][3]*b[3][0];
        c[0][1]=a[0][0]*b[0][1] + a[0][1]*b[1][1] + a[0][2]*b[2][1]+a[0][3]*b[3][1];
        c[0][2]=a[0][0]*b[0][2] + a[0][1]*b[1][2] + a[0][2]*b[2][2]+a[0][3]*b[3][2];  
        c[0][3]=a[0][0]*b[0][3] + a[0][1]*b[1][3] + a[0][2]*b[2][3]+a[0][3]*b[3][3];  

        c[1][0]=a[1][0]*b[0][0] + a[1][1]*b[1][0] + a[1][2]*b[2][0]+ a[1][3]*b[3][0];
        c[1][1]=a[1][0]*b[0][1] + a[1][1]*b[1][1] + a[1][2]*b[2][1]+ a[1][3]*b[3][1];
        c[1][2]=a[1][0]*b[0][2] + a[1][1]*b[1][2] + a[1][2]*b[2][2]+ a[1][3]*b[3][2];
        c[1][3]=a[1][0]*b[0][3] + a[1][1]*b[1][3] + a[1][2]*b[2][3]+ a[1][3]*b[3][3];

        c[2][0]=a[2][0]*b[0][0] + a[2][1]*b[1][0] + a[2][2]*b[2][0]+ a[2][3]*b[3][0];
        c[2][1]=a[2][0]*b[0][1] + a[2][1]*b[1][1] + a[2][2]*b[2][1]+ a[2][3]*b[3][1];
        c[2][2]=a[2][0]*b[0][2] + a[2][1]*b[1][2] + a[2][2]*b[2][2]+ a[2][3]*b[3][2];   
        c[2][3]=a[2][0]*b[0][3] + a[2][1]*b[1][3] + a[2][2]*b[2][3]+ a[2][3]*b[3][3];

        c[3][0]=a[3][0]*b[0][0] + a[3][1]*b[1][0] + a[3][2]*b[2][0]+ a[3][3]*b[3][0];
        c[3][1]=a[3][0]*b[0][1] + a[3][1]*b[1][1] + a[3][2]*b[2][1]+ a[3][3]*b[3][1];
        c[3][2]=a[3][0]*b[0][2] + a[3][1]*b[1][2] + a[3][2]*b[2][2]+ a[3][3]*b[3][2];   
        c[3][3]=a[3][0]*b[0][3] + a[3][1]*b[1][3] + a[3][2]*b[2][3]+ a[3][3]*b[3][3];

        return true;        
        
    }
    /* 3*3 matrix multiply 3*3 matrix*/
    bool DynamicsInterface::Multiply3333(double a[3][3],double b[3][3],double c[3][3])
    {
        //[ a11*b11 + a12*b21 + a13*b31, a11*b12 + a12*b22 + a13*b32, a11*b13 + a12*b23 + a13*b33]
        //[ a21*b11 + a22*b21 + a23*b31, a21*b12 + a22*b22 + a23*b32, a21*b13 + a22*b23 + a23*b33]
        //[ a31*b11 + a32*b21 + a33*b31, a31*b12 + a32*b22 + a33*b32, a31*b13 + a32*b23 + a33*b33]        

        c[0][0]=a[0][0]*b[0][0] + a[0][1]*b[1][0] + a[0][2]*b[2][0];
        c[0][1]=a[0][0]*b[0][1] + a[0][1]*b[1][1] + a[0][2]*b[2][1];
        c[0][2]=a[0][0]*b[0][2] + a[0][1]*b[1][2] + a[0][2]*b[2][2];     
        c[1][0]=a[1][0]*b[0][0] + a[1][1]*b[1][0] + a[1][2]*b[2][0];
        c[1][1]=a[1][0]*b[0][1] + a[1][1]*b[1][1]+ a[1][2]*b[2][1];
        c[1][2]=a[1][0]*b[0][2] + a[1][1]*b[1][2] + a[1][2]*b[2][2];
        c[2][0]=a[2][0]*b[0][0] + a[2][1]*b[1][0] + a[2][2]*b[2][0];
        c[2][1]=a[2][0]*b[0][1] + a[2][1]*b[1][1] + a[2][2]*b[2][1];
        c[2][2]=a[2][0]*b[0][2] + a[2][1]*b[1][2] + a[2][2]*b[2][2];   

        return true;
    }
    /*
    Description: get the current inertia of all axes in view of motor shaft.
                 'current' means the status when we calling function computeAccMax();
    input: none
    output: inertia: the current inertia
    return: true -> inertia is avaiable
            false-> inertia is not avaiable, error happened in computation 
    */    
    bool DynamicsInterface::getInertia(const double q[MAXAXES],double inertia[MAXAXES])
    {
        double A[4][4],TA[4][4];
        double R[3][3],TR[3][3];
        double IT[3][3];
        double Re[3][3];
        double I[4][4]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};

        for(int i=0;i<MAXAXES;i++)
        {
            if(!getA(i+1,q,A))
                return false;
            if(!Multiply4444(I,A,I))
                return false;

            if(!getR(I,R))
                return false;  

            if(!getTranspose(I,TA))
                return false;
            if(!getR(TA,TR))
                return false;

            if(!Multiply3333(R,robot_model_.i[i],IT))
                return false;

            if(!Multiply3333(IT,TR,Re))
                return false;

            inertia[i]=Re[0][0]+Re[1][1]+Re[2][2];

            //FST_INFO("inertia[%d]=%f\n",i,inertia[i]);
        }
      
        return true;
    }
    /*
    Description: get the current counter torque of all axes in view of motor shaft.
                 'current' means the status when we calling function computeAccMax();
    input: alpha: current accerlation
    output: counter_torque: the current counter torque
    return: true -> counter_torque is avaiable
            false-> counter_torque is not avaiable, error happened in computation 
    */  
    bool DynamicsInterface::getCounterTorque(const double alpha[MAXAXES], double counter_torque[MAXAXES])
    {
         if(!getT(q_,dq_,alpha,counter_torque))
            return false;
         //for(int i=0;i<MAXAXES;i++)
         //   FST_INFO("counter_torque[%d]=%f\n",i,counter_torque[i]);
         return true;
    }   
    /*  deconstructed function */
    DynamicsInterface::~DynamicsInterface()
    {
        return;
    }
    /*  constructed function */
    DynamicsInterface::DynamicsInterface()
    {
        //default value
        robot_model_.model_name="fst_robotics_dynamic_model";
        
        memset(robot_model_.a,0,sizeof(robot_model_.a));
        robot_model_.a[0]=0.03;
        robot_model_.a[1]=0.34;
        robot_model_.a[2]=0.34;
        
        memset(robot_model_.d,0,sizeof(robot_model_.d));
        robot_model_.d[0]=0.365;
        robot_model_.d[3]=0.35;
        robot_model_.d[5]=0.0965;
        
        memset(robot_model_.ap,0,sizeof(robot_model_.ap));
        robot_model_.ap[0]=M_PI/2;
        robot_model_.ap[2]=M_PI/2;
        robot_model_.ap[3]=-M_PI/2;
        robot_model_.ap[4]=M_PI/2;
        
        memset(robot_model_.offset,0,sizeof(robot_model_.offset));
        robot_model_.offset[1]=M_PI/2;
        
        robot_model_.r[0][0]=-0.011566;
        robot_model_.r[0][1]=0.114118;
        robot_model_.r[0][2]=-0.000213;
        robot_model_.r[1][0]=0.00622;
        robot_model_.r[1][1]=0.272354;
        robot_model_.r[1][2]=-0.011401;
        robot_model_.r[2][0]=-0.00496;
        robot_model_.r[2][1]=0.08564;
        robot_model_.r[2][2]=-0.02475;
        robot_model_.r[3][0]=0.016623;
        robot_model_.r[3][1]=0.025455;
        robot_model_.r[3][2]=-0.016126;
        robot_model_.r[4][0]=-0.000363;
        robot_model_.r[4][1]=-0.001235;
        robot_model_.r[4][2]=-0.128822;
        robot_model_.r[5][0]=0.02624;
        robot_model_.r[5][1]=-0.000009;
        robot_model_.r[5][2]=-0.002562;

        robot_model_.i[0][0][0]=0.086797;
        robot_model_.i[0][1][1]=0.077083;
        robot_model_.i[0][2][2]=0.10396;
        robot_model_.i[0][0][1]=0.001279;
        robot_model_.i[0][1][2]=-0.000094;
        robot_model_.i[0][0][2]=0.000326;
        robot_model_.i[0][1][0]=robot_model_.i[0][0][1];
        robot_model_.i[0][2][1]=robot_model_.i[0][1][2];
        robot_model_.i[0][2][0]=robot_model_.i[0][0][2];

        robot_model_.i[1][0][0]=0.066751;
        robot_model_.i[1][1][1]=0.054766;
        robot_model_.i[1][2][2]=0.04449;
        robot_model_.i[1][0][1]=0.004723;
        robot_model_.i[1][1][2]=-0.006461;
        robot_model_.i[1][0][2]=-0.000977;
        robot_model_.i[1][1][0]=robot_model_.i[1][0][1];
        robot_model_.i[1][2][1]=robot_model_.i[1][1][2];
        robot_model_.i[1][2][0]=robot_model_.i[1][0][2];

        robot_model_.i[2][0][0]=0.257019;
        robot_model_.i[2][1][1]=0.07705;
        robot_model_.i[2][2][2]=0.212872;
        robot_model_.i[2][0][1]=-0.004;
        robot_model_.i[2][1][2]=0.012631;
        robot_model_.i[2][0][2]=-0.000938;
        robot_model_.i[2][1][0]=robot_model_.i[2][0][1];
        robot_model_.i[2][2][1]=robot_model_.i[2][1][2];
        robot_model_.i[2][2][0]=robot_model_.i[2][0][2];

        robot_model_.i[3][0][0]=0.037799;
        robot_model_.i[3][1][1]=0.047205;
        robot_model_.i[3][2][2]=0.043194;
        robot_model_.i[3][0][1]=0.00067;
        robot_model_.i[3][1][2]=0.002673;
        robot_model_.i[3][0][2]=0.002746;
        robot_model_.i[3][1][0]=robot_model_.i[3][0][1];
        robot_model_.i[3][2][1]=robot_model_.i[3][1][2];
        robot_model_.i[3][2][0]=robot_model_.i[3][0][2];

        robot_model_.i[4][0][0]=0.027242;
        robot_model_.i[4][1][1]=0.029597;
        robot_model_.i[4][2][2]=0.014197;
        robot_model_.i[4][0][1]=0.000098;
        robot_model_.i[4][1][2]=-0.000012;
        robot_model_.i[4][0][2]=-0.000245;
        robot_model_.i[4][1][0]=robot_model_.i[4][0][1];
        robot_model_.i[4][2][1]=robot_model_.i[4][1][2];
        robot_model_.i[4][2][0]=robot_model_.i[4][0][2];

        robot_model_.i[5][0][0]=0.00187;
        robot_model_.i[5][1][1]=0.004282;
        robot_model_.i[5][2][2]=0.003963;
        robot_model_.i[5][0][1]=-0.000002;
        robot_model_.i[5][1][2]=0;
        robot_model_.i[5][0][2]=0.000148;
        robot_model_.i[5][1][0]=robot_model_.i[5][0][1];
        robot_model_.i[5][2][1]=robot_model_.i[5][1][2];
        robot_model_.i[5][2][0]=robot_model_.i[5][0][2];

        robot_model_.m[0]=12.378129;
        robot_model_.m[1]=5.077933;
        robot_model_.m[2]=10.913478;
        robot_model_.m[3]=9.226203;
        robot_model_.m[4]=5.396943;
        robot_model_.m[5]=2.174118;

        robot_model_.mdh=true;
        memset(robot_model_.b,0,sizeof(robot_model_.b));  
        memset(robot_model_.tc,0,sizeof(robot_model_.tc));   
    
        servo_model_[0].rated_torque=7.16*81;
        servo_model_[0].jm=0.000013;
        servo_model_[0].gr=81;
        servo_model_[1].rated_torque=5.73*121;
        servo_model_[1].jm=0.000059;
        servo_model_[1].gr=121;  
        servo_model_[2].rated_torque=3.82*81;
        servo_model_[2].jm=0.000044;
        servo_model_[2].gr=81;      
        servo_model_[3].rated_torque=1.91*51.0*40.0/38.0;
        servo_model_[3].jm=0.000018;
        servo_model_[3].gr=51*40/38;
        servo_model_[4].rated_torque=1.11*200/3.0;
        servo_model_[4].jm=0.000017;
        servo_model_[4].gr=200/3;
        servo_model_[5].rated_torque=1.11*50*24/28.0;
        servo_model_[5].jm=0.000017;
        servo_model_[5].gr=50*24/28;

        is_robot_model_ready_=true;
        is_servo_model_ready_=true;
    }
}

