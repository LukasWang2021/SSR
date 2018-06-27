/*************************************************************************
	> File Name: dynamics_interface.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年04月19日 星期四 09时25分48秒
 ************************************************************************/

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
    bool DynamicsInterface::getkineJacobian(const double q[MAX_AXES],double jacob[MAX_AXES][MAX_AXES])
    {
        double JOT[3][MAXAXES];
        double JPT[3][MAXAXES];        
        double AQ[4][4];
        double pe[3];
        double A[4][4];
        double pi[3];
        double zi[3];
        double df[3];
        double czd[3];

        double z0[3]={0,0,1};
        double p0[3]={0,0,0};

        if(!getA(MAXAXES,q,A))
            return false;

        pe[0]=A[0][3];pe[1]=A[1][3];pe[2]=A[2][3];
        double I[4][4]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        for(int i=0;i<MAXAXES;i++)
        {

            if(i==0)
            {
                df[0]=pe[0]-p0[0];df[1]=pe[1]-p0[1];df[2]=pe[2]-p0[2];
                if(!Cross(z0,df,czd))
                    return false;
                JPT[0][i]=czd[0];
                JPT[1][i]=czd[1];
                JPT[2][i]=czd[2];
                JOT[0][i]=z0[0];
                JOT[1][i]=z0[1];
                JOT[2][i]=z0[2];
            }  
            else
            {
                if(!getA(i,q,AQ))
                    return false;

                Multiply4444(I,AQ,I);
                pi[0]=I[0][3];pi[1]=I[1][3];pi[2]=I[2][3];
                zi[0]=I[0][2];zi[1]=I[1][2];zi[2]=I[2][2];

                df[0]=pe[0]-pi[0];df[1]=pe[1]-pi[1];df[2]=pe[2]-pi[2];
                if(!Cross(zi,df,czd))
                    return false;  
                JPT[0][i]=czd[0];
                JPT[1][i]=czd[1];
                JPT[2][i]=czd[2];
                JOT[0][i]=zi[0];
                JOT[1][i]=zi[1];
                JOT[2][i]=zi[2];              
            }

        }        
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
    bool DynamicsInterface::getTranspose33(const double M[3][3],double TM[3][3])
    {
        TM[0][0]=M[0][0];TM[0][1]=M[1][0];TM[0][2]=M[2][0];
        TM[1][0]=M[0][1];TM[1][1]=M[1][1];TM[1][2]=M[2][1];
        TM[2][0]=M[0][2];TM[2][1]=M[1][2];TM[2][2]=M[2][2];
        TM[3][0]=M[0][3];TM[3][1]=M[1][3];TM[3][2]=M[2][3];        
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
        x2=robot_model_.r[1][0];y2=robot_model_.r[1][1];z2=robot_model_.r[1][2];
        x3=robot_model_.r[2][0];y3=robot_model_.r[2][1];z3=robot_model_.r[2][2];
        x4=robot_model_.r[3][0];y4=robot_model_.r[3][1];z4=robot_model_.r[3][2];
        x5=robot_model_.r[4][0];y5=robot_model_.r[4][1];z5=robot_model_.r[4][2];
        x6=robot_model_.r[5][0];y6=robot_model_.r[5][1];z6=robot_model_.r[5][2];    
            
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
                FST_INFO("---11");
                return false;
            }
            for(int j=0;j<MAXAXES;j++)
            {
                if(!getM(i+1,j+1,q,m_[i][j]))
                {
                    FST_INFO("---12");
                    return false;
                }
            }
        }*/
        //getSymbolicM(q);
        //for(int i=0;i<6;i++)
        //    for(int j=0;j<6;j++)
        //        FST_INFO("M[%d][%d]=%f",i,j,m_[i][j]);
        //getSymbolicG(q);
        //finish = clock();
        //Total_time = (double)(finish-start) / CLOCKS_PER_SEC;
        //FST_INFO( "compute G & M:%f seconds/n", Total_time);

        //start = clock(); 
        //getSymbolicC(q);
        
        /*for(int i=0;i<MAXAXES;i++)
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
    //功能: 求矩阵(n*n)的行列式  
    //入口参数: 矩阵的首地址，矩阵的行数  
    //返回值: 矩阵的行列式值  
    //----------------------------------------------  
    float DynamicsInterface::MatDet(float *p, int n)  
    {  
        int r, c, m;  
        int lop = 0;  
        float result = 0;  
        float mid = 1;  
      
        if (n != 1)  
        {  
            lop = (n == 2) ? 1 : n;            //控制求和循环次数,若为2阶，则循环1次，否则为n次  
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
    //返回值: k*k矩阵中元素A(m, n)的代数余之式  
    //----------------------------------------------------------------------------  
    float DynamicsInterface::Creat_M(float *p, int m, int n, int k)  
    {  
        int len;  
        int i, j;  
        float mid_result = 0;  
        int sign = 1;  
        float *p_creat, *p_mid;  
      
        len = (k - 1)*(k - 1);            //k阶矩阵的代数余之式为k-1阶矩阵  
        p_creat = (float*)calloc(len, sizeof(float)); //分配内存单元  
        p_mid = p_creat;  
        for (i = 0; i < k; i++)  
        {  
            for (j = 0; j < k; j++)  
            {  
                if (i != m && j != n) //将除第i行和第j列外的所有元素存储到以p_mid为首地址的内存单元  
                {  
                    *p_mid++ = *(p + i*k + j);  
                }  
            }  
        }  
        sign = (m + n) % 2 == 0 ? 1 : -1;    //代数余之式前面的正、负号  
        mid_result = (float)sign*MatDet(p_creat, k - 1);  
        free(p_creat);  
        return mid_result;  
    }  
    //------------------------------------------------------------------  
    //功能: 采用部分主元的高斯消去法求方阵A的逆矩阵B  
    //入口参数: 输入方阵，输出方阵，方阵阶数  
    //返回值: true or false  
    //-------------------------------------------------------------------  
    bool DynamicsInterface::Gauss(const double A[MAX_AXES][MAX_AXES], double B[MAX_AXES][MAX_AXES])  
    {  
        int i, j, k;  
        double max, temp;  
        double t[MAX_AXES][MAX_AXES];                //临时矩阵  
        //将A矩阵存放在临时矩阵t[n][n]中  
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
            //如果主元所在行不是第i行，进行行交换  
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
            //判断主元是否为0, 若是, 则矩阵A不是满秩矩阵,不存在逆矩阵  
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
            for (j = 0; j < MAX_AXES; j++)        //第0行->第n行  
            {  
                if (j != i)                //不是第i行  
                {  
                    temp = t[j][i];  
                    for (k = 0; k < MAX_AXES; k++)        //第j行元素 - i行元素*j列i行元素  
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
        double ddq[MAXAXES]={0,0,0,0,0,0};
        for(int i=0;i<MAXAXES;i++)
        {
            q_[i]=joint[i];
            dq_[i]=omega[i];
        }

        double mq[6][6],mdq[6][6],mddq[6][6];
    //double it[6];

        for(int i=0;i<6;i++)
        {
            for(int j=0;j<6;j++)
            {
                mq[j][i]=q_[j];

                mdq[j][i]=0;
                if(i==j)
                    mddq[j][i]=1;
                else
                    mddq[j][i]=0;

            }
            //FST_INFO("%f %f %f %f %f %f",mddq[0][i],mddq[1][i],mddq[2][i],mddq[3][i],mddq[4][i],mddq[5][i]);
        }
        rne_M(mq,mdq,mddq,m_);
        //FST_INFO("M");
        //for (int i=0;i<6;i++)
        //{
        //    FST_INFO("%f %f %f %f %f %f",m_[i][0],m_[i][1],m_[i][2],m_[i][3],m_[i][4],m_[i][5]);
        //}
        //FST_INFO("---2");

        //start = clock();
        rne_tau(q_,dq_,ddq,Tm);

        //FST_INFO("---3");
        //finish = clock();
        //Total_time = (double)(finish-start) / CLOCKS_PER_SEC;
        //FST_INFO( "getTm:%f seconds/n", Total_time);

        for(int i=0;i<MAXAXES;i++)
        {
            diffT[0][i]=servo_model_[i].rated_torque-Tm[i];
            diffT[1][i]=-servo_model_[i].rated_torque-Tm[i];
            //FST_INFO("%f %f ",diffT[0][i],diffT[1][i]);
            //FST_INFO("axis:%d rt=%f at=%f\n",i+1,servo_model_[i].rated_torque,Tm[i]);
        //    FST_INFO("neg-axis:%d rt=%f at=%f\n",i+1,-servo_model_[i].rated_torque,Tm[i]);
        }
        //FST_INFO("---4");
        //start = clock();
        //for (int i=0;i<6;i++)
        //   for(int j=0;j<6;j++)
        //        FST_INFO("m[%d][%d]=%f",i,j,m_[i][j]);

        if(!getInverseM(m_,IB))
            return false;
        //for (int i=0;i<6;i++)
        //       FST_INFO("%f %f %f %f %f %f",IB[i][0],IB[i][1],IB[i][2],IB[i][3],IB[i][4],IB[i][5]);
        
        //for (int i=0;i<6;i++)
        //    FST_INFO("diff0[%d]=%f",i,diffT[0][i]);

        //for (int i=0;i<6;i++)
        //    FST_INFO("diff1[%d]=%f",i,diffT[1][i]);
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
        //       FST_INFO("acc[%d][%d]=%f\n",j,i,alpha_max[j][i]);
        //   }
       // }


        return true;
    }
        /* 4*4 matrix multiply 4*4 4*4 matrix*/
    bool DynamicsInterface::Multiply4444(double a[4][4],double b[4][4],double c[4][4])
    {
        //[ a00*b00 + a01*b10 + a02*b20 + a03*b30, a00*b01 + a01*b11 + a02*b21 + a03*b31, a00*b02 + a01*b12 + a02*b22 + a03*b32, a00*b03 + a01*b13 + a02*b23 + a03*b33]
        //[ a10*b00 + a11*b10 + a12*b20 + a13*b30, a10*b01 + a11*b11 + a12*b21 + a13*b31, a10*b02 + a11*b12 + a12*b22 + a13*b32, a10*b03 + a11*b13 + a12*b23 + a13*b33]
        //[ a20*b00 + a21*b10 + a22*b20 + a23*b30, a20*b01 + a21*b11 + a22*b21 + a23*b31, a20*b02 + a21*b12 + a22*b22 + a23*b32, a20*b03 + a21*b13 + a22*b23 + a23*b33]
        //[ a30*b00 + a31*b10 + a32*b20 + a33*b30, a30*b01 + a31*b11 + a32*b21 + a33*b31, a30*b02 + a31*b12 + a32*b22 + a33*b32, a30*b03 + a31*b13 + a32*b23 + a33*b33]
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
    /* Recursive Newton-Euler for standard Denavit-Hartenberg notation. */
    bool DynamicsInterface::rne_tau(const double q[MAX_AXES],const double dq[MAX_AXES],const double ddq[MAX_AXES],double T[MAX_AXES])
    {
        double z0[3]={0,0,1};
        double Rb[3][3]={1,0,0,0,1,0,0,0,1};
        double Tj[4][4];
        double w[3]={0,0,0};
        double wd[3]={0,0,0};
        double vd[3];
        double r[3];
        double pstar[3];
        double Fm[3][MAXAXES],Nm[3][MAXAXES],Rm[MAXAXES][3][3],pstarm[3][MAXAXES];
        double Rt[3][3];
        double grav[3]={0,0,g0};
        double d,alpha,a;
        double fext[MAXAXES]={0,0,0,0,0,0};
        double wbase[MAXAXES];

        int n = MAX_AXES;

        Multiply3331(Rb,grav,vd);

        for(int j=0;j<n;j++)
        {
            if(!getA(j+1,q,Tj))
                return false;
            if(!getR(Tj,Rm[j]))
                return false;     

            //FST_INFO("Rm[%d]",j);
            //FST_INFO("%f %f %f",Rm[j][0][0],Rm[j][0][1],Rm[j][0][2]);  
            //FST_INFO("%f %f %f",Rm[j][1][0],Rm[j][1][1],Rm[j][1][2]); 
            //FST_INFO("%f %f %f",Rm[j][2][0],Rm[j][2][1],Rm[j][2][2]);      
            d = robot_model_.d[j];
            alpha=robot_model_.ap[j];
            a=robot_model_.a[j];
            pstar[0]=a;
            pstar[1]=d*sin(alpha);
            pstar[2]=d*cos(alpha);
            pstarm[0][j]=pstar[0];
            pstarm[1][j]=pstar[1];
            pstarm[2][j]=pstar[2];
            //FST_INFO("pstarm[%d]",j);
            //FST_INFO("%f",pstarm[0][j]);  
            //FST_INFO("%f",pstarm[1][j]); 
            //FST_INFO("%f",pstarm[2][j]);             
        }

        //the forward recursion
        double z0qdd[3];
        double z0qd[3];
        double crosswz0qd[3];
        double ttlwd[3];
        double ttlw[3];
        double twd[3];
        double tw[3];
        double crosswdpstar[3];
        double crosswpstar[3];
        double crosswwpstar[3];
        double Rtvd[3];
        double crosswdr[3];
        double crosswr[3];
        double crosswwr[3];
        double vhat[3];
        double F[3],N[3];
        double linkiwd[3];
        double linkiw[3];
        double crosswlinkiw[3];

        for(int j=0;j<n;j++)
        {
            //transpose Rm[j]
            getTranspose33(Rm[j],Rt);
            pstar[0]=pstarm[0][j];
            pstar[1]=pstarm[1][j];
            pstar[2]=pstarm[2][j];
            r[0]=robot_model_.r[j][0];
            r[1]=robot_model_.r[j][1];
            r[2]=robot_model_.r[j][2];

            //statement order is important here
            z0qdd[0]=z0[0]*ddq[j];
            z0qdd[1]=z0[1]*ddq[j];
            z0qdd[2]=z0[2]*ddq[j];
            z0qd[0]=z0[0]*dq[j];
            z0qd[1]=z0[1]*dq[j];
            z0qd[2]=z0[2]*dq[j];
            Cross(w,z0qd,crosswz0qd);

            ttlwd[0]=wd[0]+z0qdd[0]+crosswz0qd[0];
            ttlwd[1]=wd[1]+z0qdd[1]+crosswz0qd[1];
            ttlwd[2]=wd[2]+z0qdd[2]+crosswz0qd[2];

            Multiply3331(Rt,ttlwd,twd);
            wd[0]=twd[0];
            wd[1]=twd[1];
            wd[2]=twd[2];
            //FST_INFO("wd[%d]",j);
            //FST_INFO("%f",wd[0]);  
            //FST_INFO("%f",wd[1]); 
            //FST_INFO("%f",wd[2]); 
            ttlw[0]=w[0]+z0qd[0];
            ttlw[1]=w[1]+z0qd[1];
            ttlw[2]=w[2]+z0qd[2];

            Multiply3331(Rt,ttlw,tw);
            w[0]=tw[0];
            w[1]=tw[1];
            w[2]=tw[2];
            //FST_INFO("w[%d]",j);
            //FST_INFO("%f",w[0]);  
            //FST_INFO("%f",w[1]); 
            //FST_INFO("%f",w[2]); 

            Cross(wd,pstar,crosswdpstar);
            Cross(w,pstar,crosswpstar);
            pstar[0]=pstarm[0][j];
            pstar[1]=pstarm[1][j];
            pstar[2]=pstarm[2][j];
            Cross(w,crosswpstar,crosswwpstar);
            Multiply3331(Rt,vd,Rtvd);

            vd[0]=crosswdpstar[0]+crosswwpstar[0]+Rtvd[0];
            vd[1]=crosswdpstar[1]+crosswwpstar[1]+Rtvd[1];
            vd[2]=crosswdpstar[2]+crosswwpstar[2]+Rtvd[2];
            //FST_INFO("vd[%d]",j);
            //FST_INFO("%f",vd[0]);  
            //FST_INFO("%f",vd[1]); 
            //FST_INFO("%f",vd[2]); 
            //whos
            Cross(wd,r,crosswdr);
            Cross(w,r,crosswr);
            Cross(w,crosswr,crosswwr);
            vhat[0]=crosswdr[0]+crosswwr[0]+vd[0];
            vhat[1]=crosswdr[1]+crosswwr[1]+vd[1];
            vhat[2]=crosswdr[2]+crosswwr[2]+vd[2];
            //FST_INFO("vhat[%d]",j);
            //FST_INFO("%f",vhat[0]);  
            //FST_INFO("%f",vhat[1]); 
            //FST_INFO("%f",vhat[2]); 
            F[0]=robot_model_.m[j]*vhat[0];
            F[1]=robot_model_.m[j]*vhat[1];
            F[2]=robot_model_.m[j]*vhat[2];

            Multiply3331(robot_model_.i[j],wd,linkiwd);
            Multiply3331(robot_model_.i[j],w,linkiw);
            Cross(w,linkiw,crosswlinkiw);
            N[0]=linkiwd[0]+crosswlinkiw[0];
            N[1]=linkiwd[1]+crosswlinkiw[1];
            N[2]=linkiwd[2]+crosswlinkiw[2];

            Fm[0][j]=F[0];
            Fm[1][j]=F[1];
            Fm[2][j]=F[2];
            Nm[0][j]=N[0];
            Nm[1][j]=N[1];
            Nm[2][j]=N[2];

        }

        //the backward recursion
        double f[3]={fext[0],fext[1],fext[2]};
        double nn[3]={fext[3],fext[4],fext[5]};
        double R[3][3];
        double RT[3][3];
        double RTpstar[3];
        double crossrtpstarf[3];
        double nnaddcrossrtpstarf[3];
        double Rnnaddcrossrtpstarf[3];
        double pstaraddr[3];
        double Fmj[3];
        double crosspstaraddrfmj[3];
        double Rf[3];
        double RTz0[3];
        double nnRTz0;
        double t;

        for(int j=5;j>=0;j--)
        {
            pstar[0]=pstarm[0][j];
            pstar[1]=pstarm[1][j];
            pstar[2]=pstarm[2][j]; 

            //order of these statements is important, since both
            // nn and f are functions of previous f.

            if(j==5)
            {
                R[0][0]=1;R[0][1]=0;R[0][2]=0;
                R[1][0]=0;R[1][1]=1;R[1][2]=0;
                R[2][0]=0;R[2][1]=0;R[2][2]=1;
            }
            else
            {
                R[0][0]=Rm[j+1][0][0];R[0][1]=Rm[j+1][0][1];R[0][2]=Rm[j+1][0][2];
                R[1][0]=Rm[j+1][1][0];R[1][1]=Rm[j+1][1][1];R[1][2]=Rm[j+1][1][2];
                R[2][0]=Rm[j+1][2][0];R[2][1]=Rm[j+1][2][1];R[2][2]=Rm[j+1][2][2];                
            }

            r[0]=robot_model_.r[j][0];
            r[1]=robot_model_.r[j][1];
            r[2]=robot_model_.r[j][2];

            getTranspose33(R,RT);
            Multiply3331(RT,pstar,RTpstar);
            Cross(RTpstar,f,crossrtpstarf);
            nnaddcrossrtpstarf[0]=nn[0]+crossrtpstarf[0];
            nnaddcrossrtpstarf[1]=nn[1]+crossrtpstarf[1];
            nnaddcrossrtpstarf[2]=nn[2]+crossrtpstarf[2];
            Multiply3331(R,nnaddcrossrtpstarf,Rnnaddcrossrtpstarf);

            pstaraddr[0]=pstar[0]+r[0];
            pstaraddr[1]=pstar[1]+r[1];
            pstaraddr[2]=pstar[2]+r[2];
            Fmj[0]=Fm[0][j];
            Fmj[1]=Fm[1][j];
            Fmj[2]=Fm[2][j];

            Cross(pstaraddr,Fmj,crosspstaraddrfmj);

            nn[0]=Rnnaddcrossrtpstarf[0]+crosspstaraddrfmj[0]+Nm[0][j];
            nn[1]=Rnnaddcrossrtpstarf[1]+crosspstaraddrfmj[1]+Nm[1][j];
            nn[2]=Rnnaddcrossrtpstarf[2]+crosspstaraddrfmj[2]+Nm[2][j];
            //FST_INFO("nn[%d]",j);
            //FST_INFO("%f",nn[0]);  
            //FST_INFO("%f",nn[1]); 
            //FST_INFO("%f",nn[2]);  
            Multiply3331(R,f,Rf);
            f[0]=Rf[0]+Fm[0][j];
            f[1]=Rf[1]+Fm[1][j];
            f[2]=Rf[2]+Fm[2][j];
            //FST_INFO("f[%d]",j);
            //FST_INFO("%f",f[0]);  
            //FST_INFO("%f",f[1]); 
            //FST_INFO("%f",f[2]);  
            R[0][0]=Rm[j][0][0];R[0][1]=Rm[j][0][1];R[0][2]=Rm[j][0][2];
            R[1][0]=Rm[j][1][0];R[1][1]=Rm[j][1][1];R[1][2]=Rm[j][1][2];
            R[2][0]=Rm[j][2][0];R[2][1]=Rm[j][2][1];R[2][2]=Rm[j][2][2]; 

            getTranspose33(R,RT);
            Multiply3331(RT,z0,RTz0);
            nnRTz0=nn[0]*RTz0[0]+nn[1]*RTz0[1]+nn[2]*RTz0[2];
            t=nnRTz0+pow(servo_model_[j].gr,2)*servo_model_[j].jm*ddq[j];
            //FST_INFO("t %f ",t);
            T[j]=t;
        }

        //this last bit needs work/testing
        double Rnn[3];

        R[0][0]=Rm[0][0][0];R[0][1]=Rm[0][0][1];R[0][2]=Rm[0][0][2];
        R[1][0]=Rm[0][1][0];R[1][1]=Rm[0][1][1];R[1][2]=Rm[0][1][2];
        R[2][0]=Rm[0][2][0];R[2][1]=Rm[0][2][1];R[2][2]=Rm[0][2][2];

        Multiply3331(R,nn,Rnn);
        nn[0]=Rnn[0];
        nn[1]=Rnn[1];
        nn[2]=Rnn[2];

        Multiply3331(R,f,Rf);
        f[0]=Rf[0];
        f[1]=Rf[1];
        f[2]=Rf[2];

        wbase[0]=f[0];
        wbase[1]=f[1];
        wbase[2]=f[2];
        wbase[3]=nn[0];
        wbase[4]=nn[1];
        wbase[5]=nn[2];
    }
    bool DynamicsInterface::rne_M(const double mq[MAX_AXES][MAX_AXES],const double mdq[MAX_AXES][MAX_AXES],const double mddq[MAX_AXES][MAX_AXES],double M[MAX_AXES][MAX_AXES])
    {
        double q[MAXAXES],dq[MAXAXES],ddq[MAXAXES];
        double z0[3]={0,0,1};
        double Rb[3][3]={1,0,0,0,1,0,0,0,1};
        double Tj[4][4];
        double w[3]={0,0,0};
        double wd[3]={0,0,0};
        double vd[3]={0,0,0};
        double r[3];
        double pstar[3];
        double Fm[3][MAXAXES],Nm[3][MAXAXES],Rm[MAXAXES][3][3],pstarm[3][MAXAXES];
        double Rt[3][3];
        double grav[3]={0,0,0};
        double d,alpha,a;
        double fext[MAXAXES]={0,0,0,0,0,0};
        double wbase[MAXAXES];
        
        int n = MAX_AXES;

        for(int i=0;i<6;i++)
        {
            q[0]=mq[0][i];q[1]=mq[1][i];q[2]=mq[2][i];q[3]=mq[3][i];q[4]=mq[4][i];q[5]=mq[5][i];
            dq[0]=mdq[0][i];dq[1]=mdq[1][i];dq[2]=mdq[2][i];dq[3]=mdq[3][i];dq[4]=mdq[4][i];dq[5]=mdq[5][i];
            ddq[0]=mddq[0][i];ddq[1]=mddq[1][i];ddq[2]=mddq[2][i];ddq[3]=mddq[3][i];ddq[4]=mddq[4][i];ddq[5]=mddq[5][i];

            for(int l=0;l<6;l++)
            {
                for(int m=0;m<3;m++)
                {
                    Fm[l][m]=0;
                    Nm[l][m]=0;
                    pstarm[l][m]=0;
                    for(int n=0;n<3;n++)
                    {
                        Rm[l][m][n]=0;
                    }
                }
            }
            Rb[0][0]=1;Rb[0][1]=0;Rb[0][2]=0;
            Rb[1][0]=0;Rb[1][1]=1;Rb[1][2]=0;
            Rb[2][0]=0;Rb[2][1]=0;Rb[2][2]=1;

            w[0]=0;w[1]=0;w[2]=0;
            wd[0]=0;wd[1]=0;wd[2]=0;
            vd[0]=0;vd[1]=0;vd[2]=0;
            //FST_INFO("%f %f %f %f %f %f",q[0],q[1],q[2],q[3],q[4],q[5]);
            for(int j=0;j<n;j++)
            {
                if(!getA(j+1,q,Tj))
                    return false;
                if(!getR(Tj,Rm[j]))
                    return false;     

                //FST_INFO("Rm[%d]",j);
                //FST_INFO("%f %f %f",Rm[j][0][0],Rm[j][0][1],Rm[j][0][2]);  
                //FST_INFO("%f %f %f",Rm[j][1][0],Rm[j][1][1],Rm[j][1][2]); 
                //FST_INFO("%f %f %f",Rm[j][2][0],Rm[j][2][1],Rm[j][2][2]);      
                d = robot_model_.d[j];
                alpha=robot_model_.ap[j];
                a=robot_model_.a[j];
                pstar[0]=a;
                pstar[1]=d*sin(alpha);
                pstar[2]=d*cos(alpha);
                pstarm[0][j]=pstar[0];
                pstarm[1][j]=pstar[1];
                pstarm[2][j]=pstar[2];
                //FST_INFO("pstarm[%d]",j);
                //FST_INFO("%f",pstarm[0][j]);  
                //FST_INFO("%f",pstarm[1][j]); 
                //FST_INFO("%f",pstarm[2][j]);             
            }

            //the forward recursion
            double z0qdd[3];
            double z0qd[3];
            double crosswz0qd[3];
            double ttlwd[3];
            double ttlw[3];
            double twd[3];
            double tw[3];
            double crosswdpstar[3];
            double crosswpstar[3];
            double crosswwpstar[3];
            double Rtvd[3];
            double crosswdr[3];
            double crosswr[3];
            double crosswwr[3];
            double vhat[3];
            double F[3],N[3];
            double linkiwd[3];
            double linkiw[3];
            double crosswlinkiw[3];

            for(int j=0;j<n;j++)
            {
                //transpose Rm[j]
                getTranspose33(Rm[j],Rt);
                pstar[0]=pstarm[0][j];
                pstar[1]=pstarm[1][j];
                pstar[2]=pstarm[2][j];
                //FST_INFO("%d pstar[%d] %f %f %f",i,j,pstar[0],pstar[1],pstar[2]);
                r[0]=robot_model_.r[j][0];
                r[1]=robot_model_.r[j][1];
                r[2]=robot_model_.r[j][2];

                //statement order is important here
                z0qdd[0]=z0[0]*ddq[j];
                z0qdd[1]=z0[1]*ddq[j];
                z0qdd[2]=z0[2]*ddq[j];
                z0qd[0]=z0[0]*dq[j];
                z0qd[1]=z0[1]*dq[j];
                z0qd[2]=z0[2]*dq[j];
                Cross(w,z0qd,crosswz0qd);

                ttlwd[0]=wd[0]+z0qdd[0]+crosswz0qd[0];
                ttlwd[1]=wd[1]+z0qdd[1]+crosswz0qd[1];
                ttlwd[2]=wd[2]+z0qdd[2]+crosswz0qd[2];

                Multiply3331(Rt,ttlwd,twd);
                wd[0]=twd[0];
                wd[1]=twd[1];
                wd[2]=twd[2];
                //FST_INFO("%d wd[%d] %f %f %f",i,j,wd[0],wd[1],wd[2]);

                ttlw[0]=w[0]+z0qd[0];
                ttlw[1]=w[1]+z0qd[1];
                ttlw[2]=w[2]+z0qd[2];

                Multiply3331(Rt,ttlw,tw);
                w[0]=tw[0];
                w[1]=tw[1];
                w[2]=tw[2];
                //FST_INFO("%d w[%d] %f %f %f",i,j,w[0],w[1],w[2]);

                Cross(wd,pstar,crosswdpstar);
                Cross(w,pstar,crosswpstar);
                pstar[0]=pstarm[0][j];
                pstar[1]=pstarm[1][j];
                pstar[2]=pstarm[2][j];
                Cross(w,crosswpstar,crosswwpstar);
                Multiply3331(Rt,vd,Rtvd);

                vd[0]=crosswdpstar[0]+crosswwpstar[0]+Rtvd[0];
                vd[1]=crosswdpstar[1]+crosswwpstar[1]+Rtvd[1];
                vd[2]=crosswdpstar[2]+crosswwpstar[2]+Rtvd[2];
                //FST_INFO("%d vd[%d] %f %f %f",i,j,vd[0],vd[1],vd[2]);

                //whos
                Cross(wd,r,crosswdr);
                Cross(w,r,crosswr);
                Cross(w,crosswr,crosswwr);
                vhat[0]=crosswdr[0]+crosswwr[0]+vd[0];
                vhat[1]=crosswdr[1]+crosswwr[1]+vd[1];
                vhat[2]=crosswdr[2]+crosswwr[2]+vd[2];
                //FST_INFO("%d vhat[%d] %f %f %f",i,j,vhat[0],vhat[1],vhat[2]);

                F[0]=robot_model_.m[j]*vhat[0];
                F[1]=robot_model_.m[j]*vhat[1];
                F[2]=robot_model_.m[j]*vhat[2];

                Multiply3331(robot_model_.i[j],wd,linkiwd);
                Multiply3331(robot_model_.i[j],w,linkiw);
                Cross(w,linkiw,crosswlinkiw);
                N[0]=linkiwd[0]+crosswlinkiw[0];
                N[1]=linkiwd[1]+crosswlinkiw[1];
                N[2]=linkiwd[2]+crosswlinkiw[2];

                Fm[0][j]=F[0];
                Fm[1][j]=F[1];
                Fm[2][j]=F[2];
                Nm[0][j]=N[0];
                Nm[1][j]=N[1];
                Nm[2][j]=N[2];

            }

            //the backward recursion
            double f[3]={fext[0],fext[1],fext[2]};
            double nn[3]={fext[3],fext[4],fext[5]};
            double R[3][3];
            double RT[3][3];
            double RTpstar[3];
            double crossrtpstarf[3];
            double nnaddcrossrtpstarf[3];
            double Rnnaddcrossrtpstarf[3];
            double pstaraddr[3];
            double Fmj[3];
            double crosspstaraddrfmj[3];
            double Rf[3];
            double RTz0[3];
            double nnRTz0;
            double t;

            for(int j=5;j>=0;j--)
            {
                pstar[0]=pstarm[0][j];
                pstar[1]=pstarm[1][j];
                pstar[2]=pstarm[2][j]; 

                //FST_INFO("%d pstar[%d] %f %f %f",i,j,pstar[0],pstar[1],pstar[2]);
                //order of these statements is important, since both
                // nn and f are functions of previous f.

                if(j==5)
                {
                    R[0][0]=1;R[0][1]=0;R[0][2]=0;
                    R[1][0]=0;R[1][1]=1;R[1][2]=0;
                    R[2][0]=0;R[2][1]=0;R[2][2]=1;
                }
                else
                {
                    R[0][0]=Rm[j+1][0][0];R[0][1]=Rm[j+1][0][1];R[0][2]=Rm[j+1][0][2];
                    R[1][0]=Rm[j+1][1][0];R[1][1]=Rm[j+1][1][1];R[1][2]=Rm[j+1][1][2];
                    R[2][0]=Rm[j+1][2][0];R[2][1]=Rm[j+1][2][1];R[2][2]=Rm[j+1][2][2];                
                }

                r[0]=robot_model_.r[j][0];
                r[1]=robot_model_.r[j][1];
                r[2]=robot_model_.r[j][2];

                getTranspose33(R,RT);
                Multiply3331(RT,pstar,RTpstar);
                Cross(RTpstar,f,crossrtpstarf);
                nnaddcrossrtpstarf[0]=nn[0]+crossrtpstarf[0];
                nnaddcrossrtpstarf[1]=nn[1]+crossrtpstarf[1];
                nnaddcrossrtpstarf[2]=nn[2]+crossrtpstarf[2];
                Multiply3331(R,nnaddcrossrtpstarf,Rnnaddcrossrtpstarf);

                pstaraddr[0]=pstar[0]+r[0];
                pstaraddr[1]=pstar[1]+r[1];
                pstaraddr[2]=pstar[2]+r[2];
                Fmj[0]=Fm[0][j];
                Fmj[1]=Fm[1][j];
                Fmj[2]=Fm[2][j];

                Cross(pstaraddr,Fmj,crosspstaraddrfmj);

                nn[0]=Rnnaddcrossrtpstarf[0]+crosspstaraddrfmj[0]+Nm[0][j];
                nn[1]=Rnnaddcrossrtpstarf[1]+crosspstaraddrfmj[1]+Nm[1][j];
                nn[2]=Rnnaddcrossrtpstarf[2]+crosspstaraddrfmj[2]+Nm[2][j];
                //FST_INFO("%d nn[%d] %f %f %f",i,j,nn[0],nn[1],nn[2]);
                //FST_INFO("%f",nn[0]);  
                //FST_INFO("%f",nn[1]); 
                //FST_INFO("%f",nn[2]);  
                Multiply3331(R,f,Rf);
                f[0]=Rf[0]+Fm[0][j];
                f[1]=Rf[1]+Fm[1][j];
                f[2]=Rf[2]+Fm[2][j];
                //FST_INFO("%d f[%d] %f %f %f",i,j,f[0],f[1],f[2]);
                //FST_INFO("%f",f[0]);  
                //FST_INFO("%f",f[1]); 
                //FST_INFO("%f",f[2]);  
                R[0][0]=Rm[j][0][0];R[0][1]=Rm[j][0][1];R[0][2]=Rm[j][0][2];
                R[1][0]=Rm[j][1][0];R[1][1]=Rm[j][1][1];R[1][2]=Rm[j][1][2];
                R[2][0]=Rm[j][2][0];R[2][1]=Rm[j][2][1];R[2][2]=Rm[j][2][2]; 

                getTranspose33(R,RT);
                Multiply3331(RT,z0,RTz0);
                nnRTz0=nn[0]*RTz0[0]+nn[1]*RTz0[1]+nn[2]*RTz0[2];
                t=nnRTz0+pow(servo_model_[j].gr,2)*servo_model_[j].jm*ddq[j];
                //FST_INFO("%d %d t %f ",i,j,t);
                M[i][j]=t;
            }

            //this last bit needs work/testing
            double Rnn[3];
            R[0][0]=Rm[0][0][0];R[0][1]=Rm[0][0][1];R[0][2]=Rm[0][0][2];
            R[1][0]=Rm[0][1][0];R[1][1]=Rm[0][1][1];R[1][2]=Rm[0][1][2];
            R[2][0]=Rm[0][2][0];R[2][1]=Rm[0][2][1];R[2][2]=Rm[0][2][2];

            //FST_INFO("%d R",i);
            //FST_INFO("%f %f %f",R[0][0],R[0][1],R[0][2]);
            //FST_INFO("%f %f %f",R[1][0],R[1][1],R[1][2]);
            //FST_INFO("%f %f %f",R[2][0],R[2][1],R[2][2]);
            Multiply3331(R,nn,Rnn);
            nn[0]=Rnn[0];
            nn[1]=Rnn[1];
            nn[2]=Rnn[2];
            //FST_INFO("%d nn %f %f %f",i,nn[0],nn[1],nn[2]);
            Multiply3331(R,f,Rf);
            f[0]=Rf[0];
            f[1]=Rf[1];
            f[2]=Rf[2];
            //FST_INFO("%d f %f %f %f",i,f[0],f[1],f[2]);
            wbase[0]=f[0];
            wbase[1]=f[1];
            wbase[2]=f[2];
            wbase[3]=nn[0];
            wbase[4]=nn[1];
            wbase[5]=nn[2]; 
        }       
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
        g0 = 9.81;
        memset(robot_model_.a,0,sizeof(robot_model_.a));
        robot_model_.a[0]=0.03;
        robot_model_.a[1]=0.34;
        robot_model_.a[2]=0.035;
        
        memset(robot_model_.d,0,sizeof(robot_model_.d));
        robot_model_.d[0]=0.365;
        robot_model_.d[3]=0.35;
        robot_model_.d[5]=0.0965;
        
        memset(robot_model_.ap,0,sizeof(robot_model_.ap));
        robot_model_.ap[0]=M_PI/2;
        robot_model_.ap[2]=M_PI/2;
        robot_model_.ap[3]=-M_PI/2;
        robot_model_.ap[4]=M_PI/2;
        
        //qlim
        //joint1
        robot_model_.qlim[0][0] = -3.00197;
        robot_model_.qlim[0][1] = 3.00197;
        //joint2
        robot_model_.qlim[1][0] = -1.79769;
        robot_model_.qlim[1][1] = 2.39110;
        //joint3
        robot_model_.qlim[2][0] = -3.49066;
        robot_model_.qlim[2][1] = 1.22173;
        //joint4
        robot_model_.qlim[3][0] = -3.31613;
        robot_model_.qlim[3][1] = 3.31613;
        //joint5
        robot_model_.qlim[4][0] = -2.02458;
        robot_model_.qlim[4][1] = 2.02458;
        //joint6
        robot_model_.qlim[5][0] = -6.28319;
        robot_model_.qlim[5][1] = 6.28319;

        memset(robot_model_.offset,0,sizeof(robot_model_.offset));
        robot_model_.offset[1]=M_PI/2;
        
        robot_model_.r[0][0]=0.00622;
        robot_model_.r[0][1]=0.011401;
        robot_model_.r[0][2]=-0.092646;

        robot_model_.r[1][0]=0.085638;
        robot_model_.r[1][1]=0.004959;
        robot_model_.r[1][2]=-0.024748;

        robot_model_.r[2][0]=0.025268;
        robot_model_.r[2][1]=-0.017188;
        robot_model_.r[2][2]=-0.016658;

        robot_model_.r[3][0]=-0.000560;
        robot_model_.r[3][1]=0.000821;
        robot_model_.r[3][2]=-0.129599;

        robot_model_.r[4][0]=0.000002;
        robot_model_.r[4][1]=-0.021732;
        robot_model_.r[4][2]=-0.002718;

        robot_model_.r[5][0]=-0.000190;
        robot_model_.r[5][1]=0;
        robot_model_.r[5][2]=0.092437;

        robot_model_.i[0][0][0]=0.110997;
        robot_model_.i[0][1][1]=0.088273;
        robot_model_.i[0][2][2]=0.055623;
        robot_model_.i[0][0][1]=0.001337;
        robot_model_.i[0][1][2]=0.001098;
        robot_model_.i[0][0][2]=0.001796;
        robot_model_.i[0][1][0]=robot_model_.i[0][0][1];
        robot_model_.i[0][2][1]=robot_model_.i[0][1][2];
        robot_model_.i[0][2][0]=robot_model_.i[0][0][2];

        robot_model_.i[1][0][0]=0.084005;
        robot_model_.i[1][1][1]=0.343746;
        robot_model_.i[1][2][2]=0.293182;
        robot_model_.i[1][0][1]=0.008635;
        robot_model_.i[1][1][2]=-0.000403;
        robot_model_.i[1][0][2]=-0.010501;
        robot_model_.i[1][1][0]=robot_model_.i[1][0][1];
        robot_model_.i[1][2][1]=robot_model_.i[1][1][2];
        robot_model_.i[1][2][0]=robot_model_.i[1][0][2];

        robot_model_.i[2][0][0]=0.051720;
        robot_model_.i[2][1][1]=0.045777;
        robot_model_.i[2][2][2]=0.051204;
        robot_model_.i[2][0][1]=-0.004611;
        robot_model_.i[2][1][2]=-0.000244;
        robot_model_.i[2][0][2]=-0.001261;
        robot_model_.i[2][1][0]=robot_model_.i[2][0][1];
        robot_model_.i[2][2][1]=robot_model_.i[2][1][2];
        robot_model_.i[2][2][0]=robot_model_.i[2][0][2];

        robot_model_.i[3][0][0]=0.118396;
        robot_model_.i[3][1][1]=0.115976;
        robot_model_.i[3][2][2]=0.013962;
        robot_model_.i[3][0][1]=-0.000194;
        robot_model_.i[3][1][2]=-0.000205;
        robot_model_.i[3][0][2]=0.000523;
        robot_model_.i[3][1][0]=robot_model_.i[3][0][1];
        robot_model_.i[3][2][1]=robot_model_.i[3][1][2];
        robot_model_.i[3][2][0]=robot_model_.i[3][0][2];

        robot_model_.i[4][0][0]=0.004587;
        robot_model_.i[4][1][1]=0.001832;
        robot_model_.i[4][2][2]=0.004252;
        robot_model_.i[4][0][1]=0;
        robot_model_.i[4][1][2]=-0.000002;
        robot_model_.i[4][0][2]=0;
        robot_model_.i[4][1][0]=robot_model_.i[4][0][1];
        robot_model_.i[4][2][1]=robot_model_.i[4][1][2];
        robot_model_.i[4][2][0]=robot_model_.i[4][0][2];

        robot_model_.i[5][0][0]=0.001107;
        robot_model_.i[5][1][1]=0.001107;
        robot_model_.i[5][2][2]=0.000052;
        robot_model_.i[5][0][1]=0;
        robot_model_.i[5][1][2]=0;
        robot_model_.i[5][0][2]=-0.000002;
        robot_model_.i[5][1][0]=robot_model_.i[5][0][1];
        robot_model_.i[5][2][1]=robot_model_.i[5][1][2];
        robot_model_.i[5][2][0]=robot_model_.i[5][0][2];

        robot_model_.m[0]=5.077933;
        robot_model_.m[1]=10.913478;
        robot_model_.m[2]=9.092074;
        robot_model_.m[3]=5.305265;
        robot_model_.m[4]=2.036364;
        robot_model_.m[5]=4.126446;

        robot_model_.mdh=true;
        memset(robot_model_.b,0,sizeof(robot_model_.b));  
        memset(robot_model_.tc,0,sizeof(robot_model_.tc));   
    
        //servo_model_[0].rated_torque=289;
        servo_model_[0].rated_torque=7.16*81;
        servo_model_[0].jm=0.00013;
        servo_model_[0].gr=81;
        //servo_model_[0].b=0; //friction factor
        //servo_model_[1].rated_torque=316;
        servo_model_[1].rated_torque=5.73*101;
        servo_model_[1].jm=0.000059;
        servo_model_[1].gr=101;  
        //servo_model_[1].b=0; //friction factor
        //servo_model_[2].rated_torque=130;
        servo_model_[2].rated_torque=3.82*81;
        servo_model_[2].jm=0.000044;
        servo_model_[2].gr=81;   
        //servo_model_[2].b=0; //friction factor   
        //servo_model_[3].rated_torque=53;
        servo_model_[3].rated_torque=1.91*60;
        servo_model_[3].jm=0.000018;
        servo_model_[3].gr=60;
        //servo_model_[3].b=0; //friction factor
        //servo_model_[4].rated_torque=29;
        servo_model_[4].rated_torque=1.11*66.7;
        servo_model_[4].jm=0.000017;
        servo_model_[4].gr=66.7;
        //servo_model_[4].b=0; //friction factor
        //servo_model_[5].rated_torque=23;
        servo_model_[5].rated_torque=1.11*44.6;
        servo_model_[5].jm=0.000017;
        servo_model_[5].gr=44.6;
        //servo_model_[5].b=0; //friction factor

        is_robot_model_ready_=true;
        is_servo_model_ready_=true;
    }
    /*DynamicsInterface::DynamicsInterface()
    {
        //default value
        robot_model_.model_name="fst_robotics_dynamic_model";
        
        memset(robot_model_.a,0,sizeof(robot_model_.a));
        robot_model_.a[0]=0.03;
        robot_model_.a[1]=0.34;
        robot_model_.a[2]=0.035;
        
        memset(robot_model_.d,0,sizeof(robot_model_.d));
        robot_model_.d[0]=0.365;
        robot_model_.d[3]=0.35;
        robot_model_.d[5]=0.0965;
        
        memset(robot_model_.ap,0,sizeof(robot_model_.ap));
        robot_model_.ap[0]=M_PI/2;
        robot_model_.ap[2]=M_PI/2;
        robot_model_.ap[3]=-M_PI/2;
        robot_model_.ap[4]=M_PI/2;
        
        //qlim
        //joint1
        robot_model_.qlim[0][0] = -3.00197;
        robot_model_.qlim[0][1] = 3.00197;
        //joint2
        robot_model_.qlim[1][0] = -1.79769;
        robot_model_.qlim[1][1] = 2.39110;
        //joint3
        robot_model_.qlim[2][0] = -3.49066;
        robot_model_.qlim[2][1] = 1.22173;
        //joint4
        robot_model_.qlim[3][0] = -3.31613;
        robot_model_.qlim[3][1] = 3.31613;
        //joint5
        robot_model_.qlim[4][0] = -2.02458;
        robot_model_.qlim[4][1] = 2.02458;
        //joint6
        robot_model_.qlim[5][0] = -6.28319;
        robot_model_.qlim[5][1] = 6.28319;

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
        robot_model_.i[0,0,0,0,-pi/2,0][3][2][2]=0.043194;
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
        servo_model_[0].jm=0.0013;
        servo_model_[0].gr=81;
        servo_model_[1].rated_torque=5.73*101;
        servo_model_[1].jm=0.00059;
        servo_model_[1].gr=101;  
        servo_model_[2].rated_torque=3.82*81;
        servo_model_[2].jm=0.00044;
        servo_model_[2].gr=81;      
        servo_model_[3].rated_torque=1.91*60;
        servo_model_[3].jm=0.00018;
        servo_model_[3].gr=60;
        servo_model_[4].rated_torque=1.11*66.7;
        servo_model_[4].jm=0.00017;
        servo_model_[4].gr=66.7;
        servo_model_[5].rated_torque=1.11*44.6;
        servo_model_[5].jm=0.00017;
        servo_model_[5].gr=44.6;

        is_robot_model_ready_=true;
        is_servo_model_ready_=true;
    }*/
         //get forward kinematics solve
    bool DynamicsInterface::getforwardkinematics(const double q[MAX_AXES],double t[4][4])
    {
        double T[4][4]=  {1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
        double A[4][4];
        double B[4][4];
        int i=0;
        for (int i=0;i<MAX_AXES;i++)
        {
            getA(i+1,q,A);

            Multiply4444(T,A,B);
            
            //FST_INFO(" B(%d)=\n",i+1);
            //for (int i=0;i<4;i++)
            //{
            //    FST_INFO( "%f %f %f %f\n", B[i][0],B[i][1],B[i][2],B[i][3]);
            //}
            //FST_INFO(" ***************************\n");   

            for (int i=0;i<4;i++)
            {
                for(int j=0;j<4;j++)
                {
                    T[i][j]=B[i][j];
                    B[i][j]=0;
                }
            }         
        }
        
        for (int i=0;i<4;i++)
        {
            for(int j=0;j<4;j++)
            {
                t[i][j]=T[i][j];
                //FST_INFO("t[%d][%d]=%f ",i,j,t[i][j]);
            }
            //FST_INFO("\n");
        }
    } 
    /*algebraic inverse kinematics solve*/
    bool DynamicsInterface::getinversekinematics(double t[4][4],double solve[8][6],int &ns)
    {
        // 将腕关节中心坐标系位姿矩阵按列赋值给n、o、a、p变量
        double tool_m[4][4]={ { 1, 0, 0, 0 },
                                          { 0, 1, 0, 0 },
                                          { 0, 0, 1, -robot_model_.d[5] },
                                          { 0, 0, 0, 1 } };
        //FST_INFO("d6 = %f\n",robot_model_.d[5]);
        double T[4][4];

        Multiply4444(t,tool_m,T);
        double nx = T[0][0], ny = T[1][0], nz = T[2][0],
               ox = T[0][1], oy = T[1][1], oz = T[2][1],
               ax = T[0][2], ay = T[1][2], az = T[2][2],
               px = T[0][3], py = T[1][3], pz = T[2][3];


        double t1, t2, t3, t4, t5, t6;                             // 六个关节角度
        double c1, c2, c3, c4, c5, c6, s1, s2, s3, s4, s5, s6;     // ci：i关节角余弦，si：i关节角正弦
        double s23, c23, c4s5, s4s5, t31, t32, t41, t42;           // 多关节角和及其正余弦
        double k1, k2, k, b1, b2;                                  // 中间变量

        double JOINTS_SOL[8][6] = {};          // 存储可能解
        //double SOL_OUT[6] = {};                // 最终选择解
        int i = 0;                             // 可选解的数量

        double d3 = robot_model_.d[2];
        double d1 = robot_model_.d[0];
        double d4 = robot_model_.d[3];
        double a3 = robot_model_.a[2];
        double a2 = robot_model_.a[1];
        double a1 = robot_model_.a[0];
        double mn1 = px*px + py*py;
        //FST_INFO("px=%f py=%f\n",px,py);
        double mn2 = d3*d3;
        //FST_INFO("mn1=%f mn2=%f d3=%f\n",mn1,mn2,d3);
        if (mn2 > mn1) {
            FST_INFO("[ERROR] IK failure solving theta1");
            return false;        //No solution for theta1, return 1001
        }//if (mn2 > mn1)   
        else {
            //Calculate 2 possible solutions of theta1
            double t1_part1 = atan2(d3, (mn1 - mn2) / 2),
                   t1_part2 = atan2(py, px),
                   t1_part3 = atan2(d3, -(mn1 - mn2) / 2);
            //FST_INFO("t1part1=%f t1part2=%f t1part3=%f\n",t1_part1,t1_part2,t1_part3);
            double t11 = t1_part1 + t1_part2;
            double t12 = t1_part3 + t1_part2;
            //FST_INFO("angle1=%f angle2=%f\n",t11,t12);
            double THETA1[2] = { t11, t12 };             // 1轴的两个解

            // 根据一轴两个解，分别计算其余各轴
            for (int i1 = 0; i1 <2; ++i1) {
                // 计算t1的正弦、余弦
                t1 = THETA1[i1];
                c1 = cos(t1);
                s1 = sin(t1);
                
                // 先计算t3
                k1 = pz - d1;
                //FST_INFO("k1=%f\n",k1);
                k2 = px*c1 + py*s1 - a1;
                //FST_INFO("k2=%f px=%f c1=%f py=%f s1=%f a1=%f\n",k2,px,c1,py,s1,a1);
                double mp1 = a3*a3 + d4*d4;
                //FST_INFO("mp1=%f\n",mp1);
                double mp6 = k1*k1 + k2*k2;
                //FST_INFO("mp6=%f\n",mp6);
                k = (mp6 - a2*a2 - mp1) / a2 / 2;
                //FST_INFO("k=%f\n",k);
                double mp2 = k*k;
                if (mp2 > mp1)
                    continue;                           // t3 无解，跳出本轮循环
                //FST_INFO("mp2=%f\n",mp2);
                double mp7 = sqrt(mp1 - mp2);
                //FST_INFO("mp7=%f\n",mp7);
                double mp3 = atan2(k, mp7);
                //FST_INFO("mp3=%f\n",mp3);
                double mp4 = atan2(a3, d4);
                //FST_INFO("mp4=%f\n",mp4);
                double mp5 = atan2(k, -mp7);
                //FST_INFO("mp5:%f\n",mp5);
                t31 = mp3 - mp4;
                t32 = mp5 - mp4;
                double THETA3[2] = { t31, t32 };       // 3轴的两个解
                
                // 根据三轴两个解，分别计算其余各轴
                for (int i2 = 0; i2 <2; ++i2) {
                    // 计算t3的正、余弦
                    t3 = THETA3[i2];
                    c3 = cos(t3);
                    s3 = sin(t3);
                    // 计算t2+t3的正、余弦
                    b1 = a3 + c3*a2;
                    b2 = d4 + s3*a2;
                    s23 = (k1*b1 + k2*b2) / mp6;    
                    c23 = (k2*b1 - k1*b2) / mp6;
                    // 计算t2及其正、余弦
                    t2 = atan2(s23, c23) - t3;
                    c2 = cos(t2);
                    s2 = sin(t2);
                    // 计算cos(t5),如果不等于正负1，则计算t4,如果等于，则暂时令t4=0
                    c5 = c1*s23*ax + s1*s23*ay - c23*az;
                    if (c5<(1 - 1e-6) && c5>(1e-6 - 1)) {   // Calculate theta4 if theta5!=0
                        c4s5 = c1*c23*ax + c23*s1*ay + s23*az;
                        s4s5 = s1*ax - c1*ay;
                        t41 = atan2(s4s5, c4s5);
                        t42 = (t41>0) ? (t41 - PI) : (t41 + PI);
                    }
                    else {                                    //Set theta4=0,if theta5=0  
                        t41 = 0;
                        t42 = 0;
                    }
                    // 根据 t4的两个解，依次计算其余轴
                    double THETA4[2] = { t41, t42 };          //  四轴的两个解
                    for (int i4 = 0; i4 <= 1; ++i4) {
                        t4 = THETA4[i4];
                        s4 = sin(t4);
                        c4 = cos(t4);
                        
                        // Calculate theta5
                        s5 = (c1*c23*c4 + s1*s4)*ax + (s1*c2*c3*c4 - s1*s2*s3*c4 - c1*s4)*ay + s23*c4*az;
                        t5 = atan2(s5, c5);
                        
                        // Calculate theta6
                        double cf_x = (c4*s1 - c1*c23*s4);
                        double cf_y=c1*c4 + c23*s1*s4; 
                        double cf_z = s23*s4;

                        s6 = cf_x*nx - cf_y*ny - cf_z*nz;
                        c6 = cf_x*ox - cf_y*oy - cf_z*oz;
                        t6 = atan2(s6, c6);

                        // 当theta5等于零时，重新分配t4和t6值，使二者离参考值最近
                        if (fabs(s5) < 1e-6) {         //if sin(theta5)=0, distribute values to make the distance minimum
                            double t46_tmp = t4 + t6;
                            double t46_0 = robot_model_.offset[3] +  robot_model_.offset[5];
                            t46_tmp = t46_tmp - round((t46_tmp - t46_0) / PI / 2) * 2 * PI;
                            t4 = ( robot_model_.offset[3] -robot_model_.offset[5] + t46_tmp) / 2;
                            t6 = t46_tmp - t4;
                        }
                        solve[i][0] = t1 - robot_model_.offset[0];
                        solve[i][1] = t2 - robot_model_.offset[1];
                        solve[i][2] = t3 - robot_model_.offset[2];
                        solve[i][3] = t4 - robot_model_.offset[3];
                        solve[i][4] = t5 - robot_model_.offset[4];
                        solve[i][5] = t6 - robot_model_.offset[5];
                        FST_INFO("%d:%f %f %f %f %f %f\n",i,solve[i][0],solve[i][1],solve[i][2],solve[i][3],solve[i][4],solve[i][5] );
                        i++;
                    } // for (int i4 = 0; i4 <= 1; ++i4)
                } // for (int i2 = 0; i2 <2; ++i2)
            } //for (int i1 = 0; i1 <2; ++i1)
        } //else

        // All solutions got, then try to choose the optimum one 
        ns = i-1;
    }
}

