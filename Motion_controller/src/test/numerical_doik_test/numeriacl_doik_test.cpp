#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <iostream>
#include <string.h>
#include <vector>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <sys/time.h>
#include <time.h>
#include "kinematics_rtm.h"
#include "basic_alg.h"


using namespace std;
using namespace basic_alg;

int main(int argc, char** argv)
{

	struct  timeval tick_s, tick_e;
	DH base_dh = { 32.0 / 1000.0, 0.0, 0.0, 0.0 };
	//DH dh[6] = { 
	//	//{ 0.0, 0.0, 0.0, 0.0 },
	//	{ 32.0, 100, 1.570796, 0.0 },
	//	{ 0.0, 120, 0.0, 0.0 },
	//	{ 0.0, 0.0, 1.570796, 1.570796 },
	//	{ 159.0, 0.0, -1.570796, 0.0 },
	//	{ 0.0, 0.0, 1.570796, 0.0 },
	//	{ 108.0, 0.0, 0.0, 0.0 }
	//};
	DH dh[6] = {
		{ 0,	100 / 1000.0,	1.570796,	0.0 },
		{ 0.0,     120 / 1000.0, 0.0,      0.0 },
		{ 0.0,           0.0,  1.570796, 1.570796 },
		{ 159.0 / 1000.0, 0.0, -1.570796, 0.0 },
		{ 0.0, 0.0, 1.570796, 0.0 },
		{ 108.0 / 1000.0, 0.0, 0.0, 0.0 }
	};
	// double joint_pos[6] = { 0, 0, 0, 0, 0, 0 };
	// double cart_pos[6];
	// double tool[6] = { 0, 0, 0, 1.570796, 0, 0 };

	KinematicsRTM* p_kinematics = new KinematicsRTM(base_dh, dh, 0);
	//Transformation* p_transform = new Transformation();
	//p_transform->init(p_kinematics);

	Joint joint_d, joint_ref, res_joint;
	Joint ik_joint;
	// PoseEuler pose;
	TransMatrix tmtx;
	Posture   posture;
	posture.arm = 1;
	posture.elbow = 1;
	posture.wrist = 1;
	posture.flip = 0;
	//test-1: 0.370000 0.198041 -0.443243 -0.232782 -0.483149 -0.030313
	/*joint.j1_ = 0.37;
	joint.j2_ = 0.198041;
	joint.j3_ = -0.443243;
	joint.j4_ = -0.232782;
	joint.j5_ = -0.483149;
	joint.j6_ = -0.030313;*/
	double jref[6] = { 0.27928,0.186593,0.304105,-0.039135,-0.004015,2.02671 };
	double jd[6] = { 0.312994,0.103806,0.304562,0.004255,-0.015615,1.918477 };
	for (int i=0; i<6; i++)
	{
		joint_d[i] = jd[i];
		joint_ref[i] = jref[i];
	}
	posture = p_kinematics->getPostureByJoint(joint_d);
	printf("joint_given:");
	joint_d.print();
	p_kinematics->doFK(joint_d, tmtx);

	printf("geomatric joint:");
	p_kinematics->doIK(tmtx, posture, ik_joint);
	ik_joint.print();
	/*for (int i = 0; i < 6; i++)
		joint[i] = 1.0;*/

	p_kinematics->doNumericalIK(tmtx, joint_ref, res_joint);
	printf("numerical joint:");
	res_joint.print();


	//test-2
	FILE *fp = fopen("rand_joint_test.csv","w+");
	time_t t;
	int s = 100;
	double freq[6] = { 3.530000,8.370000,0.900000,4.340000,0.580000,4.790000 };
	double a[6] = { 1.0, 1.0, 0.3/* 2.0 */, 1.0, 2.0, 3.0 };
	Joint joint, ref_joint;
	/* ��ʼ������������� */
	srand((unsigned)time(&t));

	for (int i = 0; i < 6; i++)
	{
		freq[i] = (rand() % 1000)/100.0;
	}

	//fprintf(fp,"%lf,%lf,%lf,%lf,%lf,%lf\n", freq[0], freq[1], freq[2], freq[3], freq[4], freq[5]); //f = 10 * rand(1, 6);
	printf("%lf,%lf,%lf,%lf,%lf,%lf\n", freq[0], freq[1], freq[2], freq[3], freq[4], freq[5]);
	for (int i = 0; i < 2000; i++)
	{	// ���ɲ��Թ켣
		//qd(1, :) = [a(1) * sin(f(1) / s) a(2) * cos(f(2) / s) a(3) * sin(f(3) / s)  a(4) * cos(f(4) / s) a(5) * sin(f(5) / s) a(6) * cos(f(6) / s)];
		joint[0] = a[0] * sin(freq[0] * i / s);
		joint[1] = a[1] * cos(freq[1] * i / s);
		joint[2] = a[2] * sin(freq[2] * i / s) + 0.6;
		joint[3] = a[3] * cos(freq[3] * i / s);
		joint[4] = a[4] * sin(freq[4] * i / s);
		joint[5] = a[5] * cos(freq[5] * i / s);
		/*if (fabs(joint[3]) < 0.1)
		{
			joint[3] = (joint[3] > 0 ? 1 : -1)*0.1;
		}*/
		if (i == 0)
		{
			ref_joint = joint;
		}
		fprintf(fp,"%g,%g,%g,%g,%g,%g,",joint[0],joint[1],joint[2], joint[3], joint[4], joint[5]);
		//printf("%d\n", i);
		//printf("[i], %lf,%lf,%lf,%lf,%lf,%lf\n", joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);
		p_kinematics->doFK(joint, tmtx);
		gettimeofday(&tick_s, NULL);
		p_kinematics->doNumericalIK(tmtx, ref_joint, res_joint);
		gettimeofday(&tick_e, NULL);
		fprintf(fp, "%g,%g,%g,%g,%g,%g,%ld\n",
			res_joint[0], res_joint[1], res_joint[2], res_joint[3], res_joint[4], res_joint[5],
			1000000 * (tick_e.tv_sec-tick_s.tv_sec) + tick_e.tv_usec-tick_s.tv_usec);
		//printf("[o], %lf,%lf,%lf,%lf,%lf,%lf\n", res_joint[0], res_joint[1], res_joint[2], res_joint[3], res_joint[4], res_joint[5]);
		ref_joint = res_joint;
		//Sleep(10);
	}

	fclose(fp);
	delete p_kinematics;
	//system("pause");
	return 0;
}
