#include <stdio.h>
#include <windows.h>
#include <iostream>
#include <string.h>
#include <vector>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <sstream>
#include "kinematics_rtm.h"
#include "transformation.h"
#include "basic_alg.h"
#include "kinematics_app.h"

/*
base_dh  = [64.5, 0.0, 0.0, 0.0]
axis0_dh = [0.0, 84.0, 1.570796, 0.0]
axis1_dh = [0.0, 88.0, 0.0, 0.0]
axis2_dh = [0.0, 0.0, 1.570796, 1.570796]
axis3_dh = [176.5, 0.0, -1.570796, 0.0]
axis4_dh = [0.0, 0.0, 1.570796, 0.0]
axis5_dh = [123.5, 0.0, 0.0, 0.0]
*/

using namespace std;
using namespace basic_alg;

#define MY_PI 3.14159265358979323846
#if 0
DH base_dh = { 64.5, 0.0, 0.0, 0.0 };
DH arm_dh[6] = {
	{ 0.0, 84.0, 1.570796, 0.0 },
	{ 0.0, 88.0, 0.0, 0.0 },
	{ 0.0, 0.0, 1.570796, 1.570796 },
	{ 176.5, 0.0, -1.570796, 0.0 },
	{ 0.0, 0.0, 1.570796, 0.0 },
	{ 123.5, 0.0, 0.0, 0.0 },
};
#endif
#if 1
DH base_dh = { 32.0, 0.0, 0.0, 0.0 };
DH arm_dh[6] = {
	{ 0.0, 100, 1.570796, 0.0 },
	{ 0.0, 120, 0.0, 0.0 },
	{ 0.0, 0.0, 1.570796, 1.570796 },
	{ 159.0, 0.0, -1.570796, 0.0 },
	{ 0.0, 0.0, 1.570796, 0.0 },
	{ 108.0, 0.0, 0.0, 0.0 },
};
#endif
#if 0
DH base_dh = { 0.0, 0.0, 0.0, 0.0 };
DH arm_dh[6] = {
	{ 41.7, 79.6, 1.5707, -0.0578 },
	{ 5.4, 87.6, 0.0008, -0.0046 },
	{ -3.8, 1.6, 1.5707, 0.9659 },
	{ 182.6, -0.3, -1.5691, -0.0101 },
	{ -1.5, 2.0, 1.5631, -0.042 },
	{ 111.5, -0.5, 0.0366, 0.0 },
};
#endif

int main(int argc, char **argv)
{
	double dh[7][4] = {
		{ 32.0, 0.0, 0.0, 0.0 },
		{ 0.0, 100, 1.570796, 0.0 },
		{ 0.0, 120, 0.0, 0.0 },
		{ 0.0, 0.0, 1.570796, 1.570796 },
		{ 159.0, 0.0, -1.570796, 0.0 },
		{ 0.0, 0.0, 1.570796, 0.0 },
		{ 108.0, 0.0, 0.0, 0.0 }
	};
	c_km_initKinematics(dh);
	// 0.370000 0.198041 -0.443243 -0.232782 -0.483149 -0.030313
	// -0.0842866, 0.272319, -1.20372, -0.312334, 1.16961, 0.128929
	int posture[4];
	double joint[6] = { -0.0842866, 0.272319, -1.20372, -0.312334, 1.16961, 0.128929 };
	c_km_getPostureByJoint(joint, posture);
#if 0
	KinematicsRTM* p_kinematics = new KinematicsRTM(base_dh, arm_dh, 0);
	Transformation* p_transform = new Transformation();
	p_transform->init(p_kinematics);

	PoseEuler tcp_in_base, fcp_in_base;

	PoseEuler uf;
	uf.point_.x_ = 0; uf.point_.y_ = 0; uf.point_.z_ = 0;
	uf.euler_.a_ = 0; uf.euler_.b_ = 0; uf.euler_.c_ = 0;
	PoseEuler tf;
	// 1.115000, -1.087000, 83.406000 - 0.000000, 0.000000, 0.000000
	tf.point_.x_ = 1.115000; tf.point_.y_ = -1.087000; tf.point_.z_ = 83.406000;
	tf.euler_.a_ = 0; tf.euler_.b_ = 0; tf.euler_.c_ = 0;
	// 359.343473,116.570139,-59.339125,-3.079538,-0.100902,1.329879
	// 0.1137    0.4045    0.0597   -1.2288    0.7327          1.3617
	// 460.000000, 40.000000, -60.000000 - 2.895195, 0.000000, 1.715951
	PoseEuler pose;
	pose.point_.x_ = 460; pose.point_.y_ = 40; pose.point_.z_ = -60;
	pose.euler_.a_ = 2.895195; pose.euler_.b_ = 0; pose.euler_.c_ = 1.715951;

	Posture posture;
	posture.arm = 1; posture.elbow = 1; posture.wrist = -1; posture.flip = 0;
	Joint joint; joint.zero();
	p_transform->convertPoseFromUserToBase(pose, uf, tcp_in_base);
	p_transform->convertTcpToFcp(tcp_in_base, tf, fcp_in_base);
	if (!p_kinematics->doIK(fcp_in_base, posture, joint))
	{
		cout << "failed" << endl;
	}
	joint.print();
#endif
#if 0

	double joint_pos[6] = { 0, 0, 0, 0, 0, 0 };
	double cart_pos[6];
	double tool[6] = { 0, 0, 0, 1.570796, 0, 0 };

	KinematicsRTM* p_kinematics = new KinematicsRTM(base_dh, arm_dh, 0);
	Transformation* p_transform = new Transformation();
	p_transform->init(p_kinematics);

	Joint joint;
	Joint ik_joint;
	PoseEuler pose;
	Posture   posture;
	posture.arm = 1;
	posture.elbow = 1;
	posture.wrist = 1;
	posture.flip = 0;
	// 0.370000 0.198041 -0.443243 -0.232782 -0.483149 -0.030313
	// -0.0842866, 0.272319, -1.20372, -0.312334, 1.16961, 0.128929
	joint.j1_ = -0.0842866;
	joint.j2_ = 0.272319;
	joint.j3_ = -1.20372;
	joint.j4_ = -0.312334;
	joint.j5_ = 1.16961;
	joint.j6_ = 0.128929;
	posture = p_kinematics->getPostureByJoint(joint);

	//p_kinematics->doFK(joint, pose);
	//pose.print();
	//426.366122, 152.957303, -85.564026 - 0.711316, -0.798039, 2.782095;
	//pose.point_.x_ = 426.366122;
	//pose.point_.y_ = 152.957303;
	//pose.point_.z_ = -85.564026;
	//pose.euler_.a_ = 0.711316;
	//pose.euler_.b_ = -0.798039;
	//pose.euler_.c_ = 2.782095;
	//p_kinematics->doIK(pose, posture, ik_joint);
	//ik_joint.print();
	//c_km_initKinematics(dh);
	//c_km_setToolFrame(tool);
	//c_km_getCartLink(joint_pos, 0, 1, cart_pos);
	//c_km_getTcpByBase(joint_pos, cart_pos);
#endif

#if 0
	KinematicsRTM* p_kinematics = new KinematicsRTM(base_dh, arm_dh, 0);
	Transformation* p_transform = new Transformation();
	p_transform->init(p_kinematics);

	vector<double> trj_data;
	string line_str;
	std::ifstream in_file("leica_joint.dat", ios::in);
	std::ofstream out_file("test_joint_out.dat", ios::out);

	stringstream ss;
	int line_count = 0;
	Joint joint;
	Joint ik_joint;
	PoseEuler pose;
	Posture   posture;
	posture.arm = 1;
	posture.elbow = 1;
	posture.wrist = 1;
	posture.flip = 0;
	//string p_list = "P = []";
	//string comment = "#p0";
	string pos_point = "pos = innertypes.POSTURE(motion.COORD_JOINT,\n        1, 1, 1, 0,\n        0, 0, 0, 0, 0, 0, 0, 0, 0,\n        ";

	while (getline(in_file, line_str))
	{
		//cout << line_str << endl;
		ss.clear();
		ss << line_str;
		ss >> joint.j1_;
		ss >> joint.j2_;
		ss >> joint.j3_;
		ss >> joint.j4_;
		ss >> joint.j5_;
		ss >> joint.j6_;
		//p_kinematics->doFK(joint, pose);
		//ik_joint.zero();
		//if (!p_kinematics->doIK(pose, posture, ik_joint))
		//{
		//	cout << "failed" << endl;
		//}

		out_file << pos_point
			<< joint.j1_ / 180.0 * MY_PI << ", " << joint.j2_ / 180 * MY_PI << ", " << joint.j3_ / 180 * MY_PI << ", "
			<< joint.j4_ / 180 * MY_PI << ", " << joint.j5_ / 180 * MY_PI << ", " << joint.j6_ / 180 * MY_PI << ", "
			<< 0.0 << ", " << 0.0 << ", " << 0.0 << ");"
			<< "\n\n" << "P.append(pos)" << "\n\n";
	}
	in_file.close();
	out_file.close();
#endif
#if 0
// 0.5000 0 -0.8660 0
// 0 1.0000 0 0
// 0.8660 0 0.5000 0.0500
// 0 0 0 1.0000
	PoseEuler pose;
	double trans_matrix[4][4] = { {0.5, 0, -0.866, 0}, {0, 1, 0, 0}, {0.866, 0, 0.5, 0.05}, {0, 0, 0, 1} };
	Matrix2PoseEuler(trans_matrix, pose);
	pose.print();
#endif

#if 0
	GivenVelocityPlanner* planner = new GivenVelocityPlanner();
	Quaternion quat;
	std::vector<basic_alg::Quaternion> quats_in;
	std::vector<basic_alg::Quaternion> quats_out;
	std::ifstream quat_in_file("quat_in.dat", ios::in);
	std::ofstream quat_out_file("quat_out.dat", ios::out);

	stringstream ss;
	string line_str;
	planner->abcSetSmoothWindow(5);
	while (getline(quat_in_file, line_str))
	{
		ss.clear();
		std::cout << line_str << std::endl;
		ss << line_str;
		ss >> quat.w_;
		ss >> quat.x_;
		ss >> quat.y_;
		ss >> quat.z_;
		quats_in.push_back(quat);
}
	quats_out = planner->testQuatSmooth(quats_in);
	for (auto iter = quats_out.begin(); iter != quats_out.end(); ++iter)
	{
		quat_out_file << iter->w_ << " " << iter->x_ << " " << iter->y_ << " " << iter->z_ << "\n";
	}
	quat_in_file.close();
	quat_out_file.close();
	delete planner;
#endif
#if 0
	/*double vp[3][6] = { 
		{0.3285, 0.0948, -0.1181,2.7896,0.0173,0.6946},
		{0.2924, 0.1051, -0.0693, 2.9867, - 0.0417, 0.5907},
		{0.2719, 0.1077, - 0.1185, 2.9245, 0.0783, 0.7557} 
	};*/
	double vp[3][6] = {
	{328.549 ,94.7862 ,-118.101 ,0.69455,0.0172525, 2.78962},
	{292.401 ,105.059 ,-69.3176 ,0.590675 ,-0.0416654 ,2.98672},
	{271.872 ,107.699 ,-118.514 ,0.755667 ,0.0782442 ,2.92448}
	};
	GivenVelocityPlanner* planner = new GivenVelocityPlanner();
	vector<PoseEuler> vp_vec;
	//cout<< sizeof(PoseEuler);
	PoseEuler pos;
	std::vector<PoseEuler> traj;
	std::vector<PoseEuler> pause_traj;

	for (int i = 0; i < 3; ++i)
	{
		pos.point_.x_ = vp[i][0];
		pos.point_.y_ = vp[i][1];
		pos.point_.z_ = vp[i][2];
		pos.euler_.a_ = vp[i][3];
		pos.euler_.b_ = vp[i][4];
		pos.euler_.c_ = vp[i][5];
		vp_vec.push_back(pos);
	}
	planner->viaPoints2Traj(vp_vec);
	planner->trajPausePlan(5000,0.0,0.0,0.0,0.0);

	traj = planner->getResampledTraj();
	pause_traj = planner->getPauseTraj();
	std::ofstream traj_file("vp2traj.dat", ios::out);
	std::ofstream pause_file("pausetraj.dat", ios::out);

	for (auto iter = traj.begin(); iter != traj.end(); ++iter)
	{
		traj_file << setprecision(10) << iter->point_.x_ << " " << iter->point_.y_ << " " << iter->point_.z_ << " "
			<< iter->euler_.a_ << " " << iter->euler_.b_ << " " << iter->euler_.c_ << "\n";
	}
	for (auto iter = pause_traj.begin(); iter != pause_traj.end(); ++iter)
	{
		pause_file << setprecision(10) << iter->point_.x_ << " " << iter->point_.y_ << " " << iter->point_.z_ << " "
			<< iter->euler_.a_ << " " << iter->euler_.b_ << " " << iter->euler_.c_ << "\n";
	}
	traj_file.close();
	pause_file.close();
	delete planner;
#endif

#if 0
	double mat[4][4] = { {1, 2, 3, 4},
		                 {1,  1, 1, 1},
		                 {1,  2, 3, 1},
		                 {1,  2, 3, 4} };

	double eig_vec[4] = {1, 1, 1, 1};
	double eig_val;
	//printf("det = %lf\n", det(mat));
	eigens(&mat[0][0], 4, eig_vec, &eig_val, 0.0001);
	printf("%4.6lf,%4.6lf,%4.6lf\n%4.6lf\n",
		eig_vec[0], eig_vec[1], eig_vec[2], eig_val);
#endif
#if 0
	KinematicsRTM *p_kinematics = new KinematicsRTM(base_dh, arm_dh, 0);
	Transformation* p_transform = new Transformation();
	p_transform->init(p_kinematics);

	vector<double> trj_data;
	string line_str;
	std::ifstream in_file("8vp_joint.txt", ios::in);
	std::ofstream out_file("8vp_cart.dat", ios::out);

	stringstream ss;
	int line_count = 0;
	Joint joint;
	Joint ik_joint;
	PoseEuler pose;
	Posture   posture;
	posture.arm = 1;
	posture.elbow = 1;
	posture.wrist = 1;
	posture.flip = 0;
	while (getline(in_file, line_str))
	{
		//cout << line_str << endl;
		ss.clear();
		ss << line_str;
		ss >> joint.j1_;
		ss >> joint.j2_;
		ss >> joint.j3_;
		ss >> joint.j4_;
		ss >> joint.j5_;
		ss >> joint.j6_;
		p_kinematics->doFK(joint, pose);
		// 322.131351, -0.487172, -125.591228, -0.086831, 0.143245, -2.815061
		pose.point_.x_ = 322.131351;
		pose.point_.y_ = -0.487172;
		pose.point_.z_ = -125.591228;
		pose.euler_.a_ = -0.086831;
		pose.euler_.b_ = 0.143245;
		pose.euler_.c_ = -2.815061;

		if (!p_kinematics->doIK(pose, posture, ik_joint))
		{
			cout << "failed" << endl;
		}

		out_file << pose.point_.x_ << " " << pose.point_.y_ << " " << pose.point_.z_ << " "
			<< pose.euler_.a_ << " " << pose.euler_.b_ << " " << pose.euler_.c_ << " "
			<< "\n";
		//ss >> pose.point_.x_;
		//ss >> pose.point_.y_;
		//ss >> pose.point_.z_;
		//ss >> pose.euler_.c_;
		//ss >> pose.euler_.b_;
		//ss >> pose.euler_.a_;
		//if (!p_kinematics->doIK(pose, posture, joint))
		//{
		//	cout << "failed" << endl;
		//}
		//else
		//{
		//	out_file << joint.j1_ << " " << joint.j2_ << " " << joint.j3_ << " " 
		//		     << joint.j4_ << " " << joint.j5_ << " " << joint.j6_ << " " 
		//		     << "\n";
		//	//joint.print("IK joint:");
		//}
	}
	in_file.close();
	out_file.close();
#endif
#if 0
	KinematicsRTM* p_kinematics = new KinematicsRTM(base_dh, arm_dh, 0);
	Transformation* p_transform = new Transformation();
	p_transform->init(p_kinematics);

	// 359.343473,116.570139,-59.339125,-3.079538,-0.100902,1.329879
	// 335.282198,-0.000186,-65.154291,3.138276,0.154838,6.282933
	PoseAndPosture pose_postrue;
	pose_postrue.pose.point_.x_ = 359.343473;
	pose_postrue.pose.point_.y_ = 116.570139;
	pose_postrue.pose.point_.z_ = -59.339125;
	pose_postrue.pose.euler_.a_ = 1.329879;
	pose_postrue.pose.euler_.b_ = -0.100902;
	pose_postrue.pose.euler_.c_ = -3.079538;

	pose_postrue.posture.arm = 1;
	pose_postrue.posture.elbow = 1;
	pose_postrue.posture.wrist = 1;
	pose_postrue.posture.flip = 0;

	IntactPoint point;
	point.pose = pose_postrue;

	PoseEuler tool_frame;
	tool_frame.point_.x_ = 0; tool_frame.point_.y_ = 0; tool_frame.point_.z_ = 0;
	tool_frame.euler_.a_ = 0; tool_frame.euler_.b_ = 0; tool_frame.euler_.c_ = 0;

	PoseEuler user_frame;
	user_frame.point_.x_ = 0; user_frame.point_.y_ = 0; user_frame.point_.z_ = 0;
	user_frame.euler_.a_ = 0; user_frame.euler_.b_ = 0; user_frame.euler_.c_ = 0;

	point.tool_frame = tool_frame;//group_ptr_->getToolFrame();
	point.user_frame = user_frame;//group_ptr_->getUserFrame();

	PoseEuler tcp_in_base, fcp_in_base;
	p_transform->convertPoseFromUserToBase(point.pose.pose, point.user_frame, tcp_in_base);
	p_transform->convertTcpToFcp(tcp_in_base, point.tool_frame, fcp_in_base);
	Joint joint;
	if (!p_kinematics->doIK(fcp_in_base, pose_postrue.posture, joint))
	{
		cout << "failed" << endl;
	}
	joint.print();
#endif
#if 0
	OnlineTrajectoryPlanner online_plan;
	online_plan.Fir_Bspline_algorithm_test();
#endif

	system("pause");
	return 0;
}
