/*************************************************************************
	> File Name: test_tarjectory_planner.cpp
	> Author: 
	> Mail: 
	> Created Time: 2019骞?1鏈?8鏃?鏄熸湡鍥?14鏃?8鍒?4绉? ************************************************************************/

#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <joint_planner.h>
#include "motion_control_datatype.h"
#include "basic_alg_datatype.h"
#include "common_enum.h"

#include "kinematics.h"
#include "kinematics_rtm.h"
#include "pose_quaternion.h"
#include "joint.h"
#include "bezier_planner.h"
#include "line_smooth_planner.h"
#include "line_planner.h"

#include "dynamic_alg_rtm.h"
#include "common_log.h"
#include "traj_planner.h"
#include "smooth_planner.h"

using namespace basic_alg;

using namespace fst_mc;
using namespace basic_alg;
using namespace std;


void test_movej()
{
    JointPlanner planner;
	planner.initPlanner(6, 3);

	Joint vel_limit, acc_limit, jerk_limit[3];
	vel_limit[0] = 5.8119; vel_limit[1] = 4.6600;  vel_limit[2] = 5.8119;
	vel_limit[3] = 7.8540; vel_limit[4] = 7.0686;  vel_limit[5] = 10.5592;

	acc_limit[0] = 12; acc_limit[1] = 10;  acc_limit[2] = 16;
	acc_limit[3] = 22; acc_limit[4] = 20;  acc_limit[5] = 20;

	jerk_limit[0][0] = 120; jerk_limit[0][1] = 100;  jerk_limit[0][2] = 160;
	jerk_limit[0][3] = 220; jerk_limit[0][4] = 200;  jerk_limit[0][5] = 200;

	jerk_limit[1][0] = 120; jerk_limit[1][1] = 100;  jerk_limit[1][2] = 160;
	jerk_limit[1][3] = 220; jerk_limit[1][4] = 200;  jerk_limit[1][5] = 200;

	jerk_limit[2][0] = 120; jerk_limit[2][1] = 100;  jerk_limit[2][2] = 160;
	jerk_limit[2][3] = 220; jerk_limit[2][4] = 200;  jerk_limit[2][5] = 200;

	planner.setLimit(vel_limit, acc_limit, jerk_limit);

	Joint start, end;
	start[0] = 0; start[1] = 0;  start[2] = 0;
	start[3] = 0; start[4] = 0;  start[5] = 0;
	end[0] = -0.680012; end[1] = -0.166237;  end[2] = -0.62847;
	end[3] = 0.018697; end[4] = -0.76088;  end[5] = -0.693459;
	double vel = 0.5;
	planner.planTrajectory(start, end, vel, 1, 1);
	double total_time = planner.getDuration();

	std::string out_file;

	out_file = "move_joint.csv";
	std::ofstream out(out_file);

	double time_step = 0.001;
	JointState state;
	for (double t = 0; t < total_time + time_step; t += time_step)
	{
		planner.sampleTrajectory(t, state);
		out << t << "," << state.angle[0] << "," << state.angle[1] << "," << state.angle[2] << "," << state.angle[3] << "," << state.angle[4] << "," << state.angle[5]
			<< "," << state.omega[0] << "," << state.omega[1] << "," << state.omega[2] << "," << state.omega[3]	<< "," << state.omega[4] << "," << state.omega[5]
			<< "," << state.alpha[0] << "," << state.alpha[1] << "," << state.alpha[2] 	<< "," << state.alpha[3] << "," << state.alpha[4]  << "," << state.alpha[5]
			<< std::endl;
	}

	out.close();
}


void test_bezier()
{
#if 0
	QuadraticBezierPlanner planner;
	double start[3], via[3], end[3], amax = 5.1, vstart = 1.0, vend = 2.0;

	start[0] = 0.3;	start[1] = 0.150;	start[2] = 0.3;
	via[0] = 0.15;	via[1] = 0.15;	via[2] = 0.3;
	end[0] = 0.3;	end[1] = 0.3;	end[2] = 0.3;

	planner.planCurve(start, via, end, vstart, vend, amax);
	double total_time = planner.getDuration();
	planner.outputCurve(0.001, "bezier_curve.csv");
#endif
}

#if 1
void test_line_smooth()
{
	DH base_dh;
	DH arm_dh[6];

	base_dh.d = 365;      base_dh.a = 0;       base_dh.alpha = 0;              base_dh.offset = 0;
	arm_dh[0].d = 0;        arm_dh[0].a = 30;        arm_dh[0].alpha = M_PI / 2;     arm_dh[0].offset = 0;
	arm_dh[1].d = 0;        arm_dh[1].a = 340.2;     arm_dh[1].alpha = 0;            arm_dh[1].offset = M_PI / 2;
	arm_dh[2].d = 0;        arm_dh[2].a = 34.85;   arm_dh[2].alpha = M_PI / 2;     arm_dh[2].offset = 0;
	arm_dh[3].d = 350.3;     arm_dh[3].a = 0;        arm_dh[3].alpha = -M_PI / 2;    arm_dh[3].offset = 0;
	arm_dh[4].d = 0;        arm_dh[4].a = 0;        arm_dh[4].alpha = M_PI / 2;     arm_dh[4].offset = 0;
	arm_dh[5].d = 96.5;   arm_dh[5].a = 0;        arm_dh[5].alpha = 0;            arm_dh[5].offset = 0;

	KinematicsRTM kinematic(base_dh, &arm_dh[0]);

	LineSmoothPlanner planner;
	planner.initPlanner(6, 0.001, &kinematic);
	planner.setLimit(100000);

	Joint start, via, end;
	start.j1_ = -0.463648;	start.j2_ = 0.025462;	start.j2_ = -0.592228;
	start.j3_ = 0.0;	start.j4_ = -1.00403;	start.j5_ = -0.463648;

	via.j1_ = -0.463648;	via.j2_ = 0.0596357;	via.j2_ = -0.546742;
	via.j3_ = 0.0;			via.j4_ = -1.08369;		via.j5_ = -0.463648;

	end.j1_ = -0.463648;	end.j2_ = 0.0596357;	end.j2_ = -0.546742;
	end.j3_ = 0.0;			end.j4_ = -1.08369;		end.j5_ = -0.463648;

 	//	planner.planTrajectory(start, via, end, 2000.0, 3000.0);

	double time_total = planner.getDuration();
	printf("plan success : time_total = %lf\n", time_total);

	double time_step = 0.001;
	uint32_t point_num = ceil(time_total / time_step);
	Joint ref = start;
	fst_mc::JointState state[5120];

	planner.sampleTrajectory(0, ref, point_num, state);

	std::string out_file("line_smooth_planner.csv");
	std::ofstream out(out_file);

	for (uint32_t i = 0; i != point_num; ++i)
	{
		out << state[i].angle.j1_
			<< "," << state[i].angle.j2_
			<< "," << state[i].angle.j3_
			<< "," << state[i].angle.j4_
			<< "," << state[i].angle.j5_
			<< "," << state[i].angle.j6_
			<< "," << state[i].omega.j1_
			<< "," << state[i].omega.j2_
			<< "," << state[i].omega.j3_
			<< "," << state[i].omega.j4_
			<< "," << state[i].omega.j5_
			<< "," << state[i].omega.j6_
			<< "," << state[i].alpha.j1_
			<< "," << state[i].alpha.j2_
			<< "," << state[i].alpha.j3_
			<< "," << state[i].alpha.j4_
			<< "," << state[i].alpha.j5_
			<< "," << state[i].alpha.j6_
			<< std::endl;
	}
	out.close();

	std::string out_file_cart("line_smooth_planner_cart.csv");
	std::ofstream out_cart(out_file_cart);
	PoseQuaternion pose;
	for (uint32_t i = 0; i != point_num; ++i)
	{
		kinematic.doFK(state[i].angle, pose);
		out_cart << pose.point_.x_
			<< "," << pose.point_.y_
			<< "," << pose.point_.z_
			<< "," << pose.quaternion_.x_
			<< "," << pose.quaternion_.y_
			<< "," << pose.quaternion_.z_
			<< "," << pose.quaternion_.w_
			<< std::endl;
	}
	out_cart.close();
}

#endif


void test_line_planner()
{
	DH base_dh;
	DH arm_dh[6];

	base_dh.d = 365;      base_dh.a = 0;       base_dh.alpha = 0;              base_dh.offset = 0;
	arm_dh[0].d = 0;        arm_dh[0].a = 30;        arm_dh[0].alpha = M_PI / 2;     arm_dh[0].offset = 0;
	arm_dh[1].d = 0;        arm_dh[1].a = 340.2;     arm_dh[1].alpha = 0;            arm_dh[1].offset = M_PI / 2;
	arm_dh[2].d = 0;        arm_dh[2].a = 34.85;   arm_dh[2].alpha = M_PI / 2;     arm_dh[2].offset = 0;
	arm_dh[3].d = 350.3;     arm_dh[3].a = 0;        arm_dh[3].alpha = -M_PI / 2;    arm_dh[3].offset = 0;
	arm_dh[4].d = 0;        arm_dh[4].a = 0;        arm_dh[4].alpha = M_PI / 2;     arm_dh[4].offset = 0;
	arm_dh[5].d = 96.5;   arm_dh[5].a = 0;        arm_dh[5].alpha = 0;            arm_dh[5].offset = 0;

	KinematicsRTM kinematic(base_dh, &arm_dh[0]);

#if 1
	LinePlanner planner;
	planner.initPlanner(6, 1);
	double jerk[3], o_jerk[3];
	jerk[0] = 500000.000000;
	jerk[1] = 150000.000000;
	jerk[2] = 150000.000000;

	o_jerk[0] = 200.000000;
	o_jerk[1] = 60.000000;
	o_jerk[2] = 60.000000;

	planner.setLimit(4000.000000, 25000.000000, jerk, 1.000000, 10.000000, o_jerk );
	PoseEuler start, end;

	//start.point_.x_ = 46.999; start.point_.y_ = -504.91; start.point_.z_ = 313.304;
	//start.euler_.a_ = -2.333000; start.euler_.b_ = 0.484000; start.euler_.c_ = -1.520000;

	//end.point_.x_ = -221.15600; end.point_.y_ = -587.70500; end.point_.z_ = 313.493000;
	//end.euler_.a_ = 2.964000; end.euler_.b_ = 0.4810; end.euler_.c_ = -1.55300;

	start.point_.x_ = 471.691; start.point_.y_ = -377.386; start.point_.z_ = 416.02;
	start.euler_.a_ = 1.028; start.euler_.b_ = -0.515; start.euler_.c_ = 1.522;

	end.point_.x_ = 471.525; end.point_.y_ = -377.2; end.point_.z_ = 410.029;
	end.euler_.a_ = -2.363; end.euler_.b_ = 0.421; end.euler_.c_ = -1.596;

	PoseQuaternion start_o, end_o;
	start.convertToPoseQuaternion(start_o);
	end.convertToPoseQuaternion(end_o);

	Joint ref;
	ref.j1_ = -1.619158;
	ref.j2_ =  -0.749180;
	ref.j3_ = -0.398753;
	ref.j4_ = -0.864317;
	ref.j5_ = 1.333900;
	ref.j6_ =     2.373117;

	Joint joint_start, joint_end;
	kinematic.doIK(start_o, ref, joint_start);
	kinematic.doIK(end_o, ref, joint_end);

	joint_start.print("joint_start : ");
	joint_end.print("joint_end : ");

	start_o.print("start_o:");
	end_o.print("end_o:");

	// planner.planTrajectory(start_o, end_o, 0.140, 0.280, 0.280);

	double total_time = planner.getDuration();
	printf("total_time = %lf\n", total_time);

	std::string out_file;

	out_file = "move_line.csv";
	std::ofstream out(out_file);

	double time_step = 0.001;
	PoseQuaternion state;

	planner.sampleTrajectory(0, state);

	std::cout << 0 << "," << state.point_.x_ << "," << state.point_.y_ << "," << state.point_.z_
		<< "," << state.quaternion_.x_ << "," << state.quaternion_.y_ << "," << state.quaternion_.z_ << "," << state.quaternion_.w_
		<< std::endl;

	Posture posture;
	posture.arm = 1; posture.elbow = 1; posture.wrist = -1; posture.flip = 0;

	Joint sample_joint;
	PoseEuler sample_pe;

#if 1
	for (double t = 0; t < total_time + time_step; t += time_step)
	{
		planner.sampleTrajectory(t, state);
		state.convertToPoseEuler(sample_pe);
		kinematic.doIK(state, posture, sample_joint);
		ref = sample_joint;
		out << t << "," << state.point_.x_ << "," << state.point_.y_ << "," << state.point_.z_ 
			<< "," << state.quaternion_.x_ << "," << state.quaternion_.y_ << "," << state.quaternion_.z_ << "," << state.quaternion_.w_ 
			<< ", "
			<< "," << sample_pe.point_.x_ << "," << sample_pe.point_.y_ << "," << sample_pe.point_.z_ 
			<< "," << sample_pe.euler_.a_ << "," << sample_pe.euler_.b_ << "," << sample_pe.euler_.b_ 
			<< ", "
			<< "," << sample_joint.j1_ << "," << sample_joint.j2_  << "," << sample_joint.j2_ 
			<< "," << sample_joint.j3_ << "," << sample_joint.j5_ << "," << sample_joint.j6_   
			<< std::endl;
	}
#endif
	out.close();
#endif

#if 0
	PoseQuaternion pq;
	PoseEuler pe;
	Joint pq_joint;
	Posture pq_posture;
	//pq.point_.x_ = -176.949; pq.point_.y_ = -574.056; pq.point_.z_ = 313.462;
	//pq.quaternion_.x_ = -0.211795; pq.quaternion_.y_ = -0.443238; 
	//pq.quaternion_.z_ = 0.496118; pq.quaternion_.w_ = -0.0151662; 

	pq.point_.x_ = -176.872; pq.point_.y_ = -574.032; pq.point_.z_ = 313.462;
	pq.quaternion_.x_ = -0.211763; pq.quaternion_.y_ = -0.442851; 
	pq.quaternion_.z_ = 0.49575; pq.quaternion_.w_ = -0.0150092; 



	pq_posture.arm = 1; pq_posture.elbow = 1; pq_posture.wrist = -1; pq_posture.flip = 0;
	kinematic.doIK(pq, pq_posture, pq_joint);
	pq.convertToPoseEuler(pe);
	pq.print("pose quaternion : ");
	printf("  \n");
	pe.print("pose euler : ");
	printf("  \n");
	printf("posrure : arm : elbow : wrist : flip = %d : %d : %d : %d\n",
		pq_posture.arm, pq_posture.elbow, pq_posture.wrist, pq_posture.flip);
	printf("  \n");
	pq_joint.print("pq_joint:");
#endif
}


bool checkOffset(double check_joint, double check_ref, double joint_mask)
{
	if (fabs(joint_mask) < fabs(check_joint - check_ref))
	{
		return false;
	}
	
	return true;
}


void test_traj_smooth()
{
#if 1 
	/*-------------------------------- init class --------------------------*/
	DH base_dh;
	DH arm_dh[6];

	base_dh.d = 365;      base_dh.a = 0;       base_dh.alpha = 0;              base_dh.offset = 0;
	arm_dh[0].d = 0;        arm_dh[0].a = 30;        arm_dh[0].alpha = M_PI / 2;     arm_dh[0].offset = 0;
	arm_dh[1].d = 0;        arm_dh[1].a = 340.2;     arm_dh[1].alpha = 0;            arm_dh[1].offset = M_PI / 2;
	arm_dh[2].d = 0;        arm_dh[2].a = 34.85;   arm_dh[2].alpha = M_PI / 2;     arm_dh[2].offset = 0;
	arm_dh[3].d = 350.3;     arm_dh[3].a = 0;        arm_dh[3].alpha = -M_PI / 2;    arm_dh[3].offset = 0;
	arm_dh[4].d = 0;        arm_dh[4].a = 0;        arm_dh[4].alpha = M_PI / 2;     arm_dh[4].offset = 0;
	arm_dh[5].d = 96.5;   arm_dh[5].a = 0;        arm_dh[5].alpha = 0;            arm_dh[5].offset = 0;

	KinematicsRTM kinematic(base_dh, &arm_dh[0]);

    DynamicAlgRTM dynamics;
    dynamics.initDynamicAlg("/root/install/share/runtime/axis_group/");

	Joint lower, upper;
	lower.j1_ = -2.9670597283903602807702743064306; 
	lower.j2_ = -2.3561944901923449288469825374596;
	lower.j3_ = -1.221730476396030703846583537942; 
	lower.j4_ = -3.3161255787892261961550124601284; 
	lower.j5_ = -2.0071286397934790134622443837619; 
	lower.j6_ = -6.283185307179586476925286766559;
	lower.j7_ = 0;
	lower.j8_ = 0;
	lower.j9_ = 0;

	upper.j1_ = 2.9670597283903602807702743064306; 
	upper.j2_ = 1.7453292519943295769236907684886; 
	upper.j3_ = 3.4906585039886591538473815369772; 
	upper.j4_ = 3.3161255787892261961550124601284; 
	upper.j5_ = 2.0071286397934790134622443837619; 
	upper.j6_ = 6.283185307179586476925286766559;
	upper.j7_ = 0;
	upper.j8_ = 0;
	upper.j9_ = 0;

	int joint_num =  6; 

	Constraint joint_constraint;
	joint_constraint.initConstraint(lower, upper, joint_num);

	double cycle_time = 0.001;

	fst_log::Logger* log_ptr_ = new fst_log::Logger();
	FST_LOG_INIT("test_traj_smooth");
	int log_level = 1;
    FST_LOG_SET_LEVEL((fst_log::MessageLevel)log_level);

	TrajectoryPlanner traj_planner, traj_planner_pre;
	traj_planner.initPlanner(joint_num, cycle_time, &kinematic, &dynamics, &joint_constraint, log_ptr_);
	traj_planner_pre.initPlanner(joint_num, cycle_time, &kinematic, &dynamics, &joint_constraint, log_ptr_);

	SmoothPlanner smooth_planner;
	smooth_planner.initPlanner(joint_num, cycle_time, &kinematic, &joint_constraint, log_ptr_);

	Joint vel_limit_joint, acc_limit_joint, jerk_limit_joint;
	vel_limit_joint.j1_ = 4.6542;
	vel_limit_joint.j2_ = 3.7326;
	vel_limit_joint.j3_ = 4.6542;
	vel_limit_joint.j4_ = 6.2832;
	vel_limit_joint.j5_ = 5.6549;
	vel_limit_joint.j6_ = 8.4446;

	acc_limit_joint.j1_ = 24.000000;
	acc_limit_joint.j2_ = 20.000000;
	acc_limit_joint.j3_ = 24.000000;
	acc_limit_joint.j4_ = 32.000000;
	acc_limit_joint.j5_ = 28.800000;
	acc_limit_joint.j6_ = 40.000000;

	jerk_limit_joint.j1_ = 240.000000;
	jerk_limit_joint.j2_ = 200.000000;
	jerk_limit_joint.j3_ = 240.000000;
	jerk_limit_joint.j4_ = 320.000000;
	jerk_limit_joint.j5_ = 280.000000;
	jerk_limit_joint.j6_ = 400.000000;

	double vel_limit_position = 4000.000000;
	double acc_limit_position = 25000.000000;
	double jerk_limit_position = 500000.000000;

	double vel_limit_orientation = 3.000000;
	double acc_limit_orientation = 30.000000;
	double jerk_limit_orientation = 300.000000;

	smooth_planner.setLimit(vel_limit_joint, acc_limit_joint, jerk_limit_joint, 
		vel_limit_position, acc_limit_position, jerk_limit_position,
		vel_limit_orientation, acc_limit_orientation, jerk_limit_orientation);
#endif

#if 1
	/*------------------------- set param -------------------------------------*/
	PoseEuler uf_frame, tf_frame;
	uf_frame.point_.x_ = 0;
	uf_frame.point_.y_ = 0;
	uf_frame.point_.z_ = 0.0;
	uf_frame.euler_.a_ = 0.0;
	uf_frame.euler_.b_ = 0;
	uf_frame.euler_.c_ = 0;

	tf_frame.point_.x_ = 0;
	tf_frame.point_.y_ = 0;
	tf_frame.point_.z_ = 0;
	tf_frame.euler_.a_ = 0;
	tf_frame.euler_.b_ = 0;
	tf_frame.euler_.c_ = 0;

	traj_planner.setUserFrame(uf_frame);
	traj_planner.setToolFrame(tf_frame);
	traj_planner_pre.setUserFrame(uf_frame);
	traj_planner_pre.setToolFrame(tf_frame);
	smooth_planner.setUserFrame(uf_frame);
	smooth_planner.setToolFrame(tf_frame);

	TransMatrix uf_matrix, tf_matrix, uf_matrix_inverse, tf_matrix_inverse;
	uf_frame.convertToTransMatrix(uf_matrix);
	tf_frame.convertToTransMatrix(tf_matrix);
	
	uf_matrix_inverse = uf_matrix;
	tf_matrix_inverse = tf_matrix;
	uf_matrix_inverse.inverse();
	tf_matrix_inverse.inverse();

	traj_planner.setUserTrans(uf_matrix);
	traj_planner.setToolTrans(tf_matrix);
	traj_planner_pre.setUserTrans(uf_matrix);
	traj_planner_pre.setToolTrans(tf_matrix);
	smooth_planner.setUserTrans(uf_matrix);
	smooth_planner.setToolTrans(tf_matrix);

	uf_matrix.print("uf_matrix : ");
	tf_matrix.print("tf_matrix : ");

	/*------------------------------ init point -----------------------------*/
	PoseEuler pose1, pose14, pose3;


	pose1.point_.x_ = 2.643; 
	pose1.point_.y_ = -469.277; 
	pose1.point_.z_ = 476.423;
	pose1.euler_.a_ = -1.377291673; 
	pose1.euler_.b_ = -1.546291904; 
	pose1.euler_.c_ = 2.95053146;

	pose14.point_.x_ = -2.644; 
	pose14.point_.y_ = -326.372;  
	pose14.point_.z_ = 472.334;
	pose14.euler_.a_ = -1.297512673; 
	pose14.euler_.b_ = -1.553343034; 
	pose14.euler_.c_ = 2.87073500;

	pose3.point_.x_ = 477.865;
	pose3.point_.y_ = -271.765;
	pose3.point_.z_ = 471.971;
	pose3.euler_.a_ = -1.570848687;
	pose3.euler_.b_ = -0.052342424; 
	pose3.euler_.c_ = -3.138939753;

	TransMatrix trans_pose1, trans_pose14, trans_pose3;
	// pose1.convertToTransMatrix(trans_pose1);
	// pose14.convertToTransMatrix(trans_pose14);
	// pose3.convertToTransMatrix(trans_pose3);

	// trans_pose1.leftMultiply(uf_matrix).rightMultiply(tf_matrix);
	// trans_pose14.leftMultiply(uf_matrix).rightMultiply(tf_matrix);
	// trans_pose3.leftMultiply(uf_matrix).rightMultiply(tf_matrix);
#if 1
	Joint joint1, joint14, joint3;

	joint1.j1_ = -1.612277;  joint1.j2_ = -0.741323; joint1.j3_ = -0.420888; joint1.j4_ = -0.852808; joint1.j5_ = 1.347654; joint1.j6_ = 2.366821;
	joint14.j1_ = -1.962508;  joint14.j2_ = -0.903112;  joint14.j3_ = -0.041930; joint14.j4_ = -0.249371; joint14.j5_ = 0.972010; joint14.j6_ = 2.231906;
	joint3.j1_ = -1.962508;  joint3.j2_ = -1.174947; joint3.j3_ = 0.063825; joint3.j4_ = -0.226984; joint3.j5_ = 1.133433; joint3.j6_ = 2.186851;

	// target-joint: -1.612277 -0.741323 -0.420888 -0.852808 1.347654 2.366821
	// target-joint: -1.962508 -0.903112 -0.041930 -0.249371 0.972010 2.231906
	// target-joint: -1.962508 -1.174947 0.063825 -0.226984 1.133433 2.186851 

	kinematic.doFK(joint1, pose1);
	kinematic.doFK(joint14, pose14);
	kinematic.doFK(joint3, pose3);

	PoseQuaternion temp_pose1, temp_pose14, temp_pose3;
	pose1.convertToPoseQuaternion(temp_pose1);
	pose14.convertToPoseQuaternion(temp_pose14);
	pose3.convertToPoseQuaternion(temp_pose3);

	printf("======== temp_pose1 ======");
	temp_pose1.print("");
	printf("======== temp_pose14 ======");
	temp_pose14.print("");
	printf("======== temp_pose3 ======");
	temp_pose3.print("");
	printf("======== over ======");

#endif
	// pose14.convertToTransMatrix(trans_pose14);
	// trans_pose1.leftMultiply(uf_matrix_inverse).rightMultiply(tf_matrix);
	// trans_pose1.convertToPoseEuler(pose14);

	MotionInfo target14, target3;
	target14.type = MOTION_LINE;
	target14.smooth_type = SMOOTH_DISTANCE;
	target14.cnt = 30;
	target14.vel = 4000;
	target14.acc = 1.0;
	target14.target.pose.pose = pose14;
	target14.target.joint = joint14;
	target14.target.pose.posture.arm = 1;
	target14.target.pose.posture.elbow = 1;
	target14.target.pose.posture.wrist = 1;
	target14.target.pose.posture.flip = 0;
	target14.target.pose.turn.j1 = 0;
	target14.target.pose.turn.j2 = 0;
	target14.target.pose.turn.j3 = 0;
	target14.target.pose.turn.j4 = 0;
	target14.target.pose.turn.j5 = 0;
	target14.target.pose.turn.j6 = 0;
	target14.target.user_frame = uf_frame;
	target14.target.tool_frame = tf_frame;

	target3.type = MOTION_LINE;
	target3.smooth_type = SMOOTH_NONE;
	target3.cnt = -1;
	target3.vel = 1000;
	target3.acc = 1.0;
	target3.target.pose.pose = pose3;
	target3.target.joint = joint3;
	target3.target.pose.posture.arm = 1;
	target3.target.pose.posture.elbow = 1;
	target3.target.pose.posture.wrist = 1;
	target3.target.pose.posture.flip = 0;
	target3.target.pose.turn.j1 = 0;
	target3.target.pose.turn.j2 = 0;
	target3.target.pose.turn.j3 = 0;
	target3.target.pose.turn.j4 = 0;
	target3.target.pose.turn.j5 = 0;
	target3.target.pose.turn.j6 = 0;
	target3.target.user_frame = uf_frame;
	target3.target.tool_frame = tf_frame;
#endif // set param

#if 1 
	/*----------------------------- start plan ---------------------------------*/
	uint32_t point_num = 1;

	std::string out_file("l2l_smooth.csv");
	std::ofstream out(out_file);

	traj_planner_pre.planTrajectory(joint1, target14, 0.3, 1.0);
	double t1 = traj_planner_pre.getDuration();
	double t_out, smooth_distance;
	traj_planner_pre.getSmoothOutTimeAndDistance(target14.cnt, 0, t_out, smooth_distance);
	printf("t1 = %lf, t_out = %lf, smooth_distance = %lf\n", t1, t_out, smooth_distance);

	JointState out_state;
	traj_planner_pre.sampleTrajectory(t_out, joint1, point_num, &out_state);

	Joint ref = joint1;
	JointState state;
	double t = 0.0;
	PoseQuaternion pose_fk;
	TransMatrix pose_trans_fk;
	double orientation_out_time = ceil(t1 / cycle_time / 2) * cycle_time;

	double joint_mask = 1.57;
#if 1
	for (; t < orientation_out_time + 0.0000001; t += cycle_time)
	{
		traj_planner_pre.sampleTrajectory(t, ref, point_num, &state);

		kinematic.doFK(state.angle, pose_fk);
		pose_fk.convertToTransMatrix(pose_trans_fk);
		pose_trans_fk.rightMultiply(tf_matrix).leftMultiply(uf_matrix_inverse).convertToPoseQuaternion(pose_fk);

		out << t
			<< "," << state.angle.j1_
			<< "," << state.angle.j2_
			<< "," << state.angle.j3_
			<< "," << state.angle.j4_
			<< "," << state.angle.j5_
			<< "," << state.angle.j6_
			<< "," << state.omega.j1_
			<< "," << state.omega.j2_
			<< "," << state.omega.j3_
			<< "," << state.omega.j4_
			<< "," << state.omega.j5_
			<< "," << state.omega.j6_
			<< "," << state.alpha.j1_
			<< "," << state.alpha.j2_
			<< "," << state.alpha.j3_
			<< "," << state.alpha.j4_
			<< "," << state.alpha.j5_
			<< "," << state.alpha.j6_
			<< "," << pose_fk.point_.x_
			<< "," << pose_fk.point_.y_
			<< "," << pose_fk.point_.z_
			<< "," << pose_fk.quaternion_.x_
			<< "," << pose_fk.quaternion_.y_
			<< "," << pose_fk.quaternion_.z_
			<< "," << pose_fk.quaternion_.w_
			<< std::endl;

		if (!checkOffset(state.angle.j1_, ref.j1_, joint_mask)
			|| !checkOffset(state.angle.j2_, ref.j2_, joint_mask)
			|| !checkOffset(state.angle.j3_, ref.j3_, joint_mask)
			|| !checkOffset(state.angle.j4_, ref.j4_, joint_mask)
			|| !checkOffset(state.angle.j5_, ref.j5_, joint_mask)
			|| !checkOffset(state.angle.j6_, ref.j6_, joint_mask))
		{
			printf("ik err : ");
			state.angle.print("state.angle");
			ref.print("ref : ");
		}

		ref = state.angle;
	}
#endif
	// out_state.angle = state.angle;
	// out_state.omega = state.omega;
	// out_state.alpha = state.alpha;

	out_state.angle.print("out_state.angle : ");
	out_state.omega.print("out_state.omega : ");
	out_state.alpha.print("out_state.alpha : ");

	traj_planner.planTrajectory(joint14, target3, 0.3, 1.0);
	double t2 = traj_planner.getDuration();
	double t_in = traj_planner.getSmoothInTime(smooth_distance);
	double orientation_in_time = floor(t2 / cycle_time / 2) * cycle_time;

	JointState in_state;
	traj_planner.sampleTrajectory(t_in, joint14, point_num, &in_state);
	in_state.angle.print("in_state.angle : ");
	in_state.omega.print("in_state.omega : ");
	in_state.alpha.print("in_state.alpha : ");

	PoseQuaternion out_pose, in_pose;
	TransMatrix out_pose_trans,   in_pose_trans;

	kinematic.doFK(out_state.angle, out_pose);
	out_pose.convertToTransMatrix(out_pose_trans);
	out_pose_trans.rightMultiply(tf_matrix).leftMultiply(uf_matrix_inverse).convertToPoseQuaternion(out_pose);

	kinematic.doFK(in_state.angle, in_pose);
	in_pose.convertToTransMatrix(in_pose_trans);
	in_pose_trans.rightMultiply(tf_matrix).leftMultiply(uf_matrix_inverse).convertToPoseQuaternion(in_pose);

	out_pose.print("out_pose :");
	in_pose.print("in_pose :");

	// smooth_planner.setMotionType(MOTION_LINE, MOTION_LINE);

	smooth_planner.setTrajectoryPlanner(traj_planner_pre, traj_planner);

	smooth_planner.updateTrajectoryInfo(t_out, t_in, 0.001, orientation_out_time, orientation_in_time);
	ErrorCode err = smooth_planner.planTrajectory(out_state, joint14, in_state);

	if(err != SUCCESS)
	{
		printf("plan smooth err : 0x%llx\n", err);
		out.close();
		return;
	}
	double smooth_time = smooth_planner.getDuration();

	printf("smooth_time total = %lf, traj1 total = %lf, traj2 total = %lf, %lf : %lf\n", 	
		smooth_time, traj_planner_pre.getDuration(), traj_planner.getDuration(), t1, t2);
	ref = joint14;

#if 1
	for (t = cycle_time; t < smooth_time; t += cycle_time)
	{
		smooth_planner.sampleTrajectory(t, ref, point_num, &state);

		kinematic.doFK(state.angle, pose_fk);
		pose_fk.convertToTransMatrix(pose_trans_fk);
		pose_trans_fk.rightMultiply(tf_matrix).leftMultiply(uf_matrix_inverse).convertToPoseQuaternion(pose_fk);

		out << (t + orientation_out_time)
			<< "," << state.angle.j1_
			<< "," << state.angle.j2_
			<< "," << state.angle.j3_
			<< "," << state.angle.j4_
			<< "," << state.angle.j5_
			<< "," << state.angle.j6_
			<< "," << state.omega.j1_
			<< "," << state.omega.j2_
			<< "," << state.omega.j3_
			<< "," << state.omega.j4_
			<< "," << state.omega.j5_
			<< "," << state.omega.j6_
			<< "," << state.alpha.j1_
			<< "," << state.alpha.j2_
			<< "," << state.alpha.j3_
			<< "," << state.alpha.j4_
			<< "," << state.alpha.j5_
			<< "," << state.alpha.j6_
			<< "," << pose_fk.point_.x_
			<< "," << pose_fk.point_.y_
			<< "," << pose_fk.point_.z_
			<< "," << pose_fk.quaternion_.x_
			<< "," << pose_fk.quaternion_.y_
			<< "," << pose_fk.quaternion_.z_
			<< "," << pose_fk.quaternion_.w_
			<< std::endl;


		if (!checkOffset(state.angle.j1_, ref.j1_, joint_mask)
			|| !checkOffset(state.angle.j2_, ref.j2_, joint_mask)
			|| !checkOffset(state.angle.j3_, ref.j3_, joint_mask)
			|| !checkOffset(state.angle.j4_, ref.j4_, joint_mask)
			|| !checkOffset(state.angle.j5_, ref.j5_, joint_mask)
			|| !checkOffset(state.angle.j6_, ref.j6_, joint_mask))
		{
			printf("ik err : ");
			state.angle.print("state.angle");
			ref.print("ref : ");
		}

		ref = state.angle;
	}
#endif
	printf("smooth traj over\n");

	double delta = cycle_time - (smooth_time - (t - cycle_time));

	ref = state.angle;

	printf("traj2 sample : start\n");
#if 1
	for ( t = orientation_in_time + delta; t < t2 + cycle_time; t += cycle_time)
	{
		traj_planner.sampleTrajectory(t, ref, point_num, &state);

		kinematic.doFK(state.angle, pose_fk);
		pose_fk.convertToTransMatrix(pose_trans_fk);
		pose_trans_fk.rightMultiply(tf_matrix).leftMultiply(uf_matrix_inverse).convertToPoseQuaternion(pose_fk);

		out << (t + orientation_out_time + smooth_time + delta - orientation_in_time - delta)
			<< "," << state.angle.j1_
			<< "," << state.angle.j2_
			<< "," << state.angle.j3_
			<< "," << state.angle.j4_
			<< "," << state.angle.j5_
			<< "," << state.angle.j6_
			<< "," << state.omega.j1_
			<< "," << state.omega.j2_
			<< "," << state.omega.j3_
			<< "," << state.omega.j4_
			<< "," << state.omega.j5_
			<< "," << state.omega.j6_
			<< "," << state.alpha.j1_
			<< "," << state.alpha.j2_
			<< "," << state.alpha.j3_
			<< "," << state.alpha.j4_
			<< "," << state.alpha.j5_
			<< "," << state.alpha.j6_
			<< "," << pose_fk.point_.x_
			<< "," << pose_fk.point_.y_
			<< "," << pose_fk.point_.z_
			<< "," << pose_fk.quaternion_.x_
			<< "," << pose_fk.quaternion_.y_
			<< "," << pose_fk.quaternion_.z_
			<< "," << pose_fk.quaternion_.w_
			<< std::endl;

		if (!checkOffset(state.angle.j1_, ref.j1_, joint_mask)
			|| !checkOffset(state.angle.j2_, ref.j2_, joint_mask)
			|| !checkOffset(state.angle.j3_, ref.j3_, joint_mask)
			|| !checkOffset(state.angle.j4_, ref.j4_, joint_mask)
			|| !checkOffset(state.angle.j5_, ref.j5_, joint_mask)
			|| !checkOffset(state.angle.j6_, ref.j6_, joint_mask))
		{
			printf("ik err t = %lf\n ", t);
			PoseQuaternion temp_pose;
			kinematic.doFK(state.angle, temp_pose);
			temp_pose.print("temp_pose state : ");
			state.angle.print("state.angle : ");
			ref.print("ref : ");
			kinematic.doFK(ref, temp_pose);
			temp_pose.print("temp_pose ref : ");
		}

		ref = state.angle;
	}

	printf("traj2 sample : over\n");
	out.close();
#endif //  start plan
#endif
	printf("t_out = %lf, t_in = %lf, orientation_out_t = %lf, orientation_in_t = %lf\n", t_out, t_in, orientation_out_time, orientation_in_time);

#if 0
	std::string bezier_out_file("bezier_smooth.csv");
	std::ofstream bezier_out(bezier_out_file);

	ref = joint14;
	cycle_time = 0.001;
	for (t = 0; t < smooth_time; t += cycle_time)
	{
		smooth_planner.sampleTrajectory(t, ref, point_num, &state);
		ref = state.angle;

		kinematic.doFK(state.angle, pose_fk);
		pose_fk.convertToTransMatrix(pose_trans_fk);
		pose_trans_fk.rightMultiply(tf_matrix).leftMultiply(uf_matrix_inverse).convertToPoseQuaternion(pose_fk);

		bezier_out << t
			<< "," << state.angle.j1_
			<< "," << state.angle.j2_
			<< "," << state.angle.j3_
			<< "," << state.angle.j4_
			<< "," << state.angle.j5_
			<< "," << state.angle.j6_
			<< "," << state.omega.j1_
			<< "," << state.omega.j2_
			<< "," << state.omega.j3_
			<< "," << state.omega.j4_
			<< "," << state.omega.j5_
			<< "," << state.omega.j6_
			<< "," << state.alpha.j1_
			<< "," << state.alpha.j2_
			<< "," << state.alpha.j3_
			<< "," << state.alpha.j4_
			<< "," << state.alpha.j5_
			<< "," << state.alpha.j6_
			<< "," << pose_fk.point_.x_
			<< "," << pose_fk.point_.y_
			<< "," << pose_fk.point_.z_
			<< "," << pose_fk.quaternion_.x_
			<< "," << pose_fk.quaternion_.y_
			<< "," << pose_fk.quaternion_.z_
			<< "," << pose_fk.quaternion_.w_
			<< std::endl;
	}

	smooth_planner.sampleTrajectory(smooth_time, ref, point_num, &state);

	kinematic.doFK(state.angle, pose_fk);
	pose_fk.convertToTransMatrix(pose_trans_fk);
	pose_trans_fk.rightMultiply(tf_matrix).leftMultiply(uf_matrix_inverse).convertToPoseQuaternion(pose_fk);

	bezier_out << smooth_time
			<< "," << state.angle.j1_
			<< "," << state.angle.j2_
			<< "," << state.angle.j3_
			<< "," << state.angle.j4_
			<< "," << state.angle.j5_
			<< "," << state.angle.j6_
			<< "," << state.omega.j1_
			<< "," << state.omega.j2_
			<< "," << state.omega.j3_
			<< "," << state.omega.j4_
			<< "," << state.omega.j5_
			<< "," << state.omega.j6_
			<< "," << state.alpha.j1_
			<< "," << state.alpha.j2_
			<< "," << state.alpha.j3_
			<< "," << state.alpha.j4_
			<< "," << state.alpha.j5_
			<< "," << state.alpha.j6_
			<< "," << pose_fk.point_.x_
			<< "," << pose_fk.point_.y_
			<< "," << pose_fk.point_.z_
			<< "," << pose_fk.quaternion_.x_
			<< "," << pose_fk.quaternion_.y_
			<< "," << pose_fk.quaternion_.z_
			<< "," << pose_fk.quaternion_.w_
			<< std::endl;

	bezier_out.close();
#endif
}




int main()
{
     test_traj_smooth();
    return 0;
}



