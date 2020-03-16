#include <fstream>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include "basic_alg.h"
#include "bezier_planner.h"
using namespace basic_alg;

BezierPlanner::BezierPlanner(uint32_t curve_power):
	curve_power_(curve_power)
{
	t_total_ = 0.0;
	vmax_ = 0.0;
	vmin_ = 0.0;
	amax_ = 0.0;
	orientation_angle_ = 0.0;

	orientation_smooth_former_time_= 0.0;
	orientation_smooth_former_angle_ = 0.0;
	orientation_smooth_last_time_ = 0.0;
	orientation_smooth_last_angle_ = 0.0;

	memset(s_, 0, sizeof(s_));
	memset(x_, 0, sizeof(x_));
	memset(y_, 0, sizeof(y_));
	memset(z_, 0, sizeof(z_));
	memset(t_, 0, sizeof(t_));
	memset(rr_, 0, sizeof(rr_));
}

void BezierPlanner::planCurve(const PoseQuaternion &start, const PoseQuaternion &via, const PoseQuaternion &end, 
		double vstart_p, double vend_p, double amax_p, double ustart, double uend, double vstart_u,double vend_u)
{
	ustart_ = ustart;
	uend_ = uend;

	start_quatern_.x_ = start.quaternion_.x_; start_quatern_.y_ = start.quaternion_.y_;
	start_quatern_.z_ = start.quaternion_.z_; start_quatern_.w_ = start.quaternion_.w_;

	end_quatern_.x_ = end.quaternion_.x_; end_quatern_.y_ = end.quaternion_.y_;
	end_quatern_.z_ = end.quaternion_.z_; end_quatern_.w_ = end.quaternion_.w_;

	via_quatern_.x_ = via.quaternion_.x_; via_quatern_.y_ = via.quaternion_.y_;
	via_quatern_.z_ = via.quaternion_.z_; via_quatern_.w_ = via.quaternion_.w_;

	amax_ = amax_p;
	double start_point[3], via_point[3], end_point[3];
	point2Double(start.point_, start_point);
	point2Double(via.point_, via_point);
	point2Double(end.point_, end_point);
	buildTable(start_point, via_point, end_point);

	double min_curva = rr_[0], p_min_curva = -1;
	for (int i = 1; i < DIV_PARTS + 1; i++)
	{
		if (rr_[i] < min_curva) {
			min_curva = rr_[i];
			p_min_curva = (double)i / (double)DIV_PARTS;
		}
	}

	vmin_ = sqrt(amax_ * min_curva);
	vmax_ = vstart_p * (1 - p_min_curva) + vend_p * p_min_curva;
	if (vmax_ - vmin_ < MINIMUM_E12) vmin_ = vmax_;

	double speed_params[4];
	getSpeedParameters(vstart_p, vend_p, vmin_, p_min_curva, speed_params);

	t_[0] = 0;
	t_total_ = 0;
	double delta_u = 1.0 / (double)DIV_PARTS, delta_u_to_u0;
	double speed_u, speed_delta_u, speed_average;
	for (int i = 0; i < DIV_PARTS; i++)
	{
		delta_u_to_u0 = (double)i / (double)DIV_PARTS;

		speed_u = getSpeed(delta_u_to_u0, speed_params);
		speed_delta_u = getSpeed(delta_u_to_u0 + delta_u, speed_params);
		speed_average = (speed_u + speed_delta_u) / 2.0;

		t_total_ += s_[i + 1] / speed_average;
		t_[i + 1] = t_total_;
	}

	orientation_angle_ = end.quaternion_.getIncludedAngle(start.quaternion_);
	orientation_smooth_former_angle_ = via.quaternion_.getIncludedAngle(start.quaternion_);
	orientation_smooth_last_angle_ = end.quaternion_.getIncludedAngle(via.quaternion_);

	orientation_smooth_former_time_ = t_total_ / 2;
	orientation_smooth_last_time_ = t_total_ / 2;

	cubic_spline_smooth_.planCurve(ustart, uend, vstart_u, vend_u, orientation_smooth_former_time_, orientation_smooth_last_time_);
}

void BezierPlanner::sampleCurve(double t, PoseQuaternion &point)
{
	int index = 1;
	for (; index < DIV_PARTS; index++)
	{
		if (t < t_[index]) break;
	}

	double t_ratio = fabs((t - t_[index - 1]) / (t_[index] - t_[index - 1]));

	point.point_.x_ = (1 - t_ratio) * x_[index - 1] + t_ratio * x_[index];
	point.point_.y_ = (1 - t_ratio) * y_[index - 1] + t_ratio * y_[index];
	point.point_.z_ = (1 - t_ratio) * z_[index - 1] + t_ratio * z_[index];

	// double t_total_ratio = fabs(t / t_total_) <  1.0 ? fabs(t / t_total_) : 1.0;
	double u_ratio = 0.0; 
	cubic_spline_smooth_.sampleCurve(t, u_ratio);

	if (t < orientation_smooth_former_time_ + MINIMUM_E9)
	{
		double ratio_last = fabs(1.0 - ustart_) < MINIMUM_E9 ? MINIMUM_E9 : fabs(1.0 - ustart_);
		double ratio_former = fabs(u_ratio - ustart_) < MINIMUM_E9 ? MINIMUM_E9 : fabs(u_ratio - ustart_);

		u_ratio = ratio_former / ratio_last;
		basic_alg::sampleSlerpInterpolationQuaternion(start_quatern_, via_quatern_, u_ratio, orientation_smooth_former_angle_, point.quaternion_);
	}
	else 
	{
		if (uend_ < MINIMUM_E9) 
		{
			uend_ = MINIMUM_E9;
			u_ratio = MINIMUM_E9;
		}
		else
		{
			u_ratio = u_ratio > uend_ ? uend_ : u_ratio;
			u_ratio = u_ratio / uend_;
		}

		basic_alg::sampleSlerpInterpolationQuaternion(via_quatern_, end_quatern_, u_ratio, orientation_smooth_last_angle_, point.quaternion_);
	}
}

void BezierPlanner::outputCurve(double time_step, const char *file_name)
{
	std::string out_file;
	PoseQuaternion ps;
	double total_time = t_total_ + time_step;

	if (file_name != NULL)
	{
		out_file = file_name;
	}
	else
	{
		out_file = "bezier_curve.csv";
	}

	std::ofstream out(out_file);

	for (double t = 0; t < total_time; t += time_step)
	{
		sampleCurve(t, ps);
		out << t
			<< "," << ps.point_.x_ << "," << ps.point_.y_ << "," << ps.point_.z_
			<< "," << ps.quaternion_.x_ << "," << ps.quaternion_.y_ << "," << ps.quaternion_.z_ << "," << ps.quaternion_.w_
			<< "," << sqrt(ps.quaternion_.x_ * ps.quaternion_.x_ + ps.quaternion_.y_ * ps.quaternion_.y_
				+ ps.quaternion_.z_ * ps.quaternion_.z_ + ps.quaternion_.w_ * ps.quaternion_.w_)
			<< std::endl;
	}
	out.close();
}


double BezierPlanner::getDuration(void)
{
	return t_total_;;
}


void BezierPlanner::getPoint(const double(&start)[3], const double(&via)[3], const double(&end)[3], double u, double(&point)[3])
{
	point[0] = (1 - u) * (1 - u) * start[0] + 2 * u * (1 - u) * via[0] + u * u * end[0];
	point[1] = (1 - u) * (1 - u) * start[1] + 2 * u * (1 - u) * via[1] + u * u * end[1];
	point[2] = (1 - u) * (1 - u) * start[2] + 2 * u * (1 - u) * via[2] + u * u * end[2];
}


void BezierPlanner::getCubicCurvePoint(const double(&start)[3], const double(&via)[3], const double(&end)[3], double u, double(&point)[3])
{
	point[0] = (1 - u) * (1 - u) * (1 - u) * start[0] + 3 * u * (1 - u) * via[0] + u * u * u * end[0];
	point[1] = (1 - u) * (1 - u) * (1 - u) * start[1] + 3 * u * (1 - u) * via[1] + u * u * u * end[1];
	point[2] = (1 - u) * (1 - u) * (1 - u) * start[2] + 3 * u * (1 - u) * via[2] + u * u * u * end[2];
}

bool BezierPlanner::getCurvature(const double(&start)[3], const double(&via)[3], const double(&end)[3], double u, double &curvature)
{
	double delta_u = 1.0 / DIV_PARTS;
	double tiny_delta_u = delta_u / 100;
	
	double p0[3], p0a[3], p1[3], p1a[3];
	if (curve_power_ == QUADRATIC)
	{
		getPoint(start, via, end, u, p0);
		getPoint(start, via, end, u + tiny_delta_u, p0a);
		getPoint(start, via, end, u + delta_u, p1);
		getPoint(start, via, end, u + delta_u + tiny_delta_u, p1a);
	}
	else
	{
		getCubicCurvePoint(start, via, end, u, p0);
		getCubicCurvePoint(start, via, end, u + tiny_delta_u, p0a);
		getCubicCurvePoint(start, via, end, u + delta_u, p1);
		getCubicCurvePoint(start, via, end, u + delta_u + tiny_delta_u, p1a);
	}

	double vector_p02p1[3];
	subVector(p1, p0, vector_p02p1);
	double vector_p02p1_norm = norm(vector_p02p1);
	vector_p02p1_norm = vector_p02p1_norm < MINIMUM_E9 ? MINIMUM_E9 : vector_p02p1_norm;

	double vector_p02p0a[3], vector_p12p1a[3];
	subVector(p0a, p0, vector_p02p0a);
	subVector(p1a, p1, vector_p12p1a);

	double angle = getAngleBetweenTwoVectors(vector_p02p0a, vector_p12p1a);

	double kk = fabs(angle) < MINIMUM_E9 ? MINIMUM_E9 : fabs(angle / vector_p02p1_norm);

	curvature = 1 / kk;
	return true;
}

void BezierPlanner::getSpeedParameters(double vstart, double vend, double vmin, double p_vmin, double(&params)[4])
{
	params[0] = p_vmin;
	params[1] = vmin;
	params[2] = (vstart - vmin) / (p_vmin * p_vmin);
	params[3] = (vend - vmin) / ((p_vmin - 1) * (p_vmin - 1));
}

double BezierPlanner::getSpeed(double u, const double(&index)[4])
{
	return (u <= index[0]) ? (index[2] * (index[0] - u) * (index[0] - u) + index[1]) : (index[3] * (index[0] - u) * (index[0] - u) + index[1]);
}

void BezierPlanner::buildTable(const double(&start)[3], const double(&via)[3], const double(&end)[3])
{
	double u = 0.0, delta_u = 1.0 / DIV_PARTS, tiny_delta_u = delta_u / 100;
	double p0[3], p0a[3], p1[3], p1a[3], vector_p02p1[3], vector_p02p0a[3], vector_p12p1a[3];
	double vector_p02p1_norm, angle, kk;

	s_[0] = 0;

	for (int i = 0; i < DIV_PARTS + 1; ++i)
	{
		u = i / (double)DIV_PARTS;

		if (curve_power_ == QUADRATIC)
		{
			getPoint(start, via, end, u, p0);
			getPoint(start, via, end, u + tiny_delta_u, p0a);
			getPoint(start, via, end, u + delta_u, p1);
			getPoint(start, via, end, u + delta_u + tiny_delta_u, p1a);
		}
		else
		{
			getCubicCurvePoint(start, via, end, u, p0);
			getCubicCurvePoint(start, via, end, u + tiny_delta_u, p0a);
			getCubicCurvePoint(start, via, end, u + delta_u, p1);
			getCubicCurvePoint(start, via, end, u + delta_u + tiny_delta_u, p1a);
		}

		x_[i] = p0[0]; 
		y_[i] = p0[1]; 
		z_[i] = p0[2];

		subVector(p1, p0, vector_p02p1);
		vector_p02p1_norm = norm(vector_p02p1);
		if (i < DIV_PARTS) s_[i + 1] = vector_p02p1_norm;

		subVector(p0a, p0, vector_p02p0a);
		subVector(p1a, p1, vector_p12p1a);
		angle = getAngleBetweenTwoVectors(vector_p02p0a, vector_p12p1a);

		kk = (fabs(angle) < MINIMUM_E12) ? MINIMUM_E12 : fabs(angle/ vector_p02p1_norm);
		rr_[i] = 1 / kk;
	}
}
