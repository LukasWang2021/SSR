#include <fstream>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include "basic_alg.h"
#include "bezier_planner/postion_planner.h"
using namespace basic_alg;

BezierPositionPlanner::BezierPositionPlanner(uint32_t curve_power):
	curve_power_(curve_power)
{
	t_total_ = 0.0;
	vmax_ = 0.0;
	vmin_ = 0.0;
	amax_ = 0.0;
	t_min_curva_ = 0.0;

	memset(s_, 0, sizeof(s_));
	memset(x_, 0, sizeof(x_));
	memset(y_, 0, sizeof(y_));
	memset(z_, 0, sizeof(z_));
	memset(t_, 0, sizeof(t_));
	memset(rr_, 0, sizeof(rr_));
}

void BezierPositionPlanner::planCurve(const basic_alg::Point &start, const basic_alg::Point &via, const basic_alg::Point &end, 
		double vstart, double vend, double amax)
{
	amax_ = amax;
	double start_point[3], via_point[3], end_point[3];
	point2Double(start, start_point);
	point2Double(via, via_point);
	point2Double(end, end_point);
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
	vmax_ = vstart * (1 - p_min_curva) + vend * p_min_curva;
	if (vmax_ - vmin_ < MINIMUM_E12) vmin_ = vmax_;

	double speed_params[4];
	getSpeedParameters(vstart, vend, vmin_, p_min_curva, speed_params);

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

	t_min_curva_ = p_min_curva * t_total_;
}


void BezierPositionPlanner::sampleCurve(double t, Point &point)
{
	int index = 1;
	for (; index < DIV_PARTS; index++)
	{
		if (t < t_[index]) break;
	}

	double t_ratio = fabs((t - t_[index - 1]) / (t_[index] - t_[index - 1]));

	point.x_ = (1 - t_ratio) * x_[index - 1] + t_ratio * x_[index];
	point.y_ = (1 - t_ratio) * y_[index - 1] + t_ratio * y_[index];
	point.z_ = (1 - t_ratio) * z_[index - 1] + t_ratio * z_[index];
}

void BezierPositionPlanner::outputCurve(double time_step, const char *file_name)
{
	std::string out_file;
	Point ps;
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
		out << t << "," << ps.x_ << "," << ps.y_ << "," << ps.z_ << std::endl;
	}
	out.close();
}


double BezierPositionPlanner::getDuration(void)
{
	return t_total_;;
}


void BezierPositionPlanner::getPoint(const double(&start)[3], const double(&via)[3], const double(&end)[3], double u, double(&point)[3])
{
	point[0] = (1 - u) * (1 - u) * start[0] + 2 * u * (1 - u) * via[0] + u * u * end[0];
	point[1] = (1 - u) * (1 - u) * start[1] + 2 * u * (1 - u) * via[1] + u * u * end[1];
	point[2] = (1 - u) * (1 - u) * start[2] + 2 * u * (1 - u) * via[2] + u * u * end[2];
}


void BezierPositionPlanner::getCubicCurvePoint(const double(&start)[3], const double(&via)[3], const double(&end)[3], double u, double(&point)[3])
{
	point[0] = (1 - u) * (1 - u) * (1 - u) * start[0] + 3 * u * (1 - u) * via[0] + u * u * u * end[0];
	point[1] = (1 - u) * (1 - u) * (1 - u) * start[1] + 3 * u * (1 - u) * via[1] + u * u * u * end[1];
	point[2] = (1 - u) * (1 - u) * (1 - u) * start[2] + 3 * u * (1 - u) * via[2] + u * u * u * end[2];
}

bool BezierPositionPlanner::getCurvature(const double(&start)[3], const double(&via)[3], const double(&end)[3], double u, double &curvature)
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

void BezierPositionPlanner::getSpeedParameters(double vstart, double vend, double vmin, double p_vmin, double(&params)[4])
{
	params[0] = p_vmin;
	params[1] = vmin;
	params[2] = (vstart - vmin) / (p_vmin * p_vmin);
	params[3] = (vend - vmin) / ((p_vmin - 1) * (p_vmin - 1));
}

double BezierPositionPlanner::getSpeed(double u, const double(&index)[4])
{
	return (u <= index[0]) ? (index[2] * (index[0] - u) * (index[0] - u) + index[1]) : (index[3] * (index[0] - u) * (index[0] - u) + index[1]);
}

void BezierPositionPlanner::buildTable(const double(&start)[3], const double(&via)[3], const double(&end)[3])
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


double BezierPositionPlanner::getMinCurvaTime(void)
{
	return t_min_curva_;
}