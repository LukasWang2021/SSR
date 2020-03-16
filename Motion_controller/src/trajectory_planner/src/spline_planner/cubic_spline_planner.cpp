#include <fstream>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <string>
#include "spline_planner/cubic_spline_planner.h"
#include "basic_constants.h"

using namespace std;

CubicSplinePlanner::CubicSplinePlanner()
{
    t_total_ = 0;
    memset(coefficients_, 0, sizeof(coefficients_));
}

void CubicSplinePlanner::planCurve(double u0, double u1, double v0, double v1, double t_total)
{
    t_total_ = t_total;
    if (t_total < MINIMUM_E6)
    {
        coefficients_[0] = u1;
        coefficients_[1] = v1;
        coefficients_[2] = 0;
        coefficients_[3] = 0;
        return;
    }

    double t[] ={1, t_total_, t_total_*t_total_, t_total_*t_total_*t_total_};

    coefficients_[0] = u0;
    coefficients_[1] = v0;
    coefficients_[2] = (u1 * 3 - u0 * 3 - v0 * t[1] * 2 - v1 * t[1]) / t[2];
    coefficients_[3] = (u0 * 2 - u1 * 2 + v0 * t[1] + v1 * t[1]) / t[3];
}

void CubicSplinePlanner::sampleCurve(double t, double &u)
{
    double t_time[] ={1, t, t*t, t*t*t};

    u = 0;
    for (int i = 0; i != COFF_NUM; ++i)
    {
        u += coefficients_[i] * t_time[i];
    }
}

void CubicSplinePlanner::sampleCurve(double t, double &u, double &v, double &a)
{
    double t_time[] ={1, t, t*t, t*t*t};

    u = 0;
    for (int i = 0; i != COFF_NUM; ++i)
    {
        u += coefficients_[i] * t_time[i];
    }

    for (int i = 1; i != COFF_NUM; ++i)
    {
        v += i * coefficients_[i] * t_time[i - 1];
    }

    a = 2 * coefficients_[2] + 6 * coefficients_[3] * t;
}

void CubicSplinePlanner::outputCurve(double time_step, const char *file_name)
{
    string out_file;
	double ps, vs, as;
	double total_time = t_total_ + time_step;

	if (file_name != NULL)
	{
		out_file = file_name;
	}
	else
	{
		out_file = "cubic_spline_curve.csv";
	}

	std::ofstream out(out_file);

	for (double t = 0; t < total_time; t += time_step)
	{
		sampleCurve(t, ps, vs, as);
		out << t << "," << ps << "," << vs << "," << as << std::endl;
	}
	out.close();
}

double CubicSplinePlanner::getDuration()
{
    return t_total_;
}

