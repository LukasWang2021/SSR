#include <fstream>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <string>
#include "spline_planner/cubic_spline_planner.h"
#include "spline_planner/cubic_spline_smooth.h"

using namespace std;

CubicSplineSmooth::CubicSplineSmooth()
{
    u0_ = 0.0;
    u1_ = 0.0;
    t_total_former_ = 0.0;
    t_total_last_ = 0.0;
}

void CubicSplineSmooth::planCurve(double u0, double u1, double v0, double v1, double t_total_former, double t_total_last)
{
    u0_ = u0;
    u1_ = u1;

    t_total_former_ = t_total_former;
    t_total_last_ = t_total_last;

    cubic_spline_former_.planCurve(u0, 1, v0, 0, t_total_former);
    cubic_spline_last_.planCurve(0, u1, 0, v1, t_total_last);
}

void CubicSplineSmooth::sampleCurve(double t, double &u)
{
    if (t < MINIMUM_E9)
    {
        u = u0_;
    }
    else if (t < t_total_former_ + MINIMUM_E12)
    {
        cubic_spline_former_.sampleCurve(t, u);
    }
    else if (t < t_total_former_ + t_total_last_ + MINIMUM_E12)
    {
        cubic_spline_last_.sampleCurve(t - t_total_former_, u);
    }
    else 
    {
        u = u1_;
    }
}

void CubicSplineSmooth::sampleCurve(double t, double &u, double &v, double &a)
{
    if (t < MINIMUM_E9)
    {
        return;
    }
    else if (t < t_total_former_ + MINIMUM_E12)
    {
        cubic_spline_former_.sampleCurve(t, u, v, a);
    }
    else
    {
        cubic_spline_last_.sampleCurve(t - t_total_former_, u, v, a);
    }
}

void CubicSplineSmooth::outputCurve(double time_step, const char *file_name)
{
    string out_file;
	double ps, vs, as;
	double total_time = t_total_former_ + t_total_last_ + time_step;

	if (file_name != NULL)
	{
		out_file = file_name;
	}
	else
	{
		out_file = "cubic_spline_smooth_curve.csv";
	}

	std::ofstream out(out_file);

	for (double t = 0; t < total_time; t += time_step)
	{
		sampleCurve(t, ps, vs, as);
		out << t << "," << ps << "," << vs << "," << as << endl;
	}
	out.close();
}

double CubicSplineSmooth::getDuration(void)
{
    if (t_total_former_ + t_total_last_ < MINIMUM_E9) return MINIMUM_E9;
    return t_total_former_ + t_total_last_;
}
