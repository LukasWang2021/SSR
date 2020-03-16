#ifndef SPLINE_ORIENTATION_PLANNER_H
#define SPLINE_ORIENTATION_PLANNER_H

#include "quaternion.h"
#include "cubic_spline_planner.h"

class SplineOrientationPlanner
{
public:
	SplineOrientationPlanner();
	~SplineOrientationPlanner(void) {}
    void setQuaternions(const basic_alg::Quaternion &start, const basic_alg::Quaternion &via, const basic_alg::Quaternion &end);
	void planCurve(double u0, double u1, double v0, double v1, double t_total_former, double t_total_last);

    void sampleQuaternion(double t, basic_alg::Quaternion &sample);
	// void outputCurve(double time_step, const char *file_name);
	double getDuration(void);

private:
    enum {CUBIC_SPLINE_PLANNER_NUM = 2};

    double u0_;
    double u1_;
    double v0_;
    double v1_;
    double t_total_former_;
    double t_total_last_;
    double former_angle_ratio_;

    basic_alg::Quaternion start_quatern_;
    basic_alg::Quaternion via_quatern_;
    basic_alg::Quaternion end_quatern_;

    double orientation_angle_start2via_;
    double orientation_angle_via2end_;

    CubicSplinePlanner cubic_spline_former_;
    CubicSplinePlanner cubic_spline_last_;

    void sampleCurve(double t, double &u, double &v, double &a);
};

#endif
