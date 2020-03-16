#ifndef QUATERNION_SMOOTH_PLANNER_H
#define QUATERNION_SMOOTH_PLANNER_H

#include "cubic_spline_planner.h"
#include "basic_constants.h"
#include "quaternion.h"

class QuaternionSmoothPlanner
{
public:
	QuaternionSmoothPlanner();
	~QuaternionSmoothPlanner(void) {}
	void planCurve(const Quaternion &start, const Quaternion &via, double v0, double v1, double t_total_former, double t_total_last);
	void sampleCurve(double t, double &u);
	void outputCurve(double time_step, const char *file_name);
	double getDuration(void);

private:
    enum {CUBIC_SPLINE_PLANNER_NUM = 2};

    double u0_;
    double u1_;
    double t_total_former_;
    double t_total_last_;

    CubicSplinePlanner cubic_spline_former_;
    CubicSplinePlanner cubic_spline_last_;

    void sampleCurve(double t, double &u, double &v, double &a);

};

#endif 