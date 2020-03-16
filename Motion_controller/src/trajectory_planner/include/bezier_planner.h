#ifndef BEZIER_PLANNER_H
#define BEZIER_PLANNER_H

#include "pose_quaternion.h"
#include "quaternion.h"

#include "spline_planner/cubic_spline_planner.h"
#include "spline_planner/cubic_spline_smooth.h"

class BezierPlanner
{
public:
	BezierPlanner(uint32_t curve_power = 3);
	~BezierPlanner(void) {}
	void planCurve(const basic_alg::PoseQuaternion &start, const basic_alg::PoseQuaternion &via, const basic_alg::PoseQuaternion &end, 
		double vstart_p, double vend_p, double amax_p, double ustart, double uend, double vstart_u,double vend_u);
	void sampleCurve(double t, basic_alg::PoseQuaternion &point);
	void outputCurve(double time_step, const char *file_name);
	double getDuration(void);

private:
	uint32_t curve_power_;
	enum { DIV_PARTS  = 80}; 
	enum { QUADRATIC = 2, CUBIC = 3 };

	double s_[DIV_PARTS + 1];
	double x_[DIV_PARTS + 1];
	double y_[DIV_PARTS + 1];
	double z_[DIV_PARTS + 1];

	double t_[DIV_PARTS + 1];
	double rr_[DIV_PARTS + 1];

	double orientation_smooth_former_time_;
	double orientation_smooth_former_angle_;
	double orientation_smooth_last_time_;
	double orientation_smooth_last_angle_;
	double orientation_angle_;

	double t_total_;
	double vmax_;
	double vmin_;
	double amax_;

	double ustart_;
	double uend_;
	basic_alg::Quaternion start_quatern_;
	basic_alg::Quaternion end_quatern_;
	basic_alg::Quaternion via_quatern_;

	CubicSplineSmooth cubic_spline_smooth_;

	void buildTable(const double(&start)[3], const double(&via)[3], const double(&end)[3]);

	void getPoint(const double(&start)[3], const double(&via)[3], const double(&end)[3], double u, double(&point)[3]);
	void getCubicCurvePoint(const double(&start)[3], const double(&via)[3], const double(&end)[3], double u, double(&point)[3]);
	bool getCurvature(const double(&start)[3], const double(&via)[3], const double(&end)[3], double u, double &curvature);
	void getSpeedParameters(double vstart, double vend, double vmin, double p_vmin, double(&params)[4]);
	double getSpeed(double u, const double(&index)[4]);
};

#endif // !BezierPlanner

