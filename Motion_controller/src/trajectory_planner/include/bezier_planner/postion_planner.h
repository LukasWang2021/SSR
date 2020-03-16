#ifndef BEZIER_POSITION_PLANNER_H
#define BEZIER_POSITION_PLANNER_H

#include "point.h"

class BezierPositionPlanner
{
public:
	BezierPositionPlanner(uint32_t curve_power = 3);
	~BezierPositionPlanner(void) {}
	void planCurve(const basic_alg::Point &start, const basic_alg::Point &via, const basic_alg::Point &end, 
		double vstart, double vend, double amax);
	void sampleCurve(double t, basic_alg::Point &point);
	void outputCurve(double time_step, const char *file_name);
	double getDuration(void);
	double getMinCurvaTime(void);

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

	double t_total_;
	double vmax_;
	double vmin_;
	double amax_;
	double t_min_curva_;

	void buildTable(const double(&start)[3], const double(&via)[3], const double(&end)[3]);

	void getPoint(const double(&start)[3], const double(&via)[3], const double(&end)[3], double u, double(&point)[3]);
	void getCubicCurvePoint(const double(&start)[3], const double(&via)[3], const double(&end)[3], double u, double(&point)[3]);
	bool getCurvature(const double(&start)[3], const double(&via)[3], const double(&end)[3], double u, double &curvature);
	void getSpeedParameters(double vstart, double vend, double vmin, double p_vmin, double(&params)[4]);
	double getSpeed(double u, const double(&index)[4]);
};

#endif 

