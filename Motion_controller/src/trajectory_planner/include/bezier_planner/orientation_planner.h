#ifndef BEZIER_ORIENTATION_PLANNER_H
#define BEZIER_ORIENTATION_PLANNER_H

#include "quaternion.h"

class BezierOrientationPlanner
{
public:
	BezierOrientationPlanner(uint32_t curve_power = 3);
	~BezierOrientationPlanner(void) {}
	void planCurve(const basic_alg::Quaternion &start, const basic_alg::Quaternion &via, const basic_alg::Quaternion &end);
    void getMidQuaternion(basic_alg::Quaternion &mid_quatern);

	// void outputCurve(const char *file_name);

private:
	uint32_t curve_power_;
	enum { DIV_PARTS  = 80}; 
	enum { QUADRATIC = 2, CUBIC = 3 };

	double x_normalize_[DIV_PARTS + 1];
	double y_normalize_[DIV_PARTS + 1];
	double s_normalize_[DIV_PARTS + 1];

	basic_alg::Quaternion start_quatern_;
	basic_alg::Quaternion via_quatern_;
	basic_alg::Quaternion end_quatern_;
	basic_alg::Quaternion mid_quatern_;

    double orientation_angle_;
	double orientation_angle_start2via_;
	double orientation_angle_via2end_;

	void buildTable(const double(&start)[2], const double(&via)[2], const double(&end)[2]);
	void getPoint(const double(&start)[2], const double(&via)[2], const double(&end)[2], double u, double(&point)[2]);
	void getCubicCurvePoint(const double(&start)[2], const double(&via)[2], const double(&end)[2], double u, double(&point)[2]);
	void computeMidQuaternion();

};

#endif 

