#ifndef CUBIC_SPLINE_PLANNER_H
#define CUBIC_SPLINE_PLANNER_H

class CubicSplinePlanner
{
public:
	CubicSplinePlanner();
	~CubicSplinePlanner(void) {}
	void planCurve(double u0, double u1, double v0, double v1, double t_total);
	void sampleCurve(double t, double &u);
    void sampleCurve(double t, double &u, double &v, double &a);
	void outputCurve(double time_step, const char *file_name);
	double getDuration(void);

private:
    enum {COFF_NUM = 4};

	double t_total_;
    double coefficients_[COFF_NUM];
};

#endif // !BezierPlanner

