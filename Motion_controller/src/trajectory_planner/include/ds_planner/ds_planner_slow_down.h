#ifndef SLOW_DOWN_DSCURVE_PLANNER_H
#define SLOW_DOWN_DSCURVE_PLANNER_H


class SlowDownDSCurvePlanner
{
public:
	SlowDownDSCurvePlanner(void);
	~SlowDownDSCurvePlanner(void);
	bool planDSCurve(double q0, double v0, double a0, double amax, double jmax);
	void sampleDSCurve(double t, double &p, double &v, double &a);
	void outputDSCurve(double time_step, const char *file_name);
	double getDuration(void);

private:
	enum { SEGMENT_NUM = 5 };
	double t_[SEGMENT_NUM];
	double s_[SEGMENT_NUM];
	double v_[SEGMENT_NUM];
	double a_[SEGMENT_NUM];

	double q0_;
	double v0_;
	double a0_;
	double amax_;	
	double jmax_;
	double jmax_t4_;	

	double t_total_;
	double s_total_;
};




#endif