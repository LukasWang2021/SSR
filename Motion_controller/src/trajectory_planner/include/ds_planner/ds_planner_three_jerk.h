
#ifndef THREE_JERK_DSCURVE_PLANNER_H
#define THREE_JERK_DSCURVE_PLANNER_H

// #include "ds_planner/ds_planner.h"
#include "ds_planner/ds_planner.h"
#include "ds_planner/ds_planner_slow_down.h"

class ThreeJerkDSCurvePlanner : public DSCurvePlanner
{
public:
	ThreeJerkDSCurvePlanner(void);
	~ThreeJerkDSCurvePlanner(void);
	virtual void planStopDSCurve(double t);
	virtual void planDSCurve(double q0, double q1, double vmax, double amax, double* jmax, double v_ratio);
	virtual void sampleDSCurve(double t, double &p, double &v, double &a);
	virtual void outputDSCurve(double time_step, const char *file_name);
	virtual double getDuration(void);
	virtual double getSegmentEndingTime(DSSetment segment);
private:
	enum { JERK_NUM = 3, SEGMENT_NUM = 7 };
	double t_[SEGMENT_NUM];
	double v_[SEGMENT_NUM];
	double s_[SEGMENT_NUM];
	double a_[SEGMENT_NUM];
	double amax_, vmax_;
	double q0_, q1_;
	double j1_, j2_, j3_;
	double t_total_;
	double coeff_inc_acc_[8];
	double coeff_dec_acc_[8];
	double coeff_inc_dec_[8];
	double coeff_dec_dec_[8];

	SlowDownDSCurvePlanner slow_down_;

	double t_stop_;
	double p_stop_;
	double v_stop_;
	double a_stop_;
	bool is_stop_success_;
	double vel_ratio_;

	void sampleFineDSCurve(double t, double &p, double &v, double &a);
	void sampleOriginDSCurve(double t, double &p, double &v, double &a);
	void rescaleTrajectoryVelocity(double vel_ratio, double t1, double t2, double t3, double t4, double t5, double t6, double t7);
};



#endif