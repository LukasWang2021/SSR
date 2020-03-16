#ifndef TWO_JERK_DSCURVE_PLANNER_H
#define TWO_JERK_DSCURVE_PLANNER_H

#include "ds_planner/ds_planner.h"
#include "ds_planner/ds_planner_slow_down.h"

class TwoJerkDSCurvePlanner : public DSCurvePlanner
{
public:
	TwoJerkDSCurvePlanner(void);
	~TwoJerkDSCurvePlanner(void);
	virtual void planStopDSCurve(double t);
	virtual void planDSCurve(double q0, double q1, double vmax, double amax, double* jmax);
	virtual void sampleDSCurve(double t, double &p, double &v, double &a);
	virtual void outputDSCurve(double time_step, const char *file_name);
	virtual double getDuration(void);
	virtual double getSegmentEndingTime(DSSetment segment);

private:
	enum {JERK_NUM = 2, SEGMENT_NUM = 7};
	double t_[SEGMENT_NUM];
	double v_[SEGMENT_NUM];
	double s_[SEGMENT_NUM];
	double a_[SEGMENT_NUM];
	double amax_, vmax_;
	double j1_, j2_;
	double q0_, q1_;
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

	void sampleFineDSCurve(double t, double &p, double &v, double &a);
	void sampleOriginDSCurve(double t, double &p, double &v, double &a);
};

#endif