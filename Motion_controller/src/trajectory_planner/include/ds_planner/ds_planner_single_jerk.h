
#ifndef SINGLE_JERK_DSCURVE_PLANNER_H
#define SINGLE_JERK_DSCURVE_PLANNER_H

#include "ds_planner/ds_planner.h"
#include "ds_planner/ds_planner_slow_down.h"

class SingleJerkDSCurvePlanner : public DSCurvePlanner
{
public: 
	SingleJerkDSCurvePlanner(void);
	~SingleJerkDSCurvePlanner(void);
	virtual void planStopDSCurve(double t);
	virtual void planDSCurve(double q0, double q1, double vmax, double amax, double* jmax, double v_ratio);
	virtual void sampleDSCurve(double t, double &p, double &v, double &a);
	virtual void outputDSCurve(double time_step, const char *file_name);
	virtual double getDuration(void);
	virtual double getSegmentEndingTime(DSSetment segment);

private: 
	double Ta_;
	double Tv_;
	double Td_;
	double Tj1_;
	double Tj2_;
	double q_0_;
	double q_1_;
	double v_0_;
	double v_1_;
	double vlim_;
	double a_max_;
	double a_min_;
	double a_lima_;
	double a_limd_;
	double j_max_;
	double j_min_;
	double t_total_;
	double vel_ratio_;
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
	void rescaleTrajectoryVelocity(double vel_ratio);
};



#endif