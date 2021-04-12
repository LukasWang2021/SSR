#ifndef DSCURVE_PLANNER_H
#define DSCURVE_PLANNER_H
#include <stdint.h>

#define MAX_JERK_NUM 3

enum DSSetment
{
	INCREASE_ACCELERATION = 1,
	CONSTANT_ACCELERATION = 2,
	DECREASE_ACCELERATION = 3,
	CONSTANT_VELOCITY = 4,
	INCREASE_DECELERATION = 5,
	CONSTANT_DECELERATION = 6,
	DECREASE_DECELERATION = 7,
};

class DSCurvePlanner
{
public:
	virtual ~DSCurvePlanner(void) {}
	virtual void planDSCurve(double p0, double p1, double vmax, double amax, double* jmax, double vel_ratio) = 0;
	virtual void planStopDSCurve(double t) = 0;
	virtual void sampleDSCurve(double t, double &p, double &v, double &a) = 0;
	virtual void outputDSCurve(double time_step, const char *file_name) = 0;
	virtual double getDuration(void) = 0;
	virtual double getSegmentEndingTime(DSSetment segment) = 0;
};

#endif // !DSCurvePlanners