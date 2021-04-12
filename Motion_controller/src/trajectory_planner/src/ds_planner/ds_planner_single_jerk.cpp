#include <string>
#include <fstream>
#include <iostream>
#include "math.h"
#include "basic_alg.h"
#include "ds_planner/ds_planner_single_jerk.h"

using namespace std;
using namespace basic_alg;

SingleJerkDSCurvePlanner::SingleJerkDSCurvePlanner(void)
{
	Ta_ = 0.0;
	Tv_ = 0.0;
	Td_ = 0.0;
	Tj1_ = 0.0;
	Tj2_ = 0.0;
	q_0_ = 0.0;
	q_1_ = 0.0;
	v_0_ = 0.0;
	v_1_ = 0.0;
	vlim_ = 0.0;
	a_max_ = 0.0;
	a_min_ = 0.0;
	a_lima_ = 0.0;
	a_limd_ = 0.0;
	j_max_ = 0.0;
	j_min_ = 0.0;
	t_total_ = 0.0;

	p_stop_ = 0.0;
	v_stop_ = 0.0; 
	a_stop_ = 0.0;
	vel_ratio_ = 0.0;

	t_stop_ = 0.0;
	is_stop_success_ = false;
}


SingleJerkDSCurvePlanner::~SingleJerkDSCurvePlanner(void)
{

}
void SingleJerkDSCurvePlanner::planDSCurve(double q0, double q1, double vmax, double amax, double* jmax, double v_ratio)
{
	is_stop_success_ = false;
	/* Ta : acceleration period
	   Tv : constant velocity period
	   Td : deceleration period
	   Tj : time - interval in which the jerk is constant during the acceleration phase or the deceleration phase
	   T = (Ta + Tv + Td) : total duration of the trajectory
	   alima : limit maximum acceleration
	   alimd : limit minmum acceleration */

	/* tarnsform initial values */

	if (jmax != NULL)
	{
		j_max_ = *jmax;
	}
	else
	{
		j_max_ = amax * 50;
	}

	j_min_ = -j_max_;
	double vmin = -vmax;
	double amin = -amax;

	int sign = q1 >= q0 ? 1 : -1;
	double q_0 = sign*q0;
	double q_1 = sign*q1;
	double v_max = ((sign + 1) / 2)*vmax + ((sign - 1) / 2)*vmin;
	//double v_min = ((sign + 1) / 2)*vmin + ((sign - 1) / 2)*vmax;
	double a_max = ((sign + 1) / 2)*amax + ((sign - 1) / 2)*amin;
	double a_min = ((sign + 1) / 2)*amin + ((sign - 1) / 2)*amax;
	double j_max = ((sign + 1) / 2)*j_max_ + ((sign - 1) / 2)*j_min_;
	double j_min = ((sign + 1) / 2)*j_min_ + ((sign - 1) / 2)*j_max_;

	double Ta, Tv,  Tj;
	double vlim, a_lima, a_limd;

	 /*  case1 */
	if (pow(a_max, 2) <= v_max * j_max)
	{
		Tj = a_max / j_max;
		Ta = Tj + v_max / a_max;
	}
	else
	{
		Tj = sqrt(fabs(v_max / j_max));
		Ta = 2 * Tj;
	}

	Tv = (q_1 - q_0) / v_max - Ta;

	/*   case 2 */
	if (Tv <= 0)
	{
		if (2 * pow(a_max, 3) / pow(j_max, 2) <= (q_1 - q_0))
		{
			Tj = a_max / j_max;
			Ta = Tj / 2 + sqrt(fabs(pow(Tj / 2, 2) + (q_1 - q_0) / a_max));
		}
		else
		{
			Tj = pow((q_1 - q_0) / (2 * j_max), 1.0 / 3.0);
			Ta = 2 * Tj;
		}

		Tv = 0;
	}

	a_lima = j_max * Tj;
	a_limd = -a_lima;
	vlim = (Ta - Tj) * a_lima;

	Ta_ = Ta;
	Tv_ = Tv;
	Td_ = Ta;
	Tj1_ = Tj;
	Tj2_ = Tj;
	q_0_ = sign * q_0;
	q_1_ = sign * q_1;
	v_0_ = 0.0;
	v_1_ = 0.0;
	vlim_ = sign * vlim;
	a_max_ = sign * a_max;
	a_min_ = sign * a_min;
	a_lima_ = sign * a_lima;
	a_limd_ = sign * a_limd;
	j_max_ = sign * j_max;
	j_min_ = sign * j_min;
	t_total_ = Ta_ + Tv_ + Td_;

	rescaleTrajectoryVelocity(v_ratio);
}

void SingleJerkDSCurvePlanner::rescaleTrajectoryVelocity(double vel_ratio)
{
	// advance acc and dec
	double p0, v0, a0, p1, v1, a1, t_delta;
	sampleOriginDSCurve(0, p0, v0, a0);
	sampleOriginDSCurve(Tj1_, p1, v1, a1);
	v0 *= vel_ratio;
	v1 *= vel_ratio;
	a0 = a0 * vel_ratio * vel_ratio;
	a1 = a1 *  vel_ratio * vel_ratio;
	t_delta = Tj1_ / vel_ratio;
	getSepticSpline(p0, v0, a0, 0, p1, v1, a1, 0, t_delta, coeff_inc_acc_);
	
	sampleOriginDSCurve(Ta_ - Tj1_, p0, v0, a0);
	sampleOriginDSCurve(Ta_, p1, v1, a1);
	v0 *= vel_ratio;
	v1 *= vel_ratio;
	a0 = a0 * vel_ratio * vel_ratio;
	a1 = a1 *  vel_ratio * vel_ratio;
	getSepticSpline(p0, v0, a0, 0, p1, v1, a1, 0, t_delta, coeff_dec_acc_);
	
	sampleOriginDSCurve(Ta_ + Tv_, p0, v0, a0);
	sampleOriginDSCurve(Ta_ + Tv_ + Tj2_, p1, v1, a1);
	v0 *= vel_ratio;
	v1 *= vel_ratio;
	a0 = a0 * vel_ratio * vel_ratio;
	a1 = a1 *  vel_ratio * vel_ratio;
	t_delta = Tj2_ / vel_ratio;
	getSepticSpline(p0, v0, a0, 0, p1, v1, a1, 0, t_delta, coeff_inc_dec_);
	
	sampleOriginDSCurve(t_total_ - Tj2_, p0, v0, a0);
	sampleOriginDSCurve(t_total_, p1, v1, a1);
	v0 *= vel_ratio;
	v1 *= vel_ratio;
	a0 = a0 * vel_ratio * vel_ratio;
	a1 = a1 *  vel_ratio * vel_ratio;
	getSepticSpline(p0, v0, a0, 0, p1, v1, a1, 0, t_delta, coeff_dec_dec_);

	Ta_ /= vel_ratio;
	Tv_ /= vel_ratio;
	Td_ /= vel_ratio;
	Tj1_ /= vel_ratio;
	Tj2_ /= vel_ratio;
	t_total_ /= vel_ratio;

	vel_ratio_ = vel_ratio;
}

void SingleJerkDSCurvePlanner::planStopDSCurve(double t)
{
	if (is_stop_success_ || t_total_ < t)
	{
		return;
	}

	t_stop_ = t;
	sampleFineDSCurve(t_stop_, p_stop_, v_stop_, a_stop_);

	if (slow_down_.planDSCurve(p_stop_, v_stop_, a_stop_, a_max_, j_max_))
	{
		double stop_duration = slow_down_.getDuration();

		if (t + stop_duration < t_total_)
		{
			t_total_ = t + stop_duration;
			is_stop_success_ = true;
		}
	}
}

void SingleJerkDSCurvePlanner::sampleDSCurve(double t, double &p, double &v, double &a)
{
	if (!is_stop_success_ || t < t_stop_)
	{
		sampleFineDSCurve(t, p, v, a);
	}
	else
	{
		double time = t - t_stop_;
		slow_down_.sampleDSCurve(time, p, v, a);
	}
}

void SingleJerkDSCurvePlanner::sampleOriginDSCurve(double t, double &p, double &v, double &a)
{
	if (t < 0)
    {
        p = q_0_;
        v = v_0_;
        a = 0;
    }
    else if (t < Tj1_)
    {
        p = q_0_ + v_0_ * t + j_max_ * pow(t, 3) / 6;
        v = v_0_ + j_max_ * pow(t, 2) / 2;
        a = j_max_ * t;
    }
    else if (t < Ta_ - Tj1_)
    {
        p = q_0_ + v_0_ * t + (a_lima_ / 6) * (3 * pow(t, 2) - 3 * Tj1_ * t + pow(Tj1_, 2));
        v = v_0_ + a_lima_ * (t - Tj1_ / 2);
        a = a_lima_;
    }
    else if (t < Ta_)
    {
        p = q_0_ + (vlim_ + v_0_)*(Ta_ / 2) - vlim_ * (Ta_ - t) - j_min_ * (pow(Ta_ - t, 3) / 6);
        v = vlim_ + j_min_ * (pow(Ta_ - t, 2) / 2);
        a = -j_min_ * (Ta_ - t);
    }
    else if (t < Ta_ + Tv_)
    {
        p = q_0_ + (vlim_ + v_0_) * (Ta_ / 2) + vlim_ * (t - Ta_);
        v = vlim_;
        a = 0;
    }
    else if (t < t_total_ - Td_ + Tj2_)
    {
        p = q_1_ - (vlim_ + v_1_) * (Td_ / 2) + vlim_ * (t - t_total_ + Td_) - j_max_ * (pow(t - t_total_ + Td_, 3) / 6);
        v = vlim_ - j_max_ * (pow(t - t_total_ + Td_, 2) / 2);
        a = -j_max_*(t - t_total_ + Td_);
    }
    else if (t < t_total_ - Tj2_)
    {
        p = q_1_ - (vlim_ + v_1_) * (Td_ / 2) + vlim_ * (t - t_total_ + Td_) + (a_limd_ / 6) * (3 * pow(t - t_total_ + Td_, 2) - 3 * Tj2_ * (t - t_total_ + Td_) + pow(Tj2_, 2));
        v = vlim_ + a_limd_ * (t - t_total_ + Td_ - Tj2_ / 2);
        a = a_limd_;
    }
    else if (t < t_total_)
    {
        p = q_1_ - v_1_ * (t_total_ - t) - j_max_ * (pow(t_total_ - t, 3) / 6);
        v = v_1_ + j_max_ * (pow(t_total_ - t, 2) / 2);
        a = -j_max_ * (t_total_ - t);
    }
    else
    {
        p = q_1_;
        v = v_1_;
        a = 0;
    }
}

void SingleJerkDSCurvePlanner::sampleFineDSCurve(double t, double &p, double &v, double &a)
{
    if (t < 0)
    {
        p = q_0_;
        v = v_0_;
        a = 0;
    }
    else if (t < Tj1_)
    {
        //p = q_0_ + v_0_ * t + j_max_ * pow(t, 3) / 6;
        //v = v_0_ + j_max_ * pow(t, 2) / 2;
        //a = j_max_ * t;
		double j;
		sampleSepticSpline(t, coeff_inc_acc_, p, v, a, j);
    }
    else if (t < Ta_ - Tj1_)
    {
		double t_temp = t * vel_ratio_;
		double Tj1_temp = Tj1_ * vel_ratio_;
        p = q_0_ + v_0_ * t_temp + (a_lima_ / 6) * (3 * pow(t_temp, 2) - 3 * Tj1_temp * t_temp + pow(Tj1_temp, 2));
        v = v_0_ + a_lima_ * (t_temp - Tj1_temp / 2);
        a = a_lima_;

		v *= vel_ratio_;
		a = a * vel_ratio_ * vel_ratio_;
    }
    else if (t < Ta_)
    {
        //p = q_0_ + (vlim_ + v_0_)*(Ta_ / 2) - vlim_ * (Ta_ - t) - j_min_ * (pow(Ta_ - t, 3) / 6);
        //v = vlim_ + j_min_ * (pow(Ta_ - t, 2) / 2);
        //a = -j_min_ * (Ta_ - t);
		double tm = t - Ta_ + Tj1_;
		double j;
		sampleSepticSpline(tm, coeff_dec_acc_, p, v, a, j);
    }
    else if (t < Ta_ + Tv_)
    {
		double t_temp = t * vel_ratio_;
		double Ta_temp = Ta_ * vel_ratio_;

        p = q_0_ + (vlim_ + v_0_) * (Ta_temp / 2) + vlim_ * (t_temp - Ta_temp);
        v = vlim_;
        a = 0;

		v *= vel_ratio_;
		a = a * vel_ratio_ * vel_ratio_;
    }
    else if (t < t_total_ - Td_ + Tj2_)
    {
        //p = q_1_ - (vlim_ + v_1_) * (Td_ / 2) + vlim_ * (t - t_total_ + Td_) - j_max_ * (pow(t - t_total_ + Td_, 3) / 6);
        //v = vlim_ - j_max_ * (pow(t - t_total_ + Td_, 2) / 2);
        //a = -j_max_*(t - t_total_ + Td_);
		double tm = t - Ta_ - Tv_;
		double j;
		sampleSepticSpline(tm, coeff_inc_dec_, p, v, a, j);
    }
    else if (t < t_total_ - Tj2_)
    {
		double t_temp = t * vel_ratio_;
		double Td_temp = Td_ * vel_ratio_;
		double Tj2_temp = Tj2_ * vel_ratio_;
		double t_total_temp = t_total_ * vel_ratio_;

        p = q_1_ - (vlim_ + v_1_) * (Td_temp / 2) + vlim_ * (t_temp - t_total_temp + Td_temp) + 
			(a_limd_ / 6) * (3 * pow(t_temp - t_total_temp + Td_temp, 2) - 3 * Tj2_temp * (t_temp - t_total_temp + Td_temp) + pow(Tj2_temp, 2));
        v = vlim_ + a_limd_ * (t_temp - t_total_temp + Td_temp - Tj2_temp / 2);
        a = a_limd_;

		v *= vel_ratio_;
		a = a * vel_ratio_ * vel_ratio_;
    }
    else if (t < t_total_)
    {
        //p = q_1_ - v_1_ * (t_total_ - t) - j_max_ * (pow(t_total_ - t, 3) / 6);
        //v = v_1_ + j_max_ * (pow(t_total_ - t, 2) / 2);
        //a = -j_max_ * (t_total_ - t);
		double tm = t - t_total_ + Tj2_;
		double j;
		sampleSepticSpline(tm, coeff_dec_dec_, p, v, a, j);
    }
    else
    {
        p = q_1_;
        v = v_1_;
        a = 0;
    }
}




void SingleJerkDSCurvePlanner::outputDSCurve(double time_step, const char *file_name)
{
	std::string out_file;
	double ps, vs, as;
	double total_time = t_total_ + time_step;

	if (file_name != NULL)
	{
		out_file = file_name;
	}
	else
	{
		out_file = "ds_curve.csv";
	}

	std::ofstream out(out_file);

	for (double t = 0; t < total_time; t += time_step)
	{
		sampleDSCurve(t, ps, vs, as);
		out << t << "," << ps << "," << vs << "," << as << std::endl;
	}

	out.close();
}

double SingleJerkDSCurvePlanner::getDuration(void)
{
	return t_total_;
}

double SingleJerkDSCurvePlanner::getSegmentEndingTime(DSSetment segment)
{
	double time_array[7] = {Tj1_, Ta_ - Tj1_, Ta_, Ta_ + Tv_, Ta_ + Tv_ + Tj2_, Ta_ + Tv_ + Td_ - Tj2_, Ta_ + Tv_ + Td_};
	return time_array[static_cast<uint32_t>(segment) - 1];
}
