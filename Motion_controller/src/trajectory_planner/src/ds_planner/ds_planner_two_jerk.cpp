#include <string>
#include <fstream>
#include <math.h>
#include <string.h>
#include <iostream>

#include "basic_alg.h"

#include "ds_planner/ds_planner_two_jerk.h"

using namespace std;
using namespace basic_alg;

TwoJerkDSCurvePlanner::TwoJerkDSCurvePlanner(void)
{
	memset(t_, 0, sizeof(double) * SEGMENT_NUM);
	memset(v_, 0, sizeof(double) * SEGMENT_NUM);
	memset(s_, 0, sizeof(double) * SEGMENT_NUM);
	memset(a_, 0, sizeof(double) * SEGMENT_NUM);

	amax_ = 0.0;
	vmax_ = 0.0;
	j1_ = 0.0;
	j2_ = 0.0;
	t_total_ = 0.0;
	p_stop_ = 0.0;
	v_stop_ = 0.0;
	a_stop_ = 0.0;

	t_stop_ = 0.0;
	is_stop_success_ = false;
}

TwoJerkDSCurvePlanner::~TwoJerkDSCurvePlanner(void)
{

}

void TwoJerkDSCurvePlanner::planDSCurve(double q0, double q1, double vmax, double amax, double* jmax)
{
	is_stop_success_ = false;
	q0_ = q0;
	q1_ = q1;
	amax_ = amax;
	vmax_ = vmax;

	if (jmax == NULL)
	{
		j1_ = 100.0;
		j2_ = 100.0;
	}
	else
	{
		j1_ = jmax[0];
		j2_ = jmax[1];
	}

	double s = q1 - q0;
	double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0, t7 = 0;
	double v1 = 0, v2 = 0, v3 = 0, v4 = 0, v5 = 0, v6 = 0, v7 = 0;
	double a1, a6;
	double sa, sb;

	/* assume we are not limited by a_max assume the max v we can achieve */
	t1 = sqrt(vmax / j1_);

	/* now we have a1 */
	a1 = t1 * j1_;
	t7 = sqrt(vmax / j2_);
	a6 = -j2_ * t7;

	sa = j1_ * pow(t1, 3);
	sb = j2_ * pow(t7, 3);

	/* check if we really less than the acc limit */
	if (a1 < amax && fabs(a6)<amax)
	{
		/* no acc limit */
		if (sa + sb > s) {
			/* no, we can not reach vmax try to lower vmax */
			while (sa + sb > s) {
				vmax = vmax * 0.95;

				t1 = sqrt(vmax / j1_);
				a1 = t1 * j1_;
				t7 = sqrt(vmax / j2_);
				a6 = -j2_ * t7;
				sa = j1_ * pow(t1, 3);
				sb = j2_ * pow(t7, 3);
			}
		}

		if (sa + sb < s) {
			/* good , we have section4 */
			t4 = (s - sa - sb) / vmax;
			t2 = 0;
			t6 = 0;

			v1 = vmax / 2;
			v2 = v1;
			v3 = vmax;
			v4 = vmax;
			v5 = vmax / 2;
			v6 = v5;
			v7 = 0;
		}
	}
	else
	{
		/* we limited by the amaxit assume we can reach the vmax */
		t1 = amax / j1_;
		t7 = amax / j2_;

		// v3 = vmax = j1_*t1*t1 + amax *t2
		t2 = (vmax - j1_ * pow(t1, 2)) / amax;
		if (t2 > 0)
		{
			sa = vmax*(2 * t1 + t2) / 2.0;
		}
		else 
		{
			/* we are limited by speed */
			t1 = sqrt(vmax / j1_);
			t2 = 0;
			sa = j1_ * pow(t1, 3);
		}

		t6 = (vmax - j2_ * pow(t7, 2)) / amax;
		if (t6 > 0) 
		{
			sb = vmax*(2 * t7 + t6) / 2.0;
		}
		else
		{
			/* we are limited by speed */
			t7 = sqrt(vmax / j2_);
			sb = j2_ * pow(t7, 3);
			t6 = 0;
		}

		if (sa + sb <s) 
		{
			/* good, we have section 4 */
			t4 = (s - sa - sb) / vmax;

			v1 = 0.5 * j1_ * pow(t1, 2);
			if (t2 == 0.0)
			{
				v2 = v1;
				v3 = 2 * v1;
			}
			else 
			{
				v2 = v1 + amax *t2;
				v3 = vmax;
			}
			v4 = v3;

			/* from back */
			v6 = 0.5 * j2_ * pow(t7, 2);
			v5 = v4 - v6;
			v7 = 0;
		}
		else
		{
			/*we can not reach the vmax, lower vmax to try. */
			while (sa + sb > s) 
			{
				vmax = vmax * 0.95;
				/* assume we can reach the lowered vmax */
				// v3 = vmax = j1_*t1*t1 + amax *t2
				t2 = (vmax - j1_* pow(t1, 2)) / amax;

				if (t2 > 0)
				{
					sa = vmax*(2 * t1 + t2) / 2.0;
				}
				else 
				{
					/* we are limited by speed */
					t1 = sqrt(vmax / j1_);
					t2 = 0;
					sa = j1_ * pow(t1, 3);
				}

				t6 = (vmax - j2_ * pow(t7, 2)) / amax;
				if (t6 > 0) 
				{
					sb = vmax*(2 * t7 + t6) / 2.0;
				}
				else 
				{
					/* we are limited by speed */
					t7 = sqrt(vmax / j2_);
					sb = j2_ * pow(t7, 3);
					t6 = 0;
				}
			}
			/* now we got the result */
			t4 = (s - sa - sb) / vmax;

			v1 = 0.5 * j1_ * pow(t1, 2);
			v2 = v1 + amax *t2;
			v3 = vmax;
			v4 = v3;

			/* from back */
			v6 = 0.5 * j2_*t7*t7;
			v5 = v4 - v6;
			v7 = 0;
		}
	}

	t_total_ = t1 * 2 + t2 + t4 + t6 + t7 * 2;

	t3 = t1;
	t5 = t7;

	t_total_ = t_total_;

	t_[0] = t1;
	t_[1] = t2;
	t_[2] = t1;
	t_[3] = t4;
	t_[4] = t7;
	t_[5] = t6;
	t_[6] = t7;

	v_[0] = v1;
	v_[1] = v2;
	v_[2] = v3;
	v_[3] = v4;
	v_[4] = v5;
	v_[5] = v6;
	v_[6] = v7;
	a_[0] = j1_ * t1;

	if (t2 > 0)
		a_[1] = amax;
	else 
		a_[1] = a_[0];

	a_[2] = a_[1] - j1_ * t1;
	a_[3] = 0;
	a_[4] = -j2_ * t7;
	if (t6 > 0)
		a_[5] = -amax;
	else 
		a_[5] = a_[4];
	a_[6] = 0;

	s_[0] = 1.0 / 6.0 * j1_ * pow(t1, 3);

	if (t2 > 0)
		s_[1] = v1 * t2 + 0.5 * amax *pow(t2, 2);
	else 
		s_[1] = 0;

	s_[2] = v2 * t3 - 1.0 / 6.0 * j1_ * pow(t3, 3) + 0.5 * a_[1] * pow(t3, 2);
	s_[3] = t4 * v3;
	s_[4] = v4 * t5 - 1.0 / 6.0 * j2_ * pow(t5, 3);

	if (t6 > 0)
		s_[5] = v5 * t6 - 0.5 * amax * pow(t6, 2);
	else 
		s_[5] = 0;
	s_[6] = 1.0 / 6.0 * j2_ * pow(t7, 3);

	// advance acc and dec
	double ap0, av0, aa0, ap1, av1, aa1;
	sampleOriginDSCurve(0, ap0, av0, aa0);
	sampleOriginDSCurve(t1, ap1, av1, aa1);
	getSepticSpline(ap0, av0, aa0, 0, ap1, av1, aa1, 0, t1, coeff_inc_acc_);
	
	sampleOriginDSCurve(t1 + t2, ap0, av0, aa0);
	sampleOriginDSCurve(t1 + t2 + t3, ap1, av1, aa1);
	getSepticSpline(ap0, av0, aa0, 0, ap1, av1, aa1, 0, t3, coeff_dec_acc_);
	
	sampleOriginDSCurve(t1 + t2 + t3 + t4, ap0, av0, aa0);
	sampleOriginDSCurve(t1 + t2 + t3 + t4 + t5, ap1, av1, aa1);
	getSepticSpline(ap0, av0, aa0, 0, ap1, av1, aa1, 0, t5, coeff_inc_dec_);
	
	sampleOriginDSCurve(t1 + t2 + t3 + t4 + t5 + t6, ap0, av0, aa0);
	sampleOriginDSCurve(t_total_, ap1, av1, aa1);
	getSepticSpline(ap0, av0, aa0, 0, ap1, av1, aa1, 0, t7, coeff_dec_dec_);
}


void TwoJerkDSCurvePlanner::sampleDSCurve(double t, double &p, double &v, double &a)
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

void TwoJerkDSCurvePlanner::sampleFineDSCurve(double t, double &p, double &v, double &a)
{
	if (t < 1E-10)
	{
		p = q0_;
		v = 0.0; 
		a = 0.0;

		return;
	}

	double t_total = 0.0, t_base = 0.0, t_ptr, s_total = 0.0;

	int i;
	for (i = 0; i < SEGMENT_NUM; i++)
	{
		t_total += t_[i];
		if (t_total >= t) break;
		s_total += s_[i];
		t_base += t_[i];
	}

	t_ptr = t - t_base;

	/*check the section we stop at. */
	switch (i)
	{
	case 0:
	{
		//s_total += (1.0 / 6.0 * j1_ * pow(t_ptr, 3));
		//a = j1_ * t_ptr;
		//v = 0.5 * j1_ * t_ptr *t_ptr;
		double j;
		sampleSepticSpline(t_ptr, coeff_inc_acc_, p, v, a, j);
	}
	break;
	case 1:
	{
		s_total += (v_[0] * t_ptr + 0.5 * amax_ * pow(t_ptr, 2));
		a = a_[1];
		v = v_[0] + a*t_ptr;
		p = q0_ + s_total;
	}
	break;
	case 2:
	{
		//s_total += (v_[1] * t_ptr - 1.0 / 6.0 * j1_ * pow(t_ptr, 3) 
		//	+ 0.5* a_[1] * pow(t_ptr, 2));
		//a = a_[1] - j1_ * t_ptr;
		//v = v_[1] + a_[1] * t_ptr - 0.5* j1_ * pow(t_ptr, 2);
		double j;
		sampleSepticSpline(t_ptr, coeff_dec_acc_, p, v, a, j);
	}
	break;
	case 3:
	{
		s_total += (v_[2] * t_ptr);
		a = 0;
		v = v_[2];
		p = q0_ + s_total;
	}
	break;
	case 4:
	{
		//s_total += (v_[3] * t_ptr - 1.0 / 6.0 * j2_ * pow(t_ptr, 3));
		//a = -j2_ * t_ptr;
		//v = v_[3] - 0.5*  j2_ * pow(t_ptr, 2);
		double j;
		sampleSepticSpline(t_ptr, coeff_inc_dec_, p, v, a, j);
	}
	break;
	case 5:
	{
		s_total += (v_[4] * t_ptr - 0.5 * amax_ * pow(t_ptr, 2));
		a = a_[4];
		v = v_[4] + a * t_ptr;
		p = q0_ + s_total;
	}
	break;
	case 6:
	{
		//s_total += (v_[5] * t_ptr + 1.0 / 6.0 * j2_ * pow(t_ptr, 3) 
		//	- 0.5* t_[6] * j2_ * pow(t_ptr, 2));
		//a = -(t_[6] - t_ptr)* j2_;
		//v = v_[5] + 0.5 * j2_ * pow(t_ptr, 2) - t_[6] * j2_ * t_ptr;
		double j;
		sampleSepticSpline(t_ptr, coeff_dec_dec_, p, v, a, j);
	}
	break;
	case 7:
	{
		s_total = q1_;
		a = 0;
		v = 0;
		p = q0_ + s_total;
	}
	break;
	default:;
	}
}


void TwoJerkDSCurvePlanner::sampleOriginDSCurve(double t, double &p, double &v, double &a)
{
	if (t < 1E-10)
	{
		p = q0_;
		v = 0.0; 
		a = 0.0;

		return;
	}

	double t_total = 0.0, t_base = 0.0, t_ptr, s_total = 0.0;

	int i;
	for (i = 0; i < SEGMENT_NUM; i++)
	{
		t_total += t_[i];
		if (t_total >= t) break;
		s_total += s_[i];
		t_base += t_[i];
	}

	t_ptr = t - t_base;

	/*check the section we stop at. */
	switch (i)
	{
	case 0:
	{
		s_total += (1.0 / 6.0 * j1_ * pow(t_ptr, 3));
		a = j1_ * t_ptr;
		v = 0.5 * j1_ * t_ptr *t_ptr;
	}
	break;
	case 1:
	{
		s_total += (v_[0] * t_ptr + 0.5 * amax_ * pow(t_ptr, 2));
		a = a_[1];
		v = v_[0] + a*t_ptr;
	}
	break;
	case 2:
	{
		s_total += (v_[1] * t_ptr - 1.0 / 6.0 * j1_ * pow(t_ptr, 3) 
			+ 0.5* a_[1] * pow(t_ptr, 2));
		a = a_[1] - j1_ * t_ptr;
		v = v_[1] + a_[1] * t_ptr - 0.5* j1_ * pow(t_ptr, 2);
	}
	break;
	case 3:
	{
		s_total += (v_[2] * t_ptr);
		a = 0;
		v = v_[2];
	}
	break;
	case 4:
	{
		s_total += (v_[3] * t_ptr - 1.0 / 6.0 * j2_ * pow(t_ptr, 3));
		a = -j2_ * t_ptr;
		v = v_[3] - 0.5*  j2_ * pow(t_ptr, 2);
	}
	break;
	case 5:
	{
		s_total += (v_[4] * t_ptr - 0.5 * amax_ * pow(t_ptr, 2));
		a = a_[4];
		v = v_[4] + a * t_ptr;
	}
	break;
	case 6:
	{
		s_total += (v_[5] * t_ptr + 1.0 / 6.0 * j2_ * pow(t_ptr, 3) 
			- 0.5* t_[6] * j2_ * pow(t_ptr, 2));
		a = -(t_[6] - t_ptr)* j2_;
		v = v_[5] + 0.5 * j2_ * pow(t_ptr, 2) - t_[6] * j2_ * t_ptr;
	}
	break;
	case 7:
	{
		s_total = q1_;
		a = 0;
		v = 0;
	}
	break;
	default:;
	}

	p = q0_ + s_total;
}

void TwoJerkDSCurvePlanner::outputDSCurve(double time_step, const char *file_name)
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

double TwoJerkDSCurvePlanner::getDuration(void)
{
	return t_total_;
}

double TwoJerkDSCurvePlanner::getSegmentEndingTime(DSSetment segment)
{
	double time_stamp = 0.0;
	uint32_t seg_num = static_cast<uint32_t>(segment);

	for (uint32_t i = 0; i < seg_num; i++)
	{
		time_stamp += t_[i];
	}

	return time_stamp;
}

void TwoJerkDSCurvePlanner::planStopDSCurve(double t)
{
	if (t_total_ < t)
	{
		return;
	}

	t_stop_ = t;
	sampleFineDSCurve(t_stop_, p_stop_, v_stop_, a_stop_);

	double stop_jmax = j1_ < j2_ ? j1_ : j2_;

	if (slow_down_.planDSCurve(p_stop_, v_stop_, a_stop_, amax_, stop_jmax))
	{
		double stop_duration = slow_down_.getDuration();
		if (t + stop_duration < t_total_)
		{
			t_total_ = t + stop_duration;
			is_stop_success_ = true;
		}
	}
}

