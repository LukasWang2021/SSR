#include <iostream>
#include <string.h>
#include <fstream>
#include <math.h>
#include "basic_alg.h"
#include "ds_planner/ds_planner_slow_down.h"

using namespace std;

SlowDownDSCurvePlanner::SlowDownDSCurvePlanner(void)
{
	memset(t_, 0, sizeof(double) * SEGMENT_NUM);
	memset(s_, 0, sizeof(double) * SEGMENT_NUM);
	memset(v_, 0, sizeof(double) * SEGMENT_NUM);
	memset(a_, 0, sizeof(double) * SEGMENT_NUM);

	amax_ = 0.0;
	jmax_ = 0.0;
	jmax_t4_ = 0.0;

	t_total_ = 0.0;
	s_total_ = 0.0;
	q0_ = 0.0;
}
SlowDownDSCurvePlanner::~SlowDownDSCurvePlanner(void)
{

}


bool SlowDownDSCurvePlanner::planDSCurve(double q0, double v0, double a0, double amax, double jmax)
{
	if (v0 < MINIMUM_E12 || fabs(a0) > amax)
	{
		cout << "input error" << endl;
		return false;
	}

	q0_ = q0;
	v0_ = v0;
	a0_ = a0;

	double t1 = 0, t2 = 0, t3 = 0, t4 = 0;
	double a1 = 0, a2 = 0, a3 = 0, a4 = 0;
	double v1 = 0, v2 = 0, v3 = 0, v4 = 0;
	double s1 = 0, s2 = 0, s3 = 0, s4 = 0;

	double a_res = amax, j_res = jmax;
	double tmp;

	if (a0 >0.0) 
	{
		t2 = amax / jmax;
		t1 = a0 / jmax;
		t3 = (v0 + 0.5 * t1 * a0) / amax - t2;
		t4 = t2;
		if (t3 > 0) 
		{
			a_res = amax;
		}
		else 
		{
			t3 = 0;
			t2 = sqrt((v0 + 0.5 * t1 * a0) / jmax);
			a_res = jmax * t2;
			t4 = t2;
		}
	}
	else
	{
		double abs_a0 = fabs(a0);

		//check if 4 only
		t4 = abs_a0 / jmax;
		if (v0 <= (0.5 * jmax * t4 * t4)) 
		{
			j_res = 2.0 * v0 / t4 / t4;
			a_res = abs_a0;

			t1 = 0;
			t2 = 0;
			t3 = 0;
		}
		else if (fabs(a0 - amax) < MINIMUM_E9) 
		{
			t4 = abs_a0 / jmax;
			t3 = (v0 - 0.5*jmax*t4*t4) / abs_a0;
			if (t3>0)
			{
				t1 = 0;
				t2 = 0;
			}
			else 
			{
				j_res = sqrt(2.0 * v0 / t4);
				a_res = abs_a0;

				t1 = 0;
				t2 = 0;
				t3 = 0;
			}
		}
		else 
		{
			//assume we have 2,3,4
			t2 = (amax - abs_a0) / jmax;
			t4 = amax / jmax;
			v2 = t2*0.5*(amax + abs_a0);

			v4 = 0.5*jmax*t4*t4;
			t3 = (v0 - v2 - v4) / amax;

			if (t3 > MINIMUM_E9)
			{
				t1 = 0;
			}
			else
			{
				t1 = 0;
				t3 = 0;
				tmp = abs_a0 * abs_a0 + 2 * jmax*v0;
				a_res = sqrt(0.5 * tmp);

				if (a_res < abs_a0)
				{
					cout << "INTERNAL ERROR, new acc calc fault!!! : "
						<< "a_res = " << a_res
						<< " , " << "amax = " << amax
						<< " , " << "a0 = " << a0 << endl;

					return false;
				}

				t2 = (a_res - abs_a0) / jmax;
				t4 = a_res / jmax;
				v2 = t2 * 0.5 * (a_res + abs_a0);
				v4 = 0.5 * a_res * a_res / jmax;

				if ((v0 - v2 - v4) > MINIMUM_E9) 
				{
					cout << "INTERNAL ERROR: a_max calc fault, V0 not equal with(v2+v4)!!! : "
						<< "a_res = " << a_res
						<< " , " << "v2 = " << v2
						<< " , " << "v4 = " << v4 << endl;
					return false;
				}
			}
		}
	}

	a1 = a0 - jmax * t1;
	a2 = a1 - jmax * t2;

	if (a2 > MINIMUM_E9) 
	{
		cout << "INTERNAL ERROR: a2 should not be positive!!" << endl;
		return false;
	}

	a3 = a2; 
	a4 = 0;

	if (fabs(j_res * t4 + a3) > MINIMUM_E9)
	{
		cout << "-----------------------------------------------------------------------" << endl;
		cout << "INTERNAL ERROR: a3 not match the s4 acc time!!" << endl;
		cout << "t[0-4]: " << 0.0 << ", " << t1 << ", " << t2 << ", " << t3 << ", " << t4 << endl;
		cout << "a[0-4]: " << a0 << ", " << a1 << ", " << a2 << ", " << a3 << ", " << a4 << endl;
		cout << "j_res = " << j_res << ", a_res = " << a_res << endl;
		cout << "----------------------------------------------------------------------" << endl;
		return false;
	}

	v1 = v0 + a0 * t1 - 0.5 * jmax * t1 * t1;
	v2 = v1 - 0.5 * t2 * fabs(a1 + a2);
	v3 = v2 - a_res * t3;
	v4 = 0;

	if (fabs(0.5*j_res*t4*t4 - v3) > 1E-10)
	{
		cout << "-----------------------------------------------------------------------" << endl;
		cout << "INTERNAL ERROR: v3 not match the s4 acc time!!" << endl;
		cout << "t[0-4]: " << 0.0 << ", " << t1 << ", " << t2 << ", " << t3 << ", " << t4 << endl;
		cout << "V[0-4]: " << v0 << ", " << v1 << ", " << v2 << ", " << v3 << ", " << v4 << endl;
		cout << "a[0-4]: " << a0 << ", " << a1 << ", " << a2 << ", " << a3 << ", " << a4 << endl;
		cout << "----------------------------------------------------------------------" << endl;

		return false;
	}

	t_total_ = t1 + t2 + t3 + t4;
	s_total_ = s1 + s2 + s3 + s4;

	s1 = v0 * t1 + 0.5 * a0 * t1 * t1 - 1.0 / 6.0 * jmax * t1 * t1 * t1;
	s2 = v1 * t2 + 0.5 * a1 * t2 * t2 - 1.0 / 6.0 * jmax * t2 * t2 * t2;
	s3 = v2 * t3 + 0.5 * a2 * t3 * t3;
	s4 = 1.0 / 6.0 * j_res * t4 * t4 * t4;

	t_[0] = 0;  t_[1] = t1; t_[2] = t2; t_[3] = t3; t_[4] = t4;
	v_[0] = v0; v_[1] = v1; v_[2] = v2; v_[3] = v3; v_[4] = v4;
	s_[0] = 0;	s_[1] = s1;	s_[2] = s2;	s_[3] = s3;	s_[4] = s4;
	a_[0] = a0;	a_[1] = a1;	a_[2] = a2;	a_[3] = a3;	a_[4] = a4;

	amax_ = a_res;
	jmax_ = jmax;
	jmax_t4_ = j_res;
	
	return true;
}


void SlowDownDSCurvePlanner::sampleDSCurve(double t, double &p, double &v, double &a)
{
	int i;
	double t_total = 0.0, t_base = 0.0, s_total = 0.0;
	t = t < t_total_ ? t : t_total_;
	
	for (i = 0; i < SEGMENT_NUM; i++) 
	{
		t_total += t_[i];
		if (t_total >= t) break;
		s_total += s_[i];
		t_base += t_[i];
	}

	double t_ptr = t - t_base;

	switch (i)
	{
	case 0 :
	{
		s_total = 0;
		v = v_[0];
		a = a_[0];
	}
	break;
	case 1:
	{
		s_total += v_[0] * t_ptr + 0.5 * a_[0] * t_ptr * t_ptr - 1.0 / 6.0 * jmax_ * t_ptr * t_ptr * t_ptr;
		v = v_[0] + a_[0] * t_ptr - 0.5 * jmax_ * t_ptr * t_ptr;
		a = a_[0] - jmax_ * t_ptr;
	}
	break;
	case 2:
	{
		s_total += v_[1] * t_ptr + 0.5 * a_[1] * t_ptr * t_ptr - 1.0 / 6.0 * jmax_ * t_ptr * t_ptr * t_ptr;
		v = v_[1] + a_[1] * t_ptr - 0.5 * jmax_ * t_ptr * t_ptr;
		a = a_[1] - jmax_ * t_ptr;
	}
	break;
	case 3:
	{
		s_total += v_[2] * t_ptr + 0.5 * a_[2] * t_ptr * t_ptr;
		v = v_[2] + a_[2] * t_ptr;
		a = a_[2];
	}
	break;
	case 4:
	{
		s_total += v_[3] * t_ptr + 0.5 * a_[3] * t_ptr * t_ptr + 1.0 / 6.0 * jmax_t4_ * t_ptr * t_ptr * t_ptr;
		a = a_[3] + jmax_t4_ * t_ptr;
		v = v_[3] + a_[3] * t_ptr + 0.5 * jmax_t4_ * t_ptr * t_ptr;
	}
	break;
	default:;
	};

	p = q0_ + s_total;
}

void SlowDownDSCurvePlanner::outputDSCurve(double time_step, const char *file_name)
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
		out_file = "ds_curve_slow_down.csv";
	}

	std::ofstream out(out_file);

	for (double t = 0; t < total_time; t += time_step)
	{
		sampleDSCurve(t, ps, vs, as);
		out << t << "," << ps << "," << vs << "," << as << std::endl;
	}

	out.close();
}

double SlowDownDSCurvePlanner::getDuration(void)
{
	return t_total_;
}