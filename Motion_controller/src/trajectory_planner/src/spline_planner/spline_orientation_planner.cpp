#include "spline_planner/spline_orientation_planner.h"
#include "basic_alg.h"

SplineOrientationPlanner::SplineOrientationPlanner()
{
    former_angle_ratio_ = 0.5;
}

void SplineOrientationPlanner::setQuaternions(const basic_alg::Quaternion &start, const basic_alg::Quaternion &via, const basic_alg::Quaternion &end)
{
	start_quatern_.x_ = start.x_; start_quatern_.y_ = start.y_;
	start_quatern_.z_ = start.z_; start_quatern_.w_ = start.w_;

	via_quatern_.x_ = via.x_; via_quatern_.y_ = via.y_;
	via_quatern_.z_ = via.z_; via_quatern_.w_ = via.w_;

	end_quatern_.x_ = end.x_; end_quatern_.y_ = end.y_;
	end_quatern_.z_ = end.z_; end_quatern_.w_ = end.w_;

	orientation_angle_start2via_ = via_quatern_.getIncludedAngle(start_quatern_);
	orientation_angle_via2end_ = end_quatern_.getIncludedAngle(via_quatern_);
}

void SplineOrientationPlanner::planCurve(double u0, double u1, double v0, double v1, double t_total_former, double t_total_last)
{
    u0_ = u0;
    u1_ = u1;
    v0_ = v0;
    v1_ = v1;
    t_total_former_ = t_total_former;
    t_total_last_ = t_total_last;

    cubic_spline_former_.planCurve(u0_, 1, v0_, 0, t_total_former_);
    cubic_spline_last_.planCurve(0, u1_, 0, v1_, t_total_last);
}

void SplineOrientationPlanner::sampleCurve(double t, double &u, double &v, double &a)
{
    if (t < MINIMUM_E9)
    {
        u = u0_;
    }
    else if (t < t_total_former_ + MINIMUM_E12)
    {
        cubic_spline_former_.sampleCurve(t, u);
        u = fabs(1 - u0_) < MINIMUM_E9 ? 1.0 :  (u - u0_) / (1 - u0_);
    }
    else if (t < t_total_former_ + t_total_last_ + MINIMUM_E12)
    {
        cubic_spline_last_.sampleCurve(t - t_total_former_, u);
        u = fabs(u1_) < MINIMUM_E9 ? u1_ : u / u1_;
    }
    else 
    {
        u = 1.0;
    }

    v = 0;
    a = 0;
}


void SplineOrientationPlanner::sampleQuaternion(double t, basic_alg::Quaternion &sample)
{
    double us, vs, as;
    sampleCurve(t, us, vs, as);

    if (t < MINIMUM_E9)
    {
        sample.x_ = start_quatern_.x_; sample.y_ = start_quatern_.y_;
	    sample.z_ = start_quatern_.z_; sample.w_ = start_quatern_.w_;
    }
    else if (t < t_total_former_ + MINIMUM_E12)
    {
        sampleSlerpInterpolationQuaternion(start_quatern_, via_quatern_, us, orientation_angle_start2via_, sample);
    }
    else if (t < t_total_former_ + t_total_last_ + MINIMUM_E12)
    {
        sampleSlerpInterpolationQuaternion(via_quatern_, end_quatern_, us, orientation_angle_via2end_, sample);
    }
    else 
    {
        sample.x_ = end_quatern_.x_; sample.y_ = end_quatern_.y_;
	    sample.z_ = end_quatern_.z_; sample.w_ = end_quatern_.w_;
    }
}

double SplineOrientationPlanner::getDuration()
{
    return t_total_former_ + t_total_last_;
}