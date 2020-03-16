#include "bezier_planner/orientation_planner.h"
#include "basic_alg.h"
#include "string.h"
#include <math.h>
using namespace basic_alg;

BezierOrientationPlanner::BezierOrientationPlanner(uint32_t curve_power ): curve_power_(curve_power)
{
    memset(x_normalize_, 0, sizeof(double) * (DIV_PARTS + 1));
    memset(y_normalize_, 0, sizeof(double) * (DIV_PARTS + 1));
    memset(s_normalize_, 0, sizeof(double) * (DIV_PARTS + 1));

    orientation_angle_ = 0;
    orientation_angle_start2via_ = 0;
    orientation_angle_via2end_ = 0;
}	

void BezierOrientationPlanner::planCurve(const basic_alg::Quaternion &start, const basic_alg::Quaternion &via, const basic_alg::Quaternion &end)
{
	start_quatern_.x_ = start.x_; start_quatern_.y_ = start.y_;
	start_quatern_.z_ = start.z_; start_quatern_.w_ = start.w_;

	via_quatern_.x_ = via.x_; via_quatern_.y_ = via.y_;
	via_quatern_.z_ = via.z_; via_quatern_.w_ = via.w_;

	end_quatern_.x_ = end.x_; end_quatern_.y_ = end.y_;
	end_quatern_.z_ = end.z_; end_quatern_.w_ = end.w_;

	orientation_angle_start2via_ = via_quatern_.getIncludedAngle(start_quatern_);
	orientation_angle_via2end_ = end_quatern_.getIncludedAngle(via_quatern_);

    double start_normalize[2], via_normalize[2], end_normalize[2];
    start_normalize[0] = 0.0; start_normalize[1] = 1.0; 
    via_normalize[0] = 0.0; via_normalize[1] = 0.0; 
    end_normalize[0] = 1.0; end_normalize[1] = 0.0; 

    buildTable(start_normalize, via_normalize, end_normalize);
	computeMidQuaternion();
}

void BezierOrientationPlanner::computeMidQuaternion()
{
    double s_ratio  = 0.5;
    int index = 0;
    
    for (; index < DIV_PARTS; index++)
	{
		if (s_ratio < s_normalize_[index]) break;
	}

	double s_segment_ratio = fabs((s_ratio - s_normalize_[index - 1]) / (s_normalize_[index] - s_normalize_[index - 1]));
	double ratio_point[2];
	ratio_point[0] = (1 - s_segment_ratio) * x_normalize_[index - 1] + s_segment_ratio * x_normalize_[index];
	ratio_point[1] = (1 - s_segment_ratio) * y_normalize_[index - 1] + s_segment_ratio * y_normalize_[index];

    double alpha = 1 - ratio_point[1];
	double beta = ratio_point[0];

	Quaternion quatern_a, quatern_b;

    sampleSlerpInterpolationQuaternion(start_quatern_, via_quatern_, alpha, orientation_angle_start2via_, quatern_a);
    sampleSlerpInterpolationQuaternion(via_quatern_, end_quatern_, beta, orientation_angle_via2end_, quatern_b);

	RotationMatrix rotation_a, rotation_via, rotation_b, rotation_via_inverse, rotation_point;
	Matrix33 matrix_b2via, matrix_point;
	quatern_a.convertToRotationMatrix(rotation_a);
	quatern_b.convertToRotationMatrix(rotation_b);
	via_quatern_.convertToRotationMatrix(rotation_via);

	rotation_via_inverse = rotation_via;
	rotation_via_inverse.matrix_[0][1] = rotation_via.matrix_[1][0];
	rotation_via_inverse.matrix_[0][2] = rotation_via.matrix_[2][0];
	rotation_via_inverse.matrix_[1][0] = rotation_via.matrix_[0][1];
	rotation_via_inverse.matrix_[1][2] = rotation_via.matrix_[2][1];
	rotation_via_inverse.matrix_[2][0] = rotation_via.matrix_[0][2];
	rotation_via_inverse.matrix_[2][1] = rotation_via.matrix_[1][2];

	matrix_b2via = rotation_via_inverse.rightMultiply(rotation_b);
	matrix_point = rotation_a.rightMultiply(matrix_b2via);
	memcpy(rotation_point.matrix_, matrix_point.matrix_, sizeof(matrix_point.matrix_));
	// rotation_point = matrix_point;
	rotation_point.convertToQuaternion(mid_quatern_);
}


void BezierOrientationPlanner::getMidQuaternion(basic_alg::Quaternion &mid_quatern)
{
	mid_quatern.x_ = mid_quatern_.x_;
	mid_quatern.y_ = mid_quatern_.y_;
	mid_quatern.z_ = mid_quatern_.z_;
	mid_quatern.w_ = mid_quatern_.w_;
}

void BezierOrientationPlanner::buildTable(const double(&start)[2], const double(&via)[2], const double(&end)[2])
{
    double u = 0.0, delta_u = 1.0 / DIV_PARTS, tiny_delta_u = delta_u / 100;
	double p0[2], p0a[2], p1[2], p1a[2], vector_p02p1[2];
	double vector_p02p1_norm;

	s_normalize_[0] = 0.0;

	for (int i = 0; i < DIV_PARTS + 1; ++i)
	{
		u = i / (double)DIV_PARTS;

		if (curve_power_ == QUADRATIC)
		{
			getPoint(start, via, end, u, p0);
			getPoint(start, via, end, u + tiny_delta_u, p0a);
			getPoint(start, via, end, u + delta_u, p1);
			getPoint(start, via, end, u + delta_u + tiny_delta_u, p1a);
		}
		else
		{
			getCubicCurvePoint(start, via, end, u, p0);
			getCubicCurvePoint(start, via, end, u + tiny_delta_u, p0a);
			getCubicCurvePoint(start, via, end, u + delta_u, p1);
			getCubicCurvePoint(start, via, end, u + delta_u + tiny_delta_u, p1a);
		}

		x_normalize_[i] = p0[0];
		y_normalize_[i] = p0[1]; 

		subVector(p1, p0, vector_p02p1);
		vector_p02p1_norm = norm(vector_p02p1);
		if (i < DIV_PARTS) 
		{
			s_normalize_[i + 1] = vector_p02p1_norm + s_normalize_[i];
		}
	}

	for (int i = 0; i != DIV_PARTS + 1; ++i)
	{
		s_normalize_[i] = s_normalize_[i] / s_normalize_[DIV_PARTS];
	}
}

void BezierOrientationPlanner::getPoint(const double(&start)[2], const double(&via)[2], const double(&end)[2], double u, double(&point)[2])
{
	point[0] = (1 - u) * (1 - u) * start[0] + 2 * u * (1 - u) * via[0] + u * u * end[0];
	point[1] = (1 - u) * (1 - u) * start[1] + 2 * u * (1 - u) * via[1] + u * u * end[1];
}

void BezierOrientationPlanner::getCubicCurvePoint(const double(&start)[2], const double(&via)[2], const double(&end)[2], double u, double(&point)[2])
{
	point[0] = (1 - u) * (1 - u) * (1 - u) * start[0] + 3 * u * (1 - u) * via[0] + u * u * u * end[0];
	point[1] = (1 - u) * (1 - u) * (1 - u) * start[1] + 3 * u * (1 - u) * via[1] + u * u * u * end[1];
}