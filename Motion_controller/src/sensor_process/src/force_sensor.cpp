#include <string.h>
#include <stdlib.h>
#include "force_sensor.h"
#include "rotation_matrix.h"
#include "controller_publish.h"
#include "common/comm_reg_3.h"
#include "joint.h"
#include "trans_matrix.h"
#include "pose_euler.h"

using namespace user_space;
using namespace log_space;
using namespace group_space;
using namespace system_model_space;
using namespace sensors_space;

ForceSensor::ForceSensor()
{
	
}
  
ForceSensor::~ForceSensor()
{

}

bool ForceSensor::init(group_space::MotionControl* group_ptr[GROUP_NUM], 
							servo_comm_space::ServoCpuCommBase* cpu_comm_ptr,
							system_model_space::ForceModel_t** force_model_ptr)
{
	bool ret =false;
	
	if(cpu_comm_ptr == NULL) 
		return false;
	cpu_comm_ptr_ = cpu_comm_ptr;
	
	for(int i = 0 ; i < GROUP_NUM; i++)
	{
		if(group_ptr[i] == NULL) return false;
		group_ptr_[i] = group_ptr[i];

		if(force_model_ptr[i] == NULL) return false;
		force_model_ptr_[i] = force_model_ptr[i];
		
		is_param_load_[i] = false;

		if(loadCalibrationParams(i) == false) return false;

		if(getRotationEnd2Tool(i) == false) return false;
	}
	
	return true;
}

bool ForceSensor::updateSourceValue(int group_id, double *dst_dat_ptr, int nums)
{
	
	CommRegTorqueData_t t_data = {0};

    if (cpu_comm_ptr_->getTorqueSensorData(&t_data))
    {
    	force_src_mutex[group_id].lock();
        for(size_t i = 0; i < 6; ++i)
        {
           	force_src[group_id].force[i] = (double)t_data.data[i]/1024.0;
        }
		force_src_mutex[group_id].unlock();
    }
	cpu_comm_ptr_->setTorqueSensorSync(NULL);

	if(dst_dat_ptr != NULL)
		memcpy(dst_dat_ptr, &force_src[group_id].force[0], nums*sizeof(double));
	
	return true;
}


bool ForceSensor::calibratedForceSensor(int group_id, double *dst_dat_ptr, int nums)
{
	Joint joint;
	TransMatrix tmx;
	double b[3]={0.0};
	double c[3]={0.0};
	
	if(group_id >= GROUP_NUM)
		return false;

	/*calibration*/
	joint = group_ptr_[group_id]->getServoJoint();
	group_ptr_[group_id]->convertJointToTmx(joint, tmx);
	
	b[0] = tmx.rotation_matrix_.matrix_[2][0]*force_calib_param_[group_id].mg_;
	b[1] = tmx.rotation_matrix_.matrix_[2][1]*force_calib_param_[group_id].mg_;
	b[2] = tmx.rotation_matrix_.matrix_[2][2]*force_calib_param_[group_id].mg_;
	
	force_calib[group_id].force[0] = force_src[group_id].force[0] - force_calib_param_[group_id].force_off_.x_ - b[0];
	force_calib[group_id].force[1] = force_src[group_id].force[1] - force_calib_param_[group_id].force_off_.y_ - b[1];
	force_calib[group_id].force[2] = force_src[group_id].force[2] - force_calib_param_[group_id].force_off_.z_ - b[2];

	c[0] = force_calib_param_[group_id].centroid_pos_.y_*b[2] - force_calib_param_[group_id].centroid_pos_.z_*b[1];
	c[1] = force_calib_param_[group_id].centroid_pos_.z_*b[0] - force_calib_param_[group_id].centroid_pos_.x_*b[2];
	c[2] = force_calib_param_[group_id].centroid_pos_.x_*b[1] - force_calib_param_[group_id].centroid_pos_.y_*b[0];

	force_calib[group_id].force[3] = force_src[group_id].force[3] - force_calib_param_[group_id].torque_off_.x_ - c[0];
	force_calib[group_id].force[4] = force_src[group_id].force[4] - force_calib_param_[group_id].torque_off_.y_ - c[1];
	force_calib[group_id].force[5] = force_src[group_id].force[5] - force_calib_param_[group_id].torque_off_.z_ - c[2];

	if(dst_dat_ptr != NULL)
		memcpy(dst_dat_ptr, &force_calib[group_id].force[0], nums*sizeof(double));
	
	return true;
}

bool ForceSensor::transCalibrated2Tool(int group_id, double *dst_dat_ptr, int nums)
{
	double b[3], c[3];

	if(group_id >= GROUP_NUM)
			return false;

	force_tool[group_id].force[0] = tmx.matrix_[0][0]*force_calib[group_id].force[0]
							+ tmx.matrix_[1][0]*force_calib[group_id].force[1]
							+ tmx.matrix_[2][0]*force_calib[group_id].force[2];
	force_tool[group_id].force[1] = tmx.matrix_[0][1]*force_calib[group_id].force[0]
							+ tmx.matrix_[1][1]*force_calib[group_id].force[1]
							+ tmx.matrix_[2][1]*force_calib[group_id].force[2];
	force_tool[group_id].force[2] = tmx.matrix_[0][2]*force_calib[group_id].force[0]
							+ tmx.matrix_[1][2]*force_calib[group_id].force[1]
							+ tmx.matrix_[2][2]*force_calib[group_id].force[2];

	b[0] = tmx.matrix_[0][0]*force_tool[group_id].force[0]
				+ tmx.matrix_[1][0]*force_tool[group_id].force[1]
				+ tmx.matrix_[2][0]*force_tool[group_id].force[2];
	b[1] = tmx.matrix_[0][1]*force_tool[group_id].force[0]
				+ tmx.matrix_[1][1]*force_tool[group_id].force[1]
				+ tmx.matrix_[2][1]*force_tool[group_id].force[2];
	b[2] = tmx.matrix_[0][2]*force_tool[group_id].force[0]
				+ tmx.matrix_[1][2]*force_tool[group_id].force[1]
				+ tmx.matrix_[2][2]*force_tool[group_id].force[2];
	c[0] = force_calib_param_[group_id].centroid_pos_.y_*b[2] - force_calib_param_[group_id].centroid_pos_.z_*b[1];
	c[1] = force_calib_param_[group_id].centroid_pos_.z_*b[0] - force_calib_param_[group_id].centroid_pos_.x_*b[2];
	c[2] = force_calib_param_[group_id].centroid_pos_.x_*b[1] - force_calib_param_[group_id].centroid_pos_.y_*b[0];

	c[0] = force_calib[group_id].force[4] - c[0];
	c[1] = force_calib[group_id].force[5] - c[1];
	c[2] = force_calib[group_id].force[6] - c[2];

	force_tool[group_id].force[4] = tmx.matrix_[0][0]*c[0]
							+ tmx.matrix_[1][0]*c[1]
							+ tmx.matrix_[2][0]*c[2];
	force_tool[group_id].force[5] = tmx.matrix_[0][1]*c[0]
							+ tmx.matrix_[1][1]*c[1]
							+ tmx.matrix_[2][1]*c[2];
	force_tool[group_id].force[6] = tmx.matrix_[0][2]*c[0]
							+ tmx.matrix_[1][2]*c[1]
							+ tmx.matrix_[2][2]*c[2];
	
	if(dst_dat_ptr != NULL)
		memcpy(dst_dat_ptr, &force_tool[group_id].force[0], nums*sizeof(double));

}

bool ForceSensor::loadCalibrationParams(int group_id)
{
	Force_Sensor_Param_t params_tmp;
	int32_t *src_ptr = NULL;
	double *dst_ptr = NULL;

	if(group_id >= GROUP_NUM)
		return false;
	
	src_ptr = &(params_tmp.fx_off);
	dst_ptr = &(force_calib_param_[group_id].force_off_.x_);
	
	for(int i = FORCE_CALIB_PARAM_INDEX; i < (FORCE_CALIB_PARAM_INDEX + FORCE_CALIB_PARAM_NUMS); i++ )
	{
		force_model_ptr_[group_id]->force_param_ptr->get(i, src_ptr);
		*dst_ptr = *src_ptr/16777216.0;
		src_ptr++;
		dst_ptr++;
	}

	is_param_load_[group_id] = true;
	return true;	
}

bool ForceSensor::getCalibrationParams(int group_id, Force_Sensor_Calib_t *calib_param)
{
	if(is_param_load_[group_id] == false || group_id >= GROUP_NUM || calib_param == NULL)
		return false;

	memcpy(calib_param, &force_calib_param_[group_id],sizeof(Force_Sensor_Calib_t));	
	return true;
}

bool ForceSensor::getRotationEnd2Tool(int group_id)
{
	PoseEuler pos_euler;
	TransMatrix tmx_tmp;
	
	if(group_id >= GROUP_NUM)
		return false;

	group_ptr_[group_id]->getUsingTool(pos_euler);
	pos_euler.convertToTransMatrix(tmx_tmp);
	tmx = tmx_tmp.rotation_matrix_;
	return true;
}


bool ForceSensor::getSourceValue(int group_id, double *dat_ptr, int nums)
{
	if(dat_ptr == NULL)
		return false;
	
	if(force_src_mutex[group_id].try_lock())
	{
		memcpy(dat_ptr, &force_src[group_id].force[0], nums*sizeof(double));
		force_src_mutex[group_id].unlock();
		return true;
	}
	return false;
}


bool ForceSensor::getCalibratedValue(int group_id, double *dat_ptr, int nums)
{
	if(dat_ptr == NULL)
			return false;
	
	if(force_calib_mutex[group_id].try_lock())
	{
		memcpy(dat_ptr, &force_calib[group_id].force[0], nums*sizeof(double));
		force_calib_mutex[group_id].unlock();
		return true;
	}
	return false;
}
