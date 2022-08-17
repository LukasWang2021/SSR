#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

/**
 * @file force_sensor.h
 * @brief The file is the header file of class "ForceSensor".
 * @author 
 */
#include <mutex>
#include "common_datatype.h"
#include "common_error_code.h"
#include "log_manager_producer.h"
#include "system_model_manager.h"
#include "motion_control.h"
#include "vector3.h"

#define FORCE_CALIB_PARAM_INDEX 127
#define FORCE_CALIB_PARAM_NUMS  10


/**
 * @brief io_space includes all io related definitions and implementation.
 */
namespace sensors_space 
{

/**
 * @brief Defines the share memory for io communication.
 */

typedef struct
{
    Vector3 force_off_;
	double  mg_;
	Vector3 torque_off_;
	Vector3 centroid_pos_;
}Force_Sensor_Calib_t;

typedef struct
{
	int32_t fx_off;  //力偏执
	int32_t fy_off;
	int32_t fz_off;
	int32_t fg;		//重力
	int32_t fmx_off;	//扭矩偏置
	int32_t fmy_off;
	int32_t fmz_off;
	int32_t xg;		//质心位置
	int32_t yg;
	int32_t zg;
}Force_Sensor_Param_t;

typedef struct
{
	double force[6];
}Force_Val_t;

/**
 * @brief ForceSensor is a sensor processing force value.
 * @details 
 */
class ForceSensor{
  public:
    /**
     * @brief Constructor of the class. 
     */    
    ForceSensor();
    /**
     * @brief Destructor of the class. 
     */    
    ~ForceSensor();

    /**
     * @brief Initialization.
     * @details Initialize internal variables .\n
     * @param [in] pointer of motion control.
     * @param [in] pointer of controller publish.
     * @param [in] pointer of force model.
     * @retval true success.
     * @retval false Failed to initialize.
     */
    bool init(group_space::MotionControl* group_ptr[GROUP_NUM], 
    			servo_comm_space::ServoCpuCommBase* cpu_comm_ptr,
   		 	system_model_space::ForceModel_t** force_model_ptr);
	
	/**
     * @brief Update source value of force sensor.
     * @details 
     * @param [in] group id[0, 1]
     * @param [out] source value of force sensor.
     * @param [in] the dimension of the force.
     * @retval true success.
     * @return  false Failed to get source value.
     */
	bool updateSourceValue(int group_id, double *dst_dat_ptr=NULL, int nums=6);

    /**
     * @brief calibrated the source value of force sensor.
     * @details 
     * @param [in] group id[0, 1]
     * @param [out] calibration value of force sensor.
     * @param [in] the dimension of the force.
     * @retval true success.
     * @return  false Failed to calibrated.
     */
    bool calibratedForceSensor(int group_id, double *dst_dat_ptr=NULL, int nums=6);

	/**
	  * @brief Translate the calibrated sensor value to the end of tool.
	  * @details 
	  * @param [in] group id[0, 1]
	  * @param [out] calibration value of force sensor.
	  * @param [in] the dimension of the force.
	  * @retval true success.
	  * @return  false Failed to calibrated.
	  */
	bool transCalibrated2Tool(int group_id, double *dst_dat_ptr=NULL, int nums=6);

    /**
     * @brief Load calibration parameters.
     * @details 
     * @param [in] group id[0, 1]
     * @retval true success.
     * @return  false Failed to load parameters.
     */
	bool loadCalibrationParams(int group_id);

	/**
     * @brief Get calibration parameters.
     * @details 
     * @param [in] group id[0, 1]
     * @param [out] calibration value of force sensor.
     * @retval true success.
     * @return  false Failed to get parameters.
     */
	bool getCalibrationParams(int group_id, Force_Sensor_Calib_t *calib_param);

	/**
     * @brief Get source value of force sensor.
     * @details 
     * @param [in] group id[0, 1]
     * @param [out] data buffer of source value.
     * @param [in] the dimension of the force.
     * @retval true success.
     * @return  false Failed to get source value.
     */
	bool getSourceValue(int group_id, double *dat_ptr, int nums);

	/**
     * @brief Get calibrated value of force sensor.
     * @details 
     * @param [in] group id[0, 1]
     * @param [out] data buffer of source value.
     * @param [in] the dimension of the force.
     * @retval true success.
     * @return  false Failed to get source value.
     */
	bool getCalibratedValue(int group_id, double *dat_ptr, int nums);

	/**
     * @brief Get force value of tool.
     * @details 
     * @param [in] group id[0, 1]
     * @param [out] data buffer of source value.
     * @param [in] the dimension of the force.
     * @retval true success.
     * @return  false Failed to get source value.
     */
	bool getToolValue(int group_id, double *dat_ptr, int nums);
	
	/**
     * @brief Get rotation matrix of frame_6 to frame_tool
     * @details 
     * @param [in] group id[0, 1]
     * @retval true success.
     * @return  false Failed to get source value.
     */
	bool getRotationEnd2Tool(int group_id);
    
  private:    	
	group_space::MotionControl* group_ptr_[GROUP_NUM];
    system_model_space::ForceModel_t* force_model_ptr_[GROUP_NUM];	
	servo_comm_space::ServoCpuCommBase* cpu_comm_ptr_;
	
	Force_Sensor_Calib_t force_calib_param_[GROUP_NUM];
	CommRegTorqueData_t force_val[GROUP_NUM];
	bool is_param_load_[GROUP_NUM];
	Force_Val_t force_src[GROUP_NUM], force_calib[GROUP_NUM], force_tool[GROUP_NUM];
	RotationMatrix tmx;
	std::mutex force_src_mutex[GROUP_NUM], force_calib_mutex[GROUP_NUM];
};
}
#endif
