#ifndef CONTROLLER_PUBLISH_H
#define CONTROLLER_PUBLISH_H

/**
 * @file controller_publish.h
 * @brief The file is the header file of class "ControllerPublish".
 * @author Feng.Wu
 */

#include <stddef.h>
#include <string>
#include <vector>
#include <list>
#include "common_datatype.h"
#include "protoc.h"
#include "tp_comm.h"
#include "log_manager_producer.h"
#include "system/core_comm_system.h"
#include "system/servo_cpu_comm_base.h"
#include "axis.h"
#include "motion_control.h"
#include "io_1000.h"
#include "io_safety.h"
#include "force_sensor.h"

/**
 * @brief user_space includes the user level implementation.
 */
namespace user_space
{

/**
 * @brief ControllerPublish deals with the publishing.
 * @details 
 */
class ControllerPublish
{
public:
    /**
     * @brief Constructor of the class.
     */
    ControllerPublish();
    /**
     * @brief Destructor of the class. 
     */  
    ~ControllerPublish();

    /**
     * @brief Initialization.
     * @details
     * @param [in] tp_comm_ptr The pointer of the communication with the upper computer.
     * @param [in] cpu_comm_ptr The pointer to communicate with the other cpu.
     * @param [in] axis_ptr The pointer of all the axes.
     * @param [in] group_ptr The pointer of all the groups.
     * @return void.
     */
    void init(user_space::TpComm* tp_comm_ptr, servo_comm_space::ServoCpuCommBase* cpu_comm_ptr, 
        axis_space::Axis* axis_ptr[AXIS_NUM], group_space::MotionControl* group_ptr[GROUP_NUM], 
        hal_space::Io1000* io_dev_ptr, hal_space::IoSafety* safety_ptr,
        sensors_space::ForceSensor* force_sensor_ptr);

    /**
     * @brief Gets the pointer of the publishing value.
     * @details
     * @return The pointer of the value.
     */
    typedef void* (ControllerPublish::*HandlePublishFuncPtr)(void);

    /**
     * @brief Update the publishing value.
     * @details
     * @return void.
     */
    typedef void (ControllerPublish::*HandleUpdateFuncPtr)(void);

    /**
     * @brief Gets the pointer of a function by hash value.
     * @details
     * @param [in] hash The hash value.
     * @return The pointer of a function.
     */
    HandlePublishFuncPtr getPublishHandlerByHash(unsigned int hash);

    /**
     * @brief Gets the pointer of a function by hash value.
     * @details
     * @param [in] hash The hash value.
     * @return The pointer of a function.
     */
    HandleUpdateFuncPtr getUpdateHandlerByHash(unsigned int hash);

    /**
     * @brief Add the pointer of the update function to a list.
     * @details
     * @param [in] func_ptr The pointer of the update function.
     * @return void.
     */
    void addTaskToUpdateList(HandleUpdateFuncPtr func_ptr);

    /**
     * @brief Delete the update function by hash.
     * @details
     * @param [in] publish_element_list The list of the publish function.
     * @return void.
     */
	void deleteTaskFromUpdateList(std::vector<user_space::TpPublishElement>& publish_element_list);

    /**
     * @brief Decrease the count of the publish element.
     * @details
     * @param [in] func_ptr The pointer of the update function.
     * @return void.
     */
    void unrefUpdateListElement(HandleUpdateFuncPtr func_ptr);

    /**
     * @brief Clean the publishing update list.
     * @details
     * @return void.
     */
    void cleanUpdateList();
    
    /**
     * @brief Update the publishing element data.
     * @details It should be cyclically called by Controller.
     * @return void.
     */    
    void processPublish();
    
private:
    user_space::TpComm* tp_comm_ptr_; 
    servo_comm_space::ServoCpuCommBase* cpu_comm_ptr_;
    axis_space::Axis* axis_ptr_[AXIS_NUM];
    group_space::MotionControl* group_ptr_[GROUP_NUM];
    hal_space::Io1000* io_dev_ptr_;
    hal_space::IoSafety* safety_ptr_;
	sensors_space::ForceSensor* force_sensor_ptr_;
		
    enum {HASH_BYTE_SIZE = 4,};
    enum {QUICK_SEARCH_TABLE_SIZE = 128,};

    typedef struct
    {
        std::string path;
        unsigned int hash;
        HandlePublishFuncPtr publish_func_ptr;
        HandleUpdateFuncPtr update_func_ptr;
    }PublishService;
    std::vector<PublishService> publish_table_;
    std::vector<PublishService> publish_quick_search_table_[QUICK_SEARCH_TABLE_SIZE]; 
    typedef struct
    {
        HandleUpdateFuncPtr func_ptr;
        int ref_count;
    }PublishUpdate;
    std::list<PublishUpdate> update_list_;

    MessageType_AxisFeedbackList axis_fdb_;
    MessageType_ServoFeedbackList servo1001_servo_fdb_;
    MessageType_Uint32List servo1001_cpu_fdb_;
    MessageType_Uint32List io_digital_fdb_;
    MessageType_Uint32List io_safety_fdb_;
	MessageType_DoubleList torque_fdb_;

    void initPublishTable();
    void initPublishQuickSearchTable();

    //get publish element ptr
    void* getAxisFdbPtr();
    void* getServo1001ServoFdbPtr();
    void* getServo1001CpuFdbPtr();
    void* getIODigitalFdbPtr();
    void* getIOSafetyFdbPtr();
    void* getTorqueFdbPtr();
	
    // update publish element
    void updateAxisFdb();
    void updateServo1001ServoFdb();
    void updateServo1001CpuFdb();
    void updateIODigitalFdb();
    void updateIOSafetyFdb();
	void updateTorqueFdb();
};

}


#endif

