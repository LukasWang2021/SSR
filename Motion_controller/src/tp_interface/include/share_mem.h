/**
 * @file share_mem_.h
 * @brief: use new API 
 * @author WangWei
 * @version 1.1.0
 * @date 2016-12-20
 */
#ifndef TP_INTERFACE_SHARE_MEM_H_
#define TP_INTERFACE_SHARE_MEM_H_


#include "struct_to_mem/struct_joint_command.h" 
#include "struct_to_mem/struct_feedback_joint_states.h"
#include "comm_interface/core_interface.h"
#include "trajplan/fst_datatype.h"
#include "fst_error.h"
#include "error_code/error_code.h"
#include "service_wrapper.h"
#include <boost/thread/mutex.hpp>

#define READ_COUNT_LIMIT    (50)
#define WRITE_COUNT_LIMIT   (50)

typedef struct _Shm_Joints_Cmd
{
	JointCommand	joint_cmd;
	bool			is_written;
}ShmjointsCmd;


class ShareMem
{
  public:	
	
	ShareMem();
	~ShareMem();

    /**
     * @brief: initialization 
     *
     * @return: 0 if success 
     */
    U64 initial();

    void setWritenFlag(bool flag);

	/**
	 * @brief: get current share memory JointCommand
	 *
	 * @return 
	 */
	ShmjointsCmd getCurrentJointCmd();
	/**
	 * @brief: set current share memory JointCommand
	 *
	 */
	void setCurrentJointCmd(JointCommand joint_cmd);
	/**
	 * @brief: get FeedbackJointState from share memory
	 *
	 * @param fbjs: output==> the FeedbackJointState
	 *
	 * @return: 0 if successfullly get the joint state 
	 */
	U64 getFeedbackJointState(FeedbackJointState &fbjs);
	/**
	 * @brief: set JointCommand to share memory
	 *
	 * @param jc_w: input==> the JointCommand
	 *
	 * @return: 0 if successfullly set JointCommand 
	 */
	U64 setJointPositions(JointCommand jc_w);
	/**
	 * @brief: judge if the JointCommand has successfullly written to Share memory 
	 *
	 * @return: true if success 
	 */
	bool isJointCommandWritten();

    /**
     * @brief: heart beat from bare metal 
     *
     * @param err_list: output==>error list of bare metal
     *
     * @return: number of errors 
     */
    int monitorHearBeat(U64 *err_list);

    /**
     * @brief: resetBareMetal 
     *
     * @return: 0 if success 
     */
    U64 resetBareMetal();

    /**
     * @brief: resetSafety 
     *
     * @return: 0 if success 
     */
    U64 resetSafety();
    
    /**
     * @brief: stopBareMetal 
     *
     * @return: 0 if success 
     */
    U64 stopBareMetal();
     
  private:	
	fst_core_interface::CoreInterface   core_interface_;
    fst_service_wrapper::ServiceWrapper service_wrapper_;
    fst_service_wrapper::ServiceWrapper service_wrapper1_;
	ShmjointsCmd        shm_jnt_cmd_;		//current JointCommand
    boost::mutex		mutex_;
};

#endif
