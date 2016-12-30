#include "common.h"
#include "share_mem.h"
#include <boost/thread.hpp>
#include "fst_error.h"


ShareMem::ShareMem()
{
	shm_jnt_cmd_.is_written = true;  //has not writen any joint command
    read_cnt_ = 0;
    write_cnt_ = 0;
}

ShareMem::~ShareMem()
{

}

U64 ShareMem::initial()
{
    U64 result = FST_SUCCESS;
    //FST_INFO("init share memory...");
#ifdef CROSS_PLATFORM
	if ((result = core_interface_.init()) != FST_SUCCESS)
    {
        return result;
    }    
    if ((result = service_wrapper_.init()) != FST_SUCCESS)
    {
        return result;
    }
#endif

    return result;
}

void ShareMem::setWritenFlag(bool flag)
{
    boost::mutex::scoped_lock lock(mutex_);
    shm_jnt_cmd_.is_written = flag;
}

/**
 * @brief: get current share memory JointCommand
 *
 * @return 
 */
ShmjointsCmd ShareMem::getCurrentJointCmd()
{
    boost::mutex::scoped_lock lock(mutex_);
	return this->shm_jnt_cmd_;
}

/**
 * @brief: set current share memory JointCommand
 *
 */
void ShareMem::setCurrentJointCmd(JointCommand joint_cmd)
{
    boost::mutex::scoped_lock lock(mutex_);
	this->shm_jnt_cmd_.joint_cmd = joint_cmd; 
}

/**
 * @brief: get FeedbackJointState from share memory
 *
 * @param fbjs: output==> the FeedbackJointState
 *
 * @return: true if successfullly get the joint state 
 */
U64 ShareMem::getFeedbackJointState(FeedbackJointState &fbjs)
{    
    boost::mutex::scoped_lock lock(mutex_);
    read_cnt_++;

	memset(&fbjs, 0, sizeof(FeedbackJointState));
#ifdef CROSS_PLATFORM
	U64 result = core_interface_.recvBareCore(fbjs);
#else
	U64 result = core_interface_.recvBareCoreFake(fbjs);
#endif
    if ((result == FST_SUCCESS) || (fbjs.state != STATE_INIT))
    {
        servo_status_ = fbjs;
        read_cnt_ = 0;
        return FST_SUCCESS;
    }		
    else if (read_cnt_ >= READ_COUNT_LIMIT)
    {
        FST_ERROR("read share memory timeout");        
        read_cnt_ = 0;
        return READ_SHARE_MEMORY_TIMEOUT;
    }
    else
    {
       //FST_ERROR("read memory failed:%d times, error:0x%x",read_cnt_,result);
       return READ_SHARE_MEMORY_FAILED;
    }
}

/**
 * @brief: set JointCommand to share memory
 *
 * @param jc_w: input==> the JointCommand
 *
 * @return: true if successfullly set JointCommand 
 */
U64 ShareMem::setJointPositions(JointCommand jc_w)
{
    boost::mutex::scoped_lock lock(mutex_);
   // FST_INFO("write share memory:%f,%f,%f,%f,%f,%f", \
            jc_w.points[0].positions[0],\
            jc_w.points[0].positions[1],\
            jc_w.points[0].positions[2],\
            jc_w.points[0].positions[3],\
            jc_w.points[0].positions[4],\
            jc_w.points[0].positions[5]);
    write_cnt_++;
#ifdef CROSS_PLATFORM
	U64 result = core_interface_.sendBareCore(jc_w);
#else
	U64 result = core_interface_.sendBareCoreFake(jc_w);
#endif
	if (result == FST_SUCCESS)
	{
		shm_jnt_cmd_.is_written = true;
        write_cnt_ = 0;
      //  FST_INFO("Write memory successfullly");
		return 0;
	}
	else 
	{
		shm_jnt_cmd_.is_written = false;
       // if(write_cnt_ <= 5)
           // FST_ERROR("write share memory error:%d times", write_cnt_);
        if (write_cnt_ >= WRITE_COUNT_LIMIT)
        {
            write_cnt_ = 0;
            //FST_ERROR("write share memory timeout");
            return WRITE_SHARE_MEMORY_TIMEOUT;
        }
        else 
        {
           // FST_INFO("Write memory failed");
            return WRITE_SHARE_MEMORY_FAILED;
        }
	}
    
}

/**
 * @brief: judge if the JointCommand has successfullly written to Share memory 
 *
 * @return: true if success 
 */
bool ShareMem::isJointCommandWritten()
{
    boost::mutex::scoped_lock lock(mutex_);
	return shm_jnt_cmd_.is_written;
}


int ShareMem::monitorHearBeat(U64 *err_list)
{
    int err_size;
    boost::mutex::scoped_lock lock(mutex_);
    memset(&resp_, 0, sizeof(ServiceResponse));
    U64 result = service_wrapper_.sendHeartbeatRequest(resp_);
    if (result == FST_SUCCESS)
    {
        err_size = *(int*)resp_.res_buff;
        memcpy(err_list, &resp_.res_buff[8], err_size*sizeof(U64));           
    }
    else
    {
        err_size = 1;
        *err_list = result;
    }
    
    return err_size;
}

U64 ShareMem::resetBareMetal()
{
    boost::mutex::scoped_lock lock(mutex_);
    return service_wrapper_.sendResetRequest();
}


U64 ShareMem::stopBareMetal()
{
    boost::mutex::scoped_lock lock(mutex_);
    return service_wrapper_.sendStopRequest();
}

