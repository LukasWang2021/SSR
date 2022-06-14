#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <string.h>
#include <fstream>
#include <vector>
#include <sys/syscall.h>
#include <sys/time.h>
#include <motion_control.h>
#include <tool_manager.h>
#include <coordinate_manager.h>
#include "error_queue.h"
#include <math.h>
#include "onlineTrj_planner.h"

using namespace std;
using namespace basic_alg;
using namespace group_space;
using namespace fst_ctrl;
using namespace base_space;
using namespace log_space;

double OnlinePointJointBuf[1200] = {0};//6*200
int online_trjPointCnt = 0;

void MotionControl::ringCommonTask(void)
{
    group_ptr_->doCommonLoop();
}

void MotionControl::ringRealTimeTask(void)
{
    group_ptr_->doRealtimeLoop();
}

void MotionControl::ringPriorityTask(void)
{
    group_ptr_->doPriorityLoop();
}

void MotionControl::ringOnlineTrajTask(void)
{
    if(online_vp_cache_state_ != 1) return ;// means no vp recieved
    online_vp_cache_state_ = 0; //reset mark means no vp recieved
    //online_traj_sem_.take();
    online_trajData_mutex_.lock();
    //int status=0;
    double vp_cache[81];
    //printf("ringOnlineTrajTask online_trj_planner_ptr->read_TmatrixPackCnt=%d|%d\n",online_trj_planner_ptr->read_TmatrixPackCnt,online_trj_planner_ptr->online_alg_params_.online_receive_Tmatrix_buffPack_len);
    memcpy(vp_cache, online_vp_cache_+online_trj_planner_ptr->read_TmatrixPackCnt*81*sizeof(double), 81*sizeof(double));
    online_trj_planner_ptr->read_TmatrixPackCnt++;
    //if(online_trj_planner_ptr->read_TmatrixPackCnt > online_trj_planner_ptr->get_onlineRecvTmatrixBuffPackLen())
    if(online_trj_planner_ptr->read_TmatrixPackCnt > 30)
    {
        online_trj_planner_ptr->read_TmatrixPackCnt=0;
    }
    receive_T_matrix_data(vp_cache[0],vp_cache);
    //printf("ringOnlineTrajTask--receive_T_matrix_data\n");
    setOnlinePointBufptr();
    //printf("ringOnlineTrajTask--setOnlinePointBufptr\n");
    if(online_trj_planner_ptr->read_TmatrixPackCnt == online_trj_planner_ptr->receive_TmatrixPackCnt)
    {
        online_trj_planner_ptr->read_TmatrixPackCnt = 0;
        online_trj_planner_ptr->receive_TmatrixPackCnt=0;
    }
    //online_traj_sem_.give();
    online_trajData_mutex_.unlock();
}

void MotionControl::ringPlannerTask(void)
{
    pthread_mutex_lock(&instruction_mutex_);
        
    if (!instruction_fifo_.empty() && group_ptr_->nextMovePermitted())
    {
        ErrorCode err = SUCCESS;
        Instruction instruction = instruction_fifo_.front();
        instruction_fifo_.pop();
        instructions_handle_counter_ ++;

        if (instruction.type == MOTION)
        {
            if (instruction.user_op_mode == USER_OP_MODE_SLOWLY_MANUAL)
            {
                if (instruction.target.type == MOTION_JOINT)
                {
                    instruction.target.vel = instruction.target.vel > 0.322886 ? 0.322886 : instruction.target.vel;
                }
                else if (instruction.target.type == MOTION_LINE || instruction.target.type == MOTION_CIRCLE)
                {
                    instruction.target.vel = instruction.target.vel > 250 ? 250 : instruction.target.vel;
                }
            }

            err = autoMove(instruction.target);
        }
        else if (instruction.type == SET_UF)
        {
            err = setUserFrame(instruction.uf_id);
        }
        else if (instruction.type == SET_TF)
        {
            err = setToolFrame(instruction.tf_id);
        }
        else if (instruction.type == SET_OVC)
        {
            err = group_ptr_->setGlobalVelRatio(instruction.ovc);
        }
        else if (instruction.type == SET_OAC)
        {
            err = group_ptr_->setGlobalAccRatio(instruction.oac);
        }
        else if (instruction.type == SET_PAYLOAD)
        {
            err = setPayload(instruction.payload_id);
        }
        else
        {
            LogProducer::error("mc","Invalid instruction type: %d", instruction.type);
            err = INVALID_PARAMETER;
        }

        if (err != SUCCESS)
        {
            LogProducer::error("mc","Execute instruction error, code: 0x%llx", err);
            uint32_t err_level = ((err >> 32) & 0xFFFF);

            if (err_level > 2)
            {
                motion_error_flag_ = true;
            }

            if (err_level >= 3 && err_level <= 7)
            {
                LogProducer::error("mc", "Call interpreter pause, line: %d", instruction.line_num);
                (*instruction.INTERP_STATE_PAUSE)();
                // (*instruction.set_line_num)(instruction.line_num);
            }
            
            ErrorQueue::instance().push(err);
        }
    }
    pthread_mutex_unlock(&instruction_mutex_);
    //printf("Received instruction: %d, handled instruction: %d, instruction list size: %d\n", instructions_recv_counter_, instructions_handle_counter_, instruction_fifo_.size());
}


MotionControl::MotionControl(int32_t id):
    Group(id)
{
    coordinate_manager_ptr_ = NULL;
    tool_manager_ptr_ = NULL;
    group_ptr_ = NULL;

    motion_error_flag_ = false;
    instructions_recv_counter_ = 0;
    instructions_handle_counter_ = 0;
    online_vp_status_ = -1;

    work_mode_ = group_space::USER_OP_MODE_MANUAL;
}

MotionControl::~MotionControl()
{
    if(online_vp_cache_ != NULL) {delete[] online_vp_cache_; online_vp_cache_ = NULL;}
    if (group_ptr_ != NULL)     {delete group_ptr_; group_ptr_ = NULL;};
}

ErrorCode MotionControl::initApplication(fst_ctrl::CoordinateManager* coordinate_manager_ptr, fst_ctrl::ToolManager* tool_manager_ptr)
{
    if (pthread_mutex_init(&instruction_mutex_, NULL) != 0)
    {
        LogProducer::error("mc","Fail to initialize motion mutex.");
        return MC_INTERNAL_FAULT;
    }

    group_ptr_ = new ArmGroup();
    if (group_ptr_ == NULL)
    {
        LogProducer::error("mc","Fail to create motion control group.");
        return MC_INTERNAL_FAULT;
    }

    if (coordinate_manager_ptr == NULL || tool_manager_ptr == NULL)
    {
        LogProducer::error("mc","coordinate-manager: %x, tool-manager: %x",
        coordinate_manager_ptr, tool_manager_ptr);
        return INVALID_PARAMETER;
    }
    coordinate_manager_ptr_ = coordinate_manager_ptr;
    tool_manager_ptr_ = tool_manager_ptr;
    user_frame_id_ = 0;
    tool_frame_id_ = 0;

    ErrorCode  err = group_ptr_->initGroup(coordinate_manager_ptr_, tool_manager_ptr_, &axis_group_, &sm_, cpu_comm_ptr_, db_ptr_);
    if (err != SUCCESS)
    {
        LogProducer::error("mc","Fail to init motion group");
        return err;
    }
    online_trj_planner_ptr = new OnlineTrajectoryPlanner();
    online_vp_cache_ = new double[256];
    return SUCCESS;
}




ManualFrame MotionControl::getManualFrame(void)
{
    return group_ptr_->getManualFrame();
}

ErrorCode MotionControl::setManualFrame(ManualFrame frame)
{
    return group_ptr_->setManualFrame(frame);
}


void MotionControl::getAxisManualStep(double (&steps)[NUM_OF_JOINT])
{
    group_ptr_->getManualStepAxis(steps);
}

double MotionControl::getPositionManualStep(void)
{
    return group_ptr_->getManualStepPosition();
}

double MotionControl::getOrientationManualStep(void)
{
    return group_ptr_->getManualStepOrientation();
}

ErrorCode MotionControl::setAxisManualStep(const double (&steps)[NUM_OF_JOINT])
{
    return group_ptr_->setManualStepAxis(steps);
}

ErrorCode MotionControl::setPositionManualStep(double step)
{
    return group_ptr_->setManualStepPosition(step);
}

ErrorCode MotionControl::setOrientationManualStep(double step)
{
    return group_ptr_->setManualStepOrientation(step);
}

ErrorCode MotionControl::doStepManualMove(const GroupDirection &direction)
{
    if (group_ptr_->getCalibratorPtr()->getCalibrateState() == MOTION_FORBIDDEN)
    {
        LogProducer::error("mc","Cannot manual move, calibrator-state = %d, all motion is forbidden.", MOTION_FORBIDDEN);
        return INVALID_SEQUENCE;
    }
    else if (group_ptr_->getCalibratorPtr()->getCalibrateState() == MOTION_LIMITED)
    {
        if (group_ptr_->getManualFrame() != JOINT)
        {
            LogProducer::error("mc","Cannot manual cartesian in limited state, calibrator-state = %d, manual cartesian is forbidden.", MOTION_LIMITED);
            return INVALID_SEQUENCE;
        }
    }

    return group_ptr_->manualMoveStep(&direction[0]);
}

ErrorCode MotionControl::doContinuousManualMove(const GroupDirection &direction)
{
    if (group_ptr_->getCalibratorPtr()->getCalibrateState() == MOTION_FORBIDDEN)
    {
        LogProducer::error("mc","Cannot manual move, calibrator-state = %d, all motion is forbidden.", MOTION_FORBIDDEN);
        return INVALID_SEQUENCE;
    }
    else if (group_ptr_->getCalibratorPtr()->getCalibrateState() == MOTION_LIMITED)
    {
        if (group_ptr_->getManualFrame() != JOINT)
        {
            LogProducer::error("mc","Cannot manual cartesian in limited state, calibrator-state = %d, manual-frame = %d, manual cartesian is forbidden.", MOTION_LIMITED, group_ptr_->getManualFrame());
            return INVALID_SEQUENCE;
        }
    }

    return group_ptr_->manualMoveContinuous(&direction[0]);
}

ErrorCode MotionControl::doGotoPointManualMove(const Joint &joint)
{
    if (group_ptr_->getCalibratorPtr()->getCalibrateState() != MOTION_NORMAL)
    {
        LogProducer::error("mc","Cannot manual move to point in current state, calibrator-state = %d.", group_ptr_->getCalibratorPtr()->getCalibrateState());
        return INVALID_SEQUENCE;
    }

    IntactPoint point;
    point.joint = joint;
    point.tool_frame = group_ptr_->getToolFrame();
    point.user_frame = group_ptr_->getUserFrame();
    PoseEuler tcp_in_base, fcp_in_base;
    group_ptr_->getKinematicsPtr()->doFK(point.joint, fcp_in_base);
    group_ptr_->getTransformationPtr()->convertFcpToTcp(fcp_in_base, point.tool_frame, tcp_in_base);
    group_ptr_->getTransformationPtr()->convertPoseFromBaseToUser(tcp_in_base, point.user_frame, point.pose.pose);
    point.pose.posture = group_ptr_->getKinematicsPtr()->getPostureByJoint(point.joint);
    point.pose.turn = group_ptr_->getKinematicsPtr()->getTurnByJoint(point.joint);
    return group_ptr_->manualMoveToPoint(point);
}

ErrorCode MotionControl::doGotoPointManualMove(const PoseAndPosture &pose, int user_frame_id, int tool_frame_id)
{
    if (group_ptr_->getCalibratorPtr()->getCalibrateState() != MOTION_NORMAL)
    {
        LogProducer::error("mc","Cannot manual move to point in current state, calibrator-state = %d.", group_ptr_->getCalibratorPtr()->getCalibrateState());
        return INVALID_SEQUENCE;
    }

    if (user_frame_id != user_frame_id_ && user_frame_id != -1)
    {
        LogProducer::error("mc","manualMove: user frame ID = %d mismatch with activated user frame = %d.", user_frame_id, user_frame_id_);
        return INVALID_PARAMETER;
    }

    if (tool_frame_id != tool_frame_id_ && tool_frame_id != -1)
    {
        LogProducer::error("mc","manualMove: tool frame ID = %d mismatch with activated tool frame = %d.", tool_frame_id, tool_frame_id_);
        return INVALID_PARAMETER;
    }

    IntactPoint point;
    point.pose = pose;
    point.tool_frame = group_ptr_->getToolFrame();
    point.user_frame = group_ptr_->getUserFrame();
    PoseEuler tcp_in_base, fcp_in_base;
    group_ptr_->getTransformationPtr()->convertPoseFromUserToBase(point.pose.pose, point.user_frame, tcp_in_base);
    group_ptr_->getTransformationPtr()->convertTcpToFcp(tcp_in_base, point.tool_frame, fcp_in_base);

    if (!group_ptr_->getKinematicsPtr()->doIK(fcp_in_base, point.pose.posture, point.pose.turn, point.joint))
    {
        const PoseEuler &pe = point.pose.pose;
        const PoseEuler &tf = point.tool_frame;
        const PoseEuler &uf = point.user_frame;
        LogProducer::error("mc","IK of manual target pose failed.");
        LogProducer::error("mc","Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pe.point_.x_, pe.point_.y_, pe.point_.z_, pe.euler_.a_, pe.euler_.b_, pe.euler_.c_);
        LogProducer::error("mc","Posture: %d, %d, %d, %d", point.pose.posture.arm, point.pose.posture.elbow, point.pose.posture.wrist, point.pose.posture.flip);
        LogProducer::error("mc","Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
        LogProducer::error("mc","User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
        return MC_COMPUTE_IK_FAIL;
    }

    return group_ptr_->manualMoveToPoint(point);
}

/**************************************************
* 函数功能: 将位置姿态轨迹文件转换为轴角轨迹文件(异地保存)
* 参数:offline_euler_trajectory_filePath---文件名称
* 返回值:错误码
*******************************************************/
ErrorCode MotionControl::convertEulerTraj2JointTraj(const std::string &offline_euler_trajectory_fileName)
{
    PoseAndPosture pos;
    Joint jnt;
    int datalineCnt = 0;
    string euler_trajectory_filePath = "/root/robot_data/trajectory/";
    euler_trajectory_filePath += offline_euler_trajectory_fileName;
    vector<vector<double>> euler_trajArr;//二维数组暂存读入的数据
    if(group_ptr_->readEulerTrajectoryFile(euler_trajectory_filePath, euler_trajArr) == SUCCESS)
    {
        datalineCnt = euler_trajArr.size();
        //新建输出文件(清除原文件)
        ofstream out_joint_trajectory_file("/root/robot_data/trajectory/out.trajectory",ios::trunc);
        if(!out_joint_trajectory_file)
        {
            LogProducer::error("convertEulerTraj2JointTraj","Fail to create out file");
            return INVALID_PARAMETER;
        }
        printf("******************convert result*****************datalineCnt = %d\n",datalineCnt);
        char str_JointData_line[188];
        double last_joint[6]={0,0,0,0,0,0};
        double joint_anglar_velocity[6]={0.0001,0.0001,0.0001,0.0001,0.0001,0.0001};
        sprintf(str_JointData_line,"%d %.3f %d", 6, 0.001, datalineCnt);//轴数,间隔时间,轨迹点数
        out_joint_trajectory_file << str_JointData_line << endl;
        //memset(str_JointData_line,0,188);
        //sprintf(str_JointData_line,"%d %d %d %d %d %d %.4f %.4f %.4f %.4f %.4f %.4f %d %d %d %d %d %d %d %d %d %d %d %d", 0,0,0,0,0,0, 0.0001,0.0001,0.0001,0.0001,0.0001,0.0001,0,0,0,0,0,0,10,10,10,10,10,10);
        //out_joint_trajectory_file << str_JointData_line << endl;    
        for(int i=0;i<datalineCnt;i++)
        {  
            //printf("<=[%3d]=> [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",i, euler_trajArr[i][0], euler_trajArr[i][1], euler_trajArr[i][2], euler_trajArr[i][3], euler_trajArr[i][4], euler_trajArr[i][5]);
            pos.pose.point_.x_ = euler_trajArr[i][0]*1000;
            pos.pose.point_.y_ = euler_trajArr[i][1]*1000;
            pos.pose.point_.z_ = euler_trajArr[i][2]*1000;
            pos.pose.euler_.a_ = euler_trajArr[i][3];
            pos.pose.euler_.b_ = euler_trajArr[i][4];
            pos.pose.euler_.c_ = euler_trajArr[i][5];
            pos.posture.arm   = 1;
            pos.posture.elbow = 1;
            pos.posture.wrist = 1;
            pos.posture.flip  = 0;
            memset(&(pos.turn), 0, 9*sizeof(int));
            memset(&jnt, 0, sizeof(jnt));
            convertCartToJoint(pos,0,0,jnt);
            if(i==0)
            {
                memset(str_JointData_line,0,188);
                sprintf(str_JointData_line,"%.4f %.4f %.4f %.4f %.4f %.4f", jnt.j1_, jnt.j2_, jnt.j3_, jnt.j4_, jnt.j5_, jnt.j6_);//起始位置6关节轴角弧度位置
                out_joint_trajectory_file << str_JointData_line << endl;
                last_joint[0] = jnt.j1_-0.0001;last_joint[1] = jnt.j2_-0.0001;last_joint[2] = jnt.j3_-0.0001;
                last_joint[3] = jnt.j4_-0.0001;last_joint[4] = jnt.j5_-0.0001;last_joint[5] = jnt.j6_-0.0001;
            }
            //printf("(%3d) [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]   \[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]\n",i, jnt.j1_, jnt.j2_, jnt.j3_, jnt.j4_, jnt.j5_, jnt.j6_,last_joint[0],last_joint[1],last_joint[2],last_joint[3],last_joint[4],last_joint[5]);
            //计算每个轴相对于上一个点位置的角速度
            joint_anglar_velocity[0] = (jnt.j1_-last_joint[0])*10430.21937344772*81.0;  //65535/6.2831852 == 10430.21937344772
            joint_anglar_velocity[1] = (jnt.j2_-last_joint[1])*10430.21937344772*100.908375;
            joint_anglar_velocity[2] = (jnt.j3_-last_joint[2])*10430.21937344772*81.053333;
            joint_anglar_velocity[3] = (jnt.j4_-last_joint[3])*10430.21937344772*59.987882; 
            joint_anglar_velocity[4] = (jnt.j5_-last_joint[4])*10430.21937344772*66.75495; 
            joint_anglar_velocity[5] = (jnt.j6_-last_joint[5])*10430.21937344772*44.671266; 
            memset(str_JointData_line,0,188);
            sprintf(str_JointData_line,"%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %d %d %d %d %d %d %d %d %d %d %d %d", jnt.j1_, jnt.j2_, jnt.j3_, jnt.j4_, jnt.j5_, jnt.j6_, joint_anglar_velocity[0],joint_anglar_velocity[1],joint_anglar_velocity[2],joint_anglar_velocity[3],joint_anglar_velocity[4],joint_anglar_velocity[5],0,0,0,0,0,0,10,10,10,10,10,10);
            out_joint_trajectory_file << str_JointData_line << endl;
            last_joint[0] = jnt.j1_;last_joint[1] = jnt.j2_;last_joint[2] = jnt.j3_;last_joint[3] = jnt.j4_;last_joint[4] = jnt.j5_;last_joint[5] = jnt.j6_;
         }
        out_joint_trajectory_file.close();
    }
    return 0;
}

ErrorCode MotionControl::Fir_Bspline_algorithm_test2(void)
{
    string euler_trajectory_filePath = "/root/robot_data/trajectory/VPp.csv";
    vector< vector<double> > euler_trajArr;//二维数组暂存读入的数据
    Vector3 Pdata;
    Vector3 Qdata;
    int datalineCnt=0;
    if(group_ptr_->readEulerTrajectoryFile(euler_trajectory_filePath, euler_trajArr) == SUCCESS)
    {
        datalineCnt = euler_trajArr.size();
        printf("******************convert result*****************datalineCnt = %d\n",datalineCnt);
        for(int i=0;i<1000;i++)
        {
            Pdata.x_ = euler_trajArr[i][0];
            Pdata.y_ = euler_trajArr[i][1];
            Pdata.z_ = euler_trajArr[i][2];
            Qdata.x_ = euler_trajArr[i][3];
            Qdata.y_ = euler_trajArr[i][4];
            Qdata.z_ = euler_trajArr[i][5];

            if(i==0)
            {
                cout << "[input "<<i<<" (" << Pdata.x_ <<"," << Pdata.y_ <<"," << Pdata.z_ <<"," << Qdata.x_ <<"," << Qdata.y_ <<"," << Qdata.z_<<") Start point"<<endl;
                online_trj_planner_ptr->traj_on_FIR_Bspline(Pdata,Qdata,0,0);
            }
            else if(i<999)
            {
                cout << "[input "<<i<<" (" << Pdata.x_ <<"," << Pdata.y_ <<"," << Pdata.z_ <<"," << Qdata.x_ <<"," << Qdata.y_ <<"," << Qdata.z_<<") Middle point"<<endl;
                online_trj_planner_ptr->traj_on_FIR_Bspline(Pdata,Qdata,1,0);
            }else if(i==999)
            {
                cout << "[input "<<i<<" (" << Pdata.x_ <<"," << Pdata.y_ <<"," << Pdata.z_ <<"," << Qdata.x_ <<"," << Qdata.y_ <<"," << Qdata.z_<<") Ending point"<<endl;
                online_trj_planner_ptr->traj_on_FIR_Bspline(Pdata,Qdata,2,0);
            }
        }
    }
    return 0;
}

void MotionControl::xzc_funTest()
{
    Matrix44  TTT;
    Vector3 res_xyz,res_abc;
TTT.matrix_[0][0]=1;TTT.matrix_[0][1]=0;TTT.matrix_[0][2]=0;TTT.matrix_[0][3]=0;
TTT.matrix_[1][0]=0;TTT.matrix_[1][1]=-1;TTT.matrix_[1][2]=0;TTT.matrix_[1][3]=0;
TTT.matrix_[2][0]=0;TTT.matrix_[2][1]=0;TTT.matrix_[2][2]=-1;TTT.matrix_[2][3]=0;
TTT.matrix_[3][0]=0;TTT.matrix_[3][1]=0;TTT.matrix_[3][2]=0;TTT.matrix_[3][3]=1;
    online_trj_planner_ptr->rtm_r2xyzabc(TTT,res_xyz,res_abc);
    res_xyz.print("res_xyz=");
    res_abc.print_abc("res_abc=");
}

ErrorCode MotionControl::setOnlineTrajectoryRatio(double ratio)
{
    online_trj_planner_ptr->setOnlineTrjRatio(ratio);
    return 0;
}
/****
 * 函数功能: 设置在线运动时接收的T矩阵数据缓存
 * 参数: status: touch点状态
 * p_marixArray: 传入的数据指针
 * data_buff_index:数据缓存索引
 * 
*/
ErrorCode MotionControl::setOnlineVpointCache(int status, double * p_marixArray)
{
    assert(p_marixArray != NULL);
    ErrorCode ret_code = 1;
    //printf("xzc-debug:OnlineTraj---p1\n");
    //online_traj_sem_.take();
    online_trajData_mutex_.lock();
    //printf("xzc-debug:OnlineTraj---p2\n");
    online_vp_status_ = status;
    if(online_vp_status_ == 0)//判断该包数据中第一个点是否为起点(起点一定在某包数据中的第一个点)
    {
        online_trj_planner_ptr->receive_TmatrixPackCnt = 0;
        online_trj_planner_ptr->read_TmatrixPackCnt = 0;
        printf("sample_time=%lf,generate_interval=%lf,N_step_P=%d,N_step_Q=%d,N_interpP=%d,N_interpQ=%d,trj_ratio=%lf,recvTmatrix_buffPackLen=%d\n",
            online_trj_planner_ptr->online_alg_params_.sample_time,
            online_trj_planner_ptr->online_alg_params_.generate_traj_interval,
            online_trj_planner_ptr->online_alg_params_.N_step_P,
            online_trj_planner_ptr->online_alg_params_.N_step_Q,
            online_trj_planner_ptr->online_alg_params_.N_interp_P,
            online_trj_planner_ptr->online_alg_params_.N_interp_Q,
            online_trj_planner_ptr->online_alg_params_.trj_ratio,
            online_trj_planner_ptr->online_alg_params_.online_receive_Tmatrix_buffPack_len
        );
    }
    memcpy(online_vp_cache_ + online_trj_planner_ptr->receive_TmatrixPackCnt*81*sizeof(double), p_marixArray, 81*sizeof(double));//1+5*16=81 
    //printf("setOnlineVpointCache online_trj_planner_ptr->receive_TmatrixPackCnt=%d\n",online_trj_planner_ptr->receive_TmatrixPackCnt);
    online_trj_planner_ptr->receive_TmatrixPackCnt+=1;
    //if(online_trj_planner_ptr->receive_TmatrixPackCnt > online_trj_planner_ptr->get_onlineRecvTmatrixBuffPackLen())
    if(online_trj_planner_ptr->receive_TmatrixPackCnt > 30)
    {
        online_trj_planner_ptr->receive_TmatrixPackCnt = 0;
    }
    online_vp_cache_state_ = 1;
    //online_traj_sem_.give();
    ret_code = SUCCESS;
    online_trajData_mutex_.unlock();
    //printf("xzc-debug:OnlineTraj---p3\n");
    return ret_code;
}


ErrorCode MotionControl::receive_T_matrix_data(int status, double * p_marixArray)
{
    static uint64_t iterCnt=0;//进入当前函数的迭代次数
    static Matrix44 T_r0_R;
    static Matrix44 Touch_h0_v;
    Matrix44 Touch_ht_v,T_res;
    int T_matrix_len=0;
    //double k = online_trj_planner_ptr->online_alg_params_.trj_ratio; 
    double k = 1;//机械臂移动距离与touch移动距离的比例系数,    即机械臂移动距离=K*touch移动距离
    Vector3 res_xyz,res_abc;
    PoseEuler StartPositionPose;
    TransMatrix start_trans_matrix;
//cout << "status="<<status<<endl;
    double temp_swap=0;
    iterCnt++;//收包次数
    if(status == 0)//起点
    {
        group_ptr_->setOnlineTrjFirstPointCondition();
        online_trjPointCnt = 0;
        iterCnt=1;
        StartPositionPose = getCurrentPose();
        //StartPositionPose.print("StartPositionPose");
        StartPositionPose.convertToTransMatrix(start_trans_matrix);
        T_r0_R.matrix_[0][0]=start_trans_matrix.rotation_matrix_.matrix_[0][0];
        T_r0_R.matrix_[0][1]=start_trans_matrix.rotation_matrix_.matrix_[0][1]; 
        T_r0_R.matrix_[0][2]=start_trans_matrix.rotation_matrix_.matrix_[0][2];
        T_r0_R.matrix_[0][3]=start_trans_matrix.trans_vector_.x_;
        T_r0_R.matrix_[1][0]=start_trans_matrix.rotation_matrix_.matrix_[1][0];
        T_r0_R.matrix_[1][1]=start_trans_matrix.rotation_matrix_.matrix_[1][1];
        T_r0_R.matrix_[1][2]=start_trans_matrix.rotation_matrix_.matrix_[1][2];
        T_r0_R.matrix_[1][3]=start_trans_matrix.trans_vector_.y_;
        T_r0_R.matrix_[2][0]=start_trans_matrix.rotation_matrix_.matrix_[2][0];
        T_r0_R.matrix_[2][1]=start_trans_matrix.rotation_matrix_.matrix_[2][1];
        T_r0_R.matrix_[2][2]=start_trans_matrix.rotation_matrix_.matrix_[2][2];
        T_r0_R.matrix_[2][3]=start_trans_matrix.trans_vector_.z_;
        T_r0_R.matrix_[3][0]=0;T_r0_R.matrix_[3][1]=0;T_r0_R.matrix_[3][2]=0;T_r0_R.matrix_[3][3]=1;

        /*T_r0_R.matrix_[0][0]=1;T_r0_R.matrix_[0][1]=0; T_r0_R.matrix_[0][2]=0;T_r0_R.matrix_[0][3]=0.36;
        T_r0_R.matrix_[1][0]=0;T_r0_R.matrix_[1][1]=-1;T_r0_R.matrix_[1][2]=0;T_r0_R.matrix_[1][3]=0;
        T_r0_R.matrix_[2][0]=0;T_r0_R.matrix_[2][1]=0;T_r0_R.matrix_[2][2]=-1;T_r0_R.matrix_[2][3]=0.0407;
        T_r0_R.matrix_[3][0]=0;T_r0_R.matrix_[3][1]=0;T_r0_R.matrix_[3][2]=0;T_r0_R.matrix_[3][3]=1;*/
//T_r0_R.print("T_r0_R start");
        /*
        res_xyz.x_ = StartPositionPose.point_.x_;
        res_xyz.y_ = StartPositionPose.point_.y_;
        res_xyz.z_ = StartPositionPose.point_.z_;
        res_abc.x_ = StartPositionPose.euler_.c_;//由于首次输入的abc姿态是从机械臂末端获取的, 运控计算cba顺序, 所以需要ac对调
        res_abc.y_ = StartPositionPose.euler_.b_;
        res_abc.z_ = StartPositionPose.euler_.a_;*/
        online_trj_planner_ptr->rtm_r2xyzabc(T_r0_R,res_xyz,res_abc);
//LogProducer::info("startPoint_xyz2alg"," %lf,%lf,%lf,%lf,%lf,%lf",res_xyz.x_,res_xyz.y_,res_xyz.z_,res_abc.x_,res_abc.y_,res_abc.z_);

        Touch_h0_v.matrix_[0][0]=*(p_marixArray+1); Touch_h0_v.matrix_[0][1]=*(p_marixArray+5); Touch_h0_v.matrix_[0][2]=*(p_marixArray+9);  Touch_h0_v.matrix_[0][3]=*(p_marixArray+13);   //  /1000;
        Touch_h0_v.matrix_[1][0]=*(p_marixArray+2); Touch_h0_v.matrix_[1][1]=*(p_marixArray+6); Touch_h0_v.matrix_[1][2]=*(p_marixArray+10);  Touch_h0_v.matrix_[1][3]=*(p_marixArray+14);  //  /1000;
        Touch_h0_v.matrix_[2][0]=*(p_marixArray+3); Touch_h0_v.matrix_[2][1]=*(p_marixArray+7); Touch_h0_v.matrix_[2][2]=*(p_marixArray+11); Touch_h0_v.matrix_[2][3]=*(p_marixArray+15);   //  /1000;
        Touch_h0_v.matrix_[3][0]=*(p_marixArray+4); Touch_h0_v.matrix_[3][1]=*(p_marixArray+8); Touch_h0_v.matrix_[3][2]=*(p_marixArray+12); Touch_h0_v.matrix_[3][3]=*(p_marixArray+16);
        T_matrix_len=4;
//Touch_h0_v.print("Touch_h0_v");

        online_trjPointCnt += online_trj_planner_ptr->traj_on_FIR_Bspline(res_xyz,res_abc,0,online_trjPointCnt);//起点
        status = 1;
    }
    else
    {
        T_matrix_len=5;
        //T_r0_R.print("T_r0_R--not start");
    }
    for(int i=5-T_matrix_len;i<5;i++)
    {
        Touch_ht_v.matrix_[0][0]=*(p_marixArray+16*i+1); Touch_ht_v.matrix_[0][1]=*(p_marixArray+16*i+5); Touch_ht_v.matrix_[0][2]=*(p_marixArray+16*i+9);  Touch_ht_v.matrix_[0][3]=*(p_marixArray+16*i+13);   // /1000;
        Touch_ht_v.matrix_[1][0]=*(p_marixArray+16*i+2); Touch_ht_v.matrix_[1][1]=*(p_marixArray+16*i+6); Touch_ht_v.matrix_[1][2]=*(p_marixArray+16*i+10);  Touch_ht_v.matrix_[1][3]=*(p_marixArray+16*i+14);  // /1000;
        Touch_ht_v.matrix_[2][0]=*(p_marixArray+16*i+3); Touch_ht_v.matrix_[2][1]=*(p_marixArray+16*i+7); Touch_ht_v.matrix_[2][2]=*(p_marixArray+16*i+11); Touch_ht_v.matrix_[2][3]=*(p_marixArray+16*i+15);   // /1000;
        Touch_ht_v.matrix_[3][0]=*(p_marixArray+16*i+4); Touch_ht_v.matrix_[3][1]=*(p_marixArray+16*i+8); Touch_ht_v.matrix_[3][2]=*(p_marixArray+16*i+12); Touch_ht_v.matrix_[3][3]=*(p_marixArray+16*i+16);
//Touch_ht_v.print("Touch_ht_v");
//cout << "------------------------------------------"<<i<<endl;
        online_trj_planner_ptr->DynamicBaseCoordTransformation(T_r0_R, Touch_h0_v, Touch_ht_v, k, T_res);
//T_res.print("CoordTransformed_input2Alg");
        online_trj_planner_ptr->rtm_r2xyzabc(T_res,res_xyz,res_abc);
        //temp_swap = res_abc.x_; res_abc.x_=res_abc.z_; res_abc.z_ = temp_swap;//a<--->c 
//LogProducer::info("xyz2alg","#%lf,%lf,%lf,%lf,%lf,%lf",res_xyz.x_,res_xyz.y_,res_xyz.z_,res_abc.x_,res_abc.y_,res_abc.z_);
        if(status == 1)//中间途经点
        {
            online_trjPointCnt += online_trj_planner_ptr->traj_on_FIR_Bspline(res_xyz,res_abc,1,online_trjPointCnt);//途中点
        }
        else if(status == 2)//终点
        {
            if(i<4)
            {
                online_trjPointCnt += online_trj_planner_ptr->traj_on_FIR_Bspline(res_xyz,res_abc,1,online_trjPointCnt);//途中点
            }
            else if(i == 4)//一包数据中五个点的最后一个
            {
                while(iterCnt%5 !=0)//每次进入函数传入5个T矩阵, 对应5个途经点,遇到提前结束的情况, 重复终点凑齐刚好结束的情况
                {
                    iterCnt++;
                    printf("------------>>>>>>>>>early termination is encountered.Continue the iteration. iterCnt=%d\n",iterCnt);
                    if(iterCnt%5 !=0)
                    {
                        for(int j=0;j<5;j++)
                        {
                            online_trjPointCnt += online_trj_planner_ptr->traj_on_FIR_Bspline(res_xyz,res_abc,1,online_trjPointCnt);///继续迭代途中点
                            printf("------------>>>>>>>>>online_trjPointCnt=%d\n",online_trjPointCnt);
                        }  
                    }
                    else
                    {
                        for(int j=0;j<4;j++)
                        {
                            online_trjPointCnt += online_trj_planner_ptr->traj_on_FIR_Bspline(res_xyz,res_abc,1,online_trjPointCnt);///继续迭代途中点
                            printf("------------>>>>>>>>>online_trjPointCnt=%d\n",online_trjPointCnt);
                        }
                    }
                }
                /*
                for(int  r=0;r<online_trj_planner_ptr->online_alg_params_.N_interp_Q;r++)//终点时不管是否带速度,都重复N_interp_Q次输入到算法后再停止
                {
                    online_trjPointCnt += online_trj_planner_ptr->traj_on_FIR_Bspline(res_xyz,res_abc,1,online_trjPointCnt);
                }*/
                online_trjPointCnt += online_trj_planner_ptr->traj_on_FIR_Bspline(res_xyz,res_abc,2,online_trjPointCnt);//终点
                printf("------------>>>END>>>online_trjPointCnt=%d\n",online_trjPointCnt);
            }
        }
    }
    return SUCCESS;
}

ErrorCode MotionControl::setOfflineTrajectory(const std::string &offline_trajectory)
{
    string trajectory_file = "/root/robot_data/trajectory/";
    trajectory_file += offline_trajectory;
    return group_ptr_->setOfflineTrajectory(trajectory_file);
}

ErrorCode MotionControl::prepareOfflineTrajectory(void)
{
    if (group_ptr_->getMotionControlState() != STANDBY)
    {
        LogProducer::error("mc","Fail to prepair offline trajectory, state = 0x%x", group_ptr_->getMotionControlState());
        return INVALID_SEQUENCE;
    }

    Joint start_joint = group_ptr_->getStartJointOfOfflineTrajectory();
    Joint current_joint = group_ptr_->getLatestJoint();

    if (start_joint.isEqual(current_joint, MINIMUM_E6))
    {
        return SUCCESS;
    }

    return doGotoPointManualMove(start_joint);
}

ErrorCode MotionControl::moveOfflineTrajectory(void)
{
    return group_ptr_->moveOfflineTrajectory();
}

ErrorCode MotionControl::moveOnlineTrajectory(void)
{
    return group_ptr_->switchToOnlineState();
}
//ErrorCode MotionControl::setOnlinePointBufptr(double * ptr)
//{
//    return group_ptr_->setOnlinePointBufData(ptr);
//}
ErrorCode MotionControl::setOnlinePointBufptr()
{
    ErrorCode err;
    if(online_trjPointCnt == 0)
    {
        //LogProducer::info("setOnlinePointBufptr","online_trjPointCnt == 0");
        return SUCCESS;
    }
    else
    {
        //printf("[###setOnlinePointBufptr### online_trjPointCnt=%d]\n",online_trjPointCnt);
        PoseAndPosture pos;  Joint jnt;
        for(int i =0;i<online_trjPointCnt; i++)
        {
            pos.pose.point_.x_ = online_trj_planner_ptr->trj_point_buf[i].x_;
            pos.pose.point_.y_ = online_trj_planner_ptr->trj_point_buf[i].y_;
            pos.pose.point_.z_ = online_trj_planner_ptr->trj_point_buf[i].z_;
            pos.pose.euler_.a_ = online_trj_planner_ptr->trj_point_buf[i].a_;
            pos.pose.euler_.b_ = online_trj_planner_ptr->trj_point_buf[i].b_;
            pos.pose.euler_.c_ = online_trj_planner_ptr->trj_point_buf[i].c_;
            pos.posture.arm   = 1;
            pos.posture.elbow = 1;
            pos.posture.wrist = 1;
            pos.posture.flip  = 0;
//LogProducer::info("AlgOutputPos"," %lf,%lf,%lf,%lf,%lf,%lf status=%d",pos.pose.point_.x_,pos.pose.point_.y_,pos.pose.point_.z_,pos.pose.euler_.a_,pos.pose.euler_.b_,pos.pose.euler_.c_,online_trj_planner_ptr->trj_point_buf[i].status);
            memset(&(pos.turn), 0, 9*sizeof(int));
            memset(&jnt, 0, sizeof(jnt));
            pos.pose.euler_.a_ = online_trj_planner_ptr->trj_point_buf[i].c_;
            pos.pose.euler_.c_ = online_trj_planner_ptr->trj_point_buf[i].a_;
            err = convertCartToJoint(pos,0,0,jnt);//将xyzabc逆解为轴角
            if(err != SUCCESS)
            {
                LogProducer::error("convertCartToJoint","doIK error!!!---Alg_output_error_pos (%lf,%lf,%lf,%lf,%lf,%lf) status=%d",pos.pose.point_.x_,pos.pose.point_.y_,pos.pose.point_.z_,pos.pose.euler_.a_,pos.pose.euler_.b_,pos.pose.euler_.c_,online_trj_planner_ptr->trj_point_buf[i].status);
                //printf("convertCartToJoint, doIK error!!!\n");
            }
            switch(online_trj_planner_ptr->trj_point_buf[i].status)
            {
                case 0: group_ptr_->setOnlinePointLevelBuf(i,1);break;
                case 1: group_ptr_->setOnlinePointLevelBuf(i,0);break;
                case 2: {
                        group_ptr_->setOnlinePointLevelBuf(i,2);
                    }break;
                default: group_ptr_->setOnlinePointLevelBuf(i,0);break;
            }
            OnlinePointJointBuf[i*6+0]=jnt.j1_;
            OnlinePointJointBuf[i*6+1]=jnt.j2_;
            OnlinePointJointBuf[i*6+2]=jnt.j3_;
            OnlinePointJointBuf[i*6+3]=jnt.j4_;
            OnlinePointJointBuf[i*6+4]=jnt.j5_;
            OnlinePointJointBuf[i*6+5]=jnt.j6_;
            //printf("setOnlinePointBufptr, converted OnlinePointJointBuf[%d]=<%lf,%lf,%lf,%lf,%lf,%lf> status=%d\n",i,OnlinePointJointBuf[i*6+0],OnlinePointJointBuf[i*6+1],OnlinePointJointBuf[i*6+2],OnlinePointJointBuf[i*6+3],OnlinePointJointBuf[i*6+4],OnlinePointJointBuf[i*6+5],online_trj_planner_ptr->trj_point_buf[i].status);
        }
        err = group_ptr_->setOnlineTrjPointBufData(OnlinePointJointBuf,online_trjPointCnt);
        if(err == SUCCESS)
        {
            online_trjPointCnt = 0;//操作完成清零
        }
        return err;
    }
}


ErrorCode MotionControl::MotionStateOnlineToStandby(void)
{
    return group_ptr_->switchOnlineStateToStandby();
}
void MotionControl::clearErrorFlag(void)
{
    motion_error_flag_ = false;
}

ErrorCode MotionControl::autoMove(const struct Instruction &instruction)
{
    /*
    MotionControlState state = group_ptr_->getMotionControlState();
    ServoState servo_state = group_ptr_->getServoState();

    if (state != STANDBY && state != STANDBY_TO_AUTO && state != AUTO)
    {
        LogProducer::error("mc","Cannot autoMove in current state: 0x%x", state);
        return INVALID_SEQUENCE;
    }

    if (servo_state != SERVO_IDLE && servo_state != SERVO_RUNNING)
    {
        LogProducer::error("mc","Cannot autoMove in current servo-state: 0x%x", servo_state);
        return INVALID_SEQUENCE;
    }
    */

    // if (state != STANDBY && state != STANDBY_TO_AUTO && state != AUTO)
    // {
    //     LogProducer::error("mc","Cannot autoMove in current state: 0x%x", state);
    //     return INVALID_SEQUENCE;
    // }

    // if (servo_state != SERVO_IDLE && servo_state != SERVO_RUNNING)
    // {
    //     LogProducer::error("mc","Cannot autoMove in current servo-state: 0x%x", servo_state);
    //     return INVALID_SEQUENCE;
    // }

    pthread_mutex_lock(&instruction_mutex_);
    instruction_fifo_.push(instruction);
    instructions_recv_counter_ ++;
    LogProducer::info("mc","New instruction receieved, add to instruction list, list-size: %d, total-received: %d", instruction_fifo_.size(), instructions_recv_counter_);

    if (instruction_fifo_.size() > 1)
    {
        LogProducer::warn("mc","Instruction fifo size larger than normal, it is a trouble.");
    }

    pthread_mutex_unlock(&instruction_mutex_);
    return SUCCESS;
}

ErrorCode MotionControl::autoMove(const MotionTarget &target)
{
    LogProducer::info("mc","MotionControl::autoMove motion-type: %d, smooth-type: %d", target.type, target.smooth_type);
    LogProducer::info("mc","vel: %.6f, acc: %.6f, cnt: %.6f, tool-frame: %d, user-frame: %d", target.vel, target.acc, target.cnt, target.tool_frame_id, target.user_frame_id);
    
    if (group_ptr_->getCalibratorPtr()->getCalibrateState() != MOTION_NORMAL)
    {
        LogProducer::error("mc","Offset of the group is abnormal, auto move is forbidden, calibrator-state = %d.", group_ptr_->getCalibratorPtr()->getCalibrateState());
        return INVALID_SEQUENCE;
    }

    if (target.type != MOTION_JOINT && target.type != MOTION_LINE && target.type != MOTION_CIRCLE)
    {
        LogProducer::error("mc","Invalid motion type = %d, autoMove aborted.", target.type);
        return INVALID_PARAMETER;
    }

    if (target.user_frame_id != user_frame_id_ && target.user_frame_id != -1)
    {
        LogProducer::error("mc","autoMove: user frame ID = %d mismatch with activated user frame = %d.", target.user_frame_id, user_frame_id_);
        return MC_FRAME_MISMATCH;
    }

    if (target.tool_frame_id != tool_frame_id_ && target.tool_frame_id != -1)
    {
        LogProducer::error("mc","autoMove: tool frame ID = %d mismatch with activated tool frame = %d.", target.tool_frame_id, tool_frame_id_);
        return MC_TOOL_MISMATCH;
    }

    ErrorCode err = SUCCESS;
    MotionInfo motion_info;
    motion_info.smooth_type = target.smooth_type;
    motion_info.type = target.type;
    motion_info.cnt = target.cnt;
    motion_info.vel = target.vel;
    motion_info.acc = target.acc;
    motion_info.is_swift = target.is_swift;
    PoseEuler user_frame = group_ptr_->getUserFrame();
    PoseEuler tool_frame = group_ptr_->getToolFrame();

    if (target.tool_frame_offset.valid)
    {
        LogProducer::info("mc","Use tool offset, type: %d, id: %d", target.tool_frame_offset.coord_type, target.tool_frame_offset.offset_frame_id);

        if (target.tool_frame_offset.coord_type == COORDINATE_JOINT)
        {
            LogProducer::error("mc","Cannot use an joint space offset onto tool frame");
            return INVALID_PARAMETER;
        }

        err = offsetToolFrame(target.tool_frame_offset.offset_frame_id, target.tool_frame_offset.offset_pose, tool_frame);

        if (err != SUCCESS)
        {
            LogProducer::error("mc","Fail to offset too frame, code = 0x%llx", err);
            return err;
        }
    }

    PoseEuler offset_frame;

    if (target.user_frame_offset.valid)
    {
        LogProducer::info("mc","Use frame offset, type: %d, id: %d", target.user_frame_offset.coord_type, target.user_frame_offset.offset_frame_id);
        
        if (target.user_frame_offset.coord_type == COORDINATE_JOINT)
        {
            char buffer[LOG_TEXT_SIZE];
            LogProducer::info("mc","Offset joint: %s", group_ptr_->printDBLine(&target.user_frame_offset.offset_joint.j1_, buffer, LOG_TEXT_SIZE));
        }
        else
        {
            const PoseEuler &offset = target.user_frame_offset.offset_pose;
            LogProducer::info("mc","Offset pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", offset.point_.x_, offset.point_.y_, offset.point_.z_, offset.euler_.a_, offset.euler_.b_, offset.euler_.c_);

            if (target.user_frame_offset.offset_frame_id < 0)
            {
                // 基于当前user-frame进行偏移
                offset_frame = user_frame;
            }
            else if (target.user_frame_offset.offset_frame_id == 0)
            {
                // 基于base-frame进行偏移
                memset(&offset_frame, 0, sizeof(offset_frame));
            }
            else
            {
                CoordInfo uf_info;
                ErrorCode err = coordinate_manager_ptr_->getCoordInfoById(target.user_frame_offset.offset_frame_id, uf_info);

                if (err != SUCCESS)
                {
                    LogProducer::error("mc","Fail to get reference user frame from given id");
                    return err;
                }

                if (!uf_info.is_valid)
                {
                    LogProducer::error("mc","Reference user frame indicated by given id is invalid");
                    return INVALID_PARAMETER;
                }

                offset_frame = uf_info.data;
            }

            LogProducer::info("mc","Offset frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", offset_frame.point_.x_, offset_frame.point_.y_, offset_frame.point_.z_, offset_frame.euler_.a_, offset_frame.euler_.b_, offset_frame.euler_.c_);
        }
    }

    if (target.type == MOTION_CIRCLE)
    {
        LogProducer::info("mc","Handle via point of this motion");
        err = handlePoint(target.via, user_frame, tool_frame, motion_info.via);

        if (err != SUCCESS)
        {
            LogProducer::error("mc","Fail to handle via point, code = 0x%llx", err);
            return err;
        }

        if (target.user_frame_offset.valid)
        {
            err = target.user_frame_offset.coord_type == COORDINATE_JOINT ? offsetPoint(target.user_frame_offset.offset_joint, motion_info.via) : offsetPoint(offset_frame, target.user_frame_offset.offset_pose, motion_info.via);

            if (err != SUCCESS)
            {
                LogProducer::error("mc","Fail to offset target point, code = 0x%llx", err);
                return err;
            }
        }
    }

    LogProducer::info("mc","Handle target point of this motion");
    err = handlePoint(target.target, user_frame, tool_frame, motion_info.target);

    if (err != SUCCESS)
    {
        LogProducer::error("mc","Fail to handle target point, code = 0x%llx", err);
        return err;
    }

    if (target.user_frame_offset.valid)
    {
        err = target.user_frame_offset.coord_type == COORDINATE_JOINT ? offsetPoint(target.user_frame_offset.offset_joint, motion_info.target) : offsetPoint(offset_frame, target.user_frame_offset.offset_pose, motion_info.target);

        if (err != SUCCESS)
        {
            LogProducer::error("mc","Fail to offset target point, code = 0x%llx", err);
            return err;
        }
    }

    char buffer[LOG_TEXT_SIZE];
    const PoseEuler &target_pose = motion_info.target.pose.pose;
    LogProducer::info("mc","Target-pose: %.4f, %.4f, %.4f - %.4f, %.4f, %.4f", target_pose.point_.x_, target_pose.point_.y_, target_pose.point_.z_, target_pose.euler_.a_, target_pose.euler_.b_, target_pose.euler_.c_);
    LogProducer::info("mc","Target-turn: %d, %d, %d, %d, %d, %d", motion_info.target.pose.turn.j1, motion_info.target.pose.turn.j2, motion_info.target.pose.turn.j3, motion_info.target.pose.turn.j4, motion_info.target.pose.turn.j5, motion_info.target.pose.turn.j6);
    LogProducer::info("mc","Target-joint: %s", group_ptr_->printDBLine(&motion_info.target.joint.j1_, buffer, LOG_TEXT_SIZE));
    return group_ptr_->autoMove(motion_info);
}

ErrorCode MotionControl::offsetPoint(const Joint &offset_joint, IntactPoint &point)
{
    Kinematics *kinematics_ptr = group_ptr_->getKinematicsPtr();
    Transformation *transformation_ptr = group_ptr_->getTransformationPtr();
    point.joint += offset_joint;
    point.pose.turn = kinematics_ptr->getTurnByJoint(point.joint);
    point.pose.posture = kinematics_ptr->getPostureByJoint(point.joint);
    PoseEuler fcp_in_base, tcp_in_base;
    kinematics_ptr->doFK(point.joint, fcp_in_base);
    transformation_ptr->convertFcpToTcp(fcp_in_base, point.tool_frame, tcp_in_base);
    transformation_ptr->convertPoseFromBaseToUser(tcp_in_base, point.user_frame, point.pose.pose);
    return SUCCESS;
}

ErrorCode MotionControl::offsetPoint(const PoseEuler &offset_frame, const PoseEuler &offset, IntactPoint &point)
{
    //LogProducer::info("mc","Offset-frame: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", offset_frame.point_.x_, offset_frame.point_.y_, offset_frame.point_.z_, offset_frame.euler_.a_, offset_frame.euler_.b_, offset_frame.euler_.c_);
    //LogProducer::info("mc","Offset: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", offset.point_.x_, offset.point_.y_, offset.point_.z_, offset.euler_.a_, offset.euler_.b_, offset.euler_.c_);
    //LogProducer::info("mc","Frame: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", point.user_frame.point_.x_, point.user_frame.point_.y_, point.user_frame.point_.z_, point.user_frame.euler_.a_, point.user_frame.euler_.b_, point.user_frame.euler_.c_);
    //LogProducer::info("mc","Pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", point.pose.pose.point_.x_, point.pose.pose.point_.y_, point.pose.pose.point_.z_, point.pose.pose.euler_.a_, point.pose.pose.euler_.b_, point.pose.pose.euler_.c_);

    TransMatrix trans_uf, trans_tf, trans_point, trans_frame, trans_offset, trans_result;
    point.pose.pose.convertToTransMatrix(trans_point);
    point.user_frame.convertToTransMatrix(trans_uf);
    point.tool_frame.convertToTransMatrix(trans_tf);
    offset_frame.convertToTransMatrix(trans_frame);
    offset.convertToTransMatrix(trans_offset);

    Point point_offset_in_uf;
    RotationMatrix rotate = trans_uf.rotation_matrix_;
    rotate.inverse();
    rotate.rightMultiply(trans_frame.rotation_matrix_);
    rotate.multiplyByTransVector(trans_offset.trans_vector_, point_offset_in_uf);
    trans_result.trans_vector_ = trans_point.trans_vector_ + point_offset_in_uf;
    
    rotate = trans_uf.rotation_matrix_;
    rotate.inverse();
    rotate.leftMultiply(trans_point.rotation_matrix_).rightMultiply(trans_frame.rotation_matrix_).rightMultiply(trans_offset.rotation_matrix_);
    trans_result.rotation_matrix_ = rotate;
    trans_result.convertToPoseEuler(point.pose.pose);
    //LogProducer::info("mc","Pose: %.6f, %.6f, %.6f, %.6f, %.6f, %.6f", point.pose.pose.point_.x_, point.pose.pose.point_.y_, point.pose.pose.point_.z_, point.pose.pose.euler_.a_, point.pose.pose.euler_.b_, point.pose.pose.euler_.c_);
    trans_tf.inverse();
    trans_result.leftMultiply(trans_uf).rightMultiply(trans_tf);
    Kinematics *kinematics_ptr = group_ptr_->getKinematicsPtr();
    
    if (!kinematics_ptr->doIK(trans_result, point.pose.posture, point.pose.turn, point.joint))
    {
        const Posture &posture = point.pose.posture;
        const PoseEuler &pose = point.pose.pose;
        const PoseEuler &tf = point.tool_frame;
        const PoseEuler &uf = point.user_frame;
        LogProducer::error("mc","IK of point failed.");
        LogProducer::error("mc","Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
        LogProducer::error("mc","Posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
        LogProducer::error("mc","Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
        LogProducer::error("mc","User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
        return MC_COMPUTE_IK_FAIL;
    }

    return SUCCESS;
}


ErrorCode MotionControl::handlePoint(const TargetPoint &origin, const PoseEuler &user_frame, const PoseEuler &tool_frame, IntactPoint &point)
{
    Kinematics *kinematics_ptr = group_ptr_->getKinematicsPtr();
    Transformation *transformation_ptr = group_ptr_->getTransformationPtr();
    point.user_frame = user_frame;
    point.tool_frame = tool_frame;

    if (origin.type == COORDINATE_JOINT)
    {
        PoseEuler fcp_in_base, tcp_in_base;
        point.joint = origin.joint;
        point.pose.turn = kinematics_ptr->getTurnByJoint(point.joint);
        point.pose.posture = kinematics_ptr->getPostureByJoint(point.joint);
        kinematics_ptr->doFK(point.joint, fcp_in_base);
        transformation_ptr->convertFcpToTcp(fcp_in_base, tool_frame, tcp_in_base);
        transformation_ptr->convertPoseFromBaseToUser(tcp_in_base, user_frame, point.pose.pose);
    }
    else
    {
        PoseEuler fcp_in_base, tcp_in_base;
        point.pose = origin.pose;
        transformation_ptr->convertPoseFromUserToBase(point.pose.pose, user_frame, tcp_in_base);
        transformation_ptr->convertTcpToFcp(tcp_in_base, tool_frame, fcp_in_base);

        if (!kinematics_ptr->doIK(fcp_in_base, point.pose.posture, point.pose.turn, point.joint))
        {
            const Posture &posture = point.pose.posture;
            const PoseEuler &pose = point.pose.pose;
            const PoseEuler &tf = point.tool_frame;
            const PoseEuler &uf = point.user_frame;
            LogProducer::error("mc","IK of point failed.");
            LogProducer::error("mc","Pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", pose.point_.x_, pose.point_.y_, pose.point_.z_, pose.euler_.a_, pose.euler_.b_, pose.euler_.c_);
            LogProducer::error("mc","Posture: %d, %d, %d, %d", posture.arm, posture.elbow, posture.wrist, posture.flip);
            LogProducer::error("mc","Tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
            LogProducer::error("mc","User frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
            return MC_COMPUTE_IK_FAIL;
        }
    }

    return SUCCESS;
}

ErrorCode MotionControl::offsetToolFrame(int tool_id, const PoseEuler &offset, PoseEuler &tool_frame)
{
    LogProducer::info("mc","Reference-tool-id: %d, offset-pose: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_id, offset.point_.x_, offset.point_.y_, offset.point_.z_, offset.euler_.a_, offset.euler_.b_, offset.euler_.c_);
    LogProducer::info("mc","Origin-tool-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_frame.point_.x_, tool_frame.point_.y_, tool_frame.point_.z_, tool_frame.euler_.a_, tool_frame.euler_.b_, tool_frame.euler_.c_);
    
    TransMatrix trans_tf, trans_tf_ref;
    tool_frame.convertToTransMatrix(trans_tf);

    if (tool_id < 0)
    {
        // 基于当前tool-frame进行偏移
        trans_tf_ref = trans_tf;
    }
    else if (tool_id == 0)
    {
        // 基于flange-frame进行偏移
        trans_tf_ref.trans_vector_.x_ = 0;
        trans_tf_ref.trans_vector_.y_ = 0;
        trans_tf_ref.trans_vector_.z_ = 0;
        trans_tf_ref.rotation_matrix_.eye();
    }
    else
    {
        ToolInfo tf_info;
        ErrorCode err = tool_manager_ptr_->getToolInfoById(tool_id, tf_info);

        if (err != SUCCESS)
        {
            LogProducer::error("mc","Fail to get reference tool frame from given id");
            return err;
        }

        if (!tf_info.is_valid)
        {
            LogProducer::error("mc","Reference tool frame indicated by given id is invalid");
            return INVALID_PARAMETER;
        }

        tf_info.data.convertToTransMatrix(trans_tf_ref);
        LogProducer::info("mc","Reference tool frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tf_info.data.point_.x_, tf_info.data.point_.y_, tf_info.data.point_.z_, tf_info.data.euler_.a_, tf_info.data.euler_.b_, tf_info.data.euler_.c_);
    }

    TransMatrix trans_offset, trans_new_tf;
    offset.convertToTransMatrix(trans_offset);
    trans_tf_ref.rotation_matrix_.multiplyByTransVector(trans_offset.trans_vector_, trans_new_tf.trans_vector_);
    trans_new_tf.trans_vector_ += trans_tf.trans_vector_;
    trans_new_tf.rotation_matrix_ = trans_tf_ref.rotation_matrix_;
    trans_new_tf.rotation_matrix_.inverse();
    trans_new_tf.rotation_matrix_.leftMultiply(trans_offset.rotation_matrix_).leftMultiply(trans_tf_ref.rotation_matrix_).rightMultiply(trans_tf.rotation_matrix_);
    trans_new_tf.convertToPoseEuler(tool_frame);
    LogProducer::info("mc","New-tool-frame: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_frame.point_.x_, tool_frame.point_.y_, tool_frame.point_.z_, tool_frame.euler_.a_, tool_frame.euler_.b_, tool_frame.euler_.c_);
    return SUCCESS;
}

ErrorCode MotionControl::pauseMove(void)
{
    return group_ptr_->pauseMove();
}

ErrorCode MotionControl::restartMove(void)
{
    return group_ptr_->restartMove();
}

bool MotionControl::isMoving(void)
{
    return group_ptr_->isMoving();
}

bool MotionControl::nextMovePermitted(void)
{
    pthread_mutex_lock(&instruction_mutex_);
    
    if (!instruction_fifo_.empty())
    {
        pthread_mutex_unlock(&instruction_mutex_);
        return false;
    }

    if (motion_error_flag_)
    {
        pthread_mutex_unlock(&instruction_mutex_);
        return false;
    }

    if (!group_ptr_->nextMovePermitted())
    {
        pthread_mutex_unlock(&instruction_mutex_);
        return false;
    }

    pthread_mutex_unlock(&instruction_mutex_);
    return true;
}

ErrorCode MotionControl::setOffset(size_t index, double offset)
{
    ErrorCode err = group_ptr_->getCalibratorPtr()->setOffset(index, offset);

    if (err == SUCCESS)
    {
        OffsetMask mask[NUM_OF_JOINT];
        group_ptr_->getCalibratorPtr()->getOffsetMask(mask);

        for (size_t i = 0; i < group_ptr_->getNumberOfJoint(); i++)
        {
            if (mask[i] == OFFSET_UNMASK)
            {
                group_ptr_->getSoftConstraintPtr()->resetMask(i);
            }
        }

        return SUCCESS;
    }
    else
    {
        return err;
    }
}

ErrorCode MotionControl::setOffset(const double (&offset)[NUM_OF_JOINT])
{
    ErrorCode err = group_ptr_->getCalibratorPtr()->setOffset(offset);

    if (err == SUCCESS)
    {
        OffsetMask mask[NUM_OF_JOINT];
        group_ptr_->getCalibratorPtr()->getOffsetMask(mask);

        for (size_t i = 0; i < group_ptr_->getNumberOfJoint(); i++)
        {
            if (mask[i] == OFFSET_UNMASK)
            {
                group_ptr_->getSoftConstraintPtr()->resetMask(i);
            }
        }

        return SUCCESS;
    }
    else
    {
        return err;
    }
}

void MotionControl::getOffset(double (&offset)[NUM_OF_JOINT])
{
    group_ptr_->getCalibratorPtr()->getOffset(offset);
}

void MotionControl::getOffsetMask(OffsetMask (&mask)[NUM_OF_JOINT])
{
    group_ptr_->getCalibratorPtr()->getOffsetMask(mask);
}

CalibrateState MotionControl::getCalibrateState(void)
{
    return group_ptr_->getCalibratorPtr()->getCalibrateState();
}

ErrorCode MotionControl::maskOffsetLostError(void)
{
    if (group_ptr_->getServoState() != SERVO_DISABLE && group_ptr_->getServoState() != SERVO_WAIT_READY)
    {
        return INVALID_SEQUENCE;
    }

    ErrorCode err = group_ptr_->getCalibratorPtr()->maskOffsetLostError();

    if (err == SUCCESS)
    {
        OffsetMask mask[NUM_OF_JOINT];
        group_ptr_->getCalibratorPtr()->getOffsetMask(mask);

        for (size_t i = 0; i < group_ptr_->getNumberOfJoint(); i++)
        {
            if (mask[i] == OFFSET_MASKED)
            {
                group_ptr_->getSoftConstraintPtr()->setMask(i);
            }
        }

        return SUCCESS;
    }
    else
    {
        return err;
    }
} 

void MotionControl::getOffsetState(OffsetState (&offset_stat)[NUM_OF_JOINT])
{
    return group_ptr_->getCalibratorPtr()->getOffsetState(offset_stat);
}

ErrorCode MotionControl::setOffsetState(size_t index, OffsetState stat)
{
    if (group_ptr_->getServoState() != SERVO_DISABLE && group_ptr_->getServoState() != SERVO_WAIT_READY)
    {
        return INVALID_SEQUENCE;
    }

    return group_ptr_->getCalibratorPtr()->setOffsetState(index, stat);
}

ErrorCode MotionControl::resetEncoderMultiTurnValue(void)
{
    if (group_ptr_->getServoState() != SERVO_DISABLE && group_ptr_->getServoState() != SERVO_WAIT_READY)
    {
        return INVALID_SEQUENCE;
    }
    
    return group_ptr_->getCalibratorPtr()->resetEncoderMultiTurnValue();
}

void MotionControl::getUsingCoord(basic_alg::PoseEuler &usingcoord)
{
    usingcoord = group_ptr_->getUserFrame();
}

void MotionControl::getUsingTool(basic_alg::PoseEuler &usingtool)
{
    usingtool = group_ptr_->getToolFrame();
}

ErrorCode MotionControl::getSoftConstraint(JointConstraint &soft_constraint)
{
    return group_ptr_->getSoftConstraint(soft_constraint);
}

ErrorCode MotionControl::getFirmConstraint(JointConstraint &firm_constraint)
{
    return group_ptr_->getFirmConstraint(firm_constraint);
}

ErrorCode MotionControl::getHardConstraint(JointConstraint &hard_constraint)
{
    return group_ptr_->getHardConstraint(hard_constraint);
}

ErrorCode MotionControl::setSoftConstraint(const JointConstraint &soft_constraint)
{
    return group_ptr_->setSoftConstraint(soft_constraint);
}

ErrorCode MotionControl::setFirmConstraint(const JointConstraint &firm_constraint)
{
    return group_ptr_->setFirmConstraint(firm_constraint);
}

ErrorCode MotionControl::setHardConstraint(const JointConstraint &hard_constraint)
{
    return group_ptr_->setHardConstraint(hard_constraint);
}

ErrorCode MotionControl::stopGroup(void)
{
    return group_ptr_->stopGroup();
}

ErrorCode MotionControl::resetGroup(void)
{
    motion_error_flag_ = false;
    return SUCCESS;
}

ErrorCode MotionControl::clearGroup(void)
{
    pthread_mutex_lock(&instruction_mutex_);
    while (!instruction_fifo_.empty()) instruction_fifo_.pop();
    pthread_mutex_unlock(&instruction_mutex_);
    return group_ptr_->clearGroup();
}

ErrorCode MotionControl::clearTeachGroup(void)
{
    return group_ptr_->clearTeachGroup();
}

void MotionControl::shiftCoordOfPose(const PoseEuler &old_coord, const PoseEuler &old_pose, const PoseEuler &new_coord, PoseEuler &new_pose)
{
    TransMatrix trans_pose, trans_old_coord, trans_new_coord;
    old_pose.convertToTransMatrix(trans_pose);
    old_coord.convertToTransMatrix(trans_old_coord);
    new_coord.convertToTransMatrix(trans_new_coord);
    trans_new_coord.inverse();
    trans_pose.leftMultiply(trans_old_coord).leftMultiply(trans_new_coord).convertToPoseEuler(new_pose);
}

void MotionControl::shiftToolOfPose(const PoseEuler &old_tool, const PoseEuler &old_pose, const PoseEuler &new_tool, PoseEuler &new_pose)
{
    TransMatrix trans_pose, trans_old_tool, trans_new_tool;
    old_pose.convertToTransMatrix(trans_pose);
    new_tool.convertToTransMatrix(trans_new_tool);
    old_tool.convertToTransMatrix(trans_old_tool);
    trans_old_tool.inverse();
    trans_pose.rightMultiply(trans_old_tool).rightMultiply(trans_new_tool).convertToPoseEuler(new_pose);
}

ErrorCode MotionControl::isLinearPathReachable(uint32_t group_id, 
                                    int32_t start_coord_id, int32_t start_tool_id, const PoseAndPosture &start, 
                                    int32_t target_coord_id, int32_t target_tool_id, const PoseAndPosture &target)
{
    if (start_coord_id < 0) start_coord_id = user_frame_id_;
    if (target_coord_id < 0) target_coord_id = user_frame_id_;
    if (start_tool_id < 0) start_tool_id = tool_frame_id_;
    if (target_tool_id < 0) target_tool_id = tool_frame_id_;
    if (start_coord_id != target_coord_id || start_tool_id != target_tool_id)
    {
        LogProducer::error("mc","Start coord=%d, tool=%d, target coord=%d, tool=%d mismatch, linear path unreachable", start_coord_id, start_tool_id, target_coord_id, target_tool_id);
        return INVALID_PARAMETER;
    }

    BaseGroup *group_ptr = group_ptr_;
    PoseEuler coord, tool;

    if (start_coord_id < 0)
    {
        coord = group_ptr->getUserFrame();
    }
    else if (start_coord_id == 0)
    {
        memset(&coord, 0, sizeof(coord));
    }
    else
    {
        CoordInfo info;
        ErrorCode err_user = coordinate_manager_ptr_->getCoordInfoById(start_coord_id, info);

        if (err_user == SUCCESS && info.is_valid)
        {
            coord = info.data;
        }
        else
        {
            LogProducer::error("mc","Fail to get coord from given ID = %d", start_coord_id);
            return err_user == SUCCESS ? INVALID_PARAMETER : err_user;
        }
    }

    if (start_tool_id < 0)
    {
        tool = group_ptr->getToolFrame();
    }
    else if (start_tool_id == 0)
    {
        memset(&tool, 0, sizeof(tool));
    }
    else
    {
        ToolInfo info;
        ErrorCode err_tool = tool_manager_ptr_->getToolInfoById(start_tool_id, info);

        if (err_tool == SUCCESS && info.is_valid)
        {
            tool = info.data;
        }
        else
        {
            LogProducer::error("mc","Fail to get tool from given ID = %d", start_tool_id);
            return err_tool == SUCCESS ? INVALID_PARAMETER : err_tool;
        }
    }

    IntactPoint start_point, target_point;
    start_point.user_frame = coord;
    start_point.tool_frame = tool;
    start_point.pose = start;
    target_point.user_frame = coord;
    target_point.tool_frame = tool;
    target_point.pose = target;

    TransMatrix trans_coord, trans_tool, trans_pose;
    coord.convertToTransMatrix(trans_coord);
    tool.convertToTransMatrix(trans_tool);
    trans_tool.inverse();
    start.pose.convertToTransMatrix(trans_pose);
    trans_pose.rightMultiply(trans_tool).leftMultiply(trans_coord);

    if (!group_ptr->getKinematicsPtr()->doIK(trans_pose, start.posture, start.turn, start_point.joint))
    {
        const Point &point = start.pose.point_;
        const Euler &euler = start.pose.euler_;
        const Posture posture = start.posture;
        const Turn &turn = start.turn;
        LogProducer::error("mc","Fail to compute IK using given pose: %.6f %.6f %.6f %.6f %.6f %.6f", point.x_, point.y_, point.z_, euler.a_, euler.b_, euler.c_);
        LogProducer::error("mc","Posture: ARM=%d, ELBOW=%d, WRIST=%d, FLIP=%d", posture.arm, posture.elbow, posture.wrist, posture.flip);
        LogProducer::error("mc","Start-turn: %d, %d, %d, %d, %d, %d", turn.j1, turn.j2, turn.j3, turn.j4, turn.j5, turn.j6);
        LogProducer::error("mc","Coord: %.6f %.6f %.6f %.6f %.6f %.6f", coord.point_.x_, coord.point_.y_, coord.point_.z_, coord.euler_.a_, coord.euler_.b_, coord.euler_.c_);
        LogProducer::error("mc","Tool: %.6f %.6f %.6f %.6f %.6f %.6f", tool.point_.x_, tool.point_.y_, tool.point_.z_, tool.euler_.a_, tool.euler_.b_, tool.euler_.c_);
        return MC_COMPUTE_IK_FAIL;
    }

    target.pose.convertToTransMatrix(trans_pose);
    trans_pose.rightMultiply(trans_tool).leftMultiply(trans_coord);

    if (!group_ptr->getKinematicsPtr()->doIK(trans_pose, target.posture, target.turn, target_point.joint))
    {
        const Point &point = target.pose.point_;
        const Euler &euler = target.pose.euler_;
        const Posture posture = target.posture;
        const Turn &turn = target.turn;
        LogProducer::error("mc","Fail to compute IK using given pose: %.6f %.6f %.6f %.6f %.6f %.6f", point.x_, point.y_, point.z_, euler.a_, euler.b_, euler.c_);
        LogProducer::error("mc","Posture: ARM=%d, ELBOW=%d, WRIST=%d, FLIP=%d", posture.arm, posture.elbow, posture.wrist, posture.flip);
        LogProducer::error("mc","Target-turn: %d, %d, %d, %d, %d, %d", turn.j1, turn.j2, turn.j3, turn.j4, turn.j5, turn.j6);
        LogProducer::error("mc","Coord: %.6f %.6f %.6f %.6f %.6f %.6f", coord.point_.x_, coord.point_.y_, coord.point_.z_, coord.euler_.a_, coord.euler_.b_, coord.euler_.c_);
        LogProducer::error("mc","Tool: %.6f %.6f %.6f %.6f %.6f %.6f", tool.point_.x_, tool.point_.y_, tool.point_.z_, tool.euler_.a_, tool.euler_.b_, tool.euler_.c_);
        return MC_COMPUTE_IK_FAIL;
    }

    return group_ptr->isLinearPathReachable(start_point, target_point);
}

ErrorCode MotionControl::isPoseReachable(uint32_t group_id, const Joint &joint)
{
    return group_ptr_->getSoftConstraintPtr()->isJointInConstraint(joint) ? SUCCESS : JOINT_OUT_OF_CONSTRAINT;
}

ErrorCode MotionControl::isPoseReachable(uint32_t group_id, int32_t coord_id, int32_t tool_id, const PoseAndPosture &pose)
{
    PoseEuler coord, tool;

    if (coord_id < 0)
    {
        coord = group_ptr_->getUserFrame();
    }
    else if (coord_id == 0)
    {
        memset(&coord, 0, sizeof(coord));
    }
    else
    {
        CoordInfo info;
        ErrorCode err_user = coordinate_manager_ptr_->getCoordInfoById(coord_id, info);

        if (err_user == SUCCESS && info.is_valid)
        {
            coord = info.data;
        }
        else
        {
            LogProducer::error("mc","Fail to get coord from given ID = %d", coord_id);
            return err_user == SUCCESS ? INVALID_PARAMETER : err_user;
        }
    }

    if (tool_id < 0)
    {
        tool = group_ptr_->getToolFrame();
    }
    else if (tool_id == 0)
    {
        memset(&tool, 0, sizeof(tool));
    }
    else
    {
        ToolInfo info;
        ErrorCode err_tool = tool_manager_ptr_->getToolInfoById(tool_id, info);

        if (err_tool == SUCCESS && info.is_valid)
        {
            tool = info.data;
        }
        else
        {
            LogProducer::error("mc","Fail to get tool from given ID = %d", tool_id);
            return err_tool == SUCCESS ? INVALID_PARAMETER : err_tool;
        }
    }

    TransMatrix trans_pose, trans_coord, trans_tool;
    pose.pose.convertToTransMatrix(trans_pose);
    coord.convertToTransMatrix(trans_coord);
    tool.convertToTransMatrix(trans_tool);
    trans_tool.inverse();
    trans_pose.rightMultiply(trans_tool).leftMultiply(trans_coord);
    Joint joint;

    if (!group_ptr_->getKinematicsPtr()->doIK(trans_pose, pose.posture, pose.turn, joint))
    {
        const Point &point = pose.pose.point_;
        const Euler &euler = pose.pose.euler_;
        const Posture posture = pose.posture;
        const Turn &turn = pose.turn;
        LogProducer::error("mc","Fail to compute IK using given pose: %.6f %.6f %.6f %.6f %.6f %.6f", point.x_, point.y_, point.z_, euler.a_, euler.b_, euler.c_);
        LogProducer::error("mc","Posture: ARM=%d, ELBOW=%d, WRIST=%d, FLIP=%d", posture.arm, posture.elbow, posture.wrist, posture.flip);
        LogProducer::error("mc","Target-turn: %d, %d, %d, %d, %d, %d", turn.j1, turn.j2, turn.j3, turn.j4, turn.j5, turn.j6);
        LogProducer::error("mc","Coord: %.6f %.6f %.6f %.6f %.6f %.6f", coord.point_.x_, coord.point_.y_, coord.point_.z_, coord.euler_.a_, coord.euler_.b_, coord.euler_.c_);
        LogProducer::error("mc","Tool: %.6f %.6f %.6f %.6f %.6f %.6f", tool.point_.x_, tool.point_.y_, tool.point_.z_, tool.euler_.a_, tool.euler_.b_, tool.euler_.c_);
        return MC_COMPUTE_IK_FAIL;
    }

    return isPoseReachable(group_id, joint);
}

ErrorCode MotionControl::convertCartToJoint(const PoseAndPosture &pose, int user_frame_id, int tool_frame_id, Joint &joint)
{
    if (user_frame_id == user_frame_id_ && tool_frame_id == tool_frame_id_)
    {
        return group_ptr_->convertCartToJoint(pose, joint);
    }
    PoseEuler tf, uf;

    if (user_frame_id == 0)
    {
        memset(&uf, 0, sizeof(uf));
    }
    else
    {
        CoordInfo uf_info;
        ErrorCode err_user = coordinate_manager_ptr_->getCoordInfoById(user_frame_id, uf_info);

        if (err_user == SUCCESS && uf_info.is_valid)
        {
            uf = uf_info.data;
        }
        else
        {
            LogProducer::error("mc","Fail to get user frame from given ID = %d", user_frame_id);
            return err_user == SUCCESS ? INVALID_PARAMETER : err_user;
        }
    }

    if (tool_frame_id == 0)
    {
        memset(&tf, 0, sizeof(tf));
    }
    else
    {
        ToolInfo tf_info;
        ErrorCode err_tool = tool_manager_ptr_->getToolInfoById(tool_frame_id, tf_info);

        if (err_tool == SUCCESS && tf_info.is_valid)
        {
            tf = tf_info.data;
        }
        else
        {
            LogProducer::error("mc","Fail to get tool frame from given ID = %d", tool_frame_id);
            return err_tool == SUCCESS ? INVALID_PARAMETER : err_tool;
        }
    }

    return group_ptr_->convertCartToJoint(pose, uf, tf, joint);
}

ErrorCode MotionControl::convertCartToJoint(const PoseEuler &pose, int user_frame_id, int tool_frame_id, Joint &joint)
{
    if (user_frame_id == user_frame_id_ && tool_frame_id == tool_frame_id_)
    {
        return group_ptr_->convertCartToJoint(pose, joint);
    }

    PoseEuler tf, uf;

    if (user_frame_id == 0)
    {
        memset(&uf, 0, sizeof(uf));
    }
    else
    {
        CoordInfo uf_info;
        ErrorCode err_user = coordinate_manager_ptr_->getCoordInfoById(user_frame_id, uf_info);

        if (err_user == SUCCESS && uf_info.is_valid)
        {
            uf = uf_info.data;
        }
        else
        {
            LogProducer::error("mc","Fail to get user frame from given ID.");
            return err_user;
        }
    }

    if (tool_frame_id == 0)
    {
        memset(&tf, 0, sizeof(tf));
    }
    else
    {
        ToolInfo tf_info;
        ErrorCode err_tool = tool_manager_ptr_->getToolInfoById(tool_frame_id, tf_info);

        if (err_tool == SUCCESS && tf_info.is_valid)
        {
            tf = tf_info.data;
        }
        else
        {
            LogProducer::error("mc","Fail to get tool frame from given id");
            return err_tool;
        }
    }

    return group_ptr_->convertCartToJoint(pose, uf, tf, joint);
}

ErrorCode MotionControl::convertJointToCart(const Joint &joint, int user_frame_id, int tool_frame_id, PoseEuler &pose)
{
    if (user_frame_id == user_frame_id_ && tool_frame_id == tool_frame_id_)
    {
        return group_ptr_->convertJointToCart(joint, pose);  // transform from base to user
    }

    PoseEuler tf, uf;

    if (user_frame_id == 0)
    {
        memset(&uf, 0, sizeof(uf));
    }
    else
    {
        CoordInfo uf_info;
        ErrorCode err_user = coordinate_manager_ptr_->getCoordInfoById(user_frame_id, uf_info);

        if (err_user == SUCCESS && uf_info.is_valid)
        {
            uf = uf_info.data;
        }
        else
        {
            LogProducer::error("mc","Fail to get user frame from given ID = %d", user_frame_id);
            return err_user == SUCCESS ? INVALID_PARAMETER : err_user;
        }
    }

    if (tool_frame_id == 0)
    {
        memset(&tf, 0, sizeof(tf));
    }
    else
    {
        ToolInfo tf_info;
        ErrorCode err_tool = tool_manager_ptr_->getToolInfoById(tool_frame_id, tf_info);

        if (err_tool == SUCCESS && tf_info.is_valid)
        {
            tf = tf_info.data;
        }
        else
        {
            LogProducer::error("mc","Fail to get tool frame from given id = %d", tool_frame_id);
            return err_tool == SUCCESS ? INVALID_PARAMETER : err_tool;
        }
    }

    return group_ptr_->convertJointToCart(joint, uf, tf, pose);
}

Posture MotionControl::getPostureFromJoint(const Joint &joint)
{
    return group_ptr_->getKinematicsPtr()->getPostureByJoint(joint);
}

Turn MotionControl::getTurnFromJoint(const Joint &joint)
{
    return group_ptr_->getKinematicsPtr()->getTurnByJoint(joint);
}

string MotionControl::getModelName(void)
{
    return "RTM-P7A";
}

size_t MotionControl::getNumberOfAxis(void)
{
    return group_ptr_->getNumberOfJoint();
}

void MotionControl::getTypeOfAxis(AxisType *types)
{
    group_ptr_->getTypeOfAxis(types);
}

int MotionControl::getGroupID(void)
{
    return group_ptr_->getID();
}

MotionControlState MotionControl::getMotionControlState(void)
{
    return group_ptr_->getMotionControlState();
}

ServoState MotionControl::getServoState(void)
{
    return group_ptr_->getServoState();
}


PoseEuler MotionControl::getCurrentPose(void)
{
    PoseEuler pose, tcp_in_base, fcp_in_base;
    group_ptr_->getKinematicsPtr()->doFK(group_ptr_->getLatestJoint(), fcp_in_base);
    group_ptr_->getTransformationPtr()->convertFcpToTcp(fcp_in_base, group_ptr_->getToolFrame(), tcp_in_base);
    group_ptr_->getTransformationPtr()->convertPoseFromBaseToUser(tcp_in_base, group_ptr_->getUserFrame(), pose);
    return pose;
}

void MotionControl::getCurrentPose(PoseEuler &pose)
{
    PoseEuler tcp_in_base, fcp_in_base;
    group_ptr_->getKinematicsPtr()->doFK(group_ptr_->getLatestJoint(), fcp_in_base);
    group_ptr_->getTransformationPtr()->convertFcpToTcp(fcp_in_base, group_ptr_->getToolFrame(), tcp_in_base);
    group_ptr_->getTransformationPtr()->convertPoseFromBaseToUser(tcp_in_base, group_ptr_->getUserFrame(), pose);
}

Joint MotionControl::getServoJoint(void)
{
    return group_ptr_->getLatestJoint();
}

ErrorCode MotionControl::setGlobalVelRatio(double ratio)
{
    return group_ptr_->setGlobalVelRatio(ratio);
}

ErrorCode MotionControl::setGlobalAccRatio(double ratio)
{
    return group_ptr_->setGlobalAccRatio(ratio);
}

double MotionControl::getGlobalVelRatio(void)
{
    return group_ptr_->getGlobalVelRatio();
}

double MotionControl::getGlobalAccRatio(void)
{
    return group_ptr_->getGlobalAccRatio();
}

void MotionControl::getToolFrame(int &id)
{
    id = tool_frame_id_;
}

ErrorCode MotionControl::setToolFrame(int id)
{
    LogProducer::info("mc","Set tool frame: id = %d, current is %d", id, tool_frame_id_);

    if (id == 0)
    {
        PoseEuler tf = {0, 0, 0, 0, 0, 0};
        group_ptr_->setToolFrame(tf);
        tool_frame_id_ = id;
        LogProducer::info("mc","Using tool-frame-%d: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_frame_id_, tf.point_.x_, tf.point_.y_, tf.point_.z_, tf.euler_.a_, tf.euler_.b_, tf.euler_.c_);
        return SUCCESS;
    }
    else
    {
        ToolInfo  tf_info;
        ErrorCode err = tool_manager_ptr_->getToolInfoById(id, tf_info);

        if (err == SUCCESS && tf_info.is_valid)
        {
            group_ptr_->setToolFrame(tf_info.data);
            tool_frame_id_ = id;
            LogProducer::info("mc","Using tool-frame-%d: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", tool_frame_id_, tf_info.data.point_.x_, tf_info.data.point_.y_, tf_info.data.point_.z_, tf_info.data.euler_.a_, tf_info.data.euler_.b_, tf_info.data.euler_.c_);
            return SUCCESS;
        }
        else
        {
            LogProducer::error("mc","Fail to get tool frame from given id");
            return err;
        }
    }
}

void MotionControl::getUserFrame(int &id)
{
    id = user_frame_id_;
}

ErrorCode MotionControl::setUserFrame(int id)
{
    LogProducer::info("mc","Set user frame id = %d, current is %d", id, user_frame_id_);

    if (id == 0)
    {
        PoseEuler uf = {0, 0, 0, 0, 0, 0};
        group_ptr_->setUserFrame(uf);
        user_frame_id_ = id;
        LogProducer::info("mc","Using user-frame-%d: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", user_frame_id_, uf.point_.x_, uf.point_.y_, uf.point_.z_, uf.euler_.a_, uf.euler_.b_, uf.euler_.c_);
        return SUCCESS;
    }
    else
    {
        CoordInfo uf_info;
        ErrorCode err = coordinate_manager_ptr_->getCoordInfoById(id, uf_info);

        if (err == SUCCESS && uf_info.is_valid)
        {
            group_ptr_->setUserFrame(uf_info.data);
            user_frame_id_ = id;
            LogProducer::info("mc","Using user-frame-%d: %.6f, %.6f, %.6f - %.6f, %.6f, %.6f", user_frame_id_, uf_info.data.point_.x_, uf_info.data.point_.y_, uf_info.data.point_.z_, uf_info.data.euler_.a_, uf_info.data.euler_.b_, uf_info.data.euler_.c_);
            return SUCCESS;
        }
        else
        {
            LogProducer::error("mc","Fail to get user frame from given id");
            return err;
        }
    }
}

// payload
ErrorCode MotionControl::setPayload(int id)
{
    MotionControlState mc_state = group_ptr_->getMotionControlState();
    ServoState servo_state = group_ptr_->getServoState();

    if (mc_state != STANDBY)
    {
        LogProducer::error("mc","Cannot set payload while group-state: %d", mc_state);
        return INVALID_SEQUENCE;
    }

    if (servo_state != SERVO_IDLE && servo_state != SERVO_DISABLE)
    {
        LogProducer::error("mc","Cannot set payload while servo-state: %d", servo_state);
        return INVALID_SEQUENCE;
    }

    return group_ptr_->setPayload(id);
}

void MotionControl::getPayload(int &id)
{
    group_ptr_->getPayload(id);
}

ErrorCode MotionControl::addPayload(const PayloadInfo& info)
{
    return group_ptr_->addPayload(info);
}

ErrorCode MotionControl::deletePayload(int id)
{
    return group_ptr_->deletePayload(id);
}

ErrorCode MotionControl::updatePayload(const basic_alg::PayloadInfo& info)
{
    return group_ptr_->updatePayload(info);
}

ErrorCode MotionControl::movePayload(int expect_id, int original_id)
{
    return group_ptr_->movePayload(expect_id, original_id);
}

ErrorCode MotionControl::getPayloadInfoById(int id, PayloadInfo& info)
{
    return group_ptr_->getPayloadInfoById(id, info);
}

std::vector<basic_alg::PayloadSummaryInfo> MotionControl::getAllValidPayloadSummaryInfo(void)
{
    return group_ptr_->getAllValidPayloadSummaryInfo();
}

void MotionControl::getAllValidPayloadSummaryInfo(vector<PayloadSummaryInfo>& info_list)
{
    group_ptr_->getAllValidPayloadSummaryInfo(info_list);
}

//ssr
void MotionControl::setWorkMode(UserOpMode mode)
{
    work_mode_ = mode;
}

UserOpMode MotionControl::getWorkMode(void)
{
    return work_mode_;
}

//pure function no realization
ErrorCode MotionControl::mcGroupHalt(double dec, double jerk)
{
    return GROUP_INVALID_PARAM;
}
ErrorCode MotionControl::mcGroupInterrupt(double dec, double jerk)
{
    return GROUP_INVALID_PARAM;
}
ErrorCode MotionControl::mcGroupContinue(void)
{
    return GROUP_INVALID_PARAM;
}
ErrorCode MotionControl::mcGroupSetPosition(const std::vector<double> &position, bool relative, CoordType_e coord_type)
{
    return GROUP_INVALID_PARAM;
}
ErrorCode MotionControl::mcMoveDirectAbsolute(const std::vector<double> &position, CoordType_e coord_type,
        double vel_pct, double acc_pct, double jerk)
{
    return GROUP_INVALID_PARAM;
}
ErrorCode MotionControl::mcMoveLinearAbsolute(const std::vector<double> &position, CoordType_e coord_type, 
        double velocity, double acc, double dec, double jerk)
{
    return GROUP_INVALID_PARAM;
}
ErrorCode MotionControl::mcGroupReadActualPosition(CoordType_e coord_type, std::vector<double> &position)
{
    position.clear();
    ErrorCode result = SUCCESS;
    if (coord_type == COORD_TYPE_ACS)
    {
        std::map<int32_t, axis_space::Axis*>::iterator it;
        size_t i = 0;
        for (it = axis_group_.begin(), i = INDEX_JOINT1; it != axis_group_.end(); ++it, ++i)
        {
            double pos = 0;
            ErrorCode err = it->second->mcReadActualPosition(pos);
            if (err != SUCCESS)
                result = err;

            if (INDEX_JOINT6 == i)
            {
                pos = group_ptr_->decouplingAxis6ByRad(position[INDEX_JOINT5], position[INDEX_JOINT6]);
            }
            position.push_back(pos);
        }
    }
    else 
    {    
        result = GROUP_INVALID_PARAM;  
        LogProducer::error("mc", "Group[%d] mcGroupReadActualPosition failed, coord = %d, err = 0x%llx", getID(), coord_type, result);
    }

    if (result != SUCCESS)
    {
        LogProducer::error("mc", "Group[%d] mcGroupReadActualPosition failed, err = 0x%llx", getID(), result);
    }   
    return result;
}
ErrorCode MotionControl::mcGroupReadActualVelocity(CoordType_e coord_type, std::vector<double> &velocity)
{
    velocity.clear();
    ErrorCode result = SUCCESS;
    if (coord_type == COORD_TYPE_ACS)
    {
        std::map<int, axis_space::Axis*>::iterator it;
        for (it = axis_group_.begin(); it != axis_group_.end(); ++it)
        {
            double vel = 0;
            ErrorCode err = it->second->mcReadActualVelocity(vel);
            if (err != SUCCESS)
                result = err;

            velocity.push_back(vel);
        }
    }
    else 
    {     
        result = GROUP_INVALID_PARAM;
        LogProducer::error("mc", "Group[%d] mcGroupReadActualVelocity failed, coord = %d, err = 0x%llx", getID(), coord_type, result);
    }

    if (result != SUCCESS)
    {
        LogProducer::error("mc", "Group[%d] mcGroupReadActualVelocity failed, err = 0x%llx", getID(), result);
    }   
    return result;
}
bool MotionControl::initApplication(void)
{
    return false;
}
bool MotionControl::reloadSystemModel(void)
{
    return true;
}
bool MotionControl::pushBackFB(void* fb_ptr)
{
    return false;
}
base_space::FBQueueStatus_e MotionControl::getFBQStatus()
{
    return FBQ_STATUS_FULL;
}
void MotionControl::processFBQ()
{
    
}
void MotionControl::processTBQ()
{
    
}
void MotionControl::clearBQ()
{
    
}