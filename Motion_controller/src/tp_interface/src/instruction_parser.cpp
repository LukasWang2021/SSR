#include "instruction_parser.h"
#include "io_interface.h"
#include "error_code.h"
#include "error_monitor.h"
#include "sub_functions.h"
#include "share_mem.h"

#define MIN_POINTS_FOR_NEXT_CMD     (150)

InstructionParser::InstructionParser(ArmGroup *arm_group):arm_group_(arm_group)
{
    line_id_ = -1;
    //prev_id_ = -1;
   // pick_wait_flag_ = false;
   // pre_move_cmd_ = NULL;
    //smooth_mode_ = 0;
    /*cur_inst_it_ = instruction_list_.end();*/
    /*first_move_it_ = instruction_list_.end();*/
}
InstructionParser::~InstructionParser()
{

}

int InstructionParser::getLineID()
{
    return line_id_;
}
#if 0
void InstructionParser::updateID(int id)
{
    if (cur_id_ == id)
        return;
    FST_INFO("update id:%d", id);
    prev_id_ = cur_id_.load();
    cur_id_ = id;
}

int InstructionParser::getPreviousCmdID()
{
	return prev_id_;
}

void InstructionParser::setPreviousCmdID(int id)
{
	prev_id_ = id;
}

int InstructionParser::getCurrentCmdID()
{
	return cur_id_;
}

void InstructionParser::setCurrentCmdID(int id)
{
	cur_id_ = id;
}
#endif

void InstructionParser::updateTrajRemainCount()
{
    if (arm_group_->getLatestCommandLength() == 0)
    {
        if (mot_target_.cnt >= 0)   //wait until fifo len < 50
        {
            if (arm_group_->getFIFOLength() < 50)
            {
                //ShareMem::instance()->setIntprtSendFlag(true);
            }
        }
        else    //fine wait servo ready
        {
            if (ShareMem::instance()->getServoState() == STATE_READY)
            {
              //  ShareMem::instance()->setIntprtSendFlag(true);
            }
        }
    }
}

void InstructionParser::pickPoints(JointPoint *jnt)
{
    /*if (jnt->source->getTrajLength() - jnt->stamp == 0)*/
    //{
        //CommandInstruction *instr = (CommandInstruction*)jnt->source->getParentInstr();
        //instr->is_pickedout = true;
    /*}*/
}

#if 0
int InstructionParser::getInstructionListSize()
{
    int size = instruction_list_.size();
    if (size == 1)
    {
        CommandInstruction instruction = instruction_list_.front();
        if (instruction.smoothDistance >= 0)
        {

            if (cur_inst_it_->path_fifo_len <= MIN_POINTS_FOR_NEXT_CMD)
            {
                size = -1;
            }            
        } 
    }
    else if (size > 0)
    {
        CommandInstruction instruction = instruction_list_.back();
        if ((instruction.pick_status == PICKED) && (instruction.smoothDistance >= 0))
        {
            size = -1;
        }
    }

    return size;
}
#endif


double InstructionParser::getGlobalVelocity()
{
    return arm_group_->getGlobalVelRatio();
}

void InstructionParser::setGlobalVelocity(double factor)
{
    if (arm_group_->setGlobalVelRatio(factor) == false)
    {
        rcs::Error::instance()->add(INVALID_PARAM_FROM_TP);
    }
}
#if 0
U64 InstructionParser::pickInstructions()
{
	U64 result = TPI_SUCCESS;
    /*if (pick_wait_flag_    //do need to wait */
    //|| (cur_inst_it_ == instruction_list_.end())
    //|| (cur_inst_it_->path_fifo_len > 0))
    //{
        //return TPI_SUCCESS;
    /*}*/

    FST_INFO("cur type :%d", target_inst_.commandtype);
    switch (target_inst_.commandtype)
    {
        case motion_spec_MOTIONTYPE_WAIT:
        {
            break;
        }
        case motion_spec_MOTIONTYPE_SET:
        {
            /*motion_spec_Set *set = &target_inst_.command_arguments.set;       */
            //IOInterface::instance()->setDO(set->path,  set->value);
            //if (cur_inst_it_->timesout > 0)
            //{
                //set->value = !set->value;
            //}
            //else
            //{
                //instruction_list_.erase(cur_inst_it_);                
            //}
            //cur_inst_it_++;
            /*pickInstructions();*/
            break;
        }
        case motion_spec_MOTIONTYPE_JOINTMOTION:
        case motion_spec_MOTIONTYPE_CARTMOTION:
        case motion_spec_MOTIONTYPE_CIRCLEMOTION:
        {
            /*if (cur_inst_it_->smoothDistance >= 0)*/
            //{
                //CommandInstruction *next_instruction = pickNextMoveInstruction();
                //if (next_instruction) //find next_instrusmoothDistancection
                //{
                    //FST_INFO("next_instruction:%d", next_instruction->id);
                    //result = moveInstructions(*cur_inst_it_, *next_instruction); 
                //}
                //else 
                //{
                    ////still no next instruction
                    ////move as fine
                    //cur_inst_it_->smoothDistance = -1;
                    //result = moveInstructions(*cur_inst_it_);
                    //pick_wait_flag_ = true;
                //}
            //}//end if (cur_inst_it_->smoothDistance >= 0)
            //else
            //{
                //result = moveInstructions(*cur_inst_it_);   
                //pick_wait_flag_ = true;
            //}//end else

            /*cur_inst_it_->path_fifo_len = cur_inst_it_->move_cmd->getTrajLength();*/
            break;
        }
        default:
            break;
    }

    FST_INFO("pic result :%llx", result);
    return result;
}


void InstructionParser::setPickWaitFlag(bool flag)
{
    pick_wait_flag_ = flag;
}


void InstructionParser::addMotionInstruction(CommandInstruction *cmd_instruction)
{
    if (instruction_list_.empty())
    {
        instruction_list_.push_back(*cmd_instruction);    
        cur_inst_it_ = instruction_list_.begin();
    }
    else
    {
        instruction_list_.push_back(*cmd_instruction);
    }

    //////////////////////////////////////////////////
    //  value first_move_it_
    //  =========================================
    /*if (first_move_it_ == instruction_list_.end())*/
    //{
        //std::list<CommandInstruction>::iterator it = instruction_list_.begin();
        //for (; it != instruction_list_.end(); ++it)
        //{
            //if ((it->commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)
            //|| (it->commandtype == motion_spec_MOTIONTYPE_CARTMOTION)
            //|| (it->commandtype == motion_spec_MOTIONTYPE_CIRCLEMOTION))
            //{
                //first_move_it_ = it;
                //break;
            //}
        //}
    /*}*/
}

void InstructionParser::runNMInstructions()
{
    if (instruction_list_.empty())
        return;
    ///////////////////////////////////////////////////////////////////////
    //process all the non move instructions
    //====================================================================
    std::list<CommandInstruction>::iterator it = instruction_list_.begin();
    do
    {
        if (it->commandtype == motion_spec_MOTIONTYPE_SET)
        {
            if (it->count++ < it->timesout)
                continue;
            
            motion_spec_Set *set = &it->command_arguments.set;
            IOInterface::instance()->setDO(set->path, set->value);
            instruction_list_.erase(it);    //delete this instruction
            if (it == cur_inst_it_)
            {
                cur_inst_it_++;
                break;
            }
        }
    }while(it++ != cur_inst_it_);

    if (cur_inst_it_->commandtype == motion_spec_MOTIONTYPE_WAIT)
    {
        motion_spec_Wait *wait = &it->command_arguments.wait;
        if ((wait->has_timeout) && (it->count++ >= it->timesout))
        {
            instruction_list_.erase(it); //delete this instruction
            if (it == cur_inst_it_)
                cur_inst_it_++;
            pick_wait_flag_ = false;
            return;
        } 
        if (wait->has_value)
        {
            unsigned char value;
            int len;
            IOInterface::instance()->getDIO(wait->path, &value, 1, len); 
            //FST_INFO("setval:%d, value:%d", wait->value, value);
            if (wait->value == value)
            {
                instruction_list_.erase(it); //delete this instruction
                if (it == cur_inst_it_)
                    cur_inst_it_++;
                pick_wait_flag_ = false;
            }
        }// if (wait->has_value)                
    }// if (cur_inst_it_->commandtype == motion_spec_MOTIONTYPE_WAIT)
}


void InstructionParser::rmMoveInstruction()
{
    std::list<CommandInstruction>::iterator it;
    for (it = instruction_list_.begin(); it != instruction_list_.end(); it++)
    {
        if ((it->commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)
        || (it->commandtype == motion_spec_MOTIONTYPE_CARTMOTION)
        || (it->commandtype == motion_spec_MOTIONTYPE_CIRCLEMOTION))
        {
            FST_INFO("delete move instruction id:%d", it->id); 
            if (cur_inst_it_ == it)
            {
                cur_inst_it_++;
                if (cur_inst_it_ != instruction_list_.end())
                {
                    FST_INFO("new instruction id:%d", cur_inst_it_->id);
                }
            }
            instruction_list_.erase(it);          
            break;
        }
    }
}

void InstructionParser::resetInstructionList()
{
    std::list<CommandInstruction>::iterator it = instruction_list_.begin();
    for (; it != instruction_list_.end(); it++)
    {
        if (it->pick_status != FRESH)
            it->pick_status = FRESH;
    }
}


bool InstructionParser::isInstructionExist()
{
    if (instruction_list_.empty())
        return false;
    
    return true;
}
#endif
void InstructionParser::motionClear()
{
    /*while (!instruction_list_.empty())*/
    //{
       //instruction_list_.pop_front();
    //}

    //prev_id_ = -1;
    //cur_id_ = -1;
        
    /*pick_wait_flag_ = false;    //don't wait any flag*/
}

void InstructionParser::pause()
{
    /*if (cur_inst_it_->commandtype == motion_spec_MOTIONTYPE_WAIT)*/
    //{

        //cur_inst_it_->count = 0;
    /*}*/
    U64 result = arm_group_->suspendMotion();
    if (TPI_SUCCESS != result)
    {
        rcs::Error::instance()->add(result);            
    }
}

bool InstructionParser::resume()
{
    U64 result = arm_group_->resumeMotion();
    //=====clear all the fifos=======
    if (TPI_SUCCESS != result)
    {
        rcs::Error::instance()->add(result);
        return false;
    }

    return true;
}

bool InstructionParser::step()
{

}
bool InstructionParser::jump()
{

}
bool InstructionParser::backwords()
{

}


  
#if 0
CommandInstruction* InstructionParser::pickNextMoveInstruction()
{
    std::list<CommandInstruction>::iterator it = cur_inst_it_;
    while (++it != instruction_list_.end())
    {
        if ((it->commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)
        || (it->commandtype == motion_spec_MOTIONTYPE_CARTMOTION)
        || (it->commandtype == motion_spec_MOTIONTYPE_CIRCLEMOTION))
        {            
            return &(*it);
        }
        else if (it->commandtype == motion_spec_MOTIONTYPE_WAIT) //if wait, then must be fine
        {
            cur_inst_it_->smoothDistance = -1;
            return NULL;
        }
    }//end for (it = instruction_list_.begin()

    return NULL;
}  

void* InstructionParser::setPulseIO(void* params)
{
    CommandInstruction *instr = (CommandInstruction*)params;
    motion_spec_Set *set = &instr->command_arguments.set;                            
    //FST_INFO("set do:path:%s, value:%d, count:%d", set->path, set->value, instruction.count);
    IOInterface::instance()->setDO(set->path,  !set->value);
    instr->pick_status = PICKED;
}

void* InstructionParser::waitTimeout(void* params)
{
    CommandInstruction *instr = (CommandInstruction*)params;
    instr->pick_status = PICKED;
}
#endif
U64 InstructionParser::moveTarget()
{

    return arm_group_->autoMove(mot_target_, 0); //id:0 default or ignore

}
#if 0

U64 InstructionParser::moveInstruction(CommandInstruction &target_inst)
{
    U64 result = TPI_SUCCESS;
    MotionTarget target;
        target.cnt = target_inst.smoothDistance;
    switch (target_inst.commandtype)
    {
        case motion_spec_MOTIONTYPE_JOINTMOTION:
        {
            motion_spec_MoveJ* moveJ =  &target_inst.command_arguments.movej;
            unitConvert(moveJ, target);
            break;
        }
        case motion_spec_MOTIONTYPE_CARTMOTION:
        {
            motion_spec_MoveL* moveL = &target_inst.command_arguments.movel;
            unitConvert(moveL, target);
            break;
        }
        case motion_spec_MOTIONTYPE_CIRCLEMOTION:
        {
            motion_spec_MoveC *moveC = &target_inst.command_arguments.movec;
            unitConvert(moveC, target);
            break;
        }
    }

    target_inst.move_cmd = new MoveCommand1(target, target_inst.id);
    target_inst.move_cmd->LinkPrevObj(pre_move_cmd_);   //link previous move command
    pre_move_cmd_ = target_inst.move_cmd;
    return arm_group_->autoMove(target, target_inst.id);
}

U64 InstructionParser::moveInstructions(CommandInstruction target_inst)
{
    U64 result = TPI_SUCCESS;

    /*switch (target_inst.commandtype)*/
    //{
        //case motion_spec_MOTIONTYPE_JOINTMOTION:
        //{                
            //motion_spec_MoveJ* moveJ =  &target_inst.command_arguments.movej;
            //moveJ->smoothPercent = target_inst.smoothDistance; //first rewrite this value
            //Joint target_jnts;
            //MoveJParam movej_param;
            //unitConvert(moveJ, target_jnts, movej_param);
            //FST_INFO("v:%f, a:%f", movej_param.vel_max, movej_param.acc_max);
            //FST_INFO("target joints:%f,%f,%f,%f,%f,%f", target_jnts.j1, target_jnts.j2, target_jnts.j3, target_jnts.j4, target_jnts.j5, target_jnts.j6);                
            

            //arm_group_->MoveJ(target_jnts, movej_param.vel_max, \
                        //movej_param.acc_max, target_inst.id, result);
            //break;
        //}//end case motion_spec_MOTIONTYPE_JOINTMOTION:
        //case motion_spec_MOTIONTYPE_CARTMOTION:
        //{
            //motion_spec_MoveL* moveL = &target_inst.command_arguments.movel;
            //moveL->waypoints[0].smoothPercent = target_inst.smoothDistance; //first rewrite this value

            //PoseEuler target_pose;
            //MoveLParam movel_param;
            //unitConvert(moveL, target_pose, movel_param);
            //FST_INFO("target pose:%f,%f,%f,%f,%f,%f", target_pose.position.x,target_pose.position.y,target_pose.position.z, target_pose.orientation.a, target_pose.orientation.b, target_pose.orientation.c);

           //arm_group_->MoveL(target_pose, movel_param.vel_max, \
                    //movel_param.acc_max, target_inst.id, result);
            //break;
        //}//end case motion_spec_MOTIONTYPE_CARTMOTION:
        //case motion_spec_MOTIONTYPE_CIRCLEMOTION:
        //{
            //motion_spec_MoveC *moveC = &target_inst.command_arguments.movec;
            //PoseEuler target_pose1, target_pose2;
            //MoveCParam target_param;
            //unitConvert(moveC, target_pose1, target_pose2, target_param); 
            //printDbLine("movec pose2:", (double*)&target_pose2, 6);

            //arm_group_->MoveC(target_pose1, target_pose2, target_param.vel_max, \
                            //target_param.acc_max, target_inst.id, result);
            //break;
        //}//end case motion_spec_MOTIONTYPE_CIRCLEMOTION:
        //default:
            //break;
    /*}*/
    
    return result;
}

U64 InstructionParser::moveInstructions(CommandInstruction target_inst, CommandInstruction next_inst)
{
    U64 result = TPI_SUCCESS;
 
    /*switch (target_inst.commandtype)*/
    //{
        //case motion_spec_MOTIONTYPE_JOINTMOTION:
        //{                
            //motion_spec_MoveJ* moveJ =  &target_inst.command_arguments.movej;
            //moveJ->smoothPercent = target_inst.smoothDistance; //first rewrite this value
            //Joint target_jnts;
            //MoveJParam target_param;
            //unitConvert(moveJ, target_jnts, target_param);
            //FST_INFO("v:%f, a:%f", target_param.vel_max, target_param.acc_max);
            //FST_INFO("target joints:%f,%f,%f,%f,%f,%f, smmoth:%f", target_jnts.j1, target_jnts.j2, target_jnts.j3, target_jnts.j4, target_jnts.j5, target_jnts.j6, target_inst.smoothDistance);                 
            
            //if (next_inst.commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)
            //{
                //motion_spec_MoveJ* moveJ_next =  &next_inst.command_arguments.movej;
                //Joint next_jnts;
                //MoveJParam next_param;
                //unitConvert(moveJ_next, next_jnts, next_param);

                //arm_group_->MoveJ(target_jnts,target_param.vel_max, target_param.acc_max,\
                        //target_param.smooth, next_jnts, next_param.vel_max, next_param.acc_max,\
                        //next_param.smooth, target_inst.id, result);
            //}
            //else if (next_inst.commandtype == motion_spec_MOTIONTYPE_CARTMOTION)
            //{
                //motion_spec_MoveL* moveL_next =  &next_inst.command_arguments.movel;
                //PoseEuler next_pose;
                //MoveLParam next_param;
                //unitConvert(moveL_next, next_pose, next_param);

                //arm_group_->MoveJ(target_jnts,target_param.vel_max, target_param.acc_max,\
                        //target_param.smooth, next_pose, next_param.vel_max, next_param.acc_max,\
                        //next_param.smooth, target_inst.id, result);
            //}      
            //else if (next_inst.commandtype == motion_spec_MOTIONTYPE_CIRCLEMOTION)
            //{
                //motion_spec_MoveC *moveC_next = &next_inst.command_arguments.movec;
                //PoseEuler next_pose1, next_pose2;
                //MoveCParam next_param;
                //unitConvert(moveC_next, next_pose1, next_pose2, next_param); 
                //arm_group_->MoveJ(target_jnts,target_param.vel_max, target_param.acc_max,\
                        //target_param.smooth, next_pose1, next_pose2, next_param.vel_max, \
                        //next_param.acc_max, next_param.smooth, target_inst.id, result);
            //}

            //break;
        //}//end case motion_spec_MOTIONTYPE_JOINTMOTION:     
        //case motion_spec_MOTIONTYPE_CARTMOTION:
        //{
            //motion_spec_MoveL* moveL = &target_inst.command_arguments.movel;
            //moveL->waypoints[0].blendInDistance = target_inst.smoothDistance; //first rewrite this value
            ////PoseEuler start_pose = getStartPose();
            //PoseEuler target_pose;
            //MoveLParam target_param;
            //unitConvert(moveL, target_pose, target_param);
            //FST_INFO("target pose:%f,%f,%f,%f,%f,%f,smooth:%f", target_pose.position.x,target_pose.position.y,target_pose.position.z, target_pose.orientation.a, target_pose.orientation.b, target_pose.orientation.c, target_param.smooth);
            //if (next_inst.commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)
            //{
                //motion_spec_MoveJ* moveJ_next =  &next_inst.command_arguments.movej;
                //Joint next_jnts;
                //MoveJParam next_param;
                //unitConvert(moveJ_next, next_jnts, next_param);

                //arm_group_->MoveL(target_pose,target_param.vel_max, target_param.acc_max,\
                        //target_param.smooth, next_jnts, next_param.vel_max, next_param.acc_max,\
                        //next_param.smooth, target_inst.id, result);
            //}
            //else if (next_inst.commandtype == motion_spec_MOTIONTYPE_CARTMOTION)
            //{
                //motion_spec_MoveL* moveL_next =  &next_inst.command_arguments.movel;
                //PoseEuler next_pose;
                //MoveLParam next_param;
                //unitConvert(moveL_next, next_pose, next_param);
                ////printDbLine("next pose:", (double*)&next_pose, 6);
                //arm_group_->MoveL(target_pose,target_param.vel_max, target_param.acc_max,\
                        //target_param.smooth, next_pose, next_param.vel_max, next_param.acc_max,\
                        //next_param.smooth, target_inst.id, result);
            //}  
            //else if (next_inst.commandtype == motion_spec_MOTIONTYPE_CIRCLEMOTION)
            //{
                //motion_spec_MoveC *moveC_next = &next_inst.command_arguments.movec;
                //PoseEuler next_pose1, next_pose2;
                //MoveCParam next_param;
                //unitConvert(moveC_next, next_pose1, next_pose2, next_param); 
                //arm_group_->MoveL(target_pose,target_param.vel_max, target_param.acc_max,\
                        //target_param.smooth,next_pose1, next_pose2, next_param.vel_max, \
                        //next_param.acc_max, next_param.smooth, target_inst.id, result);
            //}
            //break;
        //}//end case motion_spec_MOTIONTYPE_CARTMOTION:
        //case motion_spec_MOTIONTYPE_CIRCLEMOTION:
        //{
            //motion_spec_MoveC *moveC = &target_inst.command_arguments.movec;
            //PoseEuler target_pose1, target_pose2;
            //MoveCParam target_param;
            //unitConvert(moveC, target_pose1, target_pose2, target_param); 
            //printDbLine("movec pose2:", (double*)&target_pose2, 6);
            
            //if (next_inst.commandtype == motion_spec_MOTIONTYPE_JOINTMOTION)
            //{
                //motion_spec_MoveJ* moveJ_next =  &next_inst.command_arguments.movej;
                //Joint next_jnts;
                //MoveJParam next_param;
                //unitConvert(moveJ_next, next_jnts, next_param);
                //arm_group_->MoveC(target_pose1, target_pose2, target_param.vel_max, \
                        //target_param.acc_max, target_param.smooth, next_jnts, next_param.vel_max,\
                        //next_param.acc_max, next_param.smooth, target_inst.id, result);             
            //}
            //else if (next_inst.commandtype == motion_spec_MOTIONTYPE_CARTMOTION)
            //{
                //motion_spec_MoveL* moveL_next =  &next_inst.command_arguments.movel;
                //PoseEuler next_pose;
                //MoveLParam next_param;
                //unitConvert(moveL_next, next_pose, next_param);
                //arm_group_->MoveC(target_pose1, target_pose2, target_param.vel_max, \
                        //target_param.acc_max, target_param.smooth, next_pose, next_param.vel_max,\
                        //next_param.acc_max, next_param.smooth, target_inst.id, result);
            //}  
            //else if (next_inst.commandtype == motion_spec_MOTIONTYPE_CIRCLEMOTION)
            //{
                //motion_spec_MoveC *moveC_next = &next_inst.command_arguments.movec;
                //PoseEuler next_pose1, next_pose2;
                //MoveCParam next_param;
                //unitConvert(moveC_next, next_pose1, next_pose2, next_param); 
                //arm_group_->MoveC(target_pose1, target_pose2, target_param.vel_max, \
                        //target_param.acc_max, target_param.smooth, next_pose1, next_pose2, \
                        //next_param.vel_max, next_param.acc_max, next_param.smooth, target_inst.id, result);
            //}
            //break;
        //}//end case motion_spec_MOTIONTYPE_CIRCLEMOTION:
        //default:
            //break;
    /*}//end switch*/


    return result;
}
#endif

/**
 * @brief: convert unit from params of TP to controller
 *
 * @param src_moveL: input==>struct  covert from
 * @param target: output==>struct covert to
 */
#if 0
void InstructionParser::unitConvert(const motion_spec_MoveL *src_moveL, MotionTarget &target)
{
    target.type = MOTION_LINE;
    
    target.linear_velocity = src_moveL->vMax*1000; //m->mm
    //!!!!need to confirm if acc is needed!!!!

	motion_spec_WayPoint way_point = src_moveL->waypoints[0];
	motion_spec_Pose pose = way_point.pose;
	target.pose_target.position.x = pose.coordinates[0] * 1000;
	target.pose_target.position.y = pose.coordinates[1] * 1000;
	target.pose_target.position.z = pose.coordinates[2] * 1000;
	target.pose_target.orientation.a = pose.coordinates[3];
	target.pose_target.orientation.b = pose.coordinates[4];
	target.pose_target.orientation.c = pose.coordinates[5];
}
/**
 * @brief convert unit from params of TP to controller
 *
 * @param src_moveJ: input==>struct  covert from
 * @param target: output==>struct covert to
 */
void InstructionParser::unitConvert(const motion_spec_MoveJ *src_moveJ, MotionTarget &target)
{
    target.type = MOTION_JOINT;
    target.percent_velocity = src_moveJ->vMax; 
    //!!!!need to confirm if acc is needed!!!!

	target.joint_target.j1 = src_moveJ->targetJointCoordinates[0];
	target.joint_target.j2 = src_moveJ->targetJointCoordinates[1];
	target.joint_target.j3 = src_moveJ->targetJointCoordinates[2];
	target.joint_target.j4 = src_moveJ->targetJointCoordinates[3];
	target.joint_target.j5 = src_moveJ->targetJointCoordinates[4];
	target.joint_target.j6 = src_moveJ->targetJointCoordinates[5];
}

void InstructionParser::unitConvert(const motion_spec_MoveC *moveC, MotionTarget &target)
{
    target.type = MOTION_CIRCLE;
    target.linear_velocity = moveC->vMax*1000; //m->mm

    target.circle_target.pose1.position.x = moveC->pose1.coordinates[0] * 1000;
    target.circle_target.pose1.position.y = moveC->pose1.coordinates[1] * 1000;
    target.circle_target.pose1.position.z = moveC->pose1.coordinates[2] * 1000;
    target.circle_target.pose1.orientation.a = moveC->pose1.coordinates[3];
    target.circle_target.pose1.orientation.b = moveC->pose1.coordinates[4];
    target.circle_target.pose1.orientation.c = moveC->pose1.coordinates[5];

    target.circle_target.pose2.position.x = moveC->pose2.coordinates[0] * 1000;
    target.circle_target.pose2.position.y = moveC->pose2.coordinates[1] * 1000;
    target.circle_target.pose2.position.z = moveC->pose2.coordinates[2] * 1000;
    target.circle_target.pose2.orientation.a = moveC->pose2.coordinates[3];
    target.circle_target.pose2.orientation.b = moveC->pose2.coordinates[4];
    target.circle_target.pose2.orientation.c = moveC->pose2.coordinates[5];
}

#endif
