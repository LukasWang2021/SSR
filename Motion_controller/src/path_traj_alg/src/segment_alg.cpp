#include "segment_alg.h"
#include <iostream>

using namespace std;
using namespace fst_mc;
using namespace basic_alg;

ComplexAxisGroupModel model;
double stack[15000];
SegmentAlgParam segment_alg_param;


void initComplexAxisGroupModel()
{
    model.robot_name = "rt-man";
    model.dh_type = DH_TYPE_STANDARD;
    model.cart_vm = 40000;
    model.cart_am = 400000;
    model.cart_jm = 4000000;
    model.link_num = 6;

    // link 0
    model.link[0].joint_type = JOINT_TYPE_ROTATE;
    model.link[0].link_kinematic.theta = 0;
    model.link[0].link_kinematic.d = 0.365;
    model.link[0].link_kinematic.alpha = M_PI/2;
    model.link[0].link_kinematic.a = 0.03;
    model.link[0].link_kinematic.theta_offset = 0;
    model.link[0].link_kinematic.d_offset = 0;
    model.link[0].link_kinematic.alpha_offset = 0;
    model.link[0].link_kinematic.a_offset = 0;
    model.link[0].link_kinematic.qlim[0] = -3.00197;
    model.link[0].link_kinematic.qlim[1] = 3.00197;
    model.link[0].link_kinematic.is_flip = false;
    model.link[0].motor_dynamic.vm = 4500;
    model.link[0].motor_dynamic.jm = 5 * 0.5 * 10000 / 1.3;
    model.link[0].motor_dynamic.gear = 81;

    // link 1
    model.link[1].joint_type = JOINT_TYPE_ROTATE;
    model.link[1].link_kinematic.theta = M_PI/2;
    model.link[1].link_kinematic.d = 0;
    model.link[1].link_kinematic.alpha = 0;
    model.link[1].link_kinematic.a = 0.3402;
    model.link[1].link_kinematic.theta_offset = 0;
    model.link[1].link_kinematic.d_offset = 0;
    model.link[1].link_kinematic.alpha_offset = 0;
    model.link[1].link_kinematic.a_offset = 0;
    model.link[1].link_kinematic.qlim[0] = -1.79769;
    model.link[1].link_kinematic.qlim[1] = 2.39110;
    model.link[1].link_kinematic.is_flip = false;
    model.link[1].motor_dynamic.vm = 4500;
    model.link[1].motor_dynamic.jm = 3.3 * 0.4 * 10000 / 0.44;
    model.link[1].motor_dynamic.gear = 100.908375;

    // link 2
    model.link[2].joint_type = JOINT_TYPE_ROTATE;
    model.link[2].link_kinematic.theta = 0;
    model.link[2].link_kinematic.d =  -0.0001952;
    model.link[2].link_kinematic.alpha = M_PI/2;
    model.link[2].link_kinematic.a = -0.03485;
    model.link[2].link_kinematic.theta_offset = 0;
    model.link[2].link_kinematic.d_offset = 0;
    model.link[2].link_kinematic.alpha_offset = 0;
    model.link[2].link_kinematic.a_offset = 0;
    model.link[2].link_kinematic.qlim[0] = -3.49066;
    model.link[2].link_kinematic.qlim[1] = 1.22173;
    model.link[2].link_kinematic.is_flip = false;
    model.link[2].motor_dynamic.vm = 4500;
    model.link[2].motor_dynamic.jm = 3.3 * 0.4 * 10000 / 0.44;
    model.link[2].motor_dynamic.gear = 81.053333;

    // link 3
    model.link[3].joint_type = JOINT_TYPE_ROTATE;
    model.link[3].link_kinematic.theta = 0;
    model.link[3].link_kinematic.d = 0.3503;
    model.link[3].link_kinematic.alpha = -M_PI/2;
    model.link[3].link_kinematic.a = 0;
    model.link[3].link_kinematic.theta_offset = 0;
    model.link[3].link_kinematic.d_offset = 0;
    model.link[3].link_kinematic.alpha_offset = 0;
    model.link[3].link_kinematic.a_offset = 0;
    model.link[3].link_kinematic.qlim[0] = -3.31613;
    model.link[3].link_kinematic.qlim[1] = 3.31613;
    model.link[3].link_kinematic.is_flip = false;
    model.link[3].motor_dynamic.vm = 4500;
    model.link[3].motor_dynamic.jm = 1.7 * 0.39 * 10000 / 0.18;
    model.link[3].motor_dynamic.gear = 59.987882;

    // link 4
    model.link[4].joint_type = JOINT_TYPE_ROTATE;
    model.link[4].link_kinematic.theta = 0;
    model.link[4].link_kinematic.d = 0;
    model.link[4].link_kinematic.alpha = M_PI/2;
    model.link[4].link_kinematic.a = 0;
    model.link[4].link_kinematic.theta_offset = 0;
    model.link[4].link_kinematic.d_offset = 0;
    model.link[4].link_kinematic.alpha_offset = 0;
    model.link[4].link_kinematic.a_offset = 0;
    model.link[4].link_kinematic.qlim[0] = -2.02458;
    model.link[4].link_kinematic.qlim[1] = 2.02458;
    model.link[4].link_kinematic.is_flip = false;
    model.link[4].motor_dynamic.vm = 4500;
    model.link[4].motor_dynamic.jm = 1.7 * 0.25 * 10000 / 0.17;
    model.link[4].motor_dynamic.gear = 66.75495;

    // link 5
    model.link[5].joint_type = JOINT_TYPE_ROTATE;
    model.link[5].link_kinematic.theta = 0;
    model.link[5].link_kinematic.d = 0.0965;
    model.link[5].link_kinematic.alpha = 0;
    model.link[5].link_kinematic.a = 0;
    model.link[5].link_kinematic.theta_offset = 0;
    model.link[5].link_kinematic.d_offset = 0;
    model.link[5].link_kinematic.alpha_offset = 0;
    model.link[5].link_kinematic.a_offset = 0;
    model.link[5].link_kinematic.qlim[0] = -6.28319;
    model.link[5].link_kinematic.qlim[1] = 6.28319;
    model.link[5].link_kinematic.is_flip = false;    
    model.link[5].motor_dynamic.vm = 4500;
    model.link[5].motor_dynamic.jm = 1.7 * 0.25 * 10000 / 0.17;
    model.link[5].motor_dynamic.gear = 44.671266;
}

void initStack(ComplexAxisGroupModel* model_ptr)
{
    // robot model related
    for(int i=0; i<model_ptr->link_num; ++i)
    {
        stack[S_RealTheta+i] = model_ptr->link[i].link_kinematic.theta + model_ptr->link[i].link_kinematic.theta_offset;
        stack[S_RealD+i] = model_ptr->link[i].link_kinematic.d + model_ptr->link[i].link_kinematic.d_offset;
        stack[S_RealAlpha+i] = model_ptr->link[i].link_kinematic.alpha + model_ptr->link[i].link_kinematic.alpha_offset;
        stack[S_RealA+i] = model_ptr->link[i].link_kinematic.a + model_ptr->link[i].link_kinematic.a_offset;
        stack[S_ConstraintJointVelMax + i] = model_ptr->link[i].motor_dynamic.vm * PI * 2 / (60 * model_ptr->link[i].motor_dynamic.gear);
    }

    // S_TransMatrix
    memset(&stack[S_TransMatrix], 0, sizeof(double) * 16);
    stack[S_TransMatrix + 15] = 1;

    // S_HomoTransMatrix
    memset(&stack[S_HomoTransMatrix], 0, sizeof(double) * 16);
    stack[S_HomoTransMatrix + 15] = 1;

    // S_NodeVector
    stack[S_BSplineNodeVector] = 0;
    stack[S_BSplineNodeVector + 1] = 0;
    stack[S_BSplineNodeVector + 2] = 0;
    stack[S_BSplineNodeVector + 3] = 1;
    stack[S_BSplineNodeVector + 4] = 1;
    stack[S_BSplineNodeVector + 5] = 1;

    // path count factor
    stack[S_PathCountFactorCartesian] = segment_alg_param.accuracy_cartesian_factor / 100.0;
    stack[S_PathCountFactorJoint] = segment_alg_param.accuracy_joint_factor / PI;

    // init start and end point vel and acc state
    int start_point_address = S_StartPointState0;
    int end_point_address = S_EndPointState0;
    for(int i = 0; i<model_ptr->link_num; ++i)
    {
        stack[start_point_address + 1] = 0; // start vel always zero
        stack[start_point_address + 2] = 0; // start acc always zero
        stack[end_point_address + 1] = 0;   // end vel always zero
        stack[end_point_address + 2] = 0;   // end acc always zero
        start_point_address += 3;
        end_point_address += 3;
    }
}

void initSegmentAlgParam(SegmentAlgParam* segment_alg_param_ptr)
{
    segment_alg_param.accuracy_cartesian_factor = segment_alg_param_ptr->accuracy_cartesian_factor;
    segment_alg_param.accuracy_joint_factor = segment_alg_param_ptr->accuracy_joint_factor;
    segment_alg_param.max_traj_points_num = segment_alg_param_ptr->max_traj_points_num;
    segment_alg_param.path_interval = segment_alg_param_ptr->path_interval;
    segment_alg_param.joint_interval = segment_alg_param_ptr->joint_interval;
    segment_alg_param.angle_interval = segment_alg_param_ptr->angle_interval;
    segment_alg_param.angle_valve = segment_alg_param_ptr->angle_valve;
    segment_alg_param.conservative_acc = segment_alg_param_ptr->conservative_acc;
    segment_alg_param.time_factor_first = segment_alg_param_ptr->time_factor_first;
    segment_alg_param.time_factor_last = segment_alg_param_ptr->time_factor_last;
    segment_alg_param.is_fake_dynamics = segment_alg_param_ptr->is_fake_dynamics;
    segment_alg_param.kinematics_ptr = segment_alg_param_ptr->kinematics_ptr;
    segment_alg_param.dynamics_ptr = segment_alg_param_ptr->dynamics_ptr;
    segment_alg_param.max_cartesian_acc = segment_alg_param_ptr->max_cartesian_acc;

    initStack(&model);
}

ErrorCode planPathJoint(const Joint &start, 
                            const MotionTarget &end, 
                            PathCache &path_cache)
{
    int i, j;
    // find max delta joint 
    double delta_joint_start2end;
    double max_delta_joint_start2end = 0;
    for(i = 0; i < model.link_num; ++i)
    {
        delta_joint_start2end = fabs(end.joint_target[i] - start[i]);
        if(delta_joint_start2end > max_delta_joint_start2end)
        {
            max_delta_joint_start2end = delta_joint_start2end;
        }
    }
    if(max_delta_joint_start2end < DOUBLE_ACCURACY)
    {
        return PATH_PLANNING_INVALID_TARGET;
    }

    // init unused data
    path_cache.smooth_in_index = -1;
    // compute interpolation points
    int path_count_minus_1 = ceil(max_delta_joint_start2end / segment_alg_param.joint_interval);
    path_cache.cache_length = path_count_minus_1 + 1;
  
    double joint_step_start2end, joint_distance_to_start;
    
    for(i = 0; i < model.link_num; ++i)
    {    
        path_cache.cache[0].joint[i] = start[i];
        packPathBlockType(PATH_POINT, MOTION_JOINT, path_cache.cache[0]);    
        joint_step_start2end = (end.joint_target[i] - start[i]) / path_count_minus_1;
        joint_distance_to_start = 0; 
        for(j = 1; j < path_count_minus_1; ++j)
        {
            joint_distance_to_start += joint_step_start2end;
            path_cache.cache[j].joint[i] = start[i] + joint_distance_to_start;
            packPathBlockType(PATH_POINT, MOTION_JOINT, path_cache.cache[j]);
        }
        path_cache.cache[path_count_minus_1].joint[i] = end.joint_target[i];
        packPathBlockType(PATH_POINT, MOTION_JOINT, path_cache.cache[path_count_minus_1]);    
    }

    if(end.cnt > DOUBLE_ACCURACY)
    {
        path_cache.smooth_out_index = path_count_minus_1 - ceil(path_cache.cache_length * end.cnt / 2.0);
    }
    else
    {
        path_cache.smooth_out_index = -1;
    }

    return SUCCESS;
}

ErrorCode planPathLine(const PoseEuler &start, 
                            const MotionTarget &end, 
                            PathCache &path_cache)
{
    int i;
    
    // init unused data
    path_cache.smooth_in_index = -1;
    // compute MoveL length
    double path_length_start2end;
    double path_vector_start2end[3];
    getMoveLPathVector(start.point_, end.pose_target.point_, path_vector_start2end, path_length_start2end);   // MoveL length
    if(path_length_start2end < DOUBLE_ACCURACY)
    {
        return PATH_PLANNING_INVALID_TARGET;
    }
    
    // compute MoveL quatern angle
    double start_quatern[4], end_quatern[4];
    getMoveEulerToQuatern(start.euler_, start_quatern);
    getMoveEulerToQuatern(end.pose_target.euler_, end_quatern);
    double angle_start2end = getQuaternsIntersectionAngle(start_quatern, end_quatern);    // MoveL quatern angle

    int path_count_ideal_start2end = ceil(path_length_start2end / segment_alg_param.path_interval);
    int angle_count_ideal_start2end = ceil(angle_start2end / segment_alg_param.angle_interval);
    int max_count_start2end = ((path_count_ideal_start2end >= angle_count_ideal_start2end) ? path_count_ideal_start2end : angle_count_ideal_start2end);

    // find Pout distance to end point
    double point_distance_to_start = 0; // not scaled
    double angle_distance_to_start = 0; // scale to [0,1]
    if(end.cnt >= DOUBLE_ACCURACY)    // cnt is valid
    {
        double path_out_vel = end.vel * end.cnt;
        double max_path_length_out2end = path_length_start2end / 2;
        double path_length_out2end = path_out_vel * path_out_vel / (2 * segment_alg_param.conservative_acc);        
        if(path_length_out2end > max_path_length_out2end)
        {
            path_length_out2end = max_path_length_out2end;
        }     
        
        int path_count_out2end = ceil(path_length_out2end * max_count_start2end / path_length_start2end);
        double path_step_out2end = path_length_out2end / path_count_out2end;        
        
        double path_length_start2out = path_length_start2end - path_length_out2end;
        double angle_distance_start2out = path_length_start2out / path_length_start2end;
        int path_count_start2out = ceil(angle_distance_start2out * max_count_start2end);
        double path_step_start2out = path_length_start2out / path_count_start2out;

        double angle_step_start2out = angle_distance_start2out / path_count_start2out;
        double angle_step_out2end = (1.0 - angle_distance_start2out) / path_count_out2end;

        packPoseByPointAndQuatern(start.point_, start_quatern, path_cache.cache[0].pose);
        packPathBlockType(PATH_POINT, MOTION_LINE, path_cache.cache[0]);
        for(i = 1; i <= path_count_start2out; ++i)
        {
            point_distance_to_start += path_step_start2out;
            angle_distance_to_start += angle_step_start2out;            
            getMoveLPathPoint(start.point_, path_vector_start2end, point_distance_to_start, path_cache.cache[i].pose.point_);
            getQuaternPoint(start_quatern, end_quatern, angle_start2end, angle_distance_to_start, path_cache.cache[i].pose.quaternion_);
            packPathBlockType(PATH_POINT, MOTION_LINE, path_cache.cache[i]);
        }
        path_cache.smooth_out_index = path_count_start2out;
        int path_count_total_minus_1 = path_count_start2out + path_count_out2end;
        path_cache.cache_length = path_count_total_minus_1 + 1;
        for(; i < path_count_total_minus_1; ++i)
        {
            point_distance_to_start += path_step_out2end;
            angle_distance_to_start += angle_step_out2end;
            getMoveLPathPoint(start.point_, path_vector_start2end, point_distance_to_start, path_cache.cache[i].pose.point_);
            getQuaternPoint(start_quatern, end_quatern, angle_start2end, angle_distance_to_start, path_cache.cache[i].pose.quaternion_);
            packPathBlockType(PATH_POINT, MOTION_LINE, path_cache.cache[i]);
        }
        packPoseByPointAndQuatern(end.pose_target.point_, end_quatern, path_cache.cache[path_count_total_minus_1].pose);
        packPathBlockType(PATH_POINT, MOTION_LINE, path_cache.cache[i]);
    }
    else    // cnt is invalid
    {    
        if(end.cnt >= -DOUBLE_ACCURACY)  // cnt = 0
        {
            path_cache.smooth_out_index = max_count_start2end;
        }
        else
        {
            path_cache.smooth_out_index = -1;
        }        
        path_cache.cache_length = max_count_start2end + 1;
        double path_step_start2end = path_length_start2end / max_count_start2end;
        double angle_step_start2end = 1.0 / max_count_start2end;
        packPoseByPointAndQuatern(start.point_, start_quatern, path_cache.cache[0].pose);
        packPathBlockType(PATH_POINT, MOTION_LINE, path_cache.cache[0]);         
        for(i = 1; i < max_count_start2end; ++i)
        {
            point_distance_to_start += path_step_start2end;
            angle_distance_to_start += angle_step_start2end;
            getMoveLPathPoint(start.point_, path_vector_start2end, point_distance_to_start, path_cache.cache[i].pose.point_);
            getQuaternPoint(start_quatern, end_quatern, angle_start2end, angle_distance_to_start, path_cache.cache[i].pose.quaternion_);
            packPathBlockType(PATH_POINT, MOTION_LINE, path_cache.cache[i]);
        }
        packPoseByPointAndQuatern(end.pose_target.point_, end_quatern, path_cache.cache[max_count_start2end].pose);
        packPathBlockType(PATH_POINT, MOTION_LINE, path_cache.cache[max_count_start2end]);
    }
    
    return SUCCESS;
}

ErrorCode planPathCircle(const PoseEuler &start, 
                                const MotionTarget &end, 
                                PathCache &path_cache)
{
    return 0;
}

ErrorCode planPathSmoothJoint(const Joint &start, 
                                    const MotionTarget &via, 
                                    const MotionTarget &end, 
                                    PathCache &path_cache)
{
    int i, j;
    // find max delta joint via2end 
    double delta_joint_via2end;
    double max_delta_joint_via2end = 0;
    for(i = 0; i < model.link_num; ++i)
    {
        delta_joint_via2end = fabs(end.joint_target[i] - via.joint_target[i]);
        if(delta_joint_via2end > max_delta_joint_via2end)
        {
            max_delta_joint_via2end = delta_joint_via2end;
        }
    }
    if(max_delta_joint_via2end < DOUBLE_ACCURACY)
    {
        return PATH_PLANNING_INVALID_TARGET;
    }
    // find joint of the in point
    int path_piece_via2end = ceil(max_delta_joint_via2end / segment_alg_param.joint_interval);
    int path_piece_via2in = floor(via.cnt * path_piece_via2end / 2.0);
    int path_piece_in2end = path_piece_via2end - path_piece_via2in;
    double joint_step_via2end;
    Joint joint_in;
    for(i = 0; i < model.link_num; ++i)
    {
        joint_step_via2end = (end.joint_target[i] - via.joint_target[i]) /  path_piece_via2end;
        joint_in[i] = start[i] + joint_step_via2end * path_piece_via2in;
    }

    // compute path start2in
    MotionTarget in;
    in.cnt = -1;
    in.joint_target = joint_in;
    planPathJoint(start, in, path_cache);
    path_cache.smooth_in_index = path_cache.cache_length - 1;

    // find smooth_out_index
    int path_cache_length_minus_1 = path_cache.smooth_in_index + path_piece_in2end;
    path_cache.cache_length = path_cache_length_minus_1 + 1;
    if(end.cnt > DOUBLE_ACCURACY)
    {
        int path_piece_out2end = floor(end.cnt * path_piece_via2end / 2.0);    
        path_cache.smooth_out_index = path_cache_length_minus_1 - path_piece_out2end;
    }
    else
    {
        if(end.cnt > -DOUBLE_ACCURACY)  // end.cnt == 0
        {
            path_cache.smooth_out_index = path_cache_length_minus_1;
        }
        else
        {
            path_cache.smooth_out_index = -1;
        }
    }    

    // compute path in2end
    double joint_distance_to_in;
    for(i = 0; i < model.link_num; ++i)
    {      
        joint_step_via2end = (end.joint_target[i] - joint_in[i]) / path_piece_in2end;
        joint_distance_to_in = 0; 
        for(j = path_cache.smooth_in_index + 1; j < path_cache_length_minus_1; ++j)
        {
            joint_distance_to_in += joint_step_via2end;
            path_cache.cache[j].joint[i] = joint_in[i] + joint_distance_to_in;
            packPathBlockType(PATH_POINT, MOTION_JOINT, path_cache.cache[j]);
        }
        path_cache.cache[path_cache_length_minus_1].joint[i] = end.joint_target[i];
        packPathBlockType(PATH_POINT, MOTION_JOINT, path_cache.cache[path_cache_length_minus_1]);    
    }

    return 0;
}

ErrorCode planPathSmoothLine(const PoseEuler &start, 
                                    const MotionTarget &via, 
                                    const MotionTarget &end, 
                                    PathCache &path_cache)
{
    int i;
    // compute path
    double path_length_start2via = getPointsDistance(start.point_, via.pose_target.point_);   
    double path_length_via2target;
    double path_vector_via2target[3];
    getMoveLPathVector(via.pose_target.point_, end.pose_target.point_, path_vector_via2target, path_length_via2target);
    double path_length_via2in = path_length_via2target / 2;
    
    if(path_length_start2via < path_length_via2in)
    {
        path_length_via2in = path_length_start2via;
    }   
    double path_length_in2target = path_length_via2target - path_length_via2in;
        
    int path_count_ideal_start2via = ceil(path_length_start2via / segment_alg_param.path_interval);
    int path_count_ideal_via2in = ceil(path_length_via2in / segment_alg_param.path_interval);
    Point point_in;
    getMoveLPathPoint(via.pose_target.point_, path_vector_via2target, path_length_via2in, point_in);
    
    // compute quatern
    double quatern_start[4], quatern_via[4], quatern_in[4], quatern_target[4];
    getMoveEulerToQuatern(start.euler_, quatern_start);
    getMoveEulerToQuatern(via.pose_target.euler_, quatern_via);
    getMoveEulerToQuatern(end.pose_target.euler_, quatern_target);
    double angle_start2via = getQuaternsIntersectionAngle(quatern_start, quatern_via);   
    double angle_via2target = getQuaternsIntersectionAngle(quatern_via, quatern_target);
    double angle_count_ideal_start2via = ceil(angle_start2via / segment_alg_param.angle_interval);    
    double angle_count_ideal_via2target = ceil(angle_via2target / segment_alg_param.angle_interval);
    double angle_distance_via2in = path_length_via2in / path_length_via2target; // scale to [0,1]   
    double angle_in2target = (1 - angle_distance_via2in) * angle_via2target;
    int angle_count_ideal_via2in = ceil(angle_distance_via2in * angle_via2target / segment_alg_param.angle_interval);
    getQuaternVector4(quatern_via, quatern_target, angle_via2target, angle_distance_via2in, quatern_in);
    double angle_transition = getQuaternsIntersectionAngle(quatern_start, quatern_in);
    
    // determine path count
    int path_count_start2via = (path_count_ideal_start2via > angle_count_ideal_start2via ? path_count_ideal_start2via : angle_count_ideal_start2via);
    int path_count_via2in = (path_count_ideal_via2in > angle_count_ideal_via2in ? path_count_ideal_via2in : angle_count_ideal_via2in);
    int path_count_transition = path_count_start2via + path_count_via2in;
    path_cache.smooth_in_index = path_count_transition;

    // determine transition angle step
    double angle_step_transition = 1.0 / path_count_transition;

    // determine out point
    double point_distance_to_in = 0; // not scaled
    double angle_distance_to_start = 0; // scale to [0,1]
    double angle_distance_to_in = 0; // scale to [0,1]
    double angle_distance_to_out = 0; // scale to [0,1]
    double start_point[3], via_point[3], in_point[3];
    if(end.cnt >= DOUBLE_ACCURACY)
    {   // FIXME: small cnt will cause pulse in vel and acc of traj
        // compute path in2out and out2target 
        double path_out_vel = end.vel * end.cnt;
        double max_path_length_out2target = path_length_via2target / 2;
        double path_length_out2target = path_out_vel * path_out_vel / (2 * segment_alg_param.conservative_acc);
        if(path_length_out2target > max_path_length_out2target)
        {
            path_length_out2target = max_path_length_out2target;
        }
        
        double path_length_in2out = path_length_in2target - path_length_out2target;        
        Point point_out;
        getMoveLPathPoint(point_in, path_vector_via2target, path_length_in2out, point_out);
        int path_count_ideal_in2out = 0;
        if(path_length_in2out > DOUBLE_ACCURACY)
        {
            path_count_ideal_in2out = ceil(path_length_in2out / segment_alg_param.path_interval);
        }
        int path_count_ideal_out2target = ceil(path_length_out2target / segment_alg_param.path_interval);       
        // compute angle in2out and out2target
        double angle_distance_in2out = path_length_in2out / path_length_in2target;
        double quatern_out[4];
        getQuaternVector4(quatern_in, quatern_target, angle_in2target, angle_distance_in2out, quatern_out);
        double angle_in2out = getQuaternsIntersectionAngle(quatern_in, quatern_out);
        double angle_out2target = angle_in2target - angle_in2out;
        int angle_count_ideal_in2out = 0;
        if(angle_in2out > DOUBLE_ACCURACY)
        {
            angle_count_ideal_in2out = ceil(angle_in2out / segment_alg_param.angle_interval);
        }
        int angle_count_ideal_out2target = ceil(angle_out2target / segment_alg_param.angle_interval);

        // determine path count
        int path_count_in2out = (path_count_ideal_in2out > angle_count_ideal_in2out ? path_count_ideal_in2out : angle_count_ideal_in2out);
        int path_count_out2target = (path_count_ideal_out2target > angle_count_ideal_out2target ? path_count_ideal_out2target : angle_count_ideal_out2target);
        path_cache.smooth_out_index = path_cache.smooth_in_index + path_count_in2out;
        int path_cache_length_minus_1 = path_count_transition + path_count_in2out + path_count_out2target;
        path_cache.cache_length = path_cache_length_minus_1 + 1;

        // determine step
        double path_step_in2out = path_length_in2out / path_count_in2out;
        double path_step_out2target = path_length_out2target / path_count_out2target;
        double angle_step_in2out = 1.0 / path_count_in2out;
        double angle_step_out2target = 1.0 / path_count_out2target;

        // compute transition path
        getMovePointToVector3(start.point_, start_point);
        getMovePointToVector3(via.pose_target.point_, via_point);
        getMovePointToVector3(point_in, in_point);
        updateTransitionBSpLineResult(2, start_point, via_point, in_point, path_cache.smooth_in_index);

        packPoseByPointAndQuatern(start.point_, quatern_start, path_cache.cache[0].pose);
        packPathBlockType(PATH_POINT, MOTION_LINE, path_cache.cache[0]);        
        for(i = 1; i < path_cache.smooth_in_index; ++i)
        {
            path_cache.cache[i].pose.point_.x_ = stack[S_BSpLineResultXBase + i];
            path_cache.cache[i].pose.point_.y_ = stack[S_BSpLineResultYBase + i];
            path_cache.cache[i].pose.point_.z_ = stack[S_BSpLineResultZBase + i];
            angle_distance_to_start += angle_step_transition;
            getQuaternPoint(quatern_start, quatern_in, angle_transition, angle_distance_to_start, path_cache.cache[i].pose.quaternion_);
            packPathBlockType(TRANSITION_POINT, MOTION_LINE, path_cache.cache[i]);
        }
        packPoseByPointAndQuatern(point_in, quatern_in, path_cache.cache[path_cache.smooth_in_index].pose);
        packPathBlockType(TRANSITION_POINT, MOTION_LINE, path_cache.cache[path_cache.smooth_in_index]);  

        // compute in2out path
        for(i = path_cache.smooth_in_index + 1; i < path_cache.smooth_out_index; ++i)
        {
            point_distance_to_in += path_step_in2out;
            angle_distance_to_in += angle_step_in2out;
            getMoveLPathPoint(point_in, path_vector_via2target, point_distance_to_in, path_cache.cache[i].pose.point_);
            getQuaternPoint(quatern_in, quatern_out, angle_in2out, angle_distance_to_in, path_cache.cache[i].pose.quaternion_);
            packPathBlockType(PATH_POINT, MOTION_LINE, path_cache.cache[i]);
        }
        packPoseByPointAndQuatern(point_out, quatern_out, path_cache.cache[path_cache.smooth_out_index].pose);
        packPathBlockType(PATH_POINT, MOTION_LINE, path_cache.cache[path_cache.smooth_out_index]);

        // compute out2target path
        for(i = path_cache.smooth_out_index + 1; i < path_cache_length_minus_1; ++i)
        {
            point_distance_to_in += path_step_out2target;
            angle_distance_to_out += angle_step_out2target;
            getMoveLPathPoint(point_in, path_vector_via2target, point_distance_to_in, path_cache.cache[i].pose.point_);
            getQuaternPoint(quatern_out, quatern_target, angle_out2target, angle_distance_to_out, path_cache.cache[i].pose.quaternion_);
            packPathBlockType(PATH_POINT, MOTION_LINE, path_cache.cache[i]);
        }
        packPoseByPointAndQuatern(end.pose_target.point_, quatern_target, path_cache.cache[path_cache_length_minus_1].pose);
        packPathBlockType(PATH_POINT, MOTION_LINE, path_cache.cache[path_cache_length_minus_1]);
    }
    else
    {       
        // compute in2target settings
        int path_count_ideal_in2target = ceil(path_length_in2target / segment_alg_param.path_interval);
        int angle_count_ideal_in2target = ceil(angle_in2target / segment_alg_param.angle_interval);
        int path_count_in2target = (path_count_ideal_in2target > angle_count_ideal_in2target ? path_count_ideal_in2target : angle_count_ideal_in2target);
        int path_cache_length_minus_1 = path_count_transition + path_count_in2target;
        path_cache.cache_length = path_cache_length_minus_1 + 1;       
        double path_step_in2target = path_length_in2target / path_count_in2target;
        double angle_step_in2target = 1.0 / path_count_in2target;
        if(end.cnt >= -DOUBLE_ACCURACY)  // cnt = 0
        {
            path_cache.smooth_out_index = path_cache_length_minus_1;
        }
        else
        {
            path_cache.smooth_out_index = -1;
        }  

        // compute transition path
        getMovePointToVector3(start.point_, start_point);
        getMovePointToVector3(via.pose_target.point_, via_point);
        getMovePointToVector3(point_in, in_point);
        updateTransitionBSpLineResult(2, start_point, via_point, in_point, path_cache.smooth_in_index);

        packPoseByPointAndQuatern(start.point_, quatern_start, path_cache.cache[0].pose);
        packPathBlockType(PATH_POINT, MOTION_LINE, path_cache.cache[0]);        
        for(i = 1; i < path_cache.smooth_in_index; ++i)
        {
            path_cache.cache[i].pose.point_.x_ = stack[S_BSpLineResultXBase + i];
            path_cache.cache[i].pose.point_.y_ = stack[S_BSpLineResultYBase + i];
            path_cache.cache[i].pose.point_.z_ = stack[S_BSpLineResultZBase + i];
            angle_distance_to_start += angle_step_transition;
            getQuaternPoint(quatern_start, quatern_in, angle_transition, angle_distance_to_start, path_cache.cache[i].pose.quaternion_);
            packPathBlockType(TRANSITION_POINT, MOTION_LINE, path_cache.cache[i]);
        }
        packPoseByPointAndQuatern(point_in, quatern_in, path_cache.cache[path_cache.smooth_in_index].pose);
        packPathBlockType(TRANSITION_POINT, MOTION_LINE, path_cache.cache[path_cache.smooth_in_index]);          

        // compute in2target path
        for(i = path_cache.smooth_in_index + 1; i < path_cache_length_minus_1; ++i)
        {
            point_distance_to_in += path_step_in2target;
            angle_distance_to_in += angle_step_in2target;
            getMoveLPathPoint(point_in, path_vector_via2target, point_distance_to_in, path_cache.cache[i].pose.point_);
            getQuaternPoint(quatern_in, quatern_target, angle_in2target, angle_distance_to_in, path_cache.cache[i].pose.quaternion_);
            packPathBlockType(PATH_POINT, MOTION_LINE, path_cache.cache[i]);
        }
        packPoseByPointAndQuatern(end.pose_target.point_, quatern_target, path_cache.cache[path_cache_length_minus_1].pose);
        packPathBlockType(PATH_POINT, MOTION_LINE, path_cache.cache[path_cache_length_minus_1]);
    }

    return SUCCESS;
}

ErrorCode planPathSmoothCircle(const PoseEuler &start, 
                                        const MotionTarget &via, 
                                        const MotionTarget &end, 
                                        PathCache &path_cache)
{
    return 0;
}

ErrorCode planTrajectory(const PathCache &path_cache, 
                                const JointState &start_state, 
                                double vel_ratio, 
                                double acc_ratio, 
                                TrajectoryCache &traj_cache)
{
    if(path_cache.cache_length < 4)
    {
        return TRAJ_PLANNING_INVALID_PATHCACHE;
    }

    int traj_pva_size, traj_pva_out_index, traj_t_size;
    double cmd_vel = path_cache.target.vel * vel_ratio;   // command velocity
    int traj_path_cache_index[25];
    switch(path_cache.target.type)
    {
        case MOTION_LINE:
        {
            updateMovLTrajP(path_cache, traj_path_cache_index, traj_pva_out_index, traj_pva_size);
            updateMovLTrajT(path_cache, cmd_vel, traj_path_cache_index, traj_pva_out_index, traj_pva_size, traj_t_size);
            break;
        }
        case MOTION_JOINT:
        {
            updateMovJTrajP(path_cache, traj_path_cache_index, traj_pva_out_index, traj_pva_size);
            //std::cout<<"traj_pva_out_index = "<<traj_pva_out_index<<std::endl;
            updateMovJTrajT(path_cache, cmd_vel, traj_path_cache_index, traj_pva_out_index, traj_pva_size, traj_t_size);
            break;
        }
        default:
        {       
            return TRAJ_PLANNING_INVALID_MOTION_TYPE;
        }
    }
    // it is not necessary to initialize the position of S_StartPointState and S_EndPointState,
    // because they are not used in updateTrajPVA.
    updateTrajPVA(S_TrajP0, S_TrajV0, S_TrajA0, traj_pva_size, S_TrajJ0,
                  &stack[S_TrajT], traj_t_size, S_StartPointState0, S_EndPointState0);    
    updateConstraintJoint(S_TrajP0, S_TrajV0, traj_pva_size);
    updateTrajPieceA(S_TrajA0, traj_pva_size, acc_ratio);       
    updateTrajPieceV(S_TrajV0, S_TrajA0, traj_pva_size, S_TrajT, vel_ratio);
    updateTrajPieceRescaleFactor(traj_t_size);
    
    if(isRescaleNeeded(traj_t_size))
    {
        updateTrajTByPieceRescaleFactor(S_TrajT, traj_t_size);
        updateTrajPVA(S_TrajP0, S_TrajV0, S_TrajA0, traj_pva_size, S_TrajJ0,
                      &stack[S_TrajT], traj_t_size, S_StartPointState0, S_EndPointState0);
    }
    updateTrajCoeff(S_TrajP0, S_TrajV0, S_TrajA0, traj_pva_size, S_TrajT, traj_t_size, S_TrajJ0, S_TrajCoeffJ0A0);
    packTrajCache(traj_path_cache_index, traj_pva_out_index, traj_pva_size, S_TrajCoeffJ0A0, S_TrajT, traj_t_size, traj_cache);  
    return SUCCESS;
}
                                
ErrorCode planTrajectorySmooth(const PathCache &path_cache, 
                                        const JointState &start_state, 
                                        const MotionTarget &via, 
                                        double vel_ratio, 
                                        double acc_ratio, 
                                        TrajectoryCache &traj_cache)
{
    if(path_cache.cache_length < 4)
    {
        return TRAJ_PLANNING_INVALID_PATHCACHE;
    }
    
    if(path_cache.smooth_in_index == -1)
    {
        return TRAJ_PLANNING_INVALID_SMOOTH_IN_INDEX;
    }

    double cmd_vel = path_cache.target.vel * vel_ratio;
    int traj_pva_in_index, traj_pva_out_index, traj_pva_size_via2end, traj_t_size_via2end;
    int traj_path_cache_index_in2end[25];
    switch(path_cache.target.type)
    {
        case MOTION_LINE:
        {
            updateMovLVia2InTrajP(path_cache, via, traj_pva_in_index);
            updateMovLIn2EndTrajP(path_cache, traj_pva_in_index, traj_path_cache_index_in2end, traj_pva_out_index, traj_pva_size_via2end);
            updateMovLVia2EndTrajT(path_cache, via, cmd_vel, traj_path_cache_index_in2end, traj_pva_in_index, traj_pva_out_index, traj_pva_size_via2end, traj_t_size_via2end);
            break;
        }
        case MOTION_JOINT:
        {
            updateMovJVia2InTrajP(path_cache, via, traj_pva_in_index);
            updateMovJIn2EndTrajP(path_cache, traj_pva_in_index, traj_path_cache_index_in2end, traj_pva_out_index, traj_pva_size_via2end);
            updateMovJVia2EndTrajT(path_cache, via, cmd_vel, traj_path_cache_index_in2end, traj_pva_in_index, traj_pva_out_index, traj_pva_size_via2end, traj_t_size_via2end);
            break;
        }
        default:
        {
            return TRAJ_PLANNING_INVALID_MOTION_TYPE;
        }
    }

    // it is not necessary to initialize the position of S_StartPointState and S_EndPointState,
    // because they are not used in updateTrajPVA.    
    updateTrajPVA(S_TrajP0, S_TrajV0, S_TrajA0, traj_pva_size_via2end, S_TrajJ0,
                  &stack[S_TrajT], traj_t_size_via2end, S_StartPointState0, S_EndPointState0); 
   
    updateConstraintJoint(S_TrajP0, S_TrajV0, traj_t_size_via2end);
    updateTrajPieceA(S_TrajA0, traj_t_size_via2end, acc_ratio);       
    updateTrajPieceV(S_TrajV0, S_TrajA0, traj_t_size_via2end, S_TrajT, vel_ratio);
    updateTrajPieceRescaleFactor(traj_t_size_via2end);

    if(isRescaleNeeded(traj_t_size_via2end))
    {
        updateTrajTByPieceRescaleFactor(S_TrajT, traj_t_size_via2end);
        updateTrajPVA(S_TrajP0, S_TrajV0, S_TrajA0, traj_pva_size_via2end, S_TrajJ0,
                      &stack[S_TrajT], traj_t_size_via2end, S_StartPointState0, S_EndPointState0);
    }
    updateTrajCoeff(S_TrajP0, S_TrajV0, S_TrajA0, traj_pva_size_via2end, S_TrajT, traj_t_size_via2end, S_TrajJ0, S_TrajCoeffJ0A0);

    int traj_pva_size_out2in, traj_t_size_out2in;
    int traj_path_cache_index_out2in[25];
    updateSmoothOut2InTrajP(path_cache, via, traj_path_cache_index_out2in, traj_pva_size_out2in);
    updateSmoothOut2InTrajT(path_cache, via, cmd_vel, traj_path_cache_index_out2in, traj_pva_size_out2in, traj_t_size_out2in);
    updateOutAndInPointState(start_state, traj_pva_in_index);
/*for(int i=0; i<traj_pva_size_out2in; ++i)
{
    std::cout<<i<<" "<<stack[S_TrajP0_Smooth + i]<<" "<<stack[S_TrajP1_Smooth + i]<<" "<<stack[S_TrajT_Smooth + i]<<std::endl;
} */   
    updateTrajPVA(S_TrajP0_Smooth, S_TrajV0_Smooth, S_TrajA0_Smooth, traj_pva_size_out2in, S_TrajJ0,
                  &stack[S_TrajT_Smooth], traj_t_size_out2in, S_OutPointState0, S_InPointState0);

    updateConstraintJoint(S_TrajP0_Smooth, S_TrajV0_Smooth, traj_t_size_out2in);
    updateTrajPieceA(S_TrajA0_Smooth, traj_pva_size_out2in, acc_ratio);
    updateTrajPieceRescaleFactor(traj_t_size_out2in);

    if(isRescaleNeeded(traj_t_size_out2in))
    {
        updateTrajTByPieceRescaleFactor(S_TrajT_Smooth, traj_t_size_out2in);
        updateTrajPVA(S_TrajP0_Smooth, S_TrajV0_Smooth, S_TrajA0_Smooth, traj_pva_size_out2in, S_TrajJ0,
                      &stack[S_TrajT_Smooth], traj_t_size_out2in, S_OutPointState0, S_InPointState0);
    }    

    updateTrajCoeff(S_TrajP0_Smooth, S_TrajV0_Smooth, S_TrajA0_Smooth, traj_pva_size_out2in, S_TrajT_Smooth, traj_t_size_out2in, S_TrajJ0, S_TrajCoeffJ0A0_Smooth);
    packTrajCacheSmooth(traj_path_cache_index_out2in, traj_pva_size_out2in, S_TrajCoeffJ0A0_Smooth, S_TrajT_Smooth, traj_t_size_out2in, 
                        traj_path_cache_index_in2end, traj_pva_size_via2end, S_TrajCoeffJ0A0, S_TrajT, traj_t_size_via2end,
                        traj_pva_in_index, traj_pva_out_index,
                        traj_cache);
    
    return SUCCESS;
}

ErrorCode planPauseTrajectory(const PathCache &path_cache, 
                                    const JointState &start_state, 
                                    double acc_ratio, 
                                    TrajectoryCache &traj_cache, 
                                    int &path_stop_index)
{
    return 0;
}

inline double getVector3Norm(double* vector)
{
    return sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
}

inline double getVector4Norm(double* vector)
{
    return sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2] + vector[3]*vector[3]);
}

void updateTransMatrix44(double* rot_vector, double* trans_vector)
{
    stack[S_TmpDouble_1] = sin(rot_vector[0]);  // sin(A)
    stack[S_TmpDouble_2] = cos(rot_vector[0]);  // cos(A)
    stack[S_TmpDouble_3] = sin(rot_vector[1]);  // sin(B)
    stack[S_TmpDouble_4] = cos(rot_vector[1]);  // cos(B)
    stack[S_TmpDouble_5] = sin(rot_vector[2]);  // sin(C)
    stack[S_TmpDouble_6] = cos(rot_vector[2]);  // cos(C)

    stack[S_TransMatrix] = stack[S_TmpDouble_4]*stack[S_TmpDouble_6];
    stack[S_TransMatrix + 1] = stack[S_TmpDouble_6]*stack[S_TmpDouble_1]*stack[S_TmpDouble_3]-stack[S_TmpDouble_2]*stack[S_TmpDouble_5];
    stack[S_TransMatrix + 2] = stack[S_TmpDouble_2]*stack[S_TmpDouble_6]*stack[S_TmpDouble_3]+stack[S_TmpDouble_1]*stack[S_TmpDouble_5];
    stack[S_TransMatrix + 3] = trans_vector[0];
    stack[S_TransMatrix + 4] = stack[S_TmpDouble_4]*stack[S_TmpDouble_5];
    stack[S_TransMatrix + 5] = stack[S_TmpDouble_2]*stack[S_TmpDouble_6]+stack[S_TmpDouble_1]*stack[S_TmpDouble_3]*stack[S_TmpDouble_5];
    stack[S_TransMatrix + 6] = -stack[S_TmpDouble_6]*stack[S_TmpDouble_1]+stack[S_TmpDouble_2]*stack[S_TmpDouble_3]*stack[S_TmpDouble_5];
    stack[S_TransMatrix + 7] = trans_vector[1];
    stack[S_TransMatrix + 8] = -stack[S_TmpDouble_3];
    stack[S_TransMatrix + 9] = stack[S_TmpDouble_4]*stack[S_TmpDouble_1];
    stack[S_TransMatrix + 10] = stack[S_TmpDouble_2]*stack[S_TmpDouble_4];
    stack[S_TransMatrix + 11] = trans_vector[2];
}

void updateHomoTransMatrix44(int target_joint_index, double target_joint_q)
{
    // standard dh
    //                T =    [    ct  -st*ca  st*sa   L.a*ct
    //                            st  ct*ca   -ct*sa  L.a*st
    //                            0   sa      ca      d
    //                            0   0       0       1];

    stack[S_TmpDouble_1] = stack[S_RealTheta + target_joint_index] + target_joint_q;        // compute real q
    stack[S_TmpDouble_2] = sin(stack[S_TmpDouble_1]);   // sin(theta)
    stack[S_TmpDouble_3] = cos(stack[S_TmpDouble_1]);   // cos(theta)
    stack[S_TmpDouble_4] = sin(stack[S_RealAlpha + target_joint_index]); // sin(alpha)
    stack[S_TmpDouble_5] = cos(stack[S_RealAlpha + target_joint_index]); // cos(alpha)
    stack[S_TmpDouble_6] = stack[S_RealA + target_joint_index];     // a

    stack[S_HomoTransMatrix] = stack[S_TmpDouble_3];
    stack[S_HomoTransMatrix + 1] = -stack[S_TmpDouble_2] * stack[S_TmpDouble_5];
    stack[S_HomoTransMatrix + 2] = stack[S_TmpDouble_2] * stack[S_TmpDouble_4];
    stack[S_HomoTransMatrix + 3] = stack[S_TmpDouble_6] * stack[S_TmpDouble_3];
    stack[S_HomoTransMatrix + 4] = stack[S_TmpDouble_2];
    stack[S_HomoTransMatrix + 5] = stack[S_TmpDouble_3] * stack[S_TmpDouble_5];
    stack[S_HomoTransMatrix + 6] = -stack[S_TmpDouble_3] * stack[S_TmpDouble_4];
    stack[S_HomoTransMatrix + 7] = stack[S_TmpDouble_6] * stack[S_TmpDouble_2];
    stack[S_HomoTransMatrix + 9] = stack[S_TmpDouble_4];
    stack[S_HomoTransMatrix + 10] = stack[S_TmpDouble_5];
    stack[S_HomoTransMatrix + 11] = stack[S_RealD + target_joint_index];
}

void getHomoTransMatrix44(int target_joint_index, double target_joint_q, double* r)
{
    // standard dh
    //                T =    [    ct  -st*ca  st*sa   L.a*ct
    //                            st  ct*ca   -ct*sa  L.a*st
    //                            0   sa      ca      d
    //                            0   0       0       1];

    stack[S_TmpDouble_1] = stack[S_RealTheta + target_joint_index] + target_joint_q;        // compute real q
    stack[S_TmpDouble_2] = sin(stack[S_TmpDouble_1]);   // sin(theta)
    stack[S_TmpDouble_3] = cos(stack[S_TmpDouble_1]);   // cos(theta)
    stack[S_TmpDouble_4] = sin(stack[S_RealAlpha + target_joint_index]); // sin(alpha)
    stack[S_TmpDouble_5] = cos(stack[S_RealAlpha + target_joint_index]); // cos(alpha)
    stack[S_TmpDouble_6] = stack[S_RealA + target_joint_index];     // a

    r[0] = stack[S_TmpDouble_3];
    r[1] = -stack[S_TmpDouble_2] * stack[S_TmpDouble_5];
    r[2] = stack[S_TmpDouble_2] * stack[S_TmpDouble_4];
    r[3] = stack[S_TmpDouble_6] * stack[S_TmpDouble_3];
    r[4] = stack[S_TmpDouble_2];
    r[5] = stack[S_TmpDouble_3] * stack[S_TmpDouble_5];
    r[6] = -stack[S_TmpDouble_3] * stack[S_TmpDouble_4];
    r[7] = stack[S_TmpDouble_6] * stack[S_TmpDouble_2];
    r[9] = stack[S_TmpDouble_4];
    r[10] = stack[S_TmpDouble_5];
    r[11] = stack[S_RealD + target_joint_index];
}


void getMatrix44MultiMatrix44(double* a, double* b, double* r)
{
    r[0] = a[0]*b[0] + a[1]*b[4] + a[2]*b[8] + a[3]*b[12];
    r[1] = a[0]*b[1] + a[1]*b[5] + a[2]*b[9] + a[3]*b[13];
    r[2] = a[0]*b[2] + a[1]*b[6] + a[2]*b[10] + a[3]*b[14];
    r[3] = a[0]*b[3] + a[1]*b[7] + a[2]*b[11] + a[3]*b[15];

    r[4] = a[4]*b[0] + a[5]*b[4] + a[6]*b[8] + a[7]*b[12];
    r[5] = a[4]*b[1] + a[5]*b[5] + a[6]*b[9] + a[7]*b[13];
    r[6] = a[4]*b[2] + a[5]*b[6] + a[6]*b[10] + a[7]*b[14];
    r[7] = a[4]*b[3] + a[5]*b[7] + a[6]*b[11] + a[7]*b[15];   

    r[8] = a[8]*b[0] + a[9]*b[4] + a[10]*b[8] + a[11]*b[12];
    r[9] = a[8]*b[1] + a[9]*b[5] + a[10]*b[9] + a[11]*b[13];
    r[10] = a[8]*b[2] + a[9]*b[6] + a[10]*b[10] + a[11]*b[14];
    r[11] = a[8]*b[3] + a[9]*b[7] + a[10]*b[11] + a[11]*b[15];  

    r[12] = a[12]*b[0] + a[13]*b[4] + a[14]*b[8] + a[15]*b[12];
    r[13] = a[12]*b[1] + a[13]*b[5] + a[14]*b[9] + a[15]*b[13];
    r[14] = a[12]*b[2] + a[13]*b[6] + a[14]*b[10] + a[15]*b[14];
    r[15] = a[12]*b[3] + a[13]*b[7] + a[14]*b[11] + a[15]*b[15];       
}

void getVector3CrossProduct(double* a, double* b, double* r)
{
    r[0] = a[1]*b[2] - a[2]*b[1];
    r[1] = a[2]*b[0] - a[0]*b[2];
    r[2] = a[0]*b[1] - a[1]*b[0];
}


void getMatrix33Transpose(double* matrix, double* matrix_t)
{
    matrix_t[0] = matrix[0]; matrix_t[1] = matrix[3]; matrix_t[2] = matrix[6];
    matrix_t[3] = matrix[1]; matrix_t[4] = matrix[4]; matrix_t[5] = matrix[7];
    matrix_t[6] = matrix[2]; matrix_t[7] = matrix[5]; matrix_t[8] = matrix[8];
}

void getMatrix33MultiVector3(double* matrix, double* vector, double* vector_r)
{
    vector_r[0] = matrix[0]*vector[0] + matrix[1]*vector[1] + matrix[2]*vector[2];
    vector_r[1] = matrix[3]*vector[0] + matrix[4]*vector[1] + matrix[5]*vector[2];
    vector_r[2] = matrix[6]*vector[0] + matrix[7]*vector[1] + matrix[8]*vector[2];
}

void getMatrix66MultiVector6(double* matrix, double* vector, double* vector_r)
{
    vector_r[0] = matrix[0]*vector[0] + matrix[1]*vector[1] + matrix[2]*vector[2] + matrix[3]*vector[3] + matrix[4]*vector[4] + matrix[5]*vector[5];
    vector_r[1] = matrix[6]*vector[0] + matrix[7]*vector[1] + matrix[8]*vector[2] + matrix[9]*vector[3] + matrix[10]*vector[4] + matrix[11]*vector[5];
    vector_r[2] = matrix[12]*vector[0] + matrix[13]*vector[1] + matrix[14]*vector[2] + matrix[15]*vector[3] + matrix[16]*vector[4] + matrix[17]*vector[5];
    vector_r[3] = matrix[18]*vector[0] + matrix[19]*vector[1] + matrix[20]*vector[2] + matrix[21]*vector[3] + matrix[22]*vector[4] + matrix[23]*vector[5];
    vector_r[4] = matrix[24]*vector[0] + matrix[25]*vector[1] + matrix[26]*vector[2] + matrix[27]*vector[3] + matrix[28]*vector[4] + matrix[29]*vector[5];
    vector_r[5] = matrix[30]*vector[0] + matrix[31]*vector[1] + matrix[32]*vector[2] + matrix[33]*vector[3] + matrix[34]*vector[4] + matrix[35]*vector[5];
}



void getRotationMatrix33FromHomoTransMatrix44(double* homo_trans, double* rotation)
{
    rotation[0] = homo_trans[0];
    rotation[1] = homo_trans[1];
    rotation[2] = homo_trans[2];
    rotation[3] = homo_trans[4];
    rotation[4] = homo_trans[5];
    rotation[5] = homo_trans[6];
    rotation[6] = homo_trans[8];
    rotation[7] = homo_trans[9];
    rotation[8] = homo_trans[10];
}

double getBaseFunction(int i, int k, double u)
{
    int base_index = S_BSplineNodeVector + i;
    if(k == 0)
    {
        if(u >= stack[base_index]
            && u <= stack[base_index+1])
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        stack[S_TmpDouble_1] = stack[base_index+k] - stack[base_index];
        stack[S_TmpDouble_2] = stack[base_index+k+1] - stack[base_index+1];
        if(stack[S_TmpDouble_1] == 0)
        {
            stack[S_TmpDouble_1] = 1;
        }
        if(stack[S_TmpDouble_2] == 0)
        {
            stack[S_TmpDouble_2] = 1;
        }   
        return (u - stack[base_index]) / stack[S_TmpDouble_1] * getBaseFunction(i, k-1, u)
               + (stack[base_index+k+1] - u) / stack[S_TmpDouble_2] * getBaseFunction(i+1, k-1, u);
    }
}

void updateTransitionBSpLineResult(int k, double* start_pos, double* mid_pos, double* end_pos, int result_count)
{
    //S_TmpDouble_1, S_TmpDouble_2 are used in getBaseFunction(), reserve them here
    stack[S_TmpDouble_3] = 1.0 / (result_count + 1);  // interpolation step
    stack[S_TmpDouble_4] = stack[S_TmpDouble_3];   // sum of interpolation step
    
    for(int i = 0; i < result_count; ++i)
    {
        stack[S_BaseFunctionNik0] = getBaseFunction(0, k, stack[S_TmpDouble_4]);
        stack[S_BaseFunctionNik1] = getBaseFunction(1, k, stack[S_TmpDouble_4]);
        stack[S_BaseFunctionNik2] = getBaseFunction(2, k, stack[S_TmpDouble_4]);

        stack[S_BSpLineResultXBase + i] = start_pos[0] * stack[S_BaseFunctionNik0]
                                        + mid_pos[0] * stack[S_BaseFunctionNik1]
                                        + end_pos[0] * stack[S_BaseFunctionNik2];
        stack[S_BSpLineResultYBase + i] = start_pos[1] * stack[S_BaseFunctionNik0]
                                        + mid_pos[1] * stack[S_BaseFunctionNik1]
                                        + end_pos[1] * stack[S_BaseFunctionNik2];
        stack[S_BSpLineResultZBase + i] = start_pos[2] * stack[S_BaseFunctionNik0]
                                        + mid_pos[2] * stack[S_BaseFunctionNik1]
                                        + end_pos[2] * stack[S_BaseFunctionNik2];
        stack[S_TmpDouble_4] += stack[S_TmpDouble_3];
    }
}

double getQuaternsIntersectionAngle(double* quatern1, double* quatern2)
{
    stack[S_TmpDouble_1] = quatern1[0] * quatern2[0] + quatern1[1] * quatern2[1] + quatern1[2] * quatern2[2] + quatern1[3] * quatern2[3];
    if(stack[S_TmpDouble_1] < 0)
    {
        quatern1[0] = -quatern1[0];
        quatern1[1] = -quatern1[1];
        quatern1[2] = -quatern1[2];
        quatern1[3] = -quatern1[3];
        if(stack[S_TmpDouble_1] < -1)
        {
            return (-PI / 2);
        }
    }

    if(stack[S_TmpDouble_1] > 1)
    {
        return (PI / 2);
    }

    return acos(stack[S_TmpDouble_1]);
}

void getEulerToRotationMatrix33(double* euler, double* rotation)
{
    stack[S_TmpDouble_1] = sin(euler[0]);  // sin(Z)
    stack[S_TmpDouble_2] = cos(euler[0]);  // cos(Z)
    stack[S_TmpDouble_3] = sin(euler[1]);  // sin(Y)
    stack[S_TmpDouble_4] = cos(euler[1]);  // cos(Y)
    stack[S_TmpDouble_5] = sin(euler[2]);  // sin(X)
    stack[S_TmpDouble_6] = cos(euler[2]);  // cos(X)

    rotation[0] = stack[S_TmpDouble_2]*stack[S_TmpDouble_4];
    rotation[1] = stack[S_TmpDouble_2]*stack[S_TmpDouble_3]*stack[S_TmpDouble_5]-stack[S_TmpDouble_6]*stack[S_TmpDouble_1];
    rotation[2] = stack[S_TmpDouble_2]*stack[S_TmpDouble_6]*stack[S_TmpDouble_3]+stack[S_TmpDouble_1]*stack[S_TmpDouble_5];
    rotation[3] = stack[S_TmpDouble_4]*stack[S_TmpDouble_1];
    rotation[4] = stack[S_TmpDouble_2]*stack[S_TmpDouble_6]+stack[S_TmpDouble_1]*stack[S_TmpDouble_3]*stack[S_TmpDouble_5];
    rotation[5] = stack[S_TmpDouble_6]*stack[S_TmpDouble_1]*stack[S_TmpDouble_3]-stack[S_TmpDouble_2]*stack[S_TmpDouble_5];
    rotation[6] = -stack[S_TmpDouble_3];
    rotation[7] = stack[S_TmpDouble_4]*stack[S_TmpDouble_5];
    rotation[8] = stack[S_TmpDouble_4]*stack[S_TmpDouble_6];   
}

void getRotationMatrix33ToEuler(double* rotation, double* euler)
{
    /*euler[1] = atan2(-rotation[6], sqrt(rotation[0]*rotation[0] + rotation[3]*rotation[3]));
    if(rotation[7]*rotation[7] + rotation[8]*rotation[8] > SQRT_DOUBLE_ACCURACY)
    {
        euler[0] = atan2(rotation[3], rotation[0]);
        euler[2] = atan2(rotation[7], rotation[8]);
    }
    else
    {
        euler[0] = atan2(-rotation[1], rotation[4]);
        euler[2] = 0;
    }*/
    euler[1] = atan2(-rotation[6], sqrt(rotation[7]*rotation[7] + rotation[8]*rotation[8]));
    if((rotation[7]*rotation[7] + rotation[8]*rotation[8]) > SQRT_DOUBLE_ACCURACY)
    {
        euler[0] = atan2(rotation[3], rotation[0]);
        euler[2] = atan2(rotation[7], rotation[8]);
    }
    else
    {
        euler[0] = atan2(-rotation[1], rotation[4]);
        euler[2] = 0;
    }    
}

void getQuaternToRotationMatrix33(double* quatern, double* rotation)
{
    // don't check if x^2+y^2+z^2+w^2 == 1, suppose the condition holds
    stack[S_TmpDouble_1] = 2 * quatern[0] * quatern[1]; // 2xy
    stack[S_TmpDouble_2] = 2 * quatern[1] * quatern[2]; // 2yz
    stack[S_TmpDouble_3] = 2 * quatern[2] * quatern[0]; // 2zx
    stack[S_TmpDouble_4] = 2 * quatern[3] * quatern[0]; // 2wx
    stack[S_TmpDouble_5] = 2 * quatern[3] * quatern[1]; // 2wy
    stack[S_TmpDouble_6] = 2 * quatern[3] * quatern[2]; // 2wz
    stack[S_TmpDouble_7] = quatern[3] * quatern[3]; // w^2

    rotation[0] = 2 * (stack[S_TmpDouble_7] + quatern[0] * quatern[0]) - 1; // r[0][0] = 2*(w^2+x^2)-1
    rotation[1] = stack[S_TmpDouble_1] - stack[S_TmpDouble_6];  // r[0][1] = 2*(xy-wz)
    rotation[2] = stack[S_TmpDouble_5] + stack[S_TmpDouble_3];  // r[0][2] = 2*(wy+zx)
    rotation[3] = stack[S_TmpDouble_1] + stack[S_TmpDouble_6];  // r[1][0] = 2*(xy+wz)-1
    rotation[4] = 2 * (stack[S_TmpDouble_7] + quatern[1] * quatern[1]) - 1; // r[1][1] = 2*(w^2+y^2)-1
    rotation[5] = stack[S_TmpDouble_2] - stack[S_TmpDouble_4];  // r[1][2] = 2*(yz-wx)
    rotation[6] = stack[S_TmpDouble_3] - stack[S_TmpDouble_5];  // r[2][0] = 2*(zx-wy)
    rotation[7] = stack[S_TmpDouble_4] + stack[S_TmpDouble_2];  // r[2][1] = 2*(wx+yz)
    rotation[8] = 2 * (stack[S_TmpDouble_7] + quatern[2] * quatern[2]) - 1; // r[2][2] = 2*(w^2+z^2)-1
}

void getRotationMatrix33ToQuatern(double* rotation, double* quatern)
{
    int max_id = 0;
    quatern[0] = sqrt(rotation[0] - rotation[4] - rotation[8] + 1) / 2; // x
    quatern[1] = sqrt(rotation[4] - rotation[0] - rotation[8] + 1) / 2; // y
    if(quatern[1] > quatern[0])
    {
        stack[S_TmpDouble_1] = quatern[1];  // store max value of {x,y,z,s}
        max_id = 1;
    }
    quatern[2] = sqrt(rotation[8] - rotation[0] - rotation[4] + 1) / 2; // z
    if(quatern[2] > stack[S_TmpDouble_1])
    {
        stack[S_TmpDouble_1] = quatern[2];
        max_id = 2;
    }
    quatern[3] = sqrt(rotation[0] + rotation[4] + rotation[8] + 1) / 2; // w
    if(quatern[3] > stack[S_TmpDouble_1])
    {
        max_id = 3;
    }

    switch(max_id)
    {
        case 0:
            if(rotation[3] + rotation[1] < 0) quatern[1] = -quatern[1];
            if(rotation[6] + rotation[2] < 0) quatern[2] = -quatern[2];
            if(rotation[7] - rotation[5] < 0) quatern[3] = -quatern[3];
            return;
        case 1:
            if(rotation[3] + rotation[1] < 0) quatern[0] = -quatern[0];
            if(rotation[7] + rotation[5] < 0) quatern[2] = -quatern[2];
            if(rotation[2] - rotation[6] < 0) quatern[3] = -quatern[3];
            return;
        case 2:
            if(rotation[6] + rotation[2] < 0) quatern[0] = -quatern[0];
            if(rotation[7] + rotation[5] < 0) quatern[1] = -quatern[1];
            if(rotation[3] - rotation[1] < 0) quatern[3] = -quatern[3];
            return;
        case 3:
            if(rotation[7] - rotation[5] < 0) quatern[0] = -quatern[0];
            if(rotation[2] - rotation[6] < 0) quatern[1] = -quatern[1];
            if(rotation[3] - rotation[1] < 0) quatern[2] = -quatern[2];
            return;
    }
}

void getEulerToQuatern(double* euler, double* quatern)
{
    getEulerToRotationMatrix33(euler, &stack[S_TmpMatrix33_1]);
    getRotationMatrix33ToQuatern(&stack[S_TmpMatrix33_1], quatern);
}

void getQuaternToEuler(double* quatern, double* euler)
{
    getQuaternToRotationMatrix33(quatern, &stack[S_TmpMatrix33_1]);
    getRotationMatrix33ToEuler(&stack[S_TmpMatrix33_1], euler);
}

void getQuaternVector4(double* start_quatern, double* end_quartern, double angle, double angle_distance_to_start, double* target_quatern)
{
    if(fabs(angle) > segment_alg_param.angle_valve)
    {
        // slerp interpolation
        stack[S_TmpDouble_1] = sin(angle);
        stack[S_TmpDouble_2] = sin((1 - angle_distance_to_start) * angle) / stack[S_TmpDouble_1];   // a(t)
        stack[S_TmpDouble_3] = sin(angle_distance_to_start * angle) / stack[S_TmpDouble_1]; // b(t)
        // target = a(t) * start + b(t) * end
        target_quatern[0] = stack[S_TmpDouble_2] * start_quatern[0] + stack[S_TmpDouble_3] * end_quartern[0];
        target_quatern[1] = stack[S_TmpDouble_2] * start_quatern[1] + stack[S_TmpDouble_3] * end_quartern[1];
        target_quatern[2] = stack[S_TmpDouble_2] * start_quatern[2] + stack[S_TmpDouble_3] * end_quartern[2];
        target_quatern[3] = stack[S_TmpDouble_2] * start_quatern[3] + stack[S_TmpDouble_3] * end_quartern[3];
    }
    else
    {
        stack[S_TmpDouble_1] = 1 - angle_distance_to_start;
        // target = (1-t) * start + t * end
        target_quatern[0] = stack[S_TmpDouble_1] * start_quatern[0] + angle_distance_to_start * end_quartern[0];
        target_quatern[1] = stack[S_TmpDouble_1] * start_quatern[1] + angle_distance_to_start * end_quartern[1];
        target_quatern[2] = stack[S_TmpDouble_1] * start_quatern[2] + angle_distance_to_start * end_quartern[2];
        target_quatern[3] = stack[S_TmpDouble_1] * start_quatern[3] + angle_distance_to_start * end_quartern[3];
    }
}


void getModelFk(ComplexAxisGroupModel* model_ptr, double* joints, double* pos_euler)
{
    getHomoTransMatrix44(0, joints[0], &stack[S_TmpMatrix44_1]);
    
    int offset = S_TmpMatrix44_1;
    for(int i = 1; i < model_ptr->link_num; ++i)
    {
        updateHomoTransMatrix44(i, joints[i]);
        getMatrix44MultiMatrix44(&stack[offset], &stack[S_HomoTransMatrix], &stack[offset + 16]);
        offset += 16;
    }
    
    // stack[offset] stroe the result
    pos_euler[0] = stack[offset + 3];   // x
    pos_euler[1] = stack[offset + 7];   // y
    pos_euler[2] = stack[offset + 11];  // z
    getRotationMatrix33FromHomoTransMatrix44(&stack[offset], &stack[S_TmpMatrix33_1]);
    getRotationMatrix33ToEuler(&stack[S_TmpMatrix33_1], &pos_euler[3]);
}


inline void doRowOperation(double* target_row, double* ref_row, double times, int row_size)
{
    for(int i = 0; i < row_size; ++i)
    {
        target_row[i] = target_row[i] - times * ref_row[i];
    }
}

inline void updateEquationSolution(double* matrix_a, double* matrix_b, int order)
{
    int l_index, diag_index, row_operation_size;
    int i, row, col, base_index;
    // AX=LUX=B, compute L & U
    for(col = 0; col < order; ++col)
    {
        diag_index = col*order+col;
        row_operation_size = order - col -1;
        for(row = col + 1; row < order; ++row)
        {
            l_index = row*order+col;
            matrix_a[l_index] = matrix_a[l_index] / matrix_a[diag_index];
            doRowOperation(&matrix_a[l_index+1], &matrix_a[diag_index+1], matrix_a[l_index], row_operation_size);
        }
    }

    // AX=LUX=B ==> make UX=C ==> LC=B, solve C
    for(row = 0; row < order; ++row)
    {
        stack[S_TmpDouble_1] = 0;
        for(i = 0; i < row; ++i)
        {
            stack[S_TmpDouble_1] += matrix_a[row*order + i] * stack[S_X + i];
        }
        stack[S_X + row] = matrix_b[row] - stack[S_TmpDouble_1];
    }

    // AX=LUX=B ==> make UX=C ==> LC=B, solve C ==> UX=C, solve X
    for(row = order - 1; row >= 0; --row)
    {
        stack[S_TmpDouble_1] = 0;
        base_index = row*order;
        for(i = row; i < order - 1; ++i)
        {
            stack[S_TmpDouble_1] += matrix_a[base_index+i+1] * stack[S_X+i+1];
        }
        stack[S_X + row] = (stack[S_X + row] - stack[S_TmpDouble_1]) / matrix_a[base_index+row];
    }
}

inline void updateMatrixA(double* traj_t, int order)
{
    int row_start, element_start;
  
    stack[S_A] = traj_t[0] + 2 * traj_t[1]; // A[0][0]
    stack[S_A + 1] = traj_t[1]; // A[0][1]
    memset(&stack[S_A + 2], 0, sizeof(double) * (order - 2));   // A[0][2] ~ A[0][order-1]
    for(int row = 1; row < (order - 1); ++row)
    {
        row_start = S_A + row * order;
        element_start = row_start + row - 1;        
        memset(&stack[row_start], 0, (row - 1) * sizeof(double));
        stack[element_start] = traj_t[row];
        stack[element_start + 1] = 2 * (traj_t[row] + traj_t[row + 1]);
        stack[element_start + 2] = traj_t[row + 1];
        memset(&stack[element_start + 3], 0, (order - row - 2) * sizeof(double));
    }

    row_start = S_A + (order - 1) * order;
    element_start = row_start + order - 2;
    memset(&stack[row_start], 0, (order - 2) * sizeof(double));
    stack[element_start] = traj_t[order - 1];
    stack[element_start + 1] = 2 * traj_t[order - 1] + traj_t[order]; 
}

inline void updateMatrixB(double* traj_p, double* traj_t, double* start_state, double* end_state, int order)
{
    stack[S_B] = -12 * (traj_p[1] - traj_p[0]) / traj_t[0]
                    + 6 * (traj_p[2] - traj_p[1]) / traj_t[1]
                    + start_state[2] * traj_t[0]
                    + 6 * start_state[1];
    for(int row = 1; row < (order - 1); ++row)
    {
        stack[S_B + row] = 6 * ((traj_p[row + 2] - traj_p[row + 1]) / traj_t[row + 1] - (traj_p[row + 1] - traj_p[row]) / traj_t[row]);
    }
    stack[S_B + order - 1] = 12 * (traj_p[order + 1] - traj_p[order]) / traj_t[order]
                                - 6 * (traj_p[order] - traj_p[order - 1]) / traj_t[order - 1]
                                + end_state[2] * traj_t[order]
                                - 6 * end_state[1];
}

void updateTrajPVA(int traj_p_address, int traj_v_address, int traj_a_address, int traj_pva_size, 
                       int traj_j_address, double* traj_t_base, int traj_t_size, int start_state_address, int end_state_address)
{
    int order = traj_t_size - 1;
    for(int i = 0; i < model.link_num; ++i)
    {
        updateMatrixA(traj_t_base, order);
        updateMatrixB(&stack[traj_p_address], traj_t_base, &stack[start_state_address], &stack[end_state_address], order);        
        updateEquationSolution(&stack[S_A], &stack[S_B], order);
        getJerkStart(&stack[traj_p_address], traj_pva_size, traj_t_base, traj_t_size, &stack[start_state_address], stack[S_X], stack[traj_j_address], stack[traj_j_address + 1]);
        getJerkEnd(&stack[traj_p_address], traj_pva_size, traj_t_base, traj_t_size, &stack[end_state_address], stack[S_X + order - 1], stack[traj_j_address + 2], stack[traj_j_address + 3]);
        updateTrajVA(&stack[traj_p_address], &stack[traj_v_address], &stack[traj_a_address], traj_pva_size, &stack[traj_j_address], traj_t_base, traj_t_size, &stack[start_state_address], &stack[end_state_address]);
        traj_p_address += 75;
        traj_v_address += 75;
        traj_a_address += 75;
        traj_j_address += 4;
        start_state_address += 3;
        end_state_address += 3;
    }
}

inline void getMoveLPathVector(const Point& start_point, const Point& end_point, double* path_vector, double& path_length)
{
    path_vector[0] = end_point.x_ - start_point.x_; 
    path_vector[1] = end_point.y_ - start_point.y_;
    path_vector[2] = end_point.z_ - start_point.z_;
    path_length = getVector3Norm(path_vector);
    path_vector[0] /= path_length;
    path_vector[1] /= path_length;
    path_vector[2] /= path_length;
}

inline double getPointsDistance(const Point& point1, const Point& point2)
{
    stack[S_TmpVector3_1] = point1.x_ - point2.x_; 
    stack[S_TmpVector3_1 + 1] = point1.y_ - point2.y_; 
    stack[S_TmpVector3_1 + 2] = point1.z_ - point2.z_; 
    return getVector3Norm(&stack[S_TmpVector3_1]);
}

inline void getMoveLPathPoint(const Point& start_point, double* path_vector, double distance, Point& target_point)
{
    target_point.x_ = start_point.x_ + path_vector[0] * distance;
    target_point.y_ = start_point.y_ + path_vector[1] * distance;
    target_point.z_ = start_point.z_ + path_vector[2] * distance;
}

inline void getMoveEulerToQuatern(const Euler& euler, double* quatern)
{
    stack[S_TmpVector3_1] = euler.a_;
    stack[S_TmpVector3_1 + 1] = euler.b_;
    stack[S_TmpVector3_1 + 2] = euler.c_;
    getEulerToQuatern(&stack[S_TmpVector3_1], quatern);
}

inline void getQuaternToQuaternVector4(const Quaternion quatern, double* quatern_vector)
{
    quatern_vector[0] = quatern.x_;
    quatern_vector[1] = quatern.y_;
    quatern_vector[2] = quatern.z_;
    quatern_vector[3] = quatern.w_;
}

inline void getMovePointToVector3(const Point& point, double* pos_vector)
{
    pos_vector[0] = point.x_;
    pos_vector[1] = point.y_;
    pos_vector[2] = point.z_;
}

inline void getQuaternPoint(double* start_quatern, double* end_quartern, double angle, double angle_distance_to_start, Quaternion& target_quatern)
{
    if(fabs(angle) > segment_alg_param.angle_valve)
    {
        // slerp interpolation
        stack[S_TmpDouble_1] = sin(angle);
        stack[S_TmpDouble_2] = sin((1 - angle_distance_to_start) * angle) / stack[S_TmpDouble_1];   // a(t)
        stack[S_TmpDouble_3] = sin(angle_distance_to_start * angle) / stack[S_TmpDouble_1]; // b(t)
        // target = a(t) * start + b(t) * end
        target_quatern.x_ = stack[S_TmpDouble_2] * start_quatern[0] + stack[S_TmpDouble_3] * end_quartern[0];
        target_quatern.y_ = stack[S_TmpDouble_2] * start_quatern[1] + stack[S_TmpDouble_3] * end_quartern[1];
        target_quatern.z_ = stack[S_TmpDouble_2] * start_quatern[2] + stack[S_TmpDouble_3] * end_quartern[2];
        target_quatern.w_ = stack[S_TmpDouble_2] * start_quatern[3] + stack[S_TmpDouble_3] * end_quartern[3];
    }
    else
    {
        stack[S_TmpDouble_1] = 1 - angle_distance_to_start;
        // target = (1-t) * start + t * end
        target_quatern.x_ = stack[S_TmpDouble_1] * start_quatern[0] + angle_distance_to_start * end_quartern[0];
        target_quatern.y_ = stack[S_TmpDouble_1] * start_quatern[1] + angle_distance_to_start * end_quartern[1];
        target_quatern.z_ = stack[S_TmpDouble_1] * start_quatern[2] + angle_distance_to_start * end_quartern[2];
        target_quatern.w_ = stack[S_TmpDouble_1] * start_quatern[3] + angle_distance_to_start * end_quartern[3];
    }

    stack[S_TmpDouble_4] = sqrt(target_quatern.x_ * target_quatern.x_
                                + target_quatern.y_ * target_quatern.y_
                                + target_quatern.z_ * target_quatern.z_
                                + target_quatern.w_ * target_quatern.w_);
    target_quatern.x_ = target_quatern.x_ / stack[S_TmpDouble_4];
    target_quatern.y_ = target_quatern.y_ / stack[S_TmpDouble_4];
    target_quatern.z_ = target_quatern.z_ / stack[S_TmpDouble_4];
    target_quatern.w_ = target_quatern.w_ / stack[S_TmpDouble_4];
}

inline void packPoseByPointAndQuatern(Point point, double quatern[4], PoseQuaternion& pose)
{
    pose.point_ = point;
    pose.quaternion_.x_ = quatern[0];
    pose.quaternion_.y_ = quatern[1];
    pose.quaternion_.z_ = quatern[2];
    pose.quaternion_.w_ = quatern[3];
}

inline void packPathBlockType(PointType point_type, MotionType motion_type, PathBlock& path_block)
{
    path_block.point_type = point_type;
    path_block.motion_type = motion_type;
}

inline void updateTrajPSingleItem(int traj_p_address, const Joint& joint)
{
    for(int i = 0; i < model.link_num; ++i)
    {
        stack[traj_p_address] = joint[i];
        traj_p_address += 75;
    }
}

inline void getTrajPFromPathStart2End(const PathCache& path_cache, double traj_piece_ideal_start2end, 
                                             int* traj_path_cache_index, int& traj_pva_out_index, int& traj_pva_size)
{
    int path_cache_length_minus_1 = path_cache.cache_length - 1;
    // decide traj_pva size & path_index step
    if(traj_piece_ideal_start2end <= 3)
    {
        traj_pva_size = 4;
    }
    else if(traj_piece_ideal_start2end >= (segment_alg_param.max_traj_points_num - 1))
    {
        traj_pva_size = segment_alg_param.max_traj_points_num;
    }
    else
    {
        traj_pva_size = ceil(traj_piece_ideal_start2end) + 1;
    }
    int traj_pva_size_minus_1 = traj_pva_size - 1;
    stack[S_PathIndexStep_Start2End] = path_cache_length_minus_1 / (double)traj_pva_size_minus_1;
    // select traj point from path cache        
    double path_index_ideal = 0;
    updateTrajPSingleItem(S_TrajP0, path_cache.cache[0].joint);
    traj_path_cache_index[0] = 0;
    for(int i = 1; i < traj_pva_size_minus_1; ++i)
    {
        path_index_ideal += stack[S_PathIndexStep_Start2End];
        traj_path_cache_index[i] = round(path_index_ideal);
        updateTrajPSingleItem(S_TrajP0 + i, path_cache.cache[traj_path_cache_index[i]].joint);
    }
    updateTrajPSingleItem(S_TrajP0 + traj_pva_size_minus_1, path_cache.cache[path_cache_length_minus_1].joint); 
    traj_path_cache_index[traj_pva_size_minus_1] = path_cache_length_minus_1;
    // mark traj_pva_out_index
    if(path_cache.smooth_out_index == -1)
    {
        traj_pva_out_index = -1;
    }
    else
    {
        traj_pva_out_index = traj_pva_size_minus_1;
    }
}

inline void getTrajPFromPathStart2Out2End(const PathCache& path_cache, double traj_piece_ideal_start2end, 
                                                   int* traj_path_cache_index, int& traj_pva_out_index, int& traj_pva_size)
{
    int i;
    int path_cache_length_minus_1 = path_cache.cache_length - 1;
    // decide traj_pva size & path_index step
    if(traj_piece_ideal_start2end <= 3)
    {
        traj_piece_ideal_start2end = 4;
    }
    else if(traj_piece_ideal_start2end >= (segment_alg_param.max_traj_points_num - 2))
    {
        traj_piece_ideal_start2end = segment_alg_param.max_traj_points_num - 1;
    }
    else
    {
        traj_piece_ideal_start2end = ceil(traj_piece_ideal_start2end) + 1;
    }
    double traj_piece_ideal_start2out = path_cache.smooth_out_index * traj_piece_ideal_start2end / path_cache_length_minus_1;
    double traj_piece_ideal_out2end = (path_cache_length_minus_1 - path_cache.smooth_out_index) * traj_piece_ideal_start2end / path_cache_length_minus_1;
    int traj_pva_size_start2out = ceil(traj_piece_ideal_start2out) + 1;
    int traj_pva_size_start2out_minus_1 = traj_pva_size_start2out - 1;
    int traj_piece_real_out2end = ceil(traj_piece_ideal_out2end);
    traj_pva_size = traj_pva_size_start2out + traj_piece_real_out2end;
    int traj_pva_size_minus_1 = traj_pva_size - 1;
    stack[S_PathIndexStep_Start2Out] = path_cache.smooth_out_index / (double)traj_pva_size_start2out_minus_1;
    stack[S_PathIndexStep_Out2End] = (path_cache_length_minus_1 - path_cache.smooth_out_index) / (double)traj_piece_real_out2end;
    // select traj point from path cache   
    double path_index_ideal = 0;
    updateTrajPSingleItem(S_TrajP0, path_cache.cache[0].joint);
    traj_path_cache_index[0] = 0;
    for(i = 1; i < traj_pva_size_start2out; ++i)
    {
        path_index_ideal += stack[S_PathIndexStep_Start2Out];
        traj_path_cache_index[i] = round(path_index_ideal);
        updateTrajPSingleItem(S_TrajP0 + i, path_cache.cache[traj_path_cache_index[i]].joint);
    }
    for(; i < traj_pva_size_minus_1; ++i)
    {
        path_index_ideal += stack[S_PathIndexStep_Out2End];
        traj_path_cache_index[i] = round(path_index_ideal);
        updateTrajPSingleItem(S_TrajP0 + i, path_cache.cache[traj_path_cache_index[i]].joint);
    }
    updateTrajPSingleItem(S_TrajP0 + traj_pva_size_minus_1, path_cache.cache[path_cache_length_minus_1].joint); 
    traj_path_cache_index[traj_pva_size_minus_1] = path_cache_length_minus_1;
    // mark traj_pva_out_index
    traj_pva_out_index = traj_pva_size_start2out_minus_1;
}

inline void getTrajPFromPathIn2End(const PathCache& path_cache, double traj_piece_ideal_in2end, int traj_pva_in_index, 
                                          int* traj_path_cache_index_in2end, int& traj_pva_out_index, int& traj_pva_size_via2end)
{
    int path_cache_length_minus_1 = path_cache.cache_length - 1;
    // decide traj_pva size & path_index step
    double traj_piece_ideal_via2end = traj_pva_in_index + traj_piece_ideal_in2end;
    if(traj_piece_ideal_via2end <= 3)
    {
        traj_pva_size_via2end = 4;
    }
    else if(traj_piece_ideal_via2end >= (segment_alg_param.max_traj_points_num - 1))
    {
        traj_pva_size_via2end = segment_alg_param.max_traj_points_num;
    }
    else
    {
        traj_pva_size_via2end = ceil(traj_piece_ideal_via2end) + 1;
    }
    int traj_pva_size_via2end_minus_1 = traj_pva_size_via2end - 1;
    int traj_piece_real_in2end = traj_pva_size_via2end_minus_1 - traj_pva_in_index;
    int traj_piece_real_in2end_minus_1 = traj_piece_real_in2end - 1;
    stack[S_PathIndexStep_In2End] = (path_cache_length_minus_1 - path_cache.smooth_in_index) / (double)traj_piece_real_in2end;   
    // select traj point from path cache        
    updateTrajPSingleItem(S_TrajP0 + traj_pva_in_index, path_cache.cache[path_cache.smooth_in_index].joint);
    traj_path_cache_index_in2end[0] = path_cache.smooth_in_index;
    double path_index_ideal = path_cache.smooth_in_index;
    for(int i = 1; i < traj_piece_real_in2end; ++i)
    {
        path_index_ideal += stack[S_PathIndexStep_In2End];
        traj_path_cache_index_in2end[i] = round(path_index_ideal);
        updateTrajPSingleItem(S_TrajP0 + traj_pva_in_index + i, path_cache.cache[traj_path_cache_index_in2end[i]].joint);
    }
    updateTrajPSingleItem(S_TrajP0 + traj_pva_size_via2end_minus_1, path_cache.cache[path_cache_length_minus_1].joint); 
    traj_path_cache_index_in2end[traj_piece_real_in2end] = path_cache_length_minus_1;
    // mark traj_pva_out_index
    if(path_cache.smooth_out_index == -1)
    {
        traj_pva_out_index = -1;
    }
    else
    {
        traj_pva_out_index = traj_pva_size_via2end_minus_1;
    }
}

inline void getTrajPFromPathIn2Out2End(const PathCache& path_cache, double traj_piece_ideal_in2end, int traj_pva_in_index, 
                                          int* traj_path_cache_index_in2end, int& traj_pva_out_index, int& traj_pva_size_via2end)
{
    double path_length_in2out = getPointsDistance(path_cache.cache[path_cache.smooth_in_index].pose.point_, path_cache.cache[path_cache.smooth_out_index].pose.point_);
    double traj_piece_ideal_in2out = path_length_in2out * stack[S_PathCountFactorCartesian];
    if(traj_piece_ideal_in2out < DOUBLE_ACCURACY)   // in and out point is the same point
    {
        getTrajPFromPathIn2End(path_cache, traj_piece_ideal_in2end, traj_pva_in_index, traj_path_cache_index_in2end, traj_pva_out_index, traj_pva_size_via2end);
        traj_pva_out_index = traj_pva_in_index;
    }
    else
    {
        int i;
        int path_cache_lenght_minus_1 = path_cache.cache_length - 1;
        // decide traj piece size and path index step
        int traj_piece_real_out2end = ceil(traj_piece_ideal_in2end - traj_piece_ideal_in2out);  // suppose out2end piece is small
        double traj_piece_ideal_via2end = traj_pva_in_index + traj_piece_real_out2end + traj_piece_ideal_in2out;
        if(traj_piece_ideal_via2end <= 3)
        {
            traj_pva_size_via2end = 4;
        }
        else if(traj_piece_ideal_via2end >= (segment_alg_param.max_traj_points_num - 1))
        {
            traj_pva_size_via2end = segment_alg_param.max_traj_points_num;
        }
        else
        {
            traj_pva_size_via2end = ceil(traj_piece_ideal_via2end) + 1;
        }
        int traj_piece_real_via2end = traj_pva_size_via2end - 1;
        int traj_piece_real_in2out = traj_piece_real_via2end - traj_pva_in_index - traj_piece_real_out2end;
        int traj_piece_real_in2end = traj_piece_real_in2out + traj_piece_real_out2end;
        traj_pva_out_index = traj_pva_in_index + traj_piece_real_in2out;
        stack[S_PathIndexStep_In2Out] = (path_cache.smooth_out_index - path_cache.smooth_in_index) / (double)traj_piece_real_in2out;
        stack[S_PathIndexStep_Out2End] = (path_cache_lenght_minus_1 - path_cache.smooth_out_index) / (double)traj_piece_real_out2end;
        // select traj point from path cache        
        updateTrajPSingleItem(S_TrajP0 + traj_pva_in_index, path_cache.cache[path_cache.smooth_in_index].joint);
        traj_path_cache_index_in2end[0] = path_cache.smooth_in_index;
        double path_index_ideal = path_cache.smooth_in_index;
        for(i = 1; i < traj_piece_real_in2out; ++i)
        {
            path_index_ideal += stack[S_PathIndexStep_In2Out];
            traj_path_cache_index_in2end[i] = round(path_index_ideal);
            updateTrajPSingleItem(S_TrajP0 + traj_pva_in_index + i, path_cache.cache[traj_path_cache_index_in2end[i]].joint);
        }
        updateTrajPSingleItem(S_TrajP0 + traj_pva_in_index + traj_piece_real_in2out, path_cache.cache[path_cache.smooth_out_index].joint); 
        traj_path_cache_index_in2end[traj_piece_real_in2out] = path_cache.smooth_out_index;
        path_index_ideal = path_cache.smooth_out_index;
        for(i = traj_piece_real_in2out + 1; i < traj_piece_real_in2end; ++i)
        {
            path_index_ideal += stack[S_PathIndexStep_Out2End];
            traj_path_cache_index_in2end[i] = round(path_index_ideal);
            updateTrajPSingleItem(S_TrajP0 + traj_pva_in_index + i, path_cache.cache[traj_path_cache_index_in2end[i]].joint);
        }
        updateTrajPSingleItem(S_TrajP0 + traj_pva_in_index + traj_piece_real_in2end, path_cache.cache[path_cache_lenght_minus_1].joint); 
        traj_path_cache_index_in2end[traj_piece_real_in2end] = path_cache_lenght_minus_1;       
    }
}

inline void getTrajPFromPathOut2In(const PathCache& path_cache, double traj_piece_ideal_out2in, 
                                         int* traj_path_cache_index_out2in, int& traj_pva_size_out2in)
{
    // decide traj_pva size & path_index step
    if(traj_piece_ideal_out2in <= 3)
    {
        traj_pva_size_out2in = 4;
    }
    else if(traj_piece_ideal_out2in >= (segment_alg_param.max_traj_points_num - 1))
    {
        traj_pva_size_out2in = segment_alg_param.max_traj_points_num;
    }
    else
    {
        traj_pva_size_out2in = ceil(traj_piece_ideal_out2in) + 1;
    }
    int traj_pva_size_out2in_minus_1 = traj_pva_size_out2in - 1;
    stack[S_PathIndexStep_Out2In] = path_cache.smooth_in_index / (double)traj_pva_size_out2in_minus_1;
    // select traj point from path cache        
    double path_index_ideal = 0;
    updateTrajPSingleItem(S_TrajP0_Smooth, path_cache.cache[0].joint);
    traj_path_cache_index_out2in[0] = 0;
    for(int i = 1; i < traj_pva_size_out2in_minus_1; ++i)
    {
        path_index_ideal += stack[S_PathIndexStep_Out2In];
        traj_path_cache_index_out2in[i] = round(path_index_ideal);
        updateTrajPSingleItem(S_TrajP0_Smooth + i, path_cache.cache[traj_path_cache_index_out2in[i]].joint);
    }
    updateTrajPSingleItem(S_TrajP0_Smooth + traj_pva_size_out2in_minus_1, path_cache.cache[path_cache.smooth_in_index].joint); 
    traj_path_cache_index_out2in[traj_pva_size_out2in_minus_1] = path_cache.smooth_in_index; 
}

inline void updateMovLTrajP(const PathCache& path_cache, int* traj_path_cache_index, 
                                int& traj_pva_out_index, int& traj_pva_size)
{
    int path_cache_length_minus_1 = path_cache.cache_length - 1;
    double path_length_start2end = getPointsDistance(path_cache.cache[0].pose.point_, path_cache.cache[path_cache_length_minus_1].pose.point_);
    double traj_piece_ideal_start2end = path_length_start2end * stack[S_PathCountFactorCartesian];
    if(path_cache.smooth_out_index == -1
        || path_cache.smooth_out_index == path_cache_length_minus_1)
    {
        getTrajPFromPathStart2End(path_cache, traj_piece_ideal_start2end, traj_path_cache_index, traj_pva_out_index, traj_pva_size);
    }
    else
    {
        getTrajPFromPathStart2Out2End(path_cache, traj_piece_ideal_start2end, traj_path_cache_index, traj_pva_out_index, traj_pva_size);
    }
}

inline void updateMovJTrajP(const PathCache& path_cache, int* traj_path_cache_index, int& traj_pva_out_index, int& traj_pva_size)
{
    int path_cache_length_minus_1 = path_cache.cache_length - 1;
    // get max delta joint
    double delta_joint_max = 0;
    for(int i = 0; i < model.link_num; ++i)
    {
        stack[S_DeltaJointVector + i] = fabs(path_cache.cache[path_cache_length_minus_1].joint[i] - path_cache.cache[0].joint[i]);
        if(stack[S_DeltaJointVector + i] > delta_joint_max)
        {
            delta_joint_max = stack[S_DeltaJointVector + i];
        }
    }
    // decide traj_pva size & path_index step
    double traj_piece_ideal_start2end = delta_joint_max * stack[S_PathCountFactorJoint];
    if(path_cache.smooth_out_index == -1
        || path_cache.smooth_out_index == path_cache_length_minus_1)
    {
        getTrajPFromPathStart2End(path_cache, traj_piece_ideal_start2end, traj_path_cache_index, traj_pva_out_index, traj_pva_size);
    }
    else
    {
        getTrajPFromPathStart2Out2End(path_cache, traj_piece_ideal_start2end, traj_path_cache_index, traj_pva_out_index, traj_pva_size);
    }
}

inline void updateMovLVia2InTrajP(const PathCache& path_cache, const MotionTarget& via, int& traj_pva_in_index)
{
    int i, j;
    // compute path vector and quatern for via2in
    double path_vector_via2in[3];
    double path_length_via2in;
    getMoveLPathVector(via.pose_target.point_, path_cache.cache[path_cache.smooth_in_index].pose.point_, path_vector_via2in, path_length_via2in);
    double via_quatern[4], in_quatern[4];
    getMoveEulerToQuatern(via.pose_target.euler_, via_quatern);
    getQuaternToQuaternVector4(path_cache.cache[path_cache.smooth_in_index].pose.quaternion_, in_quatern);
    double angle_via2in = getQuaternsIntersectionAngle(via_quatern, in_quatern);
    // compute via joint
    Joint joint_via;
    segment_alg_param.kinematics_ptr->inverseKinematicsInUser(via.pose_target, path_cache.cache[0].joint, joint_via);
    // decide traj_pva_in_index & length_step & angle_step, fill TrajP via2in if in is not on via
    double traj_piece_ideal_via2in = path_length_via2in * stack[S_PathCountFactorCartesian];
    if(traj_piece_ideal_via2in < DOUBLE_ACCURACY)
    {
        traj_pva_in_index = 0;        
    }
    else
    {
        traj_pva_in_index = ceil(traj_piece_ideal_via2in);
        double length_step = path_length_via2in / traj_pva_in_index;
        double angle_step = angle_via2in / traj_pva_in_index;
        double length_distance_to_via = 0, angle_distance_to_via = 0;
        PoseQuaternion pose;
        Joint joint_ref = joint_via;
        Joint joint_result;
        int traj_p_address;
        for(i = 0; i <= traj_pva_in_index; ++i)
        {
            getMoveLPathPoint(via.pose_target.point_, path_vector_via2in, length_distance_to_via, pose.point_);
            getQuaternPoint(via_quatern, in_quatern, angle_via2in, angle_distance_to_via, pose.quaternion_);
            segment_alg_param.kinematics_ptr->inverseKinematicsInUser(pose, joint_ref, joint_result);
            traj_p_address = S_TrajP0;
            for(j = 0; j < model.link_num; ++j)
            {
                stack[traj_p_address + i] = joint_result[j];
                traj_p_address += 75;
            }            
            joint_ref = joint_result;
            length_distance_to_via += length_step;
            angle_distance_to_via += angle_step;
        }        
    }
}

inline void updateMovJVia2InTrajP(const PathCache& path_cache, const MotionTarget& via, int& traj_pva_in_index)
{
    int i, j;
    // compute max delta joint for via2in
    double delta_joint_max_via2in = 0;
    for(int i = 0; i < model.link_num; ++i)
    {
        stack[S_DeltaJointVector + i] = fabs(path_cache.cache[path_cache.smooth_in_index].joint[i] - via.joint_target[i]);
        if(stack[S_DeltaJointVector + i] > delta_joint_max_via2in)
        {
            delta_joint_max_via2in = stack[S_DeltaJointVector + i];
        }
    }    
    
    // decide traj_pva_in_index & length_step & angle_step, fill TrajP via2in if in is not on via
    double traj_piece_ideal_via2in = delta_joint_max_via2in * stack[S_PathCountFactorJoint];
    if(traj_piece_ideal_via2in < DOUBLE_ACCURACY)
    {
        traj_pva_in_index = 0;        
    }
    else
    {
        double joint_step_via2in;
        int traj_p_address = S_TrajP0;
        traj_pva_in_index = ceil(traj_piece_ideal_via2in);
        for(i = 0; i < model.link_num; ++i)
        {
            joint_step_via2in = (path_cache.cache[path_cache.smooth_in_index].joint[i] - via.joint_target[i]) / traj_pva_in_index;
            for(j = 0; j <= traj_pva_in_index; ++j)
            {
                stack[traj_p_address + j] = via.joint_target[i] + j * joint_step_via2in;
            }
            traj_p_address += 75;
        }       
    }
}


inline void updateMovLIn2EndTrajP(const PathCache& path_cache, int traj_pva_in_index, 
                                        int* traj_path_cache_index_in2end, int& traj_pva_out_index, int& traj_pva_size_via2end)
{
    int i, j;
    int path_cache_length_minus_1 = path_cache.cache_length - 1;
    double path_length_in2end = getPointsDistance(path_cache.cache[path_cache.smooth_in_index].pose.point_, path_cache.cache[path_cache_length_minus_1].pose.point_);
    double traj_piece_ideal_in2end = path_length_in2end * stack[S_PathCountFactorCartesian];
    if(path_cache.smooth_out_index == -1
        || path_cache.smooth_out_index == path_cache_length_minus_1)
    {
        getTrajPFromPathIn2End(path_cache, traj_piece_ideal_in2end, traj_pva_in_index, traj_path_cache_index_in2end, traj_pva_out_index, traj_pva_size_via2end);
    }
    else
    {
        getTrajPFromPathIn2Out2End(path_cache, traj_piece_ideal_in2end, traj_pva_in_index, traj_path_cache_index_in2end, traj_pva_out_index, traj_pva_size_via2end);
    }
}

inline void updateMovJIn2EndTrajP(const PathCache& path_cache, int traj_pva_in_index, 
                                        int* traj_path_cache_index_in2end, int& traj_pva_out_index, int& traj_pva_size_via2end)
{
    int i, j;
    int path_cache_length_minus_1 = path_cache.cache_length - 1;
    double delta_joint_max_in2end = 0;
    for(int i = 0; i < model.link_num; ++i)
    {
        stack[S_DeltaJointVector + i] = fabs(path_cache.cache[path_cache_length_minus_1].joint[i] - path_cache.cache[path_cache.smooth_in_index].joint[i]);
        if(stack[S_DeltaJointVector + i] > delta_joint_max_in2end)
        {
            delta_joint_max_in2end = stack[S_DeltaJointVector + i];
        }
    }    
    double traj_piece_ideal_in2end = delta_joint_max_in2end * stack[S_PathCountFactorJoint];
    if(path_cache.smooth_out_index == -1
        || path_cache.smooth_out_index == path_cache_length_minus_1)
    {
        getTrajPFromPathIn2End(path_cache, traj_piece_ideal_in2end, traj_pva_in_index, traj_path_cache_index_in2end, traj_pva_out_index, traj_pva_size_via2end);
    }
    else
    {
        getTrajPFromPathIn2Out2End(path_cache, traj_piece_ideal_in2end, traj_pva_in_index, traj_path_cache_index_in2end, traj_pva_out_index, traj_pva_size_via2end);
    }    
}

inline void updateMovLTrajT(const PathCache& path_cache, double cmd_vel,
                                int* traj_path_cache_index, int traj_pva_out_index, int traj_pva_size,
                                int& traj_t_size)
{
    int i;
    // compute total time
    int path_cache_length_minus_1 = path_cache.cache_length - 1;
    double path_length_start2end = getPointsDistance(path_cache.cache[0].pose.point_, path_cache.cache[path_cache_length_minus_1].pose.point_);
    double critical_length = cmd_vel * cmd_vel / segment_alg_param.max_cartesian_acc;
    double time_span_start2end;
    if(path_length_start2end > critical_length) // can reach vel
    {
        time_span_start2end = 2 * sqrt(critical_length / segment_alg_param.max_cartesian_acc) + (path_length_start2end - critical_length) / cmd_vel;
    }
    else    // can't reach vel
    {
        time_span_start2end = 2 * sqrt(path_length_start2end / segment_alg_param.max_cartesian_acc);
    }

    // compute time duration for each traj piece
    traj_t_size = traj_pva_size - 1;
    if(path_cache.smooth_out_index == -1
        || path_cache.smooth_out_index == path_cache_length_minus_1)
    {
        double time_duration_start2end = time_span_start2end / traj_t_size;
        for(i = 0; i < traj_t_size; ++i)
        {
            stack[S_TrajT + i] = time_duration_start2end;
        }        
    }
    else
    {
        double time_duration_start2out = path_cache.smooth_out_index * time_span_start2end / (path_cache_length_minus_1 * traj_pva_out_index);
        double time_duration_out2end = (path_cache_length_minus_1 - path_cache.smooth_out_index) * time_span_start2end / (path_cache_length_minus_1 * (traj_pva_size - traj_pva_out_index - 1));
        for(i = 0; i < traj_pva_out_index; ++i)
        {
            stack[S_TrajT + i] = time_duration_start2out;
        }
        for(; i < traj_t_size; ++i)
        {
            stack[S_TrajT + i] = time_duration_out2end;
        }
    }
    // adjust first and last piece of time
    stack[S_TrajT] = segment_alg_param.time_factor_first * stack[S_TrajT];
    stack[S_TrajT + traj_t_size - 1] = segment_alg_param.time_factor_last * stack[S_TrajT + traj_t_size - 1];
}

inline void updateMovJTrajT(const PathCache& path_cache, double cmd_vel, 
                                int* traj_path_cache_index, int traj_pva_out_index, int traj_pva_size, 
                                int& traj_t_size)
{
    int i;
    int path_cache_length_minus_1 = path_cache.cache_length - 1;
    // get max time span of all axes
    double time_span_start2end_max = 0;
    double time_span_start2end;
    for(i = 0; i < model.link_num; ++i)
    {
        time_span_start2end = stack[S_DeltaJointVector + i] / (cmd_vel * stack[S_ConstraintJointVelMax + i]);
        if(time_span_start2end > time_span_start2end_max)
        {
            time_span_start2end_max = time_span_start2end;
        }
    }

    // compute time duration of each traj piece
    traj_t_size = traj_pva_size - 1;
    if(path_cache.smooth_out_index == -1
        || path_cache.smooth_out_index == path_cache_length_minus_1)
    {    
        double time_duration_start2end = time_span_start2end_max / traj_t_size;
        for(i = 0; i <traj_t_size; ++i)
        {
            stack[S_TrajT + i] = time_duration_start2end;
        }
    }
    else
    {
        double time_duration_start2out = path_cache.smooth_out_index * time_span_start2end_max / (path_cache_length_minus_1 * traj_pva_out_index);
        double time_duration_out2end = (path_cache_length_minus_1 - path_cache.smooth_out_index) * time_span_start2end_max / (path_cache_length_minus_1 * (traj_pva_size - traj_pva_out_index - 1));
        for(i = 0; i < traj_pva_out_index; ++i)
        {
            stack[S_TrajT + i] = time_duration_start2out;
        }
        for(; i < traj_t_size; ++i)
        {
            stack[S_TrajT + i] = time_duration_out2end;
        }
    }
    // adjust first and last piece of time
    stack[S_TrajT] = segment_alg_param.time_factor_first * stack[S_TrajT];
    stack[S_TrajT + traj_t_size - 1] = segment_alg_param.time_factor_last * stack[S_TrajT + traj_t_size - 1];
}

inline void updateSmoothOut2InTrajP(const PathCache& path_cache, const MotionTarget& via, int* traj_path_cache_index_out2in, int& traj_pva_size_out2in)
{
    double traj_piece_ideal_out2in;
    switch(path_cache.target.type)
    {
        case MOTION_LINE:
        {
            double path_length_out2via = getPointsDistance(path_cache.cache[0].pose.point_, via.pose_target.point_);
            double path_length_via2in = getPointsDistance(via.pose_target.point_, path_cache.cache[path_cache.smooth_in_index].pose.point_);
            traj_piece_ideal_out2in = (path_length_out2via + path_length_via2in) * stack[S_PathCountFactorCartesian];
            break;
        }
        case MOTION_JOINT:
        {
            double delta_joint_max_out2in = 0;
            double delta_joint_out2in;
            int out_path_index;
            if(path_cache.smooth_out_index == -1)
            {
                out_path_index = path_cache.cache_length - 1;
            }
            else
            {
                out_path_index = path_cache.smooth_out_index;
            }
                
            for(int i = 0; i < model.link_num; ++i)
            {
                delta_joint_out2in = fabs(path_cache.cache[path_cache.smooth_in_index].joint[i] - path_cache.cache[out_path_index].joint[i]);
                if(delta_joint_out2in > delta_joint_max_out2in)
                {
                    delta_joint_max_out2in = delta_joint_out2in;
                }
            }    
            traj_piece_ideal_out2in = delta_joint_max_out2in * stack[S_PathCountFactorJoint];
            break;
        }
    }
    getTrajPFromPathOut2In(path_cache, traj_piece_ideal_out2in, traj_path_cache_index_out2in, traj_pva_size_out2in);
}

inline void updateMovLVia2EndTrajT(const PathCache& path_cache, const MotionTarget& via, double cmd_vel,
                                   int* traj_path_cache_index_in2end, int traj_pva_in_index, int traj_pva_out_index, int traj_pva_size_via2end,
                                   int& traj_t_size)
{
    int i;
    // compute time span
    int path_cache_length_minus_1 = path_cache.cache_length - 1;
    double path_length_via2end = getPointsDistance(via.pose_target.point_, path_cache.cache[path_cache_length_minus_1].pose.point_);
    double critical_length = cmd_vel * cmd_vel / segment_alg_param.max_cartesian_acc;
    double time_span_via2end;
    if(path_length_via2end > critical_length) // can reach vel
    {
        time_span_via2end = 2 * sqrt(critical_length / segment_alg_param.max_cartesian_acc) + (path_length_via2end - critical_length) / cmd_vel;
    }
    else    // can't reach vel
    {
        time_span_via2end = 2 * sqrt(path_length_via2end / segment_alg_param.max_cartesian_acc);
    }
    // compute time duration for each traj piece, via2in
    double path_length_via2in = getPointsDistance(via.pose_target.point_, path_cache.cache[path_cache.smooth_in_index].pose.point_);
    double time_span_via2in = time_span_via2end * path_length_via2in / path_length_via2end;    
    double time_duration_via2in = time_span_via2in / traj_pva_in_index;
    for(i = 0; i < traj_pva_in_index; ++i)
    {
        stack[S_TrajT + i] = time_duration_via2in;
    }
    // compute time duration for each traj piece, in2end
    traj_t_size = traj_pva_size_via2end - 1;
    if(path_cache.smooth_out_index == -1
        || path_cache.smooth_out_index == path_cache_length_minus_1)
    {
        double time_duration_in2end = (time_span_via2end - time_span_via2in) / (traj_t_size - traj_pva_in_index);
        for(i = traj_pva_in_index; i < traj_t_size; ++i)
        {
            stack[S_TrajT + i] = time_duration_in2end;
        }        
    }
    else
    {
        double path_length_in2out = getPointsDistance(path_cache.cache[path_cache.smooth_in_index].pose.point_, path_cache.cache[path_cache.smooth_out_index].pose.point_);
        double time_span_in2out = time_span_via2end * path_length_in2out / path_length_via2end;
        double time_duration_in2out = time_span_in2out / (traj_pva_out_index - traj_pva_in_index);
        double path_length_out2end = getPointsDistance(path_cache.cache[path_cache.smooth_out_index].pose.point_, path_cache.cache[path_cache_length_minus_1].pose.point_);
        double time_span_out2end = time_span_via2end * path_length_out2end / path_length_via2end;
        double time_duration_out2end = time_span_out2end / (traj_pva_size_via2end - traj_pva_out_index - 1);

        for(i = traj_pva_in_index; i < traj_pva_out_index; ++i)
        {
            stack[S_TrajT + i] = time_duration_in2out;
        }
        for(i = traj_pva_out_index; i < traj_t_size; ++i)
        {
            stack[S_TrajT + i] = time_duration_out2end;
        }
    }
    // adjust first and last piece of time
    stack[S_TrajT] = segment_alg_param.time_factor_first * stack[S_TrajT];
    stack[S_TrajT + traj_t_size - 1] = segment_alg_param.time_factor_last * stack[S_TrajT + traj_t_size - 1];
}

inline void updateMovJVia2EndTrajT(const PathCache& path_cache, const MotionTarget& via, double cmd_vel,
                                  int* traj_path_cache_index_in2end, int traj_pva_in_index, int traj_pva_out_index, int traj_pva_size_via2end,
                                  int& traj_t_size)
{
    int i;
    int path_cache_length_minus_1 = path_cache.cache_length - 1;
    // get max delta joint & max time span of all axes
    double time_span_via2end_max = 0;
    double time_span_via2end;
    double delta_joint_via2end_max = 0;
    double delta_joint_via2end;
    int delta_joint_max_id = 0;
    for(i = 0; i < model.link_num; ++i)
    {
        delta_joint_via2end = fabs(path_cache.cache[path_cache_length_minus_1].joint[i] - via.joint_target[i]);
        if(delta_joint_via2end > delta_joint_via2end_max)
        {
            delta_joint_via2end_max = delta_joint_via2end;
            delta_joint_max_id = i;
        }
        time_span_via2end = delta_joint_via2end / (cmd_vel * stack[S_ConstraintJointVelMax + i]);
        if(time_span_via2end > time_span_via2end_max)
        {
            time_span_via2end_max = time_span_via2end;
        }
    }

    // compute time duration for each traj piece, via2in
    double delta_joint_via2in_max = fabs(path_cache.cache[path_cache.smooth_in_index].joint[delta_joint_max_id] - via.joint_target[delta_joint_max_id]);
    double time_span_via2in = delta_joint_via2in_max * time_span_via2end_max / delta_joint_via2end_max;
    double time_duration_via2in = time_span_via2in / traj_pva_in_index;
    for(i = 0; i < traj_pva_in_index; ++i)
    {
        stack[S_TrajT + i] = time_duration_via2in;
    }

    // compute time duration for each traj piece, in2end
    traj_t_size = traj_pva_size_via2end - 1;
    if(path_cache.smooth_out_index == -1
        || path_cache.smooth_out_index == path_cache_length_minus_1)
    {
        double time_duration_in2end = (time_span_via2end_max - time_span_via2in) / (traj_t_size - traj_pva_in_index);
        for(i = traj_pva_in_index; i < traj_t_size; ++i)
        {
            stack[S_TrajT + i] = time_duration_in2end;
        }        
    }
    else
    {
        double delta_joint_in2out_max = fabs(path_cache.cache[path_cache.smooth_in_index].joint[delta_joint_max_id] - path_cache.cache[path_cache.smooth_out_index].joint[delta_joint_max_id]);
        double time_span_in2out = delta_joint_in2out_max * time_span_via2end_max / delta_joint_via2end_max;
        double time_duration_in2out = time_span_in2out / (traj_pva_out_index - traj_pva_in_index);
        double delta_joint_out2end_max = fabs(path_cache.cache[path_cache.smooth_out_index].joint[delta_joint_max_id] - path_cache.cache[path_cache_length_minus_1].joint[delta_joint_max_id]);
        double time_span_out2end = delta_joint_out2end_max * time_span_via2end_max / delta_joint_via2end_max;
        double time_duration_out2end = time_span_out2end / (traj_pva_size_via2end - traj_pva_out_index - 1);        
        for(i = traj_pva_in_index; i < traj_pva_out_index; ++i)
        {
            stack[S_TrajT + i] = time_duration_in2out;
        }
        for(i = traj_pva_out_index; i < traj_t_size; ++i)
        {
            stack[S_TrajT + i] = time_duration_out2end;
        }
    }

    // adjust first and last piece of time
    stack[S_TrajT] = segment_alg_param.time_factor_first * stack[S_TrajT];
    stack[S_TrajT + traj_t_size - 1] = segment_alg_param.time_factor_last * stack[S_TrajT + traj_t_size - 1];
}

inline void updateSmoothOut2InTrajT(const PathCache& path_cache, const MotionTarget& via, double cmd_vel, 
                                           int* traj_path_cache_index_out2in, int traj_pva_size_out2in, 
                                           int& traj_t_size_out2in)
{
    traj_t_size_out2in = traj_pva_size_out2in - 1;
    double time_span_out2in;
    switch(path_cache.target.type)
    {
        case MOTION_LINE:
        {
            double path_length_out2via = getPointsDistance(path_cache.cache[0].pose.point_, via.pose_target.point_);
            double path_length_via2in = getPointsDistance(via.pose_target.point_, path_cache.cache[path_cache.smooth_in_index].pose.point_);
            time_span_out2in = (path_length_out2via + path_length_via2in) / cmd_vel;
            break;
        }
        case MOTION_JOINT:
        {            
            time_span_out2in = 0;
            double time_span_out2in_tmp;
            int out_path_index;
            if(path_cache.smooth_out_index == -1)
            {
                out_path_index = path_cache.cache_length - 1;
            }
            else
            {
                out_path_index = path_cache.smooth_out_index;
            }
            for(int i = 0; i < model.link_num; ++i)
            {
                time_span_out2in_tmp = fabs(path_cache.cache[path_cache.smooth_in_index].joint[i] - path_cache.cache[out_path_index].joint[i]) / (cmd_vel * stack[S_ConstraintJointVelMax + i]);
                if(time_span_out2in_tmp > time_span_out2in)
                {
                    time_span_out2in = time_span_out2in_tmp;
                }
            }
            break;
        }
    }

    double time_duration_out2in = time_span_out2in / traj_t_size_out2in;  
    for(int i = 0; i < traj_t_size_out2in; ++i)
    {
        stack[S_TrajT_Smooth + i] = time_duration_out2in;
    }
}

inline void getJerkStart(double* traj_p_base, int traj_pva_size, double* traj_t_base, int traj_t_size, double* start_state, double a2,
                             double& jerk1, double& jerk2)
{
    double duration_square = traj_t_base[0] * traj_t_base[0];
    double duration_cubic = duration_square * traj_t_base[0];
    jerk1 = 12 * (traj_p_base[1] - traj_p_base[0]) / duration_cubic
            - 12 * start_state[1] / duration_square
            - 5 * start_state[2] / traj_t_base[0]
            - a2 / traj_t_base[0];
    jerk2 = -12 * (traj_p_base[1] - traj_p_base[0]) / duration_cubic
            + 12 * start_state[1] / duration_square
            + 3 * start_state[2] / traj_t_base[0]
            + 3 * a2 / traj_t_base[0];
}

inline void getJerkEnd(double* traj_p_base, int traj_pva_size, double* traj_t_base, int traj_t_size, double* end_state, double an_1,
                          double& jerkn_1, double& jerkn)
{
    double duration_square = traj_t_base[traj_t_size - 1] * traj_t_base[traj_t_size - 1];
    double duration_cubic = duration_square * traj_t_base[traj_t_size - 1];
    jerkn_1 = -12 * (traj_p_base[traj_pva_size - 1] - traj_p_base[traj_pva_size - 2]) / duration_cubic
              + 12 * end_state[1] / duration_square
              - 3 * (an_1 + end_state[2]) / traj_t_base[traj_t_size - 1];
    jerkn = 12 * (traj_p_base[traj_pva_size - 1] - traj_p_base[traj_pva_size - 2]) / duration_cubic
            - 12 * end_state[1] / duration_square
            + (an_1 + 5 * end_state[2]) / traj_t_base[traj_t_size - 1];             
}

inline void updateTrajVA(double* traj_p_base, double* traj_v_base, double* traj_a_base, int traj_pva_size, 
                             double* traj_j_base, double* traj_t_base, int traj_t_size,
                             double* start_state, double* end_state)
{
    // start point
    traj_a_base[0] = start_state[2];
    traj_v_base[0] = start_state[1];

    traj_a_base[1] = stack[S_X];
    traj_v_base[1] = start_state[1] + (start_state[2] + ((traj_j_base[1] - traj_j_base[0]) / 6 + traj_j_base[0] / 2) * traj_t_base[0]) * traj_t_base[0];    

    // traj_pva_size - 1 = traj_t_size
    for(int i = 2; i < traj_t_size; ++i)
    {
        traj_a_base[i] = stack[S_X + i - 1];
        traj_v_base[i] = traj_v_base[i - 1] + (traj_a_base[i] + traj_a_base[i - 1]) * traj_t_base[i - 1] / 2;
    }

    // end point
    traj_a_base[traj_t_size] = end_state[2];
    traj_v_base[traj_t_size] = end_state[1];    
}

inline void updateConstraintJoint(int traj_p_address, int traj_v_address, int traj_pva_size)
{
    int i, j;
    int constraint_joint_pos_acc_address, constraint_joint_neg_acc_address;
    if(segment_alg_param.is_fake_dynamics)
    {
        for(i = 0; i < traj_pva_size; ++i)
        {
            constraint_joint_pos_acc_address = S_ConstraintJointPosA0;
            constraint_joint_neg_acc_address = S_ConstraintJointNegA0;
            for(j = 0; j < model.link_num; ++j)
            {
                stack[constraint_joint_pos_acc_address + i] = 100;
                stack[constraint_joint_neg_acc_address + i] = -100;
                constraint_joint_pos_acc_address += 50;
                constraint_joint_neg_acc_address += 50;
            }
        }
    }
    else
    {
        float joint[6], omega[6], alpha[2][6];
        int traj_p_address_local, traj_v_address_local;
        for(i = 0; i < traj_pva_size; ++i)
        {
            traj_p_address_local = traj_p_address;
            traj_v_address_local = traj_v_address;
            for(j = 0; j < 6; ++j)
            {
                joint[j] = (float)stack[traj_p_address_local + i];
                omega[j] = (float)stack[traj_p_address_local + i];
                traj_p_address_local += 75;
                traj_p_address_local += 75;
            }
            segment_alg_param.dynamics_ptr->computeAccMax(joint, omega, alpha);
            constraint_joint_pos_acc_address = S_ConstraintJointPosA0;
            constraint_joint_neg_acc_address = S_ConstraintJointNegA0;
            for(j = 0; j < 6; ++j)
            {
                stack[constraint_joint_pos_acc_address + i] = (double)alpha[0][j];
                stack[constraint_joint_neg_acc_address + i] = (double)alpha[1][j];
                constraint_joint_pos_acc_address += 50;
                constraint_joint_neg_acc_address += 50;
            }
        }     
    }
}

inline void updateTrajPieceA(int traj_a_address, int traj_pva_size, double acc_ratio)
{
    int i, j;
    int traj_a_address_local, traj_piece_a_address, constraint_joint_pos_acc_address, constraint_joint_neg_acc_address;
    int traj_piece_size = traj_pva_size - 1;
    int traj_piece_size_half = (traj_piece_size >> 1);
    // first half
    for(i = 0; i < /*traj_piece_size_half*/(traj_piece_size - 1); ++i)
    {
        traj_a_address_local = traj_a_address;
        traj_piece_a_address = S_TrajPieceA0;
        constraint_joint_pos_acc_address = S_ConstraintJointPosA0;
        constraint_joint_neg_acc_address = S_ConstraintJointNegA0;
        for(j = 0; j < model.link_num; ++j)
        {        
            if(stack[traj_a_address_local + i + 1] >= 0)
            {
                if(stack[traj_a_address_local + i + 1] > stack[constraint_joint_pos_acc_address + i + 1])
                {
                    stack[traj_piece_a_address + i] = stack[traj_a_address_local + i + 1] / (stack[constraint_joint_pos_acc_address + i + 1] * acc_ratio);
                }
                else
                {
                    stack[traj_piece_a_address + i] = 1;
                }
            }
            else
            {
                if(stack[traj_a_address_local + i + 1] < stack[constraint_joint_neg_acc_address + i + 1])
                {
                    stack[traj_piece_a_address + i] = stack[traj_a_address_local + i + 1] / (stack[constraint_joint_neg_acc_address + i + 1] * acc_ratio);
                }
                else
                {
                    stack[traj_piece_a_address + i] = 1;
                }
            }
            
            traj_a_address_local += 75;
            traj_piece_a_address += 25;
            constraint_joint_pos_acc_address += 50;
            constraint_joint_neg_acc_address += 50;
        }
    }
    // second half
    for(; i < traj_piece_size; ++i)
    {
        traj_a_address_local = traj_a_address;
        traj_piece_a_address = S_TrajPieceA0;
        constraint_joint_pos_acc_address = S_ConstraintJointPosA0;
        constraint_joint_neg_acc_address = S_ConstraintJointNegA0;
        for(j = 0; j < model.link_num; ++j)
        {        
            if(stack[traj_a_address_local + i] >= 0)
            {
                if(stack[traj_a_address_local + i] > stack[constraint_joint_pos_acc_address + i])
                {
                    stack[traj_piece_a_address + i] = stack[traj_a_address_local + i] / (stack[constraint_joint_pos_acc_address + i] * acc_ratio);
                }
                else
                {
                    stack[traj_piece_a_address + i] = 1;
                }
            }
            else
            {
                if(stack[traj_a_address_local + i] < stack[constraint_joint_neg_acc_address + i])
                {
                    stack[traj_piece_a_address + i] = stack[traj_a_address_local + i] / (stack[constraint_joint_neg_acc_address + i] * acc_ratio);
                }
                else
                {
                    stack[traj_piece_a_address + i] = 1;
                }
            }
            
            traj_a_address_local += 75;
            traj_piece_a_address += 25;
            constraint_joint_pos_acc_address += 50;
            constraint_joint_neg_acc_address += 50;
        }
    }    
}

inline void updateTrajPieceV(int traj_v_address, int traj_a_address, int traj_pva_size, int traj_t_address, double vel_ratio)
{
    int traj_v_address_local, traj_a_address_local, traj_piece_v_address;
    int traj_piece_size = traj_pva_size - 1;
    double acc_product;
    for(int i = 0; i < traj_piece_size; ++i)
    {
        traj_v_address_local = traj_v_address;
        traj_a_address_local = traj_a_address;
        traj_piece_v_address = S_TrajPieceV0;
        for(int j = 0; j < model.link_num; ++j)
        {
            acc_product = stack[traj_a_address_local + i] * stack[traj_a_address_local + i + 1];
            if(acc_product >= 0)
            {
                if(fabs(stack[traj_a_address_local + i]) > fabs(stack[traj_a_address_local + i + 1]))
                {
                    stack[traj_piece_v_address + i] = fabs(stack[traj_v_address_local + i]) / (stack[S_ConstraintJointVelMax + j] * vel_ratio);
                }
                else
                {
                    stack[traj_piece_v_address + i] = fabs(stack[traj_v_address_local + i + 1]) / (stack[S_ConstraintJointVelMax + j] * vel_ratio);
                }
            }
            else
            {
                if(stack[traj_a_address_local + i] >= 0)
                {
                    stack[traj_piece_v_address + i] = fabs(stack[traj_v_address_local + i] 
                                                    + stack[traj_a_address_local + i] * stack[traj_a_address_local + i] * stack[traj_t_address + i] 
                                                    / (2 * (stack[traj_a_address_local + i] - stack[traj_a_address_local + i + 1])))
                                                    / (stack[S_ConstraintJointVelMax + j] * vel_ratio);
                }
                else
                {
                    stack[traj_piece_v_address + i] = fabs(stack[traj_v_address_local + i] 
                                                    - stack[traj_a_address_local + i] * stack[traj_a_address_local + i] * stack[traj_t_address + i] 
                                                    / (2 * (stack[traj_a_address_local + i + 1] - stack[traj_a_address_local + i])))
                                                    / (stack[S_ConstraintJointVelMax + j] * vel_ratio);
                }
            }
            traj_v_address_local += 75;
            traj_a_address_local += 75;
            traj_piece_v_address += 25;
        }
    }
}

inline double getMaxOfAllAxes(int traj_piece_address)
{
    double max_value = 1;
    for(int i = 0; i < model.link_num; ++i)
    {
        if(stack[traj_piece_address] > max_value)
        {
            max_value = stack[traj_piece_address];
            traj_piece_address += 25;
        }
    }
    return max_value;
}

inline void updateTrajPieceRescaleFactor(int traj_piece_size)
{
    double traj_piece_v_max, traj_piece_a_max, traj_piece_v_max_sqrt;
    for(int i= 0 ; i < traj_piece_size; ++i)
    {
        traj_piece_a_max = getMaxOfAllAxes(S_TrajPieceA0 + i);        
        traj_piece_v_max = getMaxOfAllAxes(S_TrajPieceV0 + i);
        traj_piece_v_max_sqrt = sqrt(traj_piece_v_max);
        stack[S_TrajPieceRescaleFactor + i] = ((traj_piece_v_max_sqrt > traj_piece_a_max) ? traj_piece_v_max_sqrt : traj_piece_a_max);
    }
}

inline void updateTrajTByPieceRescaleFactor(int traj_t_address, int traj_t_size)
{
    for(int i = 0; i < traj_t_size; ++i)
    {
        stack[traj_t_address + i] = stack[S_TrajPieceRescaleFactor + i] * stack[traj_t_address + i];
    }
}

inline void updateOutAndInPointState(const JointState& out_state, int traj_pva_in_index)
{
    int out_point_state_address = S_OutPointState0;
    int in_point_state_address = S_InPointState0;
    int traj_p_address = S_TrajP0;
    int traj_v_address = S_TrajV0;
    int traj_a_address = S_TrajA0;
    for(int i = 0; i < model.link_num; ++i)
    {
        stack[out_point_state_address] = out_state.angle[i];
        stack[out_point_state_address + 1] = out_state.omega[i];
        stack[out_point_state_address + 2] = out_state.alpha[i];
        stack[in_point_state_address] = stack[traj_p_address + traj_pva_in_index];
        stack[in_point_state_address + 1] = stack[traj_v_address + traj_pva_in_index];
        stack[in_point_state_address + 2] = stack[traj_a_address + traj_pva_in_index];
        out_point_state_address += 3;
        in_point_state_address += 3;
        traj_p_address += 75;
        traj_v_address += 75;
        traj_a_address += 75;
    }
}

inline bool isRescaleNeeded(int traj_piece_size)
{
    double max_rescale_factor = 1;
    for(int i = 0; i < traj_piece_size; ++i)
    {
        if(stack[S_TrajPieceRescaleFactor + i ] > max_rescale_factor)
        {
            max_rescale_factor = stack[S_TrajPieceRescaleFactor + i ];
        }
    }
    return (max_rescale_factor > (1 + DOUBLE_ACCURACY)) ? true : false;
}

inline void updateTrajCoeff(int traj_p_address, int traj_v_address, int traj_a_address, int traj_pva_size, 
                                int traj_t_address, int traj_t_size, int traj_j_address, int traj_coeff_address)
{
    int traj_t_size_minus_1 = traj_t_size - 1;
    int traj_coeff_address_local = traj_coeff_address;
    for(int i = 0; i < model.link_num; ++i)
    {
        // first piece, quatern
        stack[traj_coeff_address] = stack[traj_p_address];
        stack[traj_coeff_address + 25] = stack[traj_v_address];
        stack[traj_coeff_address + 50] = stack[traj_a_address] / 2;
        stack[traj_coeff_address + 75] = stack[traj_j_address] / 6;
        stack[traj_coeff_address + 100] = (stack[traj_j_address + 1] - stack[traj_j_address]) / (24 * stack[traj_t_address]);
        stack[traj_coeff_address + 125] = 0;
        // middle pieces, cubic
        for(int j = 1; j < traj_t_size_minus_1; ++j)
        {            
            stack[traj_coeff_address + j] = stack[traj_p_address + j];
            stack[traj_coeff_address + 25 + j] = stack[traj_v_address + j];
            stack[traj_coeff_address + 50 + j] = stack[traj_a_address + j] / 2;
            stack[traj_coeff_address + 75 + j] = (stack[traj_a_address + j + 1] - stack[traj_a_address + j]) / (6 * stack[traj_t_address + j]);
            stack[traj_coeff_address + 100 + j] = 0;
            stack[traj_coeff_address + 125 + j] = 0;
        }
        // last piece, quatern
        stack[traj_coeff_address + traj_t_size_minus_1] = stack[traj_p_address + traj_t_size_minus_1];
        stack[traj_coeff_address + traj_t_size_minus_1 + 25] = stack[traj_v_address + traj_t_size_minus_1];
        stack[traj_coeff_address + traj_t_size_minus_1 + 50] = stack[traj_a_address + traj_t_size_minus_1] / 2;
        stack[traj_coeff_address + traj_t_size_minus_1 + 75] = stack[traj_j_address + 2] / 6;
        stack[traj_coeff_address + traj_t_size_minus_1 + 100] = (stack[traj_j_address + 3] - stack[traj_j_address + 2]) / (24 * stack[traj_t_address + traj_t_size_minus_1]);
        stack[traj_coeff_address + traj_t_size_minus_1 + 125] = 0;
        // prepare address for next axis
        traj_coeff_address += 150;
        traj_p_address += 75;
        traj_v_address += 75;
        traj_a_address += 75;
        traj_j_address += 4;
    }
}

inline void packTrajCache(int* traj_path_cache_index, int traj_pva_out_index, int traj_pva_size, 
                              int traj_coeff_address, int traj_t_address, int traj_t_size, TrajectoryCache& traj_cache)
{
    int traj_coeff_address_local = traj_coeff_address;
    traj_cache.cache_length = traj_t_size;
    if(traj_pva_out_index == -1)
    {
        traj_cache.smooth_out_index = -1;
    }
    else
    {
        traj_cache.smooth_out_index = traj_pva_out_index - 1;
    }
    
    for(int i = 0; i < traj_t_size; ++i)
    {
        traj_cache.cache[i].index_in_path_cache = traj_path_cache_index[i + 1];
        traj_cache.cache[i].duration = stack[traj_t_address + i];
        traj_coeff_address_local = traj_coeff_address;
        for(int j = 0; j < model.link_num; ++j)
        {
            traj_cache.cache[i].axis[j].data[0] = stack[traj_coeff_address_local + i];        // A0
            traj_cache.cache[i].axis[j].data[1] = stack[traj_coeff_address_local + i + 25];   // A1
            traj_cache.cache[i].axis[j].data[2] = stack[traj_coeff_address_local + i + 50];   // A2
            traj_cache.cache[i].axis[j].data[3] = stack[traj_coeff_address_local + i + 75];   // A3
            traj_cache.cache[i].axis[j].data[4] = stack[traj_coeff_address_local + i + 100];  // A4
            traj_cache.cache[i].axis[j].data[5] = stack[traj_coeff_address_local + i + 125];  // A5
            traj_coeff_address_local += 150;
        }        
    }
}

inline void packTrajCacheSmooth(int* traj_path_cache_index_out2in, int traj_pva_size_out2in, int traj_coeff_address_out2in, int traj_t_address_out2in, int traj_t_size_out2in, 
                                      int* traj_path_cache_index_in2end, int traj_pva_size_via2end, int traj_coeff_address_via2end, int traj_t_address_via2end, int traj_t_size_via2end,
                                      int traj_pva_in_index, int traj_pva_out_index,
                                      TrajectoryCache& traj_cache)
{
    int i;
    int traj_coeff_address_local;
    traj_cache.cache_length = traj_t_size_out2in + traj_t_size_via2end - traj_pva_in_index;
    if(traj_pva_out_index == -1)
    {
        traj_cache.smooth_out_index = -1;
    }
    else
    {
        traj_cache.smooth_out_index = traj_t_size_out2in + traj_pva_out_index - traj_pva_in_index - 1;
    }
    for(i = 0; i < traj_t_size_out2in; ++i)
    {
        traj_cache.cache[i].index_in_path_cache = traj_path_cache_index_out2in[i + 1];
        traj_cache.cache[i].duration = stack[traj_t_address_out2in + i];
        traj_coeff_address_local = traj_coeff_address_out2in;
        for(int j = 0; j < model.link_num; ++j)
        {
            traj_cache.cache[i].axis[j].data[0] = stack[traj_coeff_address_local + i];        // A0
            traj_cache.cache[i].axis[j].data[1] = stack[traj_coeff_address_local + i + 25];   // A1
            traj_cache.cache[i].axis[j].data[2] = stack[traj_coeff_address_local + i + 50];   // A2
            traj_cache.cache[i].axis[j].data[3] = stack[traj_coeff_address_local + i + 75];   // A3
            traj_cache.cache[i].axis[j].data[4] = stack[traj_coeff_address_local + i + 100];  // A4
            traj_cache.cache[i].axis[j].data[5] = stack[traj_coeff_address_local + i + 125];  // A5
            traj_coeff_address_local += 150;
        } 
    }

    int traj_t_size_in2end = traj_t_size_via2end - traj_pva_in_index;
    for(i = 0; i < traj_t_size_in2end; ++i)
    {
        traj_cache.cache[traj_t_size_out2in + i].index_in_path_cache = traj_path_cache_index_in2end[i + 1];
        traj_cache.cache[traj_t_size_out2in + i].duration = stack[traj_t_address_via2end + traj_pva_in_index + i];
        traj_coeff_address_local = traj_coeff_address_via2end + traj_pva_in_index;
        for(int j = 0; j < model.link_num; ++j)
        {
            traj_cache.cache[traj_t_size_out2in + i].axis[j].data[0] = stack[traj_coeff_address_local + i];        // A0
            traj_cache.cache[traj_t_size_out2in + i].axis[j].data[1] = stack[traj_coeff_address_local + i + 25];   // A1
            traj_cache.cache[traj_t_size_out2in + i].axis[j].data[2] = stack[traj_coeff_address_local + i + 50];   // A2
            traj_cache.cache[traj_t_size_out2in + i].axis[j].data[3] = stack[traj_coeff_address_local + i + 75];   // A3
            traj_cache.cache[traj_t_size_out2in + i].axis[j].data[4] = stack[traj_coeff_address_local + i + 100];  // A4
            traj_cache.cache[traj_t_size_out2in + i].axis[j].data[5] = stack[traj_coeff_address_local + i + 125];  // A5
            traj_coeff_address_local += 150;
        } 
    }
}

void printTraj(TrajectoryCache &traj_cache, int index, double time_step, int end_segment)
{
    double absolute_time_vector[50];
    absolute_time_vector[0] = 0;

    for(int i = 1; i < traj_cache.cache_length + 1; ++i)
    {
        absolute_time_vector[i] = absolute_time_vector[i - 1] + traj_cache.cache[i - 1].duration;
    }
  
    int segment_index;
    double cur_time = 0;
    double delta_time = 0;
    double p_value, v_value, a_value;   
    while(cur_time < absolute_time_vector[end_segment])
    {
        for(segment_index = end_segment - 1; segment_index >= 0; --segment_index)
        {
            if(cur_time >= absolute_time_vector[segment_index])
            {
                break;
            }
        }

        delta_time = cur_time - absolute_time_vector[segment_index];
        p_value = traj_cache.cache[segment_index].axis[index].data[5] * delta_time * delta_time * delta_time * delta_time * delta_time
                  + traj_cache.cache[segment_index].axis[index].data[4] * delta_time * delta_time * delta_time * delta_time
                  + traj_cache.cache[segment_index].axis[index].data[3] * delta_time * delta_time * delta_time
                  + traj_cache.cache[segment_index].axis[index].data[2] * delta_time * delta_time
                  + traj_cache.cache[segment_index].axis[index].data[1] * delta_time
                  + traj_cache.cache[segment_index].axis[index].data[0];
        v_value = 5 * traj_cache.cache[segment_index].axis[index].data[5] * delta_time * delta_time * delta_time * delta_time
                  + 4 * traj_cache.cache[segment_index].axis[index].data[4] * delta_time * delta_time * delta_time
                  + 3 * traj_cache.cache[segment_index].axis[index].data[3] * delta_time * delta_time
                  + 2 * traj_cache.cache[segment_index].axis[index].data[2] * delta_time
                  + traj_cache.cache[segment_index].axis[index].data[1];
        a_value = 20 * traj_cache.cache[segment_index].axis[index].data[5] * delta_time * delta_time * delta_time
                  + 12 * traj_cache.cache[segment_index].axis[index].data[4] * delta_time * delta_time
                  + 6 * traj_cache.cache[segment_index].axis[index].data[3] * delta_time
                  + 2 * traj_cache.cache[segment_index].axis[index].data[2];

        std::cout<<segment_index<<" "<<cur_time<<" "<<p_value<<"  "<<v_value<<"  "<<a_value<<std::endl;
        cur_time += time_step;
    }


}


