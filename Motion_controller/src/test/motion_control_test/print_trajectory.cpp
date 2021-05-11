/*************************************************************************
	> File Name: output_trajectory.cpp
	> Author: 
	> Mail: 
	> Created Time: 2020年04月15日 星期三 16时55分03秒
 ************************************************************************/
#include <unistd.h>
#include <string.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <time.h>
#include <sys/file.h> 
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <basic_alg.h>
#include <motion_control_base_group.h>
#include <kinematics.h>
#include <kinematics_rtm.h>

using namespace std;
using namespace basic_alg;
using namespace group_space;

int main(int argc, char** argv)
{
	int fd = shm_open("rtm_trajectory", O_CREAT|O_RDWR, 00777);
    
    if (-1 == fd)
    {
        printf("Fail to create rtm_trajectory\n");
        return -1;
    }

    void *ptr = mmap(NULL, TRAJECTORY_LOG_CONTROL_SIZE + TRAJECTORY_LOG_DATA_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    
    if (ptr == MAP_FAILED)
    {
        close(fd);
        printf("Fail to map rtm_trajectory\n");
        return -1;
    }

    const TrajectoryLogControl *ctrl_ptr = static_cast<TrajectoryLogControl*>(ptr);
	TrajectoryPoint *points = reinterpret_cast<TrajectoryPoint*>((char*)ptr + TRAJECTORY_LOG_CONTROL_SIZE);
	printf("Model: %s\n", ctrl_ptr->model_name);
	printf("Number of points: %d/%d\n", ctrl_ptr->num_of_points, ctrl_ptr->max_of_points);
	basic_alg::Kinematics *kinematics_ptr;
	bool compute_fk = false;

	if (strcmp(ctrl_ptr->model_name, "RTM-P7A") == 0)
	{
		kinematics_ptr = new KinematicsRTM("/root/install/share/runtime/axis_group/");

		if (kinematics_ptr == NULL || !kinematics_ptr->isValid())
		{
			printf("Fail to create kinematics for model: %s\n", ctrl_ptr->model_name);
			return -1;
		}

		compute_fk = true;
	}
	else
	{
		printf("Fail to create kinematics for model: %s\n", ctrl_ptr->model_name);
	}

	uint32_t num_of_print = argc > 1 ? atoi(argv[1]) : ctrl_ptr->num_of_points;
	uint32_t index = ctrl_ptr->num_of_points < ctrl_ptr->max_of_points ? 0 : ctrl_ptr->write_index;
	num_of_print = num_of_print < ctrl_ptr->num_of_points ? num_of_print : ctrl_ptr->num_of_points;
	index += ctrl_ptr->num_of_points - num_of_print;
	index = index < ctrl_ptr->max_of_points ? index : (index - ctrl_ptr->max_of_points);
	printf("Start at %d, total %d\n", index, num_of_print);

	for (uint32_t i = 0; i < num_of_print; i++)
	{
		TrajectoryPoint &point = points[index];
		JointState &state = point.state;
		index = (index + 1 < ctrl_ptr->max_of_points) ? (index + 1) : 0;

		if (compute_fk)
		{
			PoseQuaternion pose;
			kinematics_ptr->doFK(state.angle, pose);
			cout << point.level << "," << point.time_stamp << ","
				<< state.angle[0] << "," << state.angle[1] << "," << state.angle[2] << "," << state.angle[3] << "," << state.angle[4] << "," << state.angle[5] << ","
				<< state.omega[0] << "," << state.omega[1] << "," << state.omega[2] << "," << state.omega[3] << "," << state.omega[4] << "," << state.omega[5] << ","
				<< state.alpha[0] << "," << state.alpha[1] << "," << state.alpha[2] << "," << state.alpha[3] << "," << state.alpha[4] << "," << state.alpha[5] << ","
				<< state.torque[0] << "," << state.torque[1] << "," << state.torque[2] << "," << state.torque[3] << "," << state.torque[4] << "," << state.torque[5] << ","
				<< pose.point_.x_ << "," << pose.point_.y_ << "," << pose.point_.z_ << "," << pose.quaternion_.x_ << "," << pose.quaternion_.y_ << "," << pose.quaternion_.z_ << "," << pose.quaternion_.w_ << endl;
		}
		else
		{
			cout << point.level << "," << point.time_stamp << ","
				<< state.angle[0] << "," << state.angle[1] << "," << state.angle[2] << "," << state.angle[3] << "," << state.angle[4] << "," << state.angle[5] << ","
				<< state.omega[0] << "," << state.omega[1] << "," << state.omega[2] << "," << state.omega[3] << "," << state.omega[4] << "," << state.omega[5] << ","
				<< state.alpha[0] << "," << state.alpha[1] << "," << state.alpha[2] << "," << state.alpha[3] << "," << state.alpha[4] << "," << state.alpha[5] << ","
				<< state.torque[0] << "," << state.torque[1] << "," << state.torque[2] << "," << state.torque[3] << "," << state.torque[4] << "," << state.torque[5] << endl;
		}
	}

	return 0;
}

