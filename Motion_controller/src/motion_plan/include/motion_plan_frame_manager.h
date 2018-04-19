#ifndef MOTION_PLAN_FRAME_MANAGER_H
#define MOTION_PLAN_FRAME_MANAGER_H

#include "fst_datatype.h"
#include "parameter_manager/parameter_manager_param_group.h"
#include "motion_plan_matrix.h"
#include <vector>
#include <string>

namespace fst_controller
{

enum 
{
    MAX_COMMENT_LENGTH = 32,
};
    
enum 
{
    MAX_USER_FRAME_NUM = 11,
    MAX_TOOL_FRAME_NUM = 11,
};

typedef struct
{
    int id;
    bool is_valid;
    char comment[MAX_COMMENT_LENGTH];
    PoseEuler data;
}Frame;

class FrameManager
{
public:
    FrameManager(std::string frame_name, int frame_set_size, std::string file_path, 
                        fst_algorithm::Matrix& frame_matrix, fst_algorithm::Matrix& inverse_frame_matrix);
    ~FrameManager();

    bool isReady();
    bool activateFrame(int id);
    int getActivatedFrame();
    bool addFrame(Frame& frame);
    bool deleteFrame(int id);
    bool updateFrame(Frame& frame);
    
private:
    std::string getFramePath(int frame_index);
    bool writeFrameToYaml(Frame& frame);
    bool readAllFrameFromYaml(int frame_set_size);
    bool updateGlobalFrame(int frame_index);

    bool is_ready_;
    int activated_frame_id_;
    std::string file_path_;
    std::string frame_name_;
    std::vector<Frame> frame_set_;
    fst_parameter::ParamGroup param_;
    fst_algorithm::Matrix& frame_matrix_;
    fst_algorithm::Matrix& inverse_frame_matrix_;
};

}

#endif

