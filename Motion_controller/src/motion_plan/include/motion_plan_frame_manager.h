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

    //------------------------------------------------------------
    // Function:    isReady
    // Summary: Return the status of FrameManager
    // In:      None
    // Out:     None
    // Return:  true: all other public function can be used
    //          false:all other public function can not be used
    //------------------------------------------------------------
    bool isReady();
    //------------------------------------------------------------
    // Function:    activateFrame
    // Summary: activate a frame by id
    // In:      id: frame index to be activated
    // Out:     None
    // Return:  true: the frame with the expected id is activated
    //          false:the frame with the expected id is not activated, 
    //                the current activated frame is unchanged
    //------------------------------------------------------------
    bool activateFrame(int id);
    //------------------------------------------------------------
    // Function:    getActivatedFrame
    // Summary: Return the current activated frame id
    // In:      None
    // Out:     None
    // Return:  the id of current activated frame
    //------------------------------------------------------------    
    int getActivatedFrame();
    //------------------------------------------------------------
    // Function:    addFrame
    // Summary: Add a new defined frame to frame repository
    // In:      frame: include all necessary data of the frame
    // Out:     None
    // Return:  true: add successfully
    //          false:add operation is terminated because of some reason
    //------------------------------------------------------------    
    bool addFrame(Frame& frame);
    //------------------------------------------------------------
    // Function:    deleteFrame
    // Summary: Delete a registered frame from the frame repository
    // In:      id: the id of the expected frame to be deleted
    // Out:     None
    // Return:  true: delete successfully
    //          false:delete operation is terminated because of some reason
    //------------------------------------------------------------      
    bool deleteFrame(int id);
    //------------------------------------------------------------
    // Function:    updateFrame
    // Summary: update a registered frame in the frame repository
    // In:      frame: include all necessary data of the frame
    // Out:     None
    // Return:  true: update successfully
    //          false:update operation is terminated because of some reason
    //------------------------------------------------------------        
    bool updateFrame(Frame& frame);

    //------------------------------------------------------------
    // Function:    getFrame
    // Summary: get a frame data in the frame repository
    // In:      id: the index of frame trying to get
    //          frame: include all necessary data of the frame
    // Out:     None
    // Return:  true: get successfully
    //          false:get operation is terminated because of some reason
    //------------------------------------------------------------        
    bool getFrame(int id, Frame& frame);

    //------------------------------------------------------------
    // Function:    getAllValidFrameId
    // Summary: get all ids of the valid frames
    // In:      None
    // Out:     None
    // Return:  list of id of valid frames
    //------------------------------------------------------------  
    std::vector<int> getAllValidFrameId();
    
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

