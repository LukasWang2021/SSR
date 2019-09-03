#ifndef COORDINATE_MANAGER_H
#define COORDINATE_MANAGER_H


#include "coordinate_manager_param.h"
#include "common_log.h"
#include <string>
#include <vector>
#include "basic_alg_datatype.h"
#include "parameter_manager/parameter_manager_param_group.h"


namespace fst_ctrl
{
typedef struct
{
    int id;
    bool is_valid;
    std::string name;
    std::string comment;
    int group_id;
    basic_alg::PoseEuler data;
}CoordInfo;

typedef struct
{
    int id;
    std::string name;
    std::string comment;
    int group_id;
}CoordSummaryInfo;

class CoordinateManager
{
public:
    CoordinateManager();
    ~CoordinateManager();

    ErrorCode init();

    ErrorCode addCoord(CoordInfo& info);
    ErrorCode deleteCoord(int id);
    ErrorCode updateCoord(CoordInfo& info);
    ErrorCode moveCoord(int expect_id, int original_id);
    ErrorCode getCoordInfoById(int id, CoordInfo& info);
    std::vector<CoordSummaryInfo> getAllValidCoordSummaryInfo();   

private:
    CoordinateManagerParam* param_ptr_;
    fst_log::Logger* log_ptr_;
    std::vector<CoordInfo> coord_set_;
    fst_parameter::ParamGroup coord_info_yaml_help_;
    std::string coord_info_file_path_;

    void packDummyCoordInfo(CoordInfo& info);
    std::string getCoordInfoPath(int coord_id);
    bool readAllCoordInfoFromYaml(int number_of_coords);
    bool writeCoordInfoToYaml(CoordInfo& info);
};

}

#endif


