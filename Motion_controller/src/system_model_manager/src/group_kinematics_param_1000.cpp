#include "group_kinematics_param_1000.h"
#include "yaml_help.h"
#include <cstring>

using namespace system_model_space;
using namespace base_space;


GroupKinematicsParam1000::GroupKinematicsParam1000(std::string style_str, std::string file_path):
    ModelBase(MODEL_TYPE_GROUP_KINEMATICS, style_str, file_path, &param_[0], GroupKinematics1000__number)
{
    memset(&param_, 0, sizeof(ParamDetail_t) * GroupKinematics1000__number);
    addParam("model_id",          GroupKinematics1000__model_id);
    addParam("BaseD",             GroupKinematics1000__BaseD);
    addParam("BaseA",             GroupKinematics1000__BaseA);
    addParam("BaseALPHA",         GroupKinematics1000__BaseALPHA);
    addParam("BaseTHETA",         GroupKinematics1000__BaseTHETA);
    addParam("BaseD_offset",      GroupKinematics1000__BaseD_offset);
    addParam("BaseA_offset",      GroupKinematics1000__BaseA_offset);
    addParam("BaseALPHA_offset",  GroupKinematics1000__BaseALPHA_offset);
    addParam("BaseTHETA_offset",  GroupKinematics1000__BaseTHETA_offset);    
    addParam("D0",                GroupKinematics1000__D0);
    addParam("D1",                GroupKinematics1000__D1);
    addParam("D2",                GroupKinematics1000__D2);
    addParam("D3",                GroupKinematics1000__D3);
    addParam("A0",                GroupKinematics1000__A0);
    addParam("A1",                GroupKinematics1000__A1);
    addParam("A2",                GroupKinematics1000__A2);
    addParam("A3",                GroupKinematics1000__A3);
    addParam("ALPHA0",            GroupKinematics1000__ALPHA0);
    addParam("ALPHA1",            GroupKinematics1000__ALPHA1);
    addParam("ALPHA2",            GroupKinematics1000__ALPHA2);
    addParam("ALPHA3",            GroupKinematics1000__ALPHA3);
    addParam("THETA0",            GroupKinematics1000__THETA0);
    addParam("THETA1",            GroupKinematics1000__THETA1);
    addParam("THETA2",            GroupKinematics1000__THETA2);
    addParam("THETA3",            GroupKinematics1000__THETA3);
    addParam("D0_offset",         GroupKinematics1000__D0_offset);
    addParam("D1_offset",         GroupKinematics1000__D1_offset);
    addParam("D2_offset",         GroupKinematics1000__D2_offset);
    addParam("D3_offset",         GroupKinematics1000__D3_offset);
    addParam("A0_offset",         GroupKinematics1000__A0_offset);
    addParam("A1_offset",         GroupKinematics1000__A1_offset);
    addParam("A2_offset",         GroupKinematics1000__A2_offset);
    addParam("A3_offset",         GroupKinematics1000__A3_offset);
    addParam("ALPHA0_offset",     GroupKinematics1000__ALPHA0_offset);
    addParam("ALPHA1_offset",     GroupKinematics1000__ALPHA1_offset);
    addParam("ALPHA2_offset",     GroupKinematics1000__ALPHA2_offset);
    addParam("ALPHA3_offset",     GroupKinematics1000__ALPHA3_offset);
    addParam("THETA0_offset",     GroupKinematics1000__THETA0_offset);
    addParam("THETA1_offset",     GroupKinematics1000__THETA1_offset);
    addParam("THETA2_offset",     GroupKinematics1000__THETA2_offset);
    addParam("THETA3_offset",     GroupKinematics1000__THETA3_offset);
}
    
GroupKinematicsParam1000::~GroupKinematicsParam1000()
{

}

