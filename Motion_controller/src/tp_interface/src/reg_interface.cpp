/**
 * @file reg_interface.cpp
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2017-06-12
 */

#include "reg_interface.h"
#include "common.h"
#include "error_code.h"
#include "error_monitor.h"
#include <boost/algorithm/string.hpp>

RegInterface::RegInterface()
{
    U64 result = initial();
    if (result != TPI_SUCCESS)
    {
        // rcs::Error::instance()->add(result);
    }
}

RegInterface::~RegInterface()
{
   if (reg_info_ != NULL)
       delete [] reg_info_;
}

RegInterface* RegInterface::instance()
{
    static RegInterface reg_interface;

    return &reg_interface;
}

U64 RegInterface::initial()
{
    reg_info_ = new RegTypeInfo[REG_TYPE_NUM];
	memset(reg_info_, 0x00, sizeof(RegTypeInfo) * REG_TYPE_NUM);
	reg_info_[0].path= "pr" ,        reg_info_[0].type = POSE_REG ;
	reg_info_[1].path= "pr_pose" ,   reg_info_[1].type = POSE_REG_POSE ;
	reg_info_[2].path= "pr_joint" ,  reg_info_[2].type = POSE_REG_JOINT ;
	reg_info_[3].path= "pr_type" ,   reg_info_[3].type = POSE_REG_TYPE ;
	reg_info_[4].path= "pr_id" ,     reg_info_[4].type = POSE_REG_ID ;
	reg_info_[5].path= "pr_comment", reg_info_[5].type = POSE_REG_COMMENT ;
	
	reg_info_[6].path= "str" ,         reg_info_[6].type = STR_REG ;
	reg_info_[7].path= "str_value" ,   reg_info_[7].type = STR_REG_VALUE ;
	reg_info_[8].path= "str_id" ,      reg_info_[8].type = STR_REG_ID ;
	reg_info_[9].path= "str_comment" , reg_info_[9].type = STR_REG_COMMENT ;
	
	reg_info_[10].path= "num" ,         reg_info_[10].type = NUM_REG ;
	reg_info_[11].path= "num_value" ,   reg_info_[11].type = NUM_REG_VALUE ;
	reg_info_[12].path= "num_id" ,      reg_info_[12].type = NUM_REG_ID ;
	reg_info_[13].path= "num_comment" , reg_info_[13].type = NUM_REG_COMMENT ;
	
	reg_info_[14].path= "mot" ,         reg_info_[14].type = MOT_REG ;
	reg_info_[15].path= "mot_value" ,   reg_info_[15].type = MOT_REG_VALUE ;
	reg_info_[16].path= "mot_id" ,      reg_info_[16].type = MOT_REG_ID ;
	reg_info_[17].path= "mot_comment" , reg_info_[17].type = MOT_REG_COMMENT ;
	
	reg_info_[18].path= "uf" ,         reg_info_[18].type = UF_REG ;
	reg_info_[19].path= "uf_coord" ,   reg_info_[19].type = UF_REG_COORD ;
	reg_info_[20].path= "uf_id" ,      reg_info_[20].type = UF_REG_ID ;
	reg_info_[21].path= "uf_comment" , reg_info_[21].type = UF_REG_COMMENT ;
	
	reg_info_[22].path= "tf" ,         reg_info_[22].type = TF_REG ;
	reg_info_[23].path= "tf_coord" ,   reg_info_[23].type = TF_REG_COORD ;
	reg_info_[24].path= "tf_id" ,      reg_info_[24].type = TF_REG_ID ;
	reg_info_[25].path= "tf_comment" , reg_info_[25].type = TF_REG_COMMENT ;
	
	reg_info_[26].path= "pl" ,        reg_info_[26].type = PL_REG ;
	reg_info_[27].path= "pl_pose" ,   reg_info_[27].type = PL_REG_POSE ;
	reg_info_[28].path= "pl_pallet" , reg_info_[28].type = PL_REG_PALLET ;
	reg_info_[29].path= "pl_flag" ,   reg_info_[29].type = PL_REG_FLAG ;
	reg_info_[30].path= "pl_id" ,     reg_info_[30].type = PL_REG_ID ;
	reg_info_[31].path= "pl_comment", reg_info_[31].type = PL_REG_COMMENT ;
	
    return TPI_SUCCESS;
}

U64 RegInterface::setReg(RegMap* reg_info)
{
    return PARSE_IO_PATH_FAILED;
}

U64 RegInterface::getReg(RegMap* reg_info, unsigned char *buffer, int buf_len, int& io_bytes_len)
{
    return PARSE_IO_PATH_FAILED;
}

U64 RegInterface::checkReg(const char *path, RegMap* reg_info)
{
    std::vector<std::string> vc_path;
    boost::split(vc_path, path, boost::is_any_of("/"));
    
    int size = vc_path.size();
    for (int i = 0; i < REG_TYPE_NUM; i++)
    {
    	if(vc_path[2] == reg_info_[i].path)
    	{
    		reg_info->type = reg_info_[i].type ;
			reg_info->index = stoi(vc_path[3]) ;
			return TPI_SUCCESS;
    	}
    }
    return PARSE_IO_PATH_FAILED;
}

