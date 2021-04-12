#include <string.h>
#include <assert.h>

#include "Servo_Para.h"
#include "Servo_General.h"

using namespace virtual_servo_device;

static PARA_AXIS_LOCAL_INFO para_local_[AXIS_MAX];
static PARA_AXIS_READ_INFO para_read_[AXIS_MAX];

/*-------------------------------------------------------------
 *函数名称 :void Servo_Para_Init(void)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :
--------------------------------------------------------------*/
void virtual_servo_device::Servo_Para_Init(void)
{
	para_read_[AXIS_0].reservd.para_value = 0xaa;
	para_read_[AXIS_0].position_tarj_info.target_point.para_value = 0x1000;
	para_read_[AXIS_0].axis_alg_info.pos_alg_loop.kp_factor.para_value = 3000000;
	para_read_[AXIS_0].axis_alg_info.sp_alg_loop.kp_factor.para_value = 16777216*1.5;
	para_read_[AXIS_0].axis_alg_info.sp_alg_loop.time_res_factor.para_value = 100;
	para_read_[AXIS_0].axis_alg_info.sp_alg_loop.inertia_factor.para_value = 1;
	para_read_[AXIS_0].axis_alg_info.current_alg_loop.ki_factor.para_value = 1222290;
	para_read_[AXIS_0].axis_alg_info.current_alg_loop.kp_factor.para_value = 6000000;

	para_read_[AXIS_1].reservd.para_value = 0xaa;
	para_read_[AXIS_1].position_tarj_info.target_point.para_value = 0x1000;
	para_read_[AXIS_1].axis_alg_info.pos_alg_loop.kp_factor.para_value = 3000000;
	para_read_[AXIS_1].axis_alg_info.sp_alg_loop.kp_factor.para_value = 16777216*1.5;
	para_read_[AXIS_1].axis_alg_info.sp_alg_loop.time_res_factor.para_value = 100;
	para_read_[AXIS_1].axis_alg_info.sp_alg_loop.inertia_factor.para_value = 1;
	para_read_[AXIS_1].axis_alg_info.sp_alg_loop.inertia_factor.para_value = 1;
	para_read_[AXIS_1].axis_alg_info.current_alg_loop.ki_factor.para_value = 1222290;
	para_read_[AXIS_1].axis_alg_info.current_alg_loop.kp_factor.para_value = 6000000;

	para_read_[AXIS_2].reservd.para_value = 0xaa;
	para_read_[AXIS_2].position_tarj_info.target_point.para_value = 0x1000;
	para_read_[AXIS_2].axis_alg_info.pos_alg_loop.kp_factor.para_value = 3000000;
	para_read_[AXIS_2].axis_alg_info.sp_alg_loop.kp_factor.para_value = 16777216*1.5;
	para_read_[AXIS_2].axis_alg_info.sp_alg_loop.time_res_factor.para_value = 100;
	para_read_[AXIS_2].axis_alg_info.sp_alg_loop.inertia_factor.para_value = 1;
	para_read_[AXIS_2].axis_alg_info.current_alg_loop.ki_factor.para_value = 1222290;
	para_read_[AXIS_2].axis_alg_info.current_alg_loop.kp_factor.para_value = 6000000;

	para_read_[AXIS_3].reservd.para_value = 0xaa;
	para_read_[AXIS_3].position_tarj_info.target_point.para_value = 0x1000;
	para_read_[AXIS_3].axis_alg_info.pos_alg_loop.kp_factor.para_value = 3000000;
	para_read_[AXIS_3].axis_alg_info.sp_alg_loop.kp_factor.para_value = 16777216*1.5;
	para_read_[AXIS_3].axis_alg_info.sp_alg_loop.time_res_factor.para_value = 100;
	para_read_[AXIS_3].axis_alg_info.sp_alg_loop.inertia_factor.para_value = 1;
	para_read_[AXIS_3].axis_alg_info.current_alg_loop.ki_factor.para_value = 1222290;
	para_read_[AXIS_3].axis_alg_info.current_alg_loop.kp_factor.para_value = 6000000;

	para_read_[AXIS_4].reservd.para_value = 0xaa;
	para_read_[AXIS_4].position_tarj_info.target_point.para_value = 0x1000;
	para_read_[AXIS_4].axis_alg_info.pos_alg_loop.kp_factor.para_value = 3000000;
	para_read_[AXIS_4].axis_alg_info.sp_alg_loop.kp_factor.para_value = 16777216*1.5;
	para_read_[AXIS_4].axis_alg_info.sp_alg_loop.time_res_factor.para_value = 100;
	para_read_[AXIS_4].axis_alg_info.sp_alg_loop.inertia_factor.para_value = 1;
	para_read_[AXIS_4].axis_alg_info.current_alg_loop.ki_factor.para_value = 1222290;
	para_read_[AXIS_4].axis_alg_info.current_alg_loop.kp_factor.para_value = 6000000;

	para_read_[AXIS_5].reservd.para_value = 0xaa;
	para_read_[AXIS_5].position_tarj_info.target_point.para_value = 0x1000;
	para_read_[AXIS_5].axis_alg_info.pos_alg_loop.kp_factor.para_value = 3000000;
	para_read_[AXIS_5].axis_alg_info.sp_alg_loop.kp_factor.para_value = 16777216*1.5;
	para_read_[AXIS_5].axis_alg_info.sp_alg_loop.time_res_factor.para_value = 100;
	para_read_[AXIS_5].axis_alg_info.sp_alg_loop.inertia_factor.para_value = 1;
	para_read_[AXIS_5].axis_alg_info.current_alg_loop.ki_factor.para_value = 1222290;
	para_read_[AXIS_5].axis_alg_info.current_alg_loop.kp_factor.para_value = 6000000;

	para_local_[AXIS_0].sync_reg = 0;
	para_local_[AXIS_1].sync_reg = 1;
	para_local_[AXIS_2].sync_reg = 2;
	para_local_[AXIS_3].sync_reg = 3;
	para_local_[AXIS_4].sync_reg = 4;
	para_local_[AXIS_5].sync_reg = 5;
	para_local_[AXIS_6].sync_reg = 6;
	para_local_[AXIS_7].sync_reg = 7;


}
/*-------------------------------------------------------------
 *函数名称 :bool Servo_Para_Manager_DownPara(SERVO_AXIS_ENUM axis_id
						,const CommBlockData_t* axis_para_ptr)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :批量设置参数
--------------------------------------------------------------*/
bool virtual_servo_device::Servo_Para_Manager_DownPara(SERVO_AXIS_ENUM axis_id
						,const CommBlockData_t* axis_para_ptr)
{
    bool ret_val = true;

    assert(axis_para_ptr!=NULL);

    memcpy((char *)&para_local_[axis_id],axis_para_ptr->memory_ptr
           ,axis_para_ptr->param1);

    ret_val = Servo_Para_SetPara(axis_id,0,axis_para_ptr->param1);

    return ret_val;
}
/*-------------------------------------------------------------
 *函数名称 :bool Servo_Para_Manager_UploadPara(SERVO_AXIS_ENUM axis_id,
							CommBlockData_t* upload_para_ptr,
                                  uint32_t copy_length)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :批量上传参数
--------------------------------------------------------------*/
bool virtual_servo_device::Servo_Para_Manager_UploadPara(SERVO_AXIS_ENUM axis_id,
							CommBlockData_t* upload_para_ptr,
                                  uint32_t copy_length)
{
    assert(upload_para_ptr!=NULL);

    memcpy(upload_para_ptr->memory_ptr
           ,(char *)&para_read_[axis_id],copy_length);

    upload_para_ptr->param1 = copy_length;

    return true;
}
/*-------------------------------------------------------------
 *函数名称 :bool Servo_Para_Manager_ReadPara(SERVO_AXIS_ENUM axis_id
		,int32_t* get_para,uint32_t index_offset)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :HPS控制FPGA保护操作
--------------------------------------------------------------*/
bool virtual_servo_device::Servo_Para_Manager_ReadPara(SERVO_AXIS_ENUM axis_id
		,int32_t* get_para,uint32_t index_offset)
{
    assert(index_offset<512);

    int32_t* temp_ptr = (int32_t*)&para_local_[axis_id];

    *get_para = *(temp_ptr+index_offset);

    return true;
}
/*-------------------------------------------------------------
 *函数名称 :bool Servo_Para_Manager_WritePara(SERVO_AXIS_ENUM axis_id
		,int32_t para_value,uint32_t index_offset)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :HPS控制FPGA保护操作
--------------------------------------------------------------*/
bool virtual_servo_device::Servo_Para_Manager_WritePara(SERVO_AXIS_ENUM axis_id
		,int32_t para_value,uint32_t index_offset)
{
    bool ret_val = true;

    assert(index_offset<512);

    int32_t* temp_ptr = (int32_t*)&para_local_[axis_id];

    *(temp_ptr+index_offset) = para_value;

    ret_val = Servo_Para_SetPara(axis_id,index_offset,1);

    return ret_val;
}
/*-------------------------------------------------------------
 *函数名称 :bool Servo_Para_SetPara(SERVO_AXIS_ENUM axis_id,
                        uint32_t index_offset
                        ,uint32_t para_length)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :HPS控制FPGA保护操作
--------------------------------------------------------------*/
bool virtual_servo_device::Servo_Para_SetPara(SERVO_AXIS_ENUM axis_id,
                        uint32_t index_offset
                        ,uint32_t para_length)
{
    int32_t* temp_src_ptr = (int32_t*)(&para_local_[axis_id]);

    SERVO_PARA_COMMO_INFO* temp_des_ptr = (SERVO_PARA_COMMO_INFO*)(&(para_read_[axis_id]));

    for(uint32_t i=0; i<para_length;i++)
    {
        (temp_des_ptr+i+index_offset)->para_value =
                                *(temp_src_ptr+i+index_offset);
    }

    return true;
}
/*-------------------------------------------------------------
 *函数名称 :bool Servo_Para_Get_LocalPtr(SERVO_AXIS_ENUM axis_id,
		PARA_AXIS_LOCAL_INFO** get_local_ptr)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :HPS控制FPGA保护操作
--------------------------------------------------------------*/
bool virtual_servo_device::Servo_Para_Get_LocalPtr(SERVO_AXIS_ENUM axis_id,
		PARA_AXIS_LOCAL_INFO** get_local_ptr)
{
    bool ret_val=true;

    *get_local_ptr = &para_local_[axis_id];

    return ret_val;
}
/*-------------------------------------------------------------
 *函数名称 :bool servo_para_get_readptr(SERVO_AXIS_ENUM axis_id,
		PARA_AXIS_READ_INFO** get_read_ptr)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :获取上传参数指针接口
--------------------------------------------------------------*/
bool virtual_servo_device::Servo_Para_Get_ReadPtr(SERVO_AXIS_ENUM axis_id,
		PARA_AXIS_READ_INFO** get_read_ptr)
{
    bool ret_val = true;

    *get_read_ptr = &para_read_[axis_id];

    return ret_val;
}
/*-------------------------------------------------------------
 *函数名称 :bool servo_para_get_readptr(SERVO_AXIS_ENUM axis_id,
		PARA_AXIS_READ_INFO** get_read_ptr)
 *函数输入 :无
 *函数输出 :无
 *函数描述 :获取上传参数指针接口
--------------------------------------------------------------*/
bool virtual_servo_device::Servo_b_Get_ServoMode(SERVO_AXIS_ENUM axis_id
								,int32_t** servo_mode)
{
	bool ret_val = true;
	*servo_mode = &para_local_[axis_id].monitor_info.operation_mode_act;
	return ret_val;
}

