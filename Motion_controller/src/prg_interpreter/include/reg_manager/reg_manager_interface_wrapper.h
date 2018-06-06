#ifndef REG_MANAGER_INTERFACE_WRAPPER_H
#define REG_MANAGER_INTERFACE_WRAPPER_H

#include "reg_manager/reg_manager_interface.h"
#include "reg_manager/forsight_registers_manager.h"

using namespace fst_reg ;

#define REGSITER_NAMES   "pr;sr;r;mr;uf;tf;pl"

void load_register_data();

bool reg_manager_interface_getPr(void *ptr, uint16_t num);
bool reg_manager_interface_setPr(void *ptr, uint16_t num);

/*
 * The operated object is an individual member of PR.
 */
bool reg_manager_interface_getPosePr(void *ptr, uint16_t num);
bool reg_manager_interface_setPosePr(void *ptr, uint16_t num);

bool reg_manager_interface_getJointPr(void *ptr, uint16_t num);
bool reg_manager_interface_setJointPr(void *ptr, uint16_t num);

bool reg_manager_interface_getTypePr(void *ptr, uint16_t num);
bool reg_manager_interface_setTypePr(void *ptr, uint16_t num);

bool reg_manager_interface_getIdPr(void *ptr, uint16_t num);
bool reg_manager_interface_setIdPr(void *ptr, uint16_t num);

bool reg_manager_interface_getCommentPr(void *ptr, uint16_t num);
bool reg_manager_interface_setCommentPr(void *ptr, uint16_t num);

/**********************
 ********* SR *********
 **********************/

bool reg_manager_interface_getSr(void *ptr, uint16_t num);
bool reg_manager_interface_setSr(void *ptr, uint16_t num);

bool reg_manager_interface_getValueSr(void *ptr, uint16_t num);
bool reg_manager_interface_setValueSr(void *ptr, uint16_t num);

bool reg_manager_interface_getIdSr(void *ptr, uint16_t num);
bool reg_manager_interface_setIdSr(void *ptr, uint16_t num);

bool reg_manager_interface_getCommentSr(void *ptr, uint16_t num);
bool reg_manager_interface_setCommentSr(void *ptr, uint16_t num);

/**********************
 ********* R **********
 **********************/
bool reg_manager_interface_getR(void *ptr, uint16_t num);
bool reg_manager_interface_setR(void *ptr, uint16_t num);

bool reg_manager_interface_getValueR(void *ptr, uint16_t num);
bool reg_manager_interface_setValueR(void *ptr, uint16_t num);

bool reg_manager_interface_getIdR(void *ptr, uint16_t num);
bool reg_manager_interface_setIdR(void *ptr, uint16_t num);

bool reg_manager_interface_getCommentR(void *ptr, uint16_t num);
bool reg_manager_interface_setCommentR(void *ptr, uint16_t num);

/**********************
 ********* MR *********
 **********************/
bool reg_manager_interface_getMr(void *ptr, uint16_t num);
bool reg_manager_interface_setMr(void *ptr, uint16_t num);
 
bool reg_manager_interface_getValueMr(void *ptr, uint16_t num);
bool reg_manager_interface_setValueMr(void *ptr, uint16_t num);

bool reg_manager_interface_getIdMr(void *ptr, uint16_t num);
bool reg_manager_interface_setIdMr(void *ptr, uint16_t num);

bool reg_manager_interface_getCommentMr(void *ptr, uint16_t num);
bool reg_manager_interface_setCommentMr(void *ptr, uint16_t num);


/**********************
 ********* UF *********
 **********************/
bool reg_manager_interface_getUf(void *ptr, uint16_t num);
bool reg_manager_interface_setUf(void *ptr, uint16_t num);

bool reg_manager_interface_getCoordinateUf(void *ptr, uint16_t num);
bool reg_manager_interface_setCoordinateUf(void *ptr, uint16_t num);

bool reg_manager_interface_getIdUf(void *ptr, uint16_t num);
bool reg_manager_interface_setIdUf(void *ptr, uint16_t num);

bool reg_manager_interface_getCommentUf(void *ptr, uint16_t num);
bool reg_manager_interface_setCommentUf(void *ptr, uint16_t num);

/**********************
 ********* TF *********
 **********************/
bool reg_manager_interface_getTf(void *ptr, uint16_t num);
bool reg_manager_interface_setTf(void *ptr, uint16_t num);

bool reg_manager_interface_getCoordinateTf(void *ptr, uint16_t num);
bool reg_manager_interface_setCoordinateTf(void *ptr, uint16_t num);

bool reg_manager_interface_getIdTf(void *ptr, uint16_t num);
bool reg_manager_interface_setIdTf(void *ptr, uint16_t num);

bool reg_manager_interface_getCommentTf(void *ptr, uint16_t num);
bool reg_manager_interface_setCommentTf(void *ptr, uint16_t num);

/**********************
 ********* PL *********
 **********************/
bool reg_manager_interface_getPl(void *ptr, uint16_t num);
bool reg_manager_interface_setPl(void *ptr, uint16_t num);

bool reg_manager_interface_getPosePl(PoseEuler& pose, int index);
bool reg_manager_interface_setPosePl(PoseEuler pose, int index);

bool reg_manager_interface_getPalletPl(void *ptr, uint16_t num);
bool reg_manager_interface_setPalletPl(void *ptr, uint16_t num);

bool reg_manager_interface_getFlagPl(void *ptr, uint16_t num);
bool reg_manager_interface_setFlagPl(void *ptr, uint16_t num);

bool reg_manager_interface_getIdPl(void *ptr, uint16_t num);
bool reg_manager_interface_setIdPl(void *ptr, uint16_t num);

bool reg_manager_interface_getCommentPl(void *ptr, uint16_t num);
bool reg_manager_interface_setCommentPl(void *ptr, uint16_t num);

std::vector<BaseRegData> reg_manager_interface_read_valid_pr_lst(int start_id, int size);
std::vector<BaseRegData> reg_manager_interface_read_valid_sr_lst(int start_id, int size);
std::vector<BaseRegData> reg_manager_interface_read_valid_r_lst(int start_id, int size);
std::vector<BaseRegData> reg_manager_interface_read_valid_mr_lst(int start_id, int size);

#endif
