// #include "stdafx.h"
#include "stdio.h"
#include "string.h"
#include "setjmp.h"
#include "time.h"
#include "math.h"
#include "ctype.h"
#include "stdlib.h" 
#include "forsight_innercmd.h"
#include "reg_manager/reg_manager_interface.h"
#include "reg_manager/reg_manager_interface_wrapper.h"
#include "motion_plan_frame_manager.h"
#ifdef WIN32
#include "interpreter_common.h"
#else
#include "common/interpreter_common.h"
#endif

using namespace fst_reg ;

// Register name
#define TXT_PR    "pr"
#define TXT_SR    "sr"
#define TXT_R     "r"
#define TXT_MR    "mr"

#define TXT_UF    "uf"
#define TXT_TF    "tf"

#define TXT_PL    "pl"

// member name of Register
#define TXT_REG_TYPE    "type"
#define TXT_REG_ID      "id"
#define TXT_REG_COMMENT "comment"
#define TXT_REG_VALUE   "value"

// member name of PR
#define TXT_PL_POSE    "pose"
#define TXT_PL_JOINT   "joint"
// member name of UF/TF
#define TXT_UF_TF_COORDINATE    "coordinate"
// member name of PL
#define TXT_PL_POSE       "pose"
#define TXT_PL_PALLET     "pallet"
#define TXT_PL_FLAG       "flag"

RegManagerInterface * g_objRegManagerInterface = NULL;

/* Return true if c is a delimiter. */
static int isdelim(char c)
{
  if(strchr(" ;,+-<>/*%^=()[]", c) || c==9 || c=='\r' || c=='\n' || c==0) 
    return 1;  
  return 0;
}

static int get_char_token(char * src, char * dst)
{
	char * tmp = src ;
	if(isalpha(*src)) { /* var or command */
		while(!isdelim(*(src))) 
			*dst++=*(src)++;
	}
	return src - tmp ;
}

static int get_num_token(char * src, char * dst)
{
	char * tmp = src ;
	if(isdigit(*src)) { /* var or command */
		while(!isdelim(*(src))) 
			*dst++=*(src)++;
	}
	return src - tmp ;
}


void load_register_data()
{
	g_objRegManagerInterface = new RegManagerInterface("share/configuration/machine");
}

bool reg_manager_interface_getPr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		bRet = g_objRegManagerInterface->getPrReg(num, ptr);
		PrRegData* data_ptr = ptr ;
		
		printf("data_ptr: id = %d, comment = %s\n", data_ptr->id, data_ptr->comment);
		printf("data_ptr: id = (%f, %f, %f, %f, %f, %f) \n", 
			data_ptr->value.joint_pos[0], data_ptr->value.joint_pos[1], 
			data_ptr->value.joint_pos[2], data_ptr->value.joint_pos[3], 
			data_ptr->value.joint_pos[4], data_ptr->value.joint_pos[5]);
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_setPr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		PrRegData objPrRegData ;
		memcpy(&objPrRegData, ptr, sizeof(PrRegData));
		printf("PrRegData: id = %d, comment = %s\n", objPrRegData.id, objPrRegData.comment);
		printf("PrRegData: id = (%f, %f, %f, %f, %f, %f) \n", 
			objPrRegData.value.joint_pos[0], objPrRegData.value.joint_pos[1], 
			objPrRegData.value.joint_pos[2], objPrRegData.value.joint_pos[3], 
			objPrRegData.value.joint_pos[4], objPrRegData.value.joint_pos[5]);
		objPrRegData.id = num ;
		
		bRet = g_objRegManagerInterface->setPrReg(&objPrRegData);
		if(bRet == false)
		{
			bRet = g_objRegManagerInterface->addPrReg(&objPrRegData);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

/*
 * The operated object is an individual member of PR.
 */
bool reg_manager_interface_getPosePr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		PrRegData objPrRegData ;
		
		
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
		   memcpy(ptr, &(objPrRegData.value.cartesian_pos), 
		   	    sizeof(objPrRegData.value.cartesian_pos));
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_setPosePr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		PrRegData objPrRegData ;
		
		
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
		    memcpy(&(objPrRegData.value.cartesian_pos), ptr, 
				sizeof(objPrRegData.value.cartesian_pos));
			reg_manager_interface_setPr(&objPrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_getJointPr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		PrRegData objPrRegData ;
		
		
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
		   memcpy(ptr, &(objPrRegData.value.joint_pos), 
		   	      sizeof(objPrRegData.value.joint_pos));
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_setJointPr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		PrRegData objPrRegData ;
		
		
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
		    memcpy(&(objPrRegData.value.joint_pos), ptr, 
				sizeof(objPrRegData.value.joint_pos));
			reg_manager_interface_setPr(&objPrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_getTypePr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		PrRegData objPrRegData ;
		
		
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
		   memcpy(ptr, &(objPrRegData.value.pos_type), 
		   	      sizeof(objPrRegData.value.pos_type));
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_setTypePr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		PrRegData objPrRegData ;
		
		
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
		    memcpy(&(objPrRegData.value.pos_type), ptr, 
				sizeof(objPrRegData.value.pos_type));
			reg_manager_interface_setPr(&objPrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_getIdPr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		PrRegData objPrRegData ;
		
		
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
		   memcpy(ptr, &(objPrRegData.id), sizeof(objPrRegData.id));
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_setIdPr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		PrRegData objPrRegData ;
		
		
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
		    memcpy(&(objPrRegData.id), ptr, sizeof(objPrRegData.id));
			reg_manager_interface_setPr(&objPrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_getCommentPr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		PrRegData objPrRegData ;
		
		
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
		   memcpy(ptr, objPrRegData.comment, sizeof(objPrRegData.comment));
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_setCommentPr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		PrRegData objPrRegData ;
		
		
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
		    memcpy(objPrRegData.comment, ptr, 
				sizeof(objPrRegData.comment));
			reg_manager_interface_setPr(&objPrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

/**********************
 ********* SR *********
 **********************/

bool reg_manager_interface_getSr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		
		
		bRet = g_objRegManagerInterface->getSrReg(num, ptr);
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_setSr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		
		SrRegData objSrRegData ;
		memcpy(&objSrRegData, ptr, sizeof(objSrRegData));
		objSrRegData.id = num ;
		
		bRet = g_objRegManagerInterface->setSrReg(&objSrRegData);
		if(bRet == false)
		{
			bRet = g_objRegManagerInterface->addSrReg(&objSrRegData);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_getValueSr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		SrRegData objSrRegData ;
		
		
		bRet = g_objRegManagerInterface->getSrReg(num, &objSrRegData);
		if(bRet)
		{
		   memcpy(ptr, objSrRegData.value.c_str(), 
		   	    sizeof(objSrRegData.value.length()));
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_setValueSr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		SrRegData objSrRegData ;
		
		
		bRet = g_objRegManagerInterface->getSrReg(num, &objSrRegData);
		if(bRet)
		{
		    objSrRegData.value = string((char *)ptr);
			reg_manager_interface_setSr(&objSrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_getIdSr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		SrRegData objSrRegData ;
		
		
		bRet = g_objRegManagerInterface->getSrReg(num, &objSrRegData);
		if(bRet)
		{
		   memcpy(ptr, &(objSrRegData.id), sizeof(objSrRegData.id));
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_setIdSr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		SrRegData objSrRegData ;
		
		
		bRet = g_objRegManagerInterface->getSrReg(num, &objSrRegData);
		if(bRet)
		{
		    memcpy(&(objSrRegData.id), ptr, sizeof(objSrRegData.id));
			reg_manager_interface_setSr(&objSrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_getCommentSr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		SrRegData objSrRegData ;
		
		
		bRet = g_objRegManagerInterface->getSrReg(num, &objSrRegData);
		if(bRet)
		{
		   memcpy(ptr, objSrRegData.comment, sizeof(objSrRegData.comment));
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_setCommentSr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		SrRegData objSrRegData ;
		
		
		bRet = g_objRegManagerInterface->getSrReg(num, &objSrRegData);
		if(bRet)
		{
		    memcpy(objSrRegData.comment, ptr, sizeof(objSrRegData.comment));
			reg_manager_interface_setPr(&objSrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

/**********************
 ********* R **********
 **********************/
bool reg_manager_interface_getR(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		printf("reg_manager_interface_getR at %d \n", num);
		bRet = g_objRegManagerInterface->getRReg(num, ptr);
		
		RRegData* data_ptr = ptr ;
		
		printf("data_ptr: id = %d, comment = %s\n", data_ptr->id, data_ptr->comment);
		printf("data_ptr: value = (%f) \n", data_ptr->value);
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_setR(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		
		RRegData * objRRegDataPtr = (RRegData *)ptr;
		// memcpy(&objRRegData, ptr, sizeof(objRRegData));
		
		printf("RRegData: id = %d, comment = %s\n", objRRegDataPtr->id, objRRegDataPtr->comment);
		printf("RRegData: id = (%f) \n", objRRegDataPtr->value);

		objRRegDataPtr->id = num ;
		
		bRet = g_objRegManagerInterface->setRReg(objRRegDataPtr);
		if(bRet == false)
		{
			bRet = g_objRegManagerInterface->addRReg(objRRegDataPtr);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_getValueR(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		RRegData objRRegData ;
		
		
		bRet = g_objRegManagerInterface->getRReg(num, &objRRegData);
		if(bRet)
		{
		   memcpy(ptr, &(objRRegData.value), 
		   	    sizeof(objRRegData.value));
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_setValueR(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		RRegData objRRegData ;
		
		bRet = g_objRegManagerInterface->getRReg(num, &objRRegData);
		if(bRet)
		{
		    objRRegData.value = *(double *)ptr;
			reg_manager_interface_setR(&objRRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_getIdR(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		RRegData objRRegData ;
		
		
		bRet = g_objRegManagerInterface->getRReg(num, &objRRegData);
		if(bRet)
		{
		   memcpy(ptr, &(objRRegData.id), sizeof(objRRegData.id));
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_setIdR(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		RRegData objRRegData ;
		
		
		bRet = g_objRegManagerInterface->getRReg(num, &objRRegData);
		if(bRet)
		{
		    memcpy(&(objRRegData.id), ptr, sizeof(objRRegData.id));
			reg_manager_interface_setR(&objRRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_getCommentR(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		RRegData objRRegData ;
		
		
		bRet = g_objRegManagerInterface->getRReg(num, &objRRegData);
		if(bRet)
		{
		   memcpy(ptr, objRRegData.comment, sizeof(objRRegData.comment));
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_setCommentR(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		RRegData objRRegData ;
		
		
		bRet = g_objRegManagerInterface->getRReg(num, &objRRegData);
		if(bRet)
		{
		    memcpy(objRRegData.comment, ptr, sizeof(objRRegData.comment));
			reg_manager_interface_setR(&objRRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

/**********************
 ********* MR *********
 **********************/
bool reg_manager_interface_getMr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		
		bRet = g_objRegManagerInterface->getMrReg(num, ptr);
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_setMr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		
		MrRegData objMrRegData ;
		memcpy(&objMrRegData, ptr, sizeof(objMrRegData));
		objMrRegData.id = num ;
		
		bRet = g_objRegManagerInterface->setMrReg(&objMrRegData);
		if(bRet == false)
		{
			bRet = g_objRegManagerInterface->addMrReg(&objMrRegData);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}
 
bool reg_manager_interface_getValueMr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		MrRegData objMrRegData ;
		
		
		bRet = g_objRegManagerInterface->getMrReg(num, &objMrRegData);
		if(bRet)
		{
		   memcpy(ptr, &(objMrRegData.value), 
		   	    sizeof(objMrRegData.value));
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_setValueMr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		MrRegData objMrRegData ;
		
		
		bRet = g_objRegManagerInterface->getMrReg(num, &objMrRegData);
		if(bRet)
		{
		    objMrRegData.value = *(int *)ptr;
			reg_manager_interface_setMr(&objMrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_getIdMr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		MrRegData objMrRegData ;
		
		
		bRet = g_objRegManagerInterface->getMrReg(num, &objMrRegData);
		if(bRet)
		{
		   memcpy(ptr, &(objMrRegData.id), sizeof(objMrRegData.id));
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_setIdMr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		MrRegData objMrRegData ;
		
		
		bRet = g_objRegManagerInterface->getMrReg(num, &objMrRegData);
		if(bRet)
		{
		    memcpy(&(objMrRegData.id), ptr, sizeof(objMrRegData.id));
			reg_manager_interface_setMr(&objMrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_getCommentMr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		MrRegData objMrRegData ;
		
		
		bRet = g_objRegManagerInterface->getMrReg(num, &objMrRegData);
		if(bRet)
		{
		   memcpy(ptr, objMrRegData.comment, sizeof(objMrRegData.comment));
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}

bool reg_manager_interface_setCommentMr(void *ptr, uint16_t num)
{
	bool bRet = false ;
	if(g_objRegManagerInterface)
	{
		MrRegData objMrRegData ;
		
		
		bRet = g_objRegManagerInterface->getMrReg(num, &objMrRegData);
		if(bRet)
		{
		    memcpy(objMrRegData.comment, ptr, sizeof(objMrRegData.comment));
			reg_manager_interface_setR(&objMrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return bRet ;
}


/**********************
 ********* UF *********
 **********************/
bool reg_manager_interface_getUf(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setUf(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_getCoordinateUf(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setCoordinateUf(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_getIdUf(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setIdUf(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_getCommentUf(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setCommentUf(void *ptr, uint16_t num)
{
	return 0 ;
}

/**********************
 ********* TF *********
 **********************/
bool reg_manager_interface_getTf(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setTf(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_getCoordinateTf(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setCoordinateTf(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_getIdTf(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setIdTf(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_getCommentTf(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setCommentTf(void *ptr, uint16_t num)
{
	return 0 ;
}

/**********************
 ********* PL *********
 **********************/
bool reg_manager_interface_getPl(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setPl(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_getPosePl(PoseEuler& pose, int index)
{
	return 0 ;
}

bool reg_manager_interface_setPosePl(PoseEuler pose, int index)
{
	return 0 ;
}

bool reg_manager_interface_getPalletPl(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setPalletPl(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_getFlagPl(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setFlagPl(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_getIdPl(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setIdPl(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_getCommentPl(void *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setCommentPl(void *ptr, uint16_t num)
{
	return 0 ;
}

std::vector<BaseRegData> reg_manager_interface_read_chg_pr_lst(int start_id, int size)
{
    std::vector<BaseRegData> vecRet ;
	bool bRet = false ;
	vecRet.clear();
	if(g_objRegManagerInterface)
	{
		vecRet = g_objRegManagerInterface->getPrRegChangedIdList(0, 255);
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return vecRet ;
}

std::vector<BaseRegData> reg_manager_interface_read_chg_sr_lst(int start_id, int size)
{
    std::vector<BaseRegData> vecRet ;
	bool bRet = false ;
	vecRet.clear();
	if(g_objRegManagerInterface)
	{
		vecRet = g_objRegManagerInterface->getSrRegChangedIdList(0, 255);
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return vecRet ;
}

std::vector<BaseRegData> reg_manager_interface_read_chg_r_lst(int start_id, int size)
{
    std::vector<BaseRegData> vecRet ;
	bool bRet = false ;
	vecRet.clear();
	if(g_objRegManagerInterface)
	{
		vecRet = g_objRegManagerInterface->getRRegChangedIdList(0, 255);
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return vecRet ;
}

std::vector<BaseRegData> reg_manager_interface_read_chg_mr_lst(int start_id, int size)
{
    std::vector<BaseRegData> vecRet ;
	bool bRet = false ;
	vecRet.clear();
	if(g_objRegManagerInterface)
	{
		vecRet = g_objRegManagerInterface->getMrRegChangedIdList(0, 255);
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
	return vecRet ;
}



