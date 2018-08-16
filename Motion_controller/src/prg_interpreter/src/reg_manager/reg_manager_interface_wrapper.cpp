// #include "stdafx.h"
#include "stdio.h"
#include "string.h"
#include "setjmp.h"
#include "time.h"
#include "math.h"
#include "ctype.h"
#include "stdlib.h" 
#include "forsight_innercmd.h"
#include "reg_manager/reg_manager_interface_wrapper.h"
#include "interpreter_common.h"
#ifndef WIN32
#include "motion_plan_frame_manager.h"
#endif


// Register name
#define TXT_PR    "pr"
#define TXT_SR    "sr"
#define TXT_R     "r"
#define TXT_MR    "mr"

#define TXT_UF    "uf"
#define TXT_TF    "tf"

#define TXT_PL    "pl"


#ifndef WIN32
#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
// RegManagerInterface * g_objRegManagerInterface = NULL;
// using namespace fst_reg ;
#else
fst_base::ProcessComm* g_process_comm_ptr       = NULL;
fst_base::InterpreterClient* g_objRegManagerInterface = NULL;
fst_base::InterpreterServer* g_objInterpreterServer   = NULL;
using namespace fst_ctrl ;
#endif
#endif

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
#ifndef WIN32

#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
	g_objRegManagerInterface = new RegManagerInterface("/root/install/share/configuration/machine");
#else
	g_process_comm_ptr = fst_base::ProcessComm::getInstance();

	if(!g_process_comm_ptr->getInterpreterClientPtr()->init())
	{
        printf("load_register_data getInterpreterClientPtr return false\n");
		return false;
	}
    if(!g_process_comm_ptr->getInterpreterServerPtr()->init())
	{
        printf("load_register_data getInterpreterServerPtr init return false\n");
		return false;
	}
    if(!g_process_comm_ptr->getInterpreterServerPtr()->open())
	{
        printf("load_register_data getInterpreterServerPtr open return false\n");
		return false;
	}
	g_objRegManagerInterface = g_process_comm_ptr->getInterpreterClientPtr();
	g_objInterpreterServer   = g_process_comm_ptr->getInterpreterServerPtr();
#endif

#endif

}

bool reg_manager_interface_getPr(fst_ctrl::PrRegData *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		bRet = g_objRegManagerInterface->getPrReg(num, ptr);
		fst_ctrl::PrRegData* data_ptr = ptr ;
		
		printf("getPr: id = %d, comment = %s\n", 
			data_ptr->id, data_ptr->comment.c_str());
		printf("getPr: id = (%f, %f, %f, %f, %f, %f) \n", 
			data_ptr->value.pos[0], data_ptr->value.pos[1], 
			data_ptr->value.pos[2], data_ptr->value.pos[3], 
			data_ptr->value.pos[4], data_ptr->value.pos[5]);
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setPr(fst_ctrl::PrRegData *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		fst_ctrl::PrRegData objPrRegData ;
		memcpy(&objPrRegData, ptr, sizeof(fst_ctrl::PrRegData));
		printf("setPr: id = %d, comment = %s\n", 
			objPrRegData.id, objPrRegData.comment.c_str());
		printf("setPr: id = (%f, %f, %f, %f, %f, %f) \n", 
			objPrRegData.value.pos[0], objPrRegData.value.pos[1], 
			objPrRegData.value.pos[2], objPrRegData.value.pos[3], 
			objPrRegData.value.pos[4], objPrRegData.value.pos[5]);
		// objPrRegData.id = num ;
		
		bRet = g_objRegManagerInterface->setPrReg(&objPrRegData);
#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
		if(bRet == false)
		{
			bRet = g_objRegManagerInterface->addPrReg(&objPrRegData);
		}
#endif
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_delPr(uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
		bRet = g_objRegManagerInterface->deletePrReg(num);
#endif
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

/*
 * The operated object is an individual member of PR.
 */
bool reg_manager_interface_getPosePr(PoseEuler *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		fst_ctrl::PrRegData objPrRegData ;
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
		   memcpy(ptr, &(objPrRegData.value.pos), 
		   	    sizeof(objPrRegData.value.pos));
#endif
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setPosePr(PoseEuler *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		fst_ctrl::PrRegData objPrRegData ;
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
		    memcpy(&(objPrRegData.value.pos), ptr, 
				sizeof(objPrRegData.value.pos));
			reg_manager_interface_setPr(&objPrRegData, num);
#endif
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_getJointPr(_Joint *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		fst_ctrl::PrRegData objPrRegData ;
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
		   memcpy(ptr, &(objPrRegData.value.pos), 
		   	      sizeof(objPrRegData.value.pos));
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setJointPr(_Joint *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		fst_ctrl::PrRegData objPrRegData ;
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
		    memcpy(&(objPrRegData.value.pos), ptr, 
				sizeof(objPrRegData.value.pos));
			reg_manager_interface_setPr(&objPrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_getTypePr(int *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		fst_ctrl::PrRegData objPrRegData ;
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
		   *ptr = objPrRegData.value.pos_type;
#endif
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setTypePr(int *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		fst_ctrl::PrRegData objPrRegData ;
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
		    objPrRegData.value.pos_type = *ptr;

			reg_manager_interface_setPr(&objPrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_getIdPr(int *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		fst_ctrl::PrRegData objPrRegData ;
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
		   *ptr = objPrRegData.id;
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setIdPr(int *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		fst_ctrl::PrRegData objPrRegData ;
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
		    objPrRegData.id = *ptr;
			reg_manager_interface_setPr(&objPrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_getCommentPr(char *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		fst_ctrl::PrRegData objPrRegData ;
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
		   memcpy(ptr, objPrRegData.comment.c_str(), objPrRegData.comment.length());
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setCommentPr(char *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		fst_ctrl::PrRegData objPrRegData ;
		bRet = g_objRegManagerInterface->getPrReg(num, &objPrRegData);
		if(bRet)
		{
		    objPrRegData.comment = string(ptr);
			reg_manager_interface_setPr(&objPrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

/**********************
 ********* SR *********
 **********************/

bool reg_manager_interface_getSr(SrRegData *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		printf("getSr[%d]\n", num);
		bRet = g_objRegManagerInterface->getSrReg(num, ptr);
		printf("getSr[%d]:(%s)\n", num, ptr->value.c_str());
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setSr(SrRegData *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		
		fst_ctrl::SrRegData objSrRegData ;
		// memcpy(&objSrRegData, ptr, sizeof(objSrRegData));
		objSrRegData.id = num ;
		objSrRegData.comment = ptr->comment;
		objSrRegData.value = ptr->value ;
		printf("setSr:(%s)\n", objSrRegData.value.c_str());
		
		bRet = g_objRegManagerInterface->setSrReg(&objSrRegData);
#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
		if(bRet == false)
		{
			bRet = g_objRegManagerInterface->addSrReg(&objSrRegData);
		}
#endif
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_delSr(uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
		bRet = g_objRegManagerInterface->deleteSrReg(num);
#endif
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_getValueSr(string &strVal, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		fst_ctrl::SrRegData objSrRegData ;
		bRet = g_objRegManagerInterface->getSrReg(num, &objSrRegData);
		if(bRet)
		{
		   strVal = objSrRegData.value;
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setValueSr(string &strVal, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		fst_ctrl::SrRegData objSrRegData ;
		bRet = g_objRegManagerInterface->getSrReg(num, &objSrRegData);
		if(bRet)
		{
		    objSrRegData.value = strVal;
		}
		else    // Not exist
		{
		    objSrRegData.id    = num ;
            objSrRegData.comment = "Empty";
		    objSrRegData.value = strVal;
		}
		printf("setValueSr:(%s)\n", objSrRegData.value.c_str());
		reg_manager_interface_setSr(&objSrRegData, num);
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_getIdSr(int *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		fst_ctrl::SrRegData objSrRegData ;
		bRet = g_objRegManagerInterface->getSrReg(num, &objSrRegData);
		if(bRet)
		{
		   *ptr = objSrRegData.id;
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setIdSr(int *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		SrRegData objSrRegData ;
		bRet = g_objRegManagerInterface->getSrReg(num, &objSrRegData);
		if(bRet)
		{
		    objSrRegData.id = *ptr;
			reg_manager_interface_setSr(&objSrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_getCommentSr(char *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		SrRegData objSrRegData ;
		bRet = g_objRegManagerInterface->getSrReg(num, &objSrRegData);
		if(bRet)
		{
		   memcpy(ptr, objSrRegData.comment.c_str(), objSrRegData.comment.length());
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setCommentSr(char *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		SrRegData objSrRegData ;
		bRet = g_objRegManagerInterface->getSrReg(num, &objSrRegData);
		if(bRet)
		{
		    objSrRegData.comment = string(ptr);
			reg_manager_interface_setSr(&objSrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

/**********************
 ********* R **********
 **********************/
bool reg_manager_interface_getR(RRegData *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		printf("reg_manager_interface_getR at %d \n", num);
		bRet = g_objRegManagerInterface->getRReg(num, ptr);
		
		RRegData* data_ptr = ptr ;
		
		printf("getR: value = (%f) \n", data_ptr->value);
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setR(RRegData *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		
		RRegData * objRRegDataPtr = (RRegData *)ptr;
		// memcpy(&objRRegData, ptr, sizeof(objRRegData));
		
		printf("RRegData: id = (%f) \n", objRRegDataPtr->value);

		objRRegDataPtr->id = num ;
		
		bRet = g_objRegManagerInterface->setRReg(objRRegDataPtr);
#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
		if(bRet == false)
		{
			bRet = g_objRegManagerInterface->addRReg(objRRegDataPtr);
		}
#endif
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_delR(uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
		bRet = g_objRegManagerInterface->deleteRReg(num);
#endif
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_getValueR(double *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		RRegData objRRegData ;
		bRet = g_objRegManagerInterface->getRReg(num, &objRRegData);
		if(bRet)
		{
		   *ptr = objRRegData.value;
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setValueR(double *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		RRegData objRRegData ;
		bRet = g_objRegManagerInterface->getRReg(num, &objRRegData);
		if(bRet)
		{
		    objRRegData.value = *ptr;
		}
		else    // Not exist
		{
		    objRRegData.id    = num ;
            objRRegData.comment = "Empty";
		    objRRegData.value = *ptr;
		}
		reg_manager_interface_setR(&objRRegData, num);
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_getIdR(int *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		RRegData objRRegData ;
		bRet = g_objRegManagerInterface->getRReg(num, &objRRegData);
		if(bRet)
		{
		   *ptr = objRRegData.id;
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setIdR(int *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		RRegData objRRegData ;
		bRet = g_objRegManagerInterface->getRReg(num, &objRRegData);
		if(bRet)
		{
		    objRRegData.id = *ptr;
			reg_manager_interface_setR(&objRRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_getCommentR(char *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		RRegData objRRegData ;
		bRet = g_objRegManagerInterface->getRReg(num, &objRRegData);
		if(bRet)
		{
		   memcpy(ptr, objRRegData.comment.c_str(), objRRegData.comment.length());
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setCommentR(char *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		RRegData objRRegData ;
		bRet = g_objRegManagerInterface->getRReg(num, &objRRegData);
		if(bRet)
		{
		    objRRegData.comment = string(ptr);
			reg_manager_interface_setR(&objRRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

/**********************
 ********* MR *********
 **********************/
bool reg_manager_interface_getMr(MrRegData *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		bRet = g_objRegManagerInterface->getMrReg(num, ptr);
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setMr(MrRegData *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		MrRegData objMrRegData ;
		memcpy(&objMrRegData, ptr, sizeof(objMrRegData));
		objMrRegData.id = num ;
		
		bRet = g_objRegManagerInterface->setMrReg(&objMrRegData);
#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
		if(bRet == false)
		{
			bRet = g_objRegManagerInterface->addMrReg(&objMrRegData);
		}
#endif
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}
 
bool reg_manager_interface_delMr(uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
		bRet = g_objRegManagerInterface->deleteMrReg(num);
#endif
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_getValueMr(int *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		MrRegData objMrRegData ;
		bRet = g_objRegManagerInterface->getMrReg(num, &objMrRegData);
		if(bRet)
		{
		   *ptr = objMrRegData.value;
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setValueMr(int *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		MrRegData objMrRegData ;
		bRet = g_objRegManagerInterface->getMrReg(num, &objMrRegData);
		if(bRet)  
		{
		    objMrRegData.value = *ptr;
		}
		else    // Not exist
		{
		    objMrRegData.id    = num ;
            objMrRegData.comment = string("Empty");
		    objMrRegData.value = *ptr;
		}
		reg_manager_interface_setMr(&objMrRegData, num);
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_getIdMr(int *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		MrRegData objMrRegData ;
		bRet = g_objRegManagerInterface->getMrReg(num, &objMrRegData);
		if(bRet)
		{
		   *ptr = objMrRegData.id;
		}
		else
		{
			printf("MrReg[%d] is NULL\n", num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setIdMr(int *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		MrRegData objMrRegData ;
		bRet = g_objRegManagerInterface->getMrReg(num, &objMrRegData);
		if(bRet)
		{
		    objMrRegData.id = *ptr;
			reg_manager_interface_setMr(&objMrRegData, num);
		}
		else
		{
			printf("MrReg[%d] is NULL\n", num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_getCommentMr(char *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		MrRegData objMrRegData ;
		bRet = g_objRegManagerInterface->getMrReg(num, &objMrRegData);
		if(bRet)
		{
		   memcpy(ptr, objMrRegData.comment.c_str(), objMrRegData.comment.length());
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setCommentMr(char *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		MrRegData objMrRegData ;
		bRet = g_objRegManagerInterface->getMrReg(num, &objMrRegData);
		if(bRet)
		{
		    objMrRegData.comment = string(ptr);
			reg_manager_interface_setMr(&objMrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_getHr(HrRegData *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		bRet = g_objRegManagerInterface->getHrReg(num, ptr);
		HrRegData* data_ptr = ptr ;
		
		printf("getHr: id = (%f, %f, %f, %f, %f, %f) \n", 
			data_ptr->value.joint_pos[0], data_ptr->value.joint_pos[1], 
			data_ptr->value.joint_pos[2], data_ptr->value.joint_pos[3], 
			data_ptr->value.joint_pos[4], data_ptr->value.joint_pos[5]);
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setHr(HrRegData *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		HrRegData objHrRegData ;
		memcpy(&objHrRegData, ptr, sizeof(HrRegData));
		printf("setHr: id = (%f, %f, %f, %f, %f, %f) \n", 
			objHrRegData.value.joint_pos[0], objHrRegData.value.joint_pos[1], 
			objHrRegData.value.joint_pos[2], objHrRegData.value.joint_pos[3], 
			objHrRegData.value.joint_pos[4], objHrRegData.value.joint_pos[5]);
		// objHrRegData.id = num ;
		
		bRet = g_objRegManagerInterface->setHrReg(&objHrRegData);
#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
		if(bRet == false)
		{
			bRet = g_objRegManagerInterface->addHrReg(&objHrRegData);
		}
#endif
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_delHr(uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
		bRet = g_objRegManagerInterface->deleteHrReg(num);
#endif
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_getJointHr(_Joint *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		HrRegData objHrRegData ;
		bRet = g_objRegManagerInterface->getHrReg(num, &objHrRegData);
		if(bRet)
		{
		   memcpy(ptr, &(objHrRegData.value.joint_pos), 
		   	      sizeof(objHrRegData.value.joint_pos));
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setJointHr(_Joint *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		HrRegData objHrRegData ;
		bRet = g_objRegManagerInterface->getHrReg(num, &objHrRegData);
		if(bRet)
		{
		    memcpy(&(objHrRegData.value.joint_pos), ptr, 
				sizeof(objHrRegData.value.joint_pos));
			reg_manager_interface_setHr(&objHrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_getIdHr(int *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		HrRegData objHrRegData ;
		bRet = g_objRegManagerInterface->getHrReg(num, &objHrRegData);
		if(bRet)
		{
		   *ptr = objHrRegData.id;
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setIdHr(int *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		HrRegData objHrRegData ;
		bRet = g_objRegManagerInterface->getHrReg(num, &objHrRegData);
		if(bRet)
		{
		    objHrRegData.id = *ptr;
			reg_manager_interface_setHr(&objHrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_getCommentHr(char *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		HrRegData objHrRegData ;
		bRet = g_objRegManagerInterface->getHrReg(num, &objHrRegData);
		if(bRet)
		{
		   memcpy(ptr, objHrRegData.comment.c_str(), objHrRegData.comment.length());
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return bRet ;
}

bool reg_manager_interface_setCommentHr(char *ptr, uint16_t num)
{
	bool bRet = false ;
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
		HrRegData objHrRegData ;
		bRet = g_objRegManagerInterface->getHrReg(num, &objHrRegData);
		if(bRet)
		{
		    objHrRegData.comment = string(ptr);
			reg_manager_interface_setHr(&objHrRegData, num);
		}
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
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

bool reg_manager_interface_getIdUf(int *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setIdUf(int *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_getCommentUf(char *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setCommentUf(char *ptr, uint16_t num)
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

bool reg_manager_interface_getIdTf(int *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setIdTf(int *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_getCommentTf(char *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setCommentTf(char *ptr, uint16_t num)
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

bool reg_manager_interface_getPosePl(PoseEuler* pose, int index)
{
	return 0 ;
}

bool reg_manager_interface_setPosePl(PoseEuler* pose, int index)
{
	return 0 ;
}

bool reg_manager_interface_getPalletPl(pl_t *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setPalletPl(pl_t *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_getFlagPl(int *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setFlagPl(int *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_getIdPl(int *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setIdPl(int *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_getCommentPl(char *ptr, uint16_t num)
{
	return 0 ;
}

bool reg_manager_interface_setCommentPl(char *ptr, uint16_t num)
{
	return 0 ;
}

std::vector<BaseRegData> reg_manager_interface_read_valid_pr_lst(int start_id, int size)
{
    std::vector<BaseRegData> vecRet ;
	bool bRet = false ;
	vecRet.clear();
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
		vecRet = g_objRegManagerInterface->getPrRegValidIdList(0, 255);
#endif
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return vecRet ;
}

std::vector<BaseRegData> reg_manager_interface_read_valid_sr_lst(int start_id, int size)
{
    std::vector<BaseRegData> vecRet ;
	bool bRet = false ;
	vecRet.clear();
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
		vecRet = g_objRegManagerInterface->getSrRegValidIdList(0, 255);
#endif
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return vecRet ;
}

std::vector<BaseRegData> reg_manager_interface_read_valid_r_lst(int start_id, int size)
{
    std::vector<BaseRegData> vecRet ;
	bool bRet = false ;
	vecRet.clear();
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
		vecRet = g_objRegManagerInterface->getRRegValidIdList(0, 255);
#endif
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return vecRet ;
}

std::vector<BaseRegData> reg_manager_interface_read_valid_mr_lst(int start_id, int size)
{
    std::vector<BaseRegData> vecRet ;
	bool bRet = false ;
	vecRet.clear();
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
		vecRet = g_objRegManagerInterface->getMrRegValidIdList(0, 255);
	bRet = true ;
#endif
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return vecRet ;
}

std::vector<BaseRegData> reg_manager_interface_read_valid_hr_lst(int start_id, int size)
{
    std::vector<BaseRegData> vecRet ;
	bool bRet = false ;
	vecRet.clear();
#ifndef WIN32
	if(g_objRegManagerInterface)
	{
#ifdef USE_LOCAL_REG_MANAGER_INTERFACE
		vecRet = g_objRegManagerInterface->getHrRegValidIdList(0, 255);
#endif
	}
	else
	{
		printf("g_objRegManagerInterface is NULL\n");
	}
#else
	bRet = true ;
#endif
	return vecRet ;
}




