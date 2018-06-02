// #include "stdafx.h"
#include "stdio.h"
#include "string.h"
#include "setjmp.h"
#include "time.h"
#include "math.h"
#include "ctype.h"
#include "stdlib.h" 
#include "forsight_innercmd.h"
#include "motion_plan_frame_manager.h"
#include "reg_manager_interface.h"
#include "reg_manager_interface_wrapper.h"

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

#ifdef WIN32
#define USE_FAKE_DATA 
#endif
int forgesight_registers_manager_get_register(
			struct thread_control_block* objThreadCntrolBlock, 
							char *name, eval_value * value)
{	
	char reg_name[16] ;
	char reg_idx[16] ;
	char reg_member[16] ;
	int  iRegIdx = 0 ;
	char * namePtr = name ;
	char *temp = NULL ;
	char reg_content_buffer[512] ;
	
	memset(reg_member, 0x00, 16);
	memset(reg_content_buffer, 0x00, 512);

	memset(reg_name, 0x00, 16);
	temp = reg_name ;
	get_char_token(namePtr, temp);
	
	namePtr += strlen(reg_name) ;
	if(namePtr[0] != '['){
		return -1 ;
	}
	namePtr++ ;
	
	memset(reg_idx, 0x00, 16);
	temp = reg_idx ;
	get_num_token(namePtr, temp);
	iRegIdx = atoi(reg_idx);
	// namePtr += strlen(reg_idx) ;

	namePtr += strlen(reg_idx) ;
	if(namePtr[0] != ']'){
		return -1 ;
	}
	namePtr++ ;

	namePtr = strchr(namePtr, '.');
	if(namePtr)
	{
		namePtr++ ;
		memset(reg_member, 0x00, 16);
		temp = reg_member ;
		get_char_token(namePtr, temp);
	}
	memset(reg_content_buffer, 0x00, sizeof(reg_content_buffer));

	printf("forgesight_registers_manager_get_register at %s \n", reg_name);

	if(!strcmp(reg_name, TXT_PR))
	{
		if(strlen(reg_member) == 0)
		{
			reg_manager_interface_getPr(
				(void *)reg_content_buffer, iRegIdx);
			PrRegData * ptr = (PrRegData *)reg_content_buffer ;
			value->setPrRegDataValue(ptr);
		}
		else if (!strcmp(reg_member, TXT_PL_POSE))
		{
			reg_manager_interface_getPosePr(reg_content_buffer, iRegIdx);
			PoseEuler * ptr = (PoseEuler *)reg_content_buffer ;
			value->setPoseValue(ptr);
		}
		else if (!strcmp(reg_member, TXT_PL_JOINT))
		{
			reg_manager_interface_getJointPr(reg_content_buffer, iRegIdx);
			Joint * ptr = (Joint *)reg_content_buffer ;
			value->setJointValue(ptr);
		}
		else if (!strcmp(reg_member, TXT_REG_TYPE))
		{
			reg_manager_interface_getTypePr(reg_content_buffer, iRegIdx);
			value->setFloatValue(atoi(reg_content_buffer));
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			reg_manager_interface_getIdPr(reg_content_buffer, iRegIdx);
			value->setFloatValue(atoi(reg_content_buffer));
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			reg_manager_interface_getCommentPr(reg_content_buffer, iRegIdx);
			value->setStringValue(reg_content_buffer);
		}
	}
	else if(!strcmp(reg_name, TXT_SR))
	{
		if(strlen(reg_member) == 0)
		{
			reg_manager_interface_getSr(reg_content_buffer, iRegIdx);
			SrRegData * ptr = (SrRegData *)reg_content_buffer ;
			// value->setStringValue(ptr->value.c_str());
			value->setSrRegDataValue(ptr);
		}
		else if (!strcmp(reg_member, TXT_REG_VALUE))
		{
			reg_manager_interface_getValueSr(reg_content_buffer, iRegIdx);
			value->setStringValue(reg_content_buffer);
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			reg_manager_interface_getIdSr(reg_content_buffer, iRegIdx);
			value->setFloatValue(atoi(reg_content_buffer));
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			reg_manager_interface_getCommentSr(reg_content_buffer, iRegIdx);
			value->setStringValue(reg_content_buffer);
		}
	}
	else if(!strcmp(reg_name, TXT_R))
	{
	
	    printf("forgesight_registers_manager_get_register at TXT_R \n");
		if(strlen(reg_member) == 0)
		{
	        printf("reg_manager_interface_getR at TXT_R \n");
            // Use TXT_REG_VALUE
			reg_manager_interface_getR(reg_content_buffer, iRegIdx);
			RRegData * ptr = (RRegData *)reg_content_buffer ;
			// value->setFloatValue(ptr->value);
			value->setRRegDataValue(ptr);
		}
		else if (!strcmp(reg_member, TXT_REG_VALUE))
		{
			reg_manager_interface_getValueR(reg_content_buffer, iRegIdx);
			value->setFloatValue(atof(reg_content_buffer));
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			reg_manager_interface_getIdR(reg_content_buffer, iRegIdx);
			value->setFloatValue(atoi(reg_content_buffer));
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			reg_manager_interface_getCommentR(reg_content_buffer, iRegIdx);
			value->setStringValue(reg_content_buffer);
		}
	}
	else if(!strcmp(reg_name, TXT_MR))
	{
		if(strlen(reg_member) == 0)
		{
			reg_manager_interface_getMr(reg_content_buffer, iRegIdx);
			MrRegData * ptr = (MrRegData *)reg_content_buffer ;
			// value->setFloatValue(ptr->value);
			value->setMrRegDataValue(ptr);
		}
		else if (!strcmp(reg_member, TXT_REG_VALUE))
		{
			reg_manager_interface_getValueMr(reg_content_buffer, iRegIdx);
			value->setFloatValue(atoi(reg_content_buffer));
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			reg_manager_interface_getIdMr(reg_content_buffer, iRegIdx);
			value->setFloatValue(atoi(reg_content_buffer));
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			reg_manager_interface_getCommentMr(reg_content_buffer, iRegIdx);
			value->setStringValue(reg_content_buffer);
		}
	}
	else if(!strcmp(reg_name, TXT_UF))
	{
//		if(strlen(reg_member) == 0)
//		{
//            // Use TXT_UF_TF_COORDINATE
//			reg_manager_interface_getUf(reg_content_buffer, iRegIdx);
//			uf_shmi_t * ptr = (uf_shmi_t *)reg_content_buffer ;
//			value->setPoseValue(&(ptr->c));
//		}
//		else 
		if (!strcmp(reg_member, TXT_UF_TF_COORDINATE))
		{
			reg_manager_interface_getCoordinateUf(reg_content_buffer, iRegIdx);
			PoseEuler * ptr = (PoseEuler *)reg_content_buffer ;
			value->setPoseValue(ptr);
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			reg_manager_interface_getIdUf(reg_content_buffer, iRegIdx);
			value->setFloatValue(atoi(reg_content_buffer));
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			reg_manager_interface_getCommentUf(reg_content_buffer, iRegIdx);
			value->setStringValue(reg_content_buffer);
		}
	}
	else if(!strcmp(reg_name, TXT_TF))
	{
//		if(strlen(reg_member) == 0)
//		{
//			reg_manager_interface_getTf(reg_content_buffer, iRegIdx);
//			tf_shmi_t * ptr = (tf_shmi_t *)reg_content_buffer ;
//			value->setPoseValue(&(ptr->c));
//		}
//		else 
		if (!strcmp(reg_member, TXT_UF_TF_COORDINATE))
		{
			reg_manager_interface_getCoordinateTf(reg_content_buffer, iRegIdx);
			PoseEuler * ptr = (PoseEuler *)reg_content_buffer ;
			value->setPoseValue(ptr);
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			reg_manager_interface_getIdTf(reg_content_buffer, iRegIdx);
			value->setFloatValue(atoi(reg_content_buffer));
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			reg_manager_interface_getCommentTf(reg_content_buffer, iRegIdx);
			value->setStringValue(reg_content_buffer);
		}
	}
	else if(!strcmp(reg_name, TXT_PL))
	{
//		if(strlen(reg_member) == 0)
//		{
//            // Use TXT_PL_POSE
//			reg_manager_interface_getPl(reg_content_buffer, iRegIdx);
//			pl_shmi_t * ptr = (pl_shmi_t *)reg_content_buffer ;
//			value->setPLValue(&(ptr->pallet));
//		}
//		else 
		if (!strcmp(reg_member, TXT_PL_POSE))
		{
			reg_manager_interface_getPalletPl(reg_content_buffer, iRegIdx);
			PoseEuler * ptr = (PoseEuler *)reg_content_buffer ;
			value->setPoseValue(ptr);
		}
//		else if (!strcmp(reg_member, TXT_PL_PALLET))
//		{
//			reg_manager_interface_getPalletPl(reg_content_buffer, iRegIdx);
//			pl_t * ptr = (pl_t *)reg_content_buffer ;
//			value->setPLValue(ptr);
//		}
		else if (!strcmp(reg_member, TXT_PL_FLAG))
		{
			reg_manager_interface_getFlagPl(reg_content_buffer, iRegIdx);
			value->setFloatValue(atoi(reg_content_buffer));
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			reg_manager_interface_getIdPl(reg_content_buffer, iRegIdx);
			value->setFloatValue(atoi(reg_content_buffer));
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			reg_manager_interface_getCommentPl(reg_content_buffer, iRegIdx);
			value->setStringValue(reg_content_buffer);
		}
	}
	return 0 ;
}

int forgesight_registers_manager_set_register(
		struct thread_control_block* objThreadCntrolBlock, 
		char *name, eval_value * valueStart)
{
	eval_value value;
	int boolValue;

	char reg_name[16] ;
	char reg_idx[16] ;
	char reg_member[16] ;
	int  iRegIdx = 0 ;
	char * namePtr = name ;
	char *temp = NULL ;
	
	PoseEuler pose ;
	Joint joint ;
	
	memset(reg_name, 0x00, 16);
	temp = reg_name ;
	get_char_token(namePtr, temp);
	if(name[strlen(reg_name)] != '['){
		return -1 ;
	}
	namePtr += strlen(reg_name) ;
	namePtr++ ;
	
	memset(reg_idx, 0x00, 16);
	temp = reg_idx ;
	get_num_token(namePtr, temp);
	iRegIdx = atoi(reg_idx);
	// namePtr += strlen(reg_idx) ;
	
	namePtr = strchr(namePtr, '.');
	memset(reg_member, 0x00, 16);
	if(namePtr)
	{
		namePtr++ ;
		temp = reg_member ;
		get_char_token(namePtr, temp);
	}

	if(!strcmp(reg_name, TXT_PR))
	{
		if(strlen(reg_member) == 0)
		{
			reg_manager_interface_setPr(valueStart, iRegIdx);
	       	return 0 ;
		}
		else if (!strcmp(reg_member, TXT_PL_POSE))
		{
			if (valueStart->getType() == TYPE_FLOAT)
			{
				pose.position.x = (double)valueStart->getFloatValue();

				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.position.y = (double)value.getFloatValue();

				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.position.z = (double)value.getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.orientation.a = (double)value.getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.orientation.b = (double)value.getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.orientation.c = (double)value.getFloatValue();

				printf("Set POSE:(%f, %f, %f, %f, %f, %f) to PR[%s]\n", 
					pose.position.x, pose.position.y, pose.position.z, 
					pose.orientation.a, pose.orientation.b, pose.orientation.c,
					reg_idx);
				reg_manager_interface_setPosePr(&pose, iRegIdx);
	 	      	return 0 ;
			}
			else if (valueStart->getType() == TYPE_POSE)
			{
				pose = valueStart->getPoseValue();
				printf("Set POSE:(%f, %f, %f, %f, %f, %f) to PR[%s]\n", 
					pose.position.x, pose.position.y, pose.position.z, 
					pose.orientation.a, pose.orientation.b, pose.orientation.c,
					reg_idx);
				reg_manager_interface_setPosePr(&pose, iRegIdx);
	    	   	return 0 ;
			}
		}
		else if (!strcmp(reg_member, TXT_PL_JOINT))
		{
			if (valueStart->getType() == TYPE_FLOAT)
			{
				joint.j1 = (double)valueStart->getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				joint.j2 = (double)value.getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				joint.j3 = (double)value.getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				joint.j4 = (double)value.getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				joint.j5 = (double)value.getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				joint.j6 = (double)value.getFloatValue();
				
				printf("Set JOINT:(%f, %f, %f, %f, %f, %f) to PR[%s]\n", 
					joint.j1, joint.j2, joint.j3, joint.j4, joint.j5, joint.j6, 
					reg_idx);
				reg_manager_interface_setJointPr(&joint, iRegIdx);
	    	   	return 0 ;
			}
			else if (valueStart->getType() == TYPE_JOINT)
			{
				joint = valueStart->getJointValue();
				printf("Set JOINT:(%f, %f, %f, %f, %f, %f) to PR[%s]\n", 
					joint.j1, joint.j2, joint.j3, joint.j4, joint.j5, joint.j6, 
					reg_idx);
				reg_manager_interface_setJointPr(&joint, iRegIdx);
	    	  	return 0 ;
			}
			
		}
		else if (!strcmp(reg_member, TXT_REG_TYPE))
		{
			int iType = (int)atof((char *)valueStart);
			printf("Set TYPE:(%d) to PR[%s]\n", iType, reg_idx);
			reg_manager_interface_setTypePr(&iType, iRegIdx);
	       	return 0 ;
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			int iID = (int)atof((char *)valueStart);
			printf("Set ID:(%d) to PR[%s]\n", iID, reg_idx);
			reg_manager_interface_setIdPr(&iID, iRegIdx);
	       	return 0 ;
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			get_token(objThreadCntrolBlock);
			printf("Set COMMENT:(%s) to PR[%s]\n", 
				objThreadCntrolBlock->token, reg_idx);
			reg_manager_interface_setCommentPr(objThreadCntrolBlock->token, iRegIdx);
	       	return 0 ;
		}
	}
	else if(!strcmp(reg_name, TXT_SR))
	{
		if(strlen(reg_member) == 0)
		{
			reg_manager_interface_setSr(valueStart, iRegIdx);
	       	return 0 ;
		}
		else if (!strcmp(reg_member, TXT_REG_VALUE))
		{
			get_token(objThreadCntrolBlock);
			printf("Set VALUE:(%s) to SR[%s]\n", 
				objThreadCntrolBlock->token, reg_idx);
			reg_manager_interface_setValueSr(objThreadCntrolBlock->token, iRegIdx);
	       	return 0 ;
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			int iID = (int)atof((char *)valueStart);
			printf("Set ID:(%d) to SR[%s]\n", iID, reg_idx);
			reg_manager_interface_setIdSr(valueStart, iRegIdx);
	       	return 0 ;
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			get_token(objThreadCntrolBlock);
			printf("Set COMMENT:(%s) to SR[%s]\n", 
				objThreadCntrolBlock->token, reg_idx);
			reg_manager_interface_setCommentSr(valueStart, iRegIdx);
	       	return 0 ;
		}
	}
	else if(!strcmp(reg_name, TXT_R))
	{
		if(strlen(reg_member) == 0)
		{
			reg_manager_interface_setR(&(valueStart->getRRegDataValue()), iRegIdx);
	       	return 0 ;
		}
		else if (!strcmp(reg_member, TXT_REG_VALUE))
		{
			float fValue = atof((char *)valueStart);
			printf("Set VALUE:(%f) to SR[%s]\n", fValue, reg_idx);
			reg_manager_interface_setValueR(valueStart, iRegIdx);
	       	return 0 ;
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			int iID = (int)atof((char *)valueStart);
			printf("Set ID:(%d) to SR[%s]\n", iID, reg_idx);
			reg_manager_interface_setIdR(valueStart, iRegIdx);
	       	return 0 ;
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			get_token(objThreadCntrolBlock);
			printf("Set COMMENT:(%s) to SR[%s]\n", 
				objThreadCntrolBlock->token, reg_idx);
			reg_manager_interface_setCommentR(valueStart, iRegIdx);
	       	return 0 ;
		}
	}
	else if(!strcmp(reg_name, TXT_MR))
	{
		if(strlen(reg_member) == 0)
		{
			reg_manager_interface_setMr(valueStart, iRegIdx);
	       	return 0 ;
		}
		else if (!strcmp(reg_member, TXT_REG_VALUE))
		{
			int iValue = atof((char *)valueStart);
			printf("Set VALUE:(%f) to MR[%s]\n", iValue, reg_idx);
			reg_manager_interface_setValueMr(valueStart, iRegIdx);
	       	return 0 ;
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			int iID = (int)atof((char *)valueStart);
			printf("Set ID:(%d) to MR[%s]\n", iID, reg_idx);
			reg_manager_interface_setIdMr(valueStart, iRegIdx);
	       	return 0 ;
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			get_token(objThreadCntrolBlock);
			printf("Set COMMENT:(%s) to MR[%s]\n", 
				objThreadCntrolBlock->token, reg_idx);
			reg_manager_interface_setCommentMr(valueStart, iRegIdx);
	       	return 0 ;
		}
	}
	else if(!strcmp(reg_name, TXT_UF))
	{
		if(strlen(reg_member) == 0)
		{
			reg_manager_interface_setUf(valueStart, iRegIdx);
	       	return 0 ;
		}
		else if (!strcmp(reg_member, TXT_UF_TF_COORDINATE))
		{
			if (valueStart->getType() == TYPE_FLOAT)
			{
				pose.position.x = (double)(atof((char *)valueStart));
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.position.y = (double)value.getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.position.z = (double)value.getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.orientation.a = (double)value.getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.orientation.b = (double)value.getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.orientation.c = (double)value.getFloatValue();
				
				printf("Set COORDINATE:(%f, %f, %f, %f, %f, %f) to UF[%s]\n", 
					pose.position.x, pose.position.y, pose.position.z, 
					pose.orientation.a, pose.orientation.b, pose.orientation.c,
					reg_idx);
				reg_manager_interface_setCoordinateUf(&pose, iRegIdx);
	       		return 0 ;
			}
			else if (valueStart->getType() == TYPE_POSE)
			{
				pose = valueStart->getPoseValue();
				printf("Set POSE:(%f, %f, %f, %f, %f, %f) to PR[%s]\n", 
					pose.position.x, pose.position.y, pose.position.z, 
					pose.orientation.a, pose.orientation.b, pose.orientation.c,
					reg_idx);
				reg_manager_interface_setCoordinateUf(&pose, iRegIdx);
	        	return 0 ;
			}
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			int iID = (int)atof((char *)valueStart);
			printf("Set ID:(%d) to MR[%s]\n", iID, reg_idx);
			reg_manager_interface_setIdUf(valueStart, iRegIdx);
	        return 0 ;
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			get_token(objThreadCntrolBlock);
			printf("Set COMMENT:(%s) to MR[%s]\n", 
				objThreadCntrolBlock->token, reg_idx);
			reg_manager_interface_setCommentUf(valueStart, iRegIdx);
	        return 0 ;
		}
	}
	else if(!strcmp(reg_name, TXT_TF))
	{
		if(strlen(reg_member) == 0)
		{
			reg_manager_interface_setTf(valueStart, iRegIdx);
	        return 0 ;
		}
		else if (!strcmp(reg_member, TXT_UF_TF_COORDINATE))
		{
			if (valueStart->getType() == TYPE_FLOAT)
			{
				pose.position.x = (double)(atof((char *)valueStart));
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.position.y = (double)value.getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.position.z = (double)value.getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.orientation.a = (double)value.getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.orientation.b = (double)value.getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.orientation.c = (double)value.getFloatValue();
				
				printf("Set COORDINATE:(%f, %f, %f, %f, %f, %f) to TF[%s]\n", 
					pose.position.x, pose.position.y, pose.position.z, 
					pose.orientation.a, pose.orientation.b, pose.orientation.c,
					reg_idx);
				reg_manager_interface_setCoordinateTf(&pose, iRegIdx);
	        	return 0 ;
			}
			else if (valueStart->getType() == TYPE_POSE)
			{
				pose = valueStart->getPoseValue();
				printf("Set POSE:(%f, %f, %f, %f, %f, %f) to PR[%s]\n", 
					pose.position.x, pose.position.y, pose.position.z, 
					pose.orientation.a, pose.orientation.b, pose.orientation.c,
					reg_idx);
				reg_manager_interface_setCoordinateTf(&pose, iRegIdx);
	        	return 0 ;
			}
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			int iID = (int)atof((char *)valueStart);
			printf("Set ID:(%d) to MR[%s]\n", iID, reg_idx);
			reg_manager_interface_setIdTf(valueStart, iRegIdx);
	        return 0 ;
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			get_token(objThreadCntrolBlock);
			printf("Set COMMENT:(%s) to MR[%s]\n", 
				objThreadCntrolBlock->token, reg_idx);
			reg_manager_interface_setCommentTf(valueStart, iRegIdx);
	        return 0 ;
		}
	}
	else if(!strcmp(reg_name, TXT_PL))
	{
		if(strlen(reg_member) == 0)
		{
			reg_manager_interface_setPl(valueStart, iRegIdx);
	        return 0 ;
		}
		else if (!strcmp(reg_member, TXT_PL_POSE))
		{
			if (valueStart->getType() == TYPE_FLOAT)
			{
				pose.position.x = (double)(atof((char *)valueStart));
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.position.y = (double)value.getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.position.z = (double)value.getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.orientation.a = (double)value.getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.orientation.b = (double)value.getFloatValue();
				
				get_exp(objThreadCntrolBlock, &value, &boolValue);
				pose.orientation.c = (double)value.getFloatValue();
				
				printf("Set COORDINATE:(%f, %f, %f, %f, %f, %f) to TF[%s]\n", 
					pose.position.x, pose.position.y, pose.position.z, 
					pose.orientation.a, pose.orientation.b, pose.orientation.c,
					reg_idx);
				reg_manager_interface_setPosePl(pose, iRegIdx);
	        	return 0 ;
			}
			else if (valueStart->getType() == TYPE_POSE)
			{
				pose = valueStart->getPoseValue();
				printf("Set POSE:(%f, %f, %f, %f, %f, %f) to PR[%s]\n", 
					pose.position.x, pose.position.y, pose.position.z, 
					pose.orientation.a, pose.orientation.b, pose.orientation.c,
					reg_idx);
				reg_manager_interface_setPosePl(pose, iRegIdx);
	        	return 0 ;
			}
		}
		else if (!strcmp(reg_member, TXT_PL_PALLET))
		{
			reg_manager_interface_setPalletPl(valueStart, iRegIdx);
	        return 0 ;
		}
		else if (!strcmp(reg_member, TXT_PL_FLAG))
		{
			int iID = (int)atof((char *)valueStart);
			printf("Set ID:(%d) to MR[%s]\n", iID, reg_idx);
			reg_manager_interface_setFlagPl(valueStart, iRegIdx);
	        return 0 ;
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			int iID = (int)atof((char *)valueStart);
			printf("Set ID:(%d) to MR[%s]\n", iID, reg_idx);
			reg_manager_interface_setIdPl(valueStart, iRegIdx);
	        return 0 ;
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			get_token(objThreadCntrolBlock);
			printf("Set COMMENT:(%s) to MR[%s]\n", 
				objThreadCntrolBlock->token, reg_idx);
			reg_manager_interface_setCommentPl(valueStart, iRegIdx);
	        return 0 ;
		}
	}
	return 0 ;
}

int forgesight_read_reg(RegMap & reg)
{
	 switch(reg.type)
	 {
	 // pose register
	 case POSE_REG:
		 reg_manager_interface_getPr(reg.value, reg.index);
		 break;
	 case POSE_REG_POSE:
		 reg_manager_interface_getPosePr(reg.value, reg.index);
		 break;
	 case POSE_REG_JOINT:
		 reg_manager_interface_getJointPr(reg.value, reg.index);
		 break;
	 case POSE_REG_TYPE:
		 reg_manager_interface_getTypePr(reg.value, reg.index);
		 break;
	 case POSE_REG_ID:
		 reg_manager_interface_getIdPr(reg.value, reg.index);
		 break;
	 case POSE_REG_COMMENT:
		 reg_manager_interface_getCommentPr(reg.value, reg.index);
		 break;
	 // string register
	 case STR_REG:
	     reg_manager_interface_getSr(reg.value, reg.index);
	 	 break;
	 case STR_REG_VALUE:
	     reg_manager_interface_getValueSr(reg.value, reg.index);
	 	 break;
	 case STR_REG_ID:
	     reg_manager_interface_getIdSr(reg.value, reg.index);
	 	 break;
	 case STR_REG_COMMENT:
	    reg_manager_interface_getCommentSr(reg.value, reg.index);
	 	break;
	 // number register
	 case NUM_REG:
	    reg_manager_interface_getR(reg.value, reg.index);
	 	break;
	 case NUM_REG_VALUE:
	    reg_manager_interface_getValueR(reg.value, reg.index);
	 	break;
	 case NUM_REG_ID:
	    reg_manager_interface_getIdR(reg.value, reg.index);
	 	break;
	 case NUM_REG_COMMENT:
	    reg_manager_interface_getCommentR(reg.value, reg.index);
	 	break;
	 // Special register for motion instruction
	 case MOT_REG:
	    reg_manager_interface_getMr(reg.value, reg.index);
	 	break;
	 case MOT_REG_VALUE:
	    reg_manager_interface_getValueMr(reg.value, reg.index);
	 	break;
	 case MOT_REG_ID:
	    reg_manager_interface_getIdMr(reg.value, reg.index);
	 	break;
	 case MOT_REG_COMMENT:
	    reg_manager_interface_getCommentMr(reg.value, reg.index);
	 	break;
	 // register of user coordinate offset
	 case UF_REG:
	    reg_manager_interface_getUf(reg.value, reg.index);
	 	break;
	 case UF_REG_COORD:
	    reg_manager_interface_getCoordinateUf(reg.value, reg.index);
	 	break;
	 case UF_REG_ID:
	    reg_manager_interface_getIdUf(reg.value, reg.index);
	 	break;
	 case UF_REG_COMMENT:
	    reg_manager_interface_getCommentUf(reg.value, reg.index);
	 	break;
	 // register of tool coordinate offset
	 case TF_REG:
	    reg_manager_interface_getTf(reg.value, reg.index);
	 	break;
	 case TF_REG_COORD:
	    reg_manager_interface_getCoordinateTf(reg.value, reg.index);
	 	break;
	 case TF_REG_ID:
	    reg_manager_interface_getIdTf(reg.value, reg.index);
	 	break;
	 case TF_REG_COMMENT:
	    reg_manager_interface_getCommentTf(reg.value, reg.index);
	 	break;
	 // stack register
	 case PL_REG:
	    reg_manager_interface_getPl(reg.value, reg.index);
	 	break;
	 case PL_REG_POSE:
	    reg_manager_interface_getPosePl((PoseEuler &)(reg.value), reg.index);
	 	break;
	 case PL_REG_PALLET:
	    reg_manager_interface_getPalletPl(reg.value, reg.index);
	 	break;
	 case PL_REG_FLAG:
	    reg_manager_interface_getFlagPl(reg.value, reg.index);
	 	break;
	 case PL_REG_ID:
	    reg_manager_interface_getIdPl(reg.value, reg.index);
	 	break;
	 case PL_REG_COMMENT:
	    reg_manager_interface_getCommentPl(reg.value, reg.index);
	 	break;
 	 }
	 return 1;
}

int forgesight_mod_reg(RegMap & reg)
{
  	 printf("reg.type = %d.\n", reg.type);
         printf("reg.value = (");
         for(int iRet = 0 ; iRet < 100 ; iRet++)
         {
              printf("%08X, ", reg.value[iRet]);
         }
         printf(") \n");

	 switch(reg.type)
	 {
	 // pose register
	 case POSE_REG:
		 reg_manager_interface_setPr(reg.value, reg.index);
		 break;
	 case POSE_REG_POSE:
		 reg_manager_interface_setPosePr(reg.value, reg.index);
		 break;
	 case POSE_REG_JOINT:
		 reg_manager_interface_setJointPr(reg.value, reg.index);
		 break;
	 case POSE_REG_TYPE:
		 reg_manager_interface_setTypePr(reg.value, reg.index);
		 break;
	 case POSE_REG_ID:
		 reg_manager_interface_setIdPr(reg.value, reg.index);
		 break;
	 case POSE_REG_COMMENT:
		 reg_manager_interface_setCommentPr(reg.value, reg.index);
		 break;
	 // string register
	 case STR_REG:
	    reg_manager_interface_setSr(reg.value, reg.index);
	 	break;
	 case STR_REG_VALUE:
	    reg_manager_interface_setValueSr(reg.value, reg.index);
	 	break;
	 case STR_REG_ID:
	    reg_manager_interface_setIdSr(reg.value, reg.index);
	 	break;
	 case STR_REG_COMMENT:
	    reg_manager_interface_setCommentSr(reg.value, reg.index);
	 	break;
	 // number register
	 case NUM_REG:
	    reg_manager_interface_setR(reg.value, reg.index);
	 	break;
	 case NUM_REG_VALUE:
	    reg_manager_interface_setValueR(reg.value, reg.index);
	 	break;
	 case NUM_REG_ID:
	    reg_manager_interface_setIdR(reg.value, reg.index);
	 	break;
	 case NUM_REG_COMMENT:
	    reg_manager_interface_setCommentR(reg.value, reg.index);
	 	break;
	 // Special register for motion instruction
	 case MOT_REG:
	    reg_manager_interface_setMr(reg.value, reg.index);
	 	break;
	 case MOT_REG_VALUE:
	    reg_manager_interface_setValueMr(reg.value, reg.index);
	 	break;
	 case MOT_REG_ID:
	    reg_manager_interface_setIdMr(reg.value, reg.index);
	 	break;
	 case MOT_REG_COMMENT:
	    reg_manager_interface_setCommentMr(reg.value, reg.index);
	 	break;
	 // register of user coordinate offset
	 case UF_REG:
	    reg_manager_interface_setUf(reg.value, reg.index);
	 	break;
	 case UF_REG_COORD:
	    reg_manager_interface_setCoordinateUf(reg.value, reg.index);
	 	break;
	 case UF_REG_ID:
	    reg_manager_interface_setIdUf(reg.value, reg.index);
	 	break;
	 case UF_REG_COMMENT:
	    reg_manager_interface_setCommentUf(reg.value, reg.index);
	 	break;
	 // register of tool coordinate offset
	 case TF_REG:
	    reg_manager_interface_setTf(reg.value, reg.index);
	 	break;
	 case TF_REG_COORD:
	    reg_manager_interface_setCoordinateTf(reg.value, reg.index);
	 	break;
	 case TF_REG_ID:
	    reg_manager_interface_setIdTf(reg.value, reg.index);
	 	break;
	 case TF_REG_COMMENT:
	    reg_manager_interface_setCommentTf(reg.value, reg.index);
	 	break;
	 // stack register
	 case PL_REG:
	    reg_manager_interface_setPl(reg.value, reg.index);
	 	break;
	 case PL_REG_POSE:
	    // reg_manager_interface_setPosePl(reg.value, reg.index);
	 	break;
	 case PL_REG_PALLET:
	    reg_manager_interface_setPalletPl(reg.value, reg.index);
	 	break;
	 case PL_REG_FLAG:
	    reg_manager_interface_setFlagPl(reg.value, reg.index);
	 	break;
	 case PL_REG_ID:
	    reg_manager_interface_setIdPl(reg.value, reg.index);
	 	break;
	 case PL_REG_COMMENT:
	    reg_manager_interface_setCommentPl(reg.value, reg.index);
	 	break;
 	 }
  	 printf("reg.type = %d end.\n", reg.type);
	 return 1;
}

std::vector<BaseRegData> forgesight_read_chg_pr_lst(int start_id, int size)
{
	return reg_manager_interface_read_chg_pr_lst(start_id, size);
}

std::vector<BaseRegData> forgesight_read_chg_sr_lst(int start_id, int size)
{
	return reg_manager_interface_read_chg_sr_lst(start_id, size);
}

std::vector<BaseRegData> forgesight_read_chg_r_lst(int start_id, int size)
{
	return reg_manager_interface_read_chg_r_lst(start_id, size);
}

std::vector<BaseRegData> forgesight_read_chg_mr_lst(int start_id, int size)
{
	return reg_manager_interface_read_chg_mr_lst(start_id, size);
}

