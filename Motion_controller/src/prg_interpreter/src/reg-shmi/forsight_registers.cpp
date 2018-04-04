// #include "stdafx.h"
#include "stdio.h"
#include "string.h"
#include "setjmp.h"
#include "time.h"
#include "math.h"
#include "ctype.h"
#include "stdlib.h" 
#include "forsight_innercmd.h"
#include "reg-shmi/forsight_op_regs_shmi.h"
#include "reg-shmi/forsight_registers.h"
#include "interpreter_common.h"

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

#define USE_FAKE_DATA 
int forgesight_get_register(struct thread_control_block* objThreadCntrolBlock, 
							char *name, eval_value * value)
{
	PoseEuler pose ;
	Joint joint ;
	pl_t  plt ;
	
	char reg_name[16] ;
	char reg_idx[16] ;
	char reg_member[16] ;
	int  iRegIdx = 0 ;
	char * namePtr = name ;
	char *temp = NULL ;

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

	if(!strcmp(reg_name, TXT_PR))
	{
		if(strlen(reg_member) == 0)
		{
			getPr(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_PL_POSE))
		{
#ifdef USE_FAKE_DATA
			pose.position.x = pose.position.y = pose.position.z = 1.1 ;
			pose.orientation.a = pose.orientation.b = pose.orientation.c = 2.2 ;
			value->setPoseValue(&pose);
#endif
			getPosePr(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_PL_JOINT))
		{
#ifdef USE_FAKE_DATA
			joint.j1 = joint.j2 = joint.j3 = 1.1 ;
			joint.j4 = joint.j5 = joint.j6 = 2.2 ;
			value->setJointValue(&joint);
#endif
			getJointPr(value, iRegIdx);

		}
		else if (!strcmp(reg_member, TXT_REG_TYPE))
		{
#ifdef USE_FAKE_DATA
			value->setFloatValue((float)1.1);
#endif
			getTypePr(value, iRegIdx);

		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
#ifdef USE_FAKE_DATA
			value->setFloatValue((float)1.1);
#endif
			getIdPr(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
#ifdef USE_FAKE_DATA
			value->setStringValue("Test Comment");
#endif
			getCommentPr(value, iRegIdx);
		}
	}
	else if(!strcmp(reg_name, TXT_SR))
	{
		if(strlen(reg_member) == 0)
		{
			getSr(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_VALUE))
		{
#ifdef USE_FAKE_DATA
			value->setStringValue("string register");
#endif
			getValueSr(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
#ifdef USE_FAKE_DATA
			value->setFloatValue((float)1.1);
#endif
			getIdSr(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
#ifdef USE_FAKE_DATA
			value->setStringValue("Test Comment");
#endif
			getCommentSr(value, iRegIdx);
		}
	}
	else if(!strcmp(reg_name, TXT_R))
	{
		if(strlen(reg_member) == 0)
		{
			getR(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_VALUE))
		{
#ifdef USE_FAKE_DATA
			value->setFloatValue((float)1.1);
#endif
			getValueR(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
#ifdef USE_FAKE_DATA
			value->setFloatValue((float)1.1);
#endif
			getIdR(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
#ifdef USE_FAKE_DATA
			value->setStringValue("Test Comment");
#endif
			getCommentR(value, iRegIdx);
		}
	}
	else if(!strcmp(reg_name, TXT_MR))
	{
		if(strlen(reg_member) == 0)
		{
			getMr(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_VALUE))
		{
#ifdef USE_FAKE_DATA
			value->setFloatValue(1);
#endif
			getValueMr(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
#ifdef USE_FAKE_DATA
			value->setFloatValue((float)1.1);
#endif
			getIdMr(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
#ifdef USE_FAKE_DATA
			value->setStringValue("Test Comment");
#endif
			getCommentMr(value, iRegIdx);
		}
	}
	else if(!strcmp(reg_name, TXT_UF))
	{
		if(strlen(reg_member) == 0)
		{
			getUf(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_UF_TF_COORDINATE))
		{
#ifdef USE_FAKE_DATA
			pose.position.x = pose.position.y = pose.position.z = 1.1 ;
			pose.orientation.a = pose.orientation.b = pose.orientation.c = 2.2 ;
			value->setPoseValue(&pose);
#endif
			getCoordinateUf(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
#ifdef USE_FAKE_DATA
			value->setFloatValue((float)1.1);
#endif
			getIdUf(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
#ifdef WIN32
			value->setStringValue("Test Comment");
#endif
			getCommentUf(value, iRegIdx);
		}
	}
	else if(!strcmp(reg_name, TXT_TF))
	{
		if(strlen(reg_member) == 0)
		{
			getTf(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_UF_TF_COORDINATE))
		{
#ifdef USE_FAKE_DATA
			pose.position.x = pose.position.y = pose.position.z = 1.1 ;
			pose.orientation.a = pose.orientation.b = pose.orientation.c = 2.2 ;
			value->setPoseValue(&pose);
#endif
			getCoordinateTf(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
#ifdef USE_FAKE_DATA
			value->setFloatValue((float)1.1);
#endif
			getIdTf(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
#ifdef WIN32
			value->setStringValue("Test Comment");
#endif
			getCommentTf(value, iRegIdx);
		}
	}
	else if(!strcmp(reg_name, TXT_PL))
	{
		if(strlen(reg_member) == 0)
		{
			getPl(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_PL_POSE))
		{
#ifdef USE_FAKE_DATA
			pose.position.x = pose.position.y = pose.position.z = 1.1 ;
			pose.orientation.a = pose.orientation.b = pose.orientation.c = 2.2 ;
			value->setPoseValue(&pose);
#endif
			getPalletPl(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_PL_PALLET))
		{
#ifdef USE_FAKE_DATA
			plt.column = plt.layer = plt.row = 0 ;
			value->setPLValue(&plt);
#endif
			getPalletPl(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_PL_FLAG))
		{
#ifdef USE_FAKE_DATA
			value->setFloatValue((float)1.1);
#endif
			getFlagPl(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
#ifdef USE_FAKE_DATA
			value->setFloatValue((float)1.1);
#endif
			getIdPl(value, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
#ifdef USE_FAKE_DATA
			value->setStringValue("Test Comment");
#endif
			getCommentPl(value, iRegIdx);
		}
	}
	return 0 ;
}

int forgesight_set_register(struct thread_control_block* objThreadCntrolBlock, 
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
			setPr(valueStart, iRegIdx);
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
				setPosePr(&pose, iRegIdx);
			}
			else if (valueStart->getType() == TYPE_POSE)
			{
				pose = valueStart->getPoseValue();
				printf("Set POSE:(%f, %f, %f, %f, %f, %f) to PR[%s]\n", 
					pose.position.x, pose.position.y, pose.position.z, 
					pose.orientation.a, pose.orientation.b, pose.orientation.c,
					reg_idx);
				setPosePr(&pose, iRegIdx);
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
				setJointPr(&joint, iRegIdx);
			}
			else if (valueStart->getType() == TYPE_JOINT)
			{
				joint = valueStart->getJointValue();
				printf("Set JOINT:(%f, %f, %f, %f, %f, %f) to PR[%s]\n", 
					joint.j1, joint.j2, joint.j3, joint.j4, joint.j5, joint.j6, 
					reg_idx);
				setJointPr(&joint, iRegIdx);
			}
			
		}
		else if (!strcmp(reg_member, TXT_REG_TYPE))
		{
			int iType = (int)atof((char *)valueStart);
			printf("Set TYPE:(%d) to PR[%s]\n", iType, reg_idx);
			setTypePr(&iType, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			int iID = (int)atof((char *)valueStart);
			printf("Set ID:(%d) to PR[%s]\n", iID, reg_idx);
			setIdPr(&iID, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			get_token(objThreadCntrolBlock);
			printf("Set COMMENT:(%s) to PR[%s]\n", 
				objThreadCntrolBlock->token, reg_idx);
			setCommentPr(objThreadCntrolBlock->token, iRegIdx);
		}
	}
	else if(!strcmp(reg_name, TXT_SR))
	{
		if(strlen(reg_member) == 0)
		{
			setSr(valueStart, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_VALUE))
		{
			get_token(objThreadCntrolBlock);
			printf("Set VALUE:(%s) to SR[%s]\n", 
				objThreadCntrolBlock->token, reg_idx);
			setValueSr(objThreadCntrolBlock->token, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			int iID = (int)atof((char *)valueStart);
			printf("Set ID:(%d) to SR[%s]\n", iID, reg_idx);
			setIdSr(valueStart, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			get_token(objThreadCntrolBlock);
			printf("Set COMMENT:(%s) to SR[%s]\n", 
				objThreadCntrolBlock->token, reg_idx);
			setCommentSr(valueStart, iRegIdx);
		}
	}
	else if(!strcmp(reg_name, TXT_R))
	{
		if(strlen(reg_member) == 0)
		{
			setR(valueStart, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_VALUE))
		{
			float fValue = atof((char *)valueStart);
			printf("Set VALUE:(%f) to SR[%s]\n", fValue, reg_idx);
			setValueR(valueStart, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			int iID = (int)atof((char *)valueStart);
			printf("Set ID:(%d) to SR[%s]\n", iID, reg_idx);
			setIdR(valueStart, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			get_token(objThreadCntrolBlock);
			printf("Set COMMENT:(%s) to SR[%s]\n", 
				objThreadCntrolBlock->token, reg_idx);
			setCommentR(valueStart, iRegIdx);
		}
	}
	else if(!strcmp(reg_name, TXT_MR))
	{
		if(strlen(reg_member) == 0)
		{
			setMr(valueStart, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_VALUE))
		{
			int iValue = atof((char *)valueStart);
			printf("Set VALUE:(%f) to MR[%s]\n", iValue, reg_idx);
			setValueMr(valueStart, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			int iID = (int)atof((char *)valueStart);
			printf("Set ID:(%d) to MR[%s]\n", iID, reg_idx);
			setIdMr(valueStart, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			get_token(objThreadCntrolBlock);
			printf("Set COMMENT:(%s) to MR[%s]\n", 
				objThreadCntrolBlock->token, reg_idx);
			setCommentMr(valueStart, iRegIdx);
		}
	}
	else if(!strcmp(reg_name, TXT_UF))
	{
		if(strlen(reg_member) == 0)
		{
			setUf(valueStart, iRegIdx);
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
				setCoordinateUf(&pose, iRegIdx);
			}
			else if (valueStart->getType() == TYPE_POSE)
			{
				pose = valueStart->getPoseValue();
				printf("Set POSE:(%f, %f, %f, %f, %f, %f) to PR[%s]\n", 
					pose.position.x, pose.position.y, pose.position.z, 
					pose.orientation.a, pose.orientation.b, pose.orientation.c,
					reg_idx);
				setCoordinateUf(&pose, iRegIdx);
			}
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			int iID = (int)atof((char *)valueStart);
			printf("Set ID:(%d) to MR[%s]\n", iID, reg_idx);
			setIdUf(valueStart, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			get_token(objThreadCntrolBlock);
			printf("Set COMMENT:(%s) to MR[%s]\n", 
				objThreadCntrolBlock->token, reg_idx);
			setCommentUf(valueStart, iRegIdx);
		}
	}
	else if(!strcmp(reg_name, TXT_TF))
	{
		if(strlen(reg_member) == 0)
		{
			setTf(valueStart, iRegIdx);
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
				setCoordinateTf(&pose, iRegIdx);
			}
			else if (valueStart->getType() == TYPE_POSE)
			{
				pose = valueStart->getPoseValue();
				printf("Set POSE:(%f, %f, %f, %f, %f, %f) to PR[%s]\n", 
					pose.position.x, pose.position.y, pose.position.z, 
					pose.orientation.a, pose.orientation.b, pose.orientation.c,
					reg_idx);
				setCoordinateTf(&pose, iRegIdx);
			}
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			int iID = (int)atof((char *)valueStart);
			printf("Set ID:(%d) to MR[%s]\n", iID, reg_idx);
			setIdTf(valueStart, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			get_token(objThreadCntrolBlock);
			printf("Set COMMENT:(%s) to MR[%s]\n", 
				objThreadCntrolBlock->token, reg_idx);
			setCommentTf(valueStart, iRegIdx);
		}
	}
	else if(!strcmp(reg_name, TXT_PL))
	{
		if(strlen(reg_member) == 0)
		{
			setPl(valueStart, iRegIdx);
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
				setPosePl(&pose, iRegIdx);
			}
			else if (valueStart->getType() == TYPE_POSE)
			{
				pose = valueStart->getPoseValue();
				printf("Set POSE:(%f, %f, %f, %f, %f, %f) to PR[%s]\n", 
					pose.position.x, pose.position.y, pose.position.z, 
					pose.orientation.a, pose.orientation.b, pose.orientation.c,
					reg_idx);
				setPosePl(&pose, iRegIdx);
			}
		}
		else if (!strcmp(reg_member, TXT_PL_PALLET))
		{
			setPalletPl(valueStart, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_PL_FLAG))
		{
			int iID = (int)atof((char *)valueStart);
			printf("Set ID:(%d) to MR[%s]\n", iID, reg_idx);
			setFlagPl(valueStart, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_ID))
		{
			int iID = (int)atof((char *)valueStart);
			printf("Set ID:(%d) to MR[%s]\n", iID, reg_idx);
			setIdPl(valueStart, iRegIdx);
		}
		else if (!strcmp(reg_member, TXT_REG_COMMENT))
		{
			get_token(objThreadCntrolBlock);
			printf("Set COMMENT:(%s) to MR[%s]\n", 
				objThreadCntrolBlock->token, reg_idx);
			setCommentPl(valueStart, iRegIdx);
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
		 getPr(reg.value, reg.index);
		 break;
	 case POSE_REG_POSE:
		 getPosePr(reg.value, reg.index);
		 break;
	 case POSE_REG_JOINT:
		 getJointPr(reg.value, reg.index);
		 break;
	 case POSE_REG_TYPE:
		 getTypePr(reg.value, reg.index);
		 break;
	 case POSE_REG_ID:
		 getIdPr(reg.value, reg.index);
		 break;
	 case POSE_REG_COMMENT:
		 getCommentPr(reg.value, reg.index);
		 break;
	 // string register
	 case STR_REG:
	    getSr(reg.value, reg.index);
	 	break;
	 case STR_REG_VALUE:
	    getValueSr(reg.value, reg.index);
	 	break;
	 case STR_REG_ID:
	    getIdSr(reg.value, reg.index);
	 	break;
	 case STR_REG_COMMENT:
	    getCommentSr(reg.value, reg.index);
	 	break;
	 // number register
	 case NUM_REG:
	    getR(reg.value, reg.index);
	 	break;
	 case NUM_REG_VALUE:
	    getValueR(reg.value, reg.index);
	 	break;
	 case NUM_REG_ID:
	    getIdR(reg.value, reg.index);
	 	break;
	 case NUM_REG_COMMENT:
	    getCommentR(reg.value, reg.index);
	 	break;
	 // Special register for motion instruction
	 case MOT_REG:
	    getMr(reg.value, reg.index);
	 	break;
	 case MOT_REG_VALUE:
	    getValueMr(reg.value, reg.index);
	 	break;
	 case MOT_REG_ID:
	    getIdMr(reg.value, reg.index);
	 	break;
	 case MOT_REG_COMMENT:
	    getCommentMr(reg.value, reg.index);
	 	break;
	 // register of user coordinate offset
	 case UF_REG:
	    getUf(reg.value, reg.index);
	 	break;
	 case UF_REG_COORD:
	    getCoordinateUf(reg.value, reg.index);
	 	break;
	 case UF_REG_ID:
	    getIdUf(reg.value, reg.index);
	 	break;
	 case UF_REG_COMMENT:
	    getCommentUf(reg.value, reg.index);
	 	break;
	 // register of tool coordinate offset
	 case TF_REG:
	    getTf(reg.value, reg.index);
	 	break;
	 case TF_REG_COORD:
	    getCoordinateTf(reg.value, reg.index);
	 	break;
	 case TF_REG_ID:
	    getIdTf(reg.value, reg.index);
	 	break;
	 case TF_REG_COMMENT:
	    getCommentTf(reg.value, reg.index);
	 	break;
	 // stack register
	 case PL_REG:
	    getPl(reg.value, reg.index);
	 	break;
	 case PL_REG_POSE:
	    getPosePl(reg.value, reg.index);
	 	break;
	 case PL_REG_PALLET:
	    getPalletPl(reg.value, reg.index);
	 	break;
	 case PL_REG_FLAG:
	    getFlagPl(reg.value, reg.index);
	 	break;
	 case PL_REG_ID:
	    getIdPl(reg.value, reg.index);
	 	break;
	 case PL_REG_COMMENT:
	    getCommentPl(reg.value, reg.index);
	 	break;
 	 }
	 return 1;
}

int forgesight_mod_reg(RegMap & reg)
{
  	 printf("reg.type = %d.\n", reg.type);
	 switch(reg.type)
	 {
	 // pose register
	 case POSE_REG:
		 setPr(reg.value, reg.index);
		 break;
	 case POSE_REG_POSE:
		 setPosePr(reg.value, reg.index);
		 break;
	 case POSE_REG_JOINT:
		 setJointPr(reg.value, reg.index);
		 break;
	 case POSE_REG_TYPE:
		 setTypePr(reg.value, reg.index);
		 break;
	 case POSE_REG_ID:
		 setIdPr(reg.value, reg.index);
		 break;
	 case POSE_REG_COMMENT:
		 setCommentPr(reg.value, reg.index);
		 break;
	 // string register
	 case STR_REG:
	    setSr(reg.value, reg.index);
	 	break;
	 case STR_REG_VALUE:
	    setValueSr(reg.value, reg.index);
	 	break;
	 case STR_REG_ID:
	    setIdSr(reg.value, reg.index);
	 	break;
	 case STR_REG_COMMENT:
	    setCommentSr(reg.value, reg.index);
	 	break;
	 // number register
	 case NUM_REG:
	    setR(reg.value, reg.index);
	 	break;
	 case NUM_REG_VALUE:
	    setValueR(reg.value, reg.index);
	 	break;
	 case NUM_REG_ID:
	    setIdR(reg.value, reg.index);
	 	break;
	 case NUM_REG_COMMENT:
	    setCommentR(reg.value, reg.index);
	 	break;
	 // Special register for motion instruction
	 case MOT_REG:
	    setMr(reg.value, reg.index);
	 	break;
	 case MOT_REG_VALUE:
	    setValueMr(reg.value, reg.index);
	 	break;
	 case MOT_REG_ID:
	    setIdMr(reg.value, reg.index);
	 	break;
	 case MOT_REG_COMMENT:
	    setCommentMr(reg.value, reg.index);
	 	break;
	 // register of user coordinate offset
	 case UF_REG:
	    setUf(reg.value, reg.index);
	 	break;
	 case UF_REG_COORD:
	    setCoordinateUf(reg.value, reg.index);
	 	break;
	 case UF_REG_ID:
	    setIdUf(reg.value, reg.index);
	 	break;
	 case UF_REG_COMMENT:
	    setCommentUf(reg.value, reg.index);
	 	break;
	 // register of tool coordinate offset
	 case TF_REG:
	    setTf(reg.value, reg.index);
	 	break;
	 case TF_REG_COORD:
	    setCoordinateTf(reg.value, reg.index);
	 	break;
	 case TF_REG_ID:
	    setIdTf(reg.value, reg.index);
	 	break;
	 case TF_REG_COMMENT:
	    setCommentTf(reg.value, reg.index);
	 	break;
	 // stack register
	 case PL_REG:
	    setPl(reg.value, reg.index);
	 	break;
	 case PL_REG_POSE:
	    setPosePl(reg.value, reg.index);
	 	break;
	 case PL_REG_PALLET:
	    setPalletPl(reg.value, reg.index);
	 	break;
	 case PL_REG_FLAG:
	    setFlagPl(reg.value, reg.index);
	 	break;
	 case PL_REG_ID:
	    setIdPl(reg.value, reg.index);
	 	break;
	 case PL_REG_COMMENT:
	    setCommentPl(reg.value, reg.index);
	 	break;
 	 }
	 return 1;
}

