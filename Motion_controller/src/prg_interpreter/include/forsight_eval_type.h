#ifndef FORSIGHT_EVAL_TYPE_H
#define FORSIGHT_EVAL_TYPE_H
#include <stdlib.h>
#include "fst_datatype.h" 

#ifdef WIN32
#include "tp_reg_manager_interface.h"
#include "reg-shmi/forsight_regs_shmi.h"
#else
#include "reg_manager/reg_manager_interface.h"
using namespace fst_reg;

typedef struct
{
	int row;
	int column;
	int layer;
}pl_t;

#endif


typedef enum _EvalValueType
{
	TYPE_NONE   = 0x00,
	TYPE_INT    = 0x01,
	TYPE_FLOAT  = 0x02,
	TYPE_STRING = 0x04,
	TYPE_POSE   = 0x08,
	TYPE_JOINT  = 0x10,
	TYPE_PL     = 0x20,
	
	TYPE_PR     = 0x40,
	TYPE_SR     = 0x80,
	TYPE_R      = 0x100,
	TYPE_MR     = 0x200,
}EvalValueType;

typedef struct _AdditionalE {
    double e1;
    double e2;
    double e3;
} AdditionalE;

#define STR_VALUE_SIZE 254

class eval_value {
public:
	eval_value() 
	{
		memset((void *)&poseFake, 0x00, sizeof(poseFake));
		memset((void *)&jointFake, 0x00, sizeof(jointFake));
		memset((void *)&prRegDataFake, 0x00, sizeof(prRegDataFake));
//		memset((void *)&srRegDataFake, 0x00, sizeof(srRegDataFake));
		memset((void *)&rRegDataFake,  0x00, sizeof(rRegDataFake));
//		memset((void *)&mrRegDataFake, 0x00, sizeof(mrRegDataFake));
		
		resetNoneValue() ;
	}
	int getType(){
		return evalType ;
	}

private:
	// Not support TYPE_INT
	void setIntValue(int iVal){
		evalType |= TYPE_INT ;
		iValue = iVal ;
	}
	void increaseIntValue(){
		iValue++ ;
	}
	int getIntValue(){
		int iType = evalType & TYPE_INT;
		if(iType != 0)
			return iValue ;
		else {
			noticeErrorType(TYPE_INT) ;
			return -1;
		}
	}

public:
	void resetNoneValue(){
		evalType = TYPE_NONE ;
		fValue = -1 ;
		
		tfIndex = -1 ;
		ufIndex = -1 ;
	}
	// TYPE_FLOAT
	void setFloatValue(float fVal){
		evalType |= TYPE_FLOAT ;
		fValue = fVal ;
	}
	void increaseFloatValue(){
		fValue++ ;
	}
	float getFloatValue(){
		int iType = evalType & TYPE_FLOAT;
		if(iType != 0)
			return fValue ;
		else {
			noticeErrorType(TYPE_FLOAT) ;
			return -1;
		}
	}
	
	// TYPE_STRING
	void setStringValue(char * cVal){
		evalType |= TYPE_STRING ;
		if(strlen(cVal) <= STR_VALUE_SIZE)
		{
			strcpy(cContent, cVal);
		}
	}

	int getStringValue(char * cVal){
		int iType = evalType & TYPE_STRING;
		if(iType != 0) {
			strcpy(cVal, cContent);
			return 1 ;
		}
		else {
			noticeErrorType(TYPE_STRING) ;
			return -1;
		}
	}
	
	// TYPE_POSE
	void setPoseValue(PoseEuler * poseVal){
		evalType |= TYPE_POSE ;
		pose     = * poseVal ;
	}
	
	PoseEuler getPoseValue(){
		int iType = evalType & TYPE_POSE ;
		if(iType != 0) {
			return pose ;
		}
		else {
			noticeErrorType(TYPE_POSE) ;
			return poseFake;
		}
	}

	// TYPE_JOINT
	void setJointValue(Joint * jointVal){
		evalType |= TYPE_JOINT ;
		joint     = * jointVal ;
	}
	
	Joint getJointValue(){
		int iType = evalType & TYPE_JOINT;
		if(iType != 0) {
			return joint ;
		}
		else {
			noticeErrorType(TYPE_JOINT) ;
			return jointFake;
		}
	}
	
	void setUFIndex(int ufParam){
		if((evalType == TYPE_JOINT) 
			|| (evalType == TYPE_POSE)) {
			ufIndex = ufParam ;
		}
		else {
			noticeErrorType(TYPE_JOINT | TYPE_POSE) ;
			return ;
		}
	}
	
	int getUFIndex(){
		if((evalType == TYPE_JOINT) 
			|| (evalType == TYPE_POSE)) {
			return ufIndex ;
		}
		else {
			noticeErrorType(TYPE_JOINT | TYPE_POSE) ;
			return -1;
		}
	}
	
	void setTFIndex(int tfParam){
		if((evalType == TYPE_JOINT) 
			|| (evalType == TYPE_POSE)) {
			tfIndex = tfParam ;
		}
		else {
			noticeErrorType(TYPE_JOINT | TYPE_POSE) ;
			return ;
		}
	}
	
	int getTFIndex(){
		if((evalType == TYPE_JOINT) 
			|| (evalType == TYPE_POSE)) {
			return tfIndex ;
		}
		else {
			noticeErrorType(TYPE_JOINT | TYPE_POSE) ;
			return -1;
		}
	}
	
	void updateAdditionalE(AdditionalE additionParam){
		if((evalType == TYPE_JOINT) 
			|| (evalType == TYPE_POSE)) {
			addition = additionParam ;
		}
		else {
			noticeErrorType(TYPE_JOINT | TYPE_POSE) ;
			return ;
		}
	}

	// TYPE_PL
	void setPLValue(pl_t * jointVal){
		evalType = TYPE_PL ;
		pallet     = * jointVal ;
	}
	
	pl_t getPLValue(){
		if(evalType == TYPE_PL) {
			return pallet ;
		}
		else {
			noticeErrorType(TYPE_PL) ;
			return palletFake;
		}
	}

	// TYPE_PR
	void setPrRegDataValue(PrRegData * prRegDataVal){
		evalType  |= TYPE_PR ;
		reg_pr     = * prRegDataVal ;
	}
	
	PrRegData getPrRegDataValue(){
		int iType = evalType & TYPE_PR ;
		if(iType != 0) {
			return reg_pr ;
		}
		else {
			noticeErrorType(TYPE_PR) ;
			return prRegDataFake;
		}
	}
	// TYPE_SR
	void setSrRegDataValue(SrRegData * srRegDataVal){
		evalType  |= TYPE_SR ;
		reg_sr     = * srRegDataVal ;
	}
	
	SrRegData getSrRegDataValue(){
		int iType = evalType & TYPE_SR ;
		if(iType != 0) {
			return reg_sr ;
		}
		else {
			noticeErrorType(TYPE_SR) ;
			return srRegDataFake;
		}
	}
	// TYPE_R
	void setRRegDataValue(RRegData * rRegDataVal){
		evalType |= TYPE_R ;
		reg_r     = * rRegDataVal ;
		evalType |= TYPE_FLOAT ;
		fValue    = rRegDataVal->value ;
	}
	
	RRegData getRRegDataValue(){
		int iType = evalType & TYPE_R ;
		if(iType != 0) {
			return reg_r ;
		}
		else {
			noticeErrorType(TYPE_R) ;
			return rRegDataFake;
		}
	}
	// TYPE_MR
	void setMrRegDataValue(MrRegData * mrRegDataVal){
		evalType  |= TYPE_MR ;
		reg_mr     = * mrRegDataVal ;
		evalType |= TYPE_FLOAT ;
		fValue    = (float)mrRegDataVal->value ;
	}
	
	MrRegData getMrRegDataValue(){
		int iType = evalType & TYPE_MR ;
		if(iType != 0) {
			return reg_mr ;
		}
		else {
			noticeErrorType(TYPE_MR) ;
			return mrRegDataFake;
		}
	}
	
public:
	void calcAdd(eval_value * operand)
	{
		if(evalType == TYPE_INT){
			iValue = iValue + operand->getIntValue();
			return ;
		}else if(evalType == TYPE_FLOAT){
			fValue = fValue + operand->getFloatValue();
			return ;
		}else if(evalType == (int)(TYPE_R | TYPE_FLOAT)){
		    if(operand->getType() == TYPE_FLOAT)
		    {
				reg_r.value = reg_r.value + operand->getFloatValue();
				fValue = fValue + operand->getFloatValue();
				
				printf("RRegData: id = %d, comment = %s\n", reg_r.id, reg_r.comment);
	        	printf("reg_r.value = %f and operand = %f\n", reg_r.value, operand->getFloatValue());
		    }
			else if(operand->getType() == (int)(TYPE_R | TYPE_FLOAT))
		    {
				reg_r.value = reg_r.value + operand->getRRegDataValue().value;
				fValue = fValue + operand->getRRegDataValue().value;
		    }
			else if(operand->getType() == (int)(TYPE_MR | TYPE_FLOAT))
		    {
				reg_r.value = reg_r.value + operand->getMrRegDataValue().value;
				fValue = fValue + operand->getMrRegDataValue().value;
		    }
			return ;
		}else if(evalType == (int)(TYPE_MR | TYPE_FLOAT)){
		    if(operand->getType() == TYPE_FLOAT)
		    {
				reg_mr.value = reg_mr.value + operand->getFloatValue();
				fValue = fValue + operand->getFloatValue();
				
				printf("RRegData: id = %d, comment = %s\n", reg_r.id, reg_r.comment);
	        	printf("reg_r.value = %f and operand = %f\n", reg_r.value, operand->getFloatValue());
		    }
			else if(operand->getType() == (int)(TYPE_R | TYPE_FLOAT))
		    {
				reg_mr.value = reg_mr.value + operand->getRRegDataValue().value;
				fValue = fValue + operand->getRRegDataValue().value;
		    }
			else if(operand->getType() == (int)(TYPE_MR | TYPE_FLOAT))
		    {
				reg_mr.value = reg_mr.value + operand->getMrRegDataValue().value;
				fValue = fValue + operand->getMrRegDataValue().value;
		    }
			return ;
		}
		else {
			noticeErrorType(operand->getType()) ;
			return ;
		}
	}
	
	void calcSubtract(eval_value * operand)
	{
		if(evalType == TYPE_INT){
			iValue = iValue - operand->getIntValue();
			return ;
		}else if(evalType == TYPE_FLOAT){
			fValue = fValue - operand->getFloatValue();
			return ;
		}else if(evalType == (int)(TYPE_R | TYPE_FLOAT)){
		    if(operand->getType() == TYPE_FLOAT)
		    {
				reg_r.value = reg_r.value - operand->getFloatValue();
				fValue = fValue - operand->getFloatValue();
				
				printf("RRegData: id = %d, comment = %s\n", reg_r.id, reg_r.comment);
	        	printf("reg_r.value = %f and operand = %f\n", reg_r.value, operand->getFloatValue());
		    }
			else if(operand->getType() == (int)(TYPE_R | TYPE_FLOAT))
		    {
				reg_r.value = reg_r.value - operand->getRRegDataValue().value;
				fValue = fValue - operand->getRRegDataValue().value;
		    }
			else if(operand->getType() == (int)(TYPE_MR | TYPE_FLOAT))
		    {
				reg_r.value = reg_r.value - operand->getMrRegDataValue().value;
				fValue = fValue - operand->getMrRegDataValue().value;
		    }
			return ;
		}else if(evalType == (int)(TYPE_MR | TYPE_FLOAT)){
		    if(operand->getType() == TYPE_FLOAT)
		    {
	        	printf("reg_mr.value = %d and operand = %f\n", reg_mr.value, operand->getFloatValue());
				reg_mr.value = reg_mr.value - (int)operand->getFloatValue();
				fValue = fValue - operand->getFloatValue();
				
				printf("MRRegData: id = %d, comment = %s\n", reg_mr.id, reg_mr.comment);
	        	printf("reg_mr.value = %d and operand = %f\n", reg_mr.value, operand->getFloatValue());
		    }
			else if(operand->getType() == (int)(TYPE_R | TYPE_FLOAT))
		    {
				reg_mr.value = reg_mr.value - operand->getRRegDataValue().value;
				fValue = fValue - operand->getRRegDataValue().value;
		    }
			else if(operand->getType() == (int)(TYPE_MR | TYPE_FLOAT))
		    {
				printf("MrRegData: id = %d, comment = %s\n", reg_mr.id, reg_mr.comment);
	        	printf("reg_mr.value = %d and operand = %d\n", reg_mr.value, operand->getMrRegDataValue().value);
				
				reg_mr.value = reg_mr.value - operand->getMrRegDataValue().value;
				fValue = fValue - operand->getMrRegDataValue().value;
				
				printf("MrRegData: id = %d, comment = %s\n", reg_mr.id, reg_mr.comment);
	        	printf("reg_mr.value = %d and operand = %d\n", reg_mr.value, operand->getMrRegDataValue().value);
		    }
			return ;
		}
		else {
			noticeErrorType(operand->getType()) ;
			return ;
		}
	}
	
	void calcMultiply(eval_value * operand)
	{
		if(evalType == TYPE_INT){
			iValue = iValue * operand->getIntValue();
			return ;
		}else if(evalType == TYPE_FLOAT){
			fValue = fValue * operand->getFloatValue();
			return ;
		}else if(evalType == (int)(TYPE_R | TYPE_FLOAT)){
		    if(operand->getType() == TYPE_FLOAT)
		    {
				reg_r.value = reg_r.value * operand->getFloatValue();
				fValue = fValue * operand->getFloatValue();
				
				printf("RRegData: id = %d, comment = %s\n", reg_r.id, reg_r.comment);
	        	printf("reg_r.value = %f and operand = %f\n", reg_r.value, operand->getFloatValue());
		    }
			else if(operand->getType() == (int)(TYPE_R | TYPE_FLOAT))
		    {
				reg_r.value = reg_r.value * operand->getRRegDataValue().value;
				fValue  = reg_r.value;
		    }
			else if(operand->getType() == (int)(TYPE_MR | TYPE_FLOAT))
		    {
				reg_r.value = reg_r.value * operand->getMrRegDataValue().value;
				fValue = fValue * operand->getMrRegDataValue().value;
		    }
			return ;
		}else if(evalType == (int)(TYPE_MR | TYPE_FLOAT)){
		    if(operand->getType() == TYPE_FLOAT)
		    {
				reg_mr.value = reg_mr.value * operand->getFloatValue();
				fValue = fValue * operand->getFloatValue();
				
				printf("RRegData: id = %d, comment = %s\n", reg_r.id, reg_r.comment);
	        	printf("reg_r.value = %f and operand = %f\n", reg_r.value, operand->getFloatValue());
		    }
			else if(operand->getType() == (int)(TYPE_R | TYPE_FLOAT))
		    {
				reg_mr.value = reg_mr.value * operand->getRRegDataValue().value;
				fValue = fValue * operand->getRRegDataValue().value;
		    }
			else if(operand->getType() == (int)(TYPE_MR | TYPE_FLOAT))
		    {
				reg_mr.value = reg_mr.value * operand->getMrRegDataValue().value;
				fValue = fValue * operand->getMrRegDataValue().value;
		    }
			return ;
		}
		else {
			noticeErrorType(operand->getType()) ;
			return ;
		}
	}

	void calcDivide(eval_value * operand)
	{
		if(evalType == TYPE_INT){
			iValue = iValue / operand->getIntValue();
			return ;
		}else if(evalType == TYPE_FLOAT){
			fValue = fValue / operand->getFloatValue();
			return ;
		}else if(evalType == (int)(TYPE_R | TYPE_FLOAT)){
		    if(operand->getType() == TYPE_FLOAT)
		    {
				reg_r.value = reg_r.value / operand->getFloatValue();
				fValue = fValue / operand->getFloatValue();
				
				printf("RRegData: id = %d, comment = %s\n", reg_r.id, reg_r.comment);
	        	printf("reg_r.value = %f and operand = %f\n", reg_r.value, operand->getFloatValue());
		    }
			else if(operand->getType() == (int)(TYPE_R | TYPE_FLOAT))
		    {
				reg_r.value = reg_r.value / operand->getRRegDataValue().value;
				fValue  = reg_r.value;
		    }
			else if(operand->getType() == (int)(TYPE_MR | TYPE_FLOAT))
		    {
				reg_r.value = reg_r.value / operand->getMrRegDataValue().value;
				fValue = fValue / operand->getMrRegDataValue().value;
		    }
			return ;
		}else if(evalType == (int)(TYPE_MR | TYPE_FLOAT)){
		    if(operand->getType() == TYPE_FLOAT)
		    {
				reg_mr.value = reg_mr.value / operand->getFloatValue();
				fValue = fValue / operand->getFloatValue();
				
				printf("RRegData: id = %d, comment = %s\n", reg_r.id, reg_r.comment);
	        	printf("reg_r.value = %f and operand = %f\n", reg_r.value, operand->getFloatValue());
		    }
			else if(operand->getType() == (int)(TYPE_R | TYPE_FLOAT))
		    {
				reg_mr.value = reg_mr.value / operand->getRRegDataValue().value;
				fValue = fValue / operand->getRRegDataValue().value;
		    }
			else if(operand->getType() == (int)(TYPE_MR | TYPE_FLOAT))
		    {
				reg_mr.value = reg_mr.value / operand->getMrRegDataValue().value;
				fValue = fValue / operand->getMrRegDataValue().value;
		    }
			return ;
		}
		else {
			noticeErrorType(operand->getType()) ;
			return ;
		}
	}
 
	void calcMod(eval_value * operand)
	{
		int iTmp = 0 ;
		if(evalType == TYPE_INT){
			iTmp = iValue / operand->getIntValue();
			iValue = iValue - (iTmp * operand->getIntValue());
		}else if(evalType == TYPE_FLOAT){
			iTmp = fValue / (int)operand->getIntValue();
			fValue = fValue - (iTmp * (int)operand->getFloatValue());
		}else if(evalType == (int)(TYPE_R | TYPE_FLOAT)){
		    if(operand->getType() == TYPE_FLOAT)
		    {
				iTmp = reg_r.value / (int)operand->getFloatValue();
				reg_r.value = reg_r.value - (iTmp * (int)operand->getFloatValue());
				fValue  = reg_r.value;
				
				printf("RRegData: id = %d, comment = %s\n", reg_r.id, reg_r.comment);
	        	printf("reg_r.value = %f and operand = %f\n", reg_r.value, operand->getFloatValue());
		    }
			else if(operand->getType() == (int)(TYPE_R | TYPE_FLOAT))
		    {
				iTmp = reg_r.value / (int)operand->getRRegDataValue().value;
				reg_r.value = reg_r.value - (iTmp * (int)operand->getRRegDataValue().value);
				fValue  = reg_r.value;
		    }
			else if(operand->getType() == (int)(TYPE_MR | TYPE_FLOAT))
		    {
				iTmp = reg_r.value / (int)operand->getMrRegDataValue().value;
				reg_r.value = reg_r.value - (iTmp * (int)operand->getMrRegDataValue().value);
				fValue  = reg_r.value;
		    }
			return ;
		}else if(evalType == (int)(TYPE_MR | TYPE_FLOAT)){
		    if(operand->getType() == TYPE_FLOAT)
		    {
				iTmp = reg_mr.value / (int)operand->getFloatValue();
				reg_mr.value = reg_mr.value - (iTmp * (int)operand->getFloatValue());
				fValue  = reg_mr.value;
				
				printf("RRegData: id = %d, comment = %s\n", reg_r.id, reg_r.comment);
	        	printf("reg_r.value = %f and operand = %f\n", reg_r.value, operand->getFloatValue());
		    }
			else if(operand->getType() == (int)(TYPE_R | TYPE_FLOAT))
		    {
				iTmp = reg_mr.value / (int)operand->getRRegDataValue().value;
				reg_mr.value = reg_mr.value - (iTmp * (int)operand->getRRegDataValue().value);
				fValue  = reg_mr.value;
		    }
			else if(operand->getType() == (int)(TYPE_MR | TYPE_FLOAT))
		    {
				iTmp = reg_mr.value / (int)operand->getMrRegDataValue().value;
				reg_mr.value = reg_mr.value - (iTmp * (int)operand->getMrRegDataValue().value);
				fValue  = reg_mr.value;
		    }
			return ;
		}
		else {
			noticeErrorType(operand->getType()) ;
			return ;
		}
	}
 
	void calcPower(eval_value * operand)
	{
		int t = 0 ;
		int ex = (int)operand->getFloatValue();
		if(evalType == TYPE_INT){
			if(ex == 0)
				iValue = 1 ;
			else {
				for(t=ex-1; t>0; --t) 
					iValue = (iValue) * ex;
			}
		}else if(evalType == TYPE_FLOAT){
			if(ex == 0)
				fValue = 1 ;
			else {
				for(t=ex-1; t>0; --t) 
					fValue = (fValue) * ex;
			}
		}else if(evalType == (int)(TYPE_R | TYPE_FLOAT)){
			if(ex == 0)
				reg_r.value = 1 ;
			else {
				for(t=ex-1; t>0; --t) 
					reg_r.value = (reg_r.value) * ex;
			}
			fValue  = reg_r.value;
		}else if(evalType == (int)(TYPE_MR | TYPE_FLOAT)){
			if(ex == 0)
				reg_mr.value = 1 ;
			else {
				for(t=ex-1; t>0; --t) 
					reg_mr.value = (reg_mr.value) * ex;
			}
			fValue  = reg_mr.value;
		}
		else {
			noticeErrorType(operand->getType()) ;
			return ;
		}
	}
	
	void calcUnary()
	{
		if(evalType == TYPE_INT){
			iValue = -(iValue);
			return ;
		}else if(evalType == TYPE_FLOAT){
			fValue = -(fValue);
			return ;
		}else if(evalType == (int)(TYPE_R | TYPE_FLOAT)){
			reg_r.value = -(reg_r.value);
			fValue  = reg_r.value;
			return ;
		}else if(evalType == (int)(TYPE_MR | TYPE_FLOAT)){
			reg_mr.value = -(reg_mr.value);
			fValue  = reg_mr.value;
			return ;
		}
		else {
			noticeErrorType(TYPE_FLOAT | TYPE_INT) ;
			return ;
		}
	}
private:
	void noticeErrorType(int type){
		printf("TypeError: call %04X in the %04X\n", type, evalType);
	}

private:
	int evalType ;
	// use the union or malloc in the future.
	// union {
		// Basic type ;
		int iValue ;
		float fValue ;
		char cContent[STR_VALUE_SIZE];
		
		// All member of register
		PoseEuler pose;
		PoseEuler poseFake;
		Joint     joint;
		Joint     jointFake;
//		Coordinate c; 
		pl_t pallet;
		pl_t     palletFake;
		// additionalE
		AdditionalE addition ;
		// tf Index & uf Index
		int tfIndex ;
		int ufIndex ;
#if 0
		// All of register
		pr_shmi_t reg_pr ;
		sr_shmi_t reg_sr ;
		r_shmi_t  reg_r  ;
		mr_shmi_t reg_mr ;
		uf_shmi_t reg_ur ;
		tf_shmi_t reg_tf ;
#else
		// All of register
        PrRegData reg_pr ;
		PrRegData     prRegDataFake;
		
        SrRegData reg_sr ;
		SrRegData     srRegDataFake;
		
        RRegData  reg_r ;
		RRegData     rRegDataFake;
		
        MrRegData reg_mr ;
		MrRegData     mrRegDataFake;
#endif
	// };
};

#endif

