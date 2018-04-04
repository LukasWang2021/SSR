#ifndef FORSIGHT_EVAL_TYPE_H
#define FORSIGHT_EVAL_TYPE_H
#include <stdlib.h>
#include "reg-shmi/forsight_regs_shmi.h"

typedef enum _EvalValueType
{
	TYPE_INT    = 0x01,
	TYPE_FLOAT  = 0x02,
	TYPE_STRING = 0x04,
	TYPE_POSE   = 0x08,
	TYPE_JOINT  = 0x10,
	TYPE_PL     = 0x20,
}EvalValueType;

class eval_value {
public:
	eval_value() 
	{
		memset((void *)&poseFake, 0x00, sizeof(poseFake));
		memset((void *)&jointFake, 0x00, sizeof(jointFake));
	}
	EvalValueType getType(){
		return evalType ;
	}

private:
	// Not support TYPE_INT
	void setIntValue(int iVal){
		evalType = TYPE_INT ;
		iValue = iVal ;
	}
	void increaseIntValue(){
		iValue++ ;
	}
	int getIntValue(){
		if(evalType == TYPE_INT)
			return iValue ;
		else {
			noticeErrorType(TYPE_FLOAT) ;
			return -1;
		}
	}
public:
	// TYPE_FLOAT
	void setFloatValue(float fVal){
		evalType = TYPE_FLOAT ;
		fValue = fVal ;
	}
	void increaseFloatValue(){
		fValue++ ;
	}
	float getFloatValue(){
		if(evalType == TYPE_FLOAT)
			return fValue ;
		else {
			noticeErrorType(TYPE_FLOAT) ;
			return -1;
		}
	}
	
	// TYPE_STRING
	void setStringValue(char * cVal){
		evalType = TYPE_STRING ;
		if(strlen(cVal) <= SR_VALUE_SIZE)
		{
			strcpy(cContent, cVal);
		}
	}

	int getStringValue(char * cVal){
		if(evalType == TYPE_STRING) {
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
		evalType = TYPE_POSE ;
		pose     = * poseVal ;
	}
	
	PoseEuler getPoseValue(){
		if(evalType == TYPE_POSE) {
			return pose ;
		}
		else {
			noticeErrorType(TYPE_STRING) ;
			return poseFake;
		}
	}

	// TYPE_JOINT
	void setJointValue(Joint * jointVal){
		evalType = TYPE_JOINT ;
		joint     = * jointVal ;
	}
	
	Joint getJointValue(){
		if(evalType == TYPE_JOINT) {
			return joint ;
		}
		else {
			noticeErrorType(TYPE_JOINT) ;
			return jointFake;
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
public:
	void calcAdd(eval_value * operand)
	{
		if(evalType == TYPE_INT){
			iValue = iValue + operand->getIntValue();
			return ;
		}else if(evalType == TYPE_FLOAT){
			fValue = fValue + operand->getFloatValue();
			return ;
		}
		else {
			noticeErrorType(TYPE_FLOAT | TYPE_INT) ;
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
		}
		else {
			noticeErrorType(TYPE_FLOAT | TYPE_INT) ;
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
		}
		else {
			noticeErrorType(TYPE_FLOAT | TYPE_INT) ;
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
		}
		else {
			noticeErrorType(TYPE_FLOAT | TYPE_INT) ;
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
			iTmp = fValue / operand->getIntValue();
			fValue = fValue - (iTmp * operand->getIntValue());
		}
		else {
			noticeErrorType(TYPE_FLOAT | TYPE_INT) ;
			return ;
		}
	}
 
	void calcPower(eval_value * operand)
	{
		int t = 0 ;
		int ex = operand->getIntValue();
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
		}
		else {
			noticeErrorType(TYPE_FLOAT | TYPE_INT) ;
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
		}
		else {
			noticeErrorType(TYPE_FLOAT | TYPE_INT) ;
			return ;
		}
	}
private:
	void noticeErrorType(int type){
		printf("TypeError: call %d in the %d\n", type, evalType);
	}

private:
	EvalValueType evalType ;
	// use the union or malloc in the future.
	union {
		// Basic type ;
		int iValue ;
		float fValue ;
		char cContent[SR_VALUE_SIZE];
		
		// All member of register
		PoseEuler pose;
		PoseEuler poseFake;
		Joint     joint;
		Joint     jointFake;
		Coordinate c; 
		pl_t pallet;
		pl_t     palletFake;
		// All of register
		pr_shmi_t reg_pr ;
		sr_shmi_t reg_sr ;
		r_shmi_t  reg_r  ;
		mr_shmi_t reg_mr ;
		uf_shmi_t reg_ur ;
		tf_shmi_t reg_tf ;
	};
};

#endif

