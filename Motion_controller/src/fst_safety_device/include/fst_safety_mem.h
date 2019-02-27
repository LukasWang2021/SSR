/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       fst_safety_mem.h
Author:     Feng.Wu 
Create:     20-Sep-2017
Modify:     27-Sep-2018
Summary:    dealing with safety board

**********************************************/
#ifndef FST_SAFETY_MEM_H
#define FST_SAFETY_MEM_H

#include <error_code.h>

#define SAFETY_INPUT_FIRSTFRAME     0X0101 // for getSafety(..)  high 4 bits define INPUT or OUTPUT;
#define SAFETY_INPUT_SECONDFRAME    0X0102 // for getSafety(..)  low 4 bits define first four bytes or second four bytes.
#define SAFETY_OUTPUT_FIRSTFRAME    0X0201 // for setSafety(..)

//#define ERR_SAFETY_FILE_OPEN (unsigned long long int)0x00010002008E0001   /*can't open file when initializing safety board*/
//#define ERR_SAFETY_FILE_MAP (unsigned long long int)0x00010002008E0002   /*Mapping is failed.*/
//#define ERR_SAFETY_FPGA_MCU_NOT_CONNECT (unsigned long long int)0x00010002008E0015   /*No communication between FPGA and MCU.*/ 
//#define ERR_SAFETY_FPGA_CORE0_NOT_CONNECT (unsigned long long int)0x00010002008E0016   /*FPGA can't read pulse from core0.*/
//#define ERR_SAFETY_FPGA_CORE1_NOT_CONNECT (unsigned long long int)0x00010007008E0017   /*FPGA can't read pulse from core1.*/ 
//#define ERR_SAFETY_PTHREAD_INIT (unsigned long long int)0x00000001008E001F   /*Mutex initialization is failed.*/
//#define ERR_SAFETY_PTHREAD_LOCK (unsigned long long int)0x00000001008E0020   /*Mutex lock is failed*/
//#define ERR_SAFETY_PTHREAD_UNLOCK (unsigned long long int)0x00000001008E0021   /*Mutex unlock is failed*/
//#define ERR_SAFETY_FRAME (unsigned long long int)0x00000001008E0029   /*The frame is out of range.*/


#ifdef __cplusplus
extern "C" { //declear c-lib to the compiler.
#endif

//  -----------------------------------------------------------------------
//  Function:		openSafety
//  Description:	open the interface of the safety, and initialize the
//					interface
//  In:
//  Out:
//  Return:			0: no error
//					ERR_SAFETY_FILE_OPEN: the safety cann't be opened.
//					ERR_SAFETY_FILE_MAP:  Failed to map the registers of safety.
//		     		ERR_SAFETY_PTHREAD_INIT: mutex init failed.
//  -----------------------------------------------------------------------
unsigned long long int openSafety(void);
void closeSafety(void);

//  -----------------------------------------------------------------------
//  Function:		getSafety
//  Description:	get the value from safety board.
//  In:				frame
//					determin the data of which frame of 4 frames
//  Out:			ERR_SAFETY_FRAME: the frame is wrong.
//  Return:			if the error occur, err will save ERR_SAFETY_FRAME which
//					indicates that the frame is wrong.
//  -----------------------------------------------------------------------
unsigned long long int getSafety(int *data, int frame);

//  -----------------------------------------------------------------------
//  Function:		setSafety
//  Description:	set a value to the safety
//  In:				data
//					frame
//  Out:
//  Return:			the error
//					0: no error
//					ERR_SAFETY_FRAME: the frame is wrong.
//  -----------------------------------------------------------------------
unsigned long long int setSafety(int data, int frame);

//  -----------------------------------------------------------------------
//  Function:		autorunSafetyData
//  Description:	this function will run one time per 50ms, and get data
//					from and set value to the safety.
//  In: 
//  Out:
//  Return:			0: no error.
//                  ERR_SAFETY_PTHREAD_LOCK: mutex lock failed.
//                  ERR_SAFETY_PTHREAD_UNLOCK: mutex unlock failed.
//					ERR_SAFETY_FPGA_MCU_NOT_CONNECT: No communication between FPGA and MCU.
//					ERR_SAFETY_FPGA_CORE0_NOT_CONNECT:	FPGA can't read pulse from core0.
//                  ERR_SAFETY_FPGA_CORE1_NOT_CONNECT:  FPGA can't read pulse from core1.
//  -----------------------------------------------------------------------
unsigned long long int autorunSafetyData(void);

void getSafetyBoardVersionFromMem(int *version);

int fake_init(void);
#ifdef __cplusplus
}
#endif

#endif