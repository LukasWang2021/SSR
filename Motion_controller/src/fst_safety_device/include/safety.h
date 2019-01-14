/**********************************************
Copyright © 2016 Foresight-Robotics Ltd. All rights reserved.
File:       safety.h
Author:     Shuguo.Zhang Feng.Wu 
Create:     20-Sep-2017
Modify:     22-Sep-2017
Summary:    dealing with safety board
**********************************************/

#define SAFETY_MAX_FRAME	8
#define R_SAFETY_MAX_FRAME	SAFETY_MAX_FRAME
#define W_SAFETY_MAX_FRAME	SAFETY_MAX_FRAME

#define FIRST_FRAME		0
#define SECOND_FRAME	1
#define THIRD_FRAME		2
#define FOURTH_FRAME	3
#define FIFTH_FRAME		4
#define SIXTH_FRAME		5
#define	SEVENTH_FRAME	6
#define EIGHTH_FRAME	7

#define R_HB_FRAME		THIRD_FRAME    // modified by Feng.Wu
#define	W_HB_FRAME		THIRD_FRAME    // modified by Feng.Wu
#define NO_HB_TIMES		3	/* NO_HB_TIMES means the times that there is no heartbeat continually. Each time expends 50ms. If it is matched, the error, ERR_SAFETY_NOT_CONNECT is reported. */

#define SAFETY_CMD_RECV 0X5F    // add by Feng.Wu
#define SAFETY_CMD_SEND 0XF5    // add by Feng.Wu
#define SAFETY_CORE_SEND    0X0100

#define SAFETY_INPUT_FIRSTFRAME     0x0101
#define SAFETY_INPUT_SECONDFRAME    0x0102

#define SAFETY_OUTPUT_FIRSTFRAME    0x0201
#define SAFETY_OUTPUT_SECONDFRAME	0x0202

#define ERR_SAFETY_FILE_OPEN (unsigned long long int)0x0011000200AF0001   /*can't open file when initializing safety board*/
#define ERR_SAFETY_FILE_MAP (unsigned long long int)0x0011000200AF0002   /*Mapping is failed.*/
#define ERR_SAFETY_RECV_CMD (unsigned long long int)0x0001000200AF0003   /*Command is not received from the safety board*/
#define ERR_SAFETY_NOT_CONNECT (unsigned long long int)0x0001000200AF0004   /*Safety board is not connected with, and there is not heartbeat from the safety board.*/
#define ERR_SAFETY_RECV_DIFF (unsigned long long int)0x0001000200AF0005   /*The data from two safety MCUs are different.*/
#define ERR_SAFETY_PTHREAD_INIT (unsigned long long int)0x0011000200AF0006   /*Mutex initialization is failed.*/
#define ERR_SAFETY_PTHREAD_LOCK (unsigned long long int)0x0001000200AF0007   /*Mutex lock is failed*/
#define ERR_SAFETY_PTHREAD_UNLOCK (unsigned long long int)0x0001000200AF0008   /*Mutex unlock is failed*/
#define ERR_SAFETY_FRAME (unsigned long long int)0x0001000200AF0009   /*The frame is out of range.*/


#ifdef __cplusplus
extern "C" {
#endif
//  -----------------------------------------------------------------------
//  Function:		getSafety
//  Description:	get the value from safety board.
//  In:				frame
//					determin the data of which frame of 4 frames
//  Out:			err
//					if the error occur, err will save ERR_SAFETY_FRAME which
//					indicates that the frame is wrong.
//					if there is no error, err will present 0.
//  Return:			the data gotten from the safety. However, when 0 returned,
//					please check err. if there is no error, 0 returned is the
//					valid data from the safety.
//  -----------------------------------------------------------------------
int getSafety(int frame, unsigned long long int *err);
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
//  Function:		openSafety
//  Description:	open the interface of the safety, and initialize the 
//					interface
//  In: 
//  Out:
//  Return:			0: no error
//					ERR_SAFETY_FILE_OPEN: the safety cann't be opened.
//					ERR_SAFETY_FILE_MAP:  Failed to map the registers of safety.
//  -----------------------------------------------------------------------
unsigned long long int openSafety(void);
void closeSafety(void);

//  -----------------------------------------------------------------------
//  Function:		autorunSafetyData
//  Description:	this function will run one time per 50ms, and get data
//					from and set value to the safety.
//  In: 
//  Out:
//  Return:			0: no error
//					ERR_SAFETY_NOT_CONNECT: SafetyBoard is not connected or 
//					there is no heartbeat from safetyboard.
//					ERR_SAFETY_RECV_CMD:	Command received is not from safety.
//                  ERR_SAFETY_RECV_DIFF:   data from two MCUs are different.
//  -----------------------------------------------------------------------
unsigned long long int autorunSafetyData(void);

//void safetyWriteDownload(void);
//void safetySetSeq(char seq);
//void safetyGetSeq(char *seq);
//unsigned long long int safetyReadUpload(void);

#ifdef __cplusplus
}
#endif 
