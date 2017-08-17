#define	R_SAFETY_BASE	0X20	/* the address of date that read from safety, and 0x10 is a offset with a base 0xC1013000. */
#define	W_SAFETY_BASE	0X18	/* the address of date that written to safety, and 0x18 is a offset with a base C1013000. */
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

#define R_HB_FRAME		SECOND_FRAME
#define	W_HB_FRAME		SECOND_FRAME
#define NO_HB_TIMES		3	/* NO_HB_TIMES means the times that there is no heartbeat continually. Each time expends 50ms. If it is matched, the error, ERR_SAFETY_NOT_CONNECT is reported. */

#define SAFETY_CMD   0XF5

#define SAFETY_INPUT_FIRSTFRAME     0x0101
#define SAFETY_INPUT_SECONDFRAME    0x0102
// #define SAFETY_INPUT_THIRDFRAME     0x0103
// #define SAFETY_INPUT_FOURTHFRAME    0x0104
// #define	SAFETY_INPUT_FIFTHFRAME		0x0105
// #define SAFETY_INPUT_SIXTHFRAME		0x0106
// #define SAFETY_INPUT_SEVENTHFRAME	0x0107
// #define SAFETY_INPUT_EIGHTHFRAME	0x0108

#define SAFETY_OUTPUT_FIRSTFRAME    0x0201
#define SAFETY_OUTPUT_SECONDFRAME	0x0202
// #define SAFETY_OUTPUT_THIRDFRAME    0x0203
// #define SAFETY_OUTPUT_FOURTHFRAME   0x0204
// #define SAFETY_OUTPUT_FIFTHFRAME    0x0205
// #define SAFETY_OUTPUT_SIXTHFRAME    0x0206
// #define SAFETY_OUTPUT_SEVENFRAME    0x0207
// #define SAFETY_OUTPUT_EIGHTHFRAME   0x0208

#define ERR_SAFETY_FILE_OPEN (unsigned long long int)0x00010002008E0001   /*can't open file when initializing safety board*/
#define ERR_SAFETY_FILE_MAP (unsigned long long int)0x00010002008E0002   /*Mapping is failed.*/
#define ERR_SAFETY_RECV_CMD (unsigned long long int)0x00010002008E0015   /*Command is not received from the safety board*/
#define ERR_SAFETY_NOT_CONNECT (unsigned long long int)0x00010002008E0016   /*Safety board is not connected with, and there is not heartbeat from the safety board.*/
#define ERR_SAFETY_PTHREAD_INIT (unsigned long long int)0x00000001008E001F   /*Mutex initialization is failed.*/
#define ERR_SAFETY_PTHREAD_LOCK (unsigned long long int)0x00000001008E0020   /*Mutex lock is failed*/
#define ERR_SAFETY_PTHREAD_UNLOCK (unsigned long long int)0x00000001008E0021   /*Mutex unlock is failed*/
#define ERR_SAFETY_FRAME (unsigned long long int)0x00000001008E0029   /*The frame is out of range.*/

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
//  -----------------------------------------------------------------------
unsigned long long int autorunSafetyData(void);
void setSafetyTimer(void);

#ifdef __cplusplus
}
#endif 
