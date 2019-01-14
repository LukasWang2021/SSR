
#ifndef IOBOARD_IOBOARD_H_
#define IOBOARD_IOBOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define IO_DATAFRAME_MAX 5
#define IOERROR (-1)

struct IODeviceData {
	uint8_t	id;
	uint8_t enable;
	uint8_t verify;
	uint8_t model;
	unsigned char input[IO_DATAFRAME_MAX];
	unsigned char output[IO_DATAFRAME_MAX];
};

#define IO_BASE 0xC0090000
#define LENGTH 0x1000
#define ADDR_FLAG_WRITE 0x30
#define ADDR_FLAG_READ 0x38
#define ADDR_DATA_WRITE 0x50
#define ADDR_DATA_READ 0x40

#define START_FRAME_VALUE 0x3C

#define OF_START 0x0
#define OF_ID 0xC
#define OF_ENABLE 0x19
#define OF_VERIFY 0x18
#define OF_MODEL 0x8
#define OF_FRAME_INPUT 0x4
#define OF_FRAME_OUTPUT 0xB

#define MASK_ID (0x0f << OF_ID)
#define MASK_ENABLE	(0x01 << OF_ENABLE)
#define MASK_VERIFY (0x01 << OF_VERIFY)
#define MASK_MODEL (0x0f << OF_MODEL)


int ioInit(void);

int ioSetIdSeq(uint8_t idseq);

int ioGetSeq(uint8_t *seq);

int ioWriteDownload(struct IODeviceData *idd);

int ioReadUpload(struct IODeviceData *idd);


#ifdef __cplusplus
}
#endif

#endif