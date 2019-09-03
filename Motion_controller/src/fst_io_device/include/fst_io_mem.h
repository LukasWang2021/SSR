
#ifndef FST_IO_MEM_H_
#define FST_IO_MEM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define IO_DATAFRAME_MAX 5

struct IODeviceData {
	uint8_t	offset;
	uint8_t enable;
	uint8_t verify;
	uint8_t model;
	unsigned char input[IO_DATAFRAME_MAX];
	unsigned char output[IO_DATAFRAME_MAX];
};

#define IO_BASE 0xC0090000
#define IO_LENGTH 0x1000
#define IO_ADDR_WRITE_ID 0x0C
#define IO_ADDR_WRITE_RO 0x14
#define IO_ADDR_WRITE_DO 0x10
#define IO_ADDR_READ_STATE 0x28
#define IO_ADDR_READ_DI 0x18
#define IO_ADDR_READ_DO 0x20
#define IO_ADDR_READ_VERSION 0x2C
#define IO_ADDR_SEND_ENABLE 4

#define IO_BOARD_NUM_MAX 4
#define IO_OFFSET_BYTES 5
//from IO_ADDR_READ_STATE
#define IO_OFFSET_ENABLE 4
#define IO_OFFSET_MODEL 16
#define IO_RORI_LENGTH 1
#define IO_DODI_LENGTH 4


int ioInit(void);

int ioWriteId(uint8_t offset, uint8_t address);

int ioWriteDownload(struct IODeviceData *io);

int ioReadUpload(struct IODeviceData *io);

int ioBoardVersionFromMem(uint8_t offset, int *version);

void ioClose(void);


#ifdef __cplusplus
}
#endif

#endif
