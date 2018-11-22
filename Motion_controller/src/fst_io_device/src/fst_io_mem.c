#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include "fst_io_mem.h"
#include <string.h>

#define PARSEDATA(s, mask, offset) ((uint8_t)(((s) & mask) >> offset))
#define INTEGDATA(s, mask, offset) (((s) << offset) & mask)

#define IO_BYTE_MAX (0x10)
#define IO_WORD_MAX (IO_BYTE_MAX >> 2) // =4

struct IODevice {
	int fd;
	void *ptr;
	uint32_t *pfw;
	uint32_t *pfr;
	uint32_t *pdw;
	uint32_t *pdr;
};
static struct IODevice *piodev;

int trueInit() {
	struct IODevice *p = piodev;

	p->fd = open("/dev/mem", O_RDWR);
	if (p->fd == -1) {
		printf("trueInit: fd = -1\n");
		return IOERROR;
	}

	p->ptr = mmap(NULL, LENGTH, PROT_WRITE|PROT_READ, MAP_SHARED, p->fd, IO_BASE);
	if (p->ptr == (void *)-1) {
		printf("trueInit: ptr = -1\n");
		return IOERROR;
	}

	p->pfw = (uint32_t *)(p->ptr + ADDR_FLAG_WRITE);
	p->pfr = (uint32_t *)(p->ptr + ADDR_FLAG_READ);
	p->pdw = (uint32_t *)(p->ptr + ADDR_DATA_WRITE);
	p->pdr = (uint32_t *)(p->ptr + ADDR_DATA_READ);

	return 0;
}

int ioInit(void) {
	int ret = 0;

	piodev = (struct IODevice *)malloc(sizeof(struct IODevice));
	if(piodev == NULL){
		printf("ioInit: malloc IODevice is failed.\n");
		ret = -1;
		return ret;
	}

	ret = trueInit();

	return ret;
}

int ioSetIdSeq(uint8_t idseq) {
	*piodev->pfw = (uint32_t)idseq;
	return 0;
}

int ioGetSeq(uint8_t *seq) {
    *seq = *(uint8_t *)piodev->pfr;
    return 0;
}

void ioIntegParameter(uint32_t *parameter, struct IODeviceData *idd) {
	uint32_t param = START_FRAME_VALUE;
	param |= INTEGDATA((uint32_t)idd->id, MASK_ID, OF_ID);
	//param |= INTEGDATA((uint32_t)idd->enable, MASK_ENABLE, OF_ENABLE);//d
	//param |= INTEGDATA((uint32_t)idd->verify, MASK_VERIFY, OF_VERIFY);//d
	//param |= INTEGDATA((uint32_t)idd->model, MASK_MODEL, OF_MODEL);//d

	*parameter = param;
}

void ioWriteIOstate(uint8_t *dst, uint8_t *src, int num) {
	int i = 0;
	uint8_t *pd = dst;
	uint8_t *ps = src;
	for (i = 0; i < num; i++)
	  pd[i] = ps[i];
}

void setFpga(uint32_t *fpga, uint32_t *buf, int num) {
	int i = 0;
	for (i = 0; i < num; i++)
	    fpga[i] = buf[i];
}

int ioWriteDownload(struct IODeviceData *idd) {
	uint32_t *ptr = piodev->pdw;
	uint32_t buf[IO_WORD_MAX];

	ioIntegParameter(buf, idd);  // transfer the data of idd to buf
	ioWriteIOstate((uint8_t *)buf + OF_FRAME_INPUT, (uint8_t *)idd->output, (int)IO_DATAFRAME_MAX);
	setFpga(ptr, buf, (int)IO_WORD_MAX);  //data from buf to mem.

	return 0;
}

void getFpga(uint32_t *buf, uint32_t *fpga, int num) {
	int i = 0;
	for (i = 0; i < num; i++) {
		buf[i] = fpga[i];
	}
}
void ioParseParameter(struct IODeviceData *idd, uint32_t *parameter) {
    uint32_t param = *parameter;

	idd->id = PARSEDATA(param, MASK_ID, OF_ID);
	idd->enable = PARSEDATA(param, MASK_ENABLE, OF_ENABLE);
	idd->verify = PARSEDATA(param, MASK_VERIFY, OF_VERIFY);
	idd->model = PARSEDATA(param, MASK_MODEL, OF_MODEL);
}

void ioReadIOstate(uint8_t *dst, uint8_t *src, int num) {
	uint8_t *pd = dst;
	uint8_t *ps = src;
	int i = 0;
	for (i = 0; i < num; i++) {
		pd[i] = ps[i];
	}
}

int ioReadUpload(struct IODeviceData *idd) {
    uint32_t *ptr = piodev->pdr;
    uint32_t buf[IO_WORD_MAX];

    getFpga(buf, ptr, (int)IO_WORD_MAX); //data from mem to buf.
    ioParseParameter(idd, buf);          //pass value to structure.
    ioReadIOstate((uint8_t *)idd->input, (uint8_t *)buf + OF_FRAME_INPUT, (int)IO_DATAFRAME_MAX);
    ioReadIOstate((uint8_t *)idd->output, (uint8_t *)buf + OF_FRAME_OUTPUT, (int)IO_DATAFRAME_MAX);

    return 0;
}

