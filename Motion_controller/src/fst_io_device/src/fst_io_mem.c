#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include "fst_io_mem.h"
#include <string.h>

struct IODevice {
	int fd;
	void *ptr;
	uint32_t *p_wId;
	uint32_t *p_wRo;
	uint32_t *p_wDo;
	uint32_t *p_rState;
	uint32_t *p_rDi;
	uint32_t *p_rDo;
	uint32_t *p_version;
};
static struct IODevice *piodev;

int ioInit(void) 
{
	piodev = (struct IODevice *)malloc(sizeof(struct IODevice));
	if(piodev == NULL)
	{
		printf("ioInit: malloc IODevice is failed.\n");
		return -1;
	}

	piodev->fd = open("/dev/mem", O_RDWR);
	if (piodev->fd == -1) 
	{
		printf("ioInit: fd = -1\n");
		return -1;
	}

	piodev->ptr = mmap(NULL, IO_LENGTH, PROT_WRITE|PROT_READ, MAP_SHARED, piodev->fd, IO_BASE);
	if (piodev->ptr == (void *)-1) 
	{
		printf("ioInit: ptr = -1\n");
		close(piodev->fd);
		return -1;
	}

    piodev->p_wId = (uint32_t *)(piodev->ptr + IO_ADDR_WRITE_ID);
	piodev->p_wRo = (uint32_t *)(piodev->ptr + IO_ADDR_WRITE_RO);
	piodev->p_wDo = (uint32_t *)(piodev->ptr + IO_ADDR_WRITE_DO);
	piodev->p_rState = (uint32_t *)(piodev->ptr + IO_ADDR_READ_STATE);
	piodev->p_rDi = (uint32_t *)(piodev->ptr + IO_ADDR_READ_DI);
	piodev->p_rDo = (uint32_t *)(piodev->ptr + IO_ADDR_READ_DO);
	piodev->p_version = (uint32_t *)(piodev->ptr + IO_ADDR_READ_VERSION);

	return 0;
}

int ioWriteId(uint8_t offset, uint8_t address)
{
	if(offset >= IO_BOARD_NUM_MAX)
	{
		return -1;
	}

	uint32_t *p_enable = (uint32_t *)(piodev->ptr + IO_ADDR_SEND_ENABLE);
	//set enable to be false first.
	*p_enable = *p_enable & 0xFFFFF0FF;

	//set id and then enable to be true.
	uint32_t *p_wId = piodev->p_wId;
	uint32_t read_id = *p_wId;
	switch(offset)
	{
		case 0:
		    read_id = (read_id & 0xFFFFFFF0) + (address & 0x0F);
			*p_enable = *p_enable | 0x0100;
			break;
		case 1:
		    read_id = (read_id & 0xFFFFFF0F) + ((address & 0x0F) << 4);
			*p_enable = *p_enable | 0x0300;
			break;
		case 2:
		    read_id = (read_id & 0xFFFFF0FF) + ((address & 0x0F) << 8);
			*p_enable = *p_enable | 0x0700;
			break;
		case 3:
		    read_id = (read_id & 0xFFFF0FFF) + ((address & 0x0F) << 12);
			*p_enable = *p_enable | 0x0F00;
			break;
		default:
		    break;
	}
	memcpy(p_wId, &read_id, sizeof(read_id));
	return 0;
}

int ioWriteDownload(struct IODeviceData *io) 
{
	if(io->offset >= IO_BOARD_NUM_MAX)
	{
		return -1;
	}

	uint8_t *p_wRo = (uint8_t *)piodev->p_wRo + (io->offset << IO_OFFSET_BYTES);
	uint8_t *p_wDo = (uint8_t *)piodev->p_wDo + (io->offset << IO_OFFSET_BYTES);

	memcpy(p_wRo, &io->output[IO_DODI_LENGTH], IO_RORI_LENGTH);
	memcpy(p_wDo, &io->output, IO_DODI_LENGTH);

	return 0;
}

int ioReadUpload(struct IODeviceData *io) 
{
	if(io->offset >= IO_BOARD_NUM_MAX)
	{
		return -1;
	}
    //write first to notify FPGA
	uint32_t *p_rState = (uint32_t *)((uint8_t *)piodev->p_rState + (io->offset << IO_OFFSET_BYTES));
	*p_rState = *p_rState | 0xF000000;

    //read the state
	uint32_t read_state = *(uint32_t *)((uint8_t *)piodev->p_rState + (io->offset << IO_OFFSET_BYTES));
	io->enable = (uint8_t)((read_state & 0x10) >> IO_OFFSET_ENABLE);
	io->verify = (uint8_t)(read_state & 0x01);
	io->model = (uint8_t)((read_state & 0xF0000) >> IO_OFFSET_MODEL);

    //read DI and RI
	uint32_t read_di = *(uint32_t *)((uint8_t *)piodev->p_rDi + (io->offset << IO_OFFSET_BYTES));
	uint8_t read_ri = *((uint8_t *)piodev->p_rDi + (io->offset << IO_OFFSET_BYTES) + IO_DODI_LENGTH);
	memcpy(&io->input, &read_di, IO_DODI_LENGTH);
	memcpy(&io->input[IO_DODI_LENGTH], &read_ri, IO_RORI_LENGTH);

    //read DO and DI
	uint32_t read_do = *(uint32_t *)((uint8_t *)piodev->p_rDo + (io->offset << IO_OFFSET_BYTES));
	uint8_t read_ro = *((uint8_t *)piodev->p_rDo + (io->offset << IO_OFFSET_BYTES) + IO_DODI_LENGTH);
    memcpy(&io->output, &read_do, IO_DODI_LENGTH);
	memcpy(&io->output[IO_DODI_LENGTH], &read_ro, IO_RORI_LENGTH);

    return 0;
}

int ioBoardVersionFromMem(uint8_t offset, int *version)
{
	if(offset >= IO_BOARD_NUM_MAX)
	{
		*version = 0;
		return -1;
	}
	if(piodev == NULL)
    {
        *version = 0;
    }
    else
    {
		*version = *(int *)((uint8_t *)piodev->p_version + (offset << IO_OFFSET_BYTES));
    }
	return 0;
}

void ioClose(void)
{
	munmap(piodev->ptr, IO_LENGTH);
	close(piodev->fd);
}