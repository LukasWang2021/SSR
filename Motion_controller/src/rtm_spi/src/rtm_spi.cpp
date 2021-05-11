/*************************************************************************
	> File Name: rtm_spi.cpp
	> Author: Yun, Feng
	> Mail: 
	> Created Time: 2020年07月09日 星期四 14时28分37秒
 ************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <unistd.h>
#include <signal.h>
#include <sys/mman.h>
#include <iostream>
#include "rtm_spi.h"
#define RTM_SPI_BASE_ADDRESS 0x400a0000
#define RTM_SPI_BIT_SOFT_CS 0x00000020
#define RTM_SPI_BIT_CS_CTRL 0x00000010
#define RTM_SPI_BIT_ENABLE 0x00000001
#define RTM_SPI_BIT_RX_FULL 0x00000020
#define RTM_SPI_BIT_RX_EMPTY 0x00000010
#define RTM_SPI_BIT_TX_FULL 0x00000008
#define RTM_SPI_BIT_TX_EMPTY 0x00000004
#define RTM_SPI_BIT_RX_BUSY 0x00000002
#define RTM_SPI_BIT_TX_BUSY 0x00000001

#define RTM_SPI_RW_BUFFER_LENGTH 2048

using namespace std;

namespace rtm_spi
{

struct SpiRegister
{
	uint32_t id;
	uint32_t ctrl;
	uint32_t rw_byte;
	uint32_t rw_world;
	uint32_t state;
	uint32_t rx_fifo_ctrl;
	uint32_t dummy;
	uint32_t tx_fifo_ctrl;
};

static SpiRegister *s_spi_reg = NULL;

void enableCS(void)
{
	if (!s_spi_reg) return;
	s_spi_reg->ctrl &= ~RTM_SPI_BIT_SOFT_CS;
}

void disableCS(void)
{
	if (!s_spi_reg) return;
	s_spi_reg->ctrl |= RTM_SPI_BIT_SOFT_CS;
}

bool isRxBusy(void)
{
	return (s_spi_reg->state & RTM_SPI_BIT_RX_BUSY) != 0;
}

bool isTxBusy(void)
{
	return (s_spi_reg->state & RTM_SPI_BIT_TX_BUSY) != 0;
}

bool isRxBufferEmpty(void)
{
	return (s_spi_reg->state & RTM_SPI_BIT_RX_EMPTY) != 0;
}

bool isTxBufferFull(void)
{
	return (s_spi_reg->state & RTM_SPI_BIT_TX_FULL) != 0;
}

int initSpi(void)
{
	int fd = open("/dev/mem", O_RDWR);

    if (fd == -1)
	{
        return -1;
    }

	void *ptr = mmap(NULL, sizeof(SpiRegister), PROT_READ|PROT_WRITE, MAP_SHARED, fd, RTM_SPI_BASE_ADDRESS);

    if (ptr == MAP_FAILED)
	{
        return -2;
    }

	s_spi_reg = (SpiRegister*)ptr;
	s_spi_reg->ctrl = RTM_SPI_BIT_ENABLE | RTM_SPI_BIT_CS_CTRL | RTM_SPI_BIT_SOFT_CS;
	//disableCS();
	uint32_t dummy;

	while (!isRxBufferEmpty()) dummy = s_spi_reg->rw_byte;

    return 0;
}

int transferData(uint8_t *w_data, uint8_t *r_data, uint32_t length)
{
	if (length > RTM_SPI_RW_BUFFER_LENGTH) return -1;
	enableCS();

	for (uint32_t i = 0; i < length; i++)
	{
		s_spi_reg->rw_byte = (uint32_t)(w_data[i]);
	}

	uint32_t loop = 0;
	while (isTxBusy() && loop < 10) {usleep(1000); loop++;}
	disableCS();

	for (uint32_t i = 0; i < length; i++)
	{
		r_data[i] = (uint8_t)s_spi_reg->rw_byte;
	}

	while (!isRxBufferEmpty()) cout << "Rx buffer not empty: " << hex << s_spi_reg->rw_byte << endl;
	return 0;
}


}
