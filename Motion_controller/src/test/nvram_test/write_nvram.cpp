/*************************************************************************
	> File Name: read_nvram.cpp
	> Author: 
	> Mail: 
	> Created Time: 2020年07月03日 星期五 15时49分04秒
 ************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include "rtm_spi.h"
#include "nvram.h"


#define NVRAM_INSTRUCTION_READ 	0x03
#define NVRAM_INSTRUCTION_WRITE 0x02
#define NVRAM_RW_BUFFER_SIZE (160 * 1024)

using namespace std;
using namespace rtm_spi;
using namespace rtm_nvram;

//static FstSpi *spi_ptr_;
uint8_t g_read_buffer[NVRAM_RW_BUFFER_SIZE];
uint8_t g_write_buffer[NVRAM_RW_BUFFER_SIZE];

bool writeNvram(uint32_t address, uint8_t *data, uint32_t length)
{
	if (length + 4 > NVRAM_RW_BUFFER_SIZE || address + length > NVARM_SIZE)
	{
		printf("Write nvram error, address: 0x%x, length: %d\n", address, length);
		return false;
	}

	g_write_buffer[0] = NVRAM_INSTRUCTION_WRITE;
	g_write_buffer[1] = (uint8_t)(address >> 16);
	g_write_buffer[2] = (uint8_t)(address >> 8);
	g_write_buffer[3] = (uint8_t)(address);
	memcpy(&g_write_buffer[4], data, length);
	
	printf("writeNvram, length(%d): 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",length + 4,
		g_write_buffer[0], g_write_buffer[1], g_write_buffer[2], g_write_buffer[3],
		g_write_buffer[4], g_write_buffer[5], g_write_buffer[6], g_write_buffer[7],
		g_write_buffer[8], g_write_buffer[9], g_write_buffer[10], g_write_buffer[11],
		g_write_buffer[12], g_write_buffer[3], g_write_buffer[14], g_write_buffer[15]);
	
	transferData(g_write_buffer, g_read_buffer, length + 4);
	return true;
}

int main(int argc, char **argv)
{
	uint8_t data[NVRAM_RW_BUFFER_SIZE];
	stringstream ss;
	uint32_t address, length = 0;
	int res = initSpi();

	if (res != 0)
	{
		cout << "fail to init spi" << endl;
		return res;
	}

	if (argc < 3)
	{
		cout << "use like: write_nvram start_addr [data_in_byte]" << endl;
		return 0;
	}

	if (argv[1][0] == '0' && argv[1][1] == 'x') {ss.clear(); ss.str(""); ss << std::hex << argv[1] + 2; ss >> address;}
	else {address = atoi(argv[1]);}
	
	for (int i = 0; i < argc - 2; i++)
	{
		if (argv[i+2][0] == '0' && argv[i+2][1] == 'x') {ss.clear(); ss.str(""); ss << std::hex << argv[i+2] + 2; ss >> data[i];}
		else {data[i] = atoi(argv[i+2]);}
		length++;
	}
	
	printf("Write nvram addr: 0x%x, length: 0x%x\n", address, length);
	writeNvram(address, data, length);
    return 0;
}

