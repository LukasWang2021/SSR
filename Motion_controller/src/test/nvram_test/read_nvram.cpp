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

bool readNvram(uint32_t address, uint8_t *data, uint32_t length)
{
	if (length + 4 > NVRAM_RW_BUFFER_SIZE || address + length > NVARM_SIZE)
	{
		printf("Read nvram error, address: 0x%x, length: %d\n", address, length);
		return false;
	}

	g_write_buffer[0] = NVRAM_INSTRUCTION_READ;
	g_write_buffer[1] = (uint8_t)(address >> 16);
	g_write_buffer[2] = (uint8_t)(address >> 8);
	g_write_buffer[3] = (uint8_t)(address);
	
	printf("writeNvram, length(%d): 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",length + 4,
		g_write_buffer[0], g_write_buffer[1], g_write_buffer[2], g_write_buffer[3],
		g_write_buffer[4], g_write_buffer[5], g_write_buffer[6], g_write_buffer[7],
		g_write_buffer[8], g_write_buffer[9], g_write_buffer[10], g_write_buffer[11],
		g_write_buffer[12], g_write_buffer[3], g_write_buffer[14], g_write_buffer[15]);

	transferData(g_write_buffer, g_read_buffer, length + 4);
	memcpy(data, &g_read_buffer[4], length);
	return true;
}

int main(int argc, char **argv)
{
	uint8_t data[NVRAM_RW_BUFFER_SIZE];
	stringstream ss;
	char buffer[1024];
	uint32_t address, length;
	int res = initSpi();

	if (res != 0)
	{
		cout << "fail to init spi" << endl;
		return res;
	}

	if (argc < 3)
	{
		cout << "use like: read_nvram start_addr read_length [dump_file]" << endl;
		return 0;
	}

	if (argv[1][0] == '0' && argv[1][1] == 'x') {ss.clear(); ss.str(""); ss << std::hex << argv[1] + 2; ss >> address;}
	else {address = atoi(argv[1]);}
	
	if (argv[2][0] == '0' && argv[2][1] == 'x') {ss.clear(); ss.str(""); ss << std::hex << argv[2] + 2; ss >> length;}
	else {length = atoi(argv[2]);}

	printf("Read nvram addr: 0x%x, length: 0x%x\n", address, length);

	if (argc == 3)
	{
		uint32_t da[16];

		if (!readNvram(address, data, length))
		{
			printf("Read nvram failed");
			return -1;
		}

		for (uint32_t i = 0; i < length;)
		{
			da[0] = data[i + 0];
			da[1] = data[i + 1];
			da[2] = data[i + 2];
			da[3] = data[i + 3];
			da[4] = data[i + 4];
			da[5] = data[i + 5];
			da[6] = data[i + 6];
			da[7] = data[i + 7];
			da[8] = data[i + 8];
			da[9] = data[i + 9];
			da[10] = data[i + 10];
			da[11] = data[i + 11];
			da[12] = data[i + 12];
			da[13] = data[i + 13];
			da[14] = data[i + 14];
			da[15] = data[i + 15];
			sprintf(buffer, "0x%08x: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", address + i,
		     	da[0], da[1], da[2], da[3], da[4], da[5], da[6], da[7], da[8], da[9], da[10], da[11], da[12], da[13], da[14], da[15]);
			cout << buffer << endl;
			i += 16;
		}
	}
	else
	{
		uint32_t da[16];
		ofstream out(argv[3]);

		if (!readNvram(address, data, length))
		{
			printf("Read nvram failed");
			return -1;
		}

		for (uint32_t i = 0; i < 128 * 1024;)
		{
			da[0] = data[i + 0];
			da[1] = data[i + 1];
			da[2] = data[i + 2];
			da[3] = data[i + 3];
			da[4] = data[i + 4];
			da[5] = data[i + 5];
			da[6] = data[i + 6];
			da[7] = data[i + 7];
			da[8] = data[i + 8];
			da[9] = data[i + 9];
			da[10] = data[i + 10];
			da[11] = data[i + 11];
			da[12] = data[i + 12];
			da[13] = data[i + 13];
			da[14] = data[i + 14];
			da[15] = data[i + 15];

			sprintf(buffer, "0x%08x: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", address + i,
				da[0], da[1], da[2], da[3], da[4], da[5], da[6], da[7], da[8], da[9], da[10], da[11], da[12], da[13], da[14], da[15]);
			out << buffer << endl;
			i += 16;
		}
	}
	

    return 0;
}

