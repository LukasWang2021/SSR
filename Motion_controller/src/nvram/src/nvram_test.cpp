/*************************************************************************
	> File Name: nvram_test.cpp
	> Author: 
	> Mail: 
	> Created Time: 2020年07月03日 星期五 14时38分00秒
 ************************************************************************/

#include <iostream>
#include "nvram_handler.h"

using namespace std;
using namespace rtm_nvram;


int main(int argc, char **argv)
{
	NvramHandler nvram_handler;
	nvram_handler.init();

	uint8_t data[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
	uint8_t read[8];
	nvram_handler.readNvram(0x600, read, sizeof(read));
	printf("%x,%x,%x,%x,%x,%x,%x,%x\n", read[0], read[1], read[2], read[3], read[4], read[5], read[6], read[7]);
	nvram_handler.writeNvram(0x600, data, sizeof(data));
	nvram_handler.readNvram(0x600, read, sizeof(read));
	printf("%x,%x,%x,%x,%x,%x,%x,%x\n", read[0], read[1], read[2], read[3], read[4], read[5], read[6], read[7]);
	return 0;
}


