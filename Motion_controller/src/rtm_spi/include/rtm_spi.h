/*************************************************************************
	> File Name: rtm_spi.h
	> Author: Yun, Feng
	> Mail: 
	> Created Time: 2020年07月09日 星期四 14时28分28秒
 ************************************************************************/

#ifndef _RTM_SPI_H
#define _RTM_SPI_H

#include <stdint.h>

namespace rtm_spi
{

int initSpi(void);
int transferData(uint8_t *w_data, uint8_t *r_data, uint32_t length);

}


#endif
