/**
 * @file ip_address.cpp
 * @brief 
 * @author WangWei
 * @version 1.0.0
 * @date 2016-09-13
 */
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <unistd.h>
#include <arpa/inet.h>
#include "ip_address.h"
/**
 * @brief : get local ip address
 *
 * @return : the ip address in the form of string
 */
std::string getLocalIP()
{
	int fd;
    struct ifreq ifr;
     
    char iface[] = "eth0";
     
    fd = socket(AF_INET, SOCK_DGRAM, 0);
 
    //Type of address to retrieve - IPv4 IP address
    ifr.ifr_addr.sa_family = AF_INET;
 
    //Copy the interface name in the ifreq structure
    strncpy(ifr.ifr_name , iface , IFNAMSIZ-1);
 
    ioctl(fd, SIOCGIFADDR, &ifr);
 
    close(fd);
 
    //display result
    //printf("%s - %s\n" , iface , inet_ntoa(( (struct sockaddr_in *)&ifr.ifr_addr )->sin_addr) );
	std::string ret = inet_ntoa(( (struct sockaddr_in *)&ifr.ifr_addr )->sin_addr);
	return ret;
}
