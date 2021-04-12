#ifndef LOCAL_IP_HPP
#define LOCAL_IP_HPP

/**
 * @file local_ip.h
 * @brief The file includes the method of acquiring local ip address.
 * @author zhengyu.shen
 */
#include <unistd.h>
#include <string>
#include <string.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <unistd.h>
#include <arpa/inet.h>

/**
 * @brief base_space includes all foundational definitions and realizations.
 */
namespace base_space
{
/**
 * @brief LocalIP provides a way to get local ip address.
 */
class LocalIP
{
public:
    /**
     * @brief Constructor of the class.
     * @details Try get local ip address when constructing.
     */    
    LocalIP()
    {
        int fd = socket(AF_INET, SOCK_DGRAM, 0);
    
        struct ifreq ifr;
        ifr.ifr_addr.sa_family = AF_INET;
    
        char iface[] = "eth0";
        strncpy(ifr.ifr_name, iface, IFNAMSIZ-1);
    
        ioctl(fd, SIOCGIFADDR, &ifr);
    
        close(fd);
    
        ip_ = inet_ntoa(((struct sockaddr_in*) & ifr.ifr_addr)->sin_addr);
    }
    /**
     * @brief Destructor of the class.
     */
    ~LocalIP(){}
    /**
     * @brief Get the local ip address in string format.
     * @return Local ip address.
     */
    std::string get()
    {
        return ip_;
    }

private:
    std::string ip_;    /**< Ip address.*/
};
}
#endif


