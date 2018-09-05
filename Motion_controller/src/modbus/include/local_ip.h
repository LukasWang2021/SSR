#ifndef MODBUS_LOCAL_IP_HPP
#define MODBUS_LOCAL_IP_HPP

#include <unistd.h>
#include <string>
#include <string.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <unistd.h>
#include <arpa/inet.h>

using namespace std;

namespace fst_modbus
{
class LocalIP
{
public:
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

    ~LocalIP(){}

    string get()
    {
        return ip_;
    }

private:
    string ip_;
};
}
#endif


