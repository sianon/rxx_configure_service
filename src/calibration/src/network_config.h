#ifndef NETWORKCONFIG_H
#define NETWORKCONFIG_H

#include <vector>
#include <string>

#include "httplib.h"

class NetworkConfig
{
public:
    NetworkConfig();
    ~NetworkConfig();

    bool GetAllInterfaceNames(std::vector<std::string>& interfaces);
    void GetAllInterfaceInfo(const httplib::Request& req, httplib::Response& resp);

    bool SetInterfaceInfo(std::string info);
    void SetInterfaceApi(const httplib::Request& req, httplib::Response& res);

    std::string GetIPAddress(const std::string interface);
    bool SetIPAddress(const std::string interface, const std::string ip_address);

    std::string GetMACAddress(const std::string interface);

    std::string GetNetmask(const std::string interface);
    bool SetNetmask(const std::string interface, const std::string netmask);

    //TODO
    std::string GetGateway(std::string gateway);
    //TODO
    bool SetGateway(std::string interface,std::string gateway);
    //TODO
    bool SetDNS(std::string interface, std::string dns);
    //TODO
    std::vector<std::string> GetDNS(std::string interface);
private:
    int sockfd;
    bool is_valid;
    std::vector<std::string> network_interfaces_;
};


#endif //NETWORKCONFIG_H
