#include <iostream>
#include <cstring>
#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <string>
#include "network_config.h"

#include "json.hpp"

NetworkConfig::NetworkConfig()
{
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1)
    {
        std::cerr << "Socket creation failed!" << std::endl;
        is_valid = false;
    }
    else
    {
        is_valid = true;
    }
}

NetworkConfig::~NetworkConfig()
{
    if (sockfd != -1)
    {
        close(sockfd);
    }
}

bool NetworkConfig::GetAllInterfaceNames(std::vector<std::string>& interfaces)
{
    if (!is_valid){
        return false;
    }

    struct ifreq ifr[50];
    struct ifconf ifc;
    char buffer[1024]{0};

    ifc.ifc_len = sizeof(buffer);
    ifc.ifc_buf = buffer;

    if (ioctl(sockfd, SIOCGIFCONF, &ifc) == -1){
        std::cerr << "Unable to Get interface information!" << std::endl;
        return false;
    }
    memcpy(ifr, buffer, ifc.ifc_len);
    int num_interfaces = ifc.ifc_len / sizeof(struct ifreq);
    for (int i = 0; i < num_interfaces; i++){
        interfaces.push_back(ifr[i].ifr_name);
    }

    return true;
}

void NetworkConfig::GetAllInterfaceInfo(const httplib::Request& req, httplib::Response& resp)
{
    nlohmann::json interfaces = nlohmann::json::array();
    std::vector<std::string> vec_if;
    GetAllInterfaceNames(vec_if);

    for (auto va : vec_if){
        auto ip = GetIPAddress(va);
        auto mask = GetNetmask(va);
        auto dns = GetDNS(va).front();
        auto gateway = GetGateway(va);

        nlohmann::json intf = {
            {"name", va},
            {"ip_address", ip},
            {"netmask", mask},
            {"gateway", gateway},
            {"dns", dns}

        };

        interfaces.push_back(intf);
    }

    nlohmann::json json_info = {
        {
            "data", {
                {"interfaces", interfaces}
            }
        }
    };

    resp.set_content(json_info.dump(-1), "application/json");
    resp.status = 200;
    resp.reason = "OK";
}

std::string NetworkConfig::GetIPAddress(const std::string interface)
{
    if (!is_valid)
    {
        return "";
    }

    struct ifreq ifr;
    struct sockaddr_in* ip_addr;

    std::strcpy(ifr.ifr_name, interface.c_str());
    if (ioctl(sockfd, SIOCGIFADDR, &ifr) == -1)
    {
        std::cerr << "Unable to Get IP address!" << std::endl;
        return "";
    }

    ip_addr = (struct sockaddr_in*)&ifr.ifr_addr;

    return inet_ntoa(ip_addr->sin_addr);
}

bool NetworkConfig::SetIPAddress(const std::string interface, const std::string ip_address)
{
    if (!is_valid)
    {
        return false;
    }

    struct ifreq ifr;
    struct sockaddr_in addr;

    std::strcpy(ifr.ifr_name, interface.c_str());
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(ip_address.c_str());
    std::memcpy(&ifr.ifr_addr, &addr, sizeof(struct sockaddr_in));

    if (ioctl(sockfd, SIOCSIFADDR, &ifr) == -1)
    {
        std::cerr << "Unable to set IP address!" << std::endl;
        return false;
    }

    return true;
}

std::string NetworkConfig::GetMACAddress(const std::string interface)
{
    if (!is_valid)
    {
        return "";
    }

    struct ifreq ifr;
    unsigned char* mac;

    std::strcpy(ifr.ifr_name, interface.c_str());
    if (ioctl(sockfd, SIOCGIFHWADDR, &ifr) == -1)
    {
        std::cerr << "Unable to Get MAC address!" << std::endl;
        return "";
    }

    mac = (unsigned char*)ifr.ifr_hwaddr.sa_data;
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return mac_str;
}

std::string NetworkConfig::GetNetmask(const std::string interface)
{
    if (!is_valid)
    {
        return "";
    }

    struct ifreq ifr;
    struct sockaddr_in* netmask_addr;

    std::strcpy(ifr.ifr_name, interface.c_str());
    if (ioctl(sockfd, SIOCGIFNETMASK, &ifr) == -1)
    {
        std::cerr << "Unable to Get Netmask!" << std::endl;
        return "";
    }

    netmask_addr = (struct sockaddr_in*)&ifr.ifr_netmask;
    return inet_ntoa(netmask_addr->sin_addr);
}

bool NetworkConfig::SetNetmask(const std::string interface, const std::string netmask)
{
    if (!is_valid)
    {
        return false;
    }

    struct ifreq ifr;
    struct sockaddr_in addr;

    std::strcpy(ifr.ifr_name, interface.c_str());
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(netmask.c_str());
    std::memcpy(&ifr.ifr_netmask, &addr, sizeof(struct sockaddr_in));

    if (ioctl(sockfd, SIOCSIFNETMASK, &ifr) == -1)
    {
        std::cerr << "Unable to set Netmask!" << std::endl;
        return false;
    }

    return true;
}

std::string NetworkConfig::GetGateway(std::string gateway)
{
    // std::ifstream route_file("/proc/net/route");
    // std::string line;
    // while (std::getline(route_file, line)) {
    //     if (line.empty()) continue;
    //
    //     // 路由表中默认网关一般显示为 "0.0.0.0" 或 "default"
    //     std::istringstream ss(line);
    //     std::string iface, destination, gateway_ip;
    //     ss >> iface >> destination >> gateway_ip;
    //
    //     if (destination == "00000000") {  // 表示默认网关
    //         unsigned int gateway_int;
    //         std::stringstream(gateway_ip) >> std::hex >> gateway_int;
    //         struct in_addr gateway_in;
    //         gateway_in.s_addr = htonl(gateway_int);
    //         gateway = inet_ntoa(gateway_in);
    //         return true;
    //     }
    // }
    return "";
}

bool NetworkConfig::SetInterfaceInfo(std::string info)
{
    nlohmann::json json_info = nlohmann::json::parse(info);

    auto interface_data = json_info["data"];
    bool res = false;
    for (const auto& iface_json : interface_data["interfaces"]){
        res = res & SetIPAddress(interface_data["name"], interface_data["ip_address"]);
        res = res & SetNetmask(interface_data["name"], interface_data["netmask"]);
        res = res & SetDNS(interface_data["name"], interface_data["dns"]);
        res = res & SetGateway(interface_data["name"], interface_data["gateway"]);
    }

    return res;
}
void NetworkConfig::SetInterfaceApi(const httplib::Request& req, httplib::Response& resp)
{
    nlohmann::json json_info = nlohmann::json::parse(req.body);

    auto interface_data = json_info["data"];
    bool res = false;
    for (const auto& iface_json : interface_data["interfaces"]){
        res = res & SetIPAddress(interface_data["name"], interface_data["ip_address"]);
        res = res & SetNetmask(interface_data["name"], interface_data["netmask"]);
        res = res & SetDNS(interface_data["name"], interface_data["dns"]);
        res = res & SetGateway(interface_data["name"], interface_data["gateway"]);
    }

    int status = res ? 0 : 1;

    nlohmann::json json_res = {
        {
            "data",{
                {"status",status},
                {"err_msg",0}
            }
        }
    };


    resp.set_content(json_res.dump(-1), "application/json");
    resp.status = 200;
    resp.reason = "OK";
}

bool NetworkConfig::SetGateway(std::string interface,std::string gateway)
{
    return true;
}

bool NetworkConfig::SetDNS(std::string dns1, std::string dns2)
{
    std::ofstream resolvConf("/etc/resolv.conf");

    if (!resolvConf.is_open()) {
        std::cerr << "Failed to open /etc/resolv.conf" << std::endl;
        return false;
    }

    resolvConf << "nameserver " << dns1 << std::endl;
    resolvConf << "nameserver " << dns2 << std::endl;

    resolvConf.close();

    return true;
}

std::vector<std::string> NetworkConfig::GetDNS(std::string interface)
{
    std::vector<std::string> dnsServers;
    std::ifstream resolvConf("/etc/resolv.conf");

    if (!resolvConf.is_open()) {
        std::cerr << "Failed to open /etc/resolv.conf" << std::endl;
        return dnsServers;
    }

    std::string line;
    while (std::getline(resolvConf, line)) {
        if (line.find("nameserver") != std::string::npos) {
            size_t pos = line.find(" ");
            if (pos != std::string::npos) {
                dnsServers.push_back(line.substr(pos + 1));
            }
        }
    }

    resolvConf.close();

    return dnsServers;
}
