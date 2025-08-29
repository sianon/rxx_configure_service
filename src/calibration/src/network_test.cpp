#include <iostream>
#include <string>
#include "network_config.h"
#include "network_control.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Calibration>();

    NetworkControl net_ctrl;
    net_ctrl.InitHttpService(10001);
    net_ctrl.SetCalibrationNode(node);

    rclcpp::spin(node);
    rclcpp::shutdown();

    // NetworkConfig net_info;
    //
    // std::vector<std::string> interfaces;
    // if (!net_info.GetAllInterfaceNames(interfaces))
    // {
    //     std::cerr << "Failed to get network interfaces!" << std::endl;
    //     return -1;
    // }
    //
    // std::cout << "Interfaces found: " << std::endl;
    // for (const auto& iface : interfaces)
    // {
    //     std::cout << iface << std::endl;
    // }
    //
    // std::string ip, mac;
    // if (net_info.GetIPAddress("eth0") == "")
    // {
    //     std::cerr << "Failed to get IP address for eth0!" << std::endl;
    // }
    // else
    // {
    //     std::cout << "IP address for eth0: " << ip << std::endl;
    // }
    // std::vector<std::string> vec = net_info.GetDNS("eth0");
    //
    // if (vec.empty())
    // {
    //     std::cerr << "Failed to get IP address for eth0!" << std::endl;
    // }
    // else
    // {
    //     std::cout << "IP address for eth0: " << ip << std::endl;
    // }
    //
    // if (net_info.GetMACAddress("eth0") == "")
    // {
    //     std::cerr << "Failed to get MAC address for eth0!" << std::endl;
    // }
    // else
    // {
    //     std::cout << "MAC address for eth0: " << mac << std::endl;
    // }
    // if (net_info.GetNetmask("eth0") == "")
    // {
    //     std::cerr << "Failed to get MAC address for eth0!" << std::endl;
    // }
    // else
    // {
    //     std::cout << "MAC address for eth0: " << mac << std::endl;
    // }

    return 0;
}
