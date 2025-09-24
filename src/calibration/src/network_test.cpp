#include "base64.h"
#include "network_control.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Calibration>();
    NetworkControl net_ctrl;
    net_ctrl.SetCalibrationNode(node);
    net_ctrl.InitHttpService(10001);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
