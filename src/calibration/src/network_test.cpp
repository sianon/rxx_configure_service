#include "network_control.h"
#include "rclcpp/rclcpp.hpp"
#include "base64.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Calibration>();
    DeviceLaunchConf grip_pawl_conf;
    // grip_pawl_conf.test();
    NetworkControl net_ctrl;
    net_ctrl.DbListenEvent();
    net_ctrl.SetCalibrationNode(node);
    net_ctrl.InitHttpService(10001);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
