
#include "network_control.h"
#include <functional>

bool NetworkControl::InitHttpService(int port)
{
    port_ = port;
    http_server_.set_base_dir("./");
    network_config_ = std::make_shared<NetworkConfig>();
    device_config_ = std::make_shared<DeviceLaunchConf>();

    ClientGetHttpFunc();
    ClientPostHttpFunc();

    bool status = http_server_.listen("0.0.0.0", port_);
    return status;
}

void NetworkControl::ClientGetHttpFunc()
{
    http_server_.Get("/rxx/CalibrationStart", std::bind(&Calibration::InitCalibration, calibration_, std::placeholders::_1, std::placeholders::_2));
    http_server_.Get("/rxx/CalibrationNext", std::bind(&Calibration::NextPoint, calibration_, std::placeholders::_1, std::placeholders::_2));
    http_server_.Get("/rxx/CalibrationDone", std::bind(&Calibration::TriggerCalibration, calibration_, std::placeholders::_1, std::placeholders::_2));

    http_server_.Get("/rxx/GetInterface", std::bind(&NetworkConfig::GetAllInterfaceInfo, network_config_, std::placeholders::_1, std::placeholders::_2));
    http_server_.Get("/rxx/SetInterface", std::bind(&NetworkConfig::SetInterfaceApi, network_config_, std::placeholders::_1, std::placeholders::_2));

    http_server_.Get("/rxx/SetGripPawlConf", std::bind(&DeviceLaunchConf::SetGripPawlConf, device_config_, std::placeholders::_1, std::placeholders::_2));
    http_server_.Get("/rxx/GetGripPawlConf", std::bind(&DeviceLaunchConf::GetGripPawlConf, device_config_, std::placeholders::_1, std::placeholders::_2));

    http_server_.Get("/rxx/GetArmConf", std::bind(&DeviceLaunchConf::GetArmConf, device_config_, std::placeholders::_1, std::placeholders::_2));
    http_server_.Get("/rxx/SetArmConf", std::bind(&DeviceLaunchConf::SetArmConf, device_config_, std::placeholders::_1, std::placeholders::_2));

    http_server_.Get("/rxx/GetCameraConf", std::bind(&DeviceLaunchConf::GetCameraConf, device_config_, std::placeholders::_1, std::placeholders::_2));
    http_server_.Get("/rxx/SetCameraConf", std::bind(&DeviceLaunchConf::SetCameraConf, device_config_, std::placeholders::_1, std::placeholders::_2));
}

void NetworkControl::ClientPostHttpFunc()
{

}

void NetworkControl::SetCalibrationNode(std::shared_ptr<Calibration> node)
{
    calibration_ = node;
}
