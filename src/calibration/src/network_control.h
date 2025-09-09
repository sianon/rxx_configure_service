#ifndef NETWORKCONTROL_H
#define NETWORKCONTROL_H

#include "httplib.h"
#include "calibration_collect.cpp"
#include "network_config.h"
#include "device_launch_conf.h"

class NetworkControl {
public:
    NetworkControl(){};
    ~NetworkControl(){};

    bool InitHttpService(int port);
    void ClientGetHttpFunc();
    void ClientPostHttpFunc();
    void SetCalibrationNode(std::shared_ptr<Calibration> node);

    void FileDownload(const httplib::Request& req, httplib::Response& res);
    void FileUpload(const httplib::Request& req, httplib::Response& res, const httplib::ContentReader &content_reader);
private:
    int port_ = 10001;
    httplib::Server http_server_;
    std::shared_ptr<Calibration> calibration_;
    std::shared_ptr<NetworkConfig> network_config_;
    std::shared_ptr<DeviceLaunchConf> device_config_;
};



#endif //NETWORKCONTROL_H
