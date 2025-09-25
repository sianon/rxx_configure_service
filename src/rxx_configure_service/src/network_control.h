#ifndef NETWORKCONTROL_H
#define NETWORKCONTROL_H

#include "httplib.h"
#include "calibration_collect.cpp"
#include "network_config.h"
#include "device_launch_conf.h"
#include "json.hpp"

class NetworkControl {
   public:
    NetworkControl() { models_data_path_ = "/home/rxx/jk/amodels/"; };
    ~NetworkControl(){
        http_server_.stop();
        http_thread_.join();
    };

    bool InitHttpService(int port);
    void StartHttpService();
    void ClientGetHttpFunc();
    void ClientPostHttpFunc();
    void SetCalibrationNode(std::shared_ptr<Calibration> node);

    void FileDownload(const httplib::Request& req, httplib::Response& res);
    void FileUpload(const httplib::Request& req, httplib::Response& res);
    void QueryModelWithUid(const httplib::Request& req, httplib::Response& res);
    void GetPose(const httplib::Request& req, httplib::Response& res);
    void ShowPose(const httplib::Request& req, httplib::Response& res);
    bool UpdateAll2DB();
    bool FetchDB();
    bool SaveData2File();
    bool SyncSingleModel2DB(nlohmann::json data, std::string url);
    bool test();
    void DbListenEvent();
    bool DownloadDbObj2Disk(httplib::Client& cli, std::string id, std::string url, std::string path);
    bool DownloadDbPose2Disk(std::string id, std::string name, nlohmann::json doc);
    bool AddFileName(httplib::Client& cli, nlohmann::json doc);

    std::map<std::string, nlohmann::json> GetAllModels() { return all_model_json_b64; };

   private:
    int port_ = 10001;
    httplib::Server http_server_;
    std::shared_ptr<Calibration> calibration_;
    std::shared_ptr<NetworkConfig> network_config_;
    std::shared_ptr<DeviceLaunchConf> device_config_;
    std::map<std::string, nlohmann::json> all_model_json_b64;
    std::string db_name_;
    std::thread db_listen_;
    std::thread http_thread_;
    std::string last_seq;
    std::map<std::string, nlohmann::json> will_be_process_evt_;
    std::string models_data_path_;
};

#endif  // NETWORKCONTROL_H
