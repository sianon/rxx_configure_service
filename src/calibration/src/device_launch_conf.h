#ifndef GRIPPAWLCONF_H
#define GRIPPAWLCONF_H

#include "httplib.h"
#include "yaml-cpp/yaml.h"
#include "json.hpp"
#include <fstream>

template <typename Field>
void UpdateField(YAML::Node& config, const std::string& name, const nlohmann::json& val, Field field) {
    std::cout << "Updating field: " << field << std::endl;
    if constexpr (std::is_same_v<Field, std::string>) {
        if (field == "ip_address") {
            config[name]["ip_address"] = val["ip_address"];
        }
    } else if constexpr (std::is_same_v<Field, int>) {
        if (field == "port") {
            config[name]["port"] = val["port"];
        } else if (field == "baud_rate") {
            config[name]["baud_rate"] = val["baud_rate"];
        }
    }
}

template <typename... Fields>
void UpdateConfig(YAML::Node& config, const nlohmann::json& data, Fields... fields) {
    for (auto& val : data) {
        std::string data_name = val["name"];

        bool flag_found = false;
        for (auto it = config.begin(); it != config.end() && !flag_found; ++it) {
            std::string config_name = it->first.as<std::string>();
            if (data_name == config_name) {
                (UpdateField(config, data_name, val, fields), ...);
                flag_found = true;
            }
        }
    }
}

class DeviceLaunchConf{
public:
    DeviceLaunchConf();

    ~DeviceLaunchConf(){
    };

public:
    void SetGripPawlConf(const httplib::Request& req, httplib::Response& res);
    void GetGripPawlConf(const httplib::Request& req, httplib::Response& res);

    void GetArmConf(const httplib::Request& req, httplib::Response& res);
    void SetArmConf(const httplib::Request& req, httplib::Response& res);

    void GetCameraConf(const httplib::Request& req, httplib::Response& res);
    void SetCameraConf(const httplib::Request& req, httplib::Response& res);

    bool SaveConfig2File(std::string file_path, YAML::Node node);
    void test(){
        auto tt =
            "{\"status\": 0,\"cur_selected\": \"test_grap\",\"data\":{\"grippawls\": [{\"name\": \"test_grap\",\"port\": 33333,\"baud_rate\": 33333},{\"name\": \"realman\",\"port\": 10001,\"baud_rate\": 115200}]}}";

        nlohmann::json json_info = nlohmann::json::parse(tt);
        auto interface_data = json_info["data"]["grippawls"];
        UpdateConfig(grap_config_,interface_data,"port", "baud_rate");

        std::cout << interface_data.dump(-1) << std::endl;
        for (auto& val : interface_data){
            std::string name = val["name"];
            int port = val["port"];
            int baud_rate = val["baud_rate"];
            bool flag_found = false;
            for (YAML::const_iterator it = grap_config_.begin(); it != grap_config_.end() & !flag_found; ++it){
                auto config_name = it->first.as<std::string>();
                if (name == config_name){
                    grap_config_[name]["port"] = port;
                    grap_config_[name]["baud_rate"] = baud_rate;
                    flag_found = true;
                }
            }
        }
        YAML::Emitter out;

        out << grap_config_;

        std::ofstream file("../config/grap_config.yaml");
        if (file.is_open()) {
            file << grap_config_;
            std::cout << "Configuration saved successfully to: " << "../config/grap_config.yaml" << std::endl;
        } else {
            std::cerr << "Failed to open file for saving: " << "../config/grap_config.yaml" << std::endl;
        }
        file.close();
        for (YAML::const_iterator it = grap_config_.begin(); it != grap_config_.end(); ++it){
            auto config_name = it->first.as<std::string>();

            int port = it->second["port"].as<int>();
            int baud_rate = it->second["baud_rate"].as<int>();

            std::cout << config_name << " Configuration:" << std::endl;
            std::cout << "Port: " << port << std::endl;
            std::cout << "Baud Rate: " << baud_rate << std::endl;
            std::cout << "--------------------------" << std::endl;
        }
    }

private:
    YAML::Node grap_config_;
    YAML::Node arm_config_;
    YAML::Node camera_config_;

    std::string grap_config_file_path_;
    std::string arm_config_file_path_;
    std::string camera_config_file_path_;
};

#endif //GRIPPAWLCONF_H
