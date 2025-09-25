#include "device_launch_conf.h"
#include "json.hpp"

#define INIT_JSON_RES(status_val, err_msg_val, resp)                                          \
    nlohmann::json json_res = {{"data", {{"status", status_val}, {"err_msg", err_msg_val}}}}; \
    resp.set_content(json_res.dump(-1), "application/json");                                  \
    resp.status = 200;                                                                        \
    resp.reason = "OK";

DeviceLaunchConf::DeviceLaunchConf() {
    grap_config_file_path_ = "config/grap_config.yaml";
    camera_config_file_path_ = "config/camera_config.yaml";
    arm_config_file_path_ = "config/arm_config.yaml";
    std::cout << "Current working directory: " << std::filesystem::current_path() << std::endl;
    grap_config_ = YAML::LoadFile(grap_config_file_path_);
    arm_config_ = YAML::LoadFile(arm_config_file_path_);
    camera_config_ = YAML::LoadFile(camera_config_file_path_);
};

bool DeviceLaunchConf::SaveConfig2File(std::string file_path, YAML::Node node) {
    if (file_path.empty()) {
        return false;
    }

    bool res = false;
    YAML::Emitter out;
    out << node;

    std::ofstream file(file_path);
    if (file.is_open()) {
        file << out.c_str();
        std::cout << "Configuration saved successfully to: " << file_path << std::endl;
        res = true;
    } else {
        std::cerr << "Failed to open file for saving: " << file_path << std::endl;
    }
    file.close();

    return res;
}

void DeviceLaunchConf::SetGripPawlConf(const httplib::Request& req, httplib::Response& resp) {
    nlohmann::json json_info = nlohmann::json::parse(req.body);
    auto interface_data = json_info["data"]["grippawls"];
    for (auto& val : interface_data) {
        std::string name = val["name"];
        int port = val["port"];
        int baud_rate = val["baud_rate"];
        int selected = val["selected"];

        bool flag_found = false;
        for (YAML::const_iterator it = grap_config_.begin(); it != grap_config_.end() & !flag_found; ++it) {
            auto config_name = it->first.as<std::string>();
            if (name == config_name) {
                grap_config_[name]["port"] = port;
                grap_config_[name]["baud_rate"] = baud_rate;
                grap_config_[name]["selected"] = selected;
                flag_found = true;
            }
        }
    }

    SaveConfig2File(grap_config_file_path_, grap_config_);
    INIT_JSON_RES(0, 0, resp);
}

void DeviceLaunchConf::GetGripPawlConf(const httplib::Request& req, httplib::Response& resp) {
    nlohmann::json grip_list = nlohmann::json::array();
    auto cur_selected = "";
    for (YAML::const_iterator it = grap_config_.begin(); it != grap_config_.end(); ++it) {
        auto config_name = it->first.as<std::string>();

        int port = it->second["port"].as<int>();
        int baud_rate = it->second["baud_rate"].as<int>();
        int selected = it->second["selected"].as<int>();

        std::cout << config_name << " Configuration:" << std::endl;
        std::cout << "Port: " << port << std::endl;
        std::cout << "Baud Rate: " << baud_rate << std::endl;
        std::cout << "--------------------------" << std::endl;
        nlohmann::json intf = {{"name", config_name}, {"port", port}, {"baud_rate", baud_rate}, {"selected", selected}};

        grip_list.push_back(intf);
    }

    nlohmann::json json_info = {{"data", {{"grippawls", grip_list}}}};

    resp.set_content(json_info.dump(-1), "application/json");
    resp.status = 200;
    resp.reason = "OK";
}

void DeviceLaunchConf::GetArmConf(const httplib::Request& req, httplib::Response& resp) {
    nlohmann::json arm_list = nlohmann::json::array();

    for (YAML::const_iterator it = arm_config_.begin(); it != arm_config_.end(); ++it) {
        auto config_name = it->first.as<std::string>();

        auto ip = it->second["ip_address"].as<std::string>();
        int selected = it->second["selected"].as<int>();

        std::cout << config_name << " Configuration:" << std::endl;
        std::cout << "ip: " << ip << std::endl;
        std::cout << "--------------------------" << std::endl;
        nlohmann::json intf = {{"name", config_name}, {"ip_addres", ip}, {"selected", selected}};

        arm_list.push_back(intf);
    }

    nlohmann::json json_info = {{"data", {{"arms", arm_list}}}};

    resp.set_content(json_info.dump(-1), "application/json");
    resp.status = 200;
    resp.reason = "OK";
}

void DeviceLaunchConf::SetArmConf(const httplib::Request& req, httplib::Response& resp) {
    nlohmann::json json_info = nlohmann::json::parse(req.body);

    auto interface_data = json_info["data"]["arms"];
    for (auto& val : interface_data) {
        std::string name = val["name"];
        std::string ip = val["ip_address"];
        int selected = val["selected"];

        bool flag_found = false;
        for (YAML::const_iterator it = arm_config_.begin(); it != arm_config_.end() & !flag_found; ++it) {
            auto config_name = it->first.as<std::string>();
            if (name == config_name) {
                arm_config_[name]["ip_address"] = ip;
                arm_config_[name]["selected"] = selected;
                flag_found = true;
            }
        }
    }
    SaveConfig2File(arm_config_file_path_, arm_config_);
    INIT_JSON_RES(0, 0, resp);
}

void DeviceLaunchConf::GetCameraConf(const httplib::Request& req, httplib::Response& resp) {
    nlohmann::json camera_list = nlohmann::json::array();

    for (YAML::const_iterator it = camera_config_.begin(); it != camera_config_.end(); ++it) {
        auto config_name = it->first.as<std::string>();

        int seria_num = it->second["seria_num"].as<int>();
        int selected = it->second["selected"].as<int>();

        std::cout << config_name << " Configuration:" << std::endl;
        std::cout << "Port: " << seria_num << std::endl;
        std::cout << "--------------------------" << std::endl;

        nlohmann::json intf = {{"name", config_name}, {"seria_num", seria_num}, {"selected", selected}};

        camera_list.push_back(intf);
    }

    nlohmann::json json_info = {{"data", {{"cameras", camera_list}}}};

    resp.set_content(json_info.dump(-1), "application/json");
    resp.status = 200;
    resp.reason = "OK";
}

void DeviceLaunchConf::SetCameraConf(const httplib::Request& req, httplib::Response& resp) {
    nlohmann::json json_info = nlohmann::json::parse(req.body);
    auto interface_data = json_info["data"]["cameras"];
    for (auto& val : interface_data) {
        std::string name = val["name"];
        std::string serial_num = val["seria_num"];
        int selected = val["selected"];

        bool flag_found = false;
        for (YAML::const_iterator it = camera_config_.begin(); it != camera_config_.end() & !flag_found; ++it) {
            auto config_name = it->first.as<std::string>();
            if (name == config_name) {
                camera_config_[name]["seria_num"] = serial_num;
                camera_config_[name]["selected"] = selected;
                flag_found = true;
            }
        }
    }
    SaveConfig2File(camera_config_file_path_, camera_config_);
    INIT_JSON_RES(0, 0, resp);
}
