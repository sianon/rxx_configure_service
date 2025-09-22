
#include "network_control.h"
#include <functional>
#include "base64.h"
#include "Kutil.h"
#include <filesystem>
#include <iostream>
bool NetworkControl::InitHttpService(int port)
{
    port_ = port;
    http_server_.set_base_dir("./");
    network_config_ = std::make_shared<NetworkConfig>();
    device_config_ = std::make_shared<DeviceLaunchConf>();
    last_seq = "now";
    db_name_ = "/my_database/";
    FetchDB();
    ClientGetHttpFunc();
    ClientPostHttpFunc();
    db_listen_ = std::thread(std::bind(&NetworkControl::DbListenEvent, this));
    // db_listen_.detach();
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

void NetworkControl::ClientPostHttpFunc(){
    http_server_.Post("/rxx/Upload", httplib::Server::Handler(std::bind(&NetworkControl::FileUpload, this, std::placeholders::_1, std::placeholders::_2)));
    http_server_.Post("/rxx/Download", httplib::Server::Handler(std::bind(&NetworkControl::FileDownload, this, std::placeholders::_1, std::placeholders::_2)));
    http_server_.Post("/rxx/Query", httplib::Server::Handler(std::bind(&NetworkControl::QueryModelWithUid, this, std::placeholders::_1, std::placeholders::_2)));
}

void NetworkControl::SetCalibrationNode(std::shared_ptr<Calibration> node)
{
    calibration_ = node;
}

void NetworkControl::FileDownload(const httplib::Request& req, httplib::Response& resp){
    nlohmann::json models_json_root = nlohmann::json::array();

    std::map<std::string, nlohmann::json> model_str;
    model_str = GetAllModels();

    if (req.body.empty()){
        for (auto va : model_str){
            models_json_root.push_back(va.second);
        }
    }else{
        nlohmann::json json_info = nlohmann::json::parse(req.body);

        int start_index= json_info["data"]["start_index"];
        int num = json_info["data"]["num"];
        std::vector<std::pair<std::string, nlohmann::json>> vec(model_str.begin(), model_str.end());
        if ((start_index + num) < vec.size()){
            for (auto it = vec.begin() + start_index; it != vec.begin() + start_index + num; ++it){
                models_json_root.push_back(it->second);
            }
        }
    }

    nlohmann::json json_info = {
        {
            "data", {
                            {"models", models_json_root}
            }
        }
    };

    resp.set_content(json_info.dump(-1), "application/json");
    resp.status = 200;
    resp.reason = "OK";
}

void NetworkControl::FileUpload(const httplib::Request& req, httplib::Response& resp){
    if (req.body.empty()){
        return;
    }

    std::map<std::string, nlohmann::json> upload_model_json;
    nlohmann::json json_info = nlohmann::json::parse(req.body);

    auto models_data = json_info["data"];
    bool res = false;
    for (const auto& model_json : models_data["models"]){
        SyncSingleModel2DB(model_json, "");
        Kutil::Base642File(model_json["content_b64"], model_json["uid"]);
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

void NetworkControl::QueryModelWithUid(const httplib::Request& req, httplib::Response& resp){
    if (req.body.empty()){
        return;
    }

    std::map<std::string, nlohmann::json> upload_model_json;
    nlohmann::json json_info = nlohmann::json::parse(req.body);

    auto models_data = json_info["data"];
    bool res = false;

    auto data_2_resp = all_model_json_b64[models_data["uid"]];

    int status = res ? 0 : 1;

    nlohmann::json json_resp = {
        {
            "data", {
                        {"models", data_2_resp}
            }
        }
    };

    resp.set_content(json_resp.dump(-1), "application/json");
    resp.status = 200;
    resp.reason = "OK";
}

bool NetworkControl::UpdateAll2DB(){

    for (auto va : all_model_json_b64){
        SyncSingleModel2DB(va.second, "");
    }

    return true;
}

bool NetworkControl::FetchDB(){
    httplib::Client cli("http://localhost:5984");

    std::string username = "admin";
    std::string password = "rxx123456";
    std::string auth = username + ":" + password;
    std::string auth_base64 = base64_encode(reinterpret_cast<const unsigned char*>(auth.c_str()), auth.length());

    cli.set_default_headers({{"Authorization", "Basic " + auth_base64}});

    auto res = cli.Get("/my_database/_all_docs");

    nlohmann::json json_info = nlohmann::json::parse(res->body);

    nlohmann::json rows = json_info["rows"];
    std::map<std::string, nlohmann::json> all_model_json;
    for (auto va : rows){
        all_model_json_b64[va["id"]] = "";
    }

    for (auto va : all_model_json_b64){
        auto tmp = cli.Get(db_name_ + va.first);

        nlohmann::json json_obj = nlohmann::json::parse(tmp->body);
        all_model_json_b64[va.first] = json_obj;

        std::cout << json_obj.dump(-1)<<std::endl;

        if (res && res->status == 200) {
            std::cout << "Document retrieved: " << res->body << std::endl;
        } else {
            std::cout << "Failed to retrieve document. Status code: " << res->status << std::endl;
        }
    }

    return true;
}

bool NetworkControl::SaveData2File(){
    for (auto va : all_model_json_b64){
        Kutil::Base642File(va.second["content_b64"], va.first + ".obj");
    }

    return true;
}

bool NetworkControl::SyncSingleModel2DB(nlohmann::json data, std::string url){
    httplib::Client cli("http://localhost:5984");

    std::string username = "admin";
    std::string password = "rxx123456";
    std::string auth = username + ":" + password;
    std::string auth_base64 = base64_encode(reinterpret_cast<const unsigned char*>(auth.c_str()), auth.length());

    cli.set_default_headers({{"Authorization", "Basic " + auth_base64}});

    std::string t_url = db_name_ + std::string(data["uid"]);

    auto res = cli.Put(url, data.dump(-1), "application/json");

    if (res->status == 200 || res->status == 201 || res->status == 202) {
        std::cout << "Document retrieved: " << res->body << std::endl;
        std::string url_t = db_name_ + std::string(data["uid"]);
        auto j_tmp = cli.Get(url_t);
        all_model_json_b64[std::string(data["uid"])] = nlohmann::json::parse(j_tmp->body);
    } else {
        std::cout << "Failed to retrieve document. Status code: " << res->status << std::endl;
    }
}

bool NetworkControl::test(){
    std::string b64_obj = Kutil::EncodeFile2Base64("tmp.obj");

    FetchDB();
    nlohmann::json pose_list = nlohmann::json::array();
    pose_list.push_back("x,y,z,r,p,y");
    pose_list.push_back("x,y,z,r,p,y");
    nlohmann::json json_obj = {
        {"name", "cellphone"},
        {"uid", "223fds1240"},
        {"pose", pose_list},
        {"content_base64", b64_obj}
    };

    int index = 1;
    for (int i = 0; i < 3; i++){
        json_obj["uid"] = json_obj["uid"].get<std::string>() + std::to_string(index);
        all_model_json_b64[std::to_string(i)] = json_obj;
        std::cout << json_obj.dump(-1)<<std::endl;
    }

    UpdateAll2DB();

    return true;
}
void NetworkControl::DbListenEvent(){
    httplib::Client cli("http://10.11.2.84:5984");
    last_seq = "now";
    db_name_ = "/rxx/";
    std::string username = "admin";
    std::string password = "rxx123456";
    std::string auth = username + ":" + password;
    std::string auth_base64 = base64_encode(reinterpret_cast<const unsigned char*>(auth.c_str()), auth.length());

    cli.set_default_headers({{"Authorization", "Basic " + auth_base64}});

    while (true){
        std::string t_url = db_name_ + "_changes" + "?include_docs=true&feed=longpoll&since=" + last_seq;
        auto res = cli.Get(t_url);
        auto tmp = nlohmann::json::parse(res->body);
        if(!tmp.contains("results")){
            continue;
        }
        nlohmann::json res_json = nlohmann::json::parse(res->body)["results"];
        for (auto va : res_json){
            if (!va.contains("doc") || va.contains("deleted") || !va["doc"].contains("_attachments")){
                continue;
            }

            auto id = std::string(va["doc"]["_id"]);

            DownloadDbPose2Disk(id, id + ".json", va["doc"]);

            std::cout << va.dump(-1) << std::endl;

            auto items = va["doc"]["_attachments"].items();
            if (items.begin() == items.end()){
                continue;
            }

            auto filename = items.begin().key();
            std::string url = db_name_ + id + "/" + filename;
            std::string pat = models_data_path_ + filename;
            std::cout << url << std::endl;
            AddFileName(cli, va["doc"]);
            DownloadDbObj2Disk(cli, id, url, filename);

        }

        last_seq = nlohmann::json::parse(res->body)["last_seq"];
    }
}

bool NetworkControl::DownloadDbObj2Disk(httplib::Client& cli, std::string id, std::string url, std::string name){
    std::string file_pat = models_data_path_ + id + "/";
    bool res = true;
    if (!std::filesystem::exists(file_pat)){
        res = std::filesystem::create_directory(file_pat);
    }

    auto response = cli.Get(url);

    if (response->status == 200 || response->status == 201 || response->status == 202){

    }
    std::ofstream outfile(file_pat + name, std::ios::binary);
    outfile.write(response->body.c_str(), response->body.size());
    outfile.close();

    return res;
}

bool NetworkControl::DownloadDbPose2Disk(std::string id, std::string name, nlohmann::json doc){
    std::string file_pat = models_data_path_ + id + "/";
    bool res = true;

    if (!doc.contains("pose")){
        return false;
    }

    if (!std::filesystem::exists(file_pat)){
        res = std::filesystem::create_directory(file_pat);
    }


    std::cout << doc.dump(-1) << std::endl;

    nlohmann::json json_pose = {
        "pose", doc["pose"]
    };
    std::cout << json_pose.dump(-1) << std::endl;
    std::ofstream outfile(file_pat + name, std::ios::out | std::ios::trunc);
    outfile.write(json_pose.dump(-1).c_str(), json_pose.dump(-1).size());
    outfile.close();

    return res;
}

bool NetworkControl::AddFileName(httplib::Client& cli, nlohmann::json doc){
    // if (doc.contains("name")){
    //     return true;
    // }

    std::string url = db_name_ + std::string(doc["_id"]);

    auto file_name = doc["_attachments"].begin().key();

    if (doc.contains("name") && doc["name"] == file_name){
        return true;
    }

    doc["name"] = file_name;
    auto response = cli.Put(url, doc.dump(), "application/json");

    nlohmann::json json_info = nlohmann::json::parse(response->body);

    return true;
}
