
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

void NetworkControl::ClientPostHttpFunc(){
    http_server_.Post("/rxx/Upload", httplib::Server::HandlerWithContentReader(std::bind(&NetworkControl::FileUpload, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)));
    http_server_.Post("/rxx/Download", httplib::Server::Handler(std::bind(&NetworkControl::FileDownload, this, std::placeholders::_1, std::placeholders::_2)));
}

void NetworkControl::SetCalibrationNode(std::shared_ptr<Calibration> node)
{
    calibration_ = node;
}

void NetworkControl::FileDownload(const httplib::Request& req, httplib::Response& res){
    std::cout << req.body << std::endl;
    std::cerr << "Server-log: download\t" << req.path_params.at("id") << "\t" << req.get_header_value("Content-Type") <<
        std::endl;

    res.set_header("Cache-Control", "no-cache");
    res.set_header("Content-Disposition", "attachment; filename=hello.txt");
    res.set_chunked_content_provider("multipart/form-data", [](size_t offset, httplib::DataSink& sink){
            const char arr[] = "hello world";
            auto ret = sink.write(arr + offset, sizeof(arr));
            sink.done();
            std::cerr << "\tdownload write:" << sizeof(arr) << std::endl;
            return !!ret;
        });
}

void NetworkControl::FileUpload(const httplib::Request& req, httplib::Response& res, const httplib::ContentReader& content_reader){
    // nlohmann::json json_info = nlohmann::json::parse(req.body);
    std::cout << res.body << std::endl;
    std::string str_data;
    using namespace  httplib;

    if (req.is_multipart_form_data()) {
        std::vector<FormData> items;
        content_reader(
          [&](const FormData &item) {
            items.push_back(item);
            return true;
          },
          [&](const char *data, size_t data_length) {
            items.back().content.append(data, data_length);
            return true;
          });

        for (const auto& item : items) {
            if (item.filename.empty()) {
                // Text field
                std::cout << "Field: " << item.name << " = " << item.content << std::endl;
            } else {
                // File
                std::cout << "File: " << item.name << " (" << item.filename << ") - "
                          << item.content.size() << " bytes" << std::endl;
            }
        }
    } else {
        std::string body;
        content_reader([&](const char *data, size_t data_length) {
            body.append(data, data_length);
            std::cerr << "\tupload read:" << data_length << std::endl;
            return true;
        });
        std::cerr << "\tupload read " << body << std::endl;
    }

}
