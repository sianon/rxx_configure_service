#ifndef CALIBRATIONCOLLECT_H
#define CALIBRATIONCOLLECT_H

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "csv_write.h"
#include <filesystem>
#include "dobot_msgs_v3/srv/get_pose.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "calibration_srv/srv/calibration_eye_in_hand.hpp"
#include "json.hpp"

using namespace std::chrono_literals;
using GetPoseClient = dobot_msgs_v3::srv::GetPose;
using CalibrationTriggerClient = calibration_srv::srv::CalibrationEyeInHand;

class Calibration : public rclcpp::Node {
   public:
    Calibration() : Node("rxx_configure_service")
    , images_index_(1) {
        rgb_sensor_subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/camera/color/image_raw", 10, std::bind(&Calibration::on_rgb_image_received_, this, std::placeholders::_1));

        get_pose_client_ = this->create_client<GetPoseClient>("/dobot_bringup_v3/srv/GetPose");
        timer_ = this->create_wall_timer(10s, std::bind(&Calibration::TimerCallback, this));

        calibration_Trigger_client_ = this->create_client<CalibrationTriggerClient>("/Calibration/EyeInHand");

        this->declare_parameter("calibration_path", "/opt/rxx_calibration");
        this->get_parameter("calibration_path", rxx_calibration_path_);

        rxx_calibration_img_path_ = rxx_calibration_path_ + "/images/";

        std::cout << "rxx_calibration_path:" << rxx_calibration_path_ << std::endl;
        // timer_calibration_ = this->create_wall_timer(10s, std::bind(&Calibration::TimerCallback, this));
    }

    bool SaveImage() {
        if (last_image_ == nullptr)
            return false;
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(last_image_, sensor_msgs::image_encodings::BGR8);

            std::string filename = std::to_string(images_index_) + ".jpg";
            auto img_path = rxx_calibration_img_path_ + filename;
            cv::imwrite(img_path, cv_ptr->image);

            RCLCPP_INFO(this->get_logger(), "Image saved as %s", filename.c_str());
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Could not convert image: %s", e.what());
        }

        return true;
    }

    void InitCalibration(const httplib::Request& req, httplib::Response& resp) {
        int status = -1;
        if (!std::filesystem::exists(rxx_calibration_path_)) {
            std::filesystem::create_directories(rxx_calibration_path_);
            std::cout << "Directory created!" << std::endl;
        } else {
            std::cout << "Directory already exists!" << std::endl;
        }

        if (!std::filesystem::exists(rxx_calibration_img_path_)) {
            std::filesystem::create_directories(rxx_calibration_img_path_);
            std::cout << "Directory created!" << std::endl;
        } else {
            std::cout << "Directory already exists!" << std::endl;
        }
        images_index_ = 1;
        csv_writer_.OpenCsvFile(rxx_calibration_path_ + "/poses.csv");
        status = 0;

        nlohmann::json json_res = {{"data", {{"status", status}, {"err_msg", 0}}}};

        resp.set_content(json_res.dump(-1), "application/json");
        resp.status = 200;
        resp.reason = "OK";
    }

    void NextPoint(const httplib::Request& req, httplib::Response& resp) {
        auto request = std::make_shared<GetPoseClient::Request>();
        int status = -1;

        auto pose_str = GetPose();
        csv_writer_.InsertStringData(ParsePoseString(pose_str));
        bool save_tag = SaveImage();
        if (status != 0 || !save_tag) {
            status = 1;
        }

        nlohmann::json json_res = {
            {"data", {
                {"status", status},
                {"err_msg", 0},
                {"point", pose_str}
                    }
                }
        };

        resp.set_content(json_res.dump(-1), "application/json");
        resp.status = 200;
        resp.reason = "OK";
        images_index_++;
    }

    void DoneTriggerPose() {
        auto request = std::make_shared<CalibrationTriggerClient::Request>();
        int status = -1;
        std::condition_variable cv;
        std::mutex mtx;
        bool finished = false;
        calibration_Trigger_client_->async_send_request(request, [&](rclcpp::Client<CalibrationTriggerClient>::SharedFuture result_future) -> void{
            auto response = result_future.get();
            std::unique_lock<std::mutex> lock(mtx);

            if (response->err_code == 0){
                RCLCPP_INFO(this->get_logger(), "标定成功");
            }else{
                RCLCPP_INFO(this->get_logger(), "标定异常");
            }

            finished = true;
            cv.notify_one();
        });

        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [&]{ return finished; });
    }

    void TriggerCalibration(const httplib::Request& req, httplib::Response& resp) {
        csv_writer_.CloseFile();
        int status = -1;
        DoneTriggerPose();

        nlohmann::json json_res = {{"data", {{"status", status}, {"err_msg", 0}}}};

        resp.set_content(json_res.dump(-1), "application/json");
        resp.status = 200;
        resp.reason = "OK";
    }

    void TimerCallback() {
        while (!get_pose_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
        }
    }

    std::string GetCurrentTimeAsFilename() {
        std::time_t current_time = std::time(nullptr);
        std::tm* local_time = std::localtime(&current_time);

        std::ostringstream filename_stream;
        filename_stream << 1900 + local_time->tm_year << "-" << 1 + local_time->tm_mon << "-" << local_time->tm_mday << "_" << local_time->tm_hour << "-" << local_time->tm_min << "-" << local_time->tm_sec;

        return filename_stream.str();
    }

    std::string ParsePoseString(std::string input) {
        if (input.empty())
            return std::string();

        std::string res;
        size_t start = input.find("{");
        std::string errorID = input.substr(0, start - 1);

        start = input.find("{") + 1;
        size_t end = input.find("}");
        std::string pose = input.substr(start, end - start);

        return pose;
    }

    std::string GetPose() {
        auto request = std::make_shared<GetPoseClient::Request>();
        std::condition_variable cv;
        std::mutex mtx;
        bool finished = false;
        std::string pose_str = "";
        request->user;
        request->tool;
        get_pose_client_->async_send_request(request, [&](rclcpp::Client<GetPoseClient>::SharedFuture result_future) -> void{
          auto response = result_future.get();
          if (response->res == 0){
              std::cout << "pose:"<<response->pose<<std::endl;
              auto tmp = response->pose;
              pose_str = ParsePoseString(tmp);
              std::unique_lock<std::mutex> lock(mtx);
                finished = true;
                cv.notify_one();
              RCLCPP_INFO(this->get_logger(), "目标点处理成功");
          }else{
              RCLCPP_INFO(this->get_logger(), "目标点处理失败");
          }
        });

        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [&]{ return finished; });

        return pose_str;
    }

   private:
    void on_pose_received_(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received pose");
        const geometry_msgs::msg::Pose& pose = msg->pose;
        const geometry_msgs::msg::Point& position = pose.position;
        const geometry_msgs::msg::Quaternion& orientation = pose.orientation;

        RCLCPP_INFO(this->get_logger(), "Position -> x: %.2f, y: %.2f, z: %.2f", position.x, position.y, position.z);

        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(), "Orientation -> roll: %.2f, pitch: %.2f, yaw: %.2f", roll, pitch, yaw);
    }
    void on_rgb_image_received_(const sensor_msgs::msg::Image::SharedPtr msg) {
        last_image_ = msg;
    }

   private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sensor_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Client<GetPoseClient>::SharedPtr get_pose_client_;
    rclcpp::Client<CalibrationTriggerClient>::SharedPtr calibration_Trigger_client_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_calibration_;
    rclcpp::TimerBase::SharedPtr test_timer_;
    CSVWriter csv_writer_;
    sensor_msgs::msg::Image::SharedPtr last_image_;
    std::string rxx_calibration_path_;
    std::string rxx_calibration_img_path_;
    int images_index_;
};

#endif //CALIBRATIONCOLLECT_H
