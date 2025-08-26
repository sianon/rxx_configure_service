#include <chrono>
#include <cstdlib>
#include <ctime>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/msg/image.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "dobot_msgs_v3/srv/get_pose.hpp"
#include "csv_write.h"

using namespace std::chrono_literals;
using GetPoseClient = dobot_msgs_v3::srv::GetPose;

class Calibration : public rclcpp::Node
{
public:
    Calibration() : Node("calibration_control"){
        // rgb_sensor_subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/turtle1/raw_image", 10,
        //     std::bind(&Calibration::on_rgb_image_received_, this, std::placeholders::_1));
        // pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/turtle1/pose", 10,
        //     std::bind(&Calibration::on_pose_received_, this, std::placeholders::_1));
        //
        // get_pose_client_ = this->create_client<GetPoseClient>("getposeclient");
        // timer_ = this->create_wall_timer(10s, std::bind(&Calibration::TimerCallback, this));
    }

    void on_pose_received_(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "Received pose");
        const geometry_msgs::msg::Pose &pose = msg->pose;
        const geometry_msgs::msg::Point &position = pose.position;
        const geometry_msgs::msg::Quaternion &orientation = pose.orientation;

        RCLCPP_INFO(this->get_logger(), "Position -> x: %.2f, y: %.2f, z: %.2f", position.x, position.y, position.z);

        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(), "Orientation -> roll: %.2f, pitch: %.2f, yaw: %.2f", roll, pitch, yaw);

  }
    void on_rgb_image_received_(const sensor_msgs::msg::Image::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received Image");
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            std::string filename = GetCurrentTimeAsFilename() + ".png";
            cv::imwrite(filename, cv_ptr->image);

            RCLCPP_INFO(this->get_logger(), "Image saved as %s", filename.c_str());
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Could not convert image: %s", e.what());
        }
    }
    void InitCalibration(){
        // auto message = geometry_msgs::msg::Twist();
        // velocity_publisher_->publish(message);
    }
    void NextPoint()
    {
        auto request = std::make_shared<GetPoseClient::Request>();
        request->user;
        request->tool;
        get_pose_client_->async_send_request(request,[&](rclcpp::Client<GetPoseClient>::SharedFuture result_future) -> void{
            auto response = result_future.get();
              if (response->res == 0){
                  response->pose;
                  RCLCPP_INFO(this->get_logger(), "目标点处理成功");
              }else{
                RCLCPP_INFO(this->get_logger(), "目标点处理失败");
              }
            });
    }
    void TriggerCalibration(){
        // auto message = geometry_msgs::msg::Twist();
        // velocity_publisher_->publish(message);
    }
    void TimerCallback(){
        while (!get_pose_client_->wait_for_service(1s)){
            if (!rclcpp::ok()){
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
        filename_stream << 1900 + local_time->tm_year << "-"
                        << 1 + local_time->tm_mon << "-"
                        << local_time->tm_mday << "_"
                        << local_time->tm_hour << "-"
                        << local_time->tm_min << "-"
                        << local_time->tm_sec;

        return filename_stream.str();
    }
    std::vector<std::string> ParsePoseString(std::string input) {
        if (input.empty())
            return std::vector<std::string>();

        std::vector<std::string> res;

        size_t start = input.find("{");
        std::string errorID = input.substr(0, start - 1);
        std::cout << "ErrorID: " << errorID << std::endl;

        start = input.find("{") + 1;
        size_t end = input.find("}");
        std::string pose = input.substr(start, end - start);

        std::stringstream ss(pose);
        std::string value;

        while (getline(ss, value, ',')) {
            res.push_back(value);
        }

        std::cout << "Position (X, Y, Z): " << res[0] << ", " << res[1] << ", " << res[2] << std::endl;
        std::cout << "Rotation (Rx, Ry, Rz): " << res[3] << ", " << res[4] << ", " << res[5] << std::endl;

        start = input.find("User=") + 5;
        size_t userEnd = input.find(",", start);
        std::string userIndex = input.substr(start, userEnd - start);

        start = input.find("Tool=") + 5;
        std::string toolIndex = input.substr(start, input.find(")", start) - start);

        std::cout << "User Index: " << userIndex << ", Tool Index: " << toolIndex << std::endl;

        return res;
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sensor_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp::Client<GetPoseClient>::SharedPtr get_pose_client_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Calibration>();
    std::string input = "1,{30,60,90,0.2,0.3,0.4},GetPose(User=1,Tool=2);";
    node->ParsePoseString(input);
    geometry_msgs::msg::PoseStamped::SharedPtr msg = std::make_shared<geometry_msgs::msg::PoseStamped>();


    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
