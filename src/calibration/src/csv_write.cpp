//
// Created by cao on 8/26/25.
//

#include "csv_write.h"
#include <iomanip>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void CSVWriter::OpenCsvFile(std::string filename, std::string format){
    csv_file_.open(filename, std::ios::in | std::ios::out | std::ios::trunc);

    if(csv_file_.tellp() == 0){
        csv_file_ << format;
    }
}

void CSVWriter::CloseFile(){
    if (csv_file_.is_open()){
        csv_file_.close();
    }
}

void CSVWriter::InsertData(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    const auto &position = msg->pose.position;
    const auto &orientation = msg->pose.orientation;

    tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    csv_file_ << position.x << ","
              << position.y << ","
              << position.z << ","
              << roll << ","
              << pitch << ","
              << yaw << "\n";

    // RCLCPP_INFO(this->get_logger(), "Pose data written to CSV.");
}

unsigned int CSVWriter::GetCountNum(){
    std::string line;
    int line_count = 0;

    while (std::getline(csv_file_, line)){
        if(line_count > 0){
            // 这里你可以选择只计算数据行，跳过头部行
            line_count++;
        }else{
            // 忽略文件头（标题行）
            line_count++;
        }
    }

    return line_count;
}