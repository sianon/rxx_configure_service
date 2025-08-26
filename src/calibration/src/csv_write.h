//
// Created by cao on 8/26/25.
//

#ifndef CSVWRITE_H
#define CSVWRITE_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>

class CSVWriter{
public:
    CSVWriter(){}

    ~CSVWriter(){
        CloseFile();
    }

    void OpenCsvFile(std::string filename, std::string format="x,y,z,r,p,y\n");
    void CloseFile();
    void InsertData(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    unsigned int GetCountNum();
private:
    std::fstream csv_file_;
};


#endif //CSVWRITE_H
