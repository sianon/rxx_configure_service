//
// Created by cao on 9/9/25.
//

#include <iostream>
#include <fstream>
#include <vector>
#include "base64.h"
#include "Kutil.h"

std::string Kutil::other_model_path_ = "";
std::string Kutil::obj_path_ = "";

std::string Kutil::EncodeFile2Base64(const std::string& filepath) {
    std::ifstream file(filepath, std::ios::binary);
    if (!file) {
        std::cerr << "无法打开文件: " << filepath << std::endl;
        return "";
    }

    std::vector<char> file_data((std::istreambuf_iterator<char>(file)),
                                std::istreambuf_iterator<char>());
    file.close();

    std::string sst(file_data.begin(), file_data.end());

    return base64_encode(sst);
}

void Kutil::Base642File(const std::string& base64_str, const std::string& output_filepath) {
    std::string decoded = base64_decode(base64_str);

    std::ofstream output_file(output_filepath, std::ios::binary);
    if (!output_file) {
        std::cerr << "无法打开文件: " << output_filepath << std::endl;
        return;
    }

    output_file.write(decoded.data(), decoded.size());
    output_file.close();

    std::cout << "文件已成功保存为: " << output_filepath << std::endl;
}

void Kutil::SetObjPath(std::string obj_path) {
    obj_path_ = obj_path;
}

std::string  Kutil::GetObjPath() {
    return obj_path_;
}

void Kutil::SetOtherPath(std::string other_model_path) {
    other_model_path_ = other_model_path;
}

std::string  Kutil::GetOtherPath() {
    return other_model_path_;
}