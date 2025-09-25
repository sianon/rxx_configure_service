//
// Created by cao on 9/9/25.
//

#ifndef KUTIL_H
#define KUTIL_H

class Kutil {
public:
    Kutil(){};
    ~Kutil(){};

    static std::string EncodeFile2Base64(const std::string& filepath);
    static void Base642File(const std::string& base64_str, const std::string& output_filepath);
    static std::string GetObjPath();
    static void SetObjPath(std::string obj_path);

    static std::string GetOtherPath();
    static void SetOtherPath(std::string obj_path);

    static std::string obj_path_;
    static std::string other_model_path_;
};

#endif //KUTIL_H
