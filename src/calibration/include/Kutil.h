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
    static void Base643File(const std::string& base64_str, const std::string& output_filepath);
};

#endif //KUTIL_H
