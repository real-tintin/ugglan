#ifndef CATCH_UTILS_HPP
#define CATCH_UTILS_HPP

#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <vector>
#include <algorithm>

#include <common_utils.hpp>

namespace catch_utils
{
inline const std::filesystem::path TEST_ROOT = common_utils::get_env("TEST_ROOT");
inline const std::filesystem::path RESOURCE_ROOT = common_utils::get_env("RESOURCE_ROOT");

std::string read_file(std::filesystem::path path);

bool str_contains_all(std::string str, std::vector<std::string> contains);
bool str_contains_non(std::string str, std::vector<std::string> contains);

void set_env(std::string env, std::string val);

class TmpDir
{
public:
    TmpDir(bool remove_when_done = true);
    ~TmpDir();

    std::filesystem::path get_path();

private:
    std::filesystem::path _path;
    bool _remove_when_done;

    static std::string _folder_name();
};

class PatchStdCout
{
public:
    PatchStdCout();
    ~PatchStdCout();

    std::string get();

private:
    std::ostringstream _patch_buf;
    std::streambuf *_org_buf;
};
} // namespace catch_utils

#endif /* CATCH_UTILS_HPP */
