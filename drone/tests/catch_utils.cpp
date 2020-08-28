#include <catch_utils.h>

std::string read_file(std::string path)
{
    std::ifstream file(path);
    std::stringstream buf;

    if (file.fail()) { throw std::runtime_error("File does not exist"); }
    buf << file.rdbuf();

    return buf.str();
}
