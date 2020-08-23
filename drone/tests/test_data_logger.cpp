#include <catch.h>

#include <ctime>
#include <string>
#include <fstream>
#include <sstream>

#include <data_logger.h>

static const std::time_t HEADER_TIME = 0;
static const std::string HEADER_PATH = "./tests/resources/header.json";

std::string read_file(std::string path)
{
    std::ifstream file(path);
    std::stringstream buf;

    if (file.fail()) { throw std::runtime_error("File does not exist"); }
    buf << file.rdbuf();

    return buf.str();
}

TEST_CASE("data_logger: generate header")
{
    std::string exp_header = read_file(HEADER_PATH);
    std::string act_header = generate_header(HEADER_TIME);

    REQUIRE(act_header.size() > 0);
    REQUIRE(exp_header.compare(act_header) == 0);
}
