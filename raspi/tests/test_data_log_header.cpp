
#include <ctime>
#include <filesystem>
#include <string>

#include <catch/catch.hpp>

#include <catch_utils.hpp>
#include <data_log_header.hpp>

static const std::time_t HEADER_TIME = 0;
static const std::filesystem::path HEADER_PATH = catch_utils::RESOURCE_ROOT / "header.json";

TEST_CASE("data_log_header")
{
    std::string exp_header = catch_utils::read_file(HEADER_PATH);
    std::string act_header = generate_header(HEADER_TIME);

    REQUIRE_FALSE(act_header.empty());
    REQUIRE(exp_header == act_header);
}
