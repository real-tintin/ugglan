#include <catch.h>
#include <catch_utils.h>

#include <string>
#include <vector>
#include <nlohmann/json.hpp>

#include <data_log_types.h>
#include <data_log_signals.h>
#include <data_log_utils.h>


TEST_CASE("add_data_log_metadata_to_json")
{
    nlohmann::ordered_json data;

    data_log::utils::add_data_log_metadata_to_json(data);

    SECTION("contains roots")
    {
        std::vector<std::string> exp_roots = {"types", "groups", "signals"};
        std::vector<std::string> act_roots = {};

        for (auto& [key, value] : data.items())
        {
            act_roots.push_back(key);
        }

        REQUIRE(exp_roots == act_roots);
    }
}
