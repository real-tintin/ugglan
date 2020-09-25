#include <catch.h>
#include <catch_utils.h>

#include <sstream>
#include <vector>
#include <string>
#include <logger.h>

void write_at_all_levels()
{
    logger.debug("");
    logger.info("");
    logger.warn("");
    logger.error("");
}

TEST_CASE("logger levels")
{
    catch_utils::PatchStdCout patched_cout;

    SECTION("off")
    {
        logger.set_level(LogLevel::off);
        write_at_all_levels();

        REQUIRE(patched_cout.get().size() == 0);
    }
    SECTION("debug")
    {
        std::vector<std::string> should_contain = {
            OUT_LEVEL_DEBUG,
            OUT_LEVEL_INFO,
            OUT_LEVEL_WARN,
            OUT_LEVEL_ERROR
        };

        logger.set_level(LogLevel::debug);
        write_at_all_levels();

        REQUIRE(catch_utils::str_contains_all(patched_cout.get(), should_contain));
    }
    SECTION("info")
    {
        std::vector<std::string> should_contain = {
            OUT_LEVEL_INFO,
            OUT_LEVEL_WARN,
            OUT_LEVEL_ERROR
        };
        std::vector<std::string> should_not_contain = {
            OUT_LEVEL_DEBUG
        };

        logger.set_level(LogLevel::info);
        write_at_all_levels();

        REQUIRE(catch_utils::str_contains_all(patched_cout.get(), should_contain));
        REQUIRE(catch_utils::str_contains_non(patched_cout.get(), should_not_contain));
    }
    SECTION("warn")
    {
        std::vector<std::string> should_contain = {
            OUT_LEVEL_WARN,
            OUT_LEVEL_ERROR
        };
        std::vector<std::string> should_not_contain = {
            OUT_LEVEL_DEBUG,
            OUT_LEVEL_INFO
        };

        logger.set_level(LogLevel::warn);
        write_at_all_levels();

        REQUIRE(catch_utils::str_contains_all(patched_cout.get(), should_contain));
        REQUIRE(catch_utils::str_contains_non(patched_cout.get(), should_not_contain));
    }
    SECTION("error")
    {
        std::vector<std::string> should_contain = {
            OUT_LEVEL_ERROR
        };
        std::vector<std::string> should_not_contain = {
            OUT_LEVEL_WARN,
            OUT_LEVEL_DEBUG,
            OUT_LEVEL_INFO
        };

        logger.set_level(LogLevel::error);
        write_at_all_levels();

        REQUIRE(catch_utils::str_contains_all(patched_cout.get(), should_contain));
        REQUIRE(catch_utils::str_contains_non(patched_cout.get(), should_not_contain));
    }
}
