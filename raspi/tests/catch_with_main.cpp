#define CATCH_CONFIG_MAIN

#include <catch.hpp>

#include <logger.hpp>

void run_before_catch()
{
    logger.set_level(LogLevel::off);
}
