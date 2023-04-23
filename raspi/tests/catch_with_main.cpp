#define CATCH_CONFIG_MAIN

#include <catch/catch.hpp>

#include <logger.hpp>

void run_before_catch()
{
    logger.set_level(LogLevel::off);
}
