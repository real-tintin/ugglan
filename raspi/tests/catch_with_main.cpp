#define CATCH_CONFIG_MAIN
#include "catch.h"
#include "logger.h"

void run_before_catch()
{
    logger.set_level(LogLevel::off);
}
