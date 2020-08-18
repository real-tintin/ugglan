#include <catch.h>

#include <thread>
#include <cstring>
#include <wall_time.h>
#include <data_log_queue.h>

TEST_CASE("data_log_queue: single thread")
{
    DataLogQueue data_log_queue;

    SECTION("is empty")
    {
        REQUIRE(data_log_queue.is_empty() == true);
    }
    SECTION("push and pop")
    {
        double data = -3.14;
        const uint8_t sleep_ms = 10;

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));

        data_log_queue.push(data, DataLogSignal::ImuAccelerationX);
        REQUIRE(data_log_queue.is_empty() == false);

        DataLogSample sample = data_log_queue.pop();
        REQUIRE(data_log_queue.is_empty() == true);

        REQUIRE(std::memcmp(&data, &sample.data, sizeof(double)) == 0);
        REQUIRE(sample.rel_timestamp_ms >= sleep_ms);
        REQUIRE(sample.type == DataLogType::DOUBLE);
        REQUIRE(sample.signal == DataLogSignal::ImuAccelerationX);
    }
}

TEST_CASE("data_log_queue: multi thread")
{
    // TODO: Launch two tasks and add some data.
    // TODO: Test what happens if a other type is pushed compared to DataLogSignalInfo.
}

TEST_CASE("data_log_queue: error handling")
{
    DataLogQueue data_log_queue;

    SECTION("unsupported type")
    {
        bool data;
        REQUIRE_THROWS_WITH(data_log_queue.push(data, DataLogSignal::ImuAccelerationX),
                            "Unsupported data type");
    }
    SECTION("signal type missmatch")
    {
        uint8_t data;
        REQUIRE_THROWS_WITH(data_log_queue.push(data, DataLogSignal::ImuAccelerationX),
                            "Data log signal type missmatch");
    }
    SECTION("timestamp overflow")
    {
        double data;
        std::this_thread::sleep_for(std::chrono::milliseconds(UINT8_MAX + 1));
        REQUIRE_THROWS_WITH(data_log_queue.push(data, DataLogSignal::ImuAccelerationX),
                            "Data log queue timstamp overflow. Called too seldom");
    }
}
