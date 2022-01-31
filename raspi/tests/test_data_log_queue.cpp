#include <catch.h>

#include <thread>
#include <cstring>
#include <task.h>
#include <wall_time.h>
#include <data_log_queue.h>

#define TEST_CASE_DATA_LOG_TYPES(name) TEMPLATE_TEST_CASE_SIG(name, "", \
    ((typename TestType, DataLogSignal data_log_signal, DataLogType data_log_type), \
        TestType, data_log_signal, data_log_type), \
    (bool, DataLogSignal::Bool, DataLogType::BOOL), \
    (uint8_t, DataLogSignal::Uint8, DataLogType::UINT8), \
    (uint16_t, DataLogSignal::Uint16, DataLogType::UINT16), \
    (uint32_t, DataLogSignal::Uint32, DataLogType::UINT32), \
    (int8_t, DataLogSignal::Sint8, DataLogType::SINT8), \
    (int16_t, DataLogSignal::Sint16, DataLogType::SINT16), \
    (int32_t, DataLogSignal::Sint32, DataLogType::SINT32), \
    (float, DataLogSignal::Float, DataLogType::FLOAT), \
    (double, DataLogSignal::Double, DataLogType::DOUBLE))

static const double FLOAT_TOL = 1e-4;

DataLogQueue data_log_queue_multi;

class TestTaskOne : public Task
{
public:
    using Task::Task;
protected:
    void _execute() override { data_log_queue_multi.push(double(1.0), DataLogSignal::ImuAccelerationX); }
};

class TestTaskTwo : public Task
{
public:
    using Task::Task;
protected:
    void _execute() override { data_log_queue_multi.push(uint8_t(0x02), DataLogSignal::EscStatus0); }
};

TEST_CASE_DATA_LOG_TYPES("data_log_queue: push and pop single thread")
{
    DataLogQueue data_log_queue;

    SECTION("is empty")
    {
        REQUIRE(data_log_queue.is_empty() == true);
    }
    SECTION("push and pop")
    {
        TestType data = 1;

        data_log_queue.push(data, data_log_signal);
        REQUIRE(data_log_queue.is_empty() == false);

        DataLogSample sample = data_log_queue.pop();
        REQUIRE(data_log_queue.is_empty() == true);

        REQUIRE(std::memcmp(&data, &sample.data, sizeof(TestType)) == 0);
        REQUIRE(sample.rel_timestamp_ms == 0);
        REQUIRE(sample.type == data_log_type);
        REQUIRE(sample.signal == data_log_signal);
    }
}

TEST_CASE("data_log_queue: push and pop multi thread")
{
    TestTaskOne task_one(10); // Runs at 100 Hz
    TestTaskTwo task_two(20); // Runs at 50 Hz

    task_one.launch();
    task_two.launch();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    task_one.teardown();
    task_two.teardown();

    REQUIRE(data_log_queue_multi.is_empty() == false);

    uint8_t n_act_one_samples = 0;
    uint8_t n_act_two_samples = 0;

    double exp_one_sample_data = 1.0;
    uint8_t exp_two_sample_data = 0x02;

    while (!data_log_queue_multi.is_empty())
    {
        DataLogSample sample = data_log_queue_multi.pop();

        if ((std::memcmp(&exp_one_sample_data, &sample.data, sizeof(double)) == 0) &&
             sample.signal == DataLogSignal::ImuAccelerationX)
        {
            n_act_one_samples++;
        }
        else if ((std::memcmp(&exp_two_sample_data, &sample.data, sizeof(uint8_t)) == 0) &&
                  sample.signal == DataLogSignal::EscStatus0)
        {
            n_act_two_samples++;
        }
        else
        {
            FAIL("Unexpected sample");
        }
    }

    REQUIRE(n_act_one_samples >= 10);
    REQUIRE(n_act_two_samples >= 5);
}

TEST_CASE_DATA_LOG_TYPES("data_log_queue: last_signal_data")
{
    DataLogQueue data_log_queue;

    SECTION("empty queue - implicit (zero) default data")
    {
        TestType act_data;
        TestType exp_data = 0;
        data_log_queue.last_signal_data(&act_data, data_log_signal);

        REQUIRE(std::memcmp(&act_data, &exp_data, sizeof(TestType)) == 0);
    }
    SECTION("empty queue - explicit default data")
    {
        TestType act_data;
        TestType exp_data = 1;
        data_log_queue.last_signal_data(&act_data, data_log_signal, TestType(1));

        REQUIRE(std::memcmp(&act_data, &exp_data, sizeof(TestType)) == 0);
    }
    SECTION("non empty queue")
    {
        /* Push a first sample */
        TestType act_data;
        TestType exp_data = 0;

        data_log_queue.push(TestType(0), data_log_signal);
        data_log_queue.last_signal_data(&act_data, data_log_signal);

        REQUIRE(std::memcmp(&act_data, &exp_data, sizeof(TestType)) == 0);

        /* Push a second sample */
        exp_data = 1;
        data_log_queue.push(TestType(1), data_log_signal);
        data_log_queue.last_signal_data(&act_data, data_log_signal);

        REQUIRE(std::memcmp(&act_data, &exp_data, sizeof(TestType)) == 0);

        /* Empty queue */
        while (!data_log_queue.is_empty()) { data_log_queue.pop(); };
        data_log_queue.last_signal_data(&act_data, data_log_signal);

        REQUIRE(std::memcmp(&act_data, &exp_data, sizeof(TestType)) == 0);
    }
}

TEST_CASE("data_log_queue: error handling")
{
    struct NewType { bool data; };
    DataLogQueue data_log_queue;

    SECTION("unsupported type")
    {
        NewType data;
        REQUIRE_THROWS_WITH(data_log_queue.push(data, DataLogSignal::Float),
                            "Unsupported data type");
    }
    SECTION("signal type missmatch")
    {
        uint8_t data = 0;
        REQUIRE_THROWS_WITH(data_log_queue.push(data, DataLogSignal::Double),
                            "Data log signal type missmatch");
    }
}
