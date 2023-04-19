#include <thread>

#include <catch/catch.hpp>

#include <task.hpp>

static const uint32_t EXEC_SLEEP_MS = 100;
static const uint32_t MAIN_SLEEP_MS = 1000;

static const uint32_t EXEC_PERIOD_MS = EXEC_SLEEP_MS + 100;

static uint8_t n_calls_setup = 0;
static uint8_t n_calls_execute = 0;
static uint8_t n_calls_finish = 0;

class TestTask : public Task
{
public:
    using Task::Task;

protected:
    void _setup() override
    {
        n_calls_setup++;
    }
    void _execute() override
    {
        n_calls_execute++;
        std::this_thread::sleep_for(std::chrono::milliseconds(EXEC_SLEEP_MS));
    }
    void _finish() override
    {
        n_calls_finish++;
    }
};

void launch_and_teardown(uint32_t exec_period_ms)
{
    TestTask test_task(exec_period_ms);

    test_task.launch();
    std::this_thread::sleep_for(std::chrono::milliseconds(MAIN_SLEEP_MS));
    test_task.teardown();
}

TEST_CASE("launch and teardown")
{
    launch_and_teardown(EXEC_PERIOD_MS);

    REQUIRE(n_calls_setup == 1);
    REQUIRE(n_calls_execute >= (MAIN_SLEEP_MS / EXEC_PERIOD_MS));
    REQUIRE(n_calls_finish == 1);
}
