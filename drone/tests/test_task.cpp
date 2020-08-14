#include <catch.h>

#include <thread>
#include <task.h>

static const uint32_t EXEC_SLEEP_MS = 100;
static const uint32_t MAIN_SLEEP_MS = 1000;

static const uint32_t NOT_ENOUGH_EXEC_PERIOD_MS = EXEC_SLEEP_MS - 50;
static const uint32_t ENOUGH_EXEC_PERIOD_MS = EXEC_SLEEP_MS + 100;

static uint8_t n_calls_setup;
static uint8_t n_calls_execute;
static uint8_t n_calls_finish;
static uint8_t n_calls_exec_period_exceeded;

class TestTask : public Task
{
public:
    TestTask(uint32_t exec_period_ms, void (*exec_period_exceeded_cb)()) :
        Task(exec_period_ms, exec_period_exceeded_cb)
    {
    }
protected:
    void _setup()
    {
        n_calls_setup++;
    }
    void _execute()
    {
        n_calls_execute++;
        std::this_thread::sleep_for(std::chrono::milliseconds(EXEC_SLEEP_MS));
    }
    void _finish()
    {
        n_calls_finish++;
    }
};

void exec_period_exceeded_cb()
{
    n_calls_exec_period_exceeded++;
}

void reset_test_vars()
{
    n_calls_setup = 0;
    n_calls_execute = 0;
    n_calls_finish = 0;
    n_calls_exec_period_exceeded = 0;
}

void launch_and_teardown(uint32_t exec_period_ms)
{
    TestTask test_task(exec_period_ms, &exec_period_exceeded_cb);

    test_task.launch();
    std::this_thread::sleep_for(std::chrono::milliseconds(MAIN_SLEEP_MS));
    test_task.teardown();
}

TEST_CASE("launch and teardown")
{
    reset_test_vars();

    SECTION("enough exec period")
    {
        launch_and_teardown(ENOUGH_EXEC_PERIOD_MS);

        REQUIRE(n_calls_setup == 1);
        REQUIRE(n_calls_execute >= (MAIN_SLEEP_MS / ENOUGH_EXEC_PERIOD_MS));
        REQUIRE(n_calls_finish == 1);
        REQUIRE(n_calls_exec_period_exceeded == 0);
    }
    SECTION("not enough exec period")
    {
        launch_and_teardown(NOT_ENOUGH_EXEC_PERIOD_MS);

        REQUIRE(n_calls_setup == 1);
        REQUIRE(n_calls_execute <= (MAIN_SLEEP_MS / NOT_ENOUGH_EXEC_PERIOD_MS));
        REQUIRE(n_calls_finish == 1);
        REQUIRE(n_calls_exec_period_exceeded >= (MAIN_SLEEP_MS / EXEC_SLEEP_MS));
    }
}
